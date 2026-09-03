"""
PncMap: routing passage → multiple RouteSegments for reference line generation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

from common.hd_map import HDMap, MakeMapId
from common.lane_info import LaneInfo
from common.map_path_point import LaneWaypoint
from common.route_segments import RouteSegments, kSegmentationEpsilon
from common.lane_types import LaneSegment
from protoclass.decision_result import ChangeLaneType
from protoclass.lane_waypoint import LaneWaypoint as RoutingLaneWaypoint
from protoclass.planning_status import LaneSegment as RoutingLaneSegment
from protoclass.vehicle_state import VehicleState
from common.vec2d import Vec2d
from protoclass.point_enu import PointENU
from protoclass.routing import Passage, RoadSegment, RoutingResponse
import config as config_module


def _lane_id_str(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if hasattr(value, "id"):
        inner = value.id
        if hasattr(inner, "id"):
            return str(inner.id)
        return str(inner)
    return str(value)


def _distance_xy(a: VehicleState, b: VehicleState) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


@dataclass
class RouteIndex:
    segment: LaneSegment
    index: Tuple[int, int, int] = (0, 0, 0)


@dataclass
class WaypointIndex:
    waypoint: LaneWaypoint
    index: int


class PncMap:
    """Apollo PncMap Python port: routing → multi-lane RouteSegments."""

    def __init__(self, hdmap: HDMap):
        self._hdmap = hdmap
        self._routing: RoutingResponse = RoutingResponse()
        self._route_indices: List[RouteIndex] = []
        self._range_start: int = 0
        self._range_end: int = 0
        self._range_lane_ids: Set[str] = set()
        self._all_lane_ids: Set[str] = set()
        self._routing_waypoint_index: List[WaypointIndex] = []
        self._next_routing_waypoint_index: int = 0
        self._adc_state: Optional[VehicleState] = None
        self._adc_route_index: int = -1
        self._adc_waypoint: LaneWaypoint = LaneWaypoint()
        self._stop_for_destination: bool = False

    def hdmap(self) -> HDMap:
        return self._hdmap

    def routing_response(self) -> RoutingResponse:
        return self._routing

    @staticmethod
    def LookForwardDistance(velocity: float) -> float:
        forward_distance = velocity * config_module.FLAGS_look_forward_time_sec
        if forward_distance > config_module.FLAGS_look_forward_short_distance:
            return config_module.FLAGS_look_forward_long_distance
        return config_module.FLAGS_look_forward_short_distance

    @staticmethod
    def ValidateRouting(routing: RoutingResponse) -> bool:
        if not routing or not routing.road:
            return False
        if routing.routing_request is None or len(routing.routing_request.waypoint) < 2:
            return False
        for waypoint in routing.routing_request.waypoint:
            if waypoint.id is None or waypoint.s is None:
                return False
        return True

    @staticmethod
    def CompareRouting(prev: RoutingResponse, routing: RoutingResponse) -> bool:
        if not PncMap.ValidateRouting(routing):
            return False
        return PncMap._routing_fingerprint(prev) != PncMap._routing_fingerprint(routing)

    @staticmethod
    def _routing_fingerprint(routing: RoutingResponse) -> Tuple:
        parts = []
        if routing.routing_request:
            for wp in routing.routing_request.waypoint:
                parts.append(("wp", wp.id, wp.s))
        for road in routing.road or []:
            for passage in road.passage or []:
                for seg in passage.segment or []:
                    parts.append(("seg", seg.id, seg.start_s, seg.end_s, passage.change_lane_type, passage.can_exit))
        return tuple(parts)

    def IsNewRouting(self, routing: RoutingResponse) -> bool:
        return PncMap.CompareRouting(self._routing, routing)

    def ToLaneWaypoint(self, waypoint: RoutingLaneWaypoint) -> LaneWaypoint:
        lane = self._hdmap.GetLaneById(MakeMapId(waypoint.id))
        if lane is None:
            return LaneWaypoint()
        return LaneWaypoint(lane=lane, s=waypoint.s or 0.0)

    def ToLaneSegment(self, segment: RoutingLaneSegment) -> LaneSegment:
        lane = self._hdmap.GetLaneById(MakeMapId(segment.id))
        if lane is None:
            return LaneSegment(None, 0.0, 0.0)
        start_s = max(0.0, segment.start_s or 0.0)
        end_s = min(lane.total_length, segment.end_s or lane.total_length)
        return LaneSegment(lane, start_s, end_s)

    def UpdateRoutingResponse(self, routing: RoutingResponse) -> bool:
        self._range_lane_ids.clear()
        self._route_indices.clear()
        self._all_lane_ids.clear()

        for road_index, road_segment in enumerate(routing.road or []):
            for passage_index, passage in enumerate(road_segment.passage or []):
                for lane_index, lane_seg in enumerate(passage.segment or []):
                    self._all_lane_ids.add(_lane_id_str(lane_seg.id))
                    lane_segment = self.ToLaneSegment(lane_seg)
                    if lane_segment.lane is None:
                        return False
                    self._route_indices.append(
                        RouteIndex(segment=lane_segment, index=(road_index, passage_index, lane_index))
                    )

        self._range_start = 0
        self._range_end = 0
        self._adc_route_index = -1
        self._next_routing_waypoint_index = 0
        self.UpdateRoutingRange(self._adc_route_index)

        self._routing_waypoint_index.clear()
        request_waypoints = routing.routing_request.waypoint if routing.routing_request else []
        if not request_waypoints:
            return False

        i = 0
        for j, route_index in enumerate(self._route_indices):
            while i < len(request_waypoints) and RouteSegments.WithinRoutingLaneSegment(
                route_index.segment, request_waypoints[i]
            ):
                self._routing_waypoint_index.append(
                    WaypointIndex(
                        waypoint=LaneWaypoint(route_index.segment.lane, request_waypoints[i].s or 0.0),
                        index=j,
                    )
                )
                i += 1

        self._routing = routing
        self._adc_waypoint = LaneWaypoint()
        self._stop_for_destination = False
        return True

    def UpdateRoutingRange(self, adc_index: int) -> None:
        self._range_lane_ids.clear()
        self._range_start = max(0, adc_index - 1)
        self._range_end = self._range_start
        while self._range_end < len(self._route_indices):
            lane_id = _lane_id_str(self._route_indices[self._range_end].segment.lane.id)
            if lane_id in self._range_lane_ids:
                break
            self._range_lane_ids.add(lane_id)
            self._range_end += 1

    def UpdateNextRoutingWaypointIndex(self, cur_index: int) -> None:
        if cur_index < 0:
            self._next_routing_waypoint_index = 0
            return
        if cur_index >= len(self._route_indices):
            self._next_routing_waypoint_index = max(0, len(self._routing_waypoint_index) - 1)
            return

        while (
            self._next_routing_waypoint_index != 0
            and self._next_routing_waypoint_index < len(self._routing_waypoint_index)
            and self._routing_waypoint_index[self._next_routing_waypoint_index].index > cur_index
        ):
            self._next_routing_waypoint_index -= 1

        while (
            self._next_routing_waypoint_index != 0
            and self._next_routing_waypoint_index < len(self._routing_waypoint_index)
            and self._routing_waypoint_index[self._next_routing_waypoint_index].index == cur_index
            and self._adc_waypoint.s
            < self._routing_waypoint_index[self._next_routing_waypoint_index].waypoint.s
        ):
            self._next_routing_waypoint_index -= 1

        while (
            self._next_routing_waypoint_index < len(self._routing_waypoint_index)
            and self._routing_waypoint_index[self._next_routing_waypoint_index].index < cur_index
        ):
            self._next_routing_waypoint_index += 1

        while (
            self._next_routing_waypoint_index < len(self._routing_waypoint_index)
            and cur_index == self._routing_waypoint_index[self._next_routing_waypoint_index].index
            and self._adc_waypoint.s
            >= self._routing_waypoint_index[self._next_routing_waypoint_index].waypoint.s
        ):
            self._next_routing_waypoint_index += 1

        if self._next_routing_waypoint_index >= len(self._routing_waypoint_index):
            self._next_routing_waypoint_index = max(0, len(self._routing_waypoint_index) - 1)

    def FutureRouteWaypoints(self) -> List[RoutingLaneWaypoint]:
        if self._routing.routing_request is None:
            return []
        waypoints = self._routing.routing_request.waypoint
        return list(waypoints[self._next_routing_waypoint_index :])

    def SearchForwardWaypointIndex(self, start: int, waypoint: LaneWaypoint) -> int:
        i = max(start, 0)
        while i < len(self._route_indices) and not RouteSegments.WithinLaneSegment(
            self._route_indices[i].segment, waypoint
        ):
            i += 1
        return i

    def SearchBackwardWaypointIndex(self, start: int, waypoint: LaneWaypoint) -> int:
        i = min(len(self._route_indices) - 1, start)
        while i >= 0 and not RouteSegments.WithinLaneSegment(self._route_indices[i].segment, waypoint):
            i -= 1
        return i

    def NextWaypointIndex(self, index: int) -> int:
        if index >= len(self._route_indices) - 1:
            return len(self._route_indices) - 1
        if index < 0:
            return 0
        return index + 1

    def GetWaypointIndex(self, waypoint: LaneWaypoint) -> int:
        forward_index = self.SearchForwardWaypointIndex(self._adc_route_index, waypoint)
        if forward_index >= len(self._route_indices):
            return self.SearchBackwardWaypointIndex(self._adc_route_index, waypoint)
        if forward_index == self._adc_route_index or forward_index == self._adc_route_index + 1:
            return forward_index
        backward_index = self.SearchBackwardWaypointIndex(self._adc_route_index, waypoint)
        if backward_index < 0:
            return forward_index
        if backward_index + 1 == self._adc_route_index:
            return backward_index
        return forward_index

    def UpdateVehicleState(self, vehicle_state: VehicleState) -> bool:
        if not PncMap.ValidateRouting(self._routing):
            return False

        if self._adc_state is None or (
            _distance_xy(self._adc_state, vehicle_state)
            > config_module.FLAGS_replan_lateral_distance_threshold + config_module.FLAGS_replan_longitudinal_distance_threshold
        ):
            self._next_routing_waypoint_index = 0
            self._adc_route_index = -1
            self._stop_for_destination = False

        self._adc_state = vehicle_state
        adc_waypoint = LaneWaypoint()
        if not self.GetNearestPointFromRouting(vehicle_state, adc_waypoint):
            return False
        self._adc_waypoint = adc_waypoint

        route_index = self.GetWaypointIndex(self._adc_waypoint)
        if route_index < 0 or route_index >= len(self._route_indices):
            return False

        self.UpdateNextRoutingWaypointIndex(route_index)
        self._adc_route_index = route_index
        self.UpdateRoutingRange(self._adc_route_index)

        if not self._routing_waypoint_index:
            return False

        if self._next_routing_waypoint_index == len(self._routing_waypoint_index) - 1:
            self._stop_for_destination = True
        return True

    def PassageToSegments(self, passage: Passage) -> RouteSegments:
        segments = RouteSegments()
        for lane_seg in passage.segment or []:
            lane = self._hdmap.GetLaneById(MakeMapId(lane_seg.id))
            if lane is None:
                return RouteSegments()
            start_s = max(0.0, lane_seg.start_s or 0.0)
            end_s = min(lane.total_length, lane_seg.end_s or lane.total_length)
            segments.append(LaneSegment(lane, start_s, end_s))
        return segments

    def GetNeighborPassages(self, road: RoadSegment, start_passage: int) -> List[int]:
        result = [start_passage]
        if start_passage < 0 or start_passage >= len(road.passage or []):
            return result

        source_passage = road.passage[start_passage]
        if source_passage.change_lane_type == ChangeLaneType.FORWARD:
            return result
        if source_passage.can_exit:
            return result

        source_segments = self.PassageToSegments(source_passage)
        if not source_segments:
            return result

        if (
            self._next_routing_waypoint_index < len(self._routing_waypoint_index)
            and source_segments.IsWaypointOnSegment(
                self._routing_waypoint_index[self._next_routing_waypoint_index].waypoint
            )
        ):
            return result

        neighbor_lanes: Set[str] = set()
        if source_passage.change_lane_type == ChangeLaneType.LEFT:
            for segment in source_segments:
                for left_id in segment.lane.lane.left_neighbor_forward_lane_id or []:
                    neighbor_lanes.add(_lane_id_str(left_id))
        elif source_passage.change_lane_type == ChangeLaneType.RIGHT:
            for segment in source_segments:
                for right_id in segment.lane.lane.right_neighbor_forward_lane_id or []:
                    neighbor_lanes.add(_lane_id_str(right_id))

        for i, target_passage in enumerate(road.passage or []):
            if i == start_passage:
                continue
            for segment in target_passage.segment or []:
                if _lane_id_str(segment.id) in neighbor_lanes:
                    result.append(i)
                    break
        return result

    def GetRouteSegments(
        self,
        vehicle_state: VehicleState,
        backward_length: Optional[float] = None,
        forward_length: Optional[float] = None,
    ) -> List[RouteSegments]:
        if backward_length is None or forward_length is None:
            forward_length = self.LookForwardDistance(vehicle_state.linear_velocity or 0.0)
            backward_length = config_module.FLAGS_look_backward_distance

        route_segments: List[RouteSegments] = []
        if not self.UpdateVehicleState(vehicle_state):
            return route_segments

        if (
            self._adc_waypoint.lane is None
            or self._adc_route_index < 0
            or self._adc_route_index >= len(self._route_indices)
        ):
            return route_segments

        route_index = self._route_indices[self._adc_route_index].index
        road_index, passage_index, _ = route_index
        road = self._routing.road[road_index]
        drive_passages = self.GetNeighborPassages(road, passage_index)

        for index in drive_passages:
            passage = road.passage[index]
            segments = self.PassageToSegments(passage)
            if not segments:
                continue

            if index == passage_index:
                nearest_point = self._adc_waypoint.lane.GetSmoothPoint(self._adc_waypoint.s)
            else:
                nearest_point = PointENU(x=vehicle_state.x, y=vehicle_state.y)

            ok, sl, segment_waypoint = segments.GetProjection(nearest_point)
            if not ok or sl is None:
                continue

            if index != passage_index and not segments.CanDriveFrom(self._adc_waypoint):
                continue

            extended = RouteSegments()
            if not self.ExtendSegments(segments, sl.s - backward_length, sl.s + forward_length, extended):
                return route_segments

            last_waypoint = segments.LastWayPoint()
            if extended.IsWaypointOnSegment(last_waypoint):
                extended.SetRouteEndWaypoint(last_waypoint)

            extended.SetCanExit(bool(passage.can_exit))
            extended.SetNextAction(passage.change_lane_type)
            extended.SetId(f"{road_index}_{index}")
            extended.SetStopForDestination(self._stop_for_destination)

            if index == passage_index:
                extended.SetIsOnSegment(True)
                extended.SetPreviousAction(ChangeLaneType.FORWARD)
            elif sl.l > 0:
                extended.SetPreviousAction(ChangeLaneType.RIGHT)
            else:
                extended.SetPreviousAction(ChangeLaneType.LEFT)

            if index != passage_index:
                extended.SetIsNeighborSegment(True)

            route_segments.append(extended)
        return route_segments

    def GetNearestPointFromRouting(self, state: VehicleState, waypoint: LaneWaypoint) -> bool:
        k_max_distance = 10.0
        k_heading_buffer = math.pi / 10.0
        waypoint.lane = None

        point = PointENU(x=state.x, y=state.y)
        heading = state.heading if state.heading is not None else 0.0
        lanes = self._hdmap.GetLanesWithHeading(point, k_max_distance, heading, math.pi / 2.0 + k_heading_buffer)
        if not lanes:
            ok, lane, lane_s, _ = self._hdmap.GetNearestLane(point)
            if ok and lane is not None:
                lanes = [lane]
            else:
                return False

        valid_lanes = [lane for lane in lanes if _lane_id_str(lane.id) in self._range_lane_ids]
        if not valid_lanes:
            valid_lanes = [lane for lane in lanes if _lane_id_str(lane.id) in self._all_lane_ids]
        if not valid_lanes:
            return False

        min_distance = float("inf")
        best_lane = None
        best_s = 0.0

        for lane in valid_lanes:
            if _lane_id_str(lane.id) not in self._range_lane_ids and self._range_lane_ids:
                continue
            ok, lane_s, lane_l = lane.GetProjection(Vec2d(point.x, point.y))
            if not ok:
                continue
            k_epsilon = 0.5
            if lane_s > lane.total_length + k_epsilon or lane_s + k_epsilon < 0.0:
                continue
            distance = abs(lane_l)
            if distance < min_distance:
                min_distance = distance
                best_lane = lane
                best_s = lane_s

        if best_lane is None:
            return False
        waypoint.lane = best_lane
        waypoint.s = best_s
        return True

    def GetRoutePredecessor(self, lane: LaneInfo) -> Optional[LaneInfo]:
        predecessor_ids = lane.lane.predecessor_id or []
        if not predecessor_ids:
            return None

        predecessor_set = {_lane_id_str(pid) for pid in predecessor_ids}
        preferred_id = _lane_id_str(predecessor_ids[0])
        for route_index in self._route_indices[1:]:
            lane_id = _lane_id_str(route_index.segment.lane.id)
            if lane_id in predecessor_set:
                preferred_id = lane_id
                break
        return self._hdmap.GetLaneById(preferred_id)

    def GetRouteSuccessor(self, lane: LaneInfo) -> Optional[LaneInfo]:
        successor_ids = lane.lane.successor_id or []
        if not successor_ids:
            return None

        preferred_id = _lane_id_str(successor_ids[0])
        for lane_id_obj in successor_ids:
            lane_id = _lane_id_str(lane_id_obj)
            if lane_id in self._range_lane_ids:
                preferred_id = lane_id
                break
        return self._hdmap.GetLaneById(preferred_id)

    def ExtendSegments(
        self,
        segments: RouteSegments,
        start_s: float,
        end_s: float,
        truncated_segments: Optional[RouteSegments] = None,
    ) -> bool:
        if not segments:
            return False
        if truncated_segments is None:
            truncated_segments = RouteSegments()
        else:
            while truncated_segments:
                truncated_segments.pop()
        truncated_segments.SetProperties(segments)

        if start_s >= end_s:
            return False

        unique_lanes: Set[str] = set()
        k_route_epsilon = 1e-3

        if start_s < 0:
            first_segment = segments[0]
            lane = first_segment.lane
            s = first_segment.start_s
            extend_s = -start_s
            extended_lane_segments: List[LaneSegment] = []
            while extend_s > k_route_epsilon:
                if s <= k_route_epsilon:
                    lane = self.GetRoutePredecessor(lane)
                    if lane is None or _lane_id_str(lane.id) in unique_lanes:
                        break
                    s = lane.total_length
                else:
                    length = min(s, extend_s)
                    extended_lane_segments.append(LaneSegment(lane, s - length, s))
                    extend_s -= length
                    s -= length
                    unique_lanes.add(_lane_id_str(lane.id))
            for seg in reversed(extended_lane_segments):
                truncated_segments.insert(0, seg)

        found_loop = False
        router_s = 0.0
        for lane_segment in segments:
            adjusted_start_s = max(start_s - router_s + lane_segment.start_s, lane_segment.start_s)
            adjusted_end_s = min(end_s - router_s + lane_segment.start_s, lane_segment.end_s)
            if adjusted_start_s < adjusted_end_s:
                lane_id = _lane_id_str(lane_segment.lane.id)
                if truncated_segments and _lane_id_str(truncated_segments[-1].lane.id) == lane_id:
                    truncated_segments[-1].end_s = adjusted_end_s
                elif lane_id not in unique_lanes:
                    truncated_segments.append(LaneSegment(lane_segment.lane, adjusted_start_s, adjusted_end_s))
                    unique_lanes.add(lane_id)
                else:
                    found_loop = True
                    break
            router_s += lane_segment.end_s - lane_segment.start_s
            if router_s > end_s:
                break

        if found_loop:
            return True

        if router_s < end_s and truncated_segments:
            back = truncated_segments[-1]
            if back.lane.total_length > back.end_s:
                origin_end_s = back.end_s
                back.end_s = min(back.end_s + end_s - router_s, back.lane.total_length)
                router_s += back.end_s - origin_end_s

        last_lane = segments[-1].lane
        while router_s < end_s - k_route_epsilon:
            last_lane = self.GetRouteSuccessor(last_lane)
            if last_lane is None or _lane_id_str(last_lane.id) in unique_lanes:
                break
            length = min(end_s - router_s, last_lane.total_length)
            truncated_segments.append(LaneSegment(last_lane, 0.0, length))
            unique_lanes.add(_lane_id_str(last_lane.id))
            router_s += length
        return True
