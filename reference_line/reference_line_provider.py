import concurrent.futures
import math
import threading
import time
from collections import deque
from copy import deepcopy
from typing import List, Optional, Set, Tuple
import config as config_module
from common.geometry_utils import AngleDiff
from common.hd_map import HDMapUtil
from common.lane_info import Id, LaneInfo, MakeMapId
from common.lane_types import LaneSegment
from common.map_path_point import LaneWaypoint, MapPathPoint
from common.path import Path as MapPath
from common.pnc_map import PncMap
from common.route_segments import RouteSegments
from common.vec2d import Vec2d
from protoclass.decision_result import ChangeLaneType
from protoclass.lane import LaneBoundary, LaneBoundaryType
from protoclass.point_enu import PointENU
from protoclass.routing import RoutingResponse
from protoclass.sl_boundary import SLPoint
from protoclass.vehicle_state import VehicleState
from reference_line import ReferenceLine
from reference_line.discrete_points_reference_line_smoother import AnchorPoint, DiscretePointsReferenceLineSmoother
from reference_line.qp_spline_reference_line_smoother import QpSplineReferenceLineSmoother
from reference_line.reference_point import ReferencePoint

def uniform_slice(start: float, end: float, num: int) -> List[float]:
    if num <= 0:
        return [start]
    if num == 1:
        return [start, end]
    step = (end - start) / num
    return [start + i * step for i in range(num + 1)]

class ReferenceLineProvider:
    """
    The class of ReferenceLineProvider.
    It provides smoothed reference line to planning.
    """

    def __init__(self, vehicle_state_provider=None, reference_line_config=None, relative_map=None, hdmap=None):
        self._is_initialized = False
        self._is_stop = False
        self._discrete_smoother = DiscretePointsReferenceLineSmoother()
        self._qp_spline_smoother = QpSplineReferenceLineSmoother()
        self._pnc_map_mutex = threading.Lock()
        self._pnc_map: Optional[PncMap] = None
        if not config_module.FLAGS_use_navigation_mode:
            self._pnc_map = PncMap(hdmap or HDMapUtil.BaseMap())
        self._relative_map = relative_map
        self._vehicle_state_mutex = threading.Lock()
        self._vehicle_state: Optional[VehicleState] = None
        self._routing_mutex = threading.Lock()
        self._routing: Optional[RoutingResponse] = None
        self._has_routing = False
        self._reference_lines_mutex = threading.Lock()
        self._reference_lines: List[ReferenceLine] = []
        self._route_segments: List[RouteSegments] = []
        self._last_calculation_time = 0.0
        self._reference_line_history: deque = deque(maxlen=3)
        self._route_segments_history: deque = deque(maxlen=3)
        self._task_future = None
        self._is_reference_line_updated = True
        self._vehicle_state_provider = vehicle_state_provider
        self._is_initialized = True

    def _active_smoother(self):
        if config_module.FLAGS_enable_qp_spline_reference_line:
            return self._qp_spline_smoother
        return self._discrete_smoother

    def UpdatePlanningCommand(self, command) -> bool:
        with self._routing_mutex:
            if command is not None and command.lane_follow_command is not None:
                self._routing = command.lane_follow_command
                self._has_routing = True
            return True

    def UpdateRoutingResponse(self, routing: RoutingResponse) -> bool:
        with self._routing_mutex:
            self._routing = routing
            self._has_routing = True
        return True

    def UpdateVehicleState(self, vehicle_state: VehicleState) -> None:
        with self._vehicle_state_mutex:
            self._vehicle_state = vehicle_state

    def Start(self) -> bool:
        self._is_stop = False
        self._task_future = concurrent.futures.ThreadPoolExecutor().submit(self.GenerateThread)
        return True

    def Stop(self) -> None:
        self._is_stop = True
        if self._task_future:
            self._task_future.result()

    def Reset(self) -> None:
        self._is_initialized = False

    def GetReferenceLines(self) -> Tuple[List[ReferenceLine], List[RouteSegments]]:
        if config_module.FLAGS_use_navigation_mode:
            return self.GetReferenceLinesFromRelativeMap()
        if config_module.FLAGS_enable_reference_line_provider_thread:
            with self._reference_lines_mutex:
                if self._reference_lines:
                    return list(self._reference_lines), list(self._route_segments)
                if self._reference_line_history:
                    return list(self._reference_line_history[-1]), list(self._route_segments_history[-1])
            return [], []
        tag, reference_lines, route_segments = self.CreateReferenceLine()
        if tag:
            self.UpdateReferenceLine(reference_lines, route_segments)
        with self._reference_lines_mutex:
            if self._reference_lines:
                return list(self._reference_lines), list(self._route_segments)
            if self._reference_line_history:
                return list(self._reference_line_history[-1]), list(self._route_segments_history[-1])
        return [], []

    def LastTimeDelay(self) -> float:
        return self._last_calculation_time

    def FutureRouteWaypoints(self) -> List[LaneWaypoint]:
        if self._pnc_map is None:
            return []
        with self._pnc_map_mutex:
            routing_waypoints = self._pnc_map.FutureRouteWaypoints()
        return [self._pnc_map.ToLaneWaypoint(wp) for wp in routing_waypoints]

    def UpdatedReferenceLine(self) -> bool:
        return self._is_reference_line_updated

    def GetEndLaneWayPoint(self, end_point: LaneWaypoint = None) -> Optional[LaneWaypoint]:
        with self._reference_lines_mutex:
            if self._route_segments:
                return self._route_segments[-1].LastWayPoint()
        return None

    def GetLaneById(self, id: Id) -> Optional[LaneInfo]:
        lane = HDMapUtil.BaseMap().GetLaneById(id)
        if lane is not None:
            return lane
        for route_segment in self._route_segments:
            for segment in route_segment:
                lane = segment.lane
                lane_id = getattr(getattr(lane, "id", None), "id", None) or getattr(lane, "_id", None)
                target_id = getattr(id, "id", id)
                if lane_id == target_id or str(lane_id) == str(target_id):
                    return lane
        return None

    def CreateReferenceLine(self) -> Tuple[bool, List[ReferenceLine], List[RouteSegments]]:
        vehicle_state = self._vehicle_state
        if vehicle_state is None and self._vehicle_state_provider is not None:
            vehicle_state = self._vehicle_state_provider.vehicle_state

        routing = self._routing
        is_new_routing = False
        if routing is not None and self._pnc_map is not None:
            with self._pnc_map_mutex:
                is_new_routing = self._pnc_map.IsNewRouting(routing)
                if is_new_routing:
                    if not self._pnc_map.UpdateRoutingResponse(routing):
                        return False, [], []

        tag, segments = self.CreateRouteSegments(vehicle_state)
        if not tag or not segments:
            return False, [], []

        reference_lines: List[ReferenceLine] = []
        route_segments: List[RouteSegments] = []

        if is_new_routing or not config_module.FLAGS_enable_reference_line_stitching:
            for segment in segments:
                try:
                    smooth_ok, reference_line = self.SmoothRouteSegment(segment)
                except Exception:
                    continue
                if not smooth_ok or reference_line is None:
                    continue
                if vehicle_state is not None:
                    ok_sl, sl = reference_line.XYToSL(Vec2d(vehicle_state.x, vehicle_state.y))
                    if ok_sl and sl is not None:
                        self.Shrink(sl, reference_line, segment)
                reference_lines.append(reference_line)
                route_segments.append(segment)
        else:
            for segment in segments:
                extend_ok, reference_line = self.ExtendReferenceLine(vehicle_state, segment)
                if not extend_ok or reference_line is None:
                    continue
                reference_lines.append(reference_line)
                route_segments.append(segment)

        return bool(reference_lines), reference_lines, route_segments

    def UpdateReferenceLine(self, reference_lines: List[ReferenceLine], route_segments: List[RouteSegments]) -> None:
        if len(reference_lines) != len(route_segments):
            return

        def same_point_xy(lhs, rhs) -> bool:
            return abs(lhs.x - rhs.x) < 1.0e-8 and abs(lhs.y - rhs.y) < 1.0e-8

        with self._reference_lines_mutex:
            if len(self._reference_lines) != len(reference_lines):
                self._reference_lines = [deepcopy(rl) for rl in reference_lines]
                self._route_segments = [deepcopy(seg) for seg in route_segments]
            else:
                for index, (reference_line, route_segment) in enumerate(
                    zip(reference_lines, route_segments)
                ):
                    points = reference_line.reference_points
                    internal_points = self._reference_lines[index].reference_points
                    if (
                        points
                        and internal_points
                        and same_point_xy(points[0], internal_points[0])
                        and same_point_xy(points[-1], internal_points[-1])
                        and abs(
                            reference_line.Length()
                            - self._reference_lines[index].Length()
                        )
                        < 1.0e-10
                    ):
                        continue
                    self._reference_lines[index] = deepcopy(reference_line)
                    self._route_segments[index] = deepcopy(route_segment)
            self._is_reference_line_updated = True
            if self._reference_lines and self._route_segments:
                self._reference_line_history.append([deepcopy(rl) for rl in self._reference_lines])
                self._route_segments_history.append([deepcopy(seg) for seg in self._route_segments])

    def GenerateThread(self) -> None:
        while not self._is_stop:
            self._is_reference_line_updated = True
            time.sleep(0.05)
            if not self._has_routing:
                continue
            start_time = time.time()
            tag, reference_lines, route_segments = self.CreateReferenceLine()
            if tag:
                self.UpdateReferenceLine(reference_lines, route_segments)
                self._last_calculation_time = time.time() - start_time
            else:
                self._is_reference_line_updated = False

    def IsValidReferenceLine(self) -> bool:
        return bool(self._reference_lines) and len(self._reference_lines) == len(self._route_segments)

    def PrioritizeChangeLane(self, route_segments: List[RouteSegments]) -> None:
        for index, segment in enumerate(route_segments):
            if not segment.IsOnSegment():
                route_segments.insert(0, route_segments.pop(index))
                break

    def CreateRouteSegments(self, vehicle_state: VehicleState) -> Tuple[bool, List[RouteSegments]]:
        if vehicle_state is None or self._pnc_map is None:
            return False, []

        with self._pnc_map_mutex:
            segments = self._pnc_map.GetRouteSegments(vehicle_state)
        if not segments:
            return False, []
        if config_module.FLAGS_prioritize_change_lane:
            self.PrioritizeChangeLane(segments)
        return True, segments

    def IsReferenceLineSmoothValid(self, raw: ReferenceLine, smoothed: ReferenceLine) -> bool:
        if raw is None or smoothed is None or not raw.reference_points or not smoothed.reference_points:
            return False
        step = 10.0
        s = 0.0
        while s < smoothed.Length():
            xy_new = smoothed.GetReferencePoint(s)
            ok, sl_new = raw.XYToSL(Vec2d(xy_new.x, xy_new.y))
            if not ok or sl_new is None:
                return False
            if abs(sl_new.l) > config_module.FLAGS_smoothed_reference_line_max_diff:
                return False
            s += step
        return True

    def SmoothReferenceLine(self, raw_reference_line: ReferenceLine) -> Tuple[bool, ReferenceLine]:
        if raw_reference_line is None:
            return False, None
        if not config_module.FLAGS_enable_smooth_reference_line:
            return True, ReferenceLine(raw_reference_line)

        anchor_points = self.GetAnchorPoints(raw_reference_line)
        smoother = self._active_smoother()
        smoother.SetAnchorPoints(anchor_points)
        smoothed = smoother.Smooth(raw_reference_line)
        if smoothed is None:
            return True, ReferenceLine(raw_reference_line)
        if not self.IsReferenceLineSmoothValid(raw_reference_line, smoothed):
            return True, ReferenceLine(raw_reference_line)
        return True, smoothed

    def SmoothPrefixedReferenceLine(
        self, prefix_ref: ReferenceLine, raw_ref: ReferenceLine
    ) -> Tuple[bool, ReferenceLine]:
        if raw_ref is None:
            return False, None
        if not config_module.FLAGS_enable_smooth_reference_line:
            return True, ReferenceLine(raw_ref)

        anchor_points = self.GetAnchorPoints(raw_ref)
        for point in anchor_points:
            ok, sl_point = prefix_ref.XYToSL(Vec2d(point.path_point.x, point.path_point.y))
            if not ok or sl_point is None:
                continue
            if sl_point.s < 0 or sl_point.s > prefix_ref.Length():
                continue
            prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s)
            point.path_point.x = prefix_ref_point.x
            point.path_point.y = prefix_ref_point.y
            point.path_point.z = 0.0
            point.path_point.theta = prefix_ref_point.heading
            point.lateral_bound = 0.0
            point.longitudinal_bound = 1e-6
            point.enforced = True
            break

        smoother = self._active_smoother()
        smoother.SetAnchorPoints(anchor_points)
        smoothed = smoother.Smooth(raw_ref)
        if smoothed is None:
            return False, None
        if not self.IsReferenceLineSmoothValid(raw_ref, smoothed):
            return False, None
        return True, smoothed

    def ExtendReferenceLine(
        self, state: VehicleState, segments: RouteSegments
    ) -> Tuple[bool, Optional[ReferenceLine]]:
        if state is None:
            return False, None

        segment_properties = RouteSegments()
        segment_properties.SetProperties(segments)

        with self._reference_lines_mutex:
            prev_segments = list(self._route_segments)
            prev_refs = list(self._reference_lines)

        prev_segment = None
        prev_ref = None
        for idx, candidate in enumerate(prev_segments):
            if candidate.IsConnectedSegment(segments):
                prev_segment = candidate
                prev_ref = prev_refs[idx] if idx < len(prev_refs) else None
                break

        if prev_segment is None or prev_ref is None:
            smooth_ok, reference_line = self.SmoothRouteSegment(segments)
            return smooth_ok, reference_line

        vec2d = Vec2d(state.x, state.y)
        has_projection, sl_point, _ = prev_segment.GetProjection(vec2d)
        if not has_projection or sl_point is None:
            smooth_ok, reference_line = self.SmoothRouteSegment(segments)
            return smooth_ok, reference_line

        prev_segment_length = RouteSegments.Length(prev_segment)
        remain_s = prev_segment_length - sl_point.s
        look_forward_required_distance = PncMap.LookForwardDistance(state.linear_velocity or 0.0)
        if remain_s > look_forward_required_distance:
            segments.clear()
            for item in prev_segment:
                segments.append(item)
            segments.SetProperties(segment_properties)
            return True, ReferenceLine(prev_ref)

        future_start_s = max(
            sl_point.s,
            prev_segment_length - config_module.FLAGS_reference_line_stitch_overlap_distance,
        )
        future_end_s = prev_segment_length + config_module.FLAGS_look_forward_extend_distance

        with self._pnc_map_mutex:
            shifted_segments = RouteSegments()
            if self._pnc_map is None or not self._pnc_map.ExtendSegments(
                prev_segment, future_start_s, future_end_s, shifted_segments
            ):
                smooth_ok, reference_line = self.SmoothRouteSegment(segments)
                return smooth_ok, reference_line

        if prev_segment.IsWaypointOnSegment(shifted_segments.LastWayPoint()):
            segments.clear()
            for item in prev_segment:
                segments.append(item)
            segments.SetProperties(segment_properties)
            return True, ReferenceLine(prev_ref)

        new_ref = ReferenceLine(MapPath(shifted_segments))
        smooth_ok, reference_line = self.SmoothPrefixedReferenceLine(prev_ref, new_ref)
        if not smooth_ok or reference_line is None:
            smooth_ok, reference_line = self.SmoothRouteSegment(segments)
            return smooth_ok, reference_line

        if not reference_line.Stitch(prev_ref):
            smooth_ok, reference_line = self.SmoothRouteSegment(segments)
            return smooth_ok, reference_line

        if not shifted_segments.Stitch(prev_segment):
            smooth_ok, reference_line = self.SmoothRouteSegment(segments)
            return smooth_ok, reference_line

        segments.clear()
        for item in shifted_segments:
            segments.append(item)
        segments.SetProperties(segment_properties)

        ok_sl, sl = reference_line.XYToSL(vec2d)
        if ok_sl and sl is not None:
            self.Shrink(sl, reference_line, segments)
        return True, reference_line

    def GetAnchorPoints(self, reference_line: ReferenceLine) -> List[AnchorPoint]:
        interval = config_module.FLAGS_max_constraint_interval
        num_of_anchors = max(2, int(reference_line.Length() / interval + 0.5))
        anchor_s_values = uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1)
        anchor_points = [self.GetAnchorPoint(reference_line, s) for s in anchor_s_values]
        if anchor_points:
            anchor_points[0].longitudinal_bound = 1e-6
            anchor_points[0].lateral_bound = 1e-6
            anchor_points[0].enforced = True
            anchor_points[-1].longitudinal_bound = 1e-6
            anchor_points[-1].lateral_bound = 1e-6
            anchor_points[-1].enforced = True
        return anchor_points

    def SmoothRouteSegment(self, segments: RouteSegments) -> Tuple[bool, ReferenceLine]:
        raw_reference_line = ReferenceLine(MapPath(segments))
        return self.SmoothReferenceLine(raw_reference_line)

    @staticmethod
    def _boundary_type_at(boundary: LaneBoundary, s: float) -> LaneBoundaryType.LaneBoundaryTypeEnum:
        if boundary is None or not boundary.boundary_type:
            return LaneBoundaryType.LaneBoundaryTypeEnum.UNKNOWN
        selected = boundary.boundary_type[0]
        for boundary_type in boundary.boundary_type:
            if boundary_type.s is None or boundary_type.s <= s:
                selected = boundary_type
            else:
                break
        if selected.types:
            return selected.types[0]
        return LaneBoundaryType.LaneBoundaryTypeEnum.UNKNOWN

    def Shrink(
        self, sl: SLPoint, reference_line: ReferenceLine, segments: RouteSegments
    ) -> bool:
        k_max_heading_diff = math.pi * 5.0 / 6.0
        new_backward_distance = sl.s
        new_forward_distance = reference_line.Length() - sl.s
        need_shrink = False

        if sl.s > config_module.FLAGS_look_backward_distance * 1.5:
            new_backward_distance = config_module.FLAGS_look_backward_distance
            need_shrink = True

        index = reference_line.GetNearestReferenceIndex(sl.s)
        ref_points = reference_line.reference_points
        if not ref_points:
            return True
        cur_heading = ref_points[index].heading
        last_index = index
        while last_index < len(ref_points) and AngleDiff(
            cur_heading, ref_points[last_index].heading
        ) < k_max_heading_diff:
            last_index += 1
        last_index -= 1
        if last_index != len(ref_points) - 1:
            need_shrink = True
            ok, forward_sl = reference_line.XYToSL(ref_points[last_index])
            if ok and forward_sl is not None:
                new_forward_distance = forward_sl.s - sl.s

        if need_shrink:
            reference_line.Segment(sl.s, new_backward_distance, new_forward_distance)
            segments.Shrink(sl.s, new_backward_distance, new_forward_distance)
        return True

    def GetAnchorPoint(self, reference_line: ReferenceLine, s: float) -> AnchorPoint:
        ref_point = reference_line.GetReferencePoint(s)
        anchor = AnchorPoint(
            path_point=ref_point.ToPathPoint(s),
            longitudinal_bound=config_module.FLAGS_longitudinal_boundary_bound,
            lateral_bound=config_module.FLAGS_max_lateral_boundary_bound,
        )
        lane_waypoints = ref_point.lane_waypoints
        if not lane_waypoints:
            return anchor

        waypoint = lane_waypoints[0]
        left_width, right_width, _ = waypoint.lane.GetWidth(waypoint.s)
        safe_lane_width = left_width + right_width - 2.0 * config_module.FLAGS_half_vehicle_width
        effective_width = 0.0
        is_lane_width_safe = True
        if safe_lane_width < 1e-8:
            effective_width = 1e-8
            is_lane_width_safe = False

        center_shift = 0.0
        lane = waypoint.lane.lane
        if self._boundary_type_at(lane.right_boundary, waypoint.s) == LaneBoundaryType.LaneBoundaryTypeEnum.CURB:
            safe_lane_width -= config_module.FLAGS_curb_shift
            if safe_lane_width < 1e-8:
                effective_width = 1e-8
                is_lane_width_safe = False
            else:
                center_shift += 0.5 * config_module.FLAGS_curb_shift
        if self._boundary_type_at(lane.left_boundary, waypoint.s) == LaneBoundaryType.LaneBoundaryTypeEnum.CURB:
            safe_lane_width -= config_module.FLAGS_curb_shift
            if safe_lane_width < 1e-8:
                effective_width = 1e-8
                is_lane_width_safe = False
            else:
                center_shift -= 0.5 * config_module.FLAGS_curb_shift

        buffered_width = safe_lane_width - 2.0 * config_module.FLAGS_lateral_buffer
        if buffered_width >= 1e-8:
            safe_lane_width = buffered_width
        if is_lane_width_safe:
            effective_width = 0.5 * safe_lane_width

        if center_shift:
            left_vec = Vec2d.CreateUnitVec2d(ref_point.heading + math.pi / 2.0)
            anchor.path_point.x = ref_point.x + left_vec.x * center_shift
            anchor.path_point.y = ref_point.y + left_vec.y * center_shift
        anchor.lateral_bound = min(config_module.FLAGS_max_lateral_boundary_bound, effective_width)
        anchor.lateral_bound = max(config_module.FLAGS_min_lateral_boundary_bound, anchor.lateral_bound)
        return anchor

    def GetReferenceLinesFromRelativeMap(self) -> Tuple[List[ReferenceLine], List[RouteSegments]]:
        relative_map = self._relative_map
        if relative_map is None or not relative_map.navigation_path:
            return [], []

        hdmap = HDMapUtil.BaseMapPtr(relative_map)
        navigation_lane_ids = set(relative_map.navigation_path.keys())
        vehicle_state = self._vehicle_state
        if vehicle_state is None and self._vehicle_state_provider is not None:
            vehicle_state = self._vehicle_state_provider.vehicle_state
        if vehicle_state is None:
            return [], []

        adc_lane_waypoint = self.GetNearestWayPointFromNavigationPath(
            vehicle_state, navigation_lane_ids
        )
        if adc_lane_waypoint is None or adc_lane_waypoint.lane is None:
            return [], []

        adc_lane_id = adc_lane_waypoint.lane.id.id
        adc_navigation_path = relative_map.navigation_path.get(adc_lane_id)
        if adc_navigation_path is None:
            return [], []
        adc_lane_priority = adc_navigation_path.path_priority or 0

        left_neighbor_lane_ids: List[str] = []
        left_lane_ptr = adc_lane_waypoint.lane
        while left_lane_ptr is not None and left_lane_ptr.lane.left_neighbor_forward_lane_id:
            neighbor_id = left_lane_ptr.lane.left_neighbor_forward_lane_id[0].id
            left_neighbor_lane_ids.append(neighbor_id)
            left_lane_ptr = hdmap.GetLaneById(MakeMapId(neighbor_id))

        right_neighbor_lane_ids: List[str] = []
        right_lane_ptr = adc_lane_waypoint.lane
        while right_lane_ptr is not None and right_lane_ptr.lane.right_neighbor_forward_lane_id:
            neighbor_id = right_lane_ptr.lane.right_neighbor_forward_lane_id[0].id
            right_neighbor_lane_ids.append(neighbor_id)
            right_lane_ptr = hdmap.GetLaneById(MakeMapId(neighbor_id))

        high_priority_lane_pairs: List[Tuple[str, int]] = []
        for lane_id, nav_path in relative_map.navigation_path.items():
            priority = nav_path.path_priority or 0
            if adc_lane_id != lane_id and priority < adc_lane_priority:
                high_priority_lane_pairs.append((lane_id, priority))

        is_lane_change_needed = bool(high_priority_lane_pairs)
        target_lane_pair = ("", adc_lane_priority)
        if high_priority_lane_pairs:
            high_priority_lane_pairs.sort(key=lambda item: item[1])
            target_lane_pair = high_priority_lane_pairs[0]

        lane_change_type = ChangeLaneType.FORWARD
        nearest_neighbor_lane_id = ""
        if is_lane_change_needed:
            if target_lane_pair[0] in left_neighbor_lane_ids:
                lane_change_type = ChangeLaneType.LEFT
                if adc_lane_waypoint.lane.lane.left_neighbor_forward_lane_id:
                    nearest_neighbor_lane_id = (
                        adc_lane_waypoint.lane.lane.left_neighbor_forward_lane_id[0].id
                    )
            elif target_lane_pair[0] in right_neighbor_lane_ids:
                lane_change_type = ChangeLaneType.RIGHT
                if adc_lane_waypoint.lane.lane.right_neighbor_forward_lane_id:
                    nearest_neighbor_lane_id = (
                        adc_lane_waypoint.lane.lane.right_neighbor_forward_lane_id[0].id
                    )

        reference_lines: List[ReferenceLine] = []
        route_segments: List[RouteSegments] = []
        for lane_id, nav_path in relative_map.navigation_path.items():
            if nav_path.path is None or not nav_path.path.path_point:
                continue
            lane_ptr = hdmap.GetLaneById(MakeMapId(lane_id))
            if lane_ptr is None:
                continue
            segment = RouteSegments()
            segment.append(LaneSegment(lane=lane_ptr, start_s=0.0, end_s=lane_ptr.total_length))
            segment.SetCanExit(True)
            segment.SetId(lane_id)
            segment.SetNextAction(ChangeLaneType.FORWARD)
            segment.SetStopForDestination(False)
            segment.SetPreviousAction(ChangeLaneType.FORWARD)
            if is_lane_change_needed:
                if lane_id == nearest_neighbor_lane_id:
                    segment.SetIsNeighborSegment(True)
                    segment.SetPreviousAction(lane_change_type)
                elif lane_id == adc_lane_id:
                    segment.SetIsOnSegment(True)
                    segment.SetNextAction(lane_change_type)

            ref_points: List[ReferencePoint] = []
            for path_point in nav_path.path.path_point:
                ref_points.append(
                    ReferencePoint(
                        MapPathPoint(
                            Vec2d(path_point.x, path_point.y),
                            path_point.theta or 0.0,
                            [LaneWaypoint(lane_ptr, path_point.s or 0.0)],
                        ),
                        path_point.kappa or 0.0,
                        path_point.dkappa or 0.0,
                    )
                )
            if len(ref_points) < 2:
                continue
            reference_line = ReferenceLine(ref_points)
            reference_line.SetPriority(nav_path.path_priority or 0)
            reference_lines.append(reference_line)
            route_segments.append(segment)

        if reference_lines:
            self.UpdateReferenceLine(reference_lines, route_segments)
        return reference_lines, route_segments

    def GetNearestWayPointFromNavigationPath(
        self, state: VehicleState, navigation_lane_ids: Set[str]
    ) -> Optional[LaneWaypoint]:
        k_max_distance = 10.0
        if state is None or state.x is None or state.y is None:
            return None
        if math.isnan(state.x) or math.isnan(state.y):
            return None

        hdmap = HDMapUtil.BaseMap()
        point = PointENU(x=state.x, y=state.y)
        lanes = hdmap.GetLanesWithHeading(
            point, k_max_distance, state.heading or 0.0, math.pi / 2.0
        )
        valid_lanes = [lane for lane in lanes if lane.id.id in navigation_lane_ids]
        if not valid_lanes:
            return None

        min_distance = float("inf")
        best_waypoint: Optional[LaneWaypoint] = None
        xy = Vec2d(state.x, state.y)
        for lane in valid_lanes:
            ok, s, _ = lane.GetProjection(xy)
            if not ok:
                continue
            if s > lane.total_length + 1e-6 or s + 1e-6 < 0.0:
                continue
            map_point, distance = lane.GetNearestPoint(xy)
            if distance < min_distance:
                nearest_xy = Vec2d(map_point.x, map_point.y)
                ok, s, _ = lane.GetProjection(nearest_xy)
                if not ok:
                    continue
                min_distance = distance
                best_waypoint = LaneWaypoint(lane, s)
        return best_waypoint
