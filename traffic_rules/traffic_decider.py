"""Traffic rules: upstream stop_point generation for lattice planner."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

from common.frame import Frame
from common.hd_map import HDMapUtil
from common.planning_context import PlanningContext
from reference_line.reference_line_info import ReferenceLineInfo
from common.st_boundary import STBoundary
from common.status import Status
from common.vec2d import Vec2d
from common.planning_util import (
    BuildStopDecision,
    BuildStopDecisionOnLane,
    CrosswalkPolygon,
    GetADCStopDeceleration,
)
from protoclass.decision_result import (
    ChangeLaneType,
    ObjectDecisionType,
    ObjectIgnore,
    ObjectStop,
    StopReasonCode,
)
from protoclass.perception_obstacle import PerceptionObstacleType
from protoclass.planning_status import StopTime
from protoclass.point_enu import PointENU
from protoclass.sl_boundary import SLPoint
from protoclass.traffic_light_detection import TrafficLight
from protoclass.lattice_structure import StopPoint
from protoclass.header import ErrorCode
from config import (
    CROSSWALK_VO_ID_PREFIX,
    FLAGS_backside_vehicle_lane_width,
    FLAGS_crosswalk_enabled,
    FLAGS_crosswalk_expand_s_distance,
    FLAGS_crosswalk_max_stop_deceleration,
    FLAGS_crosswalk_min_pass_s_distance,
    FLAGS_crosswalk_stop_distance,
    FLAGS_crosswalk_stop_loose_l_distance,
    FLAGS_crosswalk_stop_strict_l_distance,
    FLAGS_crosswalk_stop_timeout,
    FLAGS_destination_obstacle_id,
    FLAGS_destination_stop_distance,
    FLAGS_keep_clear_align_with_traffic_sign_tolerance,
    FLAGS_keep_clear_junction_enabled,
    FLAGS_keep_clear_min_pass_s_distance,
    FLAGS_keep_clear_zone_enabled,
    FLAGS_reference_line_end_min_remain_length,
    FLAGS_reference_line_end_stop_distance,
    FLAGS_rerouting_cooldown_time,
    FLAGS_rerouting_prepare_time,
    FLAGS_stop_sign_enabled,
    FLAGS_stop_sign_stop_distance,
    FLAGS_traffic_light_enabled,
    FLAGS_traffic_light_max_stop_deceleration,
    FLAGS_traffic_light_stop_distance,
    FLAGS_virtual_stop_wall_length,
    FLAGS_yield_sign_enabled,
    FLAGS_yield_sign_stop_distance,
    FRONT_EDGE_TO_CENTER,
    KEEP_CLEAR_JUNCTION_VO_ID_PREFIX,
    KEEP_CLEAR_VO_ID_PREFIX,
    REF_LINE_END_VO_ID_PREFIX,
    STOP_SIGN_VO_ID_PREFIX,
    TRAFFIC_LIGHT_VO_ID_PREFIX,
    YIELD_SIGN_VO_ID_PREFIX,
)


@dataclass
class TrafficRuleConfig:
    rule_id: str
    enabled: bool = True


@dataclass
class TrafficRuleConfigs:
    config: List[TrafficRuleConfig] = field(default_factory=list)


DEFAULT_TRAFFIC_RULE_CONFIGS = TrafficRuleConfigs(
    config=[
        TrafficRuleConfig("BACKSIDE_VEHICLE", enabled=False),
        TrafficRuleConfig("CROSSWALK"),
        TrafficRuleConfig("DESTINATION"),
        TrafficRuleConfig("KEEP_CLEAR"),
        TrafficRuleConfig("REFERENCE_LINE_END"),
        TrafficRuleConfig("REROUTING"),
        TrafficRuleConfig("STOP_SIGN"),
        TrafficRuleConfig("TRAFFIC_LIGHT"),
        TrafficRuleConfig("YIELD_SIGN"),
    ]
)


class TrafficRule:
    def __init__(self, config: TrafficRuleConfig, planning_context: Optional[PlanningContext] = None):
        self.config = config
        self.planning_context = planning_context

    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        raise NotImplementedError


class ReferenceLineEndRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        reference_line = reference_line_info.reference_line
        remain_s = reference_line.Length() - reference_line_info.AdcSlBoundary().end_s
        if remain_s > FLAGS_reference_line_end_min_remain_length:
            return Status.OK()

        virtual_obstacle_id = REF_LINE_END_VO_ID_PREFIX + reference_line_info.Lanes().Id()
        obstacle_start_s = reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length
        obstacle = frame.CreateStopObstacle(reference_line_info, virtual_obstacle_id, obstacle_start_s)
        if obstacle is None:
            return Status(ErrorCode.PLANNING_ERROR, "Failed to create reference line end obstacle")

        stop_wall = reference_line_info.AddObstacle(obstacle)
        if stop_wall is None:
            return Status(ErrorCode.PLANNING_ERROR, "Failed to add reference line end obstacle")

        stop_line_s = obstacle_start_s - FLAGS_reference_line_end_stop_distance
        stop_point = reference_line.GetReferencePoint(stop_line_s)
        stop_obj = ObjectStop(
            reason_code=StopReasonCode.STOP_REASON_DESTINATION,
            distance_s=-FLAGS_reference_line_end_stop_distance,
            stop_heading=stop_point.heading,
            stop_point=PointENU(x=stop_point.x, y=stop_point.y, z=0.0),
        )
        stop = ObjectDecisionType()
        stop.object_tag = stop_obj
        reference_line_info.path_decision.AddLongitudinalDecision(
            self.config.rule_id, stop_wall.Id(), stop
        )
        return Status.OK()


class DestinationRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not frame.is_near_destination:
            return Status.OK()

        routing = frame.local_view.routing
        if routing is None or routing.routing_request is None or len(routing.routing_request.waypoint) < 2:
            return Status(ErrorCode.PLANNING_ERROR, "routing_request has no end")

        reference_line = reference_line_info.reference_line
        routing_end = routing.routing_request.waypoint[-1]
        if routing_end.pose is not None:
            ok, dest_sl = reference_line.XYToSL(routing_end.pose)
        else:
            ok = False
            dest_sl = None
        if not ok or dest_sl is None:
            return Status.OK()

        adc_sl = reference_line_info.AdcSlBoundary()
        has_passed_destination = False
        if self.planning_context is not None:
            has_passed_destination = bool(self.planning_context.planning_status.destination.has_passed_destination)
        if adc_sl.start_s > dest_sl.s and not has_passed_destination:
            return Status.OK()

        stop_wall_id = FLAGS_destination_obstacle_id
        dest_lane_s = max(
            0.0,
            (routing_end.s or 0.0) - FLAGS_virtual_stop_wall_length - FLAGS_destination_stop_distance,
        )
        BuildStopDecisionOnLane(
            stop_wall_id,
            routing_end.id,
            dest_lane_s,
            FLAGS_destination_stop_distance,
            StopReasonCode.STOP_REASON_DESTINATION,
            [],
            self.config.rule_id,
            frame,
            reference_line_info,
        )
        return Status.OK()


class StopSignRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not FLAGS_stop_sign_enabled:
            return Status.OK()

        stop_sign_status = None
        if self.planning_context is not None:
            stop_sign_status = self.planning_context.planning_status.stop_sign

        adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s
        stop_sign_overlaps = reference_line_info.reference_line.map_path.stop_sign_overlaps
        for stop_sign_overlap in stop_sign_overlaps:
            if stop_sign_overlap.end_s <= adc_back_edge_s:
                continue
            if (
                stop_sign_status is not None
                and stop_sign_overlap.object_id == stop_sign_status.done_stop_sign_overlap_id
            ):
                continue

            wait_for_obstacle_ids = list(stop_sign_status.wait_for_obstacle_id or []) if stop_sign_status else []
            virtual_obstacle_id = STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id
            BuildStopDecision(
                virtual_obstacle_id,
                stop_sign_overlap.start_s,
                FLAGS_stop_sign_stop_distance,
                StopReasonCode.STOP_REASON_STOP_SIGN,
                wait_for_obstacle_ids,
                self.config.rule_id,
                frame,
                reference_line_info,
            )
        return Status.OK()


class TrafficLightRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not FLAGS_traffic_light_enabled:
            return Status.OK()

        traffic_light_status = None
        if self.planning_context is not None:
            traffic_light_status = self.planning_context.planning_status.traffic_light

        adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s
        adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s
        reference_line = reference_line_info.reference_line
        vehicle_state = reference_line_info._vehicle_state

        for traffic_light_overlap in reference_line_info.reference_line.map_path.signal_overlaps:
            if traffic_light_overlap.end_s <= adc_back_edge_s:
                continue

            if traffic_light_status is not None:
                done_ids = traffic_light_status.done_traffic_light_overlap_id or []
                if traffic_light_overlap.object_id in done_ids:
                    continue

            k_s_discrepancy_tolerance = 10.0
            ok, traffic_light_point = reference_line.SLToXY(
                SLPoint(s=traffic_light_overlap.start_s, l=0.0)
            )
            if ok and vehicle_state is not None:
                adc_position = Vec2d(vehicle_state.x, vehicle_state.y)
                distance = math.hypot(
                    traffic_light_point.x - adc_position.x,
                    traffic_light_point.y - adc_position.y,
                )
                s_distance = traffic_light_overlap.start_s - adc_front_edge_s
                if s_distance >= 0 and abs(s_distance - distance) > k_s_discrepancy_tolerance:
                    continue

            signal_color = frame.GetSignal(traffic_light_overlap.object_id).color
            stop_deceleration = GetADCStopDeceleration(
                vehicle_state, adc_front_edge_s, traffic_light_overlap.start_s
            )
            if signal_color == TrafficLight.Color.GREEN:
                continue
            if stop_deceleration > FLAGS_traffic_light_max_stop_deceleration:
                continue

            virtual_obstacle_id = TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_overlap.object_id
            BuildStopDecision(
                virtual_obstacle_id,
                traffic_light_overlap.start_s,
                FLAGS_traffic_light_stop_distance,
                StopReasonCode.STOP_REASON_SIGNAL,
                [],
                self.config.rule_id,
                frame,
                reference_line_info,
            )
        return Status.OK()


def _check_stop_for_crosswalk_obstacle(
    reference_line_info: ReferenceLineInfo,
    crosswalk_ptr,
    obstacle,
    stop_deceleration: float,
    adc_path_point,
) -> bool:
    perception = obstacle.Perception()
    obstacle_type = perception.type
    allowed_types = {
        PerceptionObstacleType.PEDESTRIAN,
        PerceptionObstacleType.BICYCLE,
        PerceptionObstacleType.UNKNOWN_MOVABLE,
        PerceptionObstacleType.UNKNOWN,
    }
    if obstacle_type not in allowed_types:
        return False

    crosswalk_poly = CrosswalkPolygon(crosswalk_ptr)
    if crosswalk_poly is None:
        return False
    point = Vec2d(perception.position.x, perception.position.y)
    if not crosswalk_poly.ExpandByDistance(FLAGS_crosswalk_expand_s_distance).IsPointIn(point):
        return False

    reference_line = reference_line_info.reference_line
    ok, obstacle_sl_point = reference_line.XYToSL(point)
    if not ok or obstacle_sl_point is None:
        return False

    obstacle_sl_boundary = obstacle.PerceptionSLBoundary()
    obstacle_l_distance = min(abs(obstacle_sl_boundary.start_l), abs(obstacle_sl_boundary.end_l))
    is_on_road = reference_line.IsOnRoad(obstacle_sl_boundary)
    is_path_cross = not obstacle.reference_line_st_boundary().IsEmpty()
    adc_end_edge_s = reference_line_info.AdcSlBoundary().start_s

    stop = False
    if obstacle_l_distance >= FLAGS_crosswalk_stop_loose_l_distance:
        if is_path_cross:
            stop = True
    elif obstacle_l_distance <= FLAGS_crosswalk_stop_strict_l_distance:
        if is_on_road:
            if obstacle_sl_point.s > adc_end_edge_s:
                stop = True
        elif is_path_cross:
            stop = True
        else:
            obstacle_v = Vec2d(perception.velocity.x, perception.velocity.y)
            adc_pt = Vec2d(adc_path_point.x, adc_path_point.y)
            obs_pos = Vec2d(perception.position.x, perception.position.y)
            obs_to_adc = adc_pt - obs_pos
            if obstacle_v.InnerProd(obs_to_adc) > 1e-6:
                stop = True
    elif is_path_cross:
        stop = True

    if stop and stop_deceleration >= FLAGS_crosswalk_max_stop_deceleration:
        if obstacle_l_distance > FLAGS_crosswalk_stop_strict_l_distance:
            stop = False
    return stop


class CrosswalkRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not FLAGS_crosswalk_enabled:
            return Status.OK()

        crosswalk_overlaps = reference_line_info.reference_line.map_path.crosswalk_overlaps
        if not crosswalk_overlaps:
            if self.planning_context is not None:
                status = self.planning_context.planning_status.crosswalk
                status.crosswalk_id = None
                status.stop_time = []
                status.finished_crosswalk = []
            return Status.OK()

        mutable_crosswalk_status = None
        if self.planning_context is not None:
            mutable_crosswalk_status = self.planning_context.planning_status.crosswalk

        adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s
        path_decision = reference_line_info.path_decision
        reference_line = reference_line_info.reference_line
        vehicle_state = reference_line_info._vehicle_state
        adc_path_point = reference_line_info._adc_planning_point.path_point

        crosswalk_stop_timer = {}
        if mutable_crosswalk_status is not None and mutable_crosswalk_status.crosswalk_id:
            stop_times = {
                item.obstacle_id: item.stop_timestamp_sec
                for item in (mutable_crosswalk_status.stop_time or [])
                if item.obstacle_id is not None
            }
            crosswalk_stop_timer[mutable_crosswalk_status.crosswalk_id] = stop_times

        finished_crosswalks = list(mutable_crosswalk_status.finished_crosswalk or []) if mutable_crosswalk_status else []
        crosswalks_to_stop = []

        for crosswalk_overlap in crosswalk_overlaps:
            crosswalk_ptr = HDMapUtil.BaseMap().GetCrosswalkById(crosswalk_overlap.object_id)
            if crosswalk_ptr is None:
                continue
            crosswalk_id = getattr(getattr(crosswalk_ptr, "id", None), "id", crosswalk_overlap.object_id)

            if adc_front_edge_s - crosswalk_overlap.end_s > FLAGS_crosswalk_min_pass_s_distance:
                if mutable_crosswalk_status is not None and mutable_crosswalk_status.crosswalk_id == crosswalk_id:
                    mutable_crosswalk_status.crosswalk_id = None
                    mutable_crosswalk_status.stop_time = []
                continue

            if crosswalk_id in finished_crosswalks:
                continue

            pedestrians = []
            for obstacle in path_decision.obstacles.values():
                stop_deceleration = GetADCStopDeceleration(
                    vehicle_state, adc_front_edge_s, crosswalk_overlap.start_s
                )
                stop = _check_stop_for_crosswalk_obstacle(
                    reference_line_info,
                    crosswalk_ptr,
                    obstacle,
                    stop_deceleration,
                    adc_path_point,
                )

                is_on_lane = reference_line.IsOnLane(obstacle.PerceptionSLBoundary())
                k_start_watch_timer_distance = 40.0
                if (
                    stop
                    and not is_on_lane
                    and crosswalk_overlap.start_s - adc_front_edge_s <= k_start_watch_timer_distance
                ):
                    perception = obstacle.Perception()
                    obstacle_speed = math.hypot(perception.velocity.x, perception.velocity.y)
                    if obstacle_speed <= 0.3:
                        timer = crosswalk_stop_timer.setdefault(crosswalk_id, {})
                        obstacle_id = obstacle.Id()
                        if obstacle_id not in timer:
                            timer[obstacle_id] = time.time()
                        elif time.time() - timer[obstacle_id] >= FLAGS_crosswalk_stop_timeout:
                            stop = False

                if stop:
                    pedestrians.append(obstacle.Id())

            if pedestrians:
                crosswalks_to_stop.append((crosswalk_overlap, pedestrians))

        min_s = float("inf")
        first_crosswalk_to_stop = None
        for crosswalk_overlap, pedestrians in crosswalks_to_stop:
            virtual_obstacle_id = CROSSWALK_VO_ID_PREFIX + crosswalk_overlap.object_id
            BuildStopDecision(
                virtual_obstacle_id,
                crosswalk_overlap.start_s,
                FLAGS_crosswalk_stop_distance,
                StopReasonCode.STOP_REASON_CROSSWALK,
                pedestrians,
                self.config.rule_id,
                frame,
                reference_line_info,
            )
            if crosswalk_overlap.start_s < min_s:
                min_s = crosswalk_overlap.start_s
                first_crosswalk_to_stop = crosswalk_overlap

        if first_crosswalk_to_stop is not None and mutable_crosswalk_status is not None:
            crosswalk = first_crosswalk_to_stop.object_id
            mutable_crosswalk_status.crosswalk_id = crosswalk
            mutable_crosswalk_status.stop_time = [
                StopTime(obstacle_id=oid, stop_timestamp_sec=ts)
                for oid, ts in crosswalk_stop_timer.get(crosswalk, {}).items()
            ]
            mutable_crosswalk_status.finished_crosswalk = [
                overlap.object_id
                for overlap in crosswalk_overlaps
                if overlap.start_s < first_crosswalk_to_stop.start_s
            ]

        return Status.OK()


class YieldSignRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not FLAGS_yield_sign_enabled:
            return Status.OK()

        yield_sign_status = None
        if self.planning_context is not None:
            yield_sign_status = self.planning_context.planning_status.yield_sign

        adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s
        for yield_sign_overlap in reference_line_info.reference_line.map_path.yield_sign_overlaps:
            if yield_sign_overlap.end_s <= adc_front_edge_s:
                continue

            done_ids = list(yield_sign_status.done_yield_sign_overlap_id or []) if yield_sign_status else []
            if yield_sign_overlap.object_id in done_ids:
                continue

            wait_for_obstacle_ids = (
                list(yield_sign_status.wait_for_obstacle_id or []) if yield_sign_status else []
            )
            BuildStopDecision(
                YIELD_SIGN_VO_ID_PREFIX + yield_sign_overlap.object_id,
                yield_sign_overlap.start_s,
                FLAGS_yield_sign_stop_distance,
                StopReasonCode.STOP_REASON_YIELD_SIGN,
                wait_for_obstacle_ids,
                self.config.rule_id,
                frame,
                reference_line_info,
            )
        return Status.OK()


class KeepClearRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if FLAGS_keep_clear_zone_enabled:
            for keep_clear_overlap in reference_line_info.reference_line.map_path.clear_area_overlaps:
                self._build_keep_clear_obstacle(
                    frame,
                    reference_line_info,
                    KEEP_CLEAR_VO_ID_PREFIX + keep_clear_overlap.object_id,
                    keep_clear_overlap.start_s,
                    keep_clear_overlap.end_s,
                )

        if FLAGS_keep_clear_junction_enabled:
            crosswalk_overlap = None
            stop_sign_overlap = None
            traffic_light_overlap = None
            pnc_junction_overlap = None
            for overlap_type, overlap in reference_line_info.FirstEncounteredOverlaps():
                if overlap_type == ReferenceLineInfo.OverlapType.CROSSWALK:
                    crosswalk_overlap = overlap
                elif overlap_type == ReferenceLineInfo.OverlapType.STOP_SIGN:
                    stop_sign_overlap = overlap
                elif overlap_type == ReferenceLineInfo.OverlapType.SIGNAL:
                    traffic_light_overlap = overlap
                elif overlap_type == ReferenceLineInfo.OverlapType.PNC_JUNCTION:
                    pnc_junction_overlap = overlap

            if pnc_junction_overlap is not None:
                adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s
                if not self._is_creeping(pnc_junction_overlap.start_s, adc_front_edge_s):
                    pnc_junction_start_s = pnc_junction_overlap.start_s
                    if (
                        traffic_light_overlap is not None
                        and abs(pnc_junction_start_s - traffic_light_overlap.start_s)
                        <= FLAGS_keep_clear_align_with_traffic_sign_tolerance
                    ):
                        pnc_junction_start_s = traffic_light_overlap.start_s
                    elif (
                        stop_sign_overlap is not None
                        and abs(pnc_junction_start_s - stop_sign_overlap.start_s)
                        <= FLAGS_keep_clear_align_with_traffic_sign_tolerance
                    ):
                        pnc_junction_start_s = stop_sign_overlap.start_s
                    elif (
                        crosswalk_overlap is not None
                        and abs(pnc_junction_start_s - crosswalk_overlap.start_s)
                        <= FLAGS_keep_clear_align_with_traffic_sign_tolerance
                    ):
                        pnc_junction_start_s = crosswalk_overlap.start_s

                    self._build_keep_clear_obstacle(
                        frame,
                        reference_line_info,
                        KEEP_CLEAR_JUNCTION_VO_ID_PREFIX + pnc_junction_overlap.object_id,
                        pnc_junction_start_s,
                        pnc_junction_overlap.end_s,
                    )
        return Status.OK()

    def _is_creeping(self, pnc_junction_start_s: float, adc_front_edge_s: float) -> bool:
        if self.planning_context is None:
            return False
        scenario = getattr(self.planning_context.planning_status, "scenario", None)
        stage_type = getattr(scenario, "stage_type", None)
        creep_stages = {
            "STOP_SIGN_UNPROTECTED_CREEP",
            "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP",
            "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP",
        }
        if stage_type not in creep_stages:
            return False
        return abs(adc_front_edge_s - pnc_junction_start_s) <= 5.0

    def _build_keep_clear_obstacle(
        self,
        frame: Frame,
        reference_line_info: ReferenceLineInfo,
        virtual_obstacle_id: str,
        keep_clear_start_s: float,
        keep_clear_end_s: float,
    ) -> bool:
        adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s
        if adc_front_edge_s - keep_clear_start_s > FLAGS_keep_clear_min_pass_s_distance:
            return False

        obstacle = frame.CreateStaticObstacle(
            reference_line_info, virtual_obstacle_id, keep_clear_start_s, keep_clear_end_s
        )
        if obstacle is None:
            return False
        path_obstacle = reference_line_info.AddObstacle(obstacle)
        if path_obstacle is None:
            return False
        path_obstacle.SetReferenceLineStBoundaryType(STBoundary.BoundaryType.KEEP_CLEAR)
        return True


class BacksideVehicleRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not reference_line_info.Lanes().IsOnSegment():
            return Status.OK()
        self._make_lane_keeping_obstacle_decision(
            reference_line_info.AdcSlBoundary(), reference_line_info.path_decision
        )
        return Status.OK()

    def _make_lane_keeping_obstacle_decision(self, adc_sl_boundary, path_decision) -> None:
        ignore = ObjectDecisionType()
        ignore.object_tag = ObjectIgnore()
        adc_length_s = adc_sl_boundary.end_s - adc_sl_boundary.start_s
        for obstacle in path_decision.obstacles.values():
            if obstacle.PerceptionSLBoundary().end_s >= adc_sl_boundary.end_s or obstacle.IsCautionLevelObstacle():
                continue

            st_boundary = obstacle.reference_line_st_boundary()
            if st_boundary.IsEmpty():
                path_decision.AddLongitudinalDecision("backside_vehicle/no-st-region", obstacle.Id(), ignore)
                path_decision.AddLateralDecision("backside_vehicle/no-st-region", obstacle.Id(), ignore)
                continue
            if st_boundary.min_s < -adc_length_s:
                path_decision.AddLongitudinalDecision("backside_vehicle/st-min-s < adc", obstacle.Id(), ignore)
                path_decision.AddLateralDecision("backside_vehicle/st-min-s < adc", obstacle.Id(), ignore)
                continue

            sl_boundary = obstacle.PerceptionSLBoundary()
            if sl_boundary.start_s < adc_sl_boundary.end_s:
                lane_boundary = FLAGS_backside_vehicle_lane_width
                if sl_boundary.start_l > lane_boundary or sl_boundary.end_l < -lane_boundary:
                    continue
                path_decision.AddLongitudinalDecision("backside_vehicle/sl < adc.end_s", obstacle.Id(), ignore)
                path_decision.AddLateralDecision("backside_vehicle/sl < adc.end_s", obstacle.Id(), ignore)


class ReroutingRule(TrafficRule):
    def ApplyRule(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        if not self._change_lane_fail_rerouting(frame, reference_line_info):
            return Status(ErrorCode.PLANNING_ERROR, "In un-successful lane change case, rerouting failed")
        return Status.OK()

    def _change_lane_fail_rerouting(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> bool:
        k_reroute_threshold_to_end = 20.0
        for ref_line_info in frame.reference_line_info:
            try:
                reached_destination = ref_line_info.ReachedDestination()
                distance_to_destination = ref_line_info.SDistanceToDestination()
            except (KeyError, AttributeError):
                reached_destination = False
                distance_to_destination = float("inf")
            if reached_destination or distance_to_destination < k_reroute_threshold_to_end:
                return True

        segments = reference_line_info.Lanes()
        if segments.NextAction() == ChangeLaneType.FORWARD:
            return True
        if not segments.IsOnSegment():
            return True
        if segments.CanExit():
            return True

        route_end_waypoint = segments.RouteEndWaypoint()
        if route_end_waypoint is None or route_end_waypoint.lane is None:
            return True

        point = route_end_waypoint.lane.GetSmoothPoint(route_end_waypoint.s)
        ok, sl_point = reference_line_info.reference_line.XYToSL(point)
        if not ok or sl_point is None:
            return False
        if not reference_line_info.reference_line.IsOnLane(sl_point):
            return True

        adc_s = reference_line_info.AdcSlBoundary().end_s
        vehicle_state = getattr(frame, "vehicle_state", None)
        speed = getattr(vehicle_state, "linear_velocity", 0.0) or 0.0
        if sl_point.s > adc_s + speed * FLAGS_rerouting_prepare_time:
            return True

        if self.planning_context is None:
            return False
        rerouting = self.planning_context.planning_status.rerouting
        current_time = time.time()
        last_time = getattr(rerouting, "last_rerouting_time", None)
        if last_time is not None and current_time - last_time < FLAGS_rerouting_cooldown_time:
            return True
        if not frame.Rerouting(self.planning_context):
            return False
        rerouting.last_rerouting_time = current_time
        return True


class TrafficDecider:
    _rule_factory = {
        "BACKSIDE_VEHICLE": BacksideVehicleRule,
        "REFERENCE_LINE_END": ReferenceLineEndRule,
        "DESTINATION": DestinationRule,
        "KEEP_CLEAR": KeepClearRule,
        "REROUTING": ReroutingRule,
        "STOP_SIGN": StopSignRule,
        "TRAFFIC_LIGHT": TrafficLightRule,
        "CROSSWALK": CrosswalkRule,
        "YIELD_SIGN": YieldSignRule,
    }

    def __init__(self, planning_context: Optional[PlanningContext] = None):
        self.rule_configs: TrafficRuleConfigs = DEFAULT_TRAFFIC_RULE_CONFIGS
        self._planning_context = planning_context

    def Init(
        self,
        config: Optional[TrafficRuleConfigs] = None,
        planning_context: Optional[PlanningContext] = None,
    ) -> bool:
        if config is not None:
            self.rule_configs = config
        if planning_context is not None:
            self._planning_context = planning_context
        return True

    def Execute(self, frame: Frame, reference_line_info: ReferenceLineInfo) -> Status:
        for rule_config in self.rule_configs.config:
            if not rule_config.enabled:
                continue
            rule_cls = self._rule_factory.get(rule_config.rule_id)
            if rule_cls is None:
                continue
            rule = rule_cls(rule_config, self._planning_context)
            rule.ApplyRule(frame, reference_line_info)
        self.BuildPlanningTarget(reference_line_info)
        return Status.OK()

    @staticmethod
    def BuildPlanningTarget(reference_line_info: ReferenceLineInfo) -> None:
        min_s = float("inf")
        stop_point = StopPoint()
        hard_reasons = {
            StopReasonCode.STOP_REASON_DESTINATION,
            StopReasonCode.STOP_REASON_CROSSWALK,
            StopReasonCode.STOP_REASON_STOP_SIGN,
            StopReasonCode.STOP_REASON_YIELD_SIGN,
            StopReasonCode.STOP_REASON_CREEPER,
            StopReasonCode.STOP_REASON_REFERENCE_END,
            StopReasonCode.STOP_REASON_SIGNAL,
        }
        for obstacle in reference_line_info.path_decision.obstacles.values():
            if not obstacle.IsVirtual() or not obstacle.HasLongitudinalDecision():
                continue
            decision = obstacle.LongitudinalDecision()
            if decision.stop is None:
                continue
            start_s = obstacle.PerceptionSLBoundary().start_s
            if start_s >= min_s:
                continue
            min_s = start_s
            stop_code = decision.stop.reason_code
            if stop_code in hard_reasons:
                stop_point.type = StopPoint.Type.HARD
            elif stop_code == StopReasonCode.STOP_REASON_YELLOW_SIGNAL:
                stop_point.type = StopPoint.Type.SOFT

        if min_s != float("inf"):
            stop_point.s = min_s - FRONT_EDGE_TO_CENTER + FLAGS_virtual_stop_wall_length / 2.0
            reference_line_info.SetLatticeStopPoint(stop_point)
