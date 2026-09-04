"""Lane-borrow state machine aligned with path_lane_borrow_decider.cc."""

from __future__ import annotations

from typing import Optional, Tuple

import config as config_module
from common.frame import Frame
from common.hd_map import HDMapUtil
from common.planning_context import PlanningContext
from common.status import Status
from protoclass.lane import Lane, LaneBoundaryType
from protoclass.point_enu import PointENU
from reference_line.reference_line_info import ReferenceLineInfo


K_INTERSECTION_CLEARANCE_DIST = 20.0
K_ADC_DISTANCE_THRESHOLD = 35.0
K_OBSTACLES_DISTANCE_THRESHOLD = 15.0


def IsParkedVehicle(reference_line, obstacle) -> bool:
    """
    Matches obstacle_blocking_analyzer.cc's IsParkedVehicle: an obstacle is
    considered parked (hence automatically non-movable/side-passable) if it
    sits on a dedicated parking lane, or hugs the road's right edge.

    :param ReferenceLine reference_line: Reference line
    :param Obstacle obstacle: Obstacle to classify
    :returns: True if the obstacle looks parked
    :rtype: bool
    """

    if not config_module.FLAGS_enable_scenario_side_pass_multiple_parked_obstacles:
        return False

    obstacle_sl = obstacle.PerceptionSLBoundary()
    _, _, road_right_width = reference_line.GetRoadWidth(obstacle_sl.start_s)
    max_road_right_width = road_right_width
    _, _, road_right_width = reference_line.GetRoadWidth(obstacle_sl.end_s)
    max_road_right_width = max(max_road_right_width, road_right_width)
    is_at_road_edge = abs(obstacle_sl.start_l) > max_road_right_width - 0.1

    obstacle_box = obstacle.PerceptionBoundingBox()
    lanes = HDMapUtil.BaseMapPtr().GetLanes(
        PointENU(x=obstacle_box.center.x, y=obstacle_box.center.y),
        min(obstacle_box.width, obstacle_box.length),
    )
    is_on_parking_lane = len(lanes) == 1 and lanes[0].lane.type == Lane.LaneType.PARKING

    is_parked = is_on_parking_lane or is_at_road_edge
    return is_parked and obstacle.IsStatic()


class PathLaneBorrowDecider:
    def Process(
        self,
        frame: Frame,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext],
    ) -> Status:
        if config_module.FLAGS_enable_skip_path_tasks and reference_line_info.path_reusable:
            return Status.OK()

        reference_line_info.set_is_path_lane_borrow(False)
        if (
            config_module.FLAGS_allow_lane_borrowing
            and planning_context is not None
            and self._is_necessary_to_borrow_lane(
                frame, reference_line_info, planning_context
            )
        ):
            reference_line_info.set_is_path_lane_borrow(True)
        return Status.OK()

    def _is_necessary_to_borrow_lane(
        self,
        frame: Frame,
        reference_line_info: ReferenceLineInfo,
        planning_context: PlanningContext,
    ) -> bool:
        status = planning_context.planning_status.path_decider
        if status.is_in_path_lane_borrow_scenario:
            if (status.able_to_use_self_lane_counter or 0) >= 6:
                status.is_in_path_lane_borrow_scenario = False
                status.decided_side_pass_direction = []
            return bool(status.is_in_path_lane_borrow_scenario)

        if len(frame.mutable_reference_line_info) != 1:
            return False
        planning_start = frame.PlanningStartPoint()
        if planning_start is None or (planning_start.v or 0.0) >= config_module.FLAGS_lane_borrow_max_speed:
            return False
        if not self._blocking_obstacle_far_from_intersection(
            reference_line_info, status.front_static_obstacle_id
        ):
            return False
        if (
            status.front_static_obstacle_cycle_counter or 0
        ) < config_module.FLAGS_long_term_blocking_obstacle_cycle_threshold:
            return False
        if not self._blocking_obstacle_within_destination(
            reference_line_info, status.front_static_obstacle_id
        ):
            return False
        if not self._is_side_passable_obstacle(
            reference_line_info, status.front_static_obstacle_id
        ):
            return False

        if not status.decided_side_pass_direction:
            left_borrowable, right_borrowable = self._check_lane_borrow(
                reference_line_info
            )
            if not left_borrowable and not right_borrowable:
                status.is_in_path_lane_borrow_scenario = False
                return False
            status.is_in_path_lane_borrow_scenario = True
            if left_borrowable:
                status.decided_side_pass_direction.append(1)
            if right_borrowable:
                status.decided_side_pass_direction.append(2)
        return bool(status.is_in_path_lane_borrow_scenario)

    @staticmethod
    def _find_obstacle(reference_line_info: ReferenceLineInfo, obstacle_id: str):
        if not obstacle_id:
            return None
        return reference_line_info.path_decision.Find(obstacle_id)

    def _blocking_obstacle_within_destination(
        self, reference_line_info: ReferenceLineInfo, obstacle_id: str
    ) -> bool:
        obstacle = self._find_obstacle(reference_line_info, obstacle_id)
        if obstacle is None:
            return True
        obstacle_s = obstacle.PerceptionSLBoundary().start_s
        adc_end_s = reference_line_info.AdcSlBoundary().end_s
        return obstacle_s - adc_end_s <= reference_line_info.SDistanceToDestination()

    def _blocking_obstacle_far_from_intersection(
        self, reference_line_info: ReferenceLineInfo, obstacle_id: str
    ) -> bool:
        obstacle = self._find_obstacle(reference_line_info, obstacle_id)
        if obstacle is None:
            return True
        obstacle_end_s = obstacle.PerceptionSLBoundary().end_s
        for overlap_type, overlap in reference_line_info.FirstEncounteredOverlaps():
            if overlap_type not in (
                ReferenceLineInfo.OverlapType.SIGNAL,
                ReferenceLineInfo.OverlapType.STOP_SIGN,
            ):
                continue
            if overlap.start_s - obstacle_end_s < K_INTERSECTION_CLEARANCE_DIST:
                return False
        return True

    def _is_side_passable_obstacle(
        self, reference_line_info: ReferenceLineInfo, obstacle_id: str
    ) -> bool:
        obstacle = self._find_obstacle(reference_line_info, obstacle_id)
        if obstacle is None or obstacle.IsVirtual() or not obstacle.IsStatic():
            return False
        obstacle_sl = obstacle.PerceptionSLBoundary()
        adc_end_s = reference_line_info.AdcSlBoundary().end_s
        if obstacle_sl.start_s > adc_end_s + K_ADC_DISTANCE_THRESHOLD:
            return False

        if IsParkedVehicle(reference_line_info.reference_line, obstacle):
            return True

        for other in reference_line_info.path_decision.obstacles.values():
            if other.Id() == obstacle.Id() or other.IsVirtual():
                continue
            other_sl = other.PerceptionSLBoundary()
            if (
                other_sl.start_l > obstacle_sl.end_l
                or other_sl.end_l < obstacle_sl.start_l
            ):
                continue
            delta_s = other_sl.start_s - obstacle_sl.end_s
            if 0.0 <= delta_s <= K_OBSTACLES_DISTANCE_THRESHOLD:
                return False
        return True

    @staticmethod
    def _check_lane_borrow(
        reference_line_info: ReferenceLineInfo,
    ) -> Tuple[bool, bool]:
        reference_line = reference_line_info.reference_line
        left_borrowable = True
        right_borrowable = True
        check_s = reference_line_info.AdcSlBoundary().end_s
        lookforward_s = min(check_s + 100.0, reference_line.Length())
        solid = {
            LaneBoundaryType.LaneBoundaryTypeEnum.SOLID_YELLOW,
            LaneBoundaryType.LaneBoundaryTypeEnum.SOLID_WHITE,
        }
        while check_s < lookforward_s:
            ref_point = reference_line.GetNearestReferencePoint(float(check_s))
            if ref_point is None or not ref_point.lane_waypoints:
                return False, False
            left_type, right_type = reference_line.GetLaneBoundaryType(check_s)
            if left_type in solid:
                left_borrowable = False
            if right_type in solid:
                right_borrowable = False
            check_s += 2.0
        return left_borrowable, right_borrowable
