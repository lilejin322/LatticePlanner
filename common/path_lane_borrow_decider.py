"""Lane-borrow state machine aligned with path_lane_borrow_decider.cc."""

from __future__ import annotations

from typing import Optional, Tuple

import config as config_module
from common.frame import Frame
from common.planning_context import PlanningContext
from common.status import Status
from protoclass.lane import LaneBoundaryType
from reference_line.reference_line_info import ReferenceLineInfo


K_INTERSECTION_CLEARANCE_DIST = 20.0
K_ADC_DISTANCE_THRESHOLD = 35.0
K_OBSTACLES_DISTANCE_THRESHOLD = 15.0


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
