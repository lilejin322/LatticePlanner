"""Path decider aligned with modules/planning/tasks/deciders/path_decider/path_decider.cc."""

from __future__ import annotations

from typing import Optional

from common.st_boundary import STBoundary
from common.path_decision import PathDecision
from common.path_data import PathData
from common.planning_context import PlanningContext
from reference_line.reference_line_info import ReferenceLineInfo
from common.status import Status
from protoclass.decision_result import (
    ObjectDecisionType,
    ObjectIgnore,
    ObjectNudge,
    ObjectStop,
    StopReasonCode,
)
from protoclass.header import ErrorCode
from protoclass.point_enu import PointENU
import config as config_module


class PathDecider:
    """Static obstacle lateral/longitudinal decisions from frenet path."""

    def __init__(self, reference_line_info: ReferenceLineInfo):
        self._reference_line_info = reference_line_info

    def Execute(
        self,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext] = None,
    ) -> Status:
        if reference_line_info.path_reusable and config_module.FLAGS_enable_skip_path_tasks:
            return Status.OK()
        blocking_obstacle = reference_line_info.GetBlockingObstacle()
        blocking_obstacle_id = blocking_obstacle.Id() if blocking_obstacle is not None else ""
        if not self.MakeObjectDecision(
            reference_line_info.path_data,
            blocking_obstacle_id,
            reference_line_info.path_decision,
            reference_line_info,
            planning_context,
        ):
            return Status(ErrorCode.PLANNING_ERROR, "Failed to make decision based on tunnel")
        return Status.OK()

    def MakeObjectDecision(
        self,
        path_data: PathData,
        blocking_obstacle_id: str,
        path_decision: PathDecision,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext],
    ) -> bool:
        return self.MakeStaticObstacleDecision(
            path_data,
            blocking_obstacle_id,
            path_decision,
            reference_line_info,
            planning_context,
        )

    def MakeStaticObstacleDecision(
        self,
        path_data: PathData,
        blocking_obstacle_id: str,
        path_decision: PathDecision,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext],
    ) -> bool:
        frenet_path = path_data.frenet_frame_path
        if not frenet_path:
            return False

        half_width = config_module.FLAGS_half_vehicle_width
        lateral_radius = half_width + config_module.FLAGS_lateral_ignore_buffer
        static_obstacle_buffer = config_module.FLAGS_path_decider_static_obstacle_buffer
        min_nudge_l = half_width + static_obstacle_buffer / 2.0

        lane_borrow = False
        if planning_context is not None:
            lane_borrow = bool(
                planning_context.planning_status.path_decider.is_in_path_lane_borrow_scenario
            )

        reference_line = reference_line_info.reference_line
        adc_sl_boundary = reference_line_info.AdcSlBoundary()

        for obstacle in path_decision.obstacles.values():
            if not obstacle.IsStatic() or obstacle.IsVirtual():
                continue

            if (
                obstacle.HasLongitudinalDecision()
                and obstacle.LongitudinalDecision().ignore is not None
                and obstacle.HasLateralDecision()
                and obstacle.LateralDecision().ignore is not None
            ):
                continue
            if obstacle.HasLongitudinalDecision() and obstacle.LongitudinalDecision().stop is not None:
                continue

            if obstacle.Id() == blocking_obstacle_id and not lane_borrow:
                stop_decision = ObjectDecisionType()
                stop_decision.object_tag = self.GenerateObjectStopDecision(obstacle, reference_line_info)
                path_decision.AddLongitudinalDecision(
                    "PathDecider/blocking_obstacle", obstacle.Id(), stop_decision
                )
                continue

            if obstacle.reference_line_st_boundary().boundary_type == STBoundary.BoundaryType.KEEP_CLEAR:
                continue

            ignore_decision = ObjectDecisionType()
            ignore_decision.object_tag = ObjectIgnore()

            sl_boundary = obstacle.PerceptionSLBoundary()
            if sl_boundary.end_s < frenet_path[0].s or sl_boundary.start_s > frenet_path[-1].s:
                path_decision.AddLongitudinalDecision("PathDecider/not-in-s", obstacle.Id(), ignore_decision)
                path_decision.AddLateralDecision("PathDecider/not-in-s", obstacle.Id(), ignore_decision)
                continue

            frenet_point = frenet_path.GetNearestPoint(sl_boundary)
            curr_l = frenet_point.l

            if curr_l - lateral_radius > sl_boundary.end_l or curr_l + lateral_radius < sl_boundary.start_l:
                path_decision.AddLateralDecision("PathDecider/not-in-l", obstacle.Id(), ignore_decision)
            elif sl_boundary.end_l >= curr_l - min_nudge_l and sl_boundary.start_l <= curr_l + min_nudge_l:
                stop_obj = self.GenerateObjectStopDecision(obstacle, reference_line_info)
                stop_decision = ObjectDecisionType()
                stop_decision.object_tag = stop_obj
                if path_decision.MergeWithMainStop(
                    stop_obj, obstacle.Id(), reference_line, adc_sl_boundary
                ):
                    path_decision.AddLongitudinalDecision(
                        "PathDecider/nearest-stop", obstacle.Id(), stop_decision
                    )
                else:
                    path_decision.AddLongitudinalDecision(
                        "PathDecider/not-nearest-stop", obstacle.Id(), ignore_decision
                    )
            else:
                nudge_decision = ObjectDecisionType()
                if sl_boundary.end_l < curr_l - min_nudge_l:
                    nudge_decision.object_tag = ObjectNudge(
                        type=ObjectNudge.Type.LEFT_NUDGE,
                        distance_l=static_obstacle_buffer,
                    )
                    path_decision.AddLateralDecision("PathDecider/left-nudge", obstacle.Id(), nudge_decision)
                elif sl_boundary.start_l > curr_l + min_nudge_l:
                    nudge_decision.object_tag = ObjectNudge(
                        type=ObjectNudge.Type.RIGHT_NUDGE,
                        distance_l=-static_obstacle_buffer,
                    )
                    path_decision.AddLateralDecision("PathDecider/right-nudge", obstacle.Id(), nudge_decision)

        return True

    def GenerateObjectStopDecision(
        self, obstacle, reference_line_info: ReferenceLineInfo
    ) -> ObjectStop:
        stop_distance = obstacle.MinRadiusStopDistance()
        sl_boundary = obstacle.PerceptionSLBoundary()
        stop_ref_s = sl_boundary.start_s - stop_distance
        stop_ref_point = reference_line_info.reference_line.GetReferencePoint(stop_ref_s)
        return ObjectStop(
            reason_code=StopReasonCode.STOP_REASON_OBSTACLE,
            distance_s=-stop_distance,
            stop_point=PointENU(x=stop_ref_point.x, y=stop_ref_point.y, z=0.0),
            stop_heading=stop_ref_point.heading,
        )
