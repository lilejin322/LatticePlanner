"""On-lane planning orchestrator aligned with on_lane_planning.cc."""

from __future__ import annotations

import time
import math
from copy import deepcopy
from logging import Logger
from typing import List, Optional, Tuple

from lattice_planner import LatticePlanner
from common.discretized_path import DiscretizedPath
from common.frame import EgoInfo, Frame, LocalView
from common.planning_context import PlanningContext
from common.pnc_map import PncMap
from common.publishable_trajectory import PublishableTrajectory
from reference_line import ReferenceLine
from reference_line.reference_line_info import ReferenceLineInfo
from reference_line.reference_line_provider import ReferenceLineProvider
from common.route_segments import RouteSegments
from common.status import Status
from common.trajectory_stitcher import TrajectoryStitcher
from common.vehicle_state_provider import VehicleStateProvider
from common.vec2d import Vec2d
from common.planning_output import FillPlanningPb, GenerateStopTrajectory
from common.planning_util import AggregateReferenceLineTrajectory
import config as config_module
from protoclass.adc_trajectory import ADCTrajectory, GearPosition
from protoclass.decision_result import DecisionResult
from protoclass.header import ErrorCode
from protoclass.localization_estimate import LocalizationEstimate
from protoclass.routing import RoutingResponse
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState


class OnLanePlanning:
    def __init__(self, reference_line_provider: Optional[ReferenceLineProvider] = None):
        self.logger = Logger("OnLanePlanning")
        self._reference_line_provider = reference_line_provider or ReferenceLineProvider()
        self._vehicle_state_provider = VehicleStateProvider()
        self._lattice_planner = LatticePlanner()
        self._last_publishable_trajectory: Optional[PublishableTrajectory] = None
        self._last_frame: Optional[Frame] = None
        self._seq_num = 0
        self._planning_context = PlanningContext()
        self._last_routing: Optional[RoutingResponse] = None
        """Persists across planning cycles like Apollo's injector_->planning_context(),
        so state such as rerouting.need_rerouting, destination.has_passed_destination,
        and path_decider.decided_side_pass_direction survives from one RunOnce to the
        next instead of resetting every cycle."""

    @property
    def reference_line_provider(self) -> ReferenceLineProvider:
        return self._reference_line_provider

    def RunOnce(
        self,
        local_view: LocalView,
        adc_trajectory: ADCTrajectory,
        planning_context: Optional[PlanningContext] = None,
    ) -> Status:
        start_timestamp = time.time()
        ctx = planning_context if planning_context is not None else self._planning_context
        vehicle_state, vehicle_status = self._resolve_vehicle_state(local_view)
        if vehicle_state is None:
            msg = vehicle_status.error_message if vehicle_status is not None else "vehicle state is unavailable"
            GenerateStopTrajectory(adc_trajectory, VehicleState(x=0.0, y=0.0, heading=0.0))
            FillPlanningPb(start_timestamp, adc_trajectory, local_view)
            return Status(ErrorCode.PLANNING_ERROR, msg)
        if vehicle_state.timestamp is None:
            vehicle_state.timestamp = start_timestamp
        if start_timestamp - vehicle_state.timestamp < config_module.FLAGS_message_latency_threshold:
            vehicle_state = self._align_time_stamp(vehicle_state, start_timestamp)

        if local_view.routing is not None:
            self._update_routing(local_view.routing, ctx)

        if not self._reference_line_provider.UpdatedReferenceLine():
            msg = "Failed to update reference line after rerouting."
            GenerateStopTrajectory(adc_trajectory, vehicle_state)
            FillPlanningPb(start_timestamp, adc_trajectory, local_view)
            return Status(ErrorCode.PLANNING_ERROR, msg)

        self._reference_line_provider.UpdateVehicleState(vehicle_state)

        planning_cycle_time = 1.0 / max(config_module.FLAGS_planning_loop_rate, 1e-3)
        replan_reason_holder: List[str] = []
        stitching_trajectory = TrajectoryStitcher.compute_stitching_trajectory(
            vehicle_state,
            start_timestamp,
            planning_cycle_time,
            config_module.FLAGS_trajectory_stitching_preserved_length,
            True,
            self._last_publishable_trajectory,
            replan_reason_holder,
        )
        replan_reason = replan_reason_holder[0] if replan_reason_holder else ""

        frame_num = self._seq_num
        self._seq_num += 1
        init_status, frame = self._init_frame(
            frame_num, local_view, stitching_trajectory[-1], vehicle_state, ctx
        )
        self._last_frame = frame
        if not init_status.ok():
            GenerateStopTrajectory(adc_trajectory, vehicle_state)
            FillPlanningPb(start_timestamp, adc_trajectory, local_view)
            return init_status

        if config_module.FLAGS_enable_traffic_rules:
            frame.ApplyTrafficRules(planning_context=ctx)
        plan_ok = self._try_path_bounds_lane_follow(
            frame, stitching_trajectory[-1], ctx
        )
        if not plan_ok:
            plan_ok = self._lattice_planner.Plan(
                stitching_trajectory[-1], frame, adc_trajectory, planning_context=ctx
            )
        if not plan_ok:
            GenerateStopTrajectory(adc_trajectory, vehicle_state)
            FillPlanningPb(start_timestamp, adc_trajectory, local_view)
            return Status(ErrorCode.PLANNING_ERROR, "planner failed to make a driving plan")

        best_ref = frame.FindDriveReferenceLineInfo()
        if best_ref is not None:
            AggregateReferenceLineTrajectory(best_ref, stitching_trajectory[-1])

        output_status = self._fill_planning_output(
            frame,
            stitching_trajectory,
            adc_trajectory,
            start_timestamp,
            local_view,
            ctx,
        )
        adc_trajectory.is_replan = len(stitching_trajectory) == 1
        if adc_trajectory.is_replan:
            adc_trajectory.replan_reason = replan_reason
        adc_trajectory.gear = GearPosition.GEAR_DRIVE
        FillPlanningPb(start_timestamp, adc_trajectory, local_view)
        frame.set_current_frame_planned_trajectory(adc_trajectory)
        return output_status

    @staticmethod
    def _is_different_routing(
        first: Optional[RoutingResponse], second: RoutingResponse
    ) -> bool:
        if first is None:
            return True
        first_header = first.header
        second_header = second.header
        if first_header is not None and second_header is not None:
            return first_header.sequence_num != second_header.sequence_num
        return True

    def _update_routing(
        self, routing: RoutingResponse, planning_context: PlanningContext
    ) -> bool:
        if not self._is_different_routing(self._last_routing, routing):
            return False
        self._last_routing = deepcopy(routing)
        self._last_frame = None
        planning_context.Clear()
        self._reference_line_provider.UpdateRoutingResponse(routing)
        self._lattice_planner = LatticePlanner()
        return True

    def _try_path_bounds_lane_follow(
        self,
        frame: Frame,
        planning_start_point: TrajectoryPoint,
        planning_context: PlanningContext,
    ) -> bool:
        """
        Optional lane-follow path task chain before pure lattice fallback.

        This mirrors the C++ separation where OnLane lane-follow tasks may
        produce a path/speed trajectory, while lattice_planner.cc remains a
        pure 1D trajectory-pair search.
        """
        if (
            not config_module.FLAGS_enable_path_bounds_decider
            or not config_module.FLAGS_enable_on_lane_combine_path_and_speed
        ):
            return False

        from common.discretized_trajectory import DiscretizedTrajectory
        from common.path_assessment_decider import PathAssessmentDecider
        from common.path_bounds_decider import PathBoundsDecider
        from common.path_lane_borrow_decider import PathLaneBorrowDecider
        from common.planning_util import (
            BuildCruiseSpeedData,
            BuildOvertakePathDataFromPathBoundary,
            BuildPathDataFromPathBoundary,
        )

        made_plan = False
        for reference_line_info in frame.mutable_reference_line_info:
            if not PathLaneBorrowDecider().Process(
                frame, reference_line_info, planning_context
            ).ok():
                continue

            if not PathBoundsDecider().Process(frame, reference_line_info, planning_context).ok():
                continue

            boundaries = list(reference_line_info.GetCandidatePathBoundaries())
            lane_borrow = bool(
                planning_context.planning_status.path_decider.is_in_path_lane_borrow_scenario
            )
            has_borrow_boundary = any(
                "left" in boundary.label or "right" in boundary.label
                for boundary in boundaries
            )

            candidate_path_data = []
            for boundary in boundaries:
                if boundary.label == "fallback":
                    continue
                if lane_borrow and has_borrow_boundary and "self" in boundary.label:
                    continue
                if "left" in boundary.label or "right" in boundary.label:
                    path_data = BuildOvertakePathDataFromPathBoundary(
                        reference_line_info, boundary
                    )
                else:
                    path_data = BuildPathDataFromPathBoundary(
                        reference_line_info, boundary
                    )
                if path_data is not None and not path_data.Empty():
                    candidate_path_data.append(path_data)

            if not candidate_path_data:
                continue

            reference_line_info.SetCandidatePathData(candidate_path_data)
            if not PathAssessmentDecider().Process(
                frame, reference_line_info, planning_context
            ).ok():
                continue

            if reference_line_info.path_data is None or reference_line_info.path_data.Empty():
                continue

            if config_module.FLAGS_enable_path_decider_after_lateral:
                frame.ApplyPathDecider(planning_context=planning_context)

            reference_line_info.SetSpeedData(
                BuildCruiseSpeedData(reference_line_info)
            )
            trajectory = DiscretizedTrajectory()
            if not reference_line_info.CombinePathAndSpeedProfile(
                planning_start_point.relative_time or 0.0,
                planning_start_point.path_point.s if planning_start_point.path_point else 0.0,
                trajectory,
            ):
                continue
            if len(trajectory) == 0:
                continue

            reference_line_info.SetTrajectory(trajectory)
            reference_line_info.SetCost(reference_line_info.PriorityCost())
            reference_line_info.SetDrivable(True)
            reference_line_info.set_trajectory_type(ADCTrajectory.TrajectoryType.NORMAL)
            made_plan = True

        return made_plan

    def _resolve_vehicle_state(self, local_view: LocalView) -> Tuple[Optional[VehicleState], Optional[Status]]:
        """Fuse localization + chassis like on_lane_planning.cc VehicleStateProvider."""
        from protoclass.chassis import Chassis

        if local_view.localization_estimate is None and local_view.chassis is None:
            return None, Status(ErrorCode.PLANNING_ERROR, "vehicle state is unavailable")

        localization = local_view.localization_estimate or LocalizationEstimate()
        chassis = local_view.chassis or Chassis()
        if chassis.speed_mps is None or (
            isinstance(chassis.speed_mps, float) and math.isnan(chassis.speed_mps)
        ):
            chassis.speed_mps = 0.0
        localization = self._normalize_localization(localization)
        status = self._vehicle_state_provider.Update(localization, chassis)
        if not status.ok():
            return None, status
        return self._vehicle_state_provider.vehicle_state, Status.OK()

    def _align_time_stamp(self, vehicle_state: VehicleState, curr_timestamp: float) -> VehicleState:
        """Mirrors on_lane_planning.cc's AlignTimeStamp: dead-reckon x/y forward
        by (curr_timestamp - vehicle_state.timestamp) using the vehicle state
        provider's constant-velocity/constant-angular-velocity estimate, to
        correct for the delay between localization and "now"."""
        future_xy = self._vehicle_state_provider.EstimateFuturePosition(
            curr_timestamp - vehicle_state.timestamp
        )
        aligned_vehicle_state = deepcopy(vehicle_state)
        aligned_vehicle_state.x = future_xy.x
        aligned_vehicle_state.y = future_xy.y
        aligned_vehicle_state.timestamp = curr_timestamp
        return aligned_vehicle_state

    @staticmethod
    def _normalize_localization(localization: LocalizationEstimate) -> LocalizationEstimate:
        from protoclass.adc_trajectory import Point3D
        from protoclass.pose import Pose

        if localization.pose is None:
            localization.pose = Pose()
        pose = localization.pose
        if pose.position is None:
            from protoclass.point_enu import PointENU

            pose.position = PointENU(x=0.0, y=0.0, z=0.0)
        if pose.heading is None:
            pose.heading = 0.0
        if pose.angular_velocity is None:
            pose.angular_velocity = Point3D(x=0.0, y=0.0, z=0.0)
        if pose.linear_acceleration is None:
            pose.linear_acceleration = Point3D(x=0.0, y=0.0, z=0.0)
        if pose.euler_angles is None:
            pose.euler_angles = Point3D(x=0.0, y=0.0, z=pose.heading or 0.0)
        if localization.measurement_time is None and localization.header is not None:
            localization.measurement_time = localization.header.timestamp_sec
        return localization

    def _init_frame(
        self,
        sequence_num: int,
        local_view: LocalView,
        planning_start_point: TrajectoryPoint,
        vehicle_state: VehicleState,
        planning_context: PlanningContext,
    ) -> Tuple[Status, Frame]:
        frame = Frame(
            sequence_num,
            local_view,
            planning_start_point,
            vehicle_state,
            self._reference_line_provider,
            planning_context=planning_context,
        )
        reference_lines, segments = self._reference_line_provider.GetReferenceLines()
        if not reference_lines or len(reference_lines) != len(segments):
            return Status(ErrorCode.PLANNING_ERROR, "Failed to create reference line"), frame

        forward_limit = PncMap.LookForwardDistance(vehicle_state.linear_velocity or 0.0)
        xy = Vec2d(vehicle_state.x, vehicle_state.y)
        for ref_line in reference_lines:
            if not ref_line.Segment(xy, config_module.FLAGS_look_backward_distance, forward_limit):
                return Status(ErrorCode.PLANNING_ERROR, "Fail to shrink reference line."), frame
        for segment in segments:
            if not segment.Shrink(xy, config_module.FLAGS_look_backward_distance, forward_limit):
                return Status(ErrorCode.PLANNING_ERROR, "Fail to shrink routing segments."), frame

        vehicle_state_provider = self._vehicle_state_provider
        if vehicle_state_provider.vehicle_state is None:
            vehicle_state_provider.Update(vehicle_state)
        ego_info = EgoInfo.FromVehicleState(vehicle_state)
        status = frame.Init(
            vehicle_state_provider,
            reference_lines,
            segments,
            self._reference_line_provider.FutureRouteWaypoints(),
            ego_info,
        )
        return status, frame

    def _fill_planning_output(
        self,
        frame: Frame,
        stitching_trajectory: List[TrajectoryPoint],
        adc_trajectory: ADCTrajectory,
        timestamp: float,
        local_view: LocalView,
        planning_context: PlanningContext,
    ) -> Status:
        best_ref_info = frame.FindDriveReferenceLineInfo()
        if best_ref_info is None:
            return Status(ErrorCode.PLANNING_ERROR, "planner failed to make a driving plan")

        current_frame_planned_path = DiscretizedPath()
        for trajectory_point in stitching_trajectory:
            current_frame_planned_path.append(trajectory_point.path_point)
        best_path = best_ref_info.path_data
        if best_path is not None and best_path.discretized_path:
            for path_point in best_path.discretized_path[1:]:
                current_frame_planned_path.append(path_point)
        frame.set_current_frame_planned_path(current_frame_planned_path)

        adc_trajectory.trajectory_type = best_ref_info.trajectory_type
        adc_trajectory.right_of_way_status = best_ref_info.GetRightOfWayStatus()
        adc_trajectory.lane_id = list(best_ref_info.TargetLaneId())
        target_ref_info = frame.FindTargetReferenceLineInfo()
        if target_ref_info is not None:
            adc_trajectory.target_lane_id = list(target_ref_info.TargetLaneId())

        decision_result, _ = best_ref_info.ExportDecision(planning_context)
        adc_trajectory.decision = decision_result

        publishable = PublishableTrajectory(timestamp, best_ref_info.trajectory)
        if len(stitching_trajectory) > 1:
            publishable.PrependTrajectoryPoints(stitching_trajectory[:-1])
        publishable.PopulateTrajectoryProtobuf(adc_trajectory)
        self._last_publishable_trajectory = publishable
        return Status.OK()
