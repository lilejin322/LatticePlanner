"""Trajectory stitcher aligned with trajectory_stitcher.cc."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import List, Optional, Tuple

from common.discretized_trajectory import DiscretizedTrajectory
from common.publishable_trajectory import PublishableTrajectory
from common.vec2d import Vec2d
from common.vehicle_model import VehicleModel
import config as config_module
from protoclass.chassis import Chassis
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState


class TrajectoryStitcher:
    @staticmethod
    def compute_trajectory_point_from_vehicle_state(
        planning_cycle_time: float, vehicle_state: VehicleState
    ) -> TrajectoryPoint:
        point = TrajectoryPoint(
            path_point=PathPoint(
                x=vehicle_state.x,
                y=vehicle_state.y,
                z=getattr(vehicle_state, "z", 0.0) or 0.0,
                theta=vehicle_state.heading,
                kappa=getattr(vehicle_state, "kappa", 0.0) or 0.0,
                s=0.0,
            ),
            v=vehicle_state.linear_velocity or 0.0,
            a=vehicle_state.linear_acceleration or 0.0,
            relative_time=planning_cycle_time,
        )
        return point

    @staticmethod
    def compute_reinit_stitching_trajectory(
        planning_cycle_time: float, vehicle_state: VehicleState
    ) -> List[TrajectoryPoint]:
        k_epsilon_v = 0.1
        k_epsilon_a = 0.4
        if abs(vehicle_state.linear_velocity or 0.0) < k_epsilon_v and abs(
            vehicle_state.linear_acceleration or 0.0
        ) < k_epsilon_a:
            reinit_state = vehicle_state
        else:
            reinit_state = VehicleModel.Predict(planning_cycle_time, vehicle_state)
        return [
            TrajectoryStitcher.compute_trajectory_point_from_vehicle_state(
                planning_cycle_time, reinit_state
            )
        ]

    @staticmethod
    def compute_position_projection(x: float, y: float, point: TrajectoryPoint) -> Tuple[float, float]:
        pp = point.path_point
        vx = x - pp.x
        vy = y - pp.y
        nx = math.cos(pp.theta)
        ny = math.sin(pp.theta)
        lon = vx * nx + vy * ny + pp.s
        lat = vx * ny - vy * nx
        return lon, lat

    @staticmethod
    def compute_stitching_trajectory(
        vehicle_state: VehicleState,
        current_timestamp: float,
        planning_cycle_time: float,
        preserved_points_num: int,
        replan_by_offset: bool,
        prev_trajectory: Optional[PublishableTrajectory],
        replan_reason: Optional[List[str]] = None,
    ) -> List[TrajectoryPoint]:
        def set_reason(msg: str) -> List[TrajectoryPoint]:
            if replan_reason is not None:
                replan_reason.clear()
                replan_reason.append(msg)
            return TrajectoryStitcher.compute_reinit_stitching_trajectory(
                planning_cycle_time, vehicle_state
            )

        if not config_module.FLAGS_enable_trajectory_stitcher:
            return set_reason("stitch is disabled by gflag.")
        if prev_trajectory is None or prev_trajectory.NumOfPoints() == 0:
            return set_reason("replan for no previous trajectory.")

        driving_mode = getattr(vehicle_state, "driving_mode", None)
        if driving_mode is not None and driving_mode != Chassis.DrivingMode.COMPLETE_AUTO_DRIVE:
            return set_reason("replan for manual mode.")

        veh_rel_time = current_timestamp - prev_trajectory.header_time
        time_matched_index = prev_trajectory.QueryLowerBoundPoint(veh_rel_time, 1e-6)
        if (
            time_matched_index == 0
            and veh_rel_time < prev_trajectory.StartPoint().relative_time
        ):
            return set_reason(
                "replan for current time smaller than the previous trajectory's first time."
            )
        if time_matched_index + 1 >= prev_trajectory.NumOfPoints():
            return set_reason(
                "replan for current time beyond the previous trajectory's last time."
            )

        time_matched_point = prev_trajectory.TrajectoryPointAt(time_matched_index)
        if time_matched_point.path_point is None:
            return set_reason("replan for previous trajectory missed path point")

        position_matched_index = prev_trajectory.QueryNearestPointWithBuffer(
            Vec2d(vehicle_state.x, vehicle_state.y), 1e-6
        )
        frenet_sd = TrajectoryStitcher.compute_position_projection(
            vehicle_state.x,
            vehicle_state.y,
            prev_trajectory.TrajectoryPointAt(position_matched_index),
        )

        if replan_by_offset:
            lon_diff = time_matched_point.path_point.s - frenet_sd[0]
            lat_diff = frenet_sd[1]
            if abs(lat_diff) > config_module.FLAGS_replan_lateral_distance_threshold:
                return set_reason(
                    f"replan triggered. lat_diff = {lat_diff}"
                )
            if abs(lon_diff) > config_module.FLAGS_replan_longitudinal_distance_threshold:
                return set_reason(
                    f"replan triggered. lon_diff = {lon_diff}"
                )

        forward_rel_time = veh_rel_time + planning_cycle_time
        forward_time_index = prev_trajectory.QueryLowerBoundPoint(forward_rel_time, 1e-6)
        matched_index = min(time_matched_index, position_matched_index)
        start_index = max(0, matched_index - preserved_points_num)
        stitching_trajectory = [
            deepcopy(tp) for tp in prev_trajectory[start_index : forward_time_index + 1]
        ]
        if not stitching_trajectory:
            return set_reason("replan for empty stitching trajectory")

        zero_s = stitching_trajectory[-1].path_point.s
        header_time = prev_trajectory.header_time
        for tp in stitching_trajectory:
            if tp.path_point is None:
                return set_reason("replan for previous trajectory missed path point")
            tp.relative_time = tp.relative_time + header_time - current_timestamp
            tp.path_point.s = tp.path_point.s - zero_s
        if replan_reason is not None:
            replan_reason.clear()
        return stitching_trajectory
