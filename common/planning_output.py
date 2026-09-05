"""Planning output helpers aligned with planning_base.cc and on_lane_planning.cc."""

from __future__ import annotations

from protoclass.adc_trajectory import ADCTrajectory, GearPosition
from protoclass.header import Header
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState
import config as config_module


def FillPlanningPb(timestamp: float, trajectory_pb: ADCTrajectory, local_view) -> None:
    if trajectory_pb.header is None:
        trajectory_pb.header = Header()
    trajectory_pb.header.timestamp_sec = timestamp
    prediction = getattr(local_view, "prediction_obstacles", None)
    if prediction is not None and prediction.header is not None:
        trajectory_pb.header.lidar_timestamp = prediction.header.lidar_timestamp
        trajectory_pb.header.camera_timestamp = prediction.header.camera_timestamp
        trajectory_pb.header.radar_timestamp = prediction.header.radar_timestamp
    routing = getattr(local_view, "routing", None)
    if routing is not None:
        trajectory_pb.routing_header = routing.header if routing.header is not None else Header()


def GenerateStopTrajectory(trajectory_pb: ADCTrajectory, vehicle_state: VehicleState) -> None:
    trajectory_pb.trajectory_point = []
    tp = TrajectoryPoint(
        path_point=PathPoint(
            x=vehicle_state.x,
            y=vehicle_state.y,
            theta=vehicle_state.heading,
            s=0.0,
        ),
        v=0.0,
        a=0.0,
        relative_time=0.0,
    )
    t = 0.0
    while t < config_module.FLAGS_fallback_total_time:
        point = TrajectoryPoint(
            path_point=PathPoint(
                x=tp.path_point.x,
                y=tp.path_point.y,
                theta=tp.path_point.theta,
                s=0.0,
            ),
            v=0.0,
            a=0.0,
            relative_time=t,
        )
        trajectory_pb.trajectory_point.append(point)
        t += config_module.FLAGS_fallback_time_unit
    trajectory_pb.total_path_length = 0.0
    trajectory_pb.gear = GearPosition.GEAR_DRIVE
