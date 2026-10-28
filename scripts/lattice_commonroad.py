"""Run the existing lattice core on a CommonRoad centerline.

Polynomial bundles, the evaluator, the combiner, and the constraint checks stay
in the lattice modules. This file only turns a lanelet centerline and the
current actor poses into the reference line those modules already consume.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

import config
from common.frame import Frame
from common.map_path_point import MapPathPoint
from common.route_segments import RouteSegments
from common.vec2d import Vec2d
from lattice_planner import LatticePlanner
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState
from reference_line import ReferenceLine
from reference_line.reference_line_info import ReferenceLineInfo
from reference_line.reference_point import ReferencePoint


def plan_on_centerline(
    center_xy,
    ego,
    actors: Sequence,
    blocking_actor_id: Optional[str] = None,
    *,
    left_width: float = 1.75,
    right_width: float = 1.75,
    reference_line: Optional[ReferenceLine] = None,
):
    """Plan one lattice cycle on ``center_xy``.

    ``ego`` and ``actors`` are already in the planner frame. Pass the same
    ``reference_line`` every cycle so the centerline is built once. Returns
    ``(ok, trajectory, type_name, obstacles)``.
    """
    from scripts.closed_loop_sim import _bind_obstacles, _obstacle_from_actor

    if reference_line is None:
        reference_line = _reference_line(center_xy, left_width, right_width)
    start = TrajectoryPoint(
        path_point=PathPoint(
            x=ego.x,
            y=ego.y,
            z=0.0,
            theta=ego.theta,
            kappa=ego.kappa,
            s=0.0,
            dkappa=0.0,
            ddkappa=0.0,
        ),
        v=max(0.0, ego.v),
        a=ego.a,
        relative_time=0.0,
    )
    vehicle_state = VehicleState(x=ego.x, y=ego.y, heading=ego.theta)
    reference_line_info = ReferenceLineInfo(
        vehicle_state, start, reference_line, RouteSegments(), share_map=True
    )
    frame = Frame(0)
    frame._reference_line_info = [reference_line_info]
    reference_line_info._frame = frame
    obstacles = [_obstacle_from_actor(actor) for actor in actors]
    _bind_obstacles(reference_line_info, obstacles, blocking_actor_id)
    ok = LatticePlanner().PlanOnReferenceLine(
        reference_line_info._adc_planning_point, frame, reference_line_info
    )
    trajectory = reference_line_info.trajectory
    traj_type = getattr(reference_line_info, "trajectory_type", None)
    type_name = traj_type.name if traj_type is not None else "UNKNOWN"
    return ok, trajectory, type_name, obstacles


def _reference_line(center_xy, left_width: float, right_width: float) -> ReferenceLine:
    import logging

    points = _reference_points(center_xy)
    # The centerline has no Apollo lane waypoints. Width is applied just below.
    disabled = logging.root.manager.disable
    logging.disable(logging.WARNING)
    try:
        reference_line = ReferenceLine(points)
    finally:
        logging.disable(disabled)
    reference_line.AddSpeedLimit(0.0, reference_line.Length(), config.FLAGS_default_cruise_speed)
    _set_width(reference_line, left_width, right_width)
    return reference_line


def _reference_points(center_xy) -> List[ReferencePoint]:
    center = [(float(point[0]), float(point[1])) for point in center_xy]
    if len(center) < 2:
        raise ValueError("centerline needs at least two points")
    headings = []
    for index, (x, y) in enumerate(center):
        if index + 1 < len(center):
            dx = center[index + 1][0] - x
            dy = center[index + 1][1] - y
        else:
            dx = x - center[index - 1][0]
            dy = y - center[index - 1][1]
        headings.append(math.atan2(dy, dx))
    points = []
    for index, (x, y) in enumerate(center):
        if index + 1 < len(center):
            step = math.hypot(center[index + 1][0] - x, center[index + 1][1] - y)
            turn = _angle_delta(headings[index + 1], headings[index])
            kappa = turn / step if step > 1e-6 else 0.0
        else:
            kappa = 0.0
        points.append(
            ReferencePoint(MapPathPoint(Vec2d(x, y), headings[index], []), kappa, 0.0)
        )
    return points


def _set_width(reference_line: ReferenceLine, left_width: float, right_width: float) -> None:
    path = reference_line.map_path
    count = len(path._lane_left_width)
    path._lane_left_width = [left_width] * count
    path._lane_right_width = [right_width] * count
    path._road_left_width = [left_width] * count
    path._road_right_width = [right_width] * count


def _angle_delta(left: float, right: float) -> float:
    return (left - right + math.pi) % (2.0 * math.pi) - math.pi
