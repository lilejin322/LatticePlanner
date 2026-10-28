"""Run the lattice core on a CommonRoad lanelet.

Polynomial bundles, the evaluator, the combiner, and the constraint checks stay
in the lattice modules. This file only turns the lanelet and the current actor
poses into the centerline and obstacles those modules already consume.
"""

from __future__ import annotations

from typing import Optional, Sequence

import config
from lanelet_path import LaneletPath, path_from_lanelet
from lattice_planner import LatticePlanner
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint


def plan_on_lanelet(
    lanelet,
    ego,
    actors: Sequence,
    blocking_actor_id: Optional[str] = None,
    *,
    path: Optional[LaneletPath] = None,
    obstacles: Optional[Sequence] = None,
    speed_limit: Optional[float] = None,
):
    """Plan one lattice cycle on ``lanelet``.

    Pass the same ``path`` every cycle so the centerline is built once. Pass
    ``obstacles`` when the caller already built them from recorded states.
    ``blocking_actor_id`` is accepted for callers that still name a leader;
    the ST graph already stops for vehicles on the lanelet. Returns
    ``(ok, trajectory, type_name, obstacles)``.
    """
    del blocking_actor_id
    from scripts.closed_loop_sim import _obstacle_from_actor

    if path is None:
        path = path_from_lanelet(
            lanelet, config.FLAGS_default_cruise_speed if speed_limit is None else speed_limit
        )
    if obstacles is None:
        obstacles = [_obstacle_from_actor(actor) for actor in actors]
    else:
        obstacles = list(obstacles)
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
    ok, trajectory, traj_type = LatticePlanner().PlanOnLanelet(
        start, obstacles, path.points, path.lane_width, path.speed_at(0.0)
    )
    type_name = traj_type.name if traj_type is not None else "UNKNOWN"
    return ok, trajectory, type_name, obstacles
