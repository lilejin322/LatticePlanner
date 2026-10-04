"""
Closed-loop lattice simulation.

The scenario suite (``lattice_scenarios.py``) is open-loop: each case calls
``LatticePlanner.Plan`` once, or skips the planner and plays a hand-drawn
S-curve (``skip_lattice``). Animation then walks that single trajectory. Ego
never feeds the executed state back into the next planning cycle, and a
moving obstacle is not stepped on the same clock as the planner.

This module runs the planner the way a simulator would:

1. Rebuild the frame from the current ego state and actor poses.
2. Give each moving actor a constant-velocity prediction at the planner's
   own time resolution (``FLAGS_trajectory_time_resolution``).
3. Call ``LatticePlanner.Plan``.
4. Execute only the next planning cycle (``1 / FLAGS_planning_loop_rate``).
5. Advance actors by that same dt and repeat.

Collision is checked on the executed prefix with
``CollisionChecker.StaticInCollision``, using each obstacle's prediction at
the trajectory point's relative time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import config
from behavior.collision_checker import CollisionChecker
from common.discretized_trajectory import DiscretizedTrajectory
from lattice_planner import LatticePlanner
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint


@dataclass
class Actor:
    """One vehicle in the simulated world. ``actor_id`` is the planning id."""

    actor_id: str
    x: float
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    a: float = 0.0
    kappa: float = 0.0
    length: float = 4.0
    width: float = 2.0
    is_static: bool = False


@dataclass
class CycleRecord:
    t: float
    plan_ok: bool
    trajectory_type: str
    ego_x: float
    ego_y: float
    ego_v: float
    min_clearance: Optional[float]
    collided: bool


@dataclass
class SimResult:
    name: str
    ok: bool
    reason: str
    horizon_s: float
    cycles: List[CycleRecord] = field(default_factory=list)
    collided: bool = False
    plan_failures: int = 0
    fallback_cycles: int = 0
    min_clearance: Optional[float] = None
    ego_x0: float = 0.0
    ego_x1: float = 0.0
    ego_v1: float = 0.0

    def summary(self) -> str:
        clearance = (
            f"{self.min_clearance:.2f}m" if self.min_clearance is not None else "n/a"
        )
        return (
            f"{self.reason} | cycles={len(self.cycles)} "
            f"fallback={self.fallback_cycles} plan_failures={self.plan_failures} "
            f"collided={self.collided} min_clearance={clearance} "
            f"x {self.ego_x0:.1f}->{self.ego_x1:.1f} v_end={self.ego_v1:.2f}"
        )


def run_straight_road(
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float = 3.0,
    road_length: float = 200.0,
    backup: Optional[bool] = None,
    blocking_actor_id: Optional[str] = None,
) -> SimResult:
    """Replan along a straight reference line until ``horizon_s``."""
    from scripts.planner_test_fixtures import build_lattice_plan_frame

    cycle_dt = 1.0 / max(config.FLAGS_planning_loop_rate, 1e-3)
    sample_dt = config.FLAGS_trajectory_time_resolution
    ego_x0 = ego.x
    records: List[CycleRecord] = []
    collided = False
    plan_failures = 0
    fallback_cycles = 0
    min_clearance: Optional[float] = None
    reason = "reached horizon"
    t = 0.0
    old_backup = config.FLAGS_enable_backup_trajectory
    if backup is not None:
        config.FLAGS_enable_backup_trajectory = backup
    try:
        while t < horizon_s - 1e-9:
            obstacles = [_obstacle_from_actor(actor) for actor in actors]
            frame, reference_line_info, start = build_lattice_plan_frame(
                obstacles,
                length=road_length,
                init_v=max(0.0, ego.v),
                start_x=ego.x,
                start_y=ego.y,
                start_heading=ego.theta,
                blocking_obstacle_id=blocking_actor_id,
            )
            start.a = ego.a
            start.path_point.kappa = ego.kappa
            plan_ok = LatticePlanner().Plan(start, frame, ADCTrajectory())
            trajectory = reference_line_info.trajectory
            if not plan_ok or trajectory is None or len(trajectory) < 2:
                plan_failures += 1
                reason = f"planning failed at t={t:.2f}"
                records.append(
                    CycleRecord(
                        t=t,
                        plan_ok=False,
                        trajectory_type="NONE",
                        ego_x=ego.x,
                        ego_y=ego.y,
                        ego_v=ego.v,
                        min_clearance=None,
                        collided=False,
                    )
                )
                break

            traj_type = reference_line_info.trajectory_type
            type_name = traj_type.name if traj_type is not None else "UNKNOWN"
            if type_name == "PATH_FALLBACK":
                fallback_cycles += 1

            executed = _executed_prefix(trajectory, cycle_dt, sample_dt)
            cycle_clearance = _min_clearance(obstacles, executed)
            cycle_collision = CollisionChecker.StaticInCollision(
                obstacles,
                executed,
                config.EGO_VEHICLE_LENGTH,
                config.EGO_VEHICLE_WIDTH,
                config.EGO_BACK_EDGE_TO_CENTER,
            )
            if cycle_clearance is not None:
                min_clearance = (
                    cycle_clearance
                    if min_clearance is None
                    else min(min_clearance, cycle_clearance)
                )
            executed_point = trajectory.Evaluate(min(cycle_dt, trajectory.GetTemporalLength()))
            _apply_executed_state(ego, executed_point)
            for actor in actors:
                _advance_actor(actor, cycle_dt)
            t += cycle_dt
            records.append(
                CycleRecord(
                    t=t,
                    plan_ok=True,
                    trajectory_type=type_name,
                    ego_x=ego.x,
                    ego_y=ego.y,
                    ego_v=ego.v,
                    min_clearance=cycle_clearance,
                    collided=cycle_collision,
                )
            )
            if cycle_collision:
                collided = True
                reason = f"collision at t={t:.2f}"
                break
    finally:
        config.FLAGS_enable_backup_trajectory = old_backup

    ok = (not collided) and plan_failures == 0 and t >= horizon_s - 1e-6
    if ok:
        reason = "reached horizon without collision"
    return SimResult(
        name=name,
        ok=ok,
        reason=reason,
        horizon_s=horizon_s,
        cycles=records,
        collided=collided,
        plan_failures=plan_failures,
        fallback_cycles=fallback_cycles,
        min_clearance=min_clearance,
        ego_x0=ego_x0,
        ego_x1=ego.x,
        ego_v1=ego.v,
    )


def _executed_prefix(trajectory, horizon: float, sample_dt: float) -> DiscretizedTrajectory:
    limit = min(horizon, trajectory.GetTemporalLength())
    points = []
    t = 0.0
    while t <= limit + 1e-9:
        points.append(trajectory.Evaluate(t))
        t += sample_dt
    if len(points) < 2:
        points.append(trajectory.Evaluate(limit))
    return DiscretizedTrajectory(points)


def _min_clearance(obstacles, executed: DiscretizedTrajectory) -> Optional[float]:
    from common.box2d import Box2d
    from common.vec2d import Vec2d

    if executed.NumOfPoints() == 0 or not obstacles:
        return None
    best: Optional[float] = None
    for i in range(executed.NumOfPoints()):
        ego_point = executed.TrajectoryPointAt(i)
        theta = ego_point.path_point.theta or 0.0
        ego_box = Box2d(
            Vec2d(ego_point.path_point.x, ego_point.path_point.y),
            theta,
            config.EGO_VEHICLE_LENGTH,
            config.EGO_VEHICLE_WIDTH,
        )
        shift = config.EGO_VEHICLE_LENGTH / 2.0 - config.EGO_BACK_EDGE_TO_CENTER
        ego_box.Shift(Vec2d(shift * math.cos(theta), shift * math.sin(theta)))
        relative_time = ego_point.relative_time or 0.0
        for obstacle in obstacles:
            other = obstacle.GetBoundingBox(obstacle.GetPointAtTime(relative_time))
            distance = float(ego_box.DistanceTo(other))
            best = distance if best is None else min(best, distance)
    return best


def _apply_executed_state(ego: Actor, point: TrajectoryPoint) -> None:
    path_point = point.path_point
    ego.x = path_point.x or 0.0
    ego.y = path_point.y or 0.0
    ego.theta = path_point.theta or 0.0
    ego.kappa = path_point.kappa or 0.0
    ego.v = point.v or 0.0
    ego.a = point.a or 0.0


def _advance_actor(actor: Actor, dt: float) -> None:
    if actor.is_static:
        return
    actor.x += actor.v * math.cos(actor.theta) * dt
    actor.y += actor.v * math.sin(actor.theta) * dt


def _obstacle_from_actor(actor: Actor):
    from common.obstacle import Obstacle
    from protoclass.adc_trajectory import Point3D
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType
    from protoclass.trajectory import Trajectory

    speed = 0.0 if actor.is_static else actor.v
    perception = PerceptionObstacle(
        id=_perception_id(actor.actor_id),
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=actor.x, y=actor.y, z=0.0),
        velocity=Point3D(
            x=speed * math.cos(actor.theta),
            y=speed * math.sin(actor.theta),
            z=0.0,
        ),
        length=actor.length,
        width=actor.width,
        height=1.5,
        theta=actor.theta,
    )
    if actor.is_static or abs(speed) < 1e-3:
        return Obstacle(actor.actor_id, perception, is_static=True)

    dt = config.FLAGS_trajectory_time_resolution
    horizon = config.FLAGS_trajectory_time_length
    points = []
    t = 0.0
    while t <= horizon + 1e-9:
        points.append(
            TrajectoryPoint(
                path_point=PathPoint(
                    x=actor.x + speed * math.cos(actor.theta) * t,
                    y=actor.y + speed * math.sin(actor.theta) * t,
                    z=0.0,
                    theta=actor.theta,
                    kappa=0.0,
                    s=0.0,
                    dkappa=0.0,
                    ddkappa=0.0,
                ),
                v=speed,
                a=0.0,
                relative_time=t,
            )
        )
        t += dt
    return Obstacle(
        actor.actor_id,
        perception,
        is_static=False,
        trajectory=Trajectory(trajectory_point=points),
    )


def _perception_id(actor_id: str) -> int:
    digits = "".join(ch for ch in actor_id if ch.isdigit())
    if digits:
        return int(digits[-6:])
    return 1
