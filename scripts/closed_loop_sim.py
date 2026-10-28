"""
Closed-loop lattice simulation.

The road is a lanelet from the converted Borregas Ave map. See
``scripts/official_map_road.py``. Each cycle plans once from the executed ego
state and commits only the next planning cycle. Other vehicles hold a constant
speed along the centerline; this is not a traffic simulator.

``x`` on an actor is station along that centerline and ``y`` is lateral
offset, positive to the left. ``replay`` means the road driver owns the pose,
so the loop does not integrate the actor a second time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import config
from behavior.collision_checker import CollisionChecker
from common.discretized_trajectory import DiscretizedTrajectory
from protoclass.trajectory_point import TrajectoryPoint


@dataclass
class Actor:
    """One vehicle in the simulated world. ``actor_id`` is the planning id.

    On a closed-loop drive, ``x`` is station along the lanelet and ``y`` is
    lateral offset. Speed is constant. ``replay`` keeps the loop from
    integrating the actor again after the road driver has set its pose.
    """

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
    replay: bool = False


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    theta: float
    v: float


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
    max_abs_y: float = 0.0
    ego_trace: List[PoseSample] = field(default_factory=list)
    actor_traces: Dict[str, List[PoseSample]] = field(default_factory=dict)
    actor_meta: Dict[str, Actor] = field(default_factory=dict)
    blocking_actor_id: Optional[str] = None
    ref_x: List[float] = field(default_factory=list)
    ref_y: List[float] = field(default_factory=list)
    other_ref_x: List[float] = field(default_factory=list)
    other_ref_y: List[float] = field(default_factory=list)
    lanelet_ids: List[int] = field(default_factory=list)
    origin_x: float = 0.0
    origin_y: float = 0.0
    commonroad_scenario: Optional[object] = field(default=None, repr=False)

    def summary(self) -> str:
        clearance = (
            f"{self.min_clearance:.2f}m" if self.min_clearance is not None else "n/a"
        )
        return (
            f"{self.reason} | cycles={len(self.cycles)} "
            f"fallback={self.fallback_cycles} plan_failures={self.plan_failures} "
            f"collided={self.collided} min_clearance={clearance} "
            f"x {self.ego_x0:.1f}->{self.ego_x1:.1f} v_end={self.ego_v1:.2f} "
            f"max|y|={self.max_abs_y:.2f}"
        )


PlanFn = Callable[[Actor, List[Actor]], tuple]
SyncFn = Callable[[float], None]
CollisionFn = Callable[[list, DiscretizedTrajectory, float], tuple]


def _drive_official(
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float,
    backup: Optional[bool],
    blocking_actor_id: Optional[str],
) -> SimResult:
    from scripts.official_map_road import open_official_drive

    drive = open_official_drive("through", ego, actors, horizon_s=horizon_s)
    try:
        plan_fn = _lattice_plan(drive, blocking_actor_id)
        result = _run_loop(
            name,
            ego,
            actors,
            plan_fn,
            horizon_s=horizon_s,
            backup=backup,
            blocking_actor_id=blocking_actor_id,
            sync_fn=drive.sync_fn,
            collision_fn=drive.collision_fn,
        )
    finally:
        drive.close()
    return drive.annotate(result)


def _lattice_plan(drive, blocking_actor_id: Optional[str]):
    from lanelet_path import path_from_lanelet
    from scripts.lattice_commonroad import plan_on_lanelet

    path = path_from_lanelet(drive.frame.lanelet, config.FLAGS_default_cruise_speed)

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        return plan_on_lanelet(
            drive.frame.lanelet,
            ego_now,
            actors_now,
            blocking_actor_id,
            path=path,
        )

    return plan_fn


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
    """Replan on the converted map's long lanelet until ``horizon_s``.

    ``ego.x`` and ``actor.x`` are station along that lanelet. ``y`` is the
    lateral offset. ``road_length`` is unused; the lanelet sets the length.
    """
    del road_length
    return _drive_official(
        name,
        ego,
        actors,
        horizon_s=horizon_s,
        backup=backup,
        blocking_actor_id=blocking_actor_id,
    )


def run_curved_road(
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float = 3.0,
    radius: float = 60.0,
    arc_length: float = 120.0,
    backup: Optional[bool] = None,
    blocking_actor_id: Optional[str] = None,
) -> SimResult:
    """Same converted lanelet as ``run_straight_road``.

    Older cases still call this name. ``radius`` and ``arc_length`` are ignored.
    """
    del radius, arc_length
    return run_straight_road(
        name,
        ego,
        actors,
        horizon_s=horizon_s,
        backup=backup,
        blocking_actor_id=blocking_actor_id,
    )


def to_plot_context(result: SimResult, scenario_name: str, title: str):
    """Build a drawable scene from the executed trace."""
    from scripts.lattice_visualization import ObstacleDraw, PlotContext

    ctx = PlotContext(
        scenario_name=scenario_name,
        title=title,
        passed=result.ok,
        note=result.summary(),
        traj_type="CLOSED_LOOP",
        ref_x=list(result.ref_x),
        ref_y=list(result.ref_y),
        other_lane_x=list(result.other_ref_x),
        other_lane_y=list(result.other_ref_y),
    )
    for sample in result.ego_trace:
        ctx.traj_t.append(sample.t)
        ctx.traj_x.append(sample.x)
        ctx.traj_y.append(sample.y)
        ctx.traj_v.append(sample.v)
        ctx.traj_a.append(0.0)
        ctx.traj_theta.append(sample.theta)
    if result.ego_trace:
        ctx.ego_x = result.ego_trace[0].x
        ctx.ego_y = result.ego_trace[0].y
    for actor_id, samples in result.actor_traces.items():
        if not samples:
            continue
        meta = result.actor_meta.get(actor_id)
        length = meta.length if meta is not None else 4.0
        width = meta.width if meta is not None else 2.0
        moved = any(
            abs(sample.x - samples[0].x) > 1e-3 or abs(sample.y - samples[0].y) > 1e-3
            for sample in samples
        )
        xs, ys = _rect_corners(samples[0].x, samples[0].y, samples[0].theta, length, width)
        draw = ObstacleDraw(
            xs=xs,
            ys=ys,
            label=actor_id,
            is_blocking=actor_id == result.blocking_actor_id,
            is_static=not moved,
            length=length,
            width=width,
        )
        if moved:
            for sample in samples:
                draw.traj_t.append(sample.t)
                draw.traj_x.append(sample.x)
                draw.traj_y.append(sample.y)
                draw.traj_theta.append(sample.theta)
        ctx.obstacles.append(draw)
    return ctx


def _run_loop(
    name: str,
    ego: Actor,
    actors: List[Actor],
    plan_fn: PlanFn,
    *,
    horizon_s: float,
    backup: Optional[bool] = None,
    blocking_actor_id: Optional[str] = None,
    sync_fn: Optional[SyncFn] = None,
    collision_fn: Optional[CollisionFn] = None,
) -> SimResult:
    cycle_dt = 1.0 / max(config.FLAGS_planning_loop_rate, 1e-3)
    sample_dt = config.FLAGS_trajectory_time_resolution
    ego_x0 = ego.x
    records: List[CycleRecord] = []
    ego_trace = [_pose(0.0, ego)]
    actor_traces = {actor.actor_id: [_pose(0.0, actor)] for actor in actors}
    actor_meta = {actor.actor_id: actor for actor in actors}
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
            if sync_fn is not None:
                sync_fn(t)
            ok, trajectory, type_name, obstacles = plan_fn(ego, actors)
            trajectory = _as_discretized(trajectory)
            if not ok or trajectory is None or trajectory.NumOfPoints() < 2:
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

            if type_name == "PATH_FALLBACK":
                fallback_cycles += 1

            executed = _executed_prefix(trajectory, cycle_dt, sample_dt)
            if collision_fn is None:
                cycle_clearance = _min_clearance(obstacles, executed)
                cycle_collision = CollisionChecker.StaticInCollision(
                    obstacles,
                    executed,
                    config.EGO_VEHICLE_LENGTH,
                    config.EGO_VEHICLE_WIDTH,
                    config.EGO_BACK_EDGE_TO_CENTER,
                )
            else:
                cycle_clearance, cycle_collision = collision_fn(obstacles, executed, t)
            if cycle_clearance is not None:
                min_clearance = (
                    cycle_clearance
                    if min_clearance is None
                    else min(min_clearance, cycle_clearance)
                )
            executed_point = trajectory.Evaluate(min(cycle_dt, trajectory.GetTemporalLength()))
            if not _executed_state_is_usable(ego, executed_point, cycle_dt):
                plan_failures += 1
                reason = f"executed state left the road at t={t:.2f}"
                records.append(
                    CycleRecord(
                        t=t,
                        plan_ok=False,
                        trajectory_type=type_name,
                        ego_x=ego.x,
                        ego_y=ego.y,
                        ego_v=ego.v,
                        min_clearance=cycle_clearance,
                        collided=False,
                    )
                )
                break
            _apply_executed_state(ego, executed_point)
            for actor in actors:
                _advance_actor(actor, cycle_dt)
            t += cycle_dt
            if sync_fn is not None:
                sync_fn(t)
            ego_trace.append(_pose(t, ego))
            for actor in actors:
                actor_traces[actor.actor_id].append(_pose(t, actor))
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
    max_abs_y = max((abs(sample.y) for sample in ego_trace), default=0.0)
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
        max_abs_y=max_abs_y,
        ego_trace=ego_trace,
        actor_traces=actor_traces,
        actor_meta=actor_meta,
        blocking_actor_id=blocking_actor_id,
    )


def _executed_state_is_usable(ego: Actor, point: TrajectoryPoint, dt: float) -> bool:
    path_point = point.path_point
    if path_point is None:
        return False
    values = (path_point.x, path_point.y, path_point.theta, point.v)
    if any(value is None or value != value or abs(value) > 1.0e5 for value in values):
        return False
    jump = math.hypot((path_point.x or 0.0) - ego.x, (path_point.y or 0.0) - ego.y)
    limit = max(2.0, (abs(ego.v) + 8.0) * max(dt, 0.1) * 5.0)
    return jump <= limit


def _pose(t: float, actor: Actor) -> PoseSample:
    return PoseSample(t=t, x=actor.x, y=actor.y, theta=actor.theta, v=actor.v)


def _rect_corners(x: float, y: float, theta: float, length: float, width: float):
    hl = length / 2.0
    hw = width / 2.0
    local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    c, s = math.cos(theta), math.sin(theta)
    xs = [x + lx * c - ly * s for lx, ly in local]
    ys = [y + lx * s + ly * c for lx, ly in local]
    xs.append(xs[0])
    ys.append(ys[0])
    return xs, ys


def _as_discretized(trajectory):
    if trajectory is None:
        return None
    if isinstance(trajectory, DiscretizedTrajectory):
        return trajectory
    points = list(trajectory)
    if len(points) < 2:
        return None
    return DiscretizedTrajectory(points)


def _executed_prefix(trajectory, horizon: float, sample_dt: float) -> DiscretizedTrajectory:
    limit = min(horizon, max(0.0, trajectory.GetTemporalLength()))
    points = []
    t = 0.0
    while t <= limit + 1e-9:
        points.append(trajectory.Evaluate(t))
        t += sample_dt
    if len(points) < 2:
        points.append(trajectory.Evaluate(limit))
    return DiscretizedTrajectory(points)


def _min_clearance(obstacles, executed: DiscretizedTrajectory) -> Optional[float]:
    from lattice_motion import ego_occupancy, occupancy_at

    if executed.NumOfPoints() == 0 or not obstacles:
        return None
    best: Optional[float] = None
    for i in range(executed.NumOfPoints()):
        ego_point = executed.TrajectoryPointAt(i)
        theta = ego_point.path_point.theta or 0.0
        ego_box = ego_occupancy(
            ego_point.path_point.x,
            ego_point.path_point.y,
            theta,
            config.EGO_VEHICLE_LENGTH,
            config.EGO_VEHICLE_WIDTH,
            config.EGO_BACK_EDGE_TO_CENTER,
        )
        relative_time = ego_point.relative_time or 0.0
        for obstacle in obstacles:
            distance = float(ego_box.shapely_object.distance(occupancy_at(obstacle, relative_time).shapely_object))
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
    if actor.is_static or actor.replay:
        return
    actor.x += actor.v * math.cos(actor.theta) * dt
    actor.y += actor.v * math.sin(actor.theta) * dt


def _obstacle_from_actor(actor: Actor):
    from lattice_motion import obstacle_from_actor

    return obstacle_from_actor(actor)
