"""
Closed-loop lattice / on-lane simulation.

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
from lattice_planner import LatticePlanner
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.path_point import PathPoint
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
    role: str,
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float,
    backup: Optional[bool],
    blocking_actor_id: Optional[str],
    plan: str,
    path_label: str = "regular/left/forward",
    s_curve: bool = True,
    borrow_direction: Optional[List[int]] = None,
) -> SimResult:
    from scripts.official_map_road import open_official_drive

    drive = open_official_drive(role, ego, actors, horizon_s=horizon_s)
    saved_flags = _set_on_lane_flags(borrow_direction) if plan == "on_lane" else None
    try:
        if plan == "lattice":
            plan_fn = _lattice_plan(drive, blocking_actor_id)
        elif plan == "path_bounds":
            plan_fn = _path_bounds_plan(drive, blocking_actor_id, path_label, s_curve)
        elif plan == "on_lane":
            plan_fn = _on_lane_plan(drive, borrow_direction)
        else:
            raise ValueError(plan)
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
        if saved_flags is not None:
            _restore_on_lane_flags(saved_flags)
        drive.close()
    return drive.annotate(result)


def _lattice_plan(drive, blocking_actor_id: Optional[str]):
    from scripts.lattice_commonroad import _reference_line, plan_on_centerline

    reference_line = _reference_line(
        drive.frame.center, drive.frame.left_width, drive.frame.right_width
    )

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        return plan_on_centerline(
            drive.frame.center,
            ego_now,
            actors_now,
            blocking_actor_id,
            reference_line=reference_line,
        )

    return plan_fn


def _path_bounds_plan(drive, blocking_actor_id: Optional[str], path_label: str, s_curve: bool):
    from scripts.planner_test_fixtures import (
        apply_borrow_path_trajectory,
        apply_path_bounds_overtake_trajectory,
    )

    reference_line = drive.frame.reference_line
    lanes = drive.frame.lanes

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        obstacles = [_obstacle_from_actor(actor) for actor in actors_now]
        reference_line_info, start = _reference_line_info_at(reference_line, lanes, ego_now)
        _bind_obstacles(reference_line_info, obstacles, blocking_actor_id)
        obstacle_x = actors_now[0].x if actors_now else ego_now.x + 30.0
        if s_curve:
            ok, _detail = apply_path_bounds_overtake_trajectory(
                reference_line_info,
                start,
                path_label,
                obstacle_x=obstacle_x,
            )
        else:
            ok, _detail = apply_borrow_path_trajectory(
                reference_line_info,
                start,
                path_label,
            )
        return ok, reference_line_info.trajectory, _trajectory_type_name(reference_line_info), obstacles

    return plan_fn


def _set_on_lane_flags(borrow_direction: Optional[List[int]]):
    import config as config_module

    saved = (
        config_module.FLAGS_enable_reference_line_provider_thread,
        config_module.FLAGS_enable_path_bounds_decider,
        config_module.FLAGS_enable_on_lane_combine_path_and_speed,
        config_module.FLAGS_enable_path_decider_after_lateral,
    )
    config_module.FLAGS_enable_reference_line_provider_thread = True
    config_module.FLAGS_enable_path_bounds_decider = bool(borrow_direction)
    config_module.FLAGS_enable_on_lane_combine_path_and_speed = bool(borrow_direction)
    config_module.FLAGS_enable_path_decider_after_lateral = bool(borrow_direction)
    return saved


def _restore_on_lane_flags(saved) -> None:
    import config as config_module

    (
        config_module.FLAGS_enable_reference_line_provider_thread,
        config_module.FLAGS_enable_path_bounds_decider,
        config_module.FLAGS_enable_on_lane_combine_path_and_speed,
        config_module.FLAGS_enable_path_decider_after_lateral,
    ) = saved


def _on_lane_plan(drive, borrow_direction: Optional[List[int]]):
    import config as config_module
    from copy import deepcopy

    import on_lane_planning as on_lane_module
    from common.planning_context import PlanningContext
    from on_lane_planning import OnLanePlanning
    from reference_line.reference_line_provider import ReferenceLineProvider

    context = PlanningContext()
    provider = ReferenceLineProvider()
    planner = OnLanePlanning(provider)
    cycle_dt = 1.0 / max(config_module.FLAGS_planning_loop_rate, 1e-3)
    sim_time = 0.0
    original_time = on_lane_module.time.time
    if borrow_direction:
        context.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
        context.planning_status.path_decider.decided_side_pass_direction = list(borrow_direction)
    reference_line = drive.frame.reference_line
    lanes = drive.frame.lanes

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        nonlocal sim_time
        # Segment() shrinks the provider line, so each cycle starts from the
        # loaded lanelet. The planner object stays so the stitcher can see
        # the previous trajectory.
        provider._reference_lines = [deepcopy(reference_line)]
        provider._route_segments = [deepcopy(lanes)]
        local_view = _local_view(ego_now, actors_now, measurement_time=sim_time)
        adc = ADCTrajectory()
        on_lane_module.time.time = lambda: sim_time
        try:
            status = planner.RunOnce(local_view, adc, planning_context=context)
        finally:
            on_lane_module.time.time = original_time
            sim_time += cycle_dt
        obstacles = [_obstacle_from_actor(actor) for actor in actors_now]
        trajectory = (
            DiscretizedTrajectory(list(adc.trajectory_point))
            if adc.trajectory_point and len(adc.trajectory_point) >= 2
            else None
        )
        type_name = adc.trajectory_type.name if adc.trajectory_type is not None else "UNKNOWN"
        return status.ok() and trajectory is not None, trajectory, type_name, obstacles

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
        "through",
        name,
        ego,
        actors,
        horizon_s=horizon_s,
        backup=backup,
        blocking_actor_id=blocking_actor_id,
        plan="lattice",
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


def run_path_bounds_road(
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float = 5.0,
    road_length: float = 160.0,
    path_label: str = "regular/left/forward",
    blocking_actor_id: Optional[str] = None,
    s_curve: bool = True,
) -> SimResult:
    """Replan a borrow on the converted map's same-direction neighbor lane.

    ``road_length`` is unused. The parallel lanelet in the converted map sets
    the corridor.
    """
    del road_length
    role = "borrow-left" if "left" in path_label else "borrow-right"
    return _drive_official(
        role,
        name,
        ego,
        actors,
        horizon_s=horizon_s,
        backup=None,
        blocking_actor_id=blocking_actor_id,
        plan="path_bounds",
        path_label=path_label,
        s_curve=s_curve,
    )


def run_on_lane_road(
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float = 3.0,
    road_length: float = 200.0,
    borrow_direction: Optional[List[int]] = None,
    blocking_actor_id: Optional[str] = None,
) -> SimResult:
    """Call OnLanePlanning.RunOnce on the converted lanelet every cycle.

    ``road_length`` is unused. A borrow uses the same-direction neighbor that
    is already in the converted map.
    """
    del road_length
    if not borrow_direction:
        role = "through"
    elif 1 in borrow_direction:
        role = "borrow-left"
    else:
        role = "borrow-right"
    return _drive_official(
        role,
        name,
        ego,
        actors,
        horizon_s=horizon_s,
        backup=None,
        blocking_actor_id=blocking_actor_id,
        plan="on_lane",
        borrow_direction=borrow_direction,
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


def _local_view(ego, actors, measurement_time: float = 0.0):
    from common.frame import LocalView
    from protoclass.adc_trajectory import Point3D
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.point_enu import PointENU
    from protoclass.pose import Pose
    from protoclass.prediction_obstacles import PredictionObstacle, PredictionObstacles

    predictions = []
    for actor in actors:
        obstacle = _obstacle_from_actor(actor)
        trajectories = []
        predicted = obstacle.Trajectory()
        if predicted is not None and predicted.trajectory_point:
            trajectories.append(predicted)
        predictions.append(
            PredictionObstacle(
                perception_obstacle=obstacle.Perception(),
                trajectory=trajectories,
                is_static=obstacle.IsStatic(),
            )
        )
    return LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(
                position=PointENU(x=ego.x, y=ego.y, z=0.0),
                heading=ego.theta,
                # Vehicle frame: y is forward, z is yaw rate.
                linear_acceleration=Point3D(x=0.0, y=ego.a, z=0.0),
                angular_velocity=Point3D(x=0.0, y=0.0, z=ego.v * ego.kappa),
            ),
            measurement_time=measurement_time,
        ),
        chassis=Chassis(
            speed_mps=max(0.0, ego.v),
            header=Header(timestamp_sec=measurement_time),
            driving_mode=Chassis.DrivingMode.COMPLETE_AUTO_DRIVE,
        ),
        prediction_obstacles=PredictionObstacles(prediction_obstacle=predictions),
    )


def _reference_line_info_at(reference_line, lanes, ego_now: Actor):
    """Place a fresh ReferenceLineInfo on a cached road, and the frame that owns it.

    The road geometry is shared, not copied. A sampled curve's map path is
    expensive to deepcopy, and the lattice plan only reads it.
    """
    from common.frame import Frame
    from protoclass.vehicle_state import VehicleState
    from reference_line.reference_line_info import ReferenceLineInfo

    start = TrajectoryPoint(
        path_point=PathPoint(
            x=ego_now.x,
            y=ego_now.y,
            z=0.0,
            theta=ego_now.theta,
            kappa=ego_now.kappa,
            s=0.0,
            dkappa=0.0,
            ddkappa=0.0,
        ),
        v=max(0.0, ego_now.v),
        a=ego_now.a,
        relative_time=0.0,
    )
    vehicle_state = VehicleState(x=ego_now.x, y=ego_now.y, heading=ego_now.theta)
    reference_line_info = ReferenceLineInfo(
        vehicle_state, start, reference_line, lanes, share_map=True
    )
    frame = Frame(0)
    frame._reference_line_info = [reference_line_info]
    reference_line_info._frame = frame
    return reference_line_info, reference_line_info._adc_planning_point


def _bind_obstacles(reference_line_info, obstacles, blocking_actor_id: Optional[str]) -> None:
    reference_line_info.Init(obstacles, 10.0)
    if blocking_actor_id is not None:
        reference_line_info.SetBlockingObstacle(blocking_actor_id)
    frame = reference_line_info._frame
    frame._obstacles = {obstacle.Id(): obstacle for obstacle in obstacles}


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


def _trajectory_type_name(reference_line_info) -> str:
    traj_type = getattr(reference_line_info, "trajectory_type", None)
    if traj_type is None:
        return "UNKNOWN"
    return traj_type.name


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
    if actor.is_static or actor.replay:
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
