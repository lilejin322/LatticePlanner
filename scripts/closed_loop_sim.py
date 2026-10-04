"""
Closed-loop lattice / on-lane simulation.

Each cycle rebuilds the world from the executed ego state and the actors'
current poses, calls the planner once, and commits only the next planning
cycle (1 / FLAGS_planning_loop_rate). The stored trace is that executed
history. Animation plays the trace; it does not play one open-loop plan.
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

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        obstacles = [_obstacle_from_actor(actor) for actor in actors_now]
        _frame, reference_line_info, start = build_lattice_plan_frame(
            obstacles,
            length=road_length,
            init_v=max(0.0, ego_now.v),
            start_x=ego_now.x,
            start_y=ego_now.y,
            start_heading=ego_now.theta,
            blocking_obstacle_id=blocking_actor_id,
        )
        start.a = ego_now.a
        start.path_point.kappa = ego_now.kappa
        ok = LatticePlanner().Plan(start, _frame, ADCTrajectory())
        return ok, reference_line_info.trajectory, _trajectory_type_name(reference_line_info), obstacles

    result = _run_loop(
        name,
        ego,
        actors,
        plan_fn,
        horizon_s=horizon_s,
        backup=backup,
        blocking_actor_id=blocking_actor_id,
    )
    result.ref_x = [0.0, road_length]
    result.ref_y = [0.0, 0.0]
    return result


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
    """Replan along a fixed circular arc."""
    from scripts.planner_test_fixtures import build_curved_lattice_plan_frame, curved_arc_pose

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        obstacles = [_obstacle_from_actor(actor) for actor in actors_now]
        _frame, reference_line_info, start = build_curved_lattice_plan_frame(
            obstacles,
            radius=radius,
            arc_length=arc_length,
            init_v=max(0.0, ego_now.v),
            blocking_obstacle_id=blocking_actor_id,
            start_x=ego_now.x,
            start_y=ego_now.y,
            start_heading=ego_now.theta,
        )
        start.a = ego_now.a
        start.path_point.kappa = ego_now.kappa
        ok = LatticePlanner().Plan(start, _frame, ADCTrajectory())
        return ok, reference_line_info.trajectory, _trajectory_type_name(reference_line_info), obstacles

    result = _run_loop(
        name,
        ego,
        actors,
        plan_fn,
        horizon_s=horizon_s,
        backup=backup,
        blocking_actor_id=blocking_actor_id,
    )
    ref_x: List[float] = []
    ref_y: List[float] = []
    s = 0.0
    while s <= arc_length + 1e-6:
        x, y, _theta = curved_arc_pose(s, radius=radius)
        ref_x.append(x)
        ref_y.append(y)
        s += 2.0
    result.ref_x = ref_x
    result.ref_y = ref_y
    return result


def run_lane_change_road(
    name: str,
    ego: Actor,
    actors: List[Actor],
    *,
    horizon_s: float = 4.0,
    road_length: float = 160.0,
    backup: Optional[bool] = None,
) -> SimResult:
    """Replan on the current lane plus the lane-change reference, every cycle."""
    from scripts.planner_test_fixtures import build_lane_change_curve, build_lane_change_frame

    npc = actors[0] if actors else None

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        leader = actors_now[0] if actors_now else None
        frame, _target, start = build_lane_change_frame(
            length=road_length,
            ego_v=max(0.0, ego_now.v),
            ego_x=ego_now.x,
            ego_y=ego_now.y,
            ego_heading=ego_now.theta,
            npc_x=leader.x if leader is not None else 20.0,
            npc_y=leader.y if leader is not None else 0.0,
            npc_v=leader.v if leader is not None else 0.0,
        )
        start.a = ego_now.a
        start.path_point.kappa = ego_now.kappa
        driven = frame.mutable_reference_line_info[0]
        ok = LatticePlanner().Plan(start, frame, ADCTrajectory())
        driven = frame.FindDriveReferenceLineInfo() or driven
        obstacles = list(frame.obstacles)
        return ok, driven.trajectory, _trajectory_type_name(driven), obstacles

    result = _run_loop(
        name,
        ego,
        actors,
        plan_fn,
        horizon_s=horizon_s,
        backup=backup,
    )
    result.ref_x = [0.0, road_length]
    result.ref_y = [0.0, 0.0]
    curve = build_lane_change_curve(road_length, -3.5, 30.0, step=2.0)
    result.other_ref_x = [point.x for point in curve]
    result.other_ref_y = [point.y for point in curve]
    del npc
    return result


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
    """Replan a PathBounds borrow path every cycle and execute one cycle of it."""
    from common.frame import Frame
    from scripts.planner_test_fixtures import (
        apply_borrow_path_trajectory,
        apply_path_bounds_overtake_trajectory,
        build_center_lane_reference_line,
    )

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        _reference_line, reference_line_info = build_center_lane_reference_line(
            length=road_length,
            init_v=max(ego_now.v, 0.0),
        )
        start = reference_line_info._adc_planning_point
        start.path_point.x = ego_now.x
        start.path_point.y = ego_now.y
        start.path_point.theta = ego_now.theta
        start.path_point.kappa = ego_now.kappa
        start.v = max(0.0, ego_now.v)
        start.a = ego_now.a
        reference_line_info._vehicle_state.x = ego_now.x
        reference_line_info._vehicle_state.y = ego_now.y
        reference_line_info._vehicle_state.heading = ego_now.theta
        obstacles = [_obstacle_from_actor(actor) for actor in actors_now]
        reference_line_info.Init(obstacles, 10.0)
        if blocking_actor_id is not None:
            reference_line_info.SetBlockingObstacle(blocking_actor_id)
        frame = Frame(0)
        frame._obstacles = {obstacle.Id(): obstacle for obstacle in obstacles}
        frame._reference_line_info = [reference_line_info]
        cruise_v = max(ego_now.v, 6.0)
        if s_curve:
            ok, _detail = apply_path_bounds_overtake_trajectory(
                reference_line_info,
                start,
                path_label,
                cruise_v=cruise_v,
                obstacle_x=actors_now[0].x if actors_now else ego_now.x + 30.0,
            )
        else:
            ok, _detail = apply_borrow_path_trajectory(
                reference_line_info,
                start,
                path_label,
            )
        return ok, reference_line_info.trajectory, _trajectory_type_name(reference_line_info), obstacles

    result = _run_loop(
        name,
        ego,
        actors,
        plan_fn,
        horizon_s=horizon_s,
        blocking_actor_id=blocking_actor_id,
    )
    result.ref_x = [0.0, road_length]
    result.ref_y = [0.0, 0.0]
    result.other_ref_x = [0.0, road_length]
    result.other_ref_y = [3.5 if "left" in path_label else -3.5] * 2
    return result


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
    """Call OnLanePlanning.RunOnce from the executed state every cycle."""
    import config as config_module
    from common.planning_context import PlanningContext
    from on_lane_planning import OnLanePlanning
    from protoclass.adc_trajectory import Point3D
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType
    from protoclass.point_enu import PointENU
    from protoclass.pose import Pose
    from protoclass.prediction_obstacles import PredictionObstacle, PredictionObstacles
    from reference_line.reference_line_provider import ReferenceLineProvider
    from scripts.planner_test_fixtures import (
        build_center_lane_reference_line,
        build_reference_line,
    )

    from copy import deepcopy

    context = PlanningContext()
    if borrow_direction:
        context.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
        context.planning_status.path_decider.decided_side_pass_direction = list(borrow_direction)
        reference_line, reference_line_info = build_center_lane_reference_line(
            length=road_length,
            init_v=max(ego.v, 0.0),
            sample_step=5.0,
        )
        lanes = reference_line_info.Lanes()
    else:
        reference_line, _discretized, reference_line_info = build_reference_line(
            length=road_length,
            init_v=max(ego.v, 0.0),
            sample_step=5.0,
        )
        lanes = reference_line_info.Lanes()

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        provider = ReferenceLineProvider()
        provider._reference_lines = [deepcopy(reference_line)]
        provider._route_segments = [deepcopy(lanes)]
        local_view = _local_view(ego_now, actors_now, Chassis, Header, LocalizationEstimate, PointENU, Pose, Point3D, PerceptionObstacle, PerceptionObstacleType, PredictionObstacle, PredictionObstacles)
        planner = OnLanePlanning(provider)
        adc = ADCTrajectory()
        status = planner.RunOnce(local_view, adc, planning_context=context)
        obstacles = [_obstacle_from_actor(actor) for actor in actors_now]
        trajectory = DiscretizedTrajectory(list(adc.trajectory_point)) if adc.trajectory_point and len(adc.trajectory_point) >= 2 else None
        type_name = adc.trajectory_type.name if adc.trajectory_type is not None else "UNKNOWN"
        return status.ok() and trajectory is not None, trajectory, type_name, obstacles

    old_values = (
        config_module.FLAGS_enable_reference_line_provider_thread,
        config_module.FLAGS_enable_path_bounds_decider,
        config_module.FLAGS_enable_on_lane_combine_path_and_speed,
        config_module.FLAGS_enable_path_decider_after_lateral,
    )
    config_module.FLAGS_enable_reference_line_provider_thread = True
    config_module.FLAGS_enable_path_bounds_decider = bool(borrow_direction)
    config_module.FLAGS_enable_on_lane_combine_path_and_speed = bool(borrow_direction)
    config_module.FLAGS_enable_path_decider_after_lateral = bool(borrow_direction)
    try:
        result = _run_loop(
            name,
            ego,
            actors,
            plan_fn,
            horizon_s=horizon_s,
            blocking_actor_id=blocking_actor_id,
        )
    finally:
        (
            config_module.FLAGS_enable_reference_line_provider_thread,
            config_module.FLAGS_enable_path_bounds_decider,
            config_module.FLAGS_enable_on_lane_combine_path_and_speed,
            config_module.FLAGS_enable_path_decider_after_lateral,
        ) = old_values
    result.ref_x = [0.0, road_length]
    result.ref_y = [0.0, 0.0]
    if borrow_direction:
        side = 3.5 if 1 in borrow_direction else -3.5
        result.other_ref_x = [0.0, road_length]
        result.other_ref_y = [side, side]
    return result


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


def _local_view(ego, actors, Chassis, Header, LocalizationEstimate, PointENU, Pose, Point3D, PerceptionObstacle, PerceptionObstacleType, PredictionObstacle, PredictionObstacles):
    from common.frame import LocalView

    predictions = []
    for actor in actors:
        speed = 0.0 if actor.is_static else actor.v
        predictions.append(
            PredictionObstacle(
                perception_obstacle=PerceptionObstacle(
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
                ),
                is_static=actor.is_static or abs(speed) < 1e-3,
            )
        )
    return LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(
                position=PointENU(x=ego.x, y=ego.y, z=0.0),
                heading=ego.theta,
            ),
            measurement_time=0.0,
        ),
        chassis=Chassis(speed_mps=max(0.0, ego.v), header=Header(timestamp_sec=0.0)),
        prediction_obstacles=PredictionObstacles(prediction_obstacle=predictions),
    )


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
