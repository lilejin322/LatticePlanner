"""
Shared scene context for lattice animations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class ObstacleDraw:
    xs: List[float]
    ys: List[float]
    label: str
    is_blocking: bool = False
    is_static: bool = True
    length: float = 4.0
    width: float = 2.0
    traj_t: List[float] = field(default_factory=list)
    traj_x: List[float] = field(default_factory=list)
    traj_y: List[float] = field(default_factory=list)
    traj_theta: List[float] = field(default_factory=list)


@dataclass
class PlotContext:
    scenario_name: str
    title: str
    passed: bool
    ref_x: List[float] = field(default_factory=list)
    ref_y: List[float] = field(default_factory=list)
    other_lane_x: List[float] = field(default_factory=list)
    other_lane_y: List[float] = field(default_factory=list)
    traj_x: List[float] = field(default_factory=list)
    traj_y: List[float] = field(default_factory=list)
    traj_t: List[float] = field(default_factory=list)
    traj_v: List[float] = field(default_factory=list)
    traj_a: List[float] = field(default_factory=list)
    path_x: List[float] = field(default_factory=list)
    path_y: List[float] = field(default_factory=list)
    obstacles: List[ObstacleDraw] = field(default_factory=list)
    ego_x: float = 0.0
    ego_y: float = 0.0
    traj_type: str = ""
    note: str = ""


def _box_corners(obstacle) -> Tuple[List[float], List[float]]:
    box = obstacle.PerceptionBoundingBox()
    corners = box.GetAllCorners()
    xs = [c.x for c in corners] + [corners[0].x]
    ys = [c.y for c in corners] + [corners[0].y]
    return xs, ys


def context_from_reference_line_info(
    reference_line_info,
    *,
    scenario_name: str,
    title: str,
    passed: bool,
    note: str = "",
    other_reference_line=None,
) -> PlotContext:
    ctx = PlotContext(
        scenario_name=scenario_name,
        title=title,
        passed=passed,
        note=note,
    )

    if reference_line_info is None:
        return ctx

    ref = reference_line_info.reference_line
    for pt in ref.reference_points:
        ctx.ref_x.append(pt.x)
        ctx.ref_y.append(pt.y)

    if other_reference_line is not None:
        other_ref = other_reference_line.reference_line
        for pt in other_ref.reference_points:
            ctx.other_lane_x.append(pt.x)
            ctx.other_lane_y.append(pt.y)

    start = reference_line_info._adc_planning_point
    if start and start.path_point:
        ctx.ego_x = start.path_point.x
        ctx.ego_y = start.path_point.y

    blocking_id = None
    try:
        blocking = reference_line_info.GetBlockingObstacle()
        if blocking is not None:
            blocking_id = blocking.Id()
    except Exception:
        pass

    for obs in reference_line_info.path_decision.obstacles.values():
        xs, ys = _box_corners(obs)
        is_static = not obs.HasTrajectory()
        draw = ObstacleDraw(
            xs=xs,
            ys=ys,
            label=obs.Id(),
            is_blocking=(obs.Id() == blocking_id),
            is_static=is_static,
        )
        if not is_static:
            perception = obs.Perception()
            draw.length = perception.length
            draw.width = perception.width
            for pt in obs.Trajectory().trajectory_point:
                draw.traj_t.append(float(pt.relative_time))
                draw.traj_x.append(pt.path_point.x)
                draw.traj_y.append(pt.path_point.y)
                draw.traj_theta.append(pt.path_point.theta or 0.0)
        ctx.obstacles.append(draw)

    traj = reference_line_info.trajectory
    if traj:
        for i, pt in enumerate(traj):
            ctx.traj_x.append(pt.path_point.x)
            ctx.traj_y.append(pt.path_point.y)
            ctx.traj_v.append(pt.v)
            ctx.traj_a.append(pt.a)
            t = pt.relative_time
            if t is None:
                t = float(i) * 0.1
            ctx.traj_t.append(float(t))
        ctx.traj_type = reference_line_info.trajectory_type.name

    path_data = reference_line_info.path_data
    if path_data is not None and path_data.discretized_path:
        for p in path_data.discretized_path:
            ctx.path_x.append(p.x)
            ctx.path_y.append(p.y)

    return ctx


def context_from_adc_trajectory(
    adc_trajectory,
    *,
    scenario_name: str,
    title: str,
    passed: bool,
    note: str = "",
) -> PlotContext:
    ctx = PlotContext(
        scenario_name=scenario_name,
        title=title,
        passed=passed,
        note=note,
        traj_type="ON_LANE",
    )
    if adc_trajectory is None:
        return ctx
    for i, pt in enumerate(adc_trajectory.trajectory_point):
        ctx.traj_x.append(pt.path_point.x)
        ctx.traj_y.append(pt.path_point.y)
        ctx.traj_v.append(pt.v)
        ctx.traj_a.append(pt.a)
        t = pt.relative_time
        if t is None:
            t = float(i) * 0.1
        ctx.traj_t.append(float(t))
    if ctx.traj_x:
        ctx.ego_x = ctx.traj_x[0]
        ctx.ego_y = ctx.traj_y[0]
    return ctx
