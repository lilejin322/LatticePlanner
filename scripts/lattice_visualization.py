"""
Shared scene context for lattice animations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


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
    traj_theta: List[float] = field(default_factory=list)
    path_x: List[float] = field(default_factory=list)
    path_y: List[float] = field(default_factory=list)
    obstacles: List[ObstacleDraw] = field(default_factory=list)
    ego_x: float = 0.0
    ego_y: float = 0.0
    traj_type: str = ""
    note: str = ""


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
