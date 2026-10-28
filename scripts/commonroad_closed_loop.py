"""Closed-loop playback as a CommonRoad scenario.

The executed trace becomes a dynamic obstacle the way the CommonRoad
scenario tutorial does it: an ``InitialState`` at time step 0 and a
``Trajectory`` / ``TrajectoryPrediction`` for the later steps. Each frame
is ``Scenario.draw`` on an ``MPRenderer`` with ``time_begin`` set to that
step, which is the interface in ``mp_renderer.py``.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

_MPL_CACHE_DIR = Path(__file__).resolve().parents[1] / "scripts" / "output" / "mpl-cache"
_MPL_CACHE_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CACHE_DIR))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

import config as config_module
from commonroad.common.common_scenario import FileInformation
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LaneletType
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from scripts.lattice_visualization import ObstacleDraw, PlotContext

LANE_HALF_WIDTH = 1.75


def _unit_normal(dx: float, dy: float) -> Tuple[float, float]:
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0, 1.0
    return -dy / length, dx / length


def _lanelet_from_polyline(lanelet_id: int, xs: Sequence[float], ys: Sequence[float]) -> Optional[Lanelet]:
    points = [(float(x), float(y)) for x, y in zip(xs, ys)]
    compact = [points[0]] if points else []
    for point in points[1:]:
        if math.hypot(point[0] - compact[-1][0], point[1] - compact[-1][1]) > 1e-3:
            compact.append(point)
    if len(compact) < 2:
        return None
    left, center, right = [], [], []
    for index, (x, y) in enumerate(compact):
        if index == 0:
            dx, dy = compact[1][0] - x, compact[1][1] - y
        elif index == len(compact) - 1:
            dx, dy = x - compact[index - 1][0], y - compact[index - 1][1]
        else:
            dx = compact[index + 1][0] - compact[index - 1][0]
            dy = compact[index + 1][1] - compact[index - 1][1]
        nx, ny = _unit_normal(dx, dy)
        center.append([x, y])
        left.append([x + nx * LANE_HALF_WIDTH, y + ny * LANE_HALF_WIDTH])
        right.append([x - nx * LANE_HALF_WIDTH, y - ny * LANE_HALF_WIDTH])
    lanelet = Lanelet(
        left_vertices=np.array(left),
        center_vertices=np.array(center),
        right_vertices=np.array(right),
        lanelet_id=lanelet_id,
        lanelet_type={LaneletType.URBAN},
    )
    return lanelet


def _heading(xs: Sequence[float], ys: Sequence[float], thetas: Sequence[float], index: int) -> float:
    if index < len(thetas):
        return float(thetas[index])
    if index + 1 < len(xs):
        return math.atan2(ys[index + 1] - ys[index], xs[index + 1] - xs[index])
    if index > 0:
        return math.atan2(ys[index] - ys[index - 1], xs[index] - xs[index - 1])
    return 0.0


def _state(time_step: int, x: float, y: float, theta: float, velocity: float):
    kwargs = dict(
        time_step=time_step,
        position=np.array([x, y], dtype=float),
        orientation=float(theta),
        velocity=float(velocity),
    )
    if time_step == 0:
        return InitialState(**kwargs)
    return CustomState(**kwargs)


def _dynamic_obstacle(
    obstacle_id: int,
    xs: Sequence[float],
    ys: Sequence[float],
    thetas: Sequence[float],
    velocities: Sequence[float],
    length: float,
    width: float,
) -> DynamicObstacle:
    shape = RectObstacleShape(width=width, length=length)
    initial = _state(0, xs[0], ys[0], _heading(xs, ys, thetas, 0), velocities[0] if velocities else 0.0)
    prediction = None
    if len(xs) > 1:
        later = [
            _state(
                step,
                xs[step],
                ys[step],
                _heading(xs, ys, thetas, step),
                velocities[step] if step < len(velocities) else velocities[-1] if velocities else 0.0,
            )
            for step in range(1, len(xs))
        ]
        prediction = TrajectoryPrediction(Trajectory(1, later), shape)
    return DynamicObstacle(obstacle_id, ObstacleType.CAR, shape, initial, prediction)


def _obstacle_pose(obs: ObstacleDraw) -> Tuple[float, float, float]:
    if obs.traj_x and obs.traj_y:
        theta = obs.traj_theta[0] if obs.traj_theta else 0.0
        return obs.traj_x[0], obs.traj_y[0], theta
    cx = sum(obs.xs[:-1]) / max(len(obs.xs) - 1, 1)
    cy = sum(obs.ys[:-1]) / max(len(obs.ys) - 1, 1)
    if len(obs.xs) >= 2:
        theta = math.atan2(obs.ys[1] - obs.ys[0], obs.xs[1] - obs.xs[0])
    else:
        theta = 0.0
    return cx, cy, theta


def _fit_series(values: Sequence[float], count: int, fallback: float) -> List[float]:
    if not values:
        return [fallback] * count
    if len(values) >= count:
        return [float(values[i]) for i in range(count)]
    padded = [float(value) for value in values]
    padded.extend([padded[-1]] * (count - len(padded)))
    return padded


def scenario_from_closed_loop(ctx: PlotContext, dt: float = 0.1) -> Scenario:
    """Executed trace as a CommonRoad scenario at ``dt`` per time step."""
    scenario = Scenario(
        dt=dt,
        scenario_id=ScenarioID(country_id="DEU", map_name=ctx.scenario_name or "closed-loop", map_id=1),
        file_information=FileInformation(author="LatticePlanner", source="closed-loop"),
    )
    network = LaneletNetwork()
    lanelet = _lanelet_from_polyline(1, ctx.ref_x, ctx.ref_y)
    if lanelet is not None:
        network.add_lanelet(lanelet)
    other = _lanelet_from_polyline(2, ctx.other_lane_x, ctx.other_lane_y)
    if other is not None:
        network.add_lanelet(other)
    if network.lanelets:
        scenario.add_objects(network)

    count = len(ctx.traj_x)
    if count == 0:
        return scenario
    thetas = list(ctx.traj_theta)
    velocities = _fit_series(ctx.traj_v, count, 0.0)
    shift = config_module.EGO_VEHICLE_LENGTH / 2.0 - config_module.EGO_BACK_EDGE_TO_CENTER
    ego_x, ego_y = [], []
    for index in range(count):
        theta = _heading(ctx.traj_x, ctx.traj_y, thetas, index)
        ego_x.append(ctx.traj_x[index] + shift * math.cos(theta))
        ego_y.append(ctx.traj_y[index] + shift * math.sin(theta))
    scenario.add_objects(
        _dynamic_obstacle(
            100,
            ego_x,
            ego_y,
            thetas,
            velocities,
            config_module.EGO_VEHICLE_LENGTH,
            config_module.EGO_VEHICLE_WIDTH,
        )
    )

    next_id = 101
    for obs in ctx.obstacles:
        if obs.is_static or len(obs.traj_x) < 2:
            x, y, theta = _obstacle_pose(obs)
            shape = RectObstacleShape(width=obs.width, length=obs.length)
            scenario.add_objects(
                StaticObstacle(
                    next_id,
                    ObstacleType.PARKED_VEHICLE if obs.is_static else ObstacleType.CAR,
                    shape,
                    InitialState(
                        time_step=0,
                        position=np.array([x, y], dtype=float),
                        orientation=theta,
                        velocity=0.0,
                    ),
                )
            )
        else:
            steps = min(count, len(obs.traj_x))
            xs = _fit_series(obs.traj_x, count, obs.traj_x[-1])
            ys = _fit_series(obs.traj_y, count, obs.traj_y[-1])
            thetas = _fit_series(obs.traj_theta, count, 0.0)
            # Keep the recorded motion, then hold the last pose for the rest of the run.
            if steps < count:
                xs[steps:] = [xs[steps - 1]] * (count - steps)
                ys[steps:] = [ys[steps - 1]] * (count - steps)
            scenario.add_objects(
                _dynamic_obstacle(next_id, xs, ys, thetas, [], obs.length, obs.width)
            )
        next_id += 1
    return scenario


def _view_limits(ctx: PlotContext, margin: float = 12.0) -> Tuple[float, float, float, float]:
    xs = list(ctx.traj_x)
    ys = list(ctx.traj_y)
    for obs in ctx.obstacles:
        xs.extend(obs.traj_x or obs.xs)
        ys.extend(obs.traj_y or obs.ys)
    if not xs:
        return -10.0, 40.0, -15.0, 15.0
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def animate_commonroad_context(
    ctx: PlotContext,
    *,
    output_gif: Optional[Path] = None,
    dt: float = 0.1,
    fps: Optional[float] = None,
    dpi: int = 100,
    show: bool = False,
) -> Tuple[Optional[Path], int]:
    """Play the executed trace with CommonRoad's time-step renderer."""
    if len(ctx.traj_x) < 1:
        return None, 0
    scenario = scenario_from_closed_loop(ctx, dt=dt)
    frame_count = len(ctx.traj_x)
    if fps is None:
        fps = 1.0 / dt
    xmin, xmax, ymin, ymax = _view_limits(ctx)
    status = "PASS" if ctx.passed else "FAIL"

    fig, ax = plt.subplots(figsize=(9, 7))
    fig.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=0.08)

    def _update(frame_idx: int):
        ax.clear()
        renderer = MPRenderer(ax=ax)
        params = MPDrawParams()
        params.time_begin = frame_idx
        params.time_end = frame_idx
        # The full prediction line would show the rest of the executed trace
        # before the vehicle gets there. The current occupancy is the driven pose.
        params.dynamic_obstacle.trajectory.draw_trajectory = False
        params.axis_visible = True
        scenario.draw(renderer, params)
        renderer.render()
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        velocity = ctx.traj_v[frame_idx] if frame_idx < len(ctx.traj_v) else 0.0
        time_s = ctx.traj_t[frame_idx] if frame_idx < len(ctx.traj_t) else frame_idx * dt
        ax.set_title(
            f"[{status}] {ctx.scenario_name}  t={time_s:.1f}s  v={velocity:.2f} m/s"
        )
        return []

    anim = animation.FuncAnimation(
        fig,
        _update,
        frames=frame_count,
        interval=1000.0 / fps,
        blit=False,
        repeat=False,
    )
    saved: Optional[Path] = None
    if output_gif is not None:
        output_gif = Path(output_gif)
        output_gif.parent.mkdir(parents=True, exist_ok=True)
        anim.save(str(output_gif), writer=animation.PillowWriter(fps=fps), dpi=dpi)
        saved = output_gif
    if show:
        plt.show()
    else:
        plt.close(fig)
    return saved, frame_count
