"""
Animation of ego vehicle motion on XY map at fixed time steps (default 0.1s).
Output GIF, or frame PNG sequence (frame_0000.png, ...).
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
_MPL_CACHE_DIR = PROJECT_ROOT / "scripts" / "output" / "mpl-cache"
_MPL_CACHE_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CACHE_DIR))

import config as config_module
from scripts.lattice_visualization import PlotContext

import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams["font.sans-serif"] = [
    "Arial Unicode MS",
    "Heiti TC",
    "Songti SC",
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon as MplPolygon

@dataclass
class EgoSample:
    t: float
    x: float
    y: float
    theta: float
    v: float


def _lerp(a: float, b: float, r: float) -> float:
    """
    Linear interpolation between a and b

    :param a: first value
    :param b: second value
    :param r: interpolation parameter
    :returns: interpolated value
    :rtype: float
    """

    return a + (b - a) * r


def _lerp_angle(a: float, b: float, r: float) -> float:
    da = math.atan2(math.sin(b - a), math.cos(b - a))
    return a + da * r


def sample_trajectory_frames(
    ctx: PlotContext,
    dt: float = 0.1,
) -> List[EgoSample]:
    """Sample trajectory points at fixed time steps dt;
    linearly interpolate between trajectory points.
    
    :param ctx: PlotContext
    :param dt: time step
    :returns: list of EgoSample objects
    :rtype: List[EgoSample]
    """

    if not ctx.traj_t or len(ctx.traj_t) < 2:
        if ctx.traj_t and ctx.traj_x:
            return [
                EgoSample(
                    t=ctx.traj_t[0],
                    x=ctx.traj_x[0],
                    y=ctx.traj_y[0],
                    theta=0.0,
                    v=ctx.traj_v[0] if ctx.traj_v else 0.0,
                )
            ]
        return []

    thetas: List[float] = []
    for i in range(len(ctx.traj_x)):
        if i + 1 < len(ctx.traj_x):
            dx = ctx.traj_x[i + 1] - ctx.traj_x[i]
            dy = ctx.traj_y[i + 1] - ctx.traj_y[i]
            thetas.append(math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else 0.0)
        else:
            thetas.append(thetas[-1] if thetas else 0.0)

    t_end = ctx.traj_t[-1]
    samples: List[EgoSample] = []
    t = 0.0
    while t <= t_end + 1e-9:
        # find the interval containing t
        j = 0
        while j + 1 < len(ctx.traj_t) and ctx.traj_t[j + 1] < t:
            j += 1
        if j + 1 >= len(ctx.traj_t):
            j = len(ctx.traj_t) - 2
        t0, t1 = ctx.traj_t[j], ctx.traj_t[j + 1]
        r = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        r = max(0.0, min(1.0, r))
        samples.append(
            EgoSample(
                t=t,
                x=_lerp(ctx.traj_x[j], ctx.traj_x[j + 1], r),
                y=_lerp(ctx.traj_y[j], ctx.traj_y[j + 1], r),
                theta=_lerp_angle(thetas[j], thetas[j + 1], r),
                v=_lerp(ctx.traj_v[j], ctx.traj_v[j + 1], r) if ctx.traj_v else 0.0,
            )
        )
        t += dt
    return samples


def _ego_polygon_xy(x: float, y: float, theta: float) -> List[Tuple[float, float]]:
    """Treat trajectory points as rear axle center,
    convert to vehicle geometric center and draw rectangle.
    
    :param x: x coordinate
    :param y: y coordinate
    :param theta: heading angle
    :returns: list of (x, y) coordinates of the polygon
    :rtype: List[Tuple[float, float]]
    """

    shift = config_module.EGO_VEHICLE_LENGTH / 2.0 - config_module.EGO_BACK_EDGE_TO_CENTER
    cx = x + shift * math.cos(theta)
    cy = y + shift * math.sin(theta)
    hl = config_module.EGO_VEHICLE_LENGTH / 2.0
    hw = config_module.EGO_VEHICLE_WIDTH / 2.0
    local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    c, s = math.cos(theta), math.sin(theta)
    return [(cx + lx * c - ly * s, cy + lx * s + ly * c) for lx, ly in local]


def _compute_view_limits(ctx: PlotContext,
                        margin: float = 3.0,
                        ) -> Tuple[float, float, float, float]:
    """
    Compute view limits for the plot
    
    :param ctx: PlotContext
    :param margin: margin around the data
    :returns: tuple of (xmin, xmax, ymin, ymax)
    :rtype: Tuple[float, float, float, float]
    """

    xs = list(ctx.ref_x) + list(ctx.traj_x)
    ys = list(ctx.ref_y) + list(ctx.traj_y)
    for obs in ctx.obstacles:
        xs.extend(obs.xs)
        ys.extend(obs.ys)
    if not xs:
        return -5, 55, -5, 5
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def _draw_static_scene(ax, ctx: PlotContext) -> None:
    if ctx.ref_x:
        ax.plot(ctx.ref_x, ctx.ref_y, color="#bbbbbb", linewidth=2.0, zorder=1)
    if ctx.path_x:
        ax.plot(
            ctx.path_x,
            ctx.path_y,
            color="#2ca02c",
            linewidth=1.0,
            linestyle="--",
            alpha=0.7,
            zorder=2,
        )
    for obs in ctx.obstacles:
        color = "#e74c3c" if obs.is_blocking else "#9b59b6"
        ax.fill(
            obs.xs,
            obs.ys,
            color=color,
            alpha=0.45,
            edgecolor=color,
            linewidth=1.5,
            zorder=3,
        )
        cx = sum(obs.xs[:-1]) / max(len(obs.xs) - 1, 1)
        cy = sum(obs.ys[:-1]) / max(len(obs.ys) - 1, 1)
        ax.text(cx, cy, obs.label, fontsize=8, ha="center", zorder=4)


def animate_context(
    ctx: PlotContext,
    *,
    output_gif: Optional[Path] = None,
    frames_dir: Optional[Path] = None,
    dt: float = 0.1,
    fps: Optional[float] = None,
    dpi: int = 100,
    show: bool = False,
) -> Tuple[Optional[Path], int]:
    """
    生成场景动画。返回 (gif路径, 帧数)。
    fps 默认 1/dt，即实时播放。
    """
    samples = sample_trajectory_frames(ctx, dt=dt)
    if not samples:
        return None, 0

    if fps is None:
        fps = 1.0 / dt

    xmin, xmax, ymin, ymax = _compute_view_limits(ctx)
    status = "PASS" if ctx.passed else "FAIL"

    fig, ax = plt.subplots(figsize=(9, 7))
    fig.subplots_adjust(left=0.08, right=0.95, top=0.92, bottom=0.08)

    trail_line, = ax.plot([], [], color="#3498db", linewidth=2.5, zorder=5)
    full_line, = ax.plot([], [], color="#aed6f1", linewidth=1.0, alpha=0.5, zorder=4)
    ego_patch = MplPolygon([[0, 0]], closed=True, facecolor="#2980b9", edgecolor="#1a5276", linewidth=2.0, zorder=8)
    ax.add_patch(ego_patch)
    time_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
        zorder=10,
    )

    if ctx.traj_x:
        full_line.set_data(ctx.traj_x, ctx.traj_y)

    _draw_static_scene(ax, ctx)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.25)
    fig.suptitle(f"[{status}] {ctx.scenario_name}  |  dt={dt:.2f}s", fontsize=12)

    def _update(frame_idx: int):
        ego = samples[frame_idx]
        trail_x = [s.x for s in samples[: frame_idx + 1]]
        trail_y = [s.y for s in samples[: frame_idx + 1]]
        trail_line.set_data(trail_x, trail_y)
        ego_patch.set_xy(_ego_polygon_xy(ego.x, ego.y, ego.theta))
        time_text.set_text(f"t = {ego.t:.1f} s\nv = {ego.v:.2f} m/s\n({ego.x:.1f}, {ego.y:.1f})")
        return trail_line, ego_patch, time_text

    anim = animation.FuncAnimation(
        fig,
        _update,
        frames=len(samples),
        interval=1000.0 / fps,
        blit=False,
        repeat=True,
    )

    saved_gif: Optional[Path] = None
    if frames_dir is not None:
        frames_dir = Path(frames_dir)
        frames_dir.mkdir(parents=True, exist_ok=True)
        for i in range(len(samples)):
            _update(i)
            fig.savefig(frames_dir / f"frame_{i:04d}.png", dpi=dpi)
        saved_gif = frames_dir

    if output_gif is not None:
        output_gif = Path(output_gif)
        output_gif.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.PillowWriter(fps=fps)
        anim.save(str(output_gif), writer=writer, dpi=dpi)
        saved_gif = output_gif

    if show:
        plt.show()
    else:
        plt.close(fig)

    return saved_gif, len(samples)


def default_animation_path(plot_dir: Path, scenario_name: str) -> Path:
    safe = scenario_name.replace("/", "_")
    return plot_dir / f"{safe}.gif"


def default_frames_dir(plot_dir: Path, scenario_name: str) -> Path:
    safe = scenario_name.replace("/", "_")
    return plot_dir / safe


def main() -> int:
    print(
        "lattice_animation.py is the animation tool module, please generate animations through the scenario runner:\n"
        "  python scripts/run_lattice_scenario_cases.py open_road --animate\n"
        "  python scripts/run_overtake_animations.py"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
