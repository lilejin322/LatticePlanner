"""
Closed-loop playback. The GIF is a CommonRoad scenario rendered one time step
at a time; see scripts/commonroad_closed_loop.py.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
_MPL_CACHE_DIR = PROJECT_ROOT / "scripts" / "output" / "mpl-cache"
_MPL_CACHE_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CACHE_DIR))

from scripts.lattice_visualization import PlotContext


def animate_context(
    ctx: PlotContext,
    *,
    output_gif: Optional[Path] = None,
    dt: float = 0.1,
    fps: Optional[float] = None,
    dpi: int = 100,
    show: bool = False,
) -> Tuple[Optional[Path], int]:
    """Play the executed trace as a CommonRoad scenario. Returns (gif_path, frame_count)."""
    from scripts.commonroad_closed_loop import animate_commonroad_context

    return animate_commonroad_context(
        ctx,
        output_gif=output_gif,
        dt=dt,
        fps=fps,
        dpi=dpi,
        show=show,
    )


def default_animation_path(plot_dir: Path, scenario_name: str) -> Path:
    safe = scenario_name.replace("/", "_")
    return plot_dir / f"{safe}.gif"


def main() -> int:
    print(
        "lattice_animation.py plays a closed-loop trace as a CommonRoad scenario:\n"
        "  python scripts/run_lattice_scenario_cases.py open_road --animate\n"
        "  python scripts/run_overtake_animations.py"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
