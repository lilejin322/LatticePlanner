#!/usr/bin/env python3
"""
Lattice scenario suite. Driving cases replan every cycle and the animation
plays that executed trace. Decider cases are single-call checks.

Usage:
  .venv/bin/python scripts/run_lattice_scenario_cases.py --list
  .venv/bin/python scripts/run_lattice_scenario_cases.py
  .venv/bin/python scripts/run_lattice_scenario_cases.py --tag lattice
  .venv/bin/python scripts/run_lattice_scenario_cases.py open_road --animate
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
_MPL_CACHE_DIR = PROJECT_ROOT / "scripts" / "output" / "mpl-cache"
_MPL_CACHE_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_MPL_CACHE_DIR))

import config
from scripts.closed_loop_sim import SimResult, to_plot_context
from scripts.lattice_scenarios import SCENARIOS, SCENARIOS_BY_NAME, Scenario

DEFAULT_ANIM_DIR = PROJECT_ROOT / "scripts" / "output" / "animations"


@dataclass
class RunOutcome:
    scenario: Scenario
    got_ok: bool
    detail: str
    error: Optional[str] = None
    scene_context: Optional[object] = None
    animation_path: Optional[Path] = None
    animation_frames: int = 0

    @property
    def passed(self) -> bool:
        if self.scenario.informational:
            return self.error is None
        return self.got_ok == self.scenario.expect_ok and self.error is None


def execute(scenario: Scenario) -> RunOutcome:
    try:
        built = scenario.builder()
        if isinstance(built, SimResult):
            return RunOutcome(
                scenario=scenario,
                got_ok=built.ok,
                detail=built.summary(),
                scene_context=to_plot_context(built, scenario.name, scenario.description),
            )

        extra = built[5] if len(built) > 5 else None
        got_ok = bool(built[3])
        return RunOutcome(
            scenario=scenario,
            got_ok=got_ok,
            detail=extra or "",
        )
    except Exception as exc:
        return RunOutcome(
            scenario=scenario,
            got_ok=False,
            detail="",
            error=f"{type(exc).__name__}: {exc}",
        )


def save_animations(
    outcomes: List[RunOutcome],
    anim_dir: Path,
    *,
    dt: float = 0.1,
    fps: Optional[float] = None,
    dpi: int = 100,
    show: bool = False,
) -> None:
    from scripts.lattice_animation import animate_context, default_animation_path

    anim_dir = Path(anim_dir)
    saved = 0
    total_frames = 0
    for out in outcomes:
        if out.scene_context is None or not out.scene_context.traj_x:
            continue
        gif_path = default_animation_path(anim_dir, out.scenario.name)
        path, n = animate_context(
            out.scene_context,
            output_gif=gif_path,
            dt=dt,
            fps=fps,
            dpi=dpi,
            show=show,
        )
        if path and n > 0:
            out.animation_path = path if path.suffix == ".gif" else gif_path
            out.animation_frames = n
            saved += 1
            total_frames += n
    print(
        f"\nSaved {saved} scenario animations ({total_frames} frames total @ {dt}s) -> {anim_dir.resolve()}"
    )


def print_outcome(out: RunOutcome) -> None:
    s = out.scenario
    tag = s.category
    if out.error:
        print(f"\n[ERROR] [{tag}] {s.name}")
        print(f"  {s.description}")
        print(f"  {out.error}")
        return

    if s.informational:
        status = "INFO"
    else:
        status = "PASS" if out.passed else "FAIL"
    expect = "success" if s.expect_ok else "failure"
    got = "success" if out.got_ok else "failure"
    print(f"\n[{status}] [{tag}] {s.name}")
    print(f"  {s.description}")
    if not s.informational:
        print(f"  Expected: {expect} | Actual: {got}")
    print(f"  {out.detail}")
    if out.animation_path is not None:
        print(f"  Animation: {out.animation_path} ({out.animation_frames} frames)")


def main() -> int:
    parser = argparse.ArgumentParser(description="Lattice full scenario test suite")
    parser.add_argument("cases", nargs="*", help="Case names")
    parser.add_argument("--list", action="store_true", help="List the cases")
    parser.add_argument(
        "--tag",
        choices=["lattice", "decider", "on_lane", "stress", "overtake", "lane_change", "sim", "all"],
        default="all",
        help="Filter by category",
    )
    parser.add_argument("--fail-fast", action="store_true", help="Exit on the first failure")
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Scenario animation GIF: the ego vehicle moves on the map one frame every 0.1s",
    )
    parser.add_argument(
        "--animate-dt",
        type=float,
        default=0.1,
        help="Animation time step [s] (default 0.1)",
    )
    parser.add_argument(
        "--animate-fps",
        type=float,
        default=None,
        help="GIF playback frame rate (default 1/animate-dt, i.e. real-time)",
    )
    parser.add_argument(
        "--anim-dir",
        type=Path,
        default=DEFAULT_ANIM_DIR,
        help="GIF output directory",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show the animation in a popup window (requires a GUI)",
    )
    parser.add_argument("--dpi", type=int, default=120, help="Animation render DPI")
    args = parser.parse_args()

    if args.cases:
        selected = []
        for name in args.cases:
            if name not in SCENARIOS_BY_NAME:
                print(f"Unknown case: {name}", file=sys.stderr)
                return 2
            selected.append(SCENARIOS_BY_NAME[name])
    else:
        selected = (
            SCENARIOS
            if args.tag == "all"
            else [s for s in SCENARIOS if s.category == args.tag]
        )

    if args.list:
        print(f"{len(selected)} scenarios total")
        if args.tag != "all":
            print(f"Filter: tag={args.tag}")
        if args.cases:
            print(f"Filter: cases={', '.join(args.cases)}")
        print()
        categories = ("lattice", "decider", "on_lane", "stress", "overtake", "lane_change", "sim")
        for cat in categories:
            items = [s for s in selected if s.category == cat]
            if not items:
                continue
            print(f"  [{cat}] ({len(items)})")
            for s in items:
                mark = "ℹ" if s.informational else ("✓" if s.expect_ok else "✗")
                print(f"    {mark} {s.name:36s} {s.description}")
        print("\nAnimation: --animate for scenario GIFs (0.1s/frame)")
        return 0

    print(f"Lattice scenario test suite | tag={args.tag} | {len(selected)} cases total")
    print(f"backup default: {config.FLAGS_enable_backup_trajectory}")
    if args.animate:
        print(f"Scenario animation dt={args.animate_dt}s -> {args.anim_dir}")
    if args.animate or args.show:
        print()
    else:
        print()

    outcomes: List[RunOutcome] = []
    for scenario in selected:
        out = execute(scenario)
        outcomes.append(out)
        if args.fail_fast and not out.passed:
            break

    if args.animate:
        try:
            save_animations(
                outcomes,
                args.anim_dir,
                dt=args.animate_dt,
                fps=args.animate_fps,
                dpi=args.dpi,
                show=args.show,
            )
        except ImportError as exc:
            print("\nAnimation requires matplotlib pillow: pip install matplotlib pillow", file=sys.stderr)
            print(f"  ({exc})", file=sys.stderr)
            return 3

    for out in outcomes:
        print_outcome(out)

    passed = sum(1 for o in outcomes if o.passed)
    failed = [o.scenario.name for o in outcomes if not o.passed]
    print(f"\nTotal: {passed}/{len(outcomes)} passed", end="")
    if failed:
        print(f" | Failed: {', '.join(failed)}", end="")
    print()
    return 0 if passed == len(outcomes) else 1


if __name__ == "__main__":
    raise SystemExit(main())
