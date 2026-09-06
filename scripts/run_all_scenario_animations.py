#!/usr/bin/env python3
"""
Generate scenario animations for all Lattice scenario cases (default 0.1s/frame GIF).

Equivalent to running planning with --animate for every scenario in
scripts/lattice_scenarios.py, skipping scenarios that cannot produce a
trajectory (e.g. stress sweeps, PathBounds-only scenarios, etc.).

Usage:
  .venv/bin/python scripts/run_all_scenario_animations.py
  .venv/bin/python scripts/run_all_scenario_animations.py --tag lattice
  .venv/bin/python scripts/run_all_scenario_animations.py --list
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

# write matplotlib's cache inside the project to avoid permission issues
os.environ.setdefault("MPLCONFIGDIR", str(PROJECT_ROOT / "scripts" / "output" / "mpl-cache"))

from scripts.lattice_scenarios import SCENARIOS
from scripts.run_lattice_scenario_cases import (
    DEFAULT_ANIM_DIR,
    RunOutcome,
    execute,
    print_outcome,
    save_animations,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate scenario animation GIFs for all scenario cases (0.1s/frame)"
    )
    parser.add_argument(
        "--tag",
        choices=["lattice", "decider", "on_lane", "stress", "overtake", "lane_change", "all"],
        default="all",
        help="Only process one category of scenarios (default: all)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List the scenarios that would be animated (including whether they're expected to produce one)",
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
        help="GIF frame rate (default 10, i.e. real-time playback)",
    )
    parser.add_argument(
        "--anim-dir",
        type=Path,
        default=DEFAULT_ANIM_DIR,
        help="GIF output directory",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=100,
        help="GIF/PNG resolution",
    )
    parser.add_argument(
        "--include-stress",
        action="store_true",
        help="Include the stress category (filtered out by default; usually has no trajectory and is skipped)",
    )
    args = parser.parse_args()

    scenarios = (
        SCENARIOS
        if args.tag == "all"
        else [s for s in SCENARIOS if s.category == args.tag]
    )
    if not args.include_stress and args.tag == "all":
        scenarios = [s for s in scenarios if s.category != "stress"]

    if args.list:
        print(f"{len(scenarios)} scenarios total (tag={args.tag}):\n")
        for s in scenarios:
            hint = "animatable" if s.category in ("lattice", "on_lane", "overtake", "lane_change") else "may have no trajectory"
            if not s.expect_ok and not s.informational:
                hint = "usually skipped"
            if s.name == "decider_bounds_assessment":
                hint = "animatable"
            elif s.name in (
                "decider_lane_borrow_bounds",
                "decider_cruise_speed_data",
            ):
                hint = "usually skipped"
            elif s.category == "stress":
                hint = "skipped (no trajectory)"
            print(f"  {s.category:8s} {s.name:36s} {hint}  {s.description}")
        return 0

    print("=" * 60)
    print("Generating animations for all scenarios")
    print(f"  Scenarios: {len(scenarios)} | dt={args.animate_dt}s | output: {args.anim_dir}")
    print("=" * 60)

    outcomes: list[RunOutcome] = []
    for scenario in scenarios:
        print(f"\n>>> Running: {scenario.name} ...", flush=True)
        out = execute(scenario)
        outcomes.append(out)

    print("\n" + "-" * 60)
    print("Generating animations ...")
    try:
        save_animations(
            outcomes,
            args.anim_dir,
            dt=args.animate_dt,
            fps=args.animate_fps,
            dpi=args.dpi,
        )
    except ImportError as exc:
        print("Missing dependency: pip install matplotlib pillow", file=sys.stderr)
        print(f"  ({exc})", file=sys.stderr)
        return 3

    print("\n" + "=" * 60)
    print("Result summary")
    print("=" * 60)

    animated = 0
    skipped = 0
    test_failed = 0

    for out in outcomes:
        print_outcome(out)
        if out.animation_path is not None and out.animation_frames > 0:
            animated += 1
        elif out.scene_context and out.scene_context.traj_x:
            skipped += 1
            print("  (has a trajectory but no animation was generated)")
        else:
            skipped += 1
            print("  (animation skipped: no trajectory, or decider/stress only)")
        if not out.passed and not out.scenario.informational:
            test_failed += 1

    print(
        f"\nAnimations: {animated} GIFs | Skipped: {skipped} | "
        f"Scenario assertion failures: {test_failed} (does not affect GIFs already generated)"
    )
    print(f"GIF directory: {args.anim_dir.resolve()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
