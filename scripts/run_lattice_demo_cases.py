#!/usr/bin/env python3
"""
Condensed demo (6 items). For the full set of scenarios use run_lattice_scenario_cases.py.

  .venv/bin/python scripts/run_lattice_demo_cases.py --list
  .venv/bin/python scripts/run_lattice_scenario_cases.py --list
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

# reuse the full-scenario runner
from scripts.run_lattice_scenario_cases import (
    DEFAULT_ANIM_DIR,
    execute,
    print_outcome,
    save_animations,
)
from scripts.lattice_scenarios import SCENARIOS_BY_NAME

DEMO_CASE_NAMES = [
    "open_road",
    "far_obstacle_stop",
    "close_obstacle_no_backup",
    "close_obstacle_with_backup",
    "higher_speed_cruise",
    "decider_bounds_assessment",
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Lattice condensed demo (6 items)")
    parser.add_argument("cases", nargs="*", help="Case names")
    parser.add_argument("--list", action="store_true")
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Scenario animation GIF (0.1s/frame)",
    )
    parser.add_argument("--animate-dt", type=float, default=0.1)
    parser.add_argument("--anim-dir", type=Path, default=DEFAULT_ANIM_DIR)
    parser.add_argument("--show", action="store_true", help="Show the animation in a popup window")
    args = parser.parse_args()

    if args.list:
        print("Condensed demo cases (see run_lattice_scenario_cases.py --list for the full list):\n")
        for name in DEMO_CASE_NAMES:
            s = SCENARIOS_BY_NAME[name]
            print(f"  {name:32s} {s.description}")
        return 0

    names = args.cases if args.cases else DEMO_CASE_NAMES
    outcomes = []
    for name in names:
        if name not in SCENARIOS_BY_NAME:
            print(f"Unknown case: {name}", file=sys.stderr)
            return 2
        out = execute(SCENARIOS_BY_NAME[name])
        outcomes.append(out)

    if args.animate:
        try:
            save_animations(
                outcomes,
                args.anim_dir,
                dt=args.animate_dt,
                show=args.show,
            )
        except ImportError:
            print("Requires: pip install matplotlib pillow", file=sys.stderr)
            return 3

    for out in outcomes:
        print_outcome(out)

    passed = sum(1 for o in outcomes if o.passed)
    print(f"\nTotal: {passed}/{len(outcomes)} passed")
    return 0 if passed == len(outcomes) else 1


if __name__ == "__main__":
    raise SystemExit(main())
