#!/usr/bin/env python3
"""
精简演示（6 项）。全量场景请用 run_lattice_scenario_cases.py。

  .venv/bin/python scripts/run_lattice_demo_cases.py --list
  .venv/bin/python scripts/run_lattice_scenario_cases.py --list
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

# 复用全场景 runner
from scripts.run_lattice_scenario_cases import (
    DEFAULT_ANIM_DIR,
    DEFAULT_FRAMES_DIR,
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
    parser = argparse.ArgumentParser(description="Lattice 精简演示（6 项）")
    parser.add_argument("cases", nargs="*", help="用例名")
    parser.add_argument("--list", action="store_true")
    parser.add_argument(
        "--animate",
        action="store_true",
        help="场景动画 GIF（0.1s/帧）",
    )
    parser.add_argument("--animate-dt", type=float, default=0.1)
    parser.add_argument("--animate-frames", action="store_true", help="同时导出逐帧 PNG")
    parser.add_argument("--anim-dir", type=Path, default=DEFAULT_ANIM_DIR)
    parser.add_argument("--frames-dir", type=Path, default=DEFAULT_FRAMES_DIR)
    parser.add_argument("--show", action="store_true", help="弹窗显示动画")
    args = parser.parse_args()

    if args.list:
        print("精简演示用例（完整列表见 run_lattice_scenario_cases.py --list）:\n")
        for name in DEMO_CASE_NAMES:
            s = SCENARIOS_BY_NAME[name]
            print(f"  {name:32s} {s.description}")
        return 0

    names = args.cases if args.cases else DEMO_CASE_NAMES
    outcomes = []
    for name in names:
        if name not in SCENARIOS_BY_NAME:
            print(f"未知用例: {name}", file=sys.stderr)
            return 2
        out = execute(SCENARIOS_BY_NAME[name])
        outcomes.append(out)

    if args.animate:
        try:
            save_animations(
                outcomes,
                args.anim_dir,
                dt=args.animate_dt,
                save_frames=args.animate_frames,
                frames_root=args.frames_dir,
                show=args.show,
            )
        except ImportError:
            print("需要: pip install matplotlib pillow", file=sys.stderr)
            return 3

    for out in outcomes:
        print_outcome(out)

    passed = sum(1 for o in outcomes if o.passed)
    print(f"\n合计: {passed}/{len(outcomes)} 通过")
    return 0 if passed == len(outcomes) else 1


if __name__ == "__main__":
    raise SystemExit(main())
