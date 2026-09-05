#!/usr/bin/env python3
"""
为全部 Lattice 场景用例生成场景动画（默认 0.1s/帧 GIF）。

等价于对 scripts/lattice_scenarios.py 中每个场景执行规划并 --animate，
跳过无法产出轨迹的场景（如 stress 扫描、仅 PathBounds 等）。

用法:
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

# matplotlib 缓存写到工程内，避免权限问题
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
        description="为全部场景用例生成场景动画 GIF（0.1s/帧）"
    )
    parser.add_argument(
        "--tag",
        choices=["lattice", "decider", "on_lane", "stress", "overtake", "all"],
        default="all",
        help="只处理某一类场景（默认 all）",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="列出将参与动画的场景（含预计是否可生成）",
    )
    parser.add_argument(
        "--animate-dt",
        type=float,
        default=0.1,
        help="动画时间步 [s]（默认 0.1）",
    )
    parser.add_argument(
        "--animate-fps",
        type=float,
        default=None,
        help="GIF 帧率（默认 10，即实时播放）",
    )
    parser.add_argument(
        "--anim-dir",
        type=Path,
        default=DEFAULT_ANIM_DIR,
        help="GIF 输出目录",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=100,
        help="GIF/PNG 分辨率",
    )
    parser.add_argument(
        "--include-stress",
        action="store_true",
        help="包含 stress 类（默认过滤；通常无轨迹，会跳过动画）",
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
        print(f"共 {len(scenarios)} 个场景（tag={args.tag}）:\n")
        for s in scenarios:
            hint = "可动画" if s.category in ("lattice", "on_lane", "overtake") else "可能无轨迹"
            if not s.expect_ok and not s.informational:
                hint = "通常跳过"
            if s.name == "decider_bounds_assessment":
                hint = "可动画"
            elif s.name in (
                "decider_lane_borrow_bounds",
                "decider_cruise_speed_data",
            ):
                hint = "通常跳过"
            elif s.category == "stress":
                hint = "跳过（无轨迹）"
            print(f"  {s.category:8s} {s.name:36s} {hint}  {s.description}")
        return 0

    print("=" * 60)
    print("全部场景动画生成")
    print(f"  场景数: {len(scenarios)} | dt={args.animate_dt}s | 输出: {args.anim_dir}")
    print("=" * 60)

    outcomes: list[RunOutcome] = []
    for scenario in scenarios:
        print(f"\n>>> 运行: {scenario.name} ...", flush=True)
        out = execute(scenario)
        outcomes.append(out)

    print("\n" + "-" * 60)
    print("生成动画 ...")
    try:
        save_animations(
            outcomes,
            args.anim_dir,
            dt=args.animate_dt,
            fps=args.animate_fps,
            dpi=args.dpi,
        )
    except ImportError as exc:
        print("需要依赖: pip install matplotlib pillow", file=sys.stderr)
        print(f"  ({exc})", file=sys.stderr)
        return 3

    print("\n" + "=" * 60)
    print("结果汇总")
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
            print("  (有轨迹但未生成动画)")
        else:
            skipped += 1
            print("  (跳过动画: 无轨迹或仅 decider/stress)")
        if not out.passed and not out.scenario.informational:
            test_failed += 1

    print(
        f"\n动画: {animated} 个 GIF | 跳过: {skipped} | "
        f"场景断言失败: {test_failed}（不影响已生成的 GIF）"
    )
    print(f"GIF 目录: {args.anim_dir.resolve()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
