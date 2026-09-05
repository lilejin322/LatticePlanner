#!/usr/bin/env python3
"""
Lattice 全场景测试（34 项）：按类别跑 Lattice / Decider / OnLane / Stress / Overtake。

用法:
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
from lattice_planner import LatticePlanner
from protoclass.adc_trajectory import ADCTrajectory
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


def trajectory_summary(reference_line_info) -> str:
    traj = reference_line_info.trajectory
    if not traj or len(traj) == 0:
        return "轨迹: 无"
    first, last = traj[0], traj[-1]
    max_v = max(pt.v for pt in traj)
    max_a = max(abs(pt.a) for pt in traj)
    max_y = max(abs(pt.path_point.y) for pt in traj)
    tname = reference_line_info.trajectory_type.name
    return (
        f"轨迹 {len(traj)} 点 | {tname} | "
        f"({first.path_point.x:.1f},{first.path_point.y:.2f},v={first.v:.2f})→"
        f"({last.path_point.x:.1f},{last.path_point.y:.2f},v={last.v:.2f}) | "
        f"max|v|={max_v:.2f} max|a|={max_a:.2f} max|y|={max_y:.2f}"
    )


def run_lattice_plan(
    frame,
    reference_line_info,
    start_point,
    backup: Optional[bool],
) -> bool:
    old = config.FLAGS_enable_backup_trajectory
    if backup is not None:
        config.FLAGS_enable_backup_trajectory = backup
    try:
        return LatticePlanner().Plan(start_point, frame, ADCTrajectory())
    finally:
        config.FLAGS_enable_backup_trajectory = old


def _build_scene_context(
    scenario: Scenario,
    passed: bool,
    *,
    reference_line_info=None,
    adc_trajectory=None,
    note: str = "",
):
    from scripts.lattice_visualization import (
        context_from_adc_trajectory,
        context_from_reference_line_info,
    )

    title = f"{scenario.description} | {note}" if note else scenario.description
    if reference_line_info is not None:
        return context_from_reference_line_info(
            reference_line_info,
            scenario_name=scenario.name,
            title=title,
            passed=passed,
            note=note,
        )
    if adc_trajectory is not None:
        return context_from_adc_trajectory(
            adc_trajectory,
            scenario_name=scenario.name,
            title=title,
            passed=passed,
            note=note,
        )
    return None


def execute(scenario: Scenario) -> RunOutcome:
    try:
        built = scenario.builder()
        extra = built[5] if len(built) > 5 else None
        adc = built[6] if len(built) > 6 else None

        # OnLane / decider-only / stress（无完整轨迹）
        if built[0] is None:
            rli = built[1]
            got_ok = bool(built[3])
            detail = extra or ""
            scene_ctx = None
            if rli is not None or adc is not None:
                scene_ctx = _build_scene_context(
                    scenario,
                    got_ok == scenario.expect_ok if not scenario.informational else True,
                    reference_line_info=rli,
                    adc_trajectory=adc,
                    note=detail,
                )
            return RunOutcome(
                scenario=scenario,
                got_ok=got_ok,
                detail=detail,
                scene_context=scene_ctx,
            )

        frame, rli, start, expect_ok, backup = built[:5]
        del expect_ok

        if extra == "decider_skip":
            return RunOutcome(
                scenario=scenario,
                got_ok=True,
                detail="decider 跳过（未产出 path）",
            )

        if scenario.skip_lattice:
            got_ok = bool(built[3])
        else:
            got_ok = run_lattice_plan(
                frame, rli, start, scenario.backup if scenario.backup is not None else backup
            )
        detail = trajectory_summary(rli)
        if extra:
            detail = f"{extra} | {detail}"

        passed = got_ok == scenario.expect_ok
        scene_ctx = _build_scene_context(scenario, passed, reference_line_info=rli, note=detail)

        return RunOutcome(
            scenario=scenario,
            got_ok=got_ok,
            detail=detail,
            scene_context=scene_ctx,
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
        f"\n已保存 {saved} 个场景动画（共 {total_frames} 帧 @ {dt}s）→ {anim_dir.resolve()}"
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
    expect = "成功" if s.expect_ok else "失败"
    got = "成功" if out.got_ok else "失败"
    print(f"\n[{status}] [{tag}] {s.name}")
    print(f"  {s.description}")
    if not s.informational:
        print(f"  期望: {expect} | 实际: {got}")
    print(f"  {out.detail}")
    if out.animation_path is not None:
        print(f"  动画: {out.animation_path} ({out.animation_frames} 帧)")


def main() -> int:
    parser = argparse.ArgumentParser(description="Lattice 全场景测试")
    parser.add_argument("cases", nargs="*", help="用例名")
    parser.add_argument("--list", action="store_true", help="列出用例")
    parser.add_argument(
        "--tag",
        choices=["lattice", "decider", "on_lane", "stress", "overtake", "lane_change", "all"],
        default="all",
        help="按类别筛选",
    )
    parser.add_argument("--fail-fast", action="store_true", help="首个失败即退出")
    parser.add_argument(
        "--animate",
        action="store_true",
        help="场景动画 GIF：地图上自车每 0.1s 一帧运动",
    )
    parser.add_argument(
        "--animate-dt",
        type=float,
        default=0.1,
        help="动画时间步长 [s]（默认 0.1）",
    )
    parser.add_argument(
        "--animate-fps",
        type=float,
        default=None,
        help="GIF 播放帧率（默认 1/animate-dt，即实时）",
    )
    parser.add_argument(
        "--anim-dir",
        type=Path,
        default=DEFAULT_ANIM_DIR,
        help="GIF 输出目录",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="弹窗显示动画（需 GUI）",
    )
    parser.add_argument("--dpi", type=int, default=120, help="动画渲染 DPI")
    args = parser.parse_args()

    if args.cases:
        selected = []
        for name in args.cases:
            if name not in SCENARIOS_BY_NAME:
                print(f"未知用例: {name}", file=sys.stderr)
                return 2
            selected.append(SCENARIOS_BY_NAME[name])
    else:
        selected = (
            SCENARIOS
            if args.tag == "all"
            else [s for s in SCENARIOS if s.category == args.tag]
        )

    if args.list:
        print(f"共 {len(selected)} 个场景")
        if args.tag != "all":
            print(f"筛选: tag={args.tag}")
        if args.cases:
            print(f"筛选: cases={', '.join(args.cases)}")
        print()
        categories = ("lattice", "decider", "on_lane", "stress", "overtake", "lane_change")
        for cat in categories:
            items = [s for s in selected if s.category == cat]
            if not items:
                continue
            print(f"  [{cat}] ({len(items)})")
            for s in items:
                mark = "ℹ" if s.informational else ("✓" if s.expect_ok else "✗")
                print(f"    {mark} {s.name:36s} {s.description}")
        print("\n动画: --animate 场景 GIF（0.1s/帧）")
        return 0

    print(f"Lattice 场景测试 | tag={args.tag} | 共 {len(selected)} 项")
    print(f"backup 默认: {config.FLAGS_enable_backup_trajectory}")
    if args.animate:
        print(f"场景动画 dt={args.animate_dt}s → {args.anim_dir}")
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
            print("\n动画需要 matplotlib pillow: pip install matplotlib pillow", file=sys.stderr)
            print(f"  ({exc})", file=sys.stderr)
            return 3

    for out in outcomes:
        print_outcome(out)

    passed = sum(1 for o in outcomes if o.passed)
    failed = [o.scenario.name for o in outcomes if not o.passed]
    print(f"\n合计: {passed}/{len(outcomes)} 通过", end="")
    if failed:
        print(f" | 未通过: {', '.join(failed)}", end="")
    print()
    return 0 if passed == len(outcomes) else 1


if __name__ == "__main__":
    raise SystemExit(main())
