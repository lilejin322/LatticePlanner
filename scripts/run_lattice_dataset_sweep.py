#!/usr/bin/env python3
"""Sweep fetched CommonRoad scenarios and classify lattice outcomes.

This is a discovery run, not a pass/fail gate. Scenario XML stays in
``data/commonroad-scenarios/``. The report is written under
``scripts/output/lattice_sweep/``, which is gitignored.

Traffic collisions and backup cycles are counted apart from numeric blow-ups,
negative-speed assertions, and first points that land far from the ego.
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import sys
import time
import warnings
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.commonroad_scenarios import DATASET_DIR
from scripts.lattice_probe import (
    EXCEPTION,
    FAR_MATCH,
    LEFT_ROAD,
    NEGATIVE_SPEED_ASSERT,
    NUMERIC_BLOWUP,
    probe_file,
)

REPORT_DIR = PROJECT_ROOT / "scripts" / "output" / "lattice_sweep"
DYNAMIC_TAG = re.compile(r"<dynamicObstacle\b")
ACCEL_LOW = -6.0
ACCEL_HIGH = 4.0


def _dynamic_count(path: Path) -> int:
    return len(DYNAMIC_TAG.findall(path.read_text(errors="ignore")))


def _select(max_dynamic: int) -> list[Path]:
    root = DATASET_DIR / "scenarios" / "recorded"
    if not root.is_dir():
        raise FileNotFoundError(
            f"{root} is missing. Run python scripts/fetch_commonroad_scenarios.py"
        )
    chosen = []
    for path in sorted(root.rglob("*.xml")):
        if ".git" in path.parts:
            continue
        if _dynamic_count(path) <= max_dynamic:
            chosen.append(path)
    return chosen


def _executed_accel_violation(result) -> float | None:
    worst = None
    for left, right in zip(result.ego_trace, result.ego_trace[1:]):
        dt = right.t - left.t
        if dt <= 1e-6:
            continue
        accel = (right.v - left.v) / dt
        if accel < ACCEL_LOW - 0.05 or accel > ACCEL_HIGH + 0.05:
            if worst is None or abs(accel) > abs(worst):
                worst = accel
    return worst


def _primary(probe_kind: str, drive: dict) -> str:
    if probe_kind == NEGATIVE_SPEED_ASSERT:
        return NEGATIVE_SPEED_ASSERT
    if probe_kind == NUMERIC_BLOWUP:
        return NUMERIC_BLOWUP
    if probe_kind == FAR_MATCH:
        return FAR_MATCH
    if str(drive.get("reason") or "").startswith("executed state left the road"):
        return LEFT_ROAD
    if probe_kind == EXCEPTION or drive.get("error"):
        return EXCEPTION
    if drive.get("accel") is not None:
        return "accel_bound"
    if drive.get("collided"):
        return "collision"
    if drive.get("plan_failures", 0) > 0:
        return "plan_failure"
    if drive.get("cycles") and drive.get("fallback_cycles") == drive.get("cycles"):
        return "fallback_only"
    if not drive.get("ok", False):
        return "other_failure"
    return "ok"


def run_one(path_str: str, horizon_s: float) -> dict:
    warnings.filterwarnings("ignore")
    logging.disable(logging.CRITICAL)
    path = Path(path_str)
    started = time.perf_counter()
    record = {"name": path.stem, "path": path_str, "group": _group(path)}
    try:
        from scripts.commonroad_drive import load_scenario, run_loaded_scenario

        probe = probe_file(path)
        record["probe"] = probe.kind
        record["probe_detail"] = probe.detail
        if probe.kind == NEGATIVE_SPEED_ASSERT:
            record["primary"] = NEGATIVE_SPEED_ASSERT
            record["seconds"] = time.perf_counter() - started
            return record
        scenario, problems = load_scenario(path)
        result = run_loaded_scenario(
            scenario, planning_problems=problems, horizon_s=horizon_s
        )
        accel = _executed_accel_violation(result)
        drive = {
            "ok": result.ok,
            "reason": result.reason,
            "cycles": len(result.cycles),
            "plan_failures": result.plan_failures,
            "fallback_cycles": result.fallback_cycles,
            "collided": result.collided,
            "clearance": result.min_clearance,
            "accel": accel,
            "error": None,
        }
        record.update(drive)
        record["primary"] = _primary(probe.kind, drive)
    except Exception as exc:
        record["probe"] = EXCEPTION
        record["error"] = f"{type(exc).__name__}: {exc}"
        record["primary"] = EXCEPTION
    record["seconds"] = time.perf_counter() - started
    return record


def _group(path: Path) -> str:
    parts = path.parts
    if "recorded" in parts:
        return parts[parts.index("recorded") + 1]
    return "other"


def main() -> int:
    parser = argparse.ArgumentParser(description="Classify lattice plans on fetched scenarios")
    parser.add_argument("--max-dynamic", type=int, default=8, help="Skip files with more dynamic obstacles")
    parser.add_argument("--horizon", type=float, default=1.0, help="Closed-loop horizon in seconds")
    parser.add_argument("--limit", type=int, default=0, help="Stop after this many files (0 means all selected)")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output", type=Path, default=REPORT_DIR / "report.jsonl")
    args = parser.parse_args()

    files = _select(args.max_dynamic)
    if args.limit > 0:
        files = files[: args.limit]
    print(
        f"scenarios {len(files)} horizon {args.horizon}s "
        f"max_dynamic {args.max_dynamic} workers {args.workers}",
        flush=True,
    )
    rows = []
    with ProcessPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = [pool.submit(run_one, str(path), args.horizon) for path in files]
        for index, future in enumerate(as_completed(futures), start=1):
            rows.append(future.result())
            if index % 40 == 0 or index == len(files):
                print(f"  {index}/{len(files)}", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(json.dumps(row) + "\n" for row in rows))
    counts = Counter(row.get("primary", "?") for row in rows)
    print(f"\nwrote {args.output}")
    for name, count in counts.most_common():
        print(f"  {name:24} {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
