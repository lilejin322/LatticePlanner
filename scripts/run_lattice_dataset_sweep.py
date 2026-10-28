#!/usr/bin/env python3
"""Sweep fetched CommonRoad scenarios and classify one lattice cycle each.

This is a discovery run, not a pass/fail gate. Scenario XML stays in
``data/commonroad-scenarios/``. The report is written under
``scripts/output/lattice_sweep/``, which is gitignored.
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
from scripts.lattice_probe import EXCEPTION, probe_file

REPORT_DIR = PROJECT_ROOT / "scripts" / "output" / "lattice_sweep"
DYNAMIC_TAG = re.compile(r"<dynamicObstacle\b")


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


def run_one(path_str: str) -> dict:
    warnings.filterwarnings("ignore")
    logging.disable(logging.CRITICAL)
    path = Path(path_str)
    started = time.perf_counter()
    record = {"name": path.stem, "path": path_str, "group": _group(path)}
    try:
        probe = probe_file(path)
        record["probe"] = probe.kind
        record["probe_detail"] = probe.detail
        record["trajectory_type"] = probe.trajectory_type
        record["primary"] = probe.kind
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
    parser.add_argument("--limit", type=int, default=0, help="Stop after this many files (0 means all selected)")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output", type=Path, default=REPORT_DIR / "report.jsonl")
    args = parser.parse_args()

    files = _select(args.max_dynamic)
    if args.limit > 0:
        files = files[: args.limit]
    print(
        f"scenarios {len(files)} max_dynamic {args.max_dynamic} workers {args.workers}",
        flush=True,
    )
    rows = []
    with ProcessPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = [pool.submit(run_one, str(path)) for path in files]
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
