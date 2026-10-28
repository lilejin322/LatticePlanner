#!/usr/bin/env python3
"""Check the lattice defects found on the fetched CommonRoad set.

The check passes while each scenario still shows its recorded defect. After a
guard stops the defect, change that scenario's expected kind to ``contained``
and this script becomes the gate.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.lattice_probe import (
    ACCEL_BOUND,
    CONTAINED,
    LEFT_ROAD,
    NEGATIVE_SPEED_ASSERT,
    NUMERIC_BLOWUP,
    commit_chain,
    probe_scenario,
)


@dataclass(frozen=True)
class RecordedDefect:
    scenario_id: str
    kind: str
    note: str


RECORDED = (
    RecordedDefect(
        "USA_Peach-3_1_T-1",
        NUMERIC_BLOWUP,
        "PATH_FALLBACK first point leaves the 1e5 m guard",
    ),
    RecordedDefect(
        "ESP_Monzon-7_5_T-1",
        NEGATIVE_SPEED_ASSERT,
        "negative Frenet speed raises AssertionError instead of a failed plan",
    ),
    RecordedDefect(
        "ZAM_Tjunction-1_266_T-1",
        LEFT_ROAD,
        "at 0.7s the backup trajectory's committed point jumps off that cycle's init",
    ),
    RecordedDefect(
        "FRA_MoissyCramayel-1_1_T-1",
        ACCEL_BOUND,
        "a later commit changes speed faster than -6 m/s^2 while sample a stays inside",
    ),
)


def main() -> int:
    mismatched = 0
    for defect in RECORDED:
        if defect.kind in (LEFT_ROAD, ACCEL_BOUND):
            observed = commit_chain(defect.scenario_id, horizon_s=1.0)
            kind = observed.kind
            detail = observed.detail
        else:
            probe = probe_scenario(defect.scenario_id)
            kind = probe.kind
            detail = probe.detail
        status = "RECORDED" if kind == defect.kind else "CHANGED"
        if kind != defect.kind:
            mismatched += 1
        print(f"[{status}] {defect.scenario_id}")
        print(f"  expected {defect.kind}: {defect.note}")
        print(f"  actual   {kind}: {detail}")
    if mismatched:
        print(
            f"\n{mismatched} recorded defect(s) changed. "
            f"Set a fixed case's expected kind to {CONTAINED!r} once a guard holds it.",
            file=sys.stderr,
        )
        return 1
    print("\nrecorded lattice defects still match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
