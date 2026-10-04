#!/usr/bin/env python3
"""
Closed-loop lattice simulations.

Unlike ``run_lattice_scenario_cases.py``, these cases replan every planning
cycle and step both ego and dynamic actors forward. They fail if planning
fails or the executed prefix overlaps an actor.

Usage:
  python scripts/run_closed_loop_sim.py
  python scripts/run_closed_loop_sim.py open_road follow_leader
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.closed_loop_sim import Actor, SimResult, run_straight_road


def open_road() -> SimResult:
    """Empty road. Ego should keep a normal lattice plan and move forward."""
    result = run_straight_road(
        "open_road",
        Actor("ego", x=0.0, v=5.0),
        [],
        horizon_s=3.0,
    )
    if result.ok and result.fallback_cycles:
        result.ok = False
        result.reason = "open road fell back to the backup trajectory"
    traveled = result.ego_x1 - result.ego_x0
    if result.ok and traveled < 12.0:
        result.ok = False
        result.reason = f"open road only traveled {traveled:.1f}m"
    return result


def follow_leader() -> SimResult:
    """Faster ego behind a slower moving leader. Must follow without overlap."""
    result = run_straight_road(
        "follow_leader",
        Actor("ego", x=0.0, v=8.0),
        [Actor("lead_1", x=28.0, v=4.0)],
        horizon_s=3.0,
    )
    if result.ok and (result.min_clearance is None or result.min_clearance <= 0.0):
        result.ok = False
        result.reason = "follow gap closed"
    return result


def stopped_leader() -> SimResult:
    """Stopped car ahead. Backup may engage; the executed path must not hit it."""
    result = run_straight_road(
        "stopped_leader",
        Actor("ego", x=0.0, v=6.0),
        [Actor("lead_1", x=22.0, v=0.0, is_static=True)],
        horizon_s=3.0,
        blocking_actor_id="lead_1",
        backup=True,
    )
    if result.ok and (result.min_clearance is None or result.min_clearance <= 0.0):
        result.ok = False
        result.reason = "stopped leader clearance closed"
    return result


CASES = {
    "open_road": open_road,
    "follow_leader": follow_leader,
    "stopped_leader": stopped_leader,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Closed-loop lattice simulation")
    parser.add_argument("cases", nargs="*", help="Case names (default: all)")
    args = parser.parse_args()
    names = args.cases or list(CASES)
    unknown = [name for name in names if name not in CASES]
    if unknown:
        print(f"Unknown case: {', '.join(unknown)}", file=sys.stderr)
        return 2

    failed = []
    for name in names:
        result = CASES[name]()
        status = "PASS" if result.ok else "FAIL"
        print(f"[{status}] {name}")
        print(f"  {result.summary()}")
        if not result.ok:
            failed.append(name)
    print(f"\nTotal: {len(names) - len(failed)}/{len(names)} passed")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
