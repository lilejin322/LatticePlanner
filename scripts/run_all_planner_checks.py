#!/usr/bin/env python3
"""Run all planner smoke / regression scripts in sequence."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable

SCRIPTS = [
    "run_lattice_component_checks.py",
    "run_minimal_lattice_plan.py",
    "run_lattice_cycle_checks.py",
    "run_lattice_known_failures.py",
    "run_lattice_synthetic_failures.py",
]


COMMANDS = [
    PROJECT_ROOT / "scripts" / script for script in SCRIPTS
] + [
    PROJECT_ROOT / "verification" / name
    for name in (
        "pinned_sampling_gaps.py",
        "verify_backup_jerk.py",
        "verify_extrapolation.py",
        "verify_cartesian_jerk.py",
    )
]


def main() -> int:
    for path in COMMANDS:
        print(f"\n=== {path.name} ===", flush=True)
        result = subprocess.run([PYTHON, str(path)], cwd=str(PROJECT_ROOT))
        if result.returncode != 0:
            print(f"{path.name} failed with code {result.returncode}", file=sys.stderr)
            return result.returncode

    print("\nall planner checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
