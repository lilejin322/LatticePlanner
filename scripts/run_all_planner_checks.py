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
    "run_extended_planner_checks.py",
    "run_minimal_lattice_plan.py",
]


def main() -> int:
    for script in SCRIPTS:
        path = PROJECT_ROOT / "scripts" / script
        print(f"\n=== {script} ===")
        result = subprocess.run([PYTHON, str(path)], cwd=str(PROJECT_ROOT))
        if result.returncode != 0:
            print(f"{script} failed with code {result.returncode}", file=sys.stderr)
            return result.returncode

    print("\nall planner checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
