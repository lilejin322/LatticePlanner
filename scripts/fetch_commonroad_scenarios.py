#!/usr/bin/env python3
"""Clone the public CommonRoad scenario repository into a gitignored directory.

The XML files stay out of this repository. Re-running the script leaves an
existing clone in place.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.commonroad_scenarios import DATASET_BRANCH, DATASET_DIR, DATASET_REPO, scenario_file


def main() -> int:
    if (DATASET_DIR / ".git").is_dir():
        print(f"CommonRoad scenarios already present at {DATASET_DIR}")
    else:
        DATASET_DIR.parent.mkdir(parents=True, exist_ok=True)
        subprocess.check_call(
            [
                "git",
                "clone",
                "--depth",
                "1",
                "--branch",
                DATASET_BRANCH,
                "--single-branch",
                DATASET_REPO,
                str(DATASET_DIR),
            ]
        )
        print(f"Cloned {DATASET_BRANCH} into {DATASET_DIR}")
    road = scenario_file()
    print(f"Closed-loop road: {road}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
