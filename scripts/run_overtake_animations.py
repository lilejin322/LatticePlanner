#!/usr/bin/env python3
"""一键：全部超车场景测试 + 场景动画 GIF。"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(PROJECT_ROOT / "scripts" / "output" / "mpl-cache"))


def main() -> int:
    extra_args = sys.argv[1:]
    cmd = [
        sys.executable,
        str(PROJECT_ROOT / "scripts" / "run_lattice_scenario_cases.py"),
        "--tag",
        "overtake",
    ]
    if "--list" not in extra_args:
        cmd.append("--animate")
    cmd.extend(extra_args)
    print("运行:", " ".join(cmd), "\n", flush=True)
    return subprocess.call(cmd, cwd=str(PROJECT_ROOT))


if __name__ == "__main__":
    raise SystemExit(main())
