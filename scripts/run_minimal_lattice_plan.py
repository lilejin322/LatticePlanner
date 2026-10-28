#!/usr/bin/env python3
"""Run a minimal lattice planning smoke test on a straight CommonRoad lanelet."""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config as config_module
from lanelet_path import path_from_lanelet, straight_lanelet
from lattice_planner import LatticePlanner
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint


def main() -> int:
    path = path_from_lanelet(straight_lanelet(100.0, 2.0, 2.0), config_module.FLAGS_default_cruise_speed)
    planning_start_point = TrajectoryPoint(
        path_point=PathPoint(
            x=0.0,
            y=0.0,
            theta=0.0,
            kappa=0.0,
            s=0.0,
            dkappa=0.0,
        ),
        v=0.1,
        a=0.0,
        relative_time=0.0,
    )
    ok, trajectory, _traj_type = LatticePlanner().PlanOnLanelet(
        planning_start_point,
        [],
        path.points,
        path.lane_width,
        path.speed_at(0.0),
    )
    if not ok or trajectory is None or len(trajectory) == 0:
        print("minimal lattice plan failed", file=sys.stderr)
        return 1
    print("minimal lattice plan succeeded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
