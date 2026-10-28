#!/usr/bin/env python3
"""Straight-road lattice defects found by one-cycle search.

Each case is a finite initial state on a straight lanelet. The script passes
while the signature is still present. When a guard contains one, set that
row's expected kind to ``contained``.
"""

from __future__ import annotations

import logging
import math
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config
from common.constraint_checker import ConstraintChecker
from common.constraint_checker1d import ConstraintChecker1d
from common.curve1d.constant_deceleration_trajectory1d import ConstantDecelerationTrajectory1d
from lanelet_path import path_from_lanelet, straight_lanelet
from lattice_motion import obstacle_from_samples
from lattice_planner import ComputeInitFrenetState, LatticePlanner
from path_matcher import PathMatcher
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint

CONTAINED = "contained"
FALLBACK_BLOWUP = "fallback_blowup"
NEGATIVE_SPEED_ASSERT = "negative_speed_assert"
BACKUP_JERK_BLIND = "backup_jerk_blind"


@dataclass(frozen=True)
class SyntheticDefect:
    name: str
    kind: str
    note: str


RECORDED = (
    SyntheticDefect(
        "stopped_heading",
        FALLBACK_BLOWUP,
        "v=0 and heading 1 rad stay finite in Frenet, then backup returns y around 1e9",
    ),
    SyntheticDefect(
        "reverse_heading",
        NEGATIVE_SPEED_ASSERT,
        "heading pi makes Frenet speed negative and raises AssertionError",
    ),
    SyntheticDefect(
        "backup_stop_jerk",
        BACKUP_JERK_BLIND,
        "backup deceleration reports jerk 0, and the combined samples jump by 40 m/s^3",
    ),
)


def _road():
    return path_from_lanelet(straight_lanelet(300.0, 2.0, 2.0), config.FLAGS_default_cruise_speed)


def _start(v: float, theta: float) -> TrajectoryPoint:
    return TrajectoryPoint(
        path_point=PathPoint(x=0.0, y=0.0, theta=theta, kappa=0.0, s=0.0, dkappa=0.0),
        v=v,
        a=0.0,
        relative_time=0.0,
    )


def _plan(start: TrajectoryPoint, obstacles):
    path = _road()
    return LatticePlanner().PlanOnLanelet(
        start,
        obstacles,
        path.points,
        path.lane_width,
        path.speed_at(0.0),
    )


def observe_stopped_heading() -> tuple[str, str]:
    path = _road()
    start = _start(0.0, 1.0)
    matched = PathMatcher.MatchToPath(path.points, 0.0, 0.0)
    init_s, init_d = ComputeInitFrenetState(matched, start)
    if abs(init_s[1]) > 1.0 or abs(init_d[1]) > 2.0:
        return CONTAINED, f"init itself left the finite range s'={init_s[1]} d'={init_d[1]}"
    ok, trajectory, traj_type = _plan(start, [])
    if not ok or trajectory is None or len(trajectory) == 0:
        return CONTAINED, "plan returned no trajectory"
    result = ConstraintChecker.ValidTrajectory(trajectory)
    point = trajectory[0]
    lateral = abs(point.path_point.y or 0.0)
    speed = abs(point.v or 0.0)
    if (
        traj_type.name == "PATH_FALLBACK"
        and result == ConstraintChecker.Result.LON_VELOCITY_OUT_OF_BOUND
        and lateral > 1.0e5
        and speed > 1000.0
    ):
        return FALLBACK_BLOWUP, (
            f"init d'={init_d[1]:.3f}, fallback point y={point.path_point.y:.3g} v={point.v:.3g}, "
            f"checker {result.name}"
        )
    return CONTAINED, f"{traj_type.name} checker {result.name} y={point.path_point.y} v={point.v}"


def observe_reverse_heading() -> tuple[str, str]:
    try:
        _plan(_start(1.0, math.pi), [])
    except AssertionError as exc:
        if "non-negative" in str(exc):
            return NEGATIVE_SPEED_ASSERT, f"{type(exc).__name__}: {exc}"
        return CONTAINED, f"{type(exc).__name__}: {exc}"
    except Exception as exc:
        return CONTAINED, f"{type(exc).__name__}: {exc}"
    return CONTAINED, "plan returned instead of raising"


def observe_backup_stop_jerk() -> tuple[str, str]:
    curve = ConstantDecelerationTrajectory1d(0.0, 5.0, -4.0)
    if not ConstraintChecker1d.IsValidLongitudinalTrajectory(curve):
        return CONTAINED, "the 1d checker now rejects the backup deceleration"
    stop = curve.ParamLength()
    if curve.Evaluate(3, 1.2) != 0.0 or curve.Evaluate(2, stop) != 0.0:
        return CONTAINED, "the backup curve no longer reports a zero jerk across the stop"
    if abs(curve.Evaluate(2, 1.2) - (-4.0)) > 1.0e-9:
        return CONTAINED, "the -4 m/s^2 piece is no longer the sample before the stop"

    obstacle = obstacle_from_samples(1, 4.8, 2.0, [(0.0, 8.0, 0.0, 0.0, 0.0)])
    ok, trajectory, traj_type = _plan(_start(5.0, 0.0), [obstacle])
    if not ok or trajectory is None:
        return CONTAINED, "plan returned no trajectory"
    result = ConstraintChecker.ValidTrajectory(trajectory)
    worst = 0.0
    when = None
    for left, right in zip(trajectory, trajectory[1:]):
        dt = (right.relative_time or 0.0) - (left.relative_time or 0.0)
        if dt <= 1.0e-9:
            continue
        jerk = ((right.a or 0.0) - (left.a or 0.0)) / dt
        if jerk > worst:
            worst = jerk
            when = left.relative_time
    if (
        traj_type.name == "PATH_FALLBACK"
        and result == ConstraintChecker.Result.LON_JERK_OUT_OF_BOUND
        and abs(worst - 40.0) < 1.0e-6
    ):
        return BACKUP_JERK_BLIND, (
            f"1d checker accepts jerk 0 at the stop T={stop:.2f}; "
            f"combined samples jump {worst:.0f} m/s^3 at t={when:.2f}"
        )
    return CONTAINED, f"{traj_type.name} checker {result.name} worst jerk {worst}"


OBSERVERS = {
    "stopped_heading": observe_stopped_heading,
    "reverse_heading": observe_reverse_heading,
    "backup_stop_jerk": observe_backup_stop_jerk,
}


def main() -> int:
    logging.disable(logging.CRITICAL)
    mismatched = 0
    for defect in RECORDED:
        kind, detail = OBSERVERS[defect.name]()
        status = "RECORDED" if kind == defect.kind else "CHANGED"
        if kind != defect.kind:
            mismatched += 1
        print(f"[{status}] {defect.name}")
        print(f"  expected {defect.kind}: {defect.note}")
        print(f"  actual   {kind}: {detail}")
    if mismatched:
        print(
            f"\n{mismatched} synthetic defect(s) changed. "
            f"Set a fixed case's expected kind to {CONTAINED!r} once a guard holds it.",
            file=sys.stderr,
        )
        return 1
    print("\nsynthetic lattice defects still match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
