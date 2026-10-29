#!/usr/bin/env python3
"""Z3 proof that backup deceleration hides the jerk at the stop.

``BackupTrajectoryGenerator`` builds ``ConstantDecelerationTrajectory1d`` with
accelerations -0.1, -1, -2, -3 and -4 m/s². That curve's ``Evaluate(3, t)`` is
0 for every t, so ``ConstraintChecker1d`` never sees a jerk. Acceleration is
nevertheless -|a| before the stop and 0 at the stop. The combiner's 0.1 s
samples therefore jump by |a|/0.1 across the stop.

For every initial speed in (0, 40] m/s the stop falls inside some sample
interval. The finite-difference jerk on that interval is |a|/0.1. It exceeds
the +2 m/s³ bound for every backup acceleration except -0.1.
"""

from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

import z3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config  # noqa: E402
from common.constraint_checker1d import ConstraintChecker1d  # noqa: E402
from common.curve1d.constant_deceleration_trajectory1d import (  # noqa: E402
    ConstantDecelerationTrajectory1d,
)

# Magnitudes of the accelerations in BackupTrajectoryGenerator.GenerateTrajectory1dPairs.
BACKUP_DECELERATIONS = (0.1, 1.0, 2.0, 3.0, 4.0)
SPEED_LIMIT = 40.0
SAMPLE_DT = 0.1
HORIZON = 8.0


def _real(value: float) -> z3.ArithRef:
    numerator, denominator = Fraction(str(value)).as_integer_ratio()
    return z3.RealVal(numerator) / z3.RealVal(denominator)


def _every_speed_has_a_straddling_sample(deceleration: float) -> None:
    """Negation is unsat: every v0 in (0, 40] has k with k*dt < v0/|a| <= (k+1)*dt."""

    speed = z3.Real("speed")
    step = z3.Int("step")
    stop = speed / _real(deceleration)
    step_dt = _real(SAMPLE_DT)
    solver = z3.Solver()
    solver.add(speed > 0, speed <= _real(SPEED_LIMIT))
    solver.add(
        z3.Not(
            z3.Exists(
                [step],
                z3.And(
                    step >= 0,
                    step <= 4000,
                    step * step_dt < stop,
                    (step + 1) * step_dt >= stop,
                ),
            )
        )
    )
    result = solver.check()
    if result != z3.unsat:
        raise AssertionError(f"straddle proof for |a|={deceleration} returned {result}")


def _witness_matches_the_curve(deceleration: float) -> float:
    """One real curve whose published samples contain the stop jump."""

    speed = min(5.0, deceleration * HORIZON / 2.0)
    curve = ConstantDecelerationTrajectory1d(0.0, speed, -deceleration)
    if not ConstraintChecker1d.IsValidLongitudinalTrajectory(curve):
        raise AssertionError(f"1d checker rejected backup deceleration {deceleration}")
    t = 0.0
    while t < curve.ParamLength():
        if curve.Evaluate(3, t) != 0.0:
            raise AssertionError("backup jerk symbol is no longer identically zero")
        t += SAMPLE_DT
    previous_t = 0.0
    previous_a = curve.Evaluate(2, 0.0)
    t = SAMPLE_DT
    found = None
    while t <= HORIZON + 1e-9:
        acceleration = curve.Evaluate(2, t)
        if previous_a == -deceleration and acceleration == 0.0:
            found = (acceleration - previous_a) / SAMPLE_DT
            break
        previous_t = t
        previous_a = acceleration
        t += SAMPLE_DT
    expected = deceleration / SAMPLE_DT
    if found is None or abs(found - expected) > 1e-9:
        raise AssertionError(
            f"|a|={deceleration} stop at {curve.ParamLength()} produced jerk {found}, expected {expected}"
        )
    del previous_t
    return found


def main() -> int:
    jerk_high = config.FLAGS_longitudinal_jerk_upper_bound
    for deceleration in BACKUP_DECELERATIONS:
        _every_speed_has_a_straddling_sample(deceleration)
        finite_difference = _witness_matches_the_curve(deceleration)
        relation = "exceeds" if finite_difference > jerk_high else "stays inside"
        visible_until = deceleration * HORIZON
        print(
            f"proved |a|={deceleration}: reported jerk is 0 on every 1d sample, "
            f"and every v0 in (0, {SPEED_LIMIT:.0f}] has a stop step whose "
            f"finite-difference jerk is {finite_difference:.0f} m/s^3, which {relation} "
            f"the +{jerk_high:.0f} bound. The 8 s trajectory contains that step when "
            f"v0 <= {visible_until:.1f}"
        )
    print("backup stop jerk proved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
