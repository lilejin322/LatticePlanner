#!/usr/bin/env python3
"""Z3 proof that a 1d-valid quartic leaves the speed bound after it ends.

``ConstraintChecker1d`` samples only while ``t < ParamLength()``.
``LatticeTrajectory1d`` then extrapolates with constant acceleration, and
``TrajectoryCombiner`` keeps evaluating out to 8 s. The quartic below is
accepted on [0, 2). At t=2 its speed is 35 m/s and its acceleration is
4 m/s², so the extrapolated speed at 8 s is 59 m/s.
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
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d  # noqa: E402
from trajectory_generation.lattice_trajectory1d import CreateLatticeTrajectory1d  # noqa: E402

# (s0, v0, a0, v1, a1, duration). End acceleration is the upper bound, so the
# 1d loop, which stops strictly before the duration, can still accept it.
PINNED = (0.0, 30.0, 1.0, 35.0, 4.0, 2.0)


def _real(value: float) -> z3.ArithRef:
    numerator, denominator = Fraction(value).as_integer_ratio()
    return z3.RealVal(numerator) / z3.RealVal(denominator)


def _derivative(coefs: list[float], order: int, t: z3.ArithRef) -> z3.ArithRef:
    terms: dict[int, float] = {}
    for index in range(order, len(coefs)):
        multiplier = 1
        for step in range(order):
            multiplier *= index - step
        terms[index - order] = coefs[index] * multiplier
    highest = max(terms)
    expr = _real(terms[highest])
    for power in range(highest - 1, -1, -1):
        expr = expr * t + _real(terms.get(power, 0.0))
    return expr


def _samples_are_inside(coefs: list[float], duration: float) -> None:
    """Unsat: no 0.1 s sample before the duration is outside a fuzzy bound."""

    epsilon = 1.0e-4
    bounds = (
        (1, config.FLAGS_speed_lower_bound, config.FLAGS_speed_upper_bound),
        (
            2,
            config.FLAGS_longitudinal_acceleration_lower_bound,
            config.FLAGS_longitudinal_acceleration_upper_bound,
        ),
        (3, config.FLAGS_longitudinal_jerk_lower_bound, config.FLAGS_longitudinal_jerk_upper_bound),
    )
    pieces = []
    t = z3.Real("t")
    step = 0.0
    while step < duration:
        sample = _real(step)
        outside = []
        for order, lower, upper in bounds:
            value = _derivative(coefs, order, sample)
            outside.append(z3.Or(value <= _real(lower - epsilon), value >= _real(upper + epsilon)))
        pieces.append(z3.And(t == sample, z3.Or(*outside)))
        step += config.FLAGS_trajectory_time_resolution
    solver = z3.Solver()
    solver.add(z3.Or(*pieces))
    if solver.check() != z3.unsat:
        raise AssertionError("a 0.1 s sample of the pinned quartic is outside a 1d bound")


def _extrapolated_speed_leaves_the_bound(coefs: list[float], duration: float) -> float:
    horizon = _real(config.FLAGS_trajectory_time_length)
    stop = _real(duration)
    t = z3.Real("t")
    speed = _derivative(coefs, 1, stop) + _derivative(coefs, 2, stop) * (t - stop)
    solver = z3.Solver()
    solver.add(t > stop, t <= horizon, speed > _real(config.FLAGS_speed_upper_bound))
    if solver.check() != z3.sat:
        raise AssertionError("extrapolated speed stays inside the bound")
    model_t = solver.model()[t]
    return float(model_t.as_fraction())


def main() -> int:
    _s0, v0, a0, v1, a1, duration = PINNED
    curve = QuarticPolynomialCurve1d(0.0, v0, a0, v1, a1, duration)
    if not ConstraintChecker1d.IsValidLongitudinalTrajectory(curve):
        raise AssertionError("1d checker no longer accepts the pinned quartic")
    coefs = [curve.Coef(index) for index in range(5)]
    _samples_are_inside(coefs, duration)
    witness_t = _extrapolated_speed_leaves_the_bound(coefs, duration)
    wrapped = CreateLatticeTrajectory1d(curve)
    speed_at_horizon = wrapped.Evaluate(1, config.FLAGS_trajectory_time_length)
    if speed_at_horizon <= config.FLAGS_speed_upper_bound:
        raise AssertionError(f"wrapper speed at the horizon is {speed_at_horizon}")
    print(
        f"proved quartic T={duration} end v={v1} a={a1}: 1d samples are inside, "
        f"extrapolated speed exceeds {config.FLAGS_speed_upper_bound} by t={witness_t:.2f}, "
        f"wrapper speed at {config.FLAGS_trajectory_time_length:.0f}s is {speed_at_horizon:.0f}"
    )
    print("extrapolation speed proved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
