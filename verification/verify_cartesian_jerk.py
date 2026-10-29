#!/usr/bin/env python3
"""Z3 proof that the Cartesian jerk sample misses the pinned quartic spike.

On a straight reference line with zero lateral offset, Cartesian acceleration
is the quartic's second derivative. ``ConstraintChecker.ValidTrajectory`` does
not read that derivative. It differences acceleration at adjacent 0.1 s
samples. The quartic is wrapped in ``LatticeTrajectory1d``, so past its
duration the acceleration is frozen and the difference from 1.8 s to 1.9 s
stays inside [-4, 2]. The instantaneous jerk at t=1.84375 s does not.
"""

from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

import z3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config  # noqa: E402
from common.constraint_checker import ConstraintChecker  # noqa: E402
from common.constraint_checker1d import ConstraintChecker1d  # noqa: E402
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d  # noqa: E402
from common.curve1d.quintic_polynomial_curve1d import QuinticPolynomialCurve1d  # noqa: E402
from protoclass.path_point import PathPoint  # noqa: E402
from trajectory_generation.lattice_trajectory1d import CreateLatticeTrajectory1d  # noqa: E402
from trajectory_generation.trajectory_combiner import TrajectoryCombiner  # noqa: E402

# Same boundary values as verification/pinned_sampling_gaps.py.
PINNED = (0.0, 30.93, 2.39, 30.57, -3.38, 1.8694)
SPIKE_T = 1.84375
SAMPLE_BEFORE = 1.8
SAMPLE_AFTER = 1.9


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


def _require(solver: z3.Solver, expected: z3.CheckSatResult, message: str) -> None:
    result = solver.check()
    if result != expected:
        raise AssertionError(f"{message}: {result}")


def main() -> int:
    _s0, v0, a0, v1, a1, duration = PINNED
    base = QuarticPolynomialCurve1d(0.0, v0, a0, v1, a1, duration)
    wrapped = CreateLatticeTrajectory1d(base)
    lateral = QuinticPolynomialCurve1d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 400.0)
    if not ConstraintChecker1d.IsValidLongitudinalTrajectory(wrapped):
        raise AssertionError("1d checker no longer accepts the wrapped quartic")
    reference = [
        PathPoint(x=0.0, y=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0),
        PathPoint(x=1.0e6, y=0.0, theta=0.0, kappa=0.0, s=1.0e6, dkappa=0.0),
    ]
    combined = TrajectoryCombiner.Combine(reference, wrapped, lateral, 0.0)
    result = ConstraintChecker.ValidTrajectory(combined)
    if result != ConstraintChecker.Result.VALID:
        raise AssertionError(f"Cartesian checker no longer accepts the wrapped quartic: {result}")

    coefs = [base.Coef(index) for index in range(5)]
    duration_expr = _real(base.ParamLength())
    spike = _derivative(coefs, 3, _real(SPIKE_T))
    frozen = _derivative(coefs, 2, duration_expr)
    before = _derivative(coefs, 2, _real(SAMPLE_BEFORE))
    finite_difference = (frozen - before) / _real(SAMPLE_AFTER - SAMPLE_BEFORE)

    outside = z3.Solver()
    outside.add(spike >= _real(config.FLAGS_longitudinal_jerk_lower_bound - 1.0e-4))
    _require(outside, z3.unsat, "instantaneous jerk is inside the fuzzy bound")

    difference_outside = z3.Solver()
    difference_outside.add(
        z3.Or(
            finite_difference < _real(config.FLAGS_longitudinal_jerk_lower_bound),
            finite_difference > _real(config.FLAGS_longitudinal_jerk_upper_bound),
        )
    )
    _require(difference_outside, z3.unsat, "sample jerk difference leaves the bound")

    value = z3.Real("jerk")
    witness = z3.Solver()
    witness.add(value == spike)
    witness.check()
    jerk = float(witness.model()[value].as_fraction())
    print(
        f"proved wrapped quartic T={base.ParamLength()}: both checkers accept it, "
        f"jerk at t={SPIKE_T} is {jerk:.4f}, and the 1.8s to 1.9s difference stays inside "
        f"[{config.FLAGS_longitudinal_jerk_lower_bound}, {config.FLAGS_longitudinal_jerk_upper_bound}]"
    )
    print("cartesian sample jerk proved blind")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
