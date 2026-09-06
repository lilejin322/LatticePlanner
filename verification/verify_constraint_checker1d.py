#!/usr/bin/env python3
"""Z3 check for sampling blind spots in ConstraintChecker1d.

ConstraintChecker1d.IsValidLongitudinalTrajectory only samples a Curve1d at
fixed time steps (FLAGS_trajectory_time_resolution). The underlying curves
(QuinticPolynomialCurve1d, QuarticPolynomialCurve1d) are closed-form
polynomials, so a genuine velocity/acceleration/jerk bound violation can sit
strictly between two samples and never get checked.

For every random candidate curve the real checker calls valid, this script
asks Z3 whether a violation exists ANYWHERE on the continuous interval
[0, duration] -- not just at the sampled points -- using exact rational
arithmetic on the curve's own coefficients (no floating-point resampling).
A SAT result is a concrete counterexample: a trajectory the planner would
treat as safe that actually is not.

Run: python3 verification/verify_constraint_checker1d.py
"""
from __future__ import annotations

import random
import sys
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Callable

import z3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config as config_module  # noqa: E402
from common.constraint_checker1d import ConstraintChecker1d  # noqa: E402
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d  # noqa: E402
from common.curve1d.quintic_polynomial_curve1d import QuinticPolynomialCurve1d  # noqa: E402

Bounds = tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
DerivFn = Callable[[object, z3.ArithRef], tuple[z3.ArithRef, z3.ArithRef, z3.ArithRef]]


def _z3_real(value: float) -> z3.ArithRef:
    """Exact Z3 rational matching a Python float's binary value bit-for-bit."""

    num, den = Fraction(value).as_integer_ratio()
    return z3.RealVal(num) / z3.RealVal(den)


def _horner(ascending_coeffs: list[float], t: z3.ArithRef) -> z3.ArithRef:
    """Build sum(c_i * t**i) via Horner's method, mirroring the curves' own Evaluate()."""

    expr = _z3_real(ascending_coeffs[-1])
    for c in reversed(ascending_coeffs[:-1]):
        expr = expr * t + _z3_real(c)
    return expr


def _quintic_derivative_exprs(curve: QuinticPolynomialCurve1d, t: z3.ArithRef):
    _, c1, c2, c3, c4, c5 = (curve.Coef(i) for i in range(6))
    v = _horner([c1, 2 * c2, 3 * c3, 4 * c4, 5 * c5], t)
    a = _horner([2 * c2, 6 * c3, 12 * c4, 20 * c5], t)
    j = _horner([6 * c3, 24 * c4, 60 * c5], t)
    return v, a, j


def _quartic_derivative_exprs(curve: QuarticPolynomialCurve1d, t: z3.ArithRef):
    _, c1, c2, c3, c4 = (curve.Coef(i) for i in range(5))
    v = _horner([c1, 2 * c2, 3 * c3, 4 * c4], t)
    a = _horner([2 * c2, 6 * c3, 12 * c4], t)
    j = _horner([6 * c3, 24 * c4], t)
    return v, a, j


def _not_fuzzy_within(expr: z3.ArithRef, lower: float, upper: float, e: float = 1.0e-4) -> z3.BoolRef:
    """Negation of ConstraintChecker1d.fuzzy_within, same epsilon and float arithmetic."""

    return z3.Or(expr <= _z3_real(lower - e), expr >= _z3_real(upper + e))


def find_intra_sample_violation(curve, deriv_fn: DerivFn, bounds: Bounds) -> float | None:
    (v_lo, v_hi), (a_lo, a_hi), (j_lo, j_hi) = bounds
    t = z3.Real("t")
    v, a, j = deriv_fn(curve, t)
    violation = z3.Or(
        _not_fuzzy_within(v, v_lo, v_hi),
        _not_fuzzy_within(a, a_lo, a_hi),
        _not_fuzzy_within(j, j_lo, j_hi),
    )

    solver = z3.Solver()
    solver.add(t >= 0, t <= _z3_real(curve.ParamLength()))
    solver.add(violation)
    if solver.check() != z3.sat:
        return None
    return float(solver.model()[t].as_fraction())


@dataclass
class Candidate:
    kind: str
    curve: object


def generate_candidates(num_random: int, seed: int):
    rng = random.Random(seed)
    v_lo = config_module.FLAGS_speed_lower_bound
    v_hi = config_module.FLAGS_speed_upper_bound
    a_lo = config_module.FLAGS_longitudinal_acceleration_lower_bound
    a_hi = config_module.FLAGS_longitudinal_acceleration_upper_bound

    for _ in range(num_random):
        v0 = rng.uniform(max(0.0, v_lo), v_hi)
        a0 = rng.uniform(a_lo, a_hi)
        # Bias toward short durations: fewer samples in the checker's fixed-step
        # loop means more room for a violation to hide between them.
        duration = rng.uniform(config_module.FLAGS_polynomial_minimal_param, 2.5)

        v1 = rng.uniform(max(0.0, v_lo), v_hi)
        a1 = rng.uniform(a_lo, a_hi)
        s1 = rng.uniform(0.0, max(1.0, v_hi * duration * 1.3))
        yield Candidate(
            "quintic",
            QuinticPolynomialCurve1d(0.0, v0, a0, s1, v1, a1, duration),
        )

        v1c = rng.uniform(max(0.0, v_lo), v_hi)
        a1c = rng.uniform(a_lo, a_hi)
        yield Candidate(
            "quartic",
            QuarticPolynomialCurve1d(0.0, v0, a0, v1c, a1c, duration),
        )


def main() -> None:
    num_random = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    bounds: Bounds = (
        (config_module.FLAGS_speed_lower_bound, config_module.FLAGS_speed_upper_bound),
        (
            config_module.FLAGS_longitudinal_acceleration_lower_bound,
            config_module.FLAGS_longitudinal_acceleration_upper_bound,
        ),
        (config_module.FLAGS_longitudinal_jerk_lower_bound, config_module.FLAGS_longitudinal_jerk_upper_bound),
    )
    deriv_fns: dict[str, DerivFn] = {
        "quintic": _quintic_derivative_exprs,
        "quartic": _quartic_derivative_exprs,
    }

    tested = 0
    checker_said_valid = 0
    counterexamples = []
    for cand in generate_candidates(num_random, seed):
        tested += 1
        if not ConstraintChecker1d.IsValidLongitudinalTrajectory(cand.curve):
            continue
        checker_said_valid += 1
        t_star = find_intra_sample_violation(cand.curve, deriv_fns[cand.kind], bounds)
        if t_star is not None:
            counterexamples.append((cand, t_star))

    print(f"tested={tested} checker_said_valid={checker_said_valid} counterexamples={len(counterexamples)}")
    for cand, t_star in counterexamples[:20]:
        v = cand.curve.Evaluate(1, t_star)
        a = cand.curve.Evaluate(2, t_star)
        j = cand.curve.Evaluate(3, t_star)
        print(
            f"[{cand.kind}] start={cand.curve.start_condition} end={cand.curve.end_condition} "
            f"T={cand.curve.ParamLength():.4f} -> checker says VALID, but at t={t_star:.6f}: "
            f"v={v:.4f} a={a:.4f} j={j:.4f}"
        )


if __name__ == "__main__":
    main()
