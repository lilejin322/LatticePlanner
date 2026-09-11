#!/usr/bin/env python3
"""Z3 check for sampling blind spots in ConstraintChecker.ValidTrajectory.

Unlike ConstraintChecker1d.IsValidLongitudinalTrajectory (see
verify_constraint_checker1d.py), ConstraintChecker.ValidTrajectory checks the
combined Cartesian trajectory (lattice_planner.py's actual gate before a
trajectory is accepted -- see common/constraint_checker.py, called from
lattice_planner.py). Its v/a/kappa checks are done point-by-point on a
DiscretizedTrajectory produced by TrajectoryCombiner.Combine, which samples
the underlying lon/lat polynomial curves at a fixed time step
(FLAGS_trajectory_time_resolution). The same class of bug applies: a genuine
bound violation can sit strictly between two samples and never get checked.

This script re-derives the frenet_to_cartesian formulas (cartesian_frenet_converter.py)
as closed-form functions of time and asks Z3 whether v/a/kappa/lat_a violate
their bounds ANYWHERE on the continuous interval the real checker only
samples -- using exact rational arithmetic, no floating-point resampling.

Scope / modeling assumptions (read before trusting a "no counterexamples found"
result):
  - The reference line is modeled as an exactly straight line (theta=kappa=0,
    x=s, y=0). PathMatcher's linear interpolation reproduces an already-linear
    field exactly, so this is not an approximation -- frenet_to_cartesian's
    rkappa/rdkappa/rtheta terms are genuinely zero here, which is what lets the
    v/a/kappa formulas reduce to a closed form. A curved reference line is out
    of scope.
  - Candidates are restricted to trajectories whose longitudinal velocity
    provably (checked by a separate Z3 query, not just sampled) stays strictly
    positive over the whole checked window. This keeps two clamps in
    TrajectoryCombiner.Combine -- `s = max(last_s, s)` and
    `s_dot = max(FLAGS_numerical_epsilon, s_dot)` -- as no-ops, so the
    Cartesian expressions below match the real combiner exactly rather than
    approximating around a clamp discontinuity.
  - Only the direct-value checks (lon_v, lon_a, kappa, lat_a) are modeled.
    ConstraintChecker.ValidTrajectory's lon_jerk/lat_jerk checks are a finite
    difference between two adjacent discrete samples, not a continuous
    function of one variable -- extending this technique there requires
    differentiating through the sqrt(1+d'^2) term via the chain rule, which is
    saved for a follow-up rather than rushed here.

Run: python3 verification/verify_constraint_checker.py
"""
from __future__ import annotations

import random
import sys
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Optional

import z3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config as config_module  # noqa: E402
from common.constraint_checker import ConstraintChecker  # noqa: E402
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d  # noqa: E402
from common.curve1d.quintic_polynomial_curve1d import QuinticPolynomialCurve1d  # noqa: E402
from protoclass.path_point import PathPoint  # noqa: E402
from trajectory_generation.trajectory_combiner import TrajectoryCombiner  # noqa: E402

SOLVER_TIMEOUT_MS = 15_000
REFERENCE_LINE_LENGTH = 1.0e6


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


def _poly_expr(curve, order: int, x_expr: z3.ArithRef) -> z3.ArithRef:
    """Evaluate(order, x_expr) as a Z3 expression, for any PolynomialCurve1d."""

    base = [curve.Coef(i) for i in range(curve.Order() + 1)]
    coeffs = []
    for i in range(order, len(base)):
        mult = 1
        for k in range(order):
            mult *= i - k
        coeffs.append(base[i] * mult)
    if not coeffs:
        coeffs = [0.0]
    return _horner(coeffs, x_expr)


def _not_within(expr: z3.ArithRef, lower: float, upper: float) -> z3.BoolRef:
    """Negation of ConstraintChecker.WithinRange (no fuzz epsilon, unlike ConstraintChecker1d)."""

    return z3.Or(expr < _z3_real(lower), expr > _z3_real(upper))


def _straight_reference_line() -> list[PathPoint]:
    """A perfectly straight reference line: theta=kappa=0, x=s, y=0.

    PathMatcher.InterpolateUsingLinearApproximation linearly interpolates x/y
    and reproduces theta/kappa unchanged between two points, so for these
    already-linear/constant fields two endpoints reproduce the true value at
    every intermediate s exactly -- this is not a discretization approximation.
    """

    return [
        PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
        PathPoint(
            x=REFERENCE_LINE_LENGTH, y=0.0, z=0.0, theta=0.0, kappa=0.0,
            s=REFERENCE_LINE_LENGTH, dkappa=0.0, ddkappa=0.0,
        ),
    ]


def _velocity_always_positive(lon_curve, eps: float, t_hi: float) -> bool:
    """Proves (via Z3, not sampling) that lon_curve's velocity stays > eps on [0, t_hi].

    This is the precondition that makes TrajectoryCombiner.Combine's
    `s_dot = max(eps, ...)` clamp (and the consequent `s = max(last_s, s)`
    monotonic clamp) a no-op, which is what the closed-form expressions below
    assume.
    """

    t = z3.Real("t")
    v_expr = _poly_expr(lon_curve, 1, t)
    solver = z3.Solver()
    solver.set("timeout", SOLVER_TIMEOUT_MS)
    solver.add(t >= 0, t <= _z3_real(t_hi))
    solver.add(v_expr <= _z3_real(eps))
    return solver.check() == z3.unsat


def _build_cartesian_exprs(lon_curve, lat_curve, t: z3.ArithRef):
    """Closed-form v(t)/a(t)/kappa(t)/lat_a(t), straight-reference-line specialization
    of CartesianFrenetConverter.frenet_to_cartesian.
    """

    s_expr = _poly_expr(lon_curve, 0, t)
    s_dot_expr = _poly_expr(lon_curve, 1, t)
    s_ddot_expr = _poly_expr(lon_curve, 2, t)
    relative_s_expr = s_expr - _z3_real(lon_curve.Evaluate(0, 0.0))

    d_prime_expr = _poly_expr(lat_curve, 1, relative_s_expr)
    d_pprime_expr = _poly_expr(lat_curve, 2, relative_s_expr)

    # w = sqrt(1 + d'^2) = 1 / cos_delta_theta, introduced as an existentially
    # quantified witness so the whole query stays polynomial (no native sqrt).
    w = z3.FreshReal("w")
    w_constraint = z3.And(w > 0, w * w == 1 + d_prime_expr * d_prime_expr)

    kappa_expr = d_pprime_expr / (w * w * w)
    v_expr = s_dot_expr * w
    a_expr = s_ddot_expr * w + (s_dot_expr * s_dot_expr * d_prime_expr * d_pprime_expr) / w
    lat_a_expr = v_expr * v_expr * kappa_expr

    return w_constraint, v_expr, a_expr, kappa_expr, lat_a_expr


def find_continuous_violation(lon_curve, lat_curve, t_hi: float) -> Optional[float]:
    t = z3.Real("t")
    w_constraint, v_expr, a_expr, kappa_expr, lat_a_expr = _build_cartesian_exprs(lon_curve, lat_curve, t)

    violation = z3.Or(
        _not_within(v_expr, config_module.FLAGS_speed_lower_bound, config_module.FLAGS_speed_upper_bound),
        _not_within(
            a_expr,
            config_module.FLAGS_longitudinal_acceleration_lower_bound,
            config_module.FLAGS_longitudinal_acceleration_upper_bound,
        ),
        _not_within(kappa_expr, -config_module.FLAGS_kappa_bound, config_module.FLAGS_kappa_bound),
        _not_within(
            lat_a_expr,
            -config_module.FLAGS_lateral_acceleration_bound,
            config_module.FLAGS_lateral_acceleration_bound,
        ),
    )

    solver = z3.Solver()
    solver.set("timeout", SOLVER_TIMEOUT_MS)
    solver.add(t >= 0, t <= _z3_real(t_hi))
    solver.add(w_constraint)
    solver.add(violation)
    if solver.check() != z3.sat:
        return None
    return float(solver.model()[t].as_fraction())


def _eval_cartesian_floats(lon_curve, lat_curve, t: float):
    """Float re-implementation of _build_cartesian_exprs, for human-readable printouts."""

    import math

    s = lon_curve.Evaluate(0, t)
    s_dot = lon_curve.Evaluate(1, t)
    s_ddot = lon_curve.Evaluate(2, t)
    relative_s = s - lon_curve.Evaluate(0, 0.0)
    d_prime = lat_curve.Evaluate(1, relative_s)
    d_pprime = lat_curve.Evaluate(2, relative_s)

    w = math.sqrt(1.0 + d_prime * d_prime)
    kappa = d_pprime / (w ** 3)
    v = s_dot * w
    a = s_ddot * w + (s_dot * s_dot * d_prime * d_pprime) / w
    lat_a = v * v * kappa
    return v, a, kappa, lat_a


@dataclass
class Candidate:
    kind: str
    lon_curve: object
    lat_curve: object


def generate_candidates(num_random: int, seed: int, t_hi: float):
    rng = random.Random(seed)
    v_lo = max(0.5, config_module.FLAGS_speed_lower_bound)
    v_hi = config_module.FLAGS_speed_upper_bound * 0.8
    a_lo = config_module.FLAGS_longitudinal_acceleration_lower_bound
    a_hi = config_module.FLAGS_longitudinal_acceleration_upper_bound

    for _ in range(num_random):
        v0 = rng.uniform(v_lo, v_hi)
        a0 = rng.uniform(a_lo, a_hi)
        v1 = rng.uniform(v_lo, v_hi)
        a1 = rng.uniform(a_lo, a_hi)
        duration = t_hi + rng.uniform(0.0, 2.0)

        d0 = rng.uniform(-1.0, 1.0)
        d0p = rng.uniform(-0.3, 0.3)
        d0pp = rng.uniform(-0.05, 0.05)
        d1 = rng.uniform(-1.0, 1.0)
        d1p = rng.uniform(-0.3, 0.3)
        d1pp = rng.uniform(-0.05, 0.05)
        lat_param = rng.uniform(20.0, 250.0)
        lat_curve = QuinticPolynomialCurve1d(d0, d0p, d0pp, d1, d1p, d1pp, lat_param)

        s1 = rng.uniform(v0 * duration * 0.5, max(1.0, v_hi * duration))
        yield Candidate("quintic", QuinticPolynomialCurve1d(0.0, v0, a0, s1, v1, a1, duration), lat_curve)

        lat_curve_2 = QuinticPolynomialCurve1d(d0, d0p, d0pp, d1, d1p, d1pp, lat_param)
        yield Candidate("quartic", QuarticPolynomialCurve1d(0.0, v0, a0, v1, a1, duration), lat_curve_2)


def main() -> None:
    num_random = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    t_hi = config_module.FLAGS_trajectory_time_length
    reference_line = _straight_reference_line()

    tested = 0
    checker_said_valid = 0
    skipped_out_of_scope = 0
    inconclusive = 0
    counterexamples = []

    for cand in generate_candidates(num_random, seed, t_hi):
        tested += 1
        combined = TrajectoryCombiner.Combine(reference_line, cand.lon_curve, cand.lat_curve, 0.0)
        if ConstraintChecker.ValidTrajectory(combined) != ConstraintChecker.Result.VALID:
            continue
        checker_said_valid += 1

        if not _velocity_always_positive(cand.lon_curve, config_module.FLAGS_numerical_epsilon, t_hi):
            skipped_out_of_scope += 1
            continue

        t_star = find_continuous_violation(cand.lon_curve, cand.lat_curve, t_hi)
        if t_star is not None:
            counterexamples.append((cand, t_star))

    print(
        f"tested={tested} checker_said_valid={checker_said_valid} "
        f"skipped_out_of_scope={skipped_out_of_scope} counterexamples={len(counterexamples)}"
    )
    for cand, t_star in counterexamples[:20]:
        v, a, kappa, lat_a = _eval_cartesian_floats(cand.lon_curve, cand.lat_curve, t_star)
        print(
            f"[{cand.kind}] lon.start={cand.lon_curve.start_condition} lon.end={cand.lon_curve.end_condition} "
            f"lon.T={cand.lon_curve.ParamLength():.4f} lat.start={cand.lat_curve.start_condition} "
            f"lat.end={cand.lat_curve.end_condition} lat.T={cand.lat_curve.ParamLength():.4f} "
            f"-> checker says VALID, but at t={t_star:.6f}: v={v:.4f} a={a:.4f} kappa={kappa:.6f} lat_a={lat_a:.4f}"
        )


if __name__ == "__main__":
    main()
