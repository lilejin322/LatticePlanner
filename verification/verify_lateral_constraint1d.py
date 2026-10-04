#!/usr/bin/env python3
"""Z3 check for sampling blind spots in IsValidLateralTrajectory.

ConstraintChecker1d.IsValidLateralTrajectory samples a longitudinal curve
s(t) and a lateral curve l(s) every FLAGS_trajectory_time_resolution (0.1s),
with the same ``while t < ParamLength()`` loop as the longitudinal checker:
the true end time is skipped whenever the duration is not a multiple of 0.1s,
and nothing is evaluated between samples.

The quantity it checks is exactly

    a(t) = l''(s) * s'(t)^2 + l'(s) * s''(t)
    j(t) = l'''(s) * s'''(t)          # the checker's own approximation

and only while s(t) < l.ParamLength(); past that it forces a = j = 0.

This script keeps pairs the real checker already calls valid, then asks Z3
whether that same formula leaves the lateral acceleration/jerk bounds
anywhere on the continuous interval [0, duration]. A sat result is a
trajectory the sampled checker accepts that the checker's own formula
rejects between samples.

Run: python3 verification/verify_lateral_constraint1d.py [num_random] [seed]
"""

from __future__ import annotations

import random
import sys
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path

import z3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config as config_module  # noqa: E402
from common.constraint_checker1d import ConstraintChecker1d  # noqa: E402
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d  # noqa: E402
from common.curve1d.quintic_polynomial_curve1d import QuinticPolynomialCurve1d  # noqa: E402

SOLVER_TIMEOUT_MS = 3_000


def _z3_real(value: float) -> z3.ArithRef:
    num, den = Fraction(value).as_integer_ratio()
    return z3.RealVal(num) / z3.RealVal(den)


def _horner(ascending_coeffs: list[float], x: z3.ArithRef) -> z3.ArithRef:
    expr = _z3_real(ascending_coeffs[-1])
    for c in reversed(ascending_coeffs[:-1]):
        expr = expr * x + _z3_real(c)
    return expr


def _poly_expr(curve, order: int, x_expr: z3.ArithRef) -> z3.ArithRef:
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


def _not_fuzzy_within(expr: z3.ArithRef, lower: float, upper: float, e: float = 1.0e-4) -> z3.BoolRef:
    """Negation of ConstraintChecker1d.fuzzy_within, same epsilon."""

    return z3.Or(expr <= _z3_real(lower - e), expr >= _z3_real(upper + e))


def _lateral_exprs(lon_curve, lat_curve, t: z3.ArithRef):
    """The checker's a(t), j(t), and the s < l.ParamLength() guard."""

    s = _poly_expr(lon_curve, 0, t)
    s_dot = _poly_expr(lon_curve, 1, t)
    s_ddot = _poly_expr(lon_curve, 2, t)
    s_dddot = _poly_expr(lon_curve, 3, t)
    l_prime = _poly_expr(lat_curve, 1, s)
    l_pprime = _poly_expr(lat_curve, 2, s)
    l_ppprime = _poly_expr(lat_curve, 3, s)
    a = l_pprime * s_dot * s_dot + l_prime * s_ddot
    j = l_ppprime * s_dddot
    active = s < _z3_real(lat_curve.ParamLength())
    return active, a, j


def find_intra_sample_violation(lon_curve, lat_curve) -> tuple[str, float | None]:
    t = z3.Real("t")
    active, a, j = _lateral_exprs(lon_curve, lat_curve, t)
    a_bound = config_module.FLAGS_lateral_acceleration_bound
    j_bound = config_module.FLAGS_lateral_jerk_bound
    violation = z3.And(
        active,
        z3.Or(
            _not_fuzzy_within(a, -a_bound, a_bound),
            _not_fuzzy_within(j, -j_bound, j_bound),
        ),
    )
    solver = z3.Solver()
    solver.set("timeout", SOLVER_TIMEOUT_MS)
    solver.add(t >= 0, t <= _z3_real(lon_curve.ParamLength()))
    solver.add(violation)
    status = solver.check()
    if status != z3.sat:
        return str(status), None
    model_t = solver.model()[t]
    if model_t is None:
        return "unknown", None
    return "sat", float(model_t.as_fraction())


def _checker_lateral(lon_curve, lat_curve, t: float) -> tuple[float, float]:
    """Float evaluation of the same a(t), j(t) the checker uses."""

    s = lon_curve.Evaluate(0, t)
    if s >= lat_curve.ParamLength():
        return 0.0, 0.0
    s_dot = lon_curve.Evaluate(1, t)
    s_ddot = lon_curve.Evaluate(2, t)
    s_dddot = lon_curve.Evaluate(3, t)
    a = lat_curve.Evaluate(2, s) * s_dot * s_dot + lat_curve.Evaluate(1, s) * s_ddot
    j = lat_curve.Evaluate(3, s) * s_dddot
    return a, j


@dataclass
class Pair:
    lon_kind: str
    lon: object
    lat: object


def generate_pairs(num_random: int, seed: int):
    """Lon/lat pairs inside the lattice sampler's own end-condition ranges.

    Lateral ends are the planner's {0, ±0.5} at s in {10, 20, 40, 80}, plus a
    wider draw so a violation is not limited to those three offsets. Durations
    are biased short: fewer 0.1s samples means more room between them.
    """

    rng = random.Random(seed)
    v_hi = min(20.0, config_module.FLAGS_speed_upper_bound)
    a_lo = config_module.FLAGS_longitudinal_acceleration_lower_bound
    a_hi = config_module.FLAGS_longitudinal_acceleration_upper_bound
    for _ in range(num_random):
        # Leave a gap after the last 0.1s sample: duration = n*0.1 + (0.01..0.09).
        # That is the same structural hole as the longitudinal checker.
        n_steps = rng.randint(0, 18)
        duration = n_steps * config_module.FLAGS_trajectory_time_resolution + rng.uniform(0.01, 0.09)
        v0 = rng.uniform(0.0, v_hi)
        a0 = rng.uniform(a_lo * 0.5, a_hi * 0.5)
        v1 = rng.uniform(0.0, v_hi)
        a1 = rng.uniform(a_lo * 0.5, a_hi * 0.5)
        s_end = rng.choice([10.0, 20.0, 40.0, 80.0, rng.uniform(8.0, 40.0)])
        d_end = rng.choice([0.0, -0.5, 0.5, rng.uniform(-2.0, 2.0)])
        lat = QuinticPolynomialCurve1d(
            rng.uniform(-1.0, 1.0),
            rng.uniform(-0.5, 0.5),
            rng.uniform(-0.2, 0.2),
            d_end,
            0.0,
            0.0,
            s_end,
        )
        quartic = QuarticPolynomialCurve1d(0.0, v0, a0, v1, a1, duration)
        s1 = rng.uniform(0.0, max(1.0, v0 * duration * 1.3))
        quintic = QuinticPolynomialCurve1d(0.0, v0, a0, s1, v1, a1, duration)
        yield Pair("quartic", quartic, lat)
        yield Pair("quintic", quintic, lat)


def main() -> None:
    num_random = int(sys.argv[1]) if len(sys.argv) > 1 else 400
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    tested = 0
    checker_said_valid = 0
    unknown = 0
    counterexamples = []
    for pair in generate_pairs(num_random, seed):
        tested += 1
        if not ConstraintChecker1d.IsValidLateralTrajectory(pair.lat, pair.lon):
            continue
        checker_said_valid += 1
        status, t_star = find_intra_sample_violation(pair.lon, pair.lat)
        if status != "sat" or t_star is None:
            if status != "unsat":
                unknown += 1
            continue
        counterexamples.append((pair, t_star))

    print(
        f"tested={tested} checker_said_valid={checker_said_valid} "
        f"counterexamples={len(counterexamples)} unresolved={unknown}"
    )
    for pair, t_star in counterexamples[:20]:
        a, j = _checker_lateral(pair.lon, pair.lat, t_star)
        print(
            f"[{pair.lon_kind}] lon_start={pair.lon.start_condition} "
            f"lon_end={pair.lon.end_condition} T={pair.lon.ParamLength():.4f} "
            f"lat_start={pair.lat.start_condition} lat_end={pair.lat.end_condition} "
            f"S={pair.lat.ParamLength():.2f} -> checker says VALID, "
            f"but at t={t_star:.6f}: a={a:.4f} j={j:.4f}"
        )


if __name__ == "__main__":
    main()
