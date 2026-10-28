#!/usr/bin/env python3
"""Two curves the 0.1 s constraint checker accepts and a denser sample does not.

The numbers are fixed. ``verify_constraint_checker1d.py`` and
``verify_lateral_constraint1d.py`` still search at random; this script only
replays the misses already written down. It passes while each curve is still
missed. When a checker change rejects the curve, or the interior point falls
back inside the bound, the script fails so the fixture can be updated.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config
from common.constraint_checker1d import ConstraintChecker1d
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d
from common.curve1d.quintic_polynomial_curve1d import QuinticPolynomialCurve1d


def _outside(value: float, lower: float, upper: float, epsilon: float = 1.0e-4) -> bool:
    return value <= lower - epsilon or value >= upper + epsilon


def _sampled_times(duration: float):
    t = 0.0
    step = config.FLAGS_trajectory_time_resolution
    while t < duration:
        yield t
        t += step


def check_longitudinal_tail() -> None:
    """Quartic printed from the seed-0 search. The tail after t=1.8 s is unchecked."""
    curve = QuarticPolynomialCurve1d([0.0, 30.93, 2.39], [30.57, -3.38], 1.8694)
    assert ConstraintChecker1d.IsValidLongitudinalTrajectory(curve)
    jerk_low = config.FLAGS_longitudinal_jerk_lower_bound
    jerk_high = config.FLAGS_longitudinal_jerk_upper_bound
    for t in _sampled_times(curve.ParamLength()):
        jerk = curve.Evaluate(3, t)
        assert not _outside(jerk, jerk_low, jerk_high), t
    t_star = 1.84375
    jerk = curve.Evaluate(3, t_star)
    assert _outside(jerk, jerk_low, jerk_high)
    assert jerk < jerk_low
    print(
        f"[quartic] T={curve.ParamLength()} checker says VALID, "
        f"but at t={t_star}: j={jerk:.4f}"
    )


def check_lateral_between_samples() -> None:
    """Quartic/quintic pair from ``verify_lateral_constraint1d.py 200 1``."""
    lon = QuarticPolynomialCurve1d(
        0.0,
        9.816185596580286,
        1.2784884989932177,
        15.381347717187586,
        -0.14727685306482385,
        0.5536301603060695,
    )
    lat = QuinticPolynomialCurve1d(
        -0.7638569389386689,
        0.2472652346880435,
        0.018114835907258403,
        0.0,
        0.0,
        0.0,
        40.0,
    )
    assert ConstraintChecker1d.IsValidLateralTrajectory(lat, lon)
    bound = config.FLAGS_lateral_acceleration_bound
    for t in _sampled_times(lon.ParamLength()):
        s = lon.Evaluate(0, t)
        if s >= lat.ParamLength():
            continue
        accel = lat.Evaluate(2, s) * lon.Evaluate(1, t) ** 2 + lat.Evaluate(1, s) * lon.Evaluate(2, t)
        assert not _outside(accel, -bound, bound), t
    t_star = 0.53125
    s = lon.Evaluate(0, t_star)
    accel = lat.Evaluate(2, s) * lon.Evaluate(1, t_star) ** 2 + lat.Evaluate(1, s) * lon.Evaluate(2, t_star)
    assert _outside(accel, -bound, bound)
    assert accel < -bound
    print(
        f"[quartic] T={lon.ParamLength():.4f} S={lat.ParamLength():.0f} "
        f"checker says VALID, but at t={t_star}: a={accel:.4f}"
    )


def main() -> int:
    check_longitudinal_tail()
    check_lateral_between_samples()
    print("pinned sampling gaps still missed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
