"""
Constraint checker 1d submodule
"""
import config as config_module
from logging import Logger
from common.curve1d.curve1d import Curve1d

logger = Logger("ConstraintChecker1d")


def _sample_times(param_length: float):
    """Fixed-step samples on [0, param_length], always including the true end.

    The historical ``while t < length: t += dt`` loop never evaluated a duration
    that is not an exact multiple of dt, so a bound violation in the final
    partial step was invisible.
    """
    if param_length < 0.0:
        return []
    dt = config_module.FLAGS_trajectory_time_resolution
    times = []
    t = 0.0
    while t < param_length:
        times.append(t)
        t += dt
    if not times or times[-1] < param_length - 1e-9:
        times.append(param_length)
    return times


def _real_roots_in_open_interval(coeffs_low_to_high, t0: float, t1: float):
    import numpy as np

    coeffs = [float(c) for c in coeffs_low_to_high]
    while len(coeffs) > 1 and abs(coeffs[-1]) <= 1e-12:
        coeffs.pop()
    if len(coeffs) <= 1 or t1 <= t0:
        return []
    roots = np.atleast_1d(np.roots(coeffs[::-1]))
    found = []
    for root in roots:
        if abs(float(np.imag(root))) > 1e-7:
            continue
        value = float(np.real(root))
        if t0 + 1e-9 < value < t1 - 1e-9:
            found.append(value)
    return found


def _polynomial_critical_times(curve: Curve1d):
    """Times in (0, T) where v, a, or j of a polynomial Curve1d can peak.

    Velocity extrema are roots of acceleration, acceleration extrema are roots
    of jerk, and jerk extrema are roots of snap. Together with the endpoints,
    that covers the continuous bounds for quartic and quintic curves.
    """
    if not hasattr(curve, "Coef") or not hasattr(curve, "Order"):
        return []
    order = int(curve.Order())
    if order < 2:
        return []
    coef = [float(curve.Coef(i)) for i in range(order + 1)]
    while len(coef) < 6:
        coef.append(0.0)
    _c0, _c1, c2, c3, c4, c5 = coef[:6]
    duration = float(curve.ParamLength())
    times = []
    times.extend(_real_roots_in_open_interval([24.0 * c4, 120.0 * c5], 0.0, duration))
    times.extend(_real_roots_in_open_interval([6.0 * c3, 24.0 * c4, 60.0 * c5], 0.0, duration))
    times.extend(_real_roots_in_open_interval([2.0 * c2, 6.0 * c3, 12.0 * c4, 20.0 * c5], 0.0, duration))
    return times

class ConstraintChecker1d:
    """
    ConstraintChecker1d class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

    def __new__(cls):
        """
        In case of instantiation, raise an error
        """
        if cls is ConstraintChecker1d:
            raise TypeError("ConstraintChecker1d class cannot be instantiated")
        return super().__new__(cls)

    @staticmethod
    def fuzzy_within(v: float, lower: float, upper: float, e:float=1.0e-4) -> bool:
        """
        fuzzy_within function

        :param float v: v
        :param float lower: lower
        :param float upper: upper
        :param float e: e, default 1.0e-4
        :returns: fuzzy_within result
        :rtype: bool
        """
        return lower - e < v < upper + e

    @staticmethod
    def IsValidLongitudinalTrajectory(lon_trajectory: Curve1d) -> bool:
        """
        IsValidLongitudinalTrajectory function

        :param Curve1d lon_trajectory: lon_trajectory
        :returns: IsValidLongitudinalTrajectory result
        :rtype: bool
        """
        times = _sample_times(lon_trajectory.ParamLength())
        times.extend(_polynomial_critical_times(lon_trajectory))
        for t in times:
            v: float = lon_trajectory.Evaluate(1, t)    # evaluate_v
            if not ConstraintChecker1d.fuzzy_within(
                v, config_module.FLAGS_speed_lower_bound, config_module.FLAGS_speed_upper_bound
            ):
                return False
            a: float = lon_trajectory.Evaluate(2, t)    # evaluate_a
            if not ConstraintChecker1d.fuzzy_within(
                a,
                config_module.FLAGS_longitudinal_acceleration_lower_bound,
                config_module.FLAGS_longitudinal_acceleration_upper_bound,
            ):
                return False
            j: float = lon_trajectory.Evaluate(3, t)    # evaluate_j
            if not ConstraintChecker1d.fuzzy_within(
                j,
                config_module.FLAGS_longitudinal_jerk_lower_bound,
                config_module.FLAGS_longitudinal_jerk_upper_bound,
            ):
                return False
        return True

    @staticmethod   
    def IsValidLateralTrajectory(lat_trajectory: Curve1d, lon_trajectory: Curve1d) -> bool:
        """
        IsValidLateralTrajectory function

        :param Curve1d lat_trajectory: lat_trajectory
        :param Curve1d lon_trajectory: lon_trajectory
        :returns: IsValidLateralTrajectory result
        :rtype: bool
        """
        for t in _sample_times(lon_trajectory.ParamLength()):
            s: float = lon_trajectory.Evaluate(0, t)
            dd_ds: float = lat_trajectory.Evaluate(1, s)
            ds_dt: float = lon_trajectory.Evaluate(1, t)

            d2d_ds2: float = lat_trajectory.Evaluate(2, s)
            d2s_dt2: float = lon_trajectory.Evaluate(2, t)

            a: float = 0.0
            if s < lat_trajectory.ParamLength():
                a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2

            if not ConstraintChecker1d.fuzzy_within(
                a,
                -config_module.FLAGS_lateral_acceleration_bound,
                config_module.FLAGS_lateral_acceleration_bound,
            ):
                return False
            
            # this is not accurate, just an approximation...
            j: float = 0.0
            if s < lat_trajectory.ParamLength():
                j = lat_trajectory.Evaluate(3, s) * lon_trajectory.Evaluate(3, t)
            if not ConstraintChecker1d.fuzzy_within(
                j,
                -config_module.FLAGS_lateral_jerk_bound,
                config_module.FLAGS_lateral_jerk_bound,
            ):
                return False
        return True
