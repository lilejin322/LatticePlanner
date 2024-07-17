from common.curve1d.Curve1d import Curve1d
from config import FLAGS_speed_lower_bound, FLAGS_speed_upper_bound, \
                   FLAGS_longitudinal_acceleration_lower_bound, FLAGS_longitudinal_acceleration_upper_bound, \
                   FLAGS_longitudinal_jerk_lower_bound, FLAGS_longitudinal_jerk_upper_bound, \
                   FLAGS_trajectory_time_resolution, FLAGS_lateral_acceleration_bound, FLAGS_lateral_jerk_bound

class ConstraintChecker1d:
    """
    ConstraintChecker1d class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

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

        t: float = 0.0
        while t < lon_trajectory.ParamLength():
            v: float = lon_trajectory.Evaluate(1, t)    # evaluate_v
            if not ConstraintChecker1d.fuzzy_within(v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound):
                return False
            a: float = lon_trajectory.Evaluate(2, t)    # evaluate_a
            if not ConstraintChecker1d.fuzzy_within(a, FLAGS_longitudinal_acceleration_lower_bound,
                                                    FLAGS_longitudinal_acceleration_upper_bound):
                return False
            j: float = lon_trajectory.Evaluate(3, t)    # evaluate_j
            if not ConstraintChecker1d.fuzzy_within(j, FLAGS_longitudinal_jerk_lower_bound,
                                                    FLAGS_longitudinal_jerk_upper_bound):
                return False
            t += FLAGS_trajectory_time_resolution
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

        t: float = 0.0
        while t < lon_trajectory.ParamLength():
            s: float = lon_trajectory.Evaluate(0, t)
            dd_ds: float = lat_trajectory.Evaluate(1, s)
            ds_dt: float = lon_trajectory.Evaluate(1, t)

            d2d_ds2: float = lat_trajectory.Evaluate(2, s)
            d2s_dt2: float = lon_trajectory.Evaluate(2, t)

            a: float = 0.0
            if s < lat_trajectory.ParamLength():
                a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2

            if not ConstraintChecker1d.fuzzy_within(a, -FLAGS_lateral_acceleration_bound,
                                                    FLAGS_lateral_acceleration_bound):
                return False
            
            # this is not accurate, just an approximation...
            j: float = 0.0
            if s < lat_trajectory.ParamLength():
                j = lat_trajectory.Evaluate(3, s) * lon_trajectory.Evaluate(3, t)
            if not ConstraintChecker1d.fuzzy_within(j, -FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound):
                return False
            t += FLAGS_trajectory_time_resolution
        return True
