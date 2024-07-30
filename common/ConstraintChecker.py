from enum import Enum
from common.DiscretizedTrajectory import DiscretizedTrajectory
from typing import List, Tuple, Any
from config import FLAGS_trajectory_time_length, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound, \
                   FLAGS_longitudinal_acceleration_lower_bound, FLAGS_longitudinal_acceleration_upper_bound, \
                   FLAGS_kappa_bound, FLAGS_longitudinal_jerk_lower_bound, FLAGS_longitudinal_jerk_upper_bound, \
                   FLAGS_lateral_acceleration_bound, FLAGS_lateral_jerk_bound

from logging import Logger

logger = Logger("ConstraintChecker")

def WithinRange(v: Any, lower: Any, upper: Any) -> bool:
    """
    Check if the value is within the range

    param Any v: the value to be checked
    param Any lower: the lower bound
    param Any upper: the upper bound
    returns: if the value is within the range
    rtype: bool
    """

    return lower <= v <= upper

class ConstraintChecker:
    """
    ConstraintChecker class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

    class Result(Enum):

        VALID = 0
        LON_VELOCITY_OUT_OF_BOUND = 1
        LON_ACCELERATION_OUT_OF_BOUND = 2
        LON_JERK_OUT_OF_BOUND = 3
        LAT_VELOCITY_OUT_OF_BOUND = 4
        LAT_ACCELERATION_OUT_OF_BOUND = 5
        LAT_JERK_OUT_OF_BOUND = 6
        CURVATURE_OUT_OF_BOUND = 7

    def __init__(self):
        """
        This class cannot be instantiated
        """

        raise NotImplementedError("This class cannot be instantiated")

    @staticmethod
    def ValidTrajectory(trajectory: DiscretizedTrajectory) -> Result:
        """
        Check if the trajectory is valid

        param: DiscretizedTrajectory trajectory: the trajectory to be checked
        returns: the result of the check
        rtype: Result
        """

        kMaxCheckRelativeTime = FLAGS_trajectory_time_length
        for p in trajectory:
            t: float = p.relative_time
            if t > kMaxCheckRelativeTime:
                break
            lon_v = p.v
            if not WithinRange(lon_v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound):
                logger.debug(f"Velocity at relative time {t} exceeds bound, value: {lon_v}, bound [{FLAGS_speed_lower_bound}, {FLAGS_speed_upper_bound}].")
                return ConstraintChecker.Result.LON_VELOCITY_OUT_OF_BOUND

            lon_a = p.a
            if not WithinRange(lon_a, FLAGS_longitudinal_acceleration_lower_bound, FLAGS_longitudinal_acceleration_upper_bound):
                logger.debug(f"Longitudinal acceleration at relative time {t} exceeds bound, value: {lon_a}, bound [{FLAGS_longitudinal_acceleration_lower_bound}, {FLAGS_longitudinal_acceleration_upper_bound}].")
                return ConstraintChecker.Result.LON_ACCELERATION_OUT_OF_BOUND

            kappa = p.kappa
            if not WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound):
                logger.debug(f"Kappa at relative time {t} exceeds bound, value: {kappa}, bound [{-FLAGS_kappa_bound}, {FLAGS_kappa_bound}].")
                return ConstraintChecker.Result.CURVATURE_OUT_OF_BOUND

        for i in range(1, trajectory.NumOfPoints()):
            p0 = trajectory.TrajectoryPointAt(i - 1)
            p1 = trajectory.TrajectoryPointAt(i)

            if p1.relative_time > kMaxCheckRelativeTime:
                break

            t = p0.relative_time

            dt = p1.relative_time - p0.relative_time
            d_lon_a = p1.a - p0.a
            lon_jerk = d_lon_a / dt
            if not WithinRange(lon_jerk, FLAGS_longitudinal_jerk_lower_bound, FLAGS_longitudinal_jerk_upper_bound):
                logger.debug(f"Longitudinal jerk at relative time {t} exceeds bound, value: {lon_jerk}, bound [{FLAGS_longitudinal_jerk_lower_bound}, {FLAGS_longitudinal_jerk_upper_bound}].")
                return ConstraintChecker.Result.LON_JERK_OUT_OF_BOUND

            lat_a = p1.v * p1.v * p1.path_point.kappa
            if not WithinRange(lat_a, -FLAGS_lateral_acceleration_bound, FLAGS_lateral_acceleration_bound):
                logger.debug(f"Lateral acceleration at relative time {t} exceeds bound, value: {lat_a}, bound [{-FLAGS_lateral_acceleration_bound}, {FLAGS_lateral_acceleration_bound}].")
                return ConstraintChecker.Result.LAT_ACCELERATION_OUT_OF_BOUND

            # TODO(zhangyajia): this is temporarily disabled
            # due to low quality reference line.
            """
            d_lat_a = p1.v * p1.v * p1.path_point.kappa - p0.v * p0.v * p0.path_point.kappa
            lat_jerk = d_lat_a / dt
            if not WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound):
                logger.debug(f"Lateral jerk at relative time {t} exceeds bound, value: {lat_jerk}, bound [{-FLAGS_lateral_jerk_bound}, {FLAGS_lateral_jerk_bound}].")
                return ConstraintChecker.Result.LAT_JERK_OUT_OF_BOUND
            """

        return ConstraintChecker.Result.VALID
