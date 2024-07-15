from typing import List
from common.DiscretizedTrajectory import DiscretizedTrajectory
from common.PathPoint import PathPoint
from common.curve1d.Curve1d import Curve1d
from common.TrajectoryPoint import TrajectoryPoint
import PathMatcher
import CartesianFrenetConverter
from config import FLAGS_numerical_epsilon, FLAGS_trajectory_time_length, FLAGS_trajectory_time_resolution
import math
class TrajectoryCombiner:
    """
    TrajectoryCombiner class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

    @staticmethod
    def Combine(reference_line: List[PathPoint], lon_trajectory: Curve1d, lat_trajectory: Curve1d, init_relative_time: float) -> DiscretizedTrajectory:
        """
        Combine the longitudinal and lateral trajectory to form a complete trajectory

        :param List[PathPoint] reference_line: the reference line
        :param Curve1d lon_trajectory: the longitudinal trajectory
        :param Curve1d lat_trajectory: the lateral trajectory
        :param float relative_time: the relative time
        :returns: the combined trajectory
        :rtype: DiscretizedTrajectory
        """

        combined_trajectory = DiscretizedTrajectory()
        
        s0: float = lon_trajectory.Evaluate(0, 0.0)
        s_ref_max: float = reference_line[-1].s
        accumulated_trajectory_s: float = 0.0
        prev_trajectory_point: PathPoint = None

        last_s: float = -FLAGS_numerical_epsilon
        t_param: float = 0.0
        while t_param < FLAGS_trajectory_time_length:
            # linear extrapolation is handled internally in LatticeTrajectory1d;
            # no worry about t_param > lon_trajectory.ParamLength() situation
            s: float = lon_trajectory.Evaluate(0, t_param)
            if last_s > 0.0:
                s = max(last_s, s)
            last_s = s
        
            s_dot: float = max(FLAGS_numerical_epsilon, lon_trajectory.Evaluate(1, t_param))
            s_ddot: float = lon_trajectory.Evaluate(2, t_param)
            if s > s_ref_max:
                break

            relative_s: float = s - s0
            # linear extrapolation is handled internally in LatticeTrajectory1d;
            # no worry about s_param > lat_trajectory.ParamLength() situation
            d: float = lat_trajectory.Evaluate(0, relative_s)
            d_prime: float = lat_trajectory.Evaluate(1, relative_s)
            d_pprime: float = lat_trajectory.Evaluate(2, relative_s)

            matched_ref_point: PathPoint = PathMatcher.MatchToPath(reference_line, s)

            x: float = 0.0
            y: float = 0.0
            theta: float = 0.0
            kappa: float = 0.0
            v: float = 0.0
            a: float = 0.0

            rs: float = matched_ref_point.s
            rx: float = matched_ref_point.x
            ry: float = matched_ref_point.y
            rtheta: float = matched_ref_point.theta
            rkappa: float = matched_ref_point.kappa
            rdkappa: float = matched_ref_point.dkappa

            s_conditions: List[float] = [rs, s_dot, s_ddot]
            d_conditions: List[float] = [d, d_prime, d_pprime]
            x, y, theta, kappa, v, a = CartesianFrenetConverter.frenet_to_cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions)

            if prev_trajectory_point.x is not None and prev_trajectory_point.y is not None:
                delta_x = x - prev_trajectory_point.x
                delta_y = y - prev_trajectory_point.y
                delta_s = math.hypot(delta_x, delta_y)
                accumulated_trajectory_s += delta_s

            trajectory_point = TrajectoryPoint()
            trajectory_point.mutable_path_point.x = x
            trajectory_point.mutable_path_point.y = y
            trajectory_point.mutable_path_point.s = accumulated_trajectory_s
            trajectory_point.mutable_path_point.theta = theta
            trajectory_point.mutable_path_point.kappa = kappa
            trajectory_point.v = v
            trajectory_point.a = a
            trajectory_point.relative_time = t_param + init_relative_time

            combined_trajectory.AppendTrajectoryPoint(trajectory_point)

            t_param += FLAGS_trajectory_time_resolution

            prev_trajectory_point = trajectory_point.path_point

        return combined_trajectory
