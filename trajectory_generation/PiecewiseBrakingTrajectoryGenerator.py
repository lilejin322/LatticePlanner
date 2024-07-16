from common.curve1d.Curve1d import Curve1d
from common.curve1d.PiecewiseAccelerationTrajectory1d import PiecewiseAccelerationTrajectory1d
import math

class PiecewiseBrakingTrajectoryGenerator:
    """
    PiecewiseBrakingTrajectoryGenerator class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

    @staticmethod
    def Generate(s_target: float, s_curr: float, v_target: float, v_curr: float, 
                 a_comfort: float, d_comfort: float, max_time: float) -> Curve1d:
        """
        Generate a piecewise braking trajectory

        :param s_target: target distance
        :param s_curr: current distance
        :param v_target: target velocity
        :param v_curr: current velocity
        :param a_comfort: comfortable acceleration
        :param d_comfort: comfortable deceleration
        :param max_time: maximum time to generate the trajectory
        :returns: generated piecewise braking trajectory
        :rtype: Curve1d
        """

        trajectory: PiecewiseAccelerationTrajectory1d = PiecewiseAccelerationTrajectory1d(s_curr, v_curr)

        s_dist: float = s_target - s_curr

        comfort_stop_dist: float = PiecewiseBrakingTrajectoryGenerator.ComputeStopDistance(v_curr, d_comfort)

        # if cannot stop using comfort deceleration, then brake in the beginning.
        if comfort_stop_dist > s_dist:
            stop_d: float = PiecewiseBrakingTrajectoryGenerator.ComputeStopDeceleration(s_dist, v_curr)
            stop_t: float = v_curr / stop_d
            trajectory.AppendSegment(-stop_d, stop_t)

            if trajectory.ParamLength < max_time:
                trajectory.AppendSegment(0.0, max_time - trajectory.ParamLength)
            return trajectory

        # otherwise, the vehicle can stop from current speed with comfort brake.
        if v_curr > v_target:
            t_cruise: float = (s_dist - comfort_stop_dist) / v_target
            t_rampdown: float = (v_curr - v_target) / d_comfort
            t_dec: float = v_target / d_comfort

            trajectory.AppendSegment(-d_comfort, t_rampdown)
            trajectory.AppendSegment(0.0, t_cruise)
            trajectory.AppendSegment(-d_comfort, t_dec)

            if trajectory.ParamLength < max_time:
                trajectory.AppendSegment(0.0, max_time - trajectory.ParamLength)
            return trajectory

        else:
            t_rampup: float = (v_target - v_curr) / a_comfort
            t_rampdown: float = (v_target - v_curr) / d_comfort
            s_ramp: float = (v_curr + v_target) * (t_rampup + t_rampdown) * 0.5

            s_rest: float = s_dist - s_ramp - comfort_stop_dist
            if s_rest > 0:
                t_cruise: float = s_rest / v_target
                t_dec: float = v_target / d_comfort

                # construct the trajectory
                trajectory.AppendSegment(a_comfort, t_rampup)
                trajectory.AppendSegment(0.0, t_cruise)
                trajectory.AppendSegment(-d_comfort, t_dec)

                if trajectory.ParamLength < max_time:
                    trajectory.AppendSegment(0.0, max_time - trajectory.ParamLength)
                return trajectory
            else:
                s_rampup_rampdown: float = s_dist - comfort_stop_dist
                v_max: float = math.sqrt(v_curr * v_curr + 2.0 * a_comfort * d_comfort *
                                         s_rampup_rampdown / (a_comfort + d_comfort))

                t_acc: float = (v_max - v_curr) / a_comfort
                t_dec: float = v_max / d_comfort

                # construct the trajectory
                trajectory.AppendSegment(a_comfort, t_acc)
                trajectory.AppendSegment(-d_comfort, t_dec)

                if trajectory.ParamLength < max_time:
                    trajectory.AppendSegment(0.0, max_time - trajectory.ParamLength)
                return trajectory

    @staticmethod
    def ComputeStopDistance(v: float, dec: float) -> float:
        """
        Compute the distance required to stop the vehicle from a given speed with a given deceleration

        :param v: current speed
        :param dec: deceleration
        :returns: distance required to stop the vehicle
        :rtype: float
        """

        assert dec > 0.0, "deceleration should be positive"
        return v * v / dec * 0.5

    @staticmethod
    def ComputeStopDeceleration(dist: float, v: float) -> float:
        """
        Compute the deceleration required to stop the vehicle from a given speed within a given distance

        :param dist: distance required to stop the vehicle
        :param v: current speed
        :returns: deceleration required to stop the vehicle
        :rtype: float
        """
        
        return v * v / dist * 0.5
