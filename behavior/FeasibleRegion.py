from typing import List
import math
from config import FLAGS_longitudinal_acceleration_upper_bound, FLAGS_longitudinal_acceleration_lower_bound

class FeasibleRegion:
    """
    FeasibleRegion class
    """

    def __init__(self, init_s: List[float]):
        """
        Constructor

        :param List[float] init_s: initial state
        """

        self.init_s_: List[float] = init_s
        v: float = init_s[1]
        assert v >= 0.0, "Velocity should be non-negative"
        max_deceleration = -FLAGS_longitudinal_acceleration_lower_bound
        self.t_at_zero_speed_: float = v / max_deceleration
        self.s_at_zero_speed_: float = init_s[0] + v * v / (2.0 * max_deceleration)

    def SUpper(self, t: float) -> float:
        """
        Get upper bound of s at time t

        :param float t: time
        :returns: upper bound of s at time t
        :rtype: float
        """

        assert t >= 0.0, "Time should be non-negative"
        return (self.init_s_[0] + self.init_s_[1] * t +
                0.5 * FLAGS_longitudinal_acceleration_upper_bound * t * t)

    def SLower(self, t: float) -> float:
        """
        Get lower bound of s at time t

        :param float t: time
        :returns: lower bound of s at time t
        :rtype: float
        """

        if t < self.t_at_zero_speed_:
            return (self.init_s_[0] + self.init_s_[1] * t +
                    0.5 * FLAGS_longitudinal_acceleration_lower_bound * t * t)
        return self.s_at_zero_speed_

    def VUpper(self, t: float) -> float:
        """
        Get upper bound of v at time t

        :param float t: time
        :returns: upper bound of v at time t
        :rtype: float
        """
        return self.init_s_[1] + FLAGS_longitudinal_acceleration_upper_bound * t

    def VLower(self, t: float) -> float:
        """
        Get lower bound of v at time t

        :param float t: time
        :returns: lower bound of v at time t
        :rtype: float
        """
        if t < self.t_at_zero_speed_:
            return self.init_s_[1] + FLAGS_longitudinal_acceleration_lower_bound * t
        return 0.0

    def TLower(self, s: float) -> float:
        """
        Get lower bound of t at s

        :param float s: position
        :returns: lower bound of t at s
        :rtype: float
        """
        assert s >= self.init_s_[0], "Position should be greater than or equal to initial position"

        delta_s: float = s - self.init_s_[0]
        v: float = self.init_s_[1]
        a = FLAGS_longitudinal_acceleration_upper_bound
        t = (math.sqrt(v * v + 2.0 * a * delta_s) - v) / a
        return t
