from typing import override, Any, List
from common.curve1d.Curve1d import Curve1d
from config import FLAGS_numerical_epsilon
from bisect import bisect_left
from logging import Logger

class PiecewiseAccelerationTrajectory1d(Curve1d):
    """
    PiecewiseAccelerationTrajectory1d class
    We understand that this class has too many code duplication,
    but we are following the original code structure.
    """

    def __init__(self, start_s: float, start_v: float):
        """
        Constructor

        :param float start_s: The start s
        :param float start_v: The start v
        """

        super().__init__()
        self._s = [start_s]
        self._v: List[float] = [start_v]
        self._a: List[float] = [0.0]
        self._t: List[float] = [0.0]
        self.logger = Logger("PiecewiseAccelerationTrajectory1d")

    def AppendSegment(self, a: float, t_duration: float) -> None:
        """
        Append a segment to the trajectory

        :param float a: The acceleration
        :param float t_duration: The duration
        """

        s0: float = self.s[-1]
        v0: float = self.v[-1]
        t0: float = self.t[-1]

        v1: float = v0 + a * t_duration
        assert v1 >= -FLAGS_numerical_epsilon, "Final velocity must be greater than or equal to -numerical_epsilon"

        delta_s: float = (v0 + v1) * t_duration * 0.5
        s1: float = s0 + delta_s
        t1: float = t0 + t_duration

        assert s1 >= s0 - FLAGS_numerical_epsilon, "Final position must be greater than or equal to initial position minus numerical_epsilon"
        s1: float = max(s1, s0)
        self._s.append(s1)
        self._v.append(v1)
        self._a.append(a)
        self._t.append(t1)

    def PopSegment(self) -> None:
        """
        Pop the last segment
        """

        if len(self._a) > 0:
            self._a.pop()
            self._v.pop()
            self._s.pop()
            self._t.pop()

    @override
    def ParamLength(self) -> float:
        """
        Get the ParamLength

        :returns: The ParamLength
        :rtype: float
        """

        assert len(self._t) > 1, "Trajectory length must be greater than 1"
        return self._t[-1] - self._t[0]

    @override
    def ToString(self) -> str:
        """
        Convert the trajectory to a string

        :returns: The trajectory as a string representation
        :rtype: str
        """
        
        return "\t".join(map(str, self._s)) + "\n" + \
               "\t".join(map(str, self._t)) + "\n" + \
               "\t".join(map(str, self._v)) + "\n" + \
               "\t".join(map(str, self._a)) + "\n"

    def lerp(self, x0: float, t0: float, x1: float, t1: float, t: float) -> float:
        """
        Linear interpolation.

        :param float x0: the x0 value.
        :param float t0: the t0 value.
        :param float x1: the x1 value.
        :param float t1: the t1 value.
        :param float t: the t value.
        :returns: the interpolated value.
        :rtype: float
        """

        if abs(t1 - t0) <= 1.0e-6:
            self.logger.error("Input time difference is too small")
            return x0
        r = (t - t0) / (t1 - t0)
        x = x0 + r * (x1 - x0)
        return x

    @override
    def Evaluate(self, *args) -> Any:

        if len(args) == 2:
            order, param = args
            assert len(self._t) > 1, "Trajectory length must be greater than 1"
            assert param >= self._t[0] and param <= self._t[-1], "Param must be within the trajectory time range"

            if order == 0:
                return self.Evaluate_s(param)
            elif order == 1:
                return self.Evaluate_v(param)
            elif order == 2:
                return self.Evaluate_a(param)
            elif order == 3:
                return self.Evaluate_j(param)
            else:
                return 0.0

        elif len(args) == 1:
            t = args[0]
            assert len(self._t) > 1, "Trajectory length must be greater than 1"
            index: int = bisect_left(self._t, t)

            s0: float = self._s[index - 1]
            v0: float = self._v[index - 1]
            t0: float = self._t[index - 1]

            v1: float = self._v[index]
            t1: float = self._t[index]

            v: float = self.lerp(v0, t0, v1, t1, t)
            s: float = (v0 + v) * (t - t0) * 0.5 + s0

            a: float = self._a[index - 1]
            j: float = 0.0

            return s, v, a, j

    def Evaluate_s(self, t: float) -> float:
        """
        Evaluate s at time t

        :param float t: The time
        :returns: The evaluation of s at time t
        :rtype: float
        """

        index: int = bisect_left(self._t, t)
        
        s0: float = self._s[index - 1]
        v0: float = self._v[index - 1]
        t0: float = self._t[index - 1]

        v1: float = self._v[index]
        t1: float = self._t[index]

        v: float = self.lerp(v0, t0, v1, t1, t)
        s: float = (v0 + v) * (t - t0) * 0.5 + s0
        return s

    def Evaluate_v(self, t: float) -> float:
        """
        Evaluate v at time t

        :param float t: The time
        :returns: The evaluation of v at time t
        :rtype: float
        """

        index: int = bisect_left(self._t, t)

        v0: float = self._v[index - 1]
        t0: float = self._t[index - 1]

        v1: float = self._v[index]
        t1: float = self._t[index]

        v: float = self.lerp(v0, t0, v1, t1, t)
        return v

    def Evaluate_a(self, t: float) -> float:
        """
        Evaluate a at time t

        :param float t: The time
        :returns: The evaluation of a at time t
        :rtype: float
        """

        index: int = bisect_left(self._t, t)
        return self._a[index - 1]

    def Evaluate_j(self, t: float) -> float:
        """
        Evaluate j at time t

        :param float t: The time
        :returns: The evaluation of j at time t
        :rtype: float
        """

        return 0.0
