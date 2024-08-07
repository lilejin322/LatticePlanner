from common.curve1d.Curve1d import Curve1d
from common.curve1d.ConstantJerkTrajectory1d import ConstantJerkTrajectory1d
from typing import override, List
from config import FLAGS_numerical_epsilon
from bisect import bisect_left

class PiecewiseJerkTrajectory1d(Curve1d):
    """
    PiecewiseJerkTrajectory1d class
    """

    def __init__(self, p: float, v: float, a: float):
        """
        Constructor

        :param float p: p
        :param float v: v
        :param float a: a
        """

        super().__init__()
        self._segments: List[ConstantJerkTrajectory1d] = []
        self._last_p: float = p
        self._last_v: float = v
        self._last_a: float = a
        self._param: List[float] = [0.0]

    @override
    def Evaluate(self, order: int, param: float) -> float:
        """
        Evaluate the trajectory at a given order and parameter

        :param int order: The order
        :param float param: The parameter
        :returns: The evaluated value
        :rtype: float
        """

        index = bisect_left(self._param, param)

        if index == 0:
            return self._segments[0].Evaluate(order, param)
        elif index == len(self._param):
            index = max(0, len(self._param) - 2)
            return self._segments[-1].Evaluate(order, param - self._param[index])
        else:
            return self._segments[index - 1].Evaluate(order, param - self._param[index - 1])

    @override
    def ParamLength(self) -> float:
        """
        Get the param

        :returns: the param
        :rtype: float
        """

        return self._param[-1]

    @override
    def __str__(self) -> str:
        """
        Convert the trajectory to a string

        :returns: The string representation of the trajectory
        :rtype: str
        """

        return ""

    def AppendSegment(self, jerk: float, param: float) -> None:
        """
        Append a segment to the trajectory

        :param float jerk: The jerk
        :param float param: The parameter
        """

        assert param > FLAGS_numerical_epsilon

        self._param.append(self._param[-1] + param)

        self._segments.append(ConstantJerkTrajectory1d(self._last_p, self._last_v, self._last_a, jerk, param))

        self._last_p = self._segments[-1].end_position
        self._last_v = self._segments[-1].end_velocity
        self._last_a = self._segments[-1].end_acceleration
