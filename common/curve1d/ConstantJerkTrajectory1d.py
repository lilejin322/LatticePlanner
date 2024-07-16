from common.curve1d.Curve1d import Curve1d
from typing import override, List
from config import FLAGS_numerical_epsilon
class ConstantJerkTrajectory1d(Curve1d):
    """
    ConstantJerkTrajectory1d class
    """

    def __init__(self, p0: float, v0: float, a0: float, j: float, param: float):
        """
        Constructor

        :param float p0: p0
        :param float v0: v0
        :param float a0: a0
        :param float jerk: jerk
        :param float param: param
        """

        super().__init__()
        assert param > FLAGS_numerical_epsilon, "param should be positive"
        self._p0: float = p0
        self._v0: float = v0
        self._a0: float = a0
        self._jerk: float = j
        self._param: float = param
        self._p1: float = self.Evaluate(0, self._param)
        self._v1: float = self.Evaluate(1, self._param)
        self._a1: float = self.Evaluate(2, self._param)

    @override
    def Evaluate(self, order: int, param: float) -> float:
        """
        Evaluate the trajectory at param by order

        :param int order: order
        :param float param: param
        :returns: the evaluated result
        :rtype: float
        """

        if order == 0:
            return self._p0 + self._v0 * param + 0.5 * self._a0 * param * param + self._jerk * param * param * param / 6.0
        elif order == 1:
            return self._v0 + self._a0 * param + 0.5 * self._jerk * param * param
        elif order == 2:
            return self._a0 + self._jerk * param
        elif order == 3:
            return self._jerk
        else:
            return 0.0

    @override
    def ParamLength(self) -> float:
        """
        Get the param

        :returns: the param
        :rtype: float
        """

        return self._param

    @override
    def ToString(self) -> str:
        """
        Convert the trajectory to a string

        :returns: The trajectory as a string representation
        :rtype: str
        """

        return ""

    def start_position(self) -> float:
        """
        Get the start position
        """

        return self._p0

    def start_velocity(self) -> float:
        """
        Get the start velocity
        """

        return self._v0

    def start_acceleration(self) -> float:
        """
        Get the start acceleration
        """

        return self._a0

    def end_position(self) -> float:
        """
        Get the end position
        """

        return self._p1

    def end_velocity(self) -> float:
        """
        Get the end velocity
        """

        return self._v1

    def end_acceleration(self) -> float:
        """
        Get the end acceleration
        """

        return self._a1

    def jerk(self) -> float:
        """
        Get the jerk
        """

        return self._jerk
