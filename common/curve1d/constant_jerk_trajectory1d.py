"""
Constant jerk trajectory1D submodule
"""
from typing import override
from logging import Logger
import config as config_module
from common.curve1d.curve1d import Curve1d

logger = Logger("ConstantJerkTrajectory1d")

class ConstantJerkTrajectory1d(Curve1d):
    """
    ConstantJerkTrajectory1d class
    """
    _p0: float
    _v0: float
    _a0: float
    _jerk: float
    _param: float
    _p1: float
    _v1: float
    _a1: float

    def __init__(self, p0: float, v0: float, a0: float, j: float, param: float) -> None:
        """
        Constructor

        :param float p0: p0
        :param float v0: v0
        :param float a0: a0
        :param float jerk: jerk
        :param float param: param
        """
        super().__init__()
        assert param > config_module.FLAGS_numerical_epsilon, "param should be positive"
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
    def __str__(self) -> str:
        """
        Convert the trajectory to a string

        :returns: The trajectory as a string representation
        :rtype: str
        """
        return f"{self._p0}\t{self._v0}\t{self._a0}\t{self._jerk}\t{self._param}\n"

    def start_position(self) -> float:
        """
        Get the start position

        :returns: the start position
        :rtype: float
        """
        return self._p0

    def start_velocity(self) -> float:
        """
        Get the start velocity

        :returns: the start velocity
        :rtype: float
        """
        return self._v0

    def start_acceleration(self) -> float:
        """
        Get the start acceleration

        :returns: the start acceleration
        :rtype: float
        """
        return self._a0

    def end_position(self) -> float:
        """
        Get the end position

        :returns: the end position
        :rtype: float
        """
        return self._p1

    def end_velocity(self) -> float:
        """
        Get the end velocity

        :returns: the end velocity
        :rtype: float
        """
        return self._v1

    def end_acceleration(self) -> float:
        """
        Get the end acceleration

        :returns: the end acceleration
        :rtype: float
        """
        return self._a1

    def jerk(self) -> float:
        """
        Get the jerk

        :returns: the jerk
        :rtype: float
        """
        return self._jerk
