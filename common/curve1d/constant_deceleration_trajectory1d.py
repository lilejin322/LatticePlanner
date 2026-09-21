"""
Constant deceleration trajectory1D submodule
"""
from logging import Logger
from typing import override
import config as config_module
from common.curve1d.curve1d import Curve1d

logger = Logger("ConstantDecelerationTrajectory1d")

class ConstantDecelerationTrajectory1d(Curve1d):
    """
    ConstantDecelerationTrajectory1d class
    """
    _init_s: float
    _deceleration: float
    _init_v: float

    def __init__(self, init_s: float, init_v: float, a: float) -> None:
        """
        Constructor

        :param float init_s: init s
        :param float init_v: init velocity
        :param float a: deceleration value
        """
        super().__init__()
        self._init_s: float = init_s
        self._deceleration: float = -a
        if init_v < -config_module.FLAGS_numerical_epsilon:
            logger.error(f"negative init v = {init_v}")
        self._init_v: float = abs(init_v)
        assert self._deceleration > 0.0, "Deceleration should be positive"
        self._end_t: float = self._init_v / self._deceleration
        self._end_s: float = self._init_v ** 2 / (2.0 * self._deceleration) + self._init_s

    @override
    def ParamLength(self) -> float:
        """
        Get the param, coz this is a cpp style

        :returns: the param
        :rtype: float
        """
        return self._end_t

    @override
    def __str__(self) -> str:
        """
        Convert the trajectory to a string

        :returns: The trajectory as a string representation
        :rtype: str
        """
        return f"{self._init_s}\t{self._init_v}\t{-self._deceleration}\t{self._end_t}\n"

    @override
    def Evaluate(self, order: int, param: float) -> float:
        """
        Evaluate the curve at specific order and parameter
        handles extrapolation internally

        :param int order: order
        :param float param: parameter
        :returns: the evaluated value
        :rtype: float
        """
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

    def Evaluate_s(self, t: float) -> float:
        """
        Evaluate s at t

        :param float t: t
        :returns: the evaluated s
        :rtype: float
        """
        if t < self._end_t:
            curr_v: float = self._init_v - self._deceleration * t
            delta_s: float = (curr_v + self._init_v) * t * 0.5
            return self._init_s + delta_s  
        else:
            return self._end_s 

    def Evaluate_v(self, t: float) -> float:
        """
        Evaluate v at t

        :param float t: t
        :returns: the evaluated v
        :rtype: float
        """
        if t < self._end_t:
            
            return self._init_v - self._deceleration * t
        else:
            return 0.0

    def Evaluate_a(self, t: float) -> float:
        """
        Evaluate a at t

        :param float t: t
        :returns: the evaluated a
        :rtype: float
        """
        if t < self._end_t:
            return -self._deceleration
        else:
            return 0.0

    def Evaluate_j(self, t: float) -> float:
        """
        Evaluate j at t, no j implemented

        :param float t: t
        :returns: the evaluated j, i.e. 0.0
        :rtype: float
        """
        return 0.0
