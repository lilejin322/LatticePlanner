from common.curve1d.Curve1d import Curve1d
from typing import List, override
from config import FLAGS_numerical_epsilon
from logging import Logger

class ConstantDecelerationTrajectory1d(Curve1d):
    """
    ConstantDecelerationTrajectory1d class
    """

    def __init__(self, init_s: float, init_v: float, a: float):
        """
        Constructor

        :param float init_s: init s
        :param float init_v: init v
        :param float a: param a
        """

        super().__init__()
        self._init_s: float = init_s
        self._deceleration: float = -a
        self.logger = Logger("ConstantDecelerationTrajectory1d")
        if init_v < -FLAGS_numerical_epsilon:
            self.logger.error(f"negative init v = {init_v}")
        self._init_v: float = abs(init_v)
        assert self._deceleration > 0.0, "Deceleration should be positive"
        self._end_t: float = self._init_v / self._deceleration
        self._end_s: float = self._init_v ** 2 / (2.0 * self._deceleration) + self._init_s

    @override
    def ParamLength(self) -> float:
        """
        Get the param

        :returns: the param
        :rtype: float
        """

        return self._end_t

    @override
    def ToString(self) -> str:
        """
        Convert the trajectory to a string

        :returns: The trajectory as a string representation
        :rtype: str
        """

        return ""

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
        Evaluate j at t

        :param float t: t
        :returns: the evaluated j
        :rtype: float
        """

        return 0.0
