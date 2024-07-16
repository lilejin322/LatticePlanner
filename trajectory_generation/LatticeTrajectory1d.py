from typing import override
from common.curve1d.Curve1d import Curve1d

class LatticeTrajectory1d:
    """
    LatticeTrajectory1d class

    this class is a wrapper for Curve1d instance
    """

    def __init__(self, trajectory1d: Curve1d):
        """
        Constructor

        :param trajectory1d: QuarticPolynomialCurve1d or QuinticPolynomialCurve1d instance
        """

        super().__init__()
        self.trajectory1d = trajectory1d

        self._target_position: float = 0.0
        self._target_velocity: float = 0.0
        self._target_time: float = 0.0

        self._has_target_position = False
        self._has_target_velocity = False
        self._has_target_time = False

    @override
    def Evaluate(self, order: int, param: float) -> float:
        """
        Evaluate the trajectory at a given order and parameter value.

        :param order: Order of derivative to evaluate
        :param param: Parameter value to evaluate
        :return: Value of the evaluated value
        :rtype: float
        """

        param_length = self.trajectory1d.param_length
        if param < param_length:
            return self.trajectory1d.evaluate(order, param)

        # do constant acceleration extrapolation;
        # to align all the trajectories with time.
        p: float = self.trajectory1d.Evaluate(0, param_length)
        v: float = self.trajectory1d.Evaluate(1, param_length)
        a: float = self.trajectory1d.Evaluate(2, param_length)

        t: float = param - param_length

        if order == 0:
            return p + v * t + 0.5 * a * t * t
        elif order == 1:
            return v + a * t
        elif order == 2:
            return a
        else:
            return 0.0

    @override
    def ParamLength(self) -> float:
        """
        ParamLength

        :return: Length of the parameter
        :rtype: float
        """

        return self.trajectory1d.ParamLength()

    @override
    def ToString(self) -> str:
        """
        ToString

        :return: String representation of the trajectory
        :rtype: str
        """
        
        return self.trajectory1d.ToString()


    def has_target_position(self) -> bool:
        """
        has_target_position

        :return: True if target position is set
        :rtype: bool
        """

        return self._has_target_position

    def has_target_velocity(self) -> bool:
        """
        has_target_velocity

        :return: True if target velocity is set
        :rtype: bool
        """

        return self._has_target_velocity

    def has_target_time(self) -> bool:
        """
        has_target_time

        :return: True if target time is set
        :rtype: bool
        """

        return self._has_target_time

    def target_position(self) -> float:
        """
        target_position

        :return: Target position
        :rtype: float
        """

        assert self._has_target_position, "Target position is not set"
        return self._target_position

    def target_velocity(self) -> float:
        """
        target_velocity

        :return: Target velocity
        :rtype: float
        """

        assert self._has_target_velocity, "Target velocity is not set"
        return self._target_velocity

    def target_time(self) -> float:
        """
        target_time

        :return: Target time
        :rtype: float
        """

        assert self._has_target_time, "Target time is not set"
        return self._target_time

    def set_target_position(self, target_position) -> None:
        """
        set_target_position

        :param target_position: Target position
        """

        self._target_position = target_position
        self._has_target_position = True

    def set_target_velocity(self, target_velocity) -> None:
        """
        set_target_velocity

        :param target_velocity: Target velocity
        """

        self._target_velocity = target_velocity
        self._has_target_velocity = True

    def set_target_time(self, target_time) -> None:
        """
        set_target_time

        :param target_time: Target time
        """
        self._target_time = target_time
        self._has_target_time = True
