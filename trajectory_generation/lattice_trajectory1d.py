from common.curve1d.curve1d import Curve1d


class LatticeTrajectory1d(Curve1d):
    """
    Apollo-compatible wrapper for a 1D trajectory.

    It delegates to the underlying curve inside ParamLength() and performs
    constant-acceleration extrapolation after the curve end.
    """

    def __init__(self, trajectory1d: Curve1d):
        super().__init__()
        self._trajectory1d = trajectory1d
        self._target_position: float = 0.0
        self._target_velocity: float = 0.0
        self._target_time: float = 0.0
        self._has_target_position = False
        self._has_target_velocity = False
        self._has_target_time = False

    def Evaluate(self, order: int, param: float) -> float:
        param_length = self._trajectory1d.ParamLength()
        if param < param_length:
            return self._trajectory1d.Evaluate(order, param)

        p: float = self._trajectory1d.Evaluate(0, param_length)
        v: float = self._trajectory1d.Evaluate(1, param_length)
        a: float = self._trajectory1d.Evaluate(2, param_length)
        t: float = param - param_length

        if order == 0:
            return p + v * t + 0.5 * a * t * t
        if order == 1:
            return v + a * t
        if order == 2:
            return a
        return 0.0

    def ParamLength(self) -> float:
        return self._trajectory1d.ParamLength()

    def __str__(self) -> str:
        return str(self._trajectory1d)

    def has_target_position(self) -> bool:
        return self._has_target_position

    def has_target_velocity(self) -> bool:
        return self._has_target_velocity

    def has_target_time(self) -> bool:
        return self._has_target_time

    def target_position(self) -> float:
        assert self._has_target_position, "Target position is not set"
        return self._target_position

    def target_velocity(self) -> float:
        assert self._has_target_velocity, "Target velocity is not set"
        return self._target_velocity

    def target_time(self) -> float:
        assert self._has_target_time, "Target time is not set"
        return self._target_time

    def set_target_position(self, target_position) -> None:
        self._target_position = target_position
        self._has_target_position = True

    def set_target_velocity(self, target_velocity) -> None:
        self._target_velocity = target_velocity
        self._has_target_velocity = True

    def set_target_time(self, target_time) -> None:
        self._target_time = target_time
        self._has_target_time = True


def CreateLatticeTrajectory1d(obj: Curve1d) -> LatticeTrajectory1d:
    return LatticeTrajectory1d(obj)
