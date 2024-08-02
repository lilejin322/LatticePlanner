from typing import List, Tuple
from bisect import bisect_left

class SpeedLimit:
    """
    Speed limit class
    """

    def __init__(self):
        """
        use a vector to represent speed limit
        the first number is s, the second number is v
        It means at distance s from the start point, the speed limit is v.
        """

        self._speed_limit_points: List[Tuple[float, float]] = []

    def AppendSpeedLimit(self, s: float, v: float) -> None:
        """
        Append a speed limit

        :param float s: The distance from the start point
        :param float v: The speed limit
        """

        if len(self._speed_limit_points) > 0:
            assert s >= self._speed_limit_points[-1][0], "s should be monotonically increasing"
        self._speed_limit_points.append((s, v))

    @property
    def speed_limit_points(self) -> List[Tuple[float, float]]:
        """
        Get the speed limit points

        :returns: The speed limit points
        :rtype: List[Tuple[float, float]]
        """

        return self._speed_limit_points

    def GetSpeedLimitByS(self, s: float) -> float:
        """
        Get the speed limit by distance s

        :param float s: The distance from the start point
        :returns: The speed limit
        :rtype: float
        """

        assert len(self._speed_limit_points) >= 2, "The speed limit points must contain at least 2 elements."
        assert s >= self._speed_limit_points[0][0], "s should be greater than the first s"
        points = [point[0] for point in self._speed_limit_points]
        index = bisect_left(points, s)
        if index == len(points):
            return self._speed_limit_points[-1][1]
        return self._speed_limit_points[index][1]

    def Clear(self) -> None:
        """
        Clear the speed limit
        """

        self._speed_limit_points.clear()
