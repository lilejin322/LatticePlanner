from protoclass.SpeedPoint import SpeedPoint
from typing import List, Tuple
from bisect import bisect_left
from logging import Logger
from copy import deepcopy
from config import FLAGS_trajectory_point_num_for_debug

class SpeedData(list):
    """
    SpeedData class, iherited from list to store List[SpeedPoint]
    """

    def __init__(self, speed_points: List[SpeedPoint]=[]):
        """
        Constructor
        """

        super().__init__(sorted(speed_points, key=lambda p: p.t))
        self.logger = Logger("SpeedData")

    def AppendSpeedPoint(self, s: float, time: float, v: float, a: float, da: float) -> None:
        """
        Append a SpeedPoint to the SpeedData
        """

        speed_point = SpeedPoint(s, time, v, a, da)
        if len(self) > 0:
            assert self[-1].t < time, "Time must be monotonous"
        self.append(speed_point)

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

    def EvaluateByTime(self, t: float) -> Tuple[bool, SpeedPoint]:
        """
        Evaluate the SpeedData by time

        :param float time: the time to evaluate
        :return: a tuple of a boolean and a SpeedPoint
        :rtype: Tuple[bool, SpeedPoint]
        """

        if len(self) < 2:
            return False, None
        if not (self[0].t < t + 1e-6 and t - 1e-6 < self[-1].t):
            return False, None
        times = [point.t for point in self]
        index = bisect_left(times, t)

        if index == len(self):
            speed_point: SpeedPoint = deepcopy(self[-1])
        elif index == 0:
            speed_point: SpeedPoint = deepcopy(self[0])
        else:
            p0: SpeedPoint = self[index - 1]
            p1: SpeedPoint = self[index]
            t0: float = p0.t
            t1: float = p1.t

            s = self.lerp(p0.s, t0, p1.s, t1, t)
            if p0.v is not None and p1.v is not None:
                v = self.lerp(p0.v, t0, p1.v, t1, t)
            else:
                v = None
            if p0.a is not None and p1.a is not None:
                a = self.lerp(p0.a, t0, p1.a, t1, t)
            else:
                a = None
            if p0.da is not None and p1.da is not None:
                da = self.lerp(p0.da, t0, p1.da, t1, t)
            else:
                da = None

            speed_point = SpeedPoint(s, t, v, a, da)

        return True, speed_point

    def EvaluateByS(self, s: float) -> Tuple[bool, SpeedPoint]:
        """
        Evaluate the SpeedData by s

        Assuming spatial traversed distance is monotonous, which is the case for
        current usage on city driving scenario

        :param float s: the s to evaluate
        :return: a tuple of a boolean and a SpeedPoint
        :rtype: Tuple[bool, SpeedPoint]
        """

        if len(self) < 2:
            return False, None
        if not (self[0].s < s + 1e-6 and s - 1e-6 < self[-1].s):
            return False, None
        distances = [point.s for point in self]
        index = bisect_left(distances, s)

        if index == len(self):
            speed_point: SpeedPoint = deepcopy(self[-1])
        elif index == 0:
            speed_point: SpeedPoint = deepcopy(self[0])
        else:
            p0: SpeedPoint = self[index - 1]
            p1: SpeedPoint = self[index]
            s0: float = p0.s
            s1: float = p1.s

            t = self.lerp(p0.t, s0, p1.t, s1, s)
            if p0.v is not None and p1.v is not None:
                v = self.lerp(p0.v, s0, p1.v, s1, s)
            else:
                v = None
            if p0.a is not None and p1.a is not None:
                a = self.lerp(p0.a, s0, p1.a, s1, s)
            else:
                a = None
            if p0.da is not None and p1.da is not None:
                da = self.lerp(p0.da, s0, p1.da, s1, s)
            else:
                da = None

            speed_point = SpeedPoint(s, t, v, a, da)
        return True, speed_point

    def TotalTime(self) -> float:
        """
        Return the total time of the SpeedData

        :return: the total time
        :rtype: float
        """

        if len(self) == 0:
            return 0.0
        return self[-1].t - self[0].t

    def TotalLength(self) -> float:
        """
        Assuming spatial traversed distance is monotonous

        :return: the total length
        :rtype: float
        """

        if len(self) == 0:
            return 0.0
        return self[-1].s - self[0].s

    def __str__(self) -> str:
        """
        Return the string representation of the SpeedData

        :return: the string representation
        :rtype: str
        """

        limit = min(len(self), FLAGS_trajectory_point_num_for_debug)
        points = [f"SpeedPoint(s={p.s}, time={p.t}, v={p.v}, a={p.a}, da={p.da})" for p in self[:limit]]
        return "[\n" + ",\n".join(points) + "\n]"
