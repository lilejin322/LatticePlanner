from protoclass.FrenetFramePoint import FrenetFramePoint
from typing import List
from protoclass.SLBoundary import SLBoundary
from bisect import bisect_left, bisect_right
from logging import Logger

logger: Logger = Logger("FrenetFramePath")

def lerp(x0: float, t0: float, x1: float, t1: float, t: float) -> float:
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
        logger.error("Input time difference is too small")
        return x0
    r = (t - t0) / (t1 - t0)
    x = x0 + r * (x1 - x0)
    return x

class FrenetFramePath(list):
    """
    FrenetFramePath class, inherited from list
    """

    def __init__(self, points: List[FrenetFramePoint]=[]):
        """
        Constructor
        """

        super().__init__(points)
    
    def Length(self) -> float:
        """
        Get the length of the path
        """

        if len(self) == 0:
            return 0.0
        return self[-1].s - self[0].s

    def GetNearestPoint(self, sl: SLBoundary) -> FrenetFramePoint:
        """
        Get the FrenetFramePoint that is within SLBoundary, or the one with
        smallest l() in SLBoundary's s range [start_s(), end_s()]

        :param SLBoundary sl: the SLBoundary
        :returns: the nearest FrenetFramePoint
        :rtype: FrenetFramePoint
        """

        assert len(self) > 1, "The path must contain more than one point."
        it_lower: int = bisect_left(self, sl.start_s, key=lambda p: p.s)
        if it_lower == len(self):
            return self[-1]
        it_upper: int = bisect_right(self, sl.end_s, lo=it_lower, key=lambda p: p.s)
        min_dist: float = float('inf')
        min_it: int = it_upper
        for it in range(it_lower, it_upper):
            if sl.start_l <= self[it].l <= sl.end_l:
                return self[it]
            elif self[it].l > sl.end_l:
                diff: float = self[it].l - sl.end_l
                if diff < min_dist:
                    min_dist = diff
                    min_it = it
            else:
                diff: float = sl.start_l - self[it].l
                if diff < min_dist:
                    min_dist = diff
                    min_it = it
        return self[min_it]

    def EvaluateByS(self, s: float) -> FrenetFramePoint:
        """
        Evaluate the FrenetFramePoint at s

        :param s: s value
        :returns: the evaluated FrenetFramePoint
        :rtype: FrenetFramePoint
        """

        assert len(self) > 1, "The path must contain more than one point."
        it_lower: int = bisect_left(self, s, key=lambda p: p.s)
        if it_lower == 0:
            return self[0]
        elif it_lower == len(self):
            return self[-1]
        p0: FrenetFramePoint = self[it_lower - 1]
        s0: float = p0.s
        p1: FrenetFramePoint = self[it_lower]
        s1: float = p1.s

        p: FrenetFramePoint = FrenetFramePoint(s,lerp(p0.l, s0, p1.l, s1, s), lerp(p0.dl, s0, p1.dl, s1, s), lerp(p0.ddl, s0, p1.ddl, s1, s))
        return p
