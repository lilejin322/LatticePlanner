from protoclass.PathPoint import PathPoint
from typing import List
from bisect import bisect_left, bisect_right
from PathMatcher import PathMatcher

class DiscretizedPath(list):
    """
    DiscretizedPath is a list of PathPoint objects can be iterated
    """

    def __init__(self, path_points: List[PathPoint]=[]):
        """
        Constructor
        """

        super().__init__(path_points)
    
    def Length(self) -> float:
        """
        Returns the length of the path
        """

        if len(self) == 0:
            return 0.0
        return self[-1].s - self[0].s
    
    def Evaluate(self, path_s: float) -> PathPoint:
        """
        Returns the PathPoint object at the specified path_s

        :param path_s: The path_s value
        :returns: The PathPoint object
        :rtype: PathPoint
        """

        assert len(self) > 0, "path is empty"
        it_lower = self.QueryLowerBound(path_s)
        if it_lower == 0:
            return self[0]
        if it_lower == len(self):
            return self[-1]
        return PathMatcher.InterpolateUsingLinearApproximation(self[it_lower -1], self[it_lower], path_s)

    def EvaluateReverse(self, path_s: float) -> PathPoint:
        """
        Returns the PathPoint object at the specified path_s in reverse order

        :param path_s: The path_s value
        :returns: The PathPoint object
        :rtype: PathPoint
        """

        assert len(self) > 0, "path is empty"
        it_upper = self.QueryUpperBound(path_s)
        if it_upper == 0:
            return self[0]
        if it_upper == len(self):
            return self[-1]
        return PathMatcher.InterpolateUsingLinearApproximation(self[it_upper - 1], self[it_upper], path_s)

    def QueryLowerBound(self, path_s: float) -> int:
        """
        Returns the index of the PathPoint object that is less than the specified path_s

        :param path_s: The path_s value
        :returns: The index of the PathPoint object
        :rtype: int
        """

        def func(tp: PathPoint):
            return tp.s < path_s
        return bisect_left(self, path_s, key=func)

    def QueryUpperBound(self, path_s: float) -> int:
        """
        Returns the index of the PathPoint object that is less than or equal to the specified path_s

        :param path_s: The path_s value
        :returns: The index of the PathPoint object
        :rtype: int
        """

        def func(tp: PathPoint):
            return tp.s < path_s
        return bisect_right(self, path_s, key=func)
