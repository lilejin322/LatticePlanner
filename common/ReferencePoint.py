from typing import List
from common.MapPathPoint import MapPathPoint
from copy import deepcopy
from protoclass.PathPoint import PathPoint

class ReferencePoint(MapPathPoint):
    """
    ReferenecePoint class
    """

    def __init__(self, map_path_point: MapPathPoint, kappa: float = 0.0, dkappa: float = 0.0):
        """
        Constructor

        :param MapPathPoint map_path_point: map_path_point
        :param float kappa: kappa
        :param float dkappa: dkappa
        """
        
        self.__dict__ = deepcopy(map_path_point.__dict__)
        self._kappa: float = kappa
        self._dkappa: float = dkappa

    def ToPathPoint(self, s: float) -> PathPoint:
        """
        Convert to PathPoint class

        :param float s: s
        :returns: PathPoint class
        :rtype: PathPoint
        """

        return PointFactory.ToPathPoint(self.x, self.y, 0.0, s, self.heading, self._kappa, self._dkappa)

    @property
    def kappa(self) -> float:
        """
        get kappa property

        :returns: kappa property
        :rtype: float
        """

        return self._kappa

    @property
    def dkappa(self) -> float:
        """
        get dkappa property

        :returns: dkappa property
        :rtype: float
        """

        return self._dkappa

    def __str__(self) -> str:
        """
        get the ReferencePoint class debug string

        :returns: debug string
        :rtype: str
        """

        return f"x: {self.x}, y: {self.y}, theta: {self.heading}, kappa: {self.kappa}, dkappa: {self.dkappa}"

    @staticmethod
    def RemoveDuplicates(points: List['ReferencePoint']) -> None:
        """
        remove duplicates from points

        :param List[ReferencePoint] points: points to remove duplicates from
        """
        
        kDuplicatedPointsEpsilon = 1e-7     # Minimum distance to remove duplicated points.
        assert points, "points should not be empty"
        count: int = 0
        for i in range(len(points)):
            last_point = points[count - 1]
            this_point = points[i]
            # Use manhattan distance for save computation time.
            if count == 0 or abs(last_point.x - this_point.x) > kDuplicatedPointsEpsilon or abs(last_point.y - this_point.y) > kDuplicatedPointsEpsilon:
                points[count] = this_point
                count += 1
            else:
                last_point.add_lane_waypoints(this_point.lane_waypoints)
        del points[count:]
