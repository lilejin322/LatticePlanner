from common.Vec2d import Vec2d
from typing import List
from dataclasses import dataclass, field
from common.ReferenceLine import LaneSegment
from common.LaneInfo import LaneInfo

@dataclass
class LaneWaypoint:
    """
    LaneWaypoint class
    """

    lane: dict = field(default_factory=dict)
    s: float = 0.0
    l: float = 0.0

    def __post_init__(self):
        """
        Check if lane is None
        """
        
        if self.lane is None:
            raise ValueError("LaneId cannot be None")

    def __str__(self):
        return f"(lane={self.lane}, s={self.s}, l={self.l})"

class MapPathPoint(Vec2d):
    """
    MapPathPoint class
    """

    def __init__(self, point=None, heading=0.0, lane_waypoints=None):
        """
        Constructor
        """

        if point is None:
            super().__init__()
        else:
            super().__init__(point.x(), point.y())
        self._heading = heading
        if isinstance(lane_waypoints, LaneWaypoint):
            self._lane_waypoints = [lane_waypoints]
        elif isinstance(lane_waypoints, list):
            self._lane_waypoints = lane_waypoints
        else:
            self._lane_waypoints = []

    @property
    def heading(self) -> float:
        """
        get heading property

        :returns: heading property
        :rtype: float
        """

        return self._heading

    def set_heading(self, heading: float) -> None:
        """
        set heading property

        :param heading: heading property
        """

        self._heading: float = heading

    @property
    def lane_waypoints(self) -> List[LaneWaypoint]:
        """
        get lane_waypoints property

        :returns: lane_waypoints property
        :rtype: List[LaneWaypoint]
        """

        return self._lane_waypoints

    def add_lane_waypoint(self, lane_waypoint: LaneWaypoint) -> None:
        """
        add lane_waypoint to self._lane_waypoints

        :param LaneWaypoint lane_waypoint: lane_waypoint to add
        """

        self._lane_waypoints.append(lane_waypoint)

    def add_lane_waypoints(self, lane_waypoints: List[LaneWaypoint]) -> None:
        """
        add lane_waypoints to self._lane_waypoints

        :param List[LaneWaypoint] lane_waypoints: lane_waypoints to add
        """

        self._lane_waypoints.extend(lane_waypoints)

    def clear_lane_waypoints(self) -> None:
        """
        clear lane_waypoints
        """

        self._lane_waypoints.clear()

    @staticmethod
    def RemoveDuplicates(points: List['MapPathPoint']) -> None:
        """
        remove duplicates from points

        :param List[MapPathPoint] points: points to remove duplicates from
        """

        kDuplicatedPointsEpsilon = 1e-7
        limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon
        assert points, "points should not be empty"
        count = 0
        for point in points:
            if count == 0 or point.DistanceSquareTo(points[count - 1]) > limit:
                points[count] = point
                count += 1
            else:
                points[count - 1].add_lane_waypoints(point.lane_waypoints)
        del points[count:]

    @staticmethod
    def GetPointsFromSegment(segment: LaneSegment) -> List['MapPathPoint']:
        """
        get points from segment

        :param LaneSegment segment: segment to get points from
        :returns: points from segment
        :rtype: List[MapPathPoint]
        """

        raise NotImplementedError

    @staticmethod
    def GetPointsFromLane(lane: LaneInfo, start_s: float, end_s: float) -> List['MapPathPoint']:
        """
        get points from lane

        :param LaneInfo lane: lane to get points from
        :param float start_s: start s
        :param float end_s: end s
        :returns: points from lane
        :rtype: List[MapPathPoint]
        """

        raise NotImplementedError

    def __str__(self) -> str:
        """
        get the MapPathPoint class debug string

        :returns: debug string
        :rtype: str
        """

        return f"x = {self._x}  y = {self._y}  heading = {self._heading}  waypoints = {self._lane_waypoints}"
