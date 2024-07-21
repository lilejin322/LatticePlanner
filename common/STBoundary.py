from common.Polygon2d import Polygon2d
from typing import List, Tuple
from enum import Enum
from common.STPoint import STPoint

class STBoundary(Polygon2d):
    """
    STBoundary class
    """

    class BoundaryType(Enum):
        """
        BoundaryType class
        """

        UNKNOWN = 0
        STOP = 1
        FOLLOW = 2
        YIELD = 3
        OVERTAKE = 4
        KEEP_CLEAR = 5

    def __init__(self, point_pairs: List[Tuple[STPoint, STPoint]], is_accurate_boundary: bool = False):
        """
        Constructors:
        STBoundary must be initialized with a vector of ST-point pairs.
        Each pair refers to a time t, with (lower_s, upper_s).

        :param List[Tuple[STPoint, STPoint]] point_pairs: List of ST-point pairs
        :param bool is_accurate_boundary: Boolean flag to indicate if the boundary is accurate
        """

        self._boundary_type = STBoundary.BoundaryType.UNKNOWN
        self._upper_points: List[STPoint] = []
        self._lower_points: List[STPoint] = []
        self._id: str = ""
        self._characteristic_length = 1.0
        self._min_s = float('inf')
        self._max_s = -float('inf')
        self._min_t = float('inf')
        self._max_t = -float('inf')
        self._bottom_left_point = None
        self._bottom_right_point = None
        self._upper_left_point = None
        self._upper_right_point = None
        self._obstacle_road_right_ending_t = 0.0

        if point_pairs:
            self.initialize(point_pairs, is_accurate_boundary)

    @staticmethod
    def CreateInstance(lower_points: List[STPoint], upper_points: List[STPoint]) -> 'STBoundary':
        """
        Wrapper of the constructor (old).

        :param List[STPoint] lower_points: List of lower ST-points
        :param List[STPoint] upper_points: List of upper ST-points
        :return: STBoundary instance
        :rtype: STBoundary
        """

        point_pairs = list(zip(lower_points, upper_points))
        return STBoundary(point_pairs)

    @staticmethod
    def CreateInstanceAccurate(lower_points: List[STPoint], upper_points: List[STPoint]) -> 'STBoundary':
        """
        Wrapper of the constructor. It doesn't use RemoveRedundantPoints
        and generates an accurate ST-boundary.
        
        :param List[STPoint] lower_points: List of lower ST-points
        :param List[STPoint] upper_points: List of upper ST-points
        :return: STBoundary instance
        :rtype: STBoundary
        """
        
        point_pairs = list(zip(lower_points, upper_points))
        return STBoundary(point_pairs, is_accurate_boundary=True)

    def IsEmpty(self) -> bool:
        """
        Check if the STBoundary is empty.

        :return: True if the STBoundary is empty, False otherwise
        :rtype: bool
        """

        return not self._lower_points

    def GetUnblockSRange(self, curr_time: float) -> Tuple[bool, float, float]:
        """
        Get the unblock s-range.

        :param float curr_time: Current time
        :return: a tuple, tuple[0] = if unblocked, tuple[1] = s_upper, tuple[2] = s_lower
        :rtype: Tuple[bool, float, float]
        """

        raise NotImplementedError

    def GetBoundarySRange(self, curr_time: float) -> Tuple[bool, float, float]:
        """
        Get the boundary s-range.

        :param float curr_time: Current time
        :return: a tuple, tuple[0] = if boundary exists, tuple[1] = s_upper, tuple[2] = s_lower
        :rtype: Tuple[bool, float, float]
        """

        raise NotImplementedError

    def GetBoundarySlopes(self, curr_time: float) -> Tuple[bool, float, float]:
        """
        Get the boundary slopes.

        :param float curr_time: Current time
        :return: a tuple, tuple[0] = if boundary exists, tuple[1] = ds_upper, tuple[2] = ds_lower
        :rtype: Tuple[bool, float, float]
        """

        raise NotImplementedError

    def PrintDebug(self, suffix: str = ""):
        """
        if you need to add boundary type, make sure you modify
        GetUnblockSRange accordingly.
        """

        raise NotImplementedError

    @staticmethod
    def TypeName(boundary_type: 'STBoundary.BoundaryType') -> str:
        """
        Get the boundary type name.

        :param STBoundary.BoundaryType boundary_type: Boundary type
        :return: Boundary type name
        :rtype: str
        """

        return boundary_type.name

    def boundary_type(self) -> 'STBoundary.BoundaryType':
        """
        Get the boundary type.

        :return: Boundary type
        :rtype: STBoundary.BoundaryType
        """

        return self._boundary_type

    def id(self) -> str:
        """
        Get the ID of the STBoundary.

        :return: ID of the STBoundary
        :rtype: str
        """

        return self._id

    def characteristic_length(self) -> float:
        """
        Get the characteristic length of the STBoundary.

        :return: Characteristic length of the STBoundary
        :rtype: float
        """

        raise NotImplementedError

    def set_id(self, id: str) -> None:
        """
        Set the ID of the STBoundary.

        :param str id: ID of the STBoundary
        """

        self._id = id

    def SetBoundaryType(self, boundary_type: 'STBoundary.BoundaryType') -> None:
        """
        Set the boundary type.

        :param STBoundary.BoundaryType boundary_type: Boundary type
        """

        self._boundary_type = boundary_type

    def SetCharacteristicLength(self, characteristic_length: float) -> None:
        """
        Set the characteristic length of the STBoundary.

        :param float characteristic_length: Characteristic length of the STBoundary
        """

        self._characteristic_length = characteristic_length

    @property
    def min_s(self) -> float:
        """
        Get the minimum s-value of the STBoundary.

        :return: Minimum s-value of the STBoundary
        :rtype: float
        """

        return self._min_s

    @property
    def min_t(self) -> float:
        """
        Get the minimum t-value of the STBoundary.

        :return: Minimum t-value of the STBoundary
        :rtype: float
        """

        return self._min_t

    @property
    def max_s(self) -> float:
        """
        Get the maximum s-value of the STBoundary.

        :return: Maximum s-value of the STBoundary
        :rtype: float
        """

        return self._max_s

    @property
    def max_t(self) -> float:
        """
        Get the maximum t-value of the STBoundary.

        :return: Maximum t-value of the STBoundary
        :rtype: float
        """

        return self._max_t

    @property
    def upper_points(self) -> List[STPoint]:
        """
        Get the upper points of the STBoundary.

        :return: Upper points of the STBoundary
        :rtype: List[STPoint]
        """

        return self._upper_points

    @property
    def lower_points(self) -> List[STPoint]:
        """
        Get the lower points of the STBoundary.

        :return: Lower points of the STBoundary
        :rtype: List[STPoint]
        """

        return self._lower_points

    def IsPointInBoundary(self, st_point: STPoint) -> bool:
        """
        Check if a point is in the boundary.
        Used by st-optimizer.

        :param STPoint st_point: ST-point
        :return: True if the point is in the boundary, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def ExpandByS(self, s: float) -> 'STBoundary':
        """
        Expand the boundary by s.
        Used by st-optimizer.

        :param float s: s-value
        :return: Expanded STBoundary
        :rtype: STBoundary
        """

        raise NotImplementedError

    def ExpandByT(self, t: float) -> 'STBoundary':
        """
        Expand the boundary by t.
        Used by st-optimizer.

        :param float t: t-value
        :return: Expanded STBoundary
        :rtype: STBoundary
        """

        raise NotImplementedError

    def CutOffByT(self, t: float) -> 'STBoundary':
        """
        Cut off the boundary by t.
        Unused function so far.

        :param float t: t-value
        :return: Cut off STBoundary
        :rtype: STBoundary
        """

        raise NotImplementedError

    @property
    def upper_left_point(self) -> STPoint:
        """
        Get the upper left point of the STBoundary.
        Used by Lattice planners.

        :return: Upper left point of the STBoundary
        :rtype: STPoint
        """

        return self._upper_left_point

    @property
    def upper_right_point(self) -> STPoint:
        """
        Get the upper right point of the STBoundary.
        Used by Lattice planners.

        :return: Upper right point of the STBoundary
        :rtype: STPoint
        """

        return self._upper_right_point

    @property
    def bottom_left_point(self) -> STPoint:
        """
        Get the bottom left point of the STBoundary.
        Used by Lattice planners.

        :return: Bottom left point of the STBoundary
        :rtype: STPoint
        """

        return self._bottom_left_point

    def bottom_right_point(self) -> STPoint:
        """
        Get the bottom right point of the STBoundary.
        Used by Lattice planners.

        :return: Bottom right point of the STBoundary
        :rtype: STPoint
        """

        return self._bottom_right_point

    def set_upper_left_point(self, st_point: STPoint) -> None:
        """
        Set the upper left point of the STBoundary.

        :param STPoint st_point: Upper left point of the STBoundary
        """

        self._upper_left_point = st_point

    def set_upper_right_point(self, st_point: STPoint) -> None:
        """
        Set the upper right point of the STBoundary.

        :param STPoint st_point: Upper right point of the STBoundary
        """

        self._upper_right_point = st_point

    def set_bottom_left_point(self, st_point: STPoint) -> None:
        """
        Set the bottom left point of the STBoundary.

        :param STPoint st_point: Bottom left point of the STBoundary
        """

        self._bottom_left_point = st_point

    def set_bottom_right_point(self, st_point: STPoint) -> None:
        """
        Set the bottom right point of the STBoundary.

        :param STPoint st_point: Bottom right point of the STBoundary
        """

        self._bottom_right_point = st_point

    def set_obstacle_road_right_ending_t(self, road_right_ending_t: float) -> None:
        """
        Set the obstacle road right ending t.

        :param float road_right_ending_t: Road right ending t
        """

        self._obstacle_road_right_ending_t = road_right_ending_t

    @property
    def obstacle_road_right_ending_t(self) -> float:
        """
        Get the obstacle road right ending t.

        :return: Obstacle road right ending t
        :rtype: float
        """

        return self._obstacle_road_right_ending_t

    def IsValid(self, point_pairs: List[Tuple[STPoint, STPoint]]) -> bool:
        """
        The sanity check function for a vector of ST-point pairs.

        :param List[Tuple[STPoint, STPoint]] point_pairs: List of ST-point pairs
        :return: True if the ST-point pairs are valid, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def IsPointNear(self, seg: LineSegment2d, point: Vec2d, max_dist: float) -> bool:
        """
        Returns true if point is within max_dist distance to seg.

        :param LineSegment2d seg: Line segment
        :param Vec2d point: Point
        :param float max_dist: Maximum distance
        :return: True if the point is near, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def RemoveRedundantPoints(self, point_pairs: List[Tuple[STPoint, STPoint]]):
        """
        Sometimes a sequence of upper and lower points lie almost on
        two straightlines. In this case, the intermediate points are removed,
        with only the end-points retained.
        TODO(all): When slope is high, this may introduce significant errors.
        Also, when accumulated for multiple t, the error can get significant.
        This function should be reconsidered, because it may be dangerous.

        :param List[Tuple[STPoint, STPoint]] point_pairs: List of ST-point pairs
        """
        
        raise NotImplementedError

    def GetIndexRange(self, points: List[STPoint], t: float) -> Tuple[bool, int, int]:
        """
        Given time t, find a segment denoted by left and right idx, that contains the time t.
        
        :param List[STPoint] points: List of ST-points
        :param float t: time
        :return: a tuple, tuple[0] = bool, If t is less than all or larger than all, return false; tuple[1] = left, tuple[2] = right
        :rtype: Tuple[bool, int, int]
        """
        
        raise NotImplementedError
