from common.Polygon2d import Polygon2d
from typing import List, Tuple
from enum import Enum
from common.STPoint import STPoint
from copy import deepcopy
from common.Vec2d import Vec2d
from config import FLAGS_speed_lon_decision_horizon
from logging import Logger
from bisect import bisect_left
from common.LineSegment2d import LineSegment2d
from common.Polygon2d import CrossProd

logger = Logger("STBoundary")

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
        self._max_s = float('-inf')
        self._min_t = float('inf')
        self._max_t = float('-inf')
        self._bottom_left_point = None
        self._bottom_right_point = None
        self._upper_left_point = None
        self._upper_right_point = None
        self._obstacle_road_right_ending_t = float('-inf')

        assert self.IsValid(point_pairs), "The input point_pairs are NOT valid"
        reduced_pairs = deepcopy(point_pairs)
        if not is_accurate_boundary:
            self.RemoveRedundantPoints(reduced_pairs)
        for item in reduced_pairs:
            # use same t for both points
            t: float = item[0].t
            self._lower_points.append(STPoint(item[0].s, t))
            self._upper_points.append(STPoint(item[1].s, t))
        for point in self._lower_points:
            self._points.append(Vec2d(point.t, point.s))
        for rit in reversed(self._upper_points):
            self._points.append(Vec2d(rit.t, rit.s))
        
        self.BuildFromPoints()

        for point in self._lower_points:
            self._min_s = min(self._min_s, point.s)
        for point in self._upper_points:
            self._max_s = max(self._max_s, point.s)
        self._min_t = self._lower_points[0].t
        self._max_t = self._lower_points[-1].t

    @staticmethod
    def CreateInstance(lower_points: List[STPoint], upper_points: List[STPoint]) -> 'STBoundary':
        """
        Wrapper of the constructor (old).

        :param List[STPoint] lower_points: List of lower ST-points
        :param List[STPoint] upper_points: List of upper ST-points
        :return: STBoundary instance
        :rtype: STBoundary
        """

        if len(lower_points) != len(upper_points) or len(lower_points) < 2:
            return STBoundary()
        
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
        
        if len(lower_points) != len(upper_points) or len(lower_points) < 2:
            return STBoundary()
        
        point_pairs = list(zip(lower_points, upper_points))
        return STBoundary(point_pairs, True)

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

        s_upper = FLAGS_speed_lon_decision_horizon
        s_lower = 0.0
        if curr_time < self._min_t or curr_time > self._max_t:
            return True, s_upper, s_lower
        
        tag, left, right = self.GetIndexRange(self._lower_points, curr_time)
        if not tag:
            logger.error("Fail to get index range.")
            return False, s_upper, s_lower
        
        if curr_time > self._upper_points[right].t:
            return True, s_upper, s_lower
        
        r: float = 0.0 if left == right else (curr_time - self._upper_points[left].t) / (self._upper_points[right].t - self._upper_points[left].t)

        upper_cross_s: float = self._upper_points[left].s + r * (self._upper_points[right].s - self._upper_points[left].s)
        lower_cross_s: float = self._lower_points[left].s + r * (self._lower_points[right].s - self._lower_points[left].s)
        if (self._boundary_type == STBoundary.BoundaryType.STOP or self._boundary_type == STBoundary.BoundaryType.YIELD or self._boundary_type == STBoundary.BoundaryType.FOLLOW):
            s_upper = lower_cross_s
        elif self._boundary_type == STBoundary.BoundaryType.OVERTAKE:
            s_lower = max(s_lower, upper_cross_s)
        else:
            logger.debug(f"boundary_type is not supported. boundary_type: {self._boundary_type.name}")
            return False, s_upper, s_lower
        return True, s_upper, s_lower

    def GetBoundarySRange(self, curr_time: float) -> Tuple[bool, float, float]:
        """
        Get the boundary s-range.

        :param float curr_time: Current time
        :return: a tuple, tuple[0] = if boundary exists, tuple[1] = s_upper, tuple[2] = s_lower
        :rtype: Tuple[bool, float, float]
        """

        if curr_time < self._min_t or curr_time > self._max_t:
            return False, float('inf'), float('-inf')
        tag, left, right = self.GetIndexRange(self._lower_points, curr_time)
        if not tag:
            logger.error("Fail to get index range.")
            return False, float('inf'), float('-inf')
        r: float = 0.0 if left == right else (curr_time - self._upper_points[left].t) / (self._upper_points[right].t - self._upper_points[left].t)
        
        s_upper: float = self._upper_points[left].s + r * (self._upper_points[right].s - self._upper_points[left].s)
        s_lower: float = self._lower_points[left].s + r * (self._lower_points[right].s - self._lower_points[left].s)

        s_upper = min(s_upper, FLAGS_speed_lon_decision_horizon)
        s_lower = max(s_lower, 0.0)
        return True, s_upper, s_lower

    def GetBoundarySlopes(self, curr_time: float) -> Tuple[bool, float, float]:
        """
        Get the boundary slopes.

        :param float curr_time: Current time
        :return: a tuple, tuple[0] = if boundary exists, tuple[1] = ds_upper, tuple[2] = ds_lower
        :rtype: Tuple[bool, float, float]
        """

        if curr_time < self._min_t or curr_time > self._max_t:
            return False, float('inf'), float('-inf')
        kTimeIncrement: float = 0.05
        t_prev: float = curr_time - kTimeIncrement
        has_prev, prev_s_upper, prev_s_lower = self.GetBoundarySRange(t_prev)
        t_next: float = curr_time + kTimeIncrement
        has_next, next_s_upper, next_s_lower = self.GetBoundarySRange(t_next)
        _, curr_s_upper, curr_s_lower = self.GetBoundarySRange(curr_time)
        if (not has_prev) and (not has_next):
            return False, float('inf'), float('-inf')
        if has_prev and has_next:
            ds_upper = ((next_s_upper - curr_s_upper) / kTimeIncrement +
                        (curr_s_upper - prev_s_upper) / kTimeIncrement) * 0.5
            ds_lower = ((next_s_lower - curr_s_lower) / kTimeIncrement +
                        (curr_s_lower - prev_s_lower) / kTimeIncrement) * 0.5
            return True, ds_upper, ds_lower
        if has_prev:
            ds_upper = (curr_s_upper - prev_s_upper) / kTimeIncrement
            ds_lower = (curr_s_lower - prev_s_lower) / kTimeIncrement
        else:
            ds_upper = (next_s_upper - curr_s_upper) / kTimeIncrement
            ds_lower = (next_s_lower - curr_s_lower) / kTimeIncrement
        return True, ds_upper, ds_lower

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

    @property
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

    @property
    def characteristic_length(self) -> float:
        """
        Get the characteristic length of the STBoundary.

        :return: Characteristic length of the STBoundary
        :rtype: float
        """

        return self._characteristic_length

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

        if st_point.t <= self._min_t or st_point.t >= self._max_t:
            return False
        tag, left, right = self.GetIndexRange(self._lower_points, st_point.t)
        if not tag:
            logger.error("Fail to get index range.")
            return False
        check_upper: float = CrossProd(st_point, self._upper_points[left], self._upper_points[right])
        check_lower: float = CrossProd(st_point, self._lower_points[left], self._lower_points[right])

        return check_upper * check_lower < 0

    def ExpandByS(self, s: float) -> 'STBoundary':
        """
        Expand the boundary by s.
        Used by st-optimizer.

        :param float s: s-value
        :return: Expanded STBoundary
        :rtype: STBoundary
        """

        if not self._lower_points:
            return STBoundary()
        point_pairs: List[Tuple[STPoint, STPoint]] = []
        for i in range(len(self._lower_points)):
            point_pairs.append((STPoint(self._lower_points[i].s - s, self._lower_points[i].t), 
                                STPoint(self._upper_points[i].s + s, self._upper_points[i].t)))
        return STBoundary(point_pairs)

    def ExpandByT(self, t: float) -> 'STBoundary':
        """
        Expand the boundary by t.
        Used by st-optimizer.

        :param float t: t-value
        :return: Expanded STBoundary
        :rtype: STBoundary
        """

        if not self._lower_points:
            logger.error("The current st_boundary has NO points.")
            return STBoundary()
        
        point_pairs: List[Tuple[STPoint, STPoint]] = []
        left_delta_t: float = self._lower_points[1].t - self._lower_points[0].t
        lower_left_delta_s: float = self._lower_points[1].s - self._lower_points[0].s
        upper_left_delta_s: float = self._upper_points[1].s - self._upper_points[0].s

        point_pairs.append(STPoint(self._lower_points[0].s - t * lower_left_delta_s / left_delta_t,
                                   self._lower_points[0].t - t),
                           STPoint(self._upper_points[0].s - t * upper_left_delta_s / left_delta_t,
                                   self._upper_points[0].t - t))
        
        kMinSEpsilon: float = 1e-3
        point_pairs[0][0].s = min(point_pairs[0][1].s - kMinSEpsilon,
                                  point_pairs[0][0].s)
        
        for i in range(len(self._lower_points)):
            point_pairs.append((self._lower_points[i], self._upper_points[i]))
        
        length: int = len(self._lower_points)
        assert length >= 2, "The length of lower_points is less than 2."
        
        right_delta_t: float = self._lower_points[length - 1].t - self._lower_points[length - 2].t
        lower_right_delta_s: float = self._lower_points[length - 1].s - self._lower_points[length - 2].s
        upper_right_delta_s: float = self._upper_points[length - 1].s - self._upper_points[length - 2].s

        point_pairs.append(STPoint(self._lower_points[-1].s + t * lower_right_delta_s / right_delta_t,
                                   self._lower_points[-1].t + t),
                           STPoint(self._upper_points[-1].s + t * upper_right_delta_s / right_delta_t,
                                   self._upper_points[-1].t + t))

        point_pairs[-1][1].s = max(point_pairs[-1][1].s, point_pairs[-1][0].s + kMinSEpsilon)
        
        return STBoundary(point_pairs)

    def CutOffByT(self, t: float) -> 'STBoundary':
        """
        Cut off the boundary by t.
        Unused function so far.

        :param float t: t-value
        :return: Cut off STBoundary
        :rtype: STBoundary
        """

        lower_points: List[STPoint] = []
        upper_points: List[STPoint] = []
        for i in range(min(len(self._lower_points), len(self._upper_points))):
            if self._lower_points[i].t < t:
                continue
            lower_points.append(STPoint(self._lower_points[i]))
            upper_points.append(STPoint(self._upper_points[i]))
        return self.CreateInstance(lower_points, upper_points)

    @property
    def upper_left_point(self) -> STPoint:
        """
        Get the upper left point of the STBoundary.
        Used by Lattice planners.

        :return: Upper left point of the STBoundary
        :rtype: STPoint
        """

        assert self._upper_points, "StBoundary has zero points."
        return self._upper_points[0]

    @property
    def upper_right_point(self) -> STPoint:
        """
        Get the upper right point of the STBoundary.
        Used by Lattice planners.

        :return: Upper right point of the STBoundary
        :rtype: STPoint
        """

        assert self._upper_points, "StBoundary has zero points."
        return self._upper_points[-1]

    @property
    def bottom_left_point(self) -> STPoint:
        """
        Get the bottom left point of the STBoundary.
        Used by Lattice planners.

        :return: Bottom left point of the STBoundary
        :rtype: STPoint
        """

        assert self._lower_points, "StBoundary has zero points."
        return self._lower_points[0]

    @property
    def bottom_right_point(self) -> STPoint:
        """
        Get the bottom right point of the STBoundary.
        Used by Lattice planners.

        :return: Bottom right point of the STBoundary
        :rtype: STPoint
        """

        assert self._lower_points, "StBoundary has zero points."
        return self._lower_points[-1]

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

        if len(point_pairs) < 2:
            logger.error(f"point_pairs.size() must >= 2, but current point_pairs.size() = {len(point_pairs)}")
            return False
    
        kStBoundaryEpsilon: float = 1e-9
        kMinDeltaT: float = 1e-6
        for i in range(len(point_pairs)):
            curr_lower = point_pairs[i][0]
            curr_upper = point_pairs[i][1]
            if curr_upper.s < curr_lower.s:
                logger.error(f"ST-boundary's upper-s must >= lower-s")
                return False
            if abs(curr_lower.t - curr_upper.t) > kStBoundaryEpsilon:
                logger.error(f"Points in every ST-point pair should be at the same time.")
                return False
            if i + 1 != len(point_pairs):
                next_lower = point_pairs[i + 1][0]
                next_upper = point_pairs[i + 1][1]
                if max(curr_lower.t, curr_upper.t) + kMinDeltaT >= min(next_lower.t, next_upper.t):
                    logger.error(f"Latter points should have larger t: [curr_lower: {curr_lower}, curr_upper: {curr_upper}, next_lower: {next_lower}, next_upper: {next_upper}]")
                    return False
        return True

    def IsPointNear(self, seg: LineSegment2d, point: Vec2d, max_dist: float) -> bool:
        """
        Returns true if point is within max_dist distance to seg.

        :param LineSegment2d seg: Line segment
        :param Vec2d point: Point
        :param float max_dist: Maximum distance
        :return: True if the point is near, False otherwise
        :rtype: bool
        """

        return seg.DistanceSquareTo(point) < max_dist ** 2

    def RemoveRedundantPoints(self, point_pairs: List[Tuple[STPoint, STPoint]]) -> None:
        """
        Sometimes a sequence of upper and lower points lie almost on
        two straightlines. In this case, the intermediate points are removed,
        with only the end-points retained.
        TODO(all): When slope is high, this may introduce significant errors.
        Also, when accumulated for multiple t, the error can get significant.
        This function should be reconsidered, because it may be dangerous.

        :param List[Tuple[STPoint, STPoint]] point_pairs: List of ST-point pairs
        """
        
        if len(point_pairs) <= 2:
            return
        
        kMaxDist: float = 0.1
        i: int = 0
        j: int = 1

        while i < len(point_pairs) and j + 1 < len(point_pairs):
            lower_seg: LineSegment2d = LineSegment2d(point_pairs[i][0], point_pairs[j + 1][0])
            upper_seg: LineSegment2d = LineSegment2d(point_pairs[i][1], point_pairs[j + 1][1])
            if (not self.IsPointNear(lower_seg, point_pairs[j][0], kMaxDist)) or (not self.IsPointNear(upper_seg, point_pairs[j][1], kMaxDist)):
                i += 1
                if i != j:
                    point_pairs[i] = point_pairs[j]
            j += 1
        i += 1
        point_pairs[i] = point_pairs[-1]
        point_pairs = point_pairs[:i + 1]

    def GetIndexRange(self, points: List[STPoint], t: float) -> Tuple[bool, int, int]:
        """
        Given time t, find a segment denoted by left and right idx, that contains the time t.
        
        :param List[STPoint] points: List of ST-points
        :param float t: time
        :return: a tuple, tuple[0] = bool, If t is less than all or larger than all, return false; tuple[1] = left, tuple[2] = right
        :rtype: Tuple[bool, int, int]
        """
        
        if t < points[0].t or t > points[-1].t:
            logger.error(f"t is out of range. t = {t}")
            return False, int('-inf'), int('inf')
        index: int = bisect_left([point.t for point in points], t)
        if index == 0:
            left, right = 0, 0
        elif index == len(points):
            left, right = len(points) - 1, len(points) - 1
        else:
            left, right = index - 1, index
        return True, left, right


    def __str__(self) -> str:
        """
        Debug string representation.

        :return: Debug string representation
        :rtype: str
        """

        type_name: str = self.TypeName(self._boundary_type)
        key_name = type_name + "_" + self._id
        reverse_upper = reversed(self._upper_points)
        points = f"lower_points: {self._lower_points}, upper_points: {reverse_upper}"
        return f"STBoundary: " + key_name + ", " + points
