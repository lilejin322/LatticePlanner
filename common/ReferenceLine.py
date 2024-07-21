from copy import deepcopy
from common.Vec2d import Vec2d, kMathEpsilon
from common.ReferencePoint import ReferencePoint
from typing import List, Tuple
from protoclass.PathPoint import PathPoint
from dataclasses import dataclass, field
from common.Box2d import Box2d
from protoclass.TrajectoryPoint import TrajectoryPoint
from protoclass.FrenetFramePoint import FrenetFramePoint
from common.SLBoundary import SLBoundary
from common.Polygon2d import Polygon2d
from logging import Logger
from bisect import bisect_left, bisect_right
from CartesianFrenetConverter import CartesianFrenetConverter
from scipy.optimize import minimize_scalar
from math import sin, cos
from protoclass.SLBoundary import SLBoundary
from common.MapPathPoint import MapPathPoint, LaneWaypoint
from protoclass.PointENU import PointENU
from config import FLAGS_default_highway_speed_limit, FLAGS_trajectory_point_num_for_debug, FLAGS_default_city_road_speed_limit, \
                   FLAGS_planning_upper_speed_limit
from protoclass.SLBoundary import SLPoint

logger = Logger("ReferenceLine")

@dataclass
class SpeedLimit:
    """
    This speed limit overrides the lane speed limit
    """

    start_s: float = 0.0
    end_s: float = 0.0
    speed_limit: float = 0.0  # unit m/s

@dataclass
class LaneSegment:
    """
    Lane segment class
    """

    lane: dict = field(default_factory=dict)
    start_s: float = 0.0
    end_s: float = 0.0

    def __post_init__(self):
        """
        Post initialization, check if property lane is not empty dict {}
        """

        assert self.lane, "lane must not be an empty dict"
    
    def Length(self) -> float:
        """
        Get length of lane segment

        :returns: Length of lane segment
        :rtype: float
        """

        return self.end_s - self.start_s

    @staticmethod
    def Join(segments: List['LaneSegment']) -> None:
        """
        Join lane segments

        :param List[LaneSegment] segments: List of LaneSegment objects
        """

        kSegmentDelta: float = 0.5
        k: int = 0
        i: int = 0
        while i < len(segments):
            j = i
            while j + 1 < len(segments) and segments[i].lane == segments[j + 1].lane:
                j += 1

            segment_k = segments[k]
            segment_k.lane = segments[i].lane
            segment_k.start_s = segments[i].start_s
            segment_k.end_s = segments[j].end_s
            if segment_k.start_s < kSegmentDelta:
                segment_k.start_s = 0.0
            if segment_k.end_s + kSegmentDelta >= segment_k.lane.total_length():
                segment_k.end_s = segment_k.lane.total_length()

            i = j + 1
            k += 1

        segments[:] = segments[:k]

    def __str__(self) -> str:
        """
        Debug string representation
        
        :returns: String representation
        :rtype: str
        """

        if not self.lane:
            return "(lane is null)"

        return f"id = {self.lane['id']}  start_s = {self.start_s}  end_s = {self.end_s}"

class ReferenceLine:
    """
    ReferenceLine class
    """

    def __init__(self, *args):
        """
        Constructor
        """

        if len(args) == 1 and isinstance(args[0], ReferenceLine):
            """
            Copy constructor

            :param ReferenceLine reference_line: ReferenceLine object to copy
            """

            self.__dict__ = deepcopy(args[0].__dict__)
        
        elif len(args) == 1 and isinstance(args[0], list):
            """
            Constructor with reference points

            : param List[ReferencePoint] reference_points: List of ReferencePoint objects
            """

            self._reference_points: List[ReferencePoint] = args[0]
            self._speed_limit: List[SpeedLimit] = []
            self._map_path = MapPath([MapPathPoint(p) for p in self._reference_points])
            self._priority: int = 0
            assert len(self._map_path) == len(self._reference_points), \
                   "Number of points in self._map_path does not match the size of self._reference_points"

        elif len(args) == 1 and isinstance(args[0], MapPath):
            """
            Constructor with map path

            : param Path map_path: Path object
            """

            self._map_path = args[0]
            self._speed_limit: List[SpeedLimit] = []
            self._reference_points: List[ReferencePoint] = []
            self._priority: int = 0
            for point in self._map_path.path_points:
                assert point.lane_waypoints, "Lane waypoints cannot be empty"
                lane_waypoint = point.lane_waypoints[0]
                map_path_point = MapPathPoint(point, point.heading, lane_waypoint)
                self._reference_points.append(ReferencePoint(map_path_point, 0.0, 0.0))
            assert len(self._map_path.num_points) == len(self._reference_points), \
                   "Number of points in self._map_path does not match the size of self._reference_points"

        else:
            """
            Default constructor
            """

            self._reference_points = []
            self._speed_limit: List[SpeedLimit] = []
            self._map_path: MapPath = MapPath([])
            self._priority: int = 0

    def Stitch(self, other: 'ReferenceLine') -> bool:
        """
        Stitch current reference line with the other reference line
        The stitching strategy is to use current reference points as much as
        possible. The following two examples show two successful stitch cases.
        Example 1
        this:   |--------A-----x-----B------|
        other:                 |-----C------x--------D-------|
        Result: |------A-----x-----B------x--------D-------|
        In the above example, A-B is current reference line, and C-D is the other
        reference line. If part B and part C matches, we update current reference
        line to A-B-D.
        Example 2
        this:                  |-----A------x--------B-------|
        other:  |--------C-----x-----D------|
        Result: |--------C-----x-----A------x--------B-------|
        In the above example, A-B is current reference line, and C-D is the other
        reference line. If part A and part D matches, we update current reference
        line to C-A-B.

        :returns: false if these two reference line cannot be stitched
        :rtype: bool
        """

        if not other.reference_points:
            logger.warning("The other reference line is empty.")
            return True
        first_point = self._reference_points.front
        first_sl: SLPoint = SLPoint()
        tag, first_sl = self.XYToSL(first_point)
        if not tag:
            logger.warning("Failed to project the first point to the other reference line.")
            return False
        first_join: bool = first_sl.s > 0 and first_sl.s < other.Length()

        last_point = self._reference_points[-1]
        last_sl: SLPoint = SLPoint()
        tag, last_sl = self.XYToSL(last_point)
        if not tag:
            logger.warning("Failed to project the last point to the other reference line.")
            return False
        last_join: bool = last_sl.s > 0 and last_sl.s < other.Length()

        if not first_join and not last_join:
            logger.warning("These reference lines are not connected.")
            return False
        
        accumulated_s: List[float] = other.map_path.accumulated_s()
        other_points = other.reference_points
        lower: float = accumulated_s[0]
        kStitchingError = 1e-1
        if first_join:
            if first_sl.l > kStitchingError:
                logger.error("lateral stitching error on first join of reference line too big, stiching fails")
                return False
            start_i: int = bisect_left(accumulated_s, first_sl.s)
            self._reference_points[:0] = other_points[:start_i]
        if last_join:
            if last_sl.l > kStitchingError:
                logger.error("lateral stitching error on first join of reference line too big, stitching fails")
                return False
            end_i: int = bisect_right(accumulated_s, last_sl.s)
            self._reference_points.extend(other_points[end_i:])
        self._map_path = MapPath([MapPathPoint(p) for p in self._reference_points])
        return True

    def Segment(self, *args) -> bool:

        if isinstance(args[0], Vec2d):
            """
            Segment by point

            :param Vec2d point: Point to segment
            :param float distance_backward: Distance backward
            :param float distance_forward: Distance forward
            :returns: True if segment is successful
            :rtype: bool
            """

            point, distance_backward, distance_forward = args
            sl: SLPoint = SLPoint()
            tag, sl = self.XYToSL(point)
            if not tag:
                logger.error(f"Failed to project point: {point}")
                return False
            return self.Segment(sl.s, distance_backward, distance_forward)
        
        elif isinstance(args[0], float):
            """
            Segment by s

            :param float s: S to segment
            :param float distance_backward: Distance backward
            :param float distance_forward: Distance forward
            :returns: True if segment is successful
            :rtype: bool
            """

            s, distance_backward, distance_forward = args
            accumulated_s: List[float] = self.map_path.accumulated_s()

            # inclusive
            start_index: int = bisect_left(accumulated_s, s - distance_backward)

            # exclusive
            end_index: int = bisect_right(accumulated_s, s + distance_forward)

            if end_index - start_index < 2:
                logger.error("Too few reference points after shrinking.")
                return False

            self._reference_points = self._reference_points[start_index:end_index]

            self._map_path = MapPath([MapPathPoint(p) for p in self._reference_points])
            return True

        else:

            return False

    @property
    def map_path(self) -> Path:
        """
        Get map path

        :returns: Map path
        :rtype: Path
        """

        return self._map_path

    @property
    def reference_points(self) -> List[ReferencePoint]:
        """
        Get reference points

        :returns: Reference points
        :rtype: List[ReferencePoint]
        """

        return self._reference_points

    def GetReferencePoint(self, *args) -> ReferencePoint:
        
        if len(args) == 1 and isinstance(args[0], float):
            """
            Get reference point by s

            :param float s: s
            :returns: Reference point
            :rtype: ReferencePoint
            """

            s = args[0]
            accumulated_s: List[float] = self._map_path.accumulated_s()
            if s < accumulated_s[0] - 1e-2:
                logger.warning(f"The requested s: {s} < 0.")
                return self._reference_points[0]
            if s > accumulated_s[-1] + 1e-2:
                logger.warning(f"The requested s: {s} > {accumulated_s[-1]}.")
                return self._reference_points[-1]
            
            interpolate_index = self._map_path.GetIndexFromS(s)

            index: int = interpolate_index.id
            next_index: int = index + 1
            if next_index >= len(self._reference_points):
                next_index = len(self._reference_points) - 1
            
            p0: ReferencePoint = self._reference_points[index]
            p1: ReferencePoint = self._reference_points[next_index]

            s0: float = accumulated_s[index]
            s1: float = accumulated_s[next_index]
            return self.InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index)

        elif len(args) == 2 and isinstance(args[0], float) and isinstance(args[1], float):
            """
            Get reference point by x and y

            :param float x: X
            :param float y: Y
            :returns: Reference point
            :rtype: ReferencePoint
            """

            x, y = args
            assert self._reference_points, "Reference points cannot be empty"
            func_distance_square = lambda point, x, y: (point.x - x) ** 2 + (point.y - y) ** 2
            
            d_min: float = func_distance_square(self._reference_points[0], x, y)
            index_min: int = 0

            for i, point in enumerate(self._reference_points):
                d_temp: float = func_distance_square(self._reference_points[i], x, y)
                if d_temp < d_min:
                    d_min = d_temp
                    index_min = i

            index_start = index_min if index_min == 0 else index_min - 1
            index_end = index_min if index_min + 1 == len(self._reference_points) else index_min + 1

            if index_start == index_end:
                return self.reference_points[index_start]
            
            s0: float = self._map_path.accumulated_s()[index_start]
            s1: float = self._map_path.accumulated_s()[index_end]

            s: float = self.FindMinDistancePoint(self._reference_points[index_start], s0, self._reference_points[index_end], s1, x, y)

            return self.Interpolate(self._reference_points[index_start], s0, self._reference_points[index_end], s1, s)

        else:

            raise ValueError(f"Incorrect number of arguments: {len(args)}")

    def GetFrenetPoint(self, path_point: PathPoint) -> FrenetFramePoint:
        """
        Get Frenet point

        :param PathPoint path_point: Path point
        :returns: Frenet frame point
        :rtype: FrenetFramePoint
        """

        if not self._reference_points:
            return FrenetFramePoint()
        
        sl_point: SLPoint = SLPoint()
        _, sl_point = self.XYToSL(path_point)
        frenet_frame_point: FrenetFramePoint = FrenetFramePoint()
        frenet_frame_point.s = sl_point.s
        frenet_frame_point.l = sl_point.l

        theta: float = path_point.theta
        kappa: float = path_point.kappa
        l: float = frenet_frame_point.l

        ref_point: ReferencePoint = self.GetReferencePoint(frenet_frame_point.s)

        theta_ref: float = ref_point.heading
        kappa_ref: float = ref_point.kappa
        dkappa_ref: float = ref_point.dkappa

        dl: float = CartesianFrenetConverter.CalculateLateralDerivative(theta_ref, theta, l, kappa_ref)
        ddl: float = CartesianFrenetConverter.CalculateSecondOrderLateralDerivative(theta_ref, theta, kappa_ref, kappa, dkappa_ref, l)
        frenet_frame_point.dl = dl
        frenet_frame_point.ddl = ddl
        return frenet_frame_point

    def ToFrenetFrame(self, traj_point: TrajectoryPoint) -> Tuple[List[float], List[float]]:
        """
        Transform trajectory point to frenet frame

        :param TrajectoryPoint traj_point: Trajectory point
        :returns: the frenet frame
        :rtype: Tuple[List[float], List[float]]
        """

        assert self._reference_points, "Reference points cannot be empty"
        
        sl_point: SLPoint = SLPoint()
        _, sl_point = self.XYToSL(traj_point.path_point.theta, [traj_point.path_point.x, traj_point.path_point.y])
        s_condition: List[float] = []
        l_condition: List[float] = []
        ref_point: ReferencePoint = self.GetReferencePoint(sl_point.s)
        s_condition, l_condition = CartesianFrenetConverter.cartesian_to_frenet(sl_point.s, ref_point.x, ref_point.y, ref_point.heading,
                                   ref_point.kappa, ref_point.dkappa, traj_point.path_point.x, traj_point.path_point.y, traj_point.v,
                                   traj_point.a, traj_point.path_point.theta, traj_point.path_point.kappa)
        logger.info(f"planning_start_point x, y, theta, kappa, v, a: {traj_point.path_point.x:.2f}, "
                    f"{traj_point.path_point.y:.2f}, "
                    f"{traj_point.path_point.theta:.2f},"
                    f"{traj_point.path_point.kappa:.2f}, "
                    f"{traj_point.v:.2f}, {traj_point.a:.2f}. ")
        logger.info(f"ref point x, y, theta, kappa, dkappa: {ref_point.x:.2f}, "
                    f"{ref_point.y:.2f}, "
                    f"{ref_point.heading:.2f}, "
                    f"{ref_point.kappa:.2f}, "
                    f"{ref_point.dkappa:.2f}.")

        return s_condition, l_condition

    def GetReferencePoints(self, start_s: float, end_s: float) -> List[ReferencePoint]:
        """
        Get reference points between start_s and end_s

        :param float start_s: Start s
        :param float end_s: End s
        :returns: Reference points
        :rtype: List[ReferencePoint]
        """

        if start_s < 0.0:
            start_s = 0.0
        if end_s > self.Length():
            end_s = self.Length()
        ref_points: List[ReferencePoint] = []
        start_index: int = self.GetNearestReferenceIndex(start_s)
        end_index: int = self.GetNearestReferenceIndex(end_s)
        if start_index < end_index:
            ref_points = self._reference_points[start_index:end_index]
        return ref_points

    def GetNearestReferenceIndex(self, s: float) -> int:
        """
        Get nearest reference index

        :param float s: s
        :returns: Nearest reference index
        :rtype: int
        """

        accumulated_s: List[float] = self._map_path.accumulated_s()
        if s < accumulated_s[0] - 1e-2:
            logger.warning(f"The requested s: {s} < 0.")
            return 0
        if s > accumulated_s[-1] + 1e-2:
            logger.warning(f"The requested s: {s} > {accumulated_s[-1]}.")
            return len(self._reference_points) - 1
        it_lower: int = bisect_left(accumulated_s, s)
        return it_lower

    def GetNearestReferencePoint(self, *args) -> ReferencePoint:

        if isinstance(args[0], Vec2d):
            """
            Get nearest reference point

            :param Vec2d xy: XY
            :returns: Nearest reference point
            :rtype: ReferencePoint
            """

            xy: Vec2d = args[0]
            min_dist: float = float('inf')
            min_index: int = 0
            for i, point in enumerate(self._reference_points):
                distance: float = DistanceXY(xy, self._reference_points[i])
                if distance < min_dist:
                    min_dist = distance
                    min_index = i
            return self._reference_points[min_index]

        elif isinstance(args[0], float):
            """
            Get nearest reference point

            :param float s: S
            :returns: Nearest reference point
            :rtype: ReferencePoint
            """

            s: float = args[0]
            
            accumulated_s: List[float] = self._map_path.accumulated_s()
            if s < accumulated_s[0] - 1e-2:
                logger.warning(f"The requested s: {s} < 0.")
                return self._reference_points[0]
            if s > accumulated_s[-1] + 1e-2:
                logger.warning(f"The requested s: {s} > {accumulated_s[-1]}.")
                return self._reference_points[-1]
            
            it_lower: int = bisect_left(accumulated_s, s)
            if it_lower == 0:
                return self._reference_points[0]
            if abs(accumulated_s[it_lower - 1] - s) < abs(accumulated_s[it_lower] - s):
                return self._reference_points[it_lower - 1]
            return self._reference_points[it_lower]

    def GetLaneSegments(self, start_s: float, end_s: float) -> List[LaneSegment]:
        """
        Get lane segments between start_s and end_s

        :param float start_s: Start s
        :param float end_s: End s
        :returns: Lane segments
        :rtype: List[LaneSegment]
        """

        return self._map_path.GetLaneSegments(start_s, end_s)

    def GetApproximateSLBoundary(self, box: Box2d, start_s: float, end_s: float) -> Tuple[bool, SLBoundary]:
        """
        Return a rough approximated SLBoundary using box length. It is guaranteed to
        be larger than the accurate SL boundary.

        :param Box2d box: Box
        :param float start_s: Start s
        :param float end_s: End s
        :returns: True if successful, SL boundary; otherwise, false, None
        :rtype: Tuple[bool, SLBoundary]
        """
        
        s: float = 0.0
        l: float = 0.0
        distance: float = 0.0
        tag, s, l, distance = self._map_path.GetProjectionWithHueristicParams(box.center, start_s, end_s)
        if not tag:
            logger.error("Cannot get projection point from path.")
            return False

        projected_point = self._map_path.GetSmoothPoint(s)
        rotated_box = box
        rotated_box.RotateFromCenter(-projected_point.heading)

        corners: List[Vec2d] = rotated_box.GetAllCorners()

        min_s: float = float('inf')
        max_s: float = float('-inf')
        min_l: float = float('inf')
        max_l: float = float('-inf')

        for point in corners:
            # x <--> s, y <--> l
            # Because the box is rotated to align the reference line
            min_s = min(min_s, point.x - rotated_box.center.x + s)
            max_s = max(max_s, point.x - rotated_box.center.x + s)
            min_l = min(min_l, point.y - rotated_box.center.y + l)
            max_l = max(max_l, point.y - rotated_box.center.y + l)
        sl_boundary = SLBoundary(start_s=min_s, end_s=max_s, start_l=min_l, end_l=max_l)
        return True, sl_boundary

    def GetSLBoundary(self, *args) -> bool:

        if isinstance(args[0], Box2d):
            """
            Get the SL Boundary of the box.

            :param Box2d box: The box to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :param SLBoundary sl_boundary: The SL boundary to store the result.
            :returns: True if success, False otherwise.
            :rtype: bool
            """

            if len(args) == 2:
                box = args[0]
                warm_start_s = -1.0
                sl_boundary = args[-1]
            else:
                box, warm_start_s, sl_boundary = args
            corners: List[Vec2d] = box.GetAllCorners()
            return self.GetSLBoundary(corners, warm_start_s, sl_boundary)

        elif isinstance(args[0], Polygon2d):
            """
            Get the SL Boundary of the polygon.

            :param Polygon2d polygon: The polygon to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :param SLBoundary sl_boundary: The SL boundary to store the result.
            :returns: True if success, False otherwise.
            :rtype: bool
            """

            if len(args) == 2:
                polygon = args[0]
                warm_start_s = -1.0
                sl_boundary = args[-1]
            else:
                polygon, warm_start_s, sl_boundary = args
            corners: List[Vec2d] = polygon.points
            return self.GetSLBoundary(corners, warm_start_s, sl_boundary)

        elif isinstance(args[0], list) and isinstance(args[0][0], Vec2d):
            """
            Get the SL Boundary of the corners.

            :param List[Vec2d] corners: The corners to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :param SLBoundary sl_boundary: The SL boundary to store the result.
            :returns: True if success, False otherwise.
            :rtype: bool
            """

            corners, warm_start_s, sl_boundary = args
            start_s: float = float('inf')
            end_s: float = float('-inf')
            start_l: float = float('inf')
            end_l: float = float('-inf')

            # The order must be counter-clockwise
            sl_corners: List[SLPoint] = []
            for point in corners:
                tag, sl_point = self.XYToSL(point)
                if not tag:
                    logger.error(f"Failed to get projection for point: {point} on reference line.")
                    return False, None
                sl_corners.append(sl_point)
            
            for i in range(len(corners)):
                index0: int = i
                index1: int = (i + 1) % len(corners)
                p0: SLPoint = corners[index0]
                p1: SLPoint = corners[index1]

                p_mid = (p0 + p1) * 0.5
                tag, sl_point_mid = self.XYToSL(p_mid)
                if not tag:
                    logger.error(f"Failed to get projection for point: {p_mid} on reference line.")
                    return False, None
                
                v0 = Vec2d(sl_corners[index1].s - sl_corners[index0].s,
                           sl_corners[index1].l - sl_corners[index0].l)
                v1 = Vec2d(sl_point_mid.s - sl_corners[index0].s,
                           sl_point_mid.l - sl_corners[index0].l)

                sl_boundary.boundary_points.append(sl_corners[index0])

                # sl_point is outside of polygon; add to the vertex list
                if v0.CrossProd(v1) < 0.0:
                    sl_boundary.boundary_points.append(sl_point_mid)
            
            for sl_point in sl_boundary.boundary_points:
                start_s = min(start_s, sl_point.s)
                end_s = max(end_s, sl_point.s)
                start_l = min(start_l, sl_point.l)
                end_l = max(end_l, sl_point.l)
            
            sl_boundary.start_s = start_s
            sl_boundary.end_s = end_s
            sl_boundary.start_l = start_l
            sl_boundary.end_l = end_l

            return True

        elif isinstance(args[0], Polygon):
            """
            Get the SL Boundary of the map-polygon.

            :param Polygon polygon: The map-polygon to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :param SLBoundary sl_boundary: The SL boundary to store the result.
            :returns: True if success, False otherwise.
            :rtype: bool
            """

            polygon = args[0]
            start_s: float = float('inf')
            end_s: float = float('-inf')
            start_l: float = float('inf')
            end_l: float = float('-inf')
            for point in polygon.point:
                tag, sl_point = self.XYToSL(sl_point)
                if not tag:
                    logger.error(f"Failed to get projection for point: {point} on reference line.")
                    return False, None
                start_s = min(start_s, sl_point.s)
                end_s = max(end_s, sl_point.s)
                start_l = min(start_l, sl_point.l)
                end_l = max(end_l, sl_point.l)
            sl_boundary.start_s = start_s
            sl_boundary.end_s = end_s
            sl_boundary.start_l = start_l
            sl_boundary.end_l = end_l

            return True

    def SLToXY(self, sl_point: SLPoint) -> Tuple[bool, Vec2d]:
        """
        Transvert Frenet to Cartesian coordinates.

        :param SLPoint sl_point: The Frenet coordinates.
        :returns: True if success, the Cartesian coordinates; otherwise, false, None
        :rtype: Tuple[bool, Vec2d]
        """

        if self._map_path.num_points < 2:
            logger.error("The reference line has too few points.")
            return False, None
        
        matched_point = self.GetReferencePoint(sl_point.s)
        angle = matched_point.heading
        return True, Vec2d((matched_point.x - sin(angle) * sl_point.l), (matched_point.y + cos(angle) * sl_point.l))

    def XYToSL(self, *args) -> Tuple[bool, SLPoint]:
        
        if isinstance(args[0], Vec2d):
            
            """
            Transvert Cartesian to Frenet coordinates.

            :param Vec2d xy_point: The Cartesian coordinates.
            :param warm_start_s: The initial s for searching mapping point on reference
            :returns: True if success, and The Cartesian coordinates; otherwise, false, None
            :rtype: Tuple[bool, Vec2d]
            """

            if len(args) == 1:
                """
                If the warm_start_s not given, search begin at the
                start of the reference line.
                """
                xy_point = args[0]
                warm_start_s = -1.0
            else:
                xy_point, warm_start_s = args
            s: float = warm_start_s
            l = 0.0
            if warm_start_s < 0.0:
                tag, s, l = self._map_path.GetProjection(xy_point)
                if not tag:
                    logger.error("Cannot get nearest point from path.")
                    return False, None
            else:
                tag, s, l = self._map_path.GetProjectionWithWarmStartS(xy_point)
                if not tag:
                    logger.error(f"Cannot get nearest point from path with warm_start_s : {warm_start_s}")
                    return False, None    

            return True, SLPoint(s, l)

        elif isinstance(args[0], float):
            """
            Transvert Cartesian to Frenet coordinates.

            :param float heading: The heading of the vehicle.
            :param Vec2d xy_point: The Cartesian coordinates.
            :param warm_start_s: The initial s for searching mapping point on reference
            :returns: True if success, and The Cartesian coordinates; otherwise, false, None
            :rtype: Tuple[bool, Vec2d]
            """
            
            if len(args) == 2:
                """
                If the warm_start_s not given, search begin at the
                start of the reference line.
                """
                heading, xy_point = args
                warm_start_s = -1.0
            
            else:
                heading, xy_point, warm_start_s = args
            s: float = warm_start_s
            l = 0.0
            if warm_start_s < 0.0:
                tag, s, l = self._map_path.GetProjection(xy_point)
                if not tag:
                    logger.error("Cannot get nearest point from path.")
                    return False, None
            else:
                tag, s, l = self._map_path.GetProjectionWithWarmStartS(xy_point)
                if not tag:
                    logger.error(f"Cannot get nearest point from path with warm_start_s : {warm_start_s}")
                    return False, None
            
            return True, SLPoint(s, l)

        else:
            if hasattr(args[0], 'x') and hasattr(args[0], 'y'):
                """
                Other objects with x and y attributes

                :param ABC xy: The Cartesian coordinates.
                """

                xy = args[0]
                return self.XYToSL(Vec2d(xy.x, xy.y))
            else:
                raise ValueError(f"Invalid arguments: {args}")

    def GetLaneWidth(self, s: float) -> Tuple[bool, float, float]:
        """
        Get lane width

        :param float s: S
        :returns: True if success, left lane width, right lane width; otherwise, false, 0.0, 0.0
        :rtype: Tuple[bool, float, float]
        """

        if not self._map_path.path_points:
            return False, 0.0, 0.0
        
        tag, lane_left_width, lane_right_width = self._map_path.GetLaneWidth(s)
        return tag, lane_left_width, lane_right_width

    def GetOffsetToMap(self, s: float) -> Tuple[bool, float]:
        """
        Get offset to map

        :param float s: S
        :returns: True if success, offset to map; otherwise, false, 0.0
        :rtype: Tuple[bool, float]
        """

        if not self._map_path.path_points:
            return False, 0.0
        
        ref_point = self.GetNearestReferencePoint(s)
        if not ref_point.lane_waypoints:
            return False, 0.0
        l_offset = ref_point.lane_waypoints[0].l
        return True, l_offset

    def GetRoadWidth(self, s) -> Tuple[bool, float, float]:
        """
        Get road width

        :param float s: S
        :returns: True if success, left road width, right road width; otherwise, false, 0.0, 0.0
        :rtype: Tuple[bool, float, float]
        """

        if not self._map_path.path_points:
            return False, 0.0, 0.0
        tag, road_left_width, road_right_width = self._map_path.GetRoadWidth(s)
        return tag, road_left_width, road_right_width

    def GetRoadType(self, s: float) -> RoadType:
        """
        Get road type

        :param float s: S
        :returns: Road type
        :rtype: Type
        """
        
        hdmap = HDMapUtil.BaseMap()
        if hdmap is None:
            raise ValueError("HDMap is None")

        road_type: RoadType = Road.UNKNOWN

        sl_point = SLPoint(s=s, l=0.0)
        _, pt = self.SLToXY(sl_point)

        point = PointENU(x=pt.x, y=pt.y, z=0.0)
        
        roads: RoadInfo = hdmap.GetRoads(point, 4.0)
        
        for road in roads:
            if road.type != Road.UNKNOWN:
                road_type = road.type
                break
        
        return road_type

    def GetLaneBoundaryType(self, s: float) -> Tuple:
        """
        Get lane boundary type

        :param float s: S
        :returns: Left lane boundary type, right lane boundary type
        :rtype: Tuple[LaneBoundaryType, LaneBoundaryType]
        """

        raise NotImplementedError

    def GetLaneFromS(self, s: float) -> List[str]:
        """
        Get lane from s

        :param float s: s
        :returns: Lane info
        :rtype: List[str]
        """

        ref_point = self.GetReferencePoint(s)
        lane_set = set()
        for lane_waypoint in ref_point.lane_waypoints:
            lane = lane_waypoint.lane
            if lane not in lane_set:
                lane_set.add(lane)
        return list(lane_set)

    def GetDrivingWidth(self, sl_boundary: SLBoundary) -> float:
        """
        Get driving width

        :param SLBoundary sl_boundary: SL boundary
        :returns: Driving width
        :rtype: float
        """

        _, lane_left_width, lane_right_width = self.GetLaneWidth(sl_boundary.start_s)
        driving_width: float = max(lane_left_width - sl_boundary.end_l, lane_right_width + sl_boundary.start_l)
        driving_width = min(lane_left_width + lane_right_width, driving_width)
        logger.debug(f"Driving width: {driving_width}.")
        return driving_width

    def IsOnLane(self, *args) -> bool:

        if isinstance(args[0], SLPoint):
            """
            check if the point is on lane along reference line

            :param SLPoint sl_point: The point to check
            :returns: True if the point is on lane; otherwise, false
            :rtype: bool
            """

            sl_point: SLPoint = args[0]
            if sl_point.s <= 0 or sl_point.s > self._map_path.length():
                return False
            left_width: float = 0.0
            right_width: float = 0.0

            tag, left_width, right_width = self.GetLaneWidth(sl_point.s)
            if not tag:
                return False
            
            return sl_point.l >= -right_width and sl_point.l <= left_width

        elif isinstance(args[0], Vec2d):
            """
            check if the point is on lane along reference line

            :param Vec2d vec2d_point: The point to check
            :returns: True if the point is on lane; otherwise, false
            :rtype: bool
            """

            vec2d_point = args[0]
            tag, sl_point = self.XYToSL(vec2d_point)
            if not tag:
                return False
            return self.IsOnLane(sl_point)

        elif isinstance(args[0], SLBoundary):
            """
            check if the boundary is on lane along reference line

            :param SLBoundary sl_boundary: The boundary to check
            :returns: True if the boundary is on lane; otherwise, false
            :rtype: bool
            """

            sl_boundary = args[0]
            if sl_boundary.end_s < 0 or sl_boundary.start_s > self.Length():
                return False
            middle_s: float = (sl_boundary.start_s + sl_boundary.end_s) / 2.0
            lane_left_width: float = 0.0
            lane_right_width: float = 0.0
            lane_left_width, lane_right_width = self._map_path.GetLaneWidth(middle_s)
            return sl_boundary.start_l <= lane_left_width and sl_boundary.end_l >= -lane_right_width

        elif hasattr(args[0], 'x') and hasattr(args[0], 'y'):
            """
            check if the object is on lane along reference line

            :param ABC xy: The object to check
            :returns: True if the object is on lane; otherwise, false
            :rtype: bool
            """

            xy = args[0]
            return self.IsOnLane(Vec2d(xy.x, xy.y))

        else:
            raise ValueError(f"Invalid arguments: {args}")

    def IsOnRoad(self, *args) -> bool:

        if isinstance(args[0], SLPoint):
            """
            Check if SL point is on road
            (not on sideways/medians) along reference line

            :param SLPoint sl_point: SL point
            :returns: True if SL point is on road; otherwise, false
            :rtype: bool
            """

            sl_point: SLPoint = args[0]
            if sl_point.s <= 0 or sl_point.s > self._map_path.length():
                return False
            tag, road_left_width, road_right_width = self.GetRoadWidth(sl_point.s)
            if not tag:
                return False
            return sl_point.l >= -road_right_width and sl_point.l <= road_left_width

        elif isinstance(args[0], Vec2d):
            """
            Check if Vec2d point is on road
            (not on sideways/medians) along reference line

            :param Vec2d vec2d_point: Vec2d point
            :returns: True if Vec2d point is on road; otherwise, false
            :rtype: bool
            """

            vec2d_point: Vec2d = args[0]
            tag, sl_point = self.XYToSL(vec2d_point)
            return tag and self.IsOnRoad(sl_point)

        elif isinstance(args[0], SLBoundary):
            """
            Check if SL boundary is on road
            (not on sideways/medians) along reference line

            :param SLBoundary sl_boundary: SL boundary
            :returns: True if SL boundary is on road; otherwise, false
            :rtype: bool
            """

            sl_boundary = args[0]
            if sl_boundary.end_s < 0 or sl_boundary.start_s > self.Length():
                return False
            middle_s: float = (sl_boundary.start_s + sl_boundary.end_s) / 2.0
            _, road_left_width, road_right_width = self.GetRoadWidth(middle_s)
            return sl_boundary.start_l <= road_left_width and sl_boundary.end_l >= -road_right_width

        else:
            raise ValueError(f"Invalid arguments: {args}")

    def IsBlockRoad(self, box2d: Box2d, gap: float) -> bool:
        """
        Check if a box is blocking the road surface. The criteria is to
        check whether the remaining space on the road surface is larger than the
        provided gap space.

        :param Box2d box2d: the provided box
        :param float gap: check the gap of the space
        :returns: True if the road is blocked; otherwise, false
        :rtype: bool
        """

        return self._map_path.OverlapWith(box2d, gap)

    def HasOverlap(self, box: Box2d) -> bool:
        """
        check if any part of the box has overlap with the road.

        :param Box2d box: The box to check
        :returns: True if there is overlap; otherwise, false
        :rtype: bool
        """

        sl_boundary = SLBoundary()
        if not self.GetSLBoundary(box, sl_boundary):
            logger.error(f"Failed to get sl boundary for box: {box}")
            return False
        if sl_boundary.end_s < 0 or sl_boundary.start_s > self.Length():
            return False
        if sl_boundary.start_l * sl_boundary.end_l < 0:
            return False

        mid_s: float = (sl_boundary.start_s + sl_boundary.end_s) / 2.0
        if mid_s < 0 or mid_s > self.Length():
            logger.error(f"ref_s is out of range: {mid_s}")
            return False
        tag, lane_left_width, lane_right_width = self.GetLaneWidth(mid_s)
        if not tag:
            logger.error(f"Failed to get width at s = {mid_s}")
            return False
        if sl_boundary.start_l > 0:
            return sl_boundary.start_l < lane_left_width
        else:
            return sl_boundary.end_l > -lane_right_width

    def Length(self) -> float:
        """
        Get length of the map path

        :returns: Length of the map path
        :rtype: float
        """

        return self.map_path.length()

    def __str__(self) -> str:
        """
        String representation for debugging

        :returns: String representation
        :rtype: str
        """

        limit: int = min(len(self._reference_points), FLAGS_trajectory_point_num_for_debug)
        point_num_str: str = f"point num:{len(self._reference_points)}"
        points_str = "".join(f"{point} " for point in self._reference_points[:limit])
        return point_num_str + points_str

    def GetSpeedLimitFromS(self, s: float) -> float:
        """
        Get speed limit from s

        :param float s: S
        :returns: Speed limit
        :rtype: float
        """
        
        for speed_limit in self._speed_limit:
            if s >= speed_limit.start_s and s <= speed_limit.end_s:
                return speed_limit.speed_limit
        map_path_point: ReferencePoint = self.GetReferencePoint(s)

        speed_limit: float = FLAGS_planning_upper_speed_limit
        speed_limit_found: bool = False
        for lane_waypoint in map_path_point.lane_waypoints:
            if lane_waypoint.lane is None:
                logger.warning("lane_waypoint.lane is None")
                continue
            speed_limit_found = True
            speed_limit = min(lane_waypoint.lane['speed_limit'], speed_limit)

        if not speed_limit_found:
            # use default speed limit based on road_type
            speed_limit = FLAGS_default_city_road_speed_limit
            road_type: RoadType = self.GetRoadType(s)
            if road_type == RoadType.HIGHWAY:
                speed_limit = FLAGS_default_highway_speed_limit

        return speed_limit    

    def AddSpeedLimit(self, start_s: float, end_s: float, speed_limit: float) -> None:
        """
        Add speed limit

        :param float start_s: Start s
        :param float end_s: End s
        :param float speed_limit: Speed limit to add
        """
        new_speed_limit: List[SpeedLimit] = []
        for limit in self._speed_limit:
            if start_s >= limit.end_s or end_s <= limit.start_s:
                new_speed_limit.append(limit)
            else:
                # start_s < speed_limit.end_s && end_s > speed_limit.start_s
                min_speed: float = min(limit.speed_limit, speed_limit)
                if start_s > limit.start_s:
                    new_speed_limit.append(SpeedLimit(limit.start_s, start_s, min_speed))
                    if end_s <= limit.end_s:
                        new_speed_limit.append(SpeedLimit(start_s, end_s, min_speed))
                        new_speed_limit.append(SpeedLimit(end_s, limit.end_s, limit.speed_limit))
                    else:
                        new_speed_limit.append(SpeedLimit(start_s, limit.end_s, min_speed))
                else:
                    new_speed_limit.append(SpeedLimit(start_s, limit.start_s, speed_limit))
                    if end_s <= limit.end_s:
                        new_speed_limit.append(SpeedLimit(limit.start_s, end_s, min_speed))
                        new_speed_limit.append(SpeedLimit(end_s, limit.end_s, limit.speed_limit))
                    else:
                        new_speed_limit.append(SpeedLimit(limit.start_s, limit.end_s, min_speed))
                start_s = limit.end_s
                end_s = max(end_s, limit.end_s)
        self._speed_limit.clear()
        if end_s > start_s:
            new_speed_limit.append(SpeedLimit(start_s, end_s, speed_limit))
        for limit in new_speed_limit:
            if limit.start_s < limit.end_s:
                self._speed_limit.append(limit)
        self._speed_limit.sort(key=lambda x: (x.start_s, x.end_s, x.speed_limit))

    @property
    def GetPriority(self) -> int:
        """
        Get priority

        :returns: Priority
        :rtype: int
        """

        return self._priority

    def SetPriority(self, priority: int) -> None:
        """
        Set priority

        :param int priority: Priority
        """

        self._priority = priority

    @property
    def GetMapPath(self) -> Path:
        """
        Get map path

        :returns: Map path
        :rtype: Path
        """

        return self._map_path

    @staticmethod
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

    @staticmethod
    def Interpolate(p0: ReferencePoint, s0: float, p1: ReferencePoint, s1: float, s: float) -> ReferencePoint:
        """
        Linearly interpolate p0 and p1 by s0 and s1.
        The input has to satisfy condition: s0 <= s <= s1
        p0 and p1 must have lane_waypoint.
        Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
        parallel neighboring lanes. Otherwise the interpolated result may not
        valid.     s0 <= s <= s1

        :param ReferencePoint p0: the first anchor point for interpolation.
        :param float s0: the longitutial distance (s) of p0 on current reference line.
        :param ReferencePoint p1: the second anchor point for interpolation
        :param float s1: the longitutial distance (s) of p1 on current reference line.
        :param float s: identifies the middle point that is going to be interpolated.
        :returns: The interpolated ReferencePoint.
        :rtype: ReferencePoint
        """

        if abs(s0 - s1) < kMathEpsilon:
            return p0
        assert s0 - 1.0e-6 <= s, f"s: {s} is less than s0: {s0}"
        assert s <= s1 + 1.0e-6, f"s: {s} is larger than s1: {s1}"

        x: float = ReferenceLine.lerp(p0.x, s0, p1.x, s1, s)
        y: float = ReferenceLine.lerp(p0.y, s0, p1.y, s1, s)
        heading: float = slerp(p0.heading, s0, p1.heading, s1, s)
        kappa: float = ReferenceLine.lerp(p0.kappa, s0, p1.kappa, s1, s)
        dkappa: float = ReferenceLine.lerp(p0.dkappa, s0, p1.dkappa, s1, s)
        waypoints: List[LaneWaypoint] = []
        if p0.lane_waypoints and p1.lane_waypoints:
            p0_waypoint = p0.lane_waypoints[0]
            if (s - s0) + p0_waypoint.s <= p0_waypoint.lane.total_length:
                lane_s: float = p0_waypoint.s + s - s0
                waypoints.append(LaneWaypoint(p0_waypoint.lane, lane_s))
            p1_waypoint = p1.lane_waypoints[0]
            if (p1_waypoint.lane.id != p0_waypoint.lane.id) and (p1_waypoint.s - s1 + s >= 0):
                lane_s: float = p1_waypoint.s - s1 + s
                waypoints.append(LaneWaypoint(p1_waypoint.lane, lane_s))
            if not waypoints:
                lane_s: float = p0_waypoint.s
                waypoints.append(LaneWaypoint(p0_waypoint.lane, lane_s))
        return ReferencePoint(MapPathPoint([x, y], heading, waypoints), kappa, dkappa)

    def InterpolateWithMatchedIndex(self, p0: ReferencePoint, s0: float, p1: ReferencePoint, s1: float, index: InterpolatedIndex) -> ReferencePoint:
        """
        Interpolate with matched index

        :param ReferencePoint p0: The first anchor point for interpolation.
        :param float s0: The longitutial distance (s) of p0 on current reference line.
        :param ReferencePoint p1: The second anchor point for interpolation
        :param float s1: The longitutial distance (s) of p1 on current reference line.
        :param int index: The index to interpolate.
        :returns: The interpolated ReferencePoint.
        :rtype: ReferencePoint
        """

        if abs(s0 - s1) < kMathEpsilon:
            return p0
        s: float = s0 + index.offset
        assert s0 - 1.0e-6 <= s, f"s: {s} is less than s0: {s0}" 
        assert s <= s1 + 1.0e-6, f"s: {s} is greater than s1: {s1}"

        map_path_point = self._map_path.GetSmoothPoint(index)
        kappa: float = ReferenceLine.lerp(p0.kappa, s0, p1.kappa, s1, s)
        dkappa: float = ReferenceLine.lerp(p0.dkappa, s0, p1.dkappa, s1, s)

        return ReferencePoint(map_path_point, kappa, dkappa)

    def FindMinDistancePoint(self, p0: ReferencePoint, s0: float, p1: ReferencePoint, s1: float, x: float, y: float) -> float:
        """
        Find the minimum distance point

        :param ReferencePoint p0: The first point
        :param float s0: The first s
        :param ReferencePoint p1: The second point
        :param float s1: The second s
        :param float x: x
        :param float y: y
        :returns: The minimum distance point
        :rtype: float
        """

        func_dist_square = lambda s: ((self.Interpolate(p0, s0, p1, s1, s).x - x) ** 2 + 
                           (self.Interpolate(p0, s0, p1, s1, s).y - y) ** 2)
        result = minimize_scalar(func_dist_square, bounds=(s0, s1), method='bounded')
        return result.x
