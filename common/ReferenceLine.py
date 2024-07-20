from copy import deepcopy
from common.Vec2d import Vec2d
from common.ReferencePoint import ReferencePoint
from typing import List, Tuple
from protoclass.PathPoint import PathPoint
from dataclasses import dataclass
from common.Box2d import Box2d
from protoclass.TrajectoryPoint import TrajectoryPoint
from protoclass.FrenetFramePoint import FrenetFramePoint
from common.SLBoundary import SLBoundary
from common.Polygon2d import Polygon2d

@dataclass
class SpeedLimit:
    """
    This speed limit overrides the lane speed limit
    """

    start_s: float = 0.0
    end_s: float = 0.0
    speed_limit: float = 0.0  # unit m/s

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
            self._map_path = Path()
            self._priority: int = 0

        elif len(args) == 1 and isinstance(args[0], Path):
            """
            Constructor with map path

            : param Path map_path: Path object
            """

            self._map_path = args[0]
            self._speed_limit: List[SpeedLimit] = []
            self._reference_points: List[ReferencePoint] = []
            self._priority: int = 0

        else:
            """
            Default constructor
            """

            self._reference_points = []
            self._speed_limit: List[SpeedLimit] = []
            self._map_path = Path()
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

        raise NotImplementedError

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
            raise NotImplementedError
        
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
            raise NotImplementedError

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
            raise NotImplementedError
        
        elif len(args) == 2 and isinstance(args[0], float) and isinstance(args[1], float):
            """
            Get reference point by x and y

            :param float x: X
            :param float y: Y
            :returns: Reference point
            :rtype: ReferencePoint
            """

            x, y = args
            raise NotImplementedError
        
        else:
            raise ValueError(f"Incorrect number of arguments: {len(args)}")

    def GetFrenetPoint(self, path_point: PathPoint) -> FrenetFramePoint:
        """
        Get Frenet point

        :param PathPoint path_point: Path point
        :returns: Frenet frame point
        :rtype: FrenetFramePoint
        """

        raise NotImplementedError

    def ToFrenetFrame(self, traj_point: TrajectoryPoint) -> Tuple[List[float], List[float]]:
        """
        Transform trajectory point to frenet frame

        :param TrajectoryPoint traj_point: Trajectory point
        :returns: the frenet frame
        :rtype: Tuple[List[float], List[float]]
        """

        raise NotImplementedError

    def GetReferencePoints(self, start_s: float, end_s: float) -> List[ReferencePoint]:
        """
        Get reference points between start_s and end_s

        :param float start_s: Start s
        :param float end_s: End s
        :returns: Reference points
        :rtype: List[ReferencePoint]
        """

        raise NotImplementedError

    def GetNearestReferenceIndex(self, s: float) -> int:
        """
        Get nearest reference index

        :param float s: s
        :returns: Nearest reference index
        :rtype: int
        """

        raise NotImplementedError

    def GetNearestReferencePoint(self, *args) -> ReferencePoint:

        if isinstance(args[0], Vec2d):
            """
            Get nearest reference point

            :param Vec2d xy: XY
            :returns: Nearest reference point
            :rtype: ReferencePoint
            """

            xy: Vec2d = args[0]
            raise NotImplementedError

        elif isinstance(args[0], float):
            """
            Get nearest reference point

            :param float s: S
            :returns: Nearest reference point
            :rtype: ReferencePoint
            """

            s: float = args[0]
            raise NotImplementedError

    def GetLaneSegments(self, start_s: float, end_s: float) -> List[LaneSegment]:
        """
        Get lane segments between start_s and end_s

        :param float start_s: Start s
        :param float end_s: End s
        :returns: Lane segments
        :rtype: List[LaneSegment]
        """

        raise NotImplementedError

    def GetApproximateSLBoundary(self, box: Box2d, start_s: float, end_s: float) -> Tuple[bool, SLBoundary]:
        """
        Get approximate SL boundary

        :param Box2d box: Box
        :param float start_s: Start s
        :param float end_s: End s
        :returns: True if successful, SL boundary; otherwise, false, None
        :rtype: Tuple[bool, SLBoundary]
        """
        
        raise NotImplementedError

    def GetSLBoundary(self, *args) -> Tuple[bool, SLBoundary]:
        """
        Get the SL Boundary of the box.
        
        :param Box2d box: The box to calculate.
        :param SLBoundary sl_boundary: Output of the SLBoundary.
        :param float warm_start_s: The initial s for searching mapping point on reference
                                   line to accelerate computation time.
        :returns: True if success, the returned sl_boundary; otherwise, false, None
        :rtype: Tuple[bool, SLBoundary]
        """

        if isinstance(args[0], Box2d):
            """
            Get the SL Boundary of the box.

            :param Box2d box: The box to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :returns: True if success, the returned sl_boundary; otherwise, false, None
            :rtype: Tuple[bool, SLBoundary]
            """

            if len(args) == 1:
                box = args[0]
                warm_start_s = -1.0
                raise NotImplementedError
            else:
                box, warm_start_s = args
                raise NotImplementedError

        elif isinstance(args[0], Polygon2d):
            """
            Get the SL Boundary of the polygon.

            :param Polygon2d polygon: The polygon to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :returns: True if success, the returned sl_boundary; otherwise, false, None
            :rtype: Tuple[bool, SLBoundary]
            """

            if len(args) == 1:
                polygon = args[0]
                warm_start_s = -1.0
                raise NotImplementedError
            else:
                polygon, warm_start_s = args
                raise NotImplementedError

        elif isinstance(args[0], list) and isinstance(args[0][0], Vec2d):
            """
            Get the SL Boundary of the corners.

            :param List[Vec2d] corners: The corners to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :returns: True if success, the returned sl_boundary; otherwise, false, None
            :rtype: Tuple[bool, SLBoundary]
            """

            corners, warm_start_s = args
            raise NotImplementedError
        
        elif isinstance(args[0], Polygon):
            """
            Get the SL Boundary of the map-polygon.

            :param Polygon polygon: The map-polygon to calculate.
            :param float warm_start_s: The initial s for searching mapping point on reference
                                       line to accelerate computation time.
            :returns: True if success, the returned sl_boundary; otherwise, false, None
            :rtype: Tuple[bool, SLBoundary]
            """

            polygon = args[0]
            raise NotImplementedError

    def SLToXY(self, sl_point: SLPoint) -> Tuple[bool, Vec2d]:
        """
        Transvert Frenet to Cartesian coordinates.

        :param SLPoint sl_point: The Frenet coordinates.
        :returns: True if success, the Cartesian coordinates; otherwise, false, None
        :rtype: Tuple[bool, Vec2d]
        """

        raise NotImplementedError

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
                raise NotImplementedError
            else:
                xy_point, warm_start_s = args
                raise NotImplementedError
        
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
                raise NotImplementedError
            
            else:
                heading, xy_point, warm_start_s = args
                raise NotImplementedError

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

        raise NotImplementedError

    def GetOffsetToMap(self, s: float) -> Tuple[bool, float]:
        """
        Get offset to map

        :param float s: S
        :returns: True if success, offset to map; otherwise, false, 0.0
        :rtype: Tuple[bool, float]
        """

        raise NotImplementedError

    def GetRoadWidth(self, s) -> Tuple[bool, float, float]:
        """
        Get road width

        :param float s: S
        :returns: True if success, left road width, right road width; otherwise, false, 0.0, 0.0
        :rtype: Tuple[bool, float, float]
        """

        raise NotImplementedError

    def GetRoadType(self, s: float) -> RoadType:
        """
        Get road type

        :param float s: S
        :returns: Road type
        :rtype: Type
        """
        
        raise NotImplementedError

    def GetLaneBoundaryType(self, s: float) -> Tuple[LaneBoundaryType, LaneBoundaryType]:
        """
        Get lane boundary type

        :param float s: S
        :returns: Left lane boundary type, right lane boundary type
        :rtype: Tuple[LaneBoundaryType, LaneBoundaryType]
        """

        raise NotImplementedError

    def GetLaneFromS(self, s: float) -> List[LaneInfo]:
        """
        Get lane from s

        :param float s: S
        :returns: Lane info
        :rtype: List[LaneInfo]
        """
        
        raise NotImplementedError

    def GetDrivingWidth(self, sl_boundary: SLBoundary) -> float:
        """
        Get driving width

        :param SLBoundary sl_boundary: SL boundary
        :returns: Driving width
        :rtype: float
        """

        raise NotImplementedError

    def IsOnLane(self, *args) -> bool:

        if isinstance(args[0], SLPoint):
            """
            check if the point is on lane along reference line

            :param SLPoint sl_point: The point to check
            :returns: True if the point is on lane; otherwise, false
            :rtype: bool
            """

            sl_point = args[0]
            raise NotImplementedError
        
        elif isinstance(args[0], Vec2d):
            """
            check if the point is on lane along reference line

            :param Vec2d vec2d_point: The point to check
            :returns: True if the point is on lane; otherwise, false
            :rtype: bool
            """

            vec2d_point = args[0]
            raise NotImplementedError

        elif isinstance(args[0], SLBoundary):
            """
            check if the boundary is on lane along reference line

            :param SLBoundary sl_boundary: The boundary to check
            :returns: True if the boundary is on lane; otherwise, false
            :rtype: bool
            """

            sl_boundary = args[0]
            raise NotImplementedError

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
            raise NotImplementedError
        
        elif isinstance(args[0], Vec2d):
            """
            Check if Vec2d point is on road
            (not on sideways/medians) along reference line

            :param Vec2d vec2d_point: Vec2d point
            :returns: True if Vec2d point is on road; otherwise, false
            :rtype: bool
            """

            vec2d_point: Vec2d = args[0]
            raise NotImplementedError
        
        elif isinstance(args[0], SLBoundary):
            """
            Check if SL boundary is on road
            (not on sideways/medians) along reference line

            :param SLBoundary sl_boundary: SL boundary
            :returns: True if SL boundary is on road; otherwise, false
            :rtype: bool
            """

            sl_boundary = args[0]
            raise  NotImplementedError
        
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

        raise NotImplementedError

    def HasOverlap(self, box: Box2d) -> bool:
        """
        check if any part of the box has overlap with the road.

        :param Box2d box: The box to check
        :returns: True if there is overlap; otherwise, false
        :rtype: bool
        """

        raise NotImplementedError

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

        raise NotImplementedError

    def GetSpeedLimitFromS(self, s: float) -> float:
        """
        Get speed limit from s

        :param float s: S
        :returns: Speed limit
        :rtype: float
        """
        
        raise NotImplementedError

    def AddSpeedLimit(self, start_s: float, end_s: float, speed_limit: float) -> None:
        """
        Add speed limit

        :param float start_s: Start s
        :param float end_s: End s
        :param float speed_limit: Speed limit
        """
        self.speed_limit_.append({'start_s': start_s, 'end_s': end_s, 'speed_limit': speed_limit})

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

    def GetMapPath(self) -> Path:
        """
        Get map path

        :returns: Map path
        :rtype: Path
        """

        return self._map_path

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

        raise NotImplementedError
    
    def InterpolateWithMatchedIndex(p0: ReferencePoint, s0: float, p1: ReferencePoint, s1: float, index: InterpolatedIndex) -> ReferencePoint:
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

        raise NotImplementedError

    def FindMinDistancePoint(p0: ReferencePoint, s0: float, p1: ReferencePoint, s1: float, x: float, y: float) -> float:
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

        raise NotImplementedError
