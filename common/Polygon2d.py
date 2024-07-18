from common.Vec2d import Vec2d
from typing import List, Tuple

class Polygon2d:
    """
    The class of polygon in 2-D.
    """

    def __init__(self, *args):
        """
        Constructor
        """

        if len(args) == 0:
            """
            Empty constructor.
            """

            self._points: List[Vec2d] = []
        
        elif isinstance(args[0], Box2d):
            """
            Constructor which takes a box.

            :param Box2d box: The box to construct the polygon.
            """
            
            pass

        elif isinstance(args[0], list):
            """
            Constructor which takes a vector of points as its vertices.

            param List[Vec2d] points: The points to construct the polygon.
            """

            points: List[Vec2d] = args[0]
            self._points: List[Vec2d] = points

        self._num_points: int = 0
        self._line_segments = []
        self._is_convex = False
        self._area = 0.0
        self._min_x, self._max_x, self._min_y, self._max_y = 0.0, 0.0, 0.0, 0.0

    @property
    def points(self) -> List[Vec2d]:
        """
        Get the vertices of the polygon.

        :returns: The vertices of the polygon.
        :rtype: List[Vec2d]
        """

        return self._points

    @property
    def line_segments(self) -> List[LineSegment2d]:
        """
        Get the edges of the polygon.

        :returns: The edges of the polygon.
        :rtype: List[LineSegment2d]
        """

        return self._line_segments

    @property
    def num_points(self) -> int:
        """
        Get the number of vertices of the polygon.

        :returns: The number of vertices of the polygon.
        :rtype: int
        """

        return self._num_points

    @property
    def is_convex(self) -> bool:
        """
        Check if the polygon is convex.

        :returns: Whether the polygon is convex or not.
        :rtype: bool
        """

        return self._is_convex
    
    @property
    def area(self) -> float:
        """
        Get the area of the polygon.

        :returns: The area of the polygon.
        :rtype: float
        """
        
        return self._area

    def DistanceToBoundary(self, point: Vec2d) -> float:
        """
        Compute the distance from a point to the boundary of the polygon.
        This distance is equal to the minimal distance from the point
        to the edges of the polygon.

        :param Vec2d point: The point to compute whose distance to the polygon.
        :returns: The distance from the point to the polygon's boundary.
        :rtype: float
        """

        raise NotImplementedError

    def DistanceTo(self, *args) -> float:

        if isinstance(args[0], Vec2d):
            """
            Compute the distance from a point to the polygon. If the point is
            within the polygon, return 0. Otherwise, this distance is
            the minimal distance from the point to the edges of the polygon.

            :param Vec2d point: The point to compute whose distance to the polygon.
            :returns: The distance from the point to the polygon.
            :rtype: float
            """

            point: Vec2d = args[0]
            raise NotImplementedError
        
        elif isinstance(args[0], LineSegment2d):
            """
            Compute the distance from a line segment to the polygon.
            If the line segment is within the polygon, or it has intersect with
            the polygon, return 0. Otherwise, this distance is
            the minimal distance between the distances from the two ends
            of the line segment to the polygon.

            :param LineSegment2d line_segment: The line segment to compute whose distance to the polygon.
            :returns: The distance from the line segment to the polygon.
            :rtype: float
            """

            line_segment: LineSegment2d = args[0]
            raise NotImplementedError
        
        elif isinstance(args[0], Box2d):
            """
            Compute the distance from a box to the polygon.
            If the box is within the polygon, or it has overlap with
            the polygon, return 0. Otherwise, this distance is
            the minimal distance among the distances from the edges
            of the box to the polygon.

            :param Box2d box: The box to compute whose distance to the polygon.
            :returns: The distance from the box to the polygon.
            :rytpe: float
            """
            
            box : Box2d = args[0]
            raise NotImplementedError

        elif isinstance(args[0], Polygon2d):
            """
            Compute the distance from another polygon to the polygon.
            If the other polygon is within this polygon, or it has overlap with
            this polygon, return 0. Otherwise, this distance is
            the minimal distance among the distances from the edges
            of the other polygon to this polygon.
            
            :param Polygon2d polygon: The polygon to compute whose distance to this polygon.
            :returns: The distance from the other polygon to this polygon.
            :rtype: float
            """

            polygon: Polygon2d = args[0]
            raise NotImplementedError

    def DistanceSquareTo(self, point: Vec2d) -> float:
        """
        Compute the square of distance from a point to the polygon.
        If the point is within the polygon, return 0. Otherwise,
        this square of distance is the minimal square of distance from
        the point to the edges of the polygon.

        :param Vec2d point: The point to compute whose square of distance to the polygon.
        :returns: The square of distance from the point to the polygon.
        :rtype: float
        """

        raise NotImplementedError

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Check if a point is within the polygon.
   
        :param Vec2d point: The target point. To check if it is within the polygon.
        :returns: Whether a point is within the polygon or not.
        :rtype: bool
        """

        raise NotImplementedError
        
    def IsPointOnBoundary(self, point: Vec2d) -> bool:
        """
        Check if a point is on the boundary of the polygon.
   
        :param Vec2d point: The target point. To check if it is on the boundary of the polygon.
        :returns: Whether a point is on the boundary of the polygon or not.
        :rtype: bool
        """

        raise NotImplementedError

    def Contains(self, *args) -> bool:

        if isinstance(args[0], LineSegment2d):
            """
            Check if the polygon contains a line segment.

            :param LineSegment2d line_segment: The target line segment. To check if the polygon contains it.
            :returns: Whether the polygon contains the line segment or not.
            :rtype: bool
            """

            line_segment: LineSegment2d = args[0]
            raise NotImplementedError

        elif isinstance(args[0], Polygon2d):
            """
            Check if the polygon contains another polygon.

            :param Polygon2d polygon: The target polygon. To check if this polygon contains it.
            :returns: Whether this polygon contains another polygon or not.
            :rtype: bool
            """

            polygon: Polygon2d = args[0]
            raise NotImplementedError

    @staticmethod
    def ComputeConvexHull(points: List[Vec2d], polygon: 'Polygon2d') -> bool:
        """
        Compute the convex hull of a group of points.
        
        :param List[Vec2d] points: The target points. To compute the convex hull of them.
        :param Polygon2d polygon: The convex hull of the points.
        :returns: If successfully compute the convex hull.
        :rtype: bool
        """

        raise NotImplementedError

    def HasOverlap(self, *args) -> bool:

        if isinstance(args[0], LineSegment2d):
            """
            Check if a line segment has overlap with this polygon.

            :param LineSegment2d line_segment: The target line segment. To check if it has overlap with this polygon.
            :returns: Whether the target line segment has overlap with this polygon or not.
            :rtype: bool
            """

            line_segment: LineSegment2d = args[0]
            raise NotImplementedError
        
        if isinstance(args[0], Polygon2d):
            """
            Check if this polygon has overlap with another polygon.

            :param Polygon2d polygon: The target polygon. To check if it has overlap with this polygon.
            :returns: If this polygon has overlap with another polygon.
            :rtype: bool
            """

            polygon: Polygon2d = args[0]
            raise NotImplementedError

    def GetOverlap(self, line_segment: LineSegment2d, first: Vec2d, last: Vec2d) -> bool:
        """
        Get the overlap of a line segment and this polygon. If they have
        overlap, output the two ends of the overlapped line segment.

        :param LineSegment2d line_segment: The target line segment. To get its overlap with this polygon.
        :param Vec2d first: First end of the overlapped line segment.
        :param Vec2d second: Second end of the overlapped line segment.
        :returns: If the target line segment has overlap with this polygon.
        """

        raise NotImplementedError

    def GetAllVertices(self) -> List[Vec2d]:
        """
        Get all vertices of the polygon

        :returns: All vertices of the polygon
        :rtype: List[Vec2d]
        """

        raise NotImplementedError
    
    def GetAllOverlaps(self, line_segment: LineSegment2d) -> List[LineSegment2d]:
        """
        Get all overlapped line segments of a line segment and this polygon.
        There are possibly multiple overlapped line segments if this polygon is not convex.
        
        :param LineSegment2d line_segment: The target line segment. To get its all overlapped
        line segments with this polygon.
        :returns: A group of overlapped line segments.
        :rtype: List[LineSegment2d]
        """

        raise NotImplementedError

    def ComputeOverlap(self, other_polygon: 'Polygon2d') -> Tuple[bool, 'Polygon2d']:
        """
        Only compute overlaps between two convex polygons.
        Compute the overlap of this polygon and the other polygon if any.
        Note: this function only works for computing overlap between
        two convex polygons.

        :param Polygon2d other_polygon: The target polygon. To compute its overlap with this polygon.
        :returns: If there is an overlapped polygon and the overlapped polygon.
        :rtype: Tuple[bool, Polygon2d]
        """

        raise NotImplementedError

    def ComputeIoU(self, other_polygon: 'Polygon2d') -> float:
        """
        Only compute intersection over union ratio between two convex polygons.
        Compute intersection over union ratio of this polygon and the other polygon.
        Note: this function only works for computing overlap between two convex polygons.

        :param Polygon2d other_polygon: The target polygon. To compute its overlap with
        this polygon.
        returns: A value between 0.0 and 1.0, meaning no intersection to full overlaping
        """

        raise NotImplementedError

    def BoundingBoxWithHeading(self, heading: float) -> Box2d:
        """
        brief Get the bound box according to a heading.
        
        :param float heading: The specified heading of the bounding box.
        :returns The bound box according to the specified heading.
        :rtype Box2d
        """
        
        raise NotImplementedError

    def MinAreaBoundingBox(self) -> Box2d:
        """
        Get the bounding box with the minimal area.

        :returns: The bounding box with the minimal area.
        :rtype: Box2d
        """

        raise NotImplementedError

    def ExtremePoints(self, heading: float) -> Tuple[Vec2d, Vec2d]:
        """
        Get the extreme points along a heading direction.

        :param float heading: The specified heading.
        :returns: The point on the boundary of this polygon with the minimal projection onto the heading direction.
                  The point on the boundary of this polygon with the maximal projection onto the heading direction.
        :rtype: Tuple[Vec2d, Vec2d]
        """

        raise NotImplementedError
    
    def ExpandByDistance(self, distance: float) -> 'Polygon2d':
        """
        Expand this polygon by a distance.

        :param float distance: The specified distance. To expand this polygon by it.
        :returns: The polygon after expansion.
        :rtype: Polygon2d
        """

        raise NotImplementedError

    def PolygonExpandByDistance(self, distance: float)  -> 'Polygon2d':
        """

        :param float distance: The specified distance. To expand the polygon by it.
        :returns: The polygon after expansion.
        :rtype: Polygon2d
        """

        raise NotImplementedError
    
    def CalculateVertices(self, shift_vec: Vec2d) -> None:
        """
        Calculate vertices based on a shift vector.

        :param Vec2d shift_vec: The shift vector.
        """

        raise NotImplementedError

    def __str__(self) -> str:
        """
        Get a string containing essential information about the polygon
        for debugging purpose.

        :returns: Essential information about the polygon for debugging purpose.
        :rtype: str
        """

        raise NotImplementedError

    @property
    def min_x(self) -> float:
        """
        Get the minimal x value of the polygon.

        :returns: The minimal x value of the polygon.
        :rtype: float
        """

        return self._min_x

    @property
    def max_x(self) -> float:
        """
        Get the maximal x value of the polygon.

        :returns: The maximal x value of the polygon.
        :rtype: float
        """

        return self._max_x

    @property
    def min_y(self) -> float:
        """
        Get the minimal y value of the polygon.

        :returns: The minimal y value of the polygon.
        :rtype: float
        """

        return self._min_y

    @property
    def max_y(self) -> float:
        """
        Get the maximal y value of the polygon.

        :returns: The maximal y value of the polygon.
        :rtype: float
        """

        return self._max_y
    
    def BuildFromPoints(self) -> None:

        raise NotImplementedError

    def Next(self, at: int) -> int:
        """

        :param int at: The index of the current point.
        :returns: The index of the next point.
        :rtype: int
        """

        raise NotImplementedError
    
    def Prev(self, at: int) -> int:
        """

        :param int at: The index of the current point.
        :returns: The index of the previous point.
        :rtype: int
        """

        raise NotImplementedError

    @staticmethod
    def ClipConvexHull(line_segment: LineSegment2d, points: List[Vec2d]) -> bool:
        """
        Clip the convex hull by a line segment.

        :param LineSegment2d line_segment: The line segment to clip the convex hull.
        :param List[Vec2d] points: The points of the convex hull.
        :returns: If successfully clip the convex hull.
        :rtype: bool
        """

        raise NotImplementedError
