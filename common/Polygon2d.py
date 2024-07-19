from common.Vec2d import Vec2d
from typing import List, Tuple
from copy import deepcopy

kMathEpsilon: float = 1e-10

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
            
            box: Box2d = args[0]
            box.GetAllCorners(self._points)
            self.BuildFromPoints()

        elif isinstance(args[0], list):
            """
            Constructor which takes a vector of points as its vertices.

            param List[Vec2d] points: The points to construct the polygon.
            """

            points: List[Vec2d] = args[0]
            self._points: List[Vec2d] = points
            self.BuildFromPoints()

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

        distance: float = float('inf')
        for i in range(self._num_points):
            distance = min(distance, self._line_segments[i].DistanceTo(point))
        return distance

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
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            if self.IsPointIn(point):
                return 0.0
            distance: float = float('inf')
            for i in range(self._num_points):
                distance = min(distance, self._line_segments[i].DistanceTo(point))
            return distance
        
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
            if line_segment.length() <= kMathEpsilon:
                return self.DistanceTo(line_segment.start())
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            if self.IsPointIn(line_segment.center()):
                return 0.0
            if any(poly_seg.HasIntersect(line_segment) for poly_seg in self._line_segments):
                return 0.0
            distance: float = min(self.DistanceTo(line_segment.start()), self.DistanceTo(line_segment.end()))
            for i in range(self._num_points):
                distance = min(distance, line_segment.DistanceTo(self._points[i]))
            return distance

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
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            return self.DistanceTo(Polygon2d(box))

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
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            assert len(polygon._points) >= 3, f"len(polygon._points) should be greater than 3, but got {len(polygon._points)}"
            if self.IsPointIn(polygon.points[0]):
                return 0.0
            if polygon.IsPointIn(self.points[0]):
                return 0.0
            distance: float = float('inf')
            for i in range(self._num_points):
                distance = min(distance, polygon.DistanceTo(self._line_segments[i]))
            return distance

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

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        if self.IsPointIn(point):
            return 0.0
        distance_sqr: float = float('inf')
        for i in range(self._num_points):
            distance_sqr = min(distance_sqr, self._line_segments[i].DistanceSquareTo(point))
        return distance_sqr

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Check if a point is within the polygon.
   
        :param Vec2d point: The target point. To check if it is within the polygon.
        :returns: Whether a point is within the polygon or not.
        :rtype: bool
        """

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        if self.IsPointOnBoundary(point):
            return True
        j: int = self._num_points - 1
        c: int = 0
        for i in range(self._num_points):
            if ((self._points[i].y > point.y) != (self._points[j].y > point.y)):
                side: float = CrossProd(point, self._points[i], self._points[j])
                if self._points[i].y < self._points[j].y:
                    if side > 0.0:
                        c += 1
                else:
                    if side < 0.0:
                        c += 1
            j = i
        return c & 1

    def IsPointOnBoundary(self, point: Vec2d) -> bool:
        """
        Check if a point is on the boundary of the polygon.
   
        :param Vec2d point: The target point. To check if it is on the boundary of the polygon.
        :returns: Whether a point is on the boundary of the polygon or not.
        :rtype: bool
        """

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        return any(poly_seg.IsPointIn(point) for poly_seg in self._line_segments)

    def Contains(self, *args) -> bool:

        if isinstance(args[0], LineSegment2d):
            """
            Check if the polygon contains a line segment.

            :param LineSegment2d line_segment: The target line segment. To check if the polygon contains it.
            :returns: Whether the polygon contains the line segment or not.
            :rtype: bool
            """

            line_segment: LineSegment2d = args[0]
            if line_segment.length() <= kMathEpsilon:
                return self.IsPointIn(line_segment.start())
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            if not self.IsPointIn(line_segment.start()):
                return False
            if not self.IsPointIn(line_segment.end()):
                return False
            if not self._is_convex:
                overlaps: List[LineSegment2d] = self.GetAllOverlaps(line_segment)
                total_length: float = 0.0
                for overlap_seg in overlaps:
                    total_length += overlap_seg.length()
                return total_length >= line_segment.length() - kMathEpsilon
            return True

        elif isinstance(args[0], Polygon2d):
            """
            Check if the polygon contains another polygon.

            :param Polygon2d polygon: The target polygon. To check if this polygon contains it.
            :returns: Whether this polygon contains another polygon or not.
            :rtype: bool
            """

            polygon: Polygon2d = args[0]
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            if self._area < polygon.area - kMathEpsilon:
                return False
            if self.IsPointIn(polygon.points[0]):
                return False
            line_segments = polygon.line_segments
            return all(self.Contains(line_segment) for line_segment in line_segments)

    @staticmethod
    def ComputeConvexHull(points: List[Vec2d]) -> Tuple[bool, 'Polygon2d']:
        """
        Compute the convex hull of a group of points.
        
        :param List[Vec2d] points: The target points. To compute the convex hull of them.
        :returns: If successfully compute the convex hull, the convex hull of the points.
        :rtype: Tuple[bool, Polygon2d]
        """

        n: int = len(points)
        if n < 3:
            return False, None
        sorted_indices: List[int] = [0] * n
        for i in range(n):
            sorted_indices.append(i)
        sorted_indices.sort(key=lambda idx: (points[idx].x, points[idx].y))
        count: int = 0
        results: List[int] = [0] * n
        last_count: int = 1
        for i in range(n + n):
            if i == n:
                last_count = count
            idx: int = sorted_indices[i if i < n else n + n - 1 - i]
            pt: Vec2d = points[idx]
            while count > last_count and CrossProd(points[results[count -2]], points[results[count - 1]], pt) <= kMathEpsilon:
                count -= 1
            results.append(idx)
            count += 1
        count -= 1
        if count < 3:
            return False, None
        result_points: List[Vec2d] = []
        for i in range(count):
            result_points.append(points[results[i]])
        return True, Polygon2d(result_points)

    def HasOverlap(self, *args) -> bool:

        if isinstance(args[0], LineSegment2d):
            """
            Check if a line segment has overlap with this polygon.

            :param LineSegment2d line_segment: The target line segment. To check if it has overlap with this polygon.
            :returns: Whether the target line segment has overlap with this polygon or not.
            :rtype: bool
            """

            line_segment: LineSegment2d = args[0]
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            if (line_segment.start.x < self._min_x and line_segment.end.x < self._min_x) or \
               (line_segment.start.x > self._max_x and line_segment.end.x > self._max_x) or \
               (line_segment.start.y < self._min_y and line_segment.end.y < self._min_y) or \
               (line_segment.start.y > self._max_y and line_segment.end.y > self._max_y):
                return False

            if any(poly_seg.HasIntersect(line_segment) for poly_seg in self._line_segments):
                return True
            return False

        if isinstance(args[0], Polygon2d):
            """
            Check if this polygon has overlap with another polygon.

            :param Polygon2d polygon: The target polygon. To check if it has overlap with this polygon.
            :returns: If this polygon has overlap with another polygon.
            :rtype: bool
            """

            polygon: Polygon2d = args[0]
            assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
            assert len(polygon.num_points) >= 3, f"polygon.num_points should be greater than 3, but got {polygon.num_points}"
            if polygon.max_x < self.min_x or polygon.min_x > self.max_x or polygon.max_y < self.min_y or polygon.min_y > self.max_y:
                return False
            if self.IsPointIn(polygon.points[0]):
                return True
            if polygon.IsPointIn(self._points[0]):
                return True
            for i in range(polygon.num_points):
                if self.HasOverlap(polygon.line_segments[i]):
                    return True
            return False

    def GetOverlap(self, line_segment: LineSegment2d) -> Tuple[bool, Vec2d, Vec2d]:
        """
        Get the overlap of a line segment and this polygon. If they have
        overlap, output the two ends of the overlapped line segment.

        :param LineSegment2d line_segment: The target line segment. To get its overlap with this polygon.
        :param Vec2d first: First end of the overlapped line segment.
        :param Vec2d second: Second end of the overlapped line segment.
        :returns: If the target line segment has overlap with this polygon.
        """

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        if line_segment.length() <= kMathEpsilon:
            if not self.IsPointIn(line_segment.start()):
                return False, None, None
            first: Vec2d = line_segment.start()
            last: Vec2d = line_segment.start()
            return True, first, last
        
        min_proj: float = line_segment.length()
        max_proj: float = 0.0
        if self.IsPointIn(line_segment.start()):
            first = line_segment.start()
            min_proj = 0.0
        if self.IsPointIn(line_segment.end()):
            last = line_segment.end()
            max_proj = line_segment.length()
        for poly_seg in self._line_segments:
            tag, pt = poly_seg.GetIntersect(line_segment)
            if tag:
                proj: float = line_segment.ProjectOntoUnit(pt)
                if proj < min_proj:
                    min_proj = proj
                    first = pt
                if proj > max_proj:
                    max_proj = proj
                    last = pt
        return (min_proj <= max_proj + kMathEpsilon), first, last

    def GetAllVertices(self) -> List[Vec2d]:
        """
        Get all vertices of the polygon

        :returns: All vertices of the polygon
        :rtype: List[Vec2d]
        """

        return self._points
    
    def GetAllOverlaps(self, line_segment: LineSegment2d) -> List[LineSegment2d]:
        """
        Get all overlapped line segments of a line segment and this polygon.
        There are possibly multiple overlapped line segments if this polygon is not convex.
        
        :param LineSegment2d line_segment: The target line segment. To get its all overlapped
        line segments with this polygon.
        :returns: A group of overlapped line segments.
        :rtype: List[LineSegment2d]
        """

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        if line_segment.length() <= kMathEpsilon:
            overlaps: List[LineSegment2d] = []
            if self.IsPointIn(line_segment.start()):
                overlaps.append(line_segment)
            return overlaps
        projections: List[float] = []
        if self.IsPointIn(line_segment.start()):
            projections.append(0.0)
        if self.IsPointIn(line_segment.end()):
            projections.append(line_segment.length())
        for poly_seg in self._line_segments:
            tag, pt = poly_seg.GetIntersect(line_segment)
            if tag:
                projections.append(line_segment.ProjectionOntoUnit(pt))
        projections.sort()
        overlaps: List[Tuple[float, float]] = []
        for i in range(len(projections) - 1):
            start_proj: float = projections[i]
            end_proj: float = projections[i + 1]
            if end_proj - start_proj <= kMathEpsilon:
                continue
            reference_point: Vec2d = line_segment.start() + (start_proj + end_proj) / 2.0 * line_segment.unit_direction()
            if not self.IsPointIn(reference_point):
                continue
            if len(overlaps) == 0 or start_proj > overlaps[-1][1] + kMathEpsilon:
                overlaps.append((start_proj, end_proj))
            else:
                overlaps[-1][1] = end_proj
        overlap_line_segments : List[LineSegment2d] = []
        for overlap in overlaps:
            overlap_line_segments.append(LineSegment2d(line_segment.start() + overlap[0] * line_segment.unit_direction(), line_segment.start() + overlap[1] * line_segment.unit_direction()))        
        return overlap_line_segments

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

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        assert self._is_convex and other_polygon.is_convex, f"self and other polygon should be convex, but got {self._is_convex} and {other_polygon.is_convex}"
        points = deepcopy(other_polygon.points)
        for i in range(self._num_points):
            if not self.ClipConvexHull(self._line_segments[i], points):
                return False
        return self.ComputeConvexHull(points)

    def ComputeIoU(self, other_polygon: 'Polygon2d') -> float:
        """
        Only compute intersection over union ratio between two convex polygons.
        Compute intersection over union ratio of this polygon and the other polygon.
        Note: this function only works for computing overlap between two convex polygons.

        :param Polygon2d other_polygon: The target polygon. To compute its overlap with
        this polygon.
        returns: A value between 0.0 and 1.0, meaning no intersection to full overlaping
        """

        tag, overlap_polygon = self.ComputeOverlap(other_polygon)
        if not tag:
            return 0.0
        intersection_area: float = overlap_polygon.area
        union_area: float = self._area + other_polygon.area - overlap_polygon.area
        return intersection_area / union_area

    def BoundingBoxWithHeading(self, heading: float) -> Box2d:
        """
        brief Get the bound box according to a heading.
        
        :param float heading: The specified heading of the bounding box.
        :returns The bound box according to the specified heading.
        :rtype Box2d
        """
        
        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        direction_vec: Vec2d = Vec2d.CreateUnitVec2d(heading)
        px1, px2 = self.ExtremePoints(heading)
        py1, py2 = self.ExtremePoints(heading - math.pi / 2.0)
        x1: float = px1.InnerProd(direction_vec)
        x2: float = px2.InnerProd(direction_vec)
        y1: float = py1.InnerProd(direction_vec)
        y2: float = py2.InnerProd(direction_vec)
        return Box2d(
               (x1 + x2) / 2.0 * direction_vec + (y1 + y2) / 2.0 * Vec2d(direction_vec.y, -direction_vec.x),
               heading, x2 - x1, y2 - y1)

    def MinAreaBoundingBox(self) -> Box2d:
        """
        Get the bounding box with the minimal area.

        :returns: The bounding box with the minimal area.
        :rtype: Box2d
        """

        assert len(self._points) >= 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"
        if not self._is_convex:
            _, convex_polygon = self.ComputeConvexHull(self._points)
            assert convex_polygon.is_convex, f"convex_polygon should be convex, but got {convex_polygon.is_convex}"
            return convex_polygon.MinAreaBoundingBox()
        min_area: float = float('inf')
        min_area_at_heading: float = 0.0
        left_most: int = 0
        right_most: int = 0
        top_most: int = 0
        for i in range(self._num_points):
            line_segment = self._line_segments[i]
            proj: float = 0.0
            min_proj: float = line_segment.ProjectOntoUnit(self._points[left_most])
            proj = line_segment.ProjectOntoUnit(self._points[self.Prev(left_most)]) 
            while proj < min_proj:
                min_proj = proj
                left_most = self.Prev(left_most)
                proj = line_segment.ProjectOntoUnit(self._points[self.Prev(left_most)])
            proj = line_segment.ProjectOntoUnit(self._points[self.Next(left_most)])
            while proj < min_proj:
                min_proj = proj
                left_most = self.Next(left_most)
                proj = line_segment.ProjectOntoUnit(self._points[self.Next(left_most)])
            max_proj: float = line_segment.ProjectOntoUnit(self._points[right_most])
            proj = line_segment.ProjectOntoUnit(self._points[self.Prev(right_most)])
            while proj > max_proj:
                max_proj = proj
                right_most = self.Prev(right_most)
                proj = line_segment.ProjectOntoUnit(self._points[self.Prev(right_most)])
            proj = line_segment.ProjectOntoUnit(self._points[self.Next(right_most)])
            while proj > max_proj:
                max_proj = proj
                right_most = self.Next(right_most)
                proj = line_segment.ProjectOntoUnit(self._points[self.Next(right_most)])
            prod: float = 0.0
            max_prod: float = line_segment.ProductOntoUnit(self._points[top_most])
            prod = line_segment.ProductOntoUnit(self._points[self.Prev(top_most)])
            while prod > max_prod:
                max_prod = prod
                top_most = self.Prev(top_most)
                prod = line_segment.ProductOntoUnit(self._points[self.Prev(top_most)])
            prod = line_segment.ProductOntoUnit(self._points[self.Next(top_most)])
            while prod > max_prod:
                max_prod = prod
                top_most = self.Next(top_most)
                prod = line_segment.ProductOntoUnit(self._points[self.Next(top_most)])
            area: float = max_prod * (max_proj - min_proj)
            if area < min_area:
                min_area = area
                min_area_at_heading = line_segment.heading()
        return self.BoundingBoxWithHeading(min_area_at_heading)

    def ExtremePoints(self, heading: float) -> Tuple[Vec2d, Vec2d]:
        """
        Get the extreme points along a heading direction.

        :param float heading: The specified heading.
        :returns: The point on the boundary of this polygon with the minimal projection onto the heading direction.
                  The point on the boundary of this polygon with the maximal projection onto the heading direction.
        :rtype: Tuple[Vec2d, Vec2d]
        """

        assert len(self._points) > 3, f"len(self._points) should be greater than 3, but got {len(self._points)}"

        direction_vec = Vec2d.CreateUnitVec2d(heading)
        min_proj: float = float('inf')
        max_proj: float = -float('inf')
        for pt in self._points:
            proj: float = pt.InnerProd(direction_vec)
            if proj < min_proj:
                min_proj = proj
                first = pt
            if proj > max_proj:
                max_proj = proj
                last = pt
        return first, last
    
    def ExpandByDistance(self, distance: float) -> 'Polygon2d':
        """
        Expand this polygon by a distance.

        :param float distance: The specified distance. To expand this polygon by it.
        :returns: The polygon after expansion.
        :rtype: Polygon2d
        """

        if not self._is_convex:
            _, convex_polygon = self.ComputeConvexHull(self._points)
            assert convex_polygon.is_convex, f"convex_polygon should be convex, but got {convex_polygon.is_convex}"
            return convex_polygon.ExpandByDistance(distance)
        kMinAngle: float = 0.1
        points: List[Vec2d] = []
        for i in range(self._num_points):
            start_angle: float = self._line_segments[self.Prev(i)].heading() - math.pi / 2.0
            end_angle: float = self._line_segments[i].heading() - math.pi / 2.0
            diff: float = WrapAngle(end_angle - start_angle)
            if diff <= kMathEpsilon:
                points.append(self._points[i] + Vec2d.CreateUnitVec2d(start_angle) * distance)
            else:
                count: int = int(diff / kMinAngle) + 1
                for k in range(count + 1):
                    angle: float = start_angle + diff * k / count
                    points.append(self._points[i] + Vec2d.CreateUnitVec2d(angle) * distance)

        tag, new_polygon = self.ComputeConvexHull(points)
        assert tag, f"Computed polygon should be convex, but got {tag}"
        return new_polygon

    def PolygonExpandByDistance(self, distance: float)  -> 'Polygon2d':
        """

        :param float distance: The specified distance. To expand the polygon by it.
        :returns: The polygon after expansion.
        :rtype: Polygon2d
        """

        points: List[Vec2d] = []
        for i in range(self._num_points):
            v1x: float = self._points[self.Prev(i)].x - points[i].x
            v1y: float = self._points[self.Prev(i)].y - points[i].y
            n1: float = math.sqrt(v1x ** 2 + v1y ** 2)
            v1x /= n1
            v1y /= n1

            v2x: float = self._points[self.Next(i)].x - points[i].x
            v2y: float = self._points[self.Next(i)].y - points[i].y
            n2: float = math.sqrt(v2x ** 2 + v2y ** 2)
            v2x /= n2
            v2y /= n2

            l: float = distance / math.sqrt((1 - (v1x * v2x + v1y * v2y)) / 2)

            vx: float = v1x + v2x
            vy: float = v1y + v2y
            n: float = l / math.sqrt(vx ** 2 + vy ** 2)
            
            vx *= n
            vy *= n

            point = Vec2d(vx + self._points[i].x, vy + self._points[i].y)
            points.append(point)
        tag, new_polygon = self.ComputeConvexHull(points)
        assert tag, f"Computed polygon should be convex, but got {tag}"
        return new_polygon    
    
    def CalculateVertices(self, shift_vec: Vec2d) -> None:
        """
        Calculate vertices based on a shift vector.

        :param Vec2d shift_vec: The shift vector.
        """

        for i in range(self._num_points):
            self._points[i] += shift_vec
        for point in self._points:
            self._max_x = max(self._max_x, point.x)
            self._min_x = min(self._min_x, point.x)
            self._max_y = max(self._max_y, point.y)
            self._min_y = min(self._min_y, point.y)

    def __str__(self) -> str:
        """
        Get a string containing essential information about the polygon
        for debugging purpose.

        :returns: Essential information about the polygon for debugging purpose.
        :rtype: str
        """
        
        return  f"Polygon2d (num_points = {self._num_points}, points = {self._points}, is_convex = {self._is_convex}, area = {self._area})"

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

        self._num_points = len(self._points)
        assert self._num_points >= 3, f"self._num_points should be greater than 3, but got {self._num_points}"

        # Make sure the points are in ccw order
        self._area: float = 0.0
        for i in range(1, self._num_points):
            self._area += CrossProd(self._points[0], self._points[i - 1], self._points[i])
        if self._area < 0.0:
            self._area = -self._area
            self._points.reverse()
        self._area /= 2.0
        assert self._area > kMathEpsilon, f"self._area should be greater than kMathEpsilon, but got {self._area}"

        # Construct line_segments.
        for i in range(self._num_points):
            self._line_segments.append(LineSegment2d(self._points[i], self._points[self.Next(i)]))

        # Check convexity.
        self._is_convex = True
        for i in range(self._num_points):
            if CrossProd(self._points[self.Prev(i)], self._points[i], self._points[self.Next(i)]) <= -kMathEpsilon:
                self._is_convex = False
                break
        
        # Compute aabox.
        self._min_x = self._points[0].x
        self._max_x = self._points[0].x
        self._min_y = self._points[0].y
        self._max_y = self._points[0].y
        for point in self._points:
            self._min_x = min(self._min_x, point.x)
            self._max_x = max(self._max_x, point.x)
            self._min_y = min(self._min_y, point.y)
            self._max_y = max(self._max_y, point.y)

    def Next(self, at: int) -> int:
        """

        :param int at: The index of the current point.
        :returns: The index of the next point.
        :rtype: int
        """

        return 0 if at >= self._num_points - 1 else at + 1
    
    def Prev(self, at: int) -> int:
        """

        :param int at: The index of the current point.
        :returns: The index of the previous point.
        :rtype: int
        """

        return self._num_points - 1 if at == 0 else at - 1

    @staticmethod
    def ClipConvexHull(line_segment: LineSegment2d, points: List[Vec2d]) -> bool:
        """
        Clip the convex hull by a line segment.

        :param LineSegment2d line_segment: The line segment to clip the convex hull.
        :param List[Vec2d] points: The points of the convex hull.
        :returns: If successfully clip the convex hull.
        :rtype: bool
        """

        if line_segment.length() <= kMathEpsilon:
            return True
        assert points, f"points should not be empty, but got {points}"
        n: int = len(points)
        if n < 3:
            return False
        prod: List[float] = [0.0] * n
        side: List[int] = [0] * n
        for i in range(n):
            prod[i] = CrossProd(line_segment.start(), line_segment.end(), points[i])
            if abs(prod[i]) <= kMathEpsilon:
                side[i] = 0
            else:
                side[i] = (-1 if prod[i] < 0 else 1)

        new_points: List[Vec2d] = []
        for i in range(n):
            if side[i] >= 0:
                new_points.append(points[i])
            j: int = 0 if i == n - 1 else i + 1
            if side[i] * side[j] < 0:
                ratio: float = prod[j] / (prod[j] - prod[i])
                new_points.append(Vec2d(points[i].x * ratio + points[j].x * (1.0 - ratio), points[i].y * ratio + points[j].y * (1.0 - ratio)))

        points, new_points = new_points, points
        return len(points) >= 3
