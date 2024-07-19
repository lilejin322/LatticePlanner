from common.Vec2d import Vec2d, kMathEpsilon
from typing import Tuple
from math import hypot
from common.Polygon2d import CrossProd

def IsWithin(val: float, bound1: float, bound2:float):
    """
    Check if a value is within the range of two bounds.
    """

    if bound1 > bound2:
        bound1, bound2 = bound2, bound1
    
    return bound1 - kMathEpsilon <= val <= bound2 + kMathEpsilon

class LineSegment2d:
    """
    LineSegment2d class, Line segment in 2-D.
    """

    def __init__(self, *args):
        """
        Constructor
        """

        if len(args) == 0:
            """
            Empty constructor
            """

            self._unit_direction = Vec2d(1, 0)
            self._heading: float = 0.0
            self._length: float = 0.0
            self._start: Vec2d = None
            self._end: Vec2d = None
        
        elif len(args) == 2:
            """
            Constructor with start point and end point.

            :param Vec2d start: The start point of the line segment.
            :param Vec2d end: The end point of the line segment.
            """

            self._start: Vec2d = args[0]
            self._end: Vec2d = args[1]
            dx: float = self._end.x - self._start.x
            dy: float = self._end.y - self._start.y
            self._length = hypot(dx, dy)
            self._unit_direction: Vec2d = Vec2d(0, 0) if self._length <= kMathEpsilon else Vec2d(dx / self._length, dy / self._length)
            self._heading = self._unit_direction.Angle()
        
        else:
            raise ValueError("Invalid number of arguments")
    
    @property
    def start(self) -> Vec2d:
        """
        Get the start point.

        :returns: The start point of the line segment.
        :rtype: Vec2d
        """

        return self._start
    
    @property
    def end(self) -> Vec2d:
        """
        Get the end point.

        :returns: The end point of the line segment.
        :rtype: Vec2d
        """

        return self._end
    
    @property
    def unit_direction(self) -> Vec2d:
        """
        Get the unit direction from the start point to the end point.

        :returns: The start point of the line segment.
        :rtype: Vec2d
        """

        return self._unit_direction
    
    @property
    def center(self) -> Vec2d:
        """
        Get the center of the line segment.

        :returns: The center of the line segment.
        :rtype: Vec2d
        """

        return (self._start + self._end) / 2.0
    
    def rotate(self, angle: float) -> Vec2d:
        """
        Get a new line-segment with the same start point, but rotated
        counterclock-wise by the given amount.

        :param float angle: The angle to rotate the line-segment by.
        :returns: The rotated line-segment's end-point.
        :rtype: Vec2d
        """

        diff_vec: Vec2d = self._end - self._start
        diff_vec.SelfRotate(angle)
        return self._start + diff_vec
    
    def heading(self) -> float:
        """
        Get the heading of the line segment.

        :returns: The heading, which is the angle between unit direction and x-axis.
        :rtype: float
        """

        return self._heading
    
    def cos_heading(self) -> float:
        """
        Get the cosine of the heading.

        :returns: The cosine of the heading.
        :rtype: float
        """

        return self._unit_direction.x
    
    def sin_heading(self) -> float:
        """
        Get the sine of the heading.

        :returns: The sine of the heading.
        :rtype: float
        """

        return self._unit_direction.y
    
    def length(self) -> float:
        """
        Get the length of the line segment.
        
        :returns: The length of the line segment.
        :rtype: float
        """

        return self._length
    
    def length_sqr(self) -> float:
        """
        Get the square of length of the line segment.
        
        :returns: The square of length of the line segment.
        :rtype: float
        """

        return self._length * self._length
    
    def DistanceTo(self, point: Vec2d) -> Tuple[float, Vec2d]:
        """
        Compute the shortest distance from a point on the line segment
        to a point in 2-D, and get the nearest point on the line segment.

        :param Vec2d point: The point to compute the distance to.
        :returns: a tuple, the first element is the shortest distance from
                  points on the line segment to the input point; the second element is
                  the nearest point on the line segment to the input point.
        :rtype: Tuple[float, Vec2d]
        """
        
        if self._length <= kMathEpsilon:
            return point.DistanceTo(self._start), self._start
        x0: float = point.x - self._start.x
        y0: float = point.y - self._start.y
        proj: float = x0 * self._unit_direction.x + y0 * self._unit_direction.y
        if proj <= 0.0:
            return hypot(x0, y0), self._start
        if proj >= self._length:
            return point.DistanceTo(self._end), self._end
        return abs(x0 * self._unit_direction.y - y0 * self._unit_direction.x), self._start + self._unit_direction * proj

    def DistanceSquareTo(self, point: Vec2d) -> Tuple[float, Vec2d]:
        """
        Compute the square of the shortest distance from a point on the line
        segment to a point in 2-D, and get the nearest point on the line segment.

        :param Vec2d point: The point to compute the squared of the distance to.
        :returns: a tuple, the first element is the the square of the shortest distance
                  from points on the line segment to the input point; the nearest point on the
                  line segment to the input point.
        :rtype: Tuple[float, Vec2d]
        """

        if self._length <= kMathEpsilon:
            return point.DistanceSquareTo(self._start), self._start
        x0: float = point.x - self._start.x
        y0: float = point.y - self._start.y
        proj: float = x0 * self._unit_direction.x + y0 * self._unit_direction.y
        if proj <= 0:
            return x0 ** 2 + y0 ** 2, self._start
        if proj >= self._length:
            return point.DistanceSquareTo(self._end), self._end
        return (x0 * self._unit_direction.y - y0 * self._unit_direction.x) ** 2, self._start + self._unit_direction * proj

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Check if a point is within the line segment.

        :param Vec2d point: The point to check if it is within the line segment.
        :returns: Whether the input point is within the line segment or not.
        """

        if self._length <= kMathEpsilon:
            tag1: bool = abs(point.x - self._start.x) <= kMathEpsilon
            tag2: bool = abs(point.y - self._start.y) <= kMathEpsilon
            return tag1 and tag2
        prod: float = CrossProd(point - self._start, self._end)
        if abs(prod) > kMathEpsilon:
            return False
        tag1: bool = IsWithin(point.x, self._start.x, self._end.x)
        tag2: bool = IsWithin(point.y, self._start.y, self._end.y)
        return tag1 and tag2

    def HasIntersect(self, other_segment: 'LineSegment2d') -> bool:
        """
        Check if the line segment has an intersect with another line segment in 2-D.

        :param LineSegment2d other_segment: The line segment to check if it has an intersect.
        :returns: Whether the line segment has an intersect with the input other_segment.
        :rtype: bool
        """

        tag, _ = self.GetIntersect(other_segment)
        return tag

    def GetIntersect(self, other_segment: 'LineSegment2d') -> Tuple[bool, Vec2d]:
        """
        Compute the intersect with another line segment in 2-D if any.

        :param LineSegment2d other_segment: The line segment to compute the intersect.
        :returns: a tuple, the first element is whether the line segment has an intersect
                  with the input other_segment; the second element is the computed intersect between
                  the line segment and the input other_segment.
        :rtype: Tuple[bool, Vec2d]
        """

        if self.IsPointIn(other_segment.start):
            return True, other_segment.start
        if self.IsPointIn(other_segment.end):
            return True, other_segment.end
        if other_segment.IsPointIn(self._start):
            return True, self._start
        if other_segment.IsPointIn(self._end):
            return True, self._end
        if self._length <= kMathEpsilon or other_segment.length() <= kMathEpsilon:
            return False, None
        cc1: float = CrossProd(self._start, self._end, other_segment.start)
        cc2: float = CrossProd(self._start, self._end, other_segment.end)
        if cc1 * cc2 >= -kMathEpsilon:
            return False, None
        cc3: float = CrossProd(other_segment.start, other_segment.end, self._start)
        cc4: float = CrossProd(other_segment.start, other_segment.end, self._end)
        if cc3 * cc4 >= -kMathEpsilon:
            return False, None
        ratio: float = cc4 / (cc4 - cc3)
        return True, Vec2d(self._start.x * ratio + self._end.x * (1.0 - ratio), self._start.y * ratio + self._end.y * (1.0 - ratio))

    def ProjectOntoUnit(self, point: Vec2d) -> float:
        """
        Compute the projection of a vector onto the line segment.

        :param Vec2d point: The end of the vector (starting from the start point of the
                            line segment) to compute the projection onto the line segment.
        :returns: The projection of the vector, which is from the start point of the line
                  segment to the input point, onto the unit direction.
        :rtype: float
        """

        return self._unit_direction.InnerProd(point - self._start)
    
    def ProductOntoUnit(self, point: Vec2d) -> float:
        """
        Compute the cross product of a vector onto the line segment.

        :param Vec2d point: The end of the vector (starting from the start point of the
                            line segment) to compute the cross product onto the line segment.
        :returns: The cross product of the unit direction and the vector, which is from the
                  start point of the line segment to the input point.
        :rtype: float
        """

        return self._unit_direction.CrossProd(point - self._start)

    def GetPerpendicularFoot(self, point: Vec2d) -> Tuple[float, Vec2d]:
        """
        Compute perpendicular foot of a point in 2-D on the straight line expanded from the line segment.
        return distance with perpendicular foot point.
        :param Vec2d point: The point to compute the perpendicular foot from.
        :returns: a tuple, the first element is the distance from the input point to the perpendicular foot;
                  the second element is the computed perpendicular foot from the input point to
                  the straight line expanded from the line segment.
        """

        if self._length <= kMathEpsilon:
            return point.DistanceTo(self._start), self._start
        x0: float = point.x - self._start.x
        y0: float = point.y - self._start.y
        proj: float = x0 * self._unit_direction.x + y0 * self._unit_direction.y
        return abs(x0 * self._unit_direction.y - y0 * self._unit_direction.x), self._start + self._unit_direction * proj

    def __str__(self) -> str:
        """
        Get the debug string including the essential information.

        :returns: Information of the line segment for debugging.
        :rtype: str
        """

        return f"segment2d: start: {self._start}, end: {self._end}"
