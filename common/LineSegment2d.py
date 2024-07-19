from common.Vec2d import Vec2d
from typing import Tuple

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

            raise NotImplementedError
        
        elif len(args) == 2:
            """
            Constructor with start point and end point.

            :param Vec2d start: The start point of the line segment.
            :param Vec2d end: The end point of the line segment.
            """

            self._start: Vec2d = args[0]
            self._end: Vec2d = args[1]
            self._unit_direction: Vec2d = None
            self._heading: float = 0.0
            self._length: float = 0.0
            raise NotImplementedError
        
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

        raise NotImplementedError
    
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

        raise NotImplementedError
    
    def length_sqr(self) -> float:
        """
        Get the square of length of the line segment.
        
        :returns: The square of length of the line segment.
        :rtype: float
        """

        raise NotImplementedError
    
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
        
        raise NotImplementedError

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

        raise NotImplementedError
    
    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Check if a point is within the line segment.

        :param Vec2d point: The point to check if it is within the line segment.
        :returns: Whether the input point is within the line segment or not.
        """

        raise NotImplementedError
    
    def HasIntersect(self, other_segment: 'LineSegment2d') -> bool:
        """
        Check if the line segment has an intersect with another line segment in 2-D.

        :param LineSegment2d other_segment: The line segment to check if it has an intersect.
        :returns: Whether the line segment has an intersect with the input other_segment.
        :rtype: bool
        """

        raise NotImplementedError
    
    def GetIntersect(self, other_segment: 'LineSegment2d') -> Tuple[bool, Vec2d]:
        """
        Compute the intersect with another line segment in 2-D if any.

        :param LineSegment2d other_segment: The line segment to compute the intersect.
        :returns: a tuple, the first element is whether the line segment has an intersect
                  with the input other_segment; the second element is the computed intersect between
                  the line segment and the input other_segment.
        :rtype: Tuple[bool, Vec2d]
        """

        raise NotImplementedError
    
    def ProjectOntoUnit(self, point: Vec2d) -> float:
        """
        Compute the projection of a vector onto the line segment.

        :param Vec2d point: The end of the vector (starting from the start point of the
                            line segment) to compute the projection onto the line segment.
        :returns: The projection of the vector, which is from the start point of the line
                  segment to the input point, onto the unit direction.
        :rtype: float
        """

        raise NotImplementedError
    
    def ProductOntoUnit(self, point: Vec2d) -> float:
        """
        Compute the cross product of a vector onto the line segment.

        :param Vec2d point: The end of the vector (starting from the start point of the
                            line segment) to compute the cross product onto the line segment.
        :returns: The cross product of the unit direction and the vector, which is from the
                  start point of the line segment to the input point.
        :rtype: float
        """

        raise NotImplementedError
    
    def GetPerpendicularFoot(self, point: Vec2d) -> Tuple[float, Vec2d]:
        """
        Compute perpendicular foot of a point in 2-D on the straight line expanded from the line segment.

        :param Vec2d point: The point to compute the perpendicular foot from.
        :returns: a tuple, the first element is the distance from the input point to the perpendicular foot;
                  the second element is the computed perpendicular foot from the input point to
                  the straight line expanded from the line segment.
        """

        raise NotImplementedError
    
    def __str__(self) -> str:
        """
        Get the debug string including the essential information.

        :returns: Information of the line segment for debugging.
        :rtype: str
        """

        raise NotImplementedError
