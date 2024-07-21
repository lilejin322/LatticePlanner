from common.Vec2d import Vec2d
from typing import List, Any
from math import hypot
from common.LineSegment2d import LineSegment2d
from common.AABox2d import AABox2d

class Box2d:
    """
    class Box2d
    Rectangular (undirected) bounding box in 2-D.
    This class is referential-agnostic, although our convention on the use of
    the word "heading" in this project (permanently set to be 0 at East)
    forces us to assume that the X/Y frame here is East/North.
    For disambiguation, we call the axis of the rectangle parallel to the
    heading direction the "heading-axis". The size of the heading-axis is
    called "length", and the size of the axis perpendicular to it "width".
    """

    def __init__(self, *args):

        if len(args) == 4:
            """
            Constructor which takes the center, heading, length and width.
            
            :param Vec2d center: The center of the rectangular bounding box.
            :param float heading: The angle between the x-axis and the heading-axis, measured counter-clockwise.
            :param float length: The size of the heading-axis.
            :param float width: The size of the axis perpendicular to the heading-axis.
            """

            center, heading, length, width = args
            raise NotImplementedError
        
        if len(args) == 5:
            """
            Constructor which takes the point on the axis, front length, back length, heading, and width.
   
            :param Vec2d point: The center of the rectangular bounding box.
            :param float heading: The angle between the x-axis and the heading-axis, measured counter-clockwise.
            :param float front_length: The length from the start point to the given point.
            :param float back_length: The length from the end point to the given point.
            :param float width: The size of the axis perpendicular to the heading-axis.
            """

            point, heading, front_length, back_length, width = args
            raise NotImplementedError

        if len(args) == 2:
            """
            Constructor which takes the heading-axis and the width of the box
            
            :param LineSegment2d axis: The heading-axis
            :param float width: The width of the box, which is taken perpendicularly to the heading direction.
            """
            
            axis, width = args
            raise NotImplementedError
        
        if len(args) == 1:
            """
            Constructor which takes an AABox2d (axes-aligned box).

            :param AABox2d aabox: The input AABox2d.
            """

            aabox = args[0]

    @staticmethod
    def CreateAABox(one_corner: Vec2d, opposite_corner: Vec2d) -> 'Box2d':
        """
        Creates an axes-aligned Box2d from two opposite corners
   
        :param Vec2d one_corner: One of the corners
        :param Vec2d opposite_corner: The opposite corner to the first one
        :returns: An axes-aligned Box2d
        :rtype: Box2d
        """

        raise NotImplementedError

    @property
    def center(self) -> Vec2d:
        """
        Getter of the center of the box

        returns: The center of the box
        rtype: Vec2d
        """

        return self._center

    @property
    def center_x(self) -> float:
        """
        Getter of the x-coordinate of the center of the box

        :returns: The x-coordinate of the center of the box
        :rtype: float
        """

        return self._center.x

    @property
    def center_y(self) -> float:
        """
        Getter of the y-coordinate of the center of the box

        :returns: The y-coordinate of the center of the box
        :rtype: float
        """

        return self._center.y

    @property
    def length(self) -> float:
        """
        Getter of the length

        :returns: The length of the heading-axis
        :rtype: float
        """

        return self._length

    @property
    def width(self) -> float:
        """
        Getter of the width

        :returns: The width of the box taken perpendicularly to the heading
        :rtype: float
        """

        return self._width

    @property
    def half_length(self) -> float:
        """
        Getter of half the length
   
        :returns: Half the length of the heading-axis
        :rtype: float
        """

        return self._half_length

    @property
    def half_width(self) -> float:
        """
        Getter of half the width

        :returns: Half the width of the box taken perpendicularly to the heading
        :rtype: float
        """

        return self._half_width

    @property
    def heading(self) -> float:
        """
        Getter of the heading

        :returns: The counter-clockwise angle between the x-axis and the heading-axis
        :rtype: float
        """

        return self._heading

    @property
    def cos_heading(self) -> float:
        """
        Getter of the cosine of the heading

        :returns: The cosine of the heading
        :rtype: float
        """

        return self._cos_heading

    @property
    def sin_heading(self) -> float:
        """
        Getter of the sine of the heading

        :returns: The sine of the heading
        :rtype: float
        """

        return self._sin_heading

    @property
    def area(self) -> float:
        """
        Getter of the area of the box

        :returns: The product of its length and width
        :rtype: float
        """

        return self._length * self._width

    @property
    def diagonal(self) -> float:
        """
        Getter of the size of the diagonal of the box

        :returns: The diagonal size of the box
        :rtype: float
        """

        return hypot(self._length, self._width)

    def GetAllCorners(self) -> List[Vec2d]:
        """
        Getter of the corners of the box

        :returns: The vector where the corners are listed
        :rtype: List[Vec2d]
        """

        raise NotImplementedError

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Tests points for membership in the box
        
        :param Vec2d point: A point that we wish to test for membership in the box
        :returns: True iff the point is contained in the box
        :rtype: bool
        """

        raise NotImplementedError

    def IsPointOnBoundary(self, point: Vec2d) -> bool:
        """
        Tests points for membership in the boundary of the box
        
        :param Vec2d point: A point that we wish to test for membership in the boundary
        :returns: True iff the point is a boundary point of the box
        :rtype: bool
        """

        raise NotImplementedError

    def DistanceTo(self, param: Any) -> float:

        if isinstance(param, Vec2d):
            """
            Determines the distance between the box and a given point

            :param Vec2d point: The point whose distance to the box we wish to compute
            :returns: A distance
            :rtype: float
            """

            raise NotImplementedError

        elif isinstance(param, LineSegment2d):
            """
            Determines the distance between the box and a given line segment

            :param LineSegment2d line_segment: The line segment whose distance to the box we compute
            :returns: A distance
            :rtype: float 
            """

            raise NotImplementedError
        
        elif isinstance(param, Box2d):
            """
            Determines the distance between two boxes

            :param Box2d box: The box whose distance to this box we want to compute
            :returns: A distance
            :rtype: float 
            """

            raise NotImplementedError
        
        else:

            raise ValueError("Unknown param type")

    def HasOverlap(self, param: Any) -> bool:

        if isinstance(param, LineSegment2d):
            """
            Determines whether this box overlaps a given line segment

            :param LineSegment2d line_segment: The line-segment
            :returns: True if they overlap
            :rtype: bool
            """

            raise NotImplementedError
        
        elif isinstance(param, Box2d):
            """
            Determines whether these two boxes overlap

            :param Box2d box: The other box
            :returns: True if they overlap
            :rtype: bool
            """

            raise NotImplementedError
        
        else:

            raise ValueError("Unknown param type")

    def GetAABox(self) -> AABox2d:
        """
        Gets the smallest axes-aligned box containing the current one
        
        :returns: An axes-aligned box
        :rtype: AABox2d
        """

        raise NotImplementedError

    def RotateFromCenter(self, rotate_angle: float) -> None:
        """
        Rotate from center.

        :param float rotate_angle: Angle to rotate.
        """
        
        raise NotImplementedError

    def Shift(self, shift_vec: Vec2d) -> None:
        """
        Shifts this box by a given vector

        :param Vec2d shift_vec: The vector determining the shift
        """

        raise NotImplementedError

    def LongitudinalExtend(self, extension_length: float) -> None:
        """
        Extend the box longitudinally

        :param float extension_length: the length to extend
        """

        raise NotImplementedError

    def LateralExtend(self, extension_length: float) -> None:
        """
        Extend the box laterally

        :param float extension_length: the length to extend
        """
        
        raise NotImplementedError

    def __str__(self) -> str:
        """
        Debug string representation

        :returns: A string representation of class Box2d
        :rtype: str
        """

        raise NotImplementedError

    def InitCorners(self) -> None:
        """
        Initialize the corners of the box
        """

        raise NotImplementedError

    @property
    def max_x(self) -> float:
        """
        Getter of the maximum x-coordinate of the box

        :returns: The maximum x-coordinate of the box
        :rtype: float
        """

        return self._max_x

    @property
    def min_x(self) -> float:
        """
        Getter of the minimum x-coordinate of the box

        :returns: The minimum x-coordinate of the box
        :rtype: float
        """

        return self._min_x

    @property
    def max_y(self) -> float:
        """
        Getter of the maximum y-coordinate of the box

        :returns: The maximum y-coordinate of the box
        :rtype: float
        """

        return self._max_y

    @property
    def min_y(self) -> float:
        """
        Getter of the minimum y-coordinate of the box

        :returns: The minimum y-coordinate of the box
        :rtype: float
        """

        return self._min_y

    def is_inside_rectangle(self, point: Vec2d) -> bool:
        """
        Check if a point is inside the rectangle

        :param Vec2d point: The point to check
        :returns: True if the point is inside the rectangle
        :rtype: bool
        """

        return (0.0 <= point.x <= self._width) and (0.0 <= point.y() <= self._length)
