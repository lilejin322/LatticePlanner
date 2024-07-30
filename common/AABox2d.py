from common.Vec2d import Vec2d
from typing import List
from common.Polygon2d import kMathEpsilon
import math

class AABox2d:
    """
    Implements a class of (undirected) axes-aligned bounding boxes in 2-D.
    """

    def __init__(self, *args):
        """
        Constructor
        """

        if len(args) == 0:
            """
            Creates an axes-aligned box with zero length and width at the origin.
            """

            self._center = Vec2d()
            self._length = 0.0
            self._width = 0.0

        elif len(args) == 3:
            """
            Creates an axes-aligned box with given center, length, and width.

            :param Vec2d center: The center of the box.
            :param float length: The size of the box along the x-axis
            :param float width: The size of the box along the y-axis
            """

            center, length, width = args
            self._center: Vec2d = center
            self._length: float = length
            self._width: float = width
            self._half_length = self._length / 2
            self._half_width = self._width / 2
            assert self._length > -kMathEpsilon and self._width > -kMathEpsilon, f"AABox length and width must greater than 0, got length: {self._length}, width: {self._width}"

        elif len(args) == 2:
            """
            Creates an axes-aligned box from two opposite corners.

            :param Vec2d one_corner: One corner of the box
            :param Vec2d opposite_corner: The opposite corner to the first one
            """

            one_corner, opposite_corner = args
            self.__init__((one_corner.x + opposite_corner.x) / 2.0, abs(one_corner.x - opposite_corner.x),
                          abs(one_corner.y - opposite_corner.y))

        elif len(args) == 1 and isinstance(args[0], list):
            """
            Creates an axes-aligned box containing all points in a given vector.

            :param List[Vec2d] points: Vector of points to be included inside the box.
            """

            points = args[0]
            assert points, f"List points should not be empty!"
            min_x = points[0].x
            max_x = points[0].x
            min_y = points[0].y
            max_y = points[0].y
            for point in points:
                min_x = min(min_x, point.x)
                max_x = max(max_x, point.x)
                min_y = min(min_y, point.y)
                max_y = max(max_y, point.y)
            
            self._center = Vec2d((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
            self._length = max_x - min_x
            self._width = max_y - min_y
            self._half_length = self._length / 2.0
            self._half_width = self._width / 2.0

    @property
    def center(self) -> Vec2d:
        """
        Getter of _center

        :returns: Center of the box
        :rtype: Vec2d
        """

        return self._center

    @property
    def center_x(self) -> float:
        """
        Getter of x-component of _center

        :returns: x-component of the center of the box
        :rtype: float
        """

        return self._center.x

    @property
    def center_y(self) -> float:
        """
        Getter of y-component of _center

        :returns: y-component of the center of the box
        :rtype: float
        """

        return self._center.y

    @property
    def length(self) -> float:
        """
        Getter of _length

        :returns: The length of the box
        :rtype: float
        """

        return self._length

    @property
    def width(self) -> float:
        """
        Getter of _width

        :returns: The width of the box
        :rtype: float
        """

        return self._width

    @property
    def half_length(self) -> float:
        """
        Getter of _half_length

        :returns: Half of the length of the box
        :rtype: float
        """

        return self._half_length

    @property
    def half_width(self) -> float:
        """
         Getter of _half_width

        :returns: Half of the width of the box
        :rtype: float
        """

        return self._half_width

    @property
    def area(self) -> float:
        """
        Getter of _length*_width

        :returns: The area of the box
        :rtype: float
        """

        return self._length * self._width

    @property
    def min_x(self) -> float:
        """
        Returns the minimum x-coordinate of the box

        :returns: x-coordinate
        :rtype: float
        """

        return self._center.x - self._half_length

    @property
    def max_x(self) -> float:
        """
        Returns the maximum x-coordinate of the box

        :returns: x-coordinate
        :rtype: float
        """

        return self._center.x + self._half_length

    @property
    def min_y(self) -> float:
        """
        Returns the minimum y-coordinate of the box

        :returns: y-coordinate
        :rtype: float
        """

        return self._center.y - self._half_width

    @property
    def max_y(self) -> float:
        """
        Returns the maximum y-coordinate of the box

        :returns: y-coordinate
        :rtype: float
        """

        return self._center.y + self._half_width

    def GetAllCorners(self) -> List[Vec2d]:
        """
        Gets all corners in counter clockwise order.

        :returns: List of all corners
        :rtype: List[Vec2d]
        """

        corners: List[Vec2d] = []
        corners.append(Vec2d(self._center.x + self._half_length, self._center.y - self._half_width))
        corners.append(Vec2d(self._center.x + self._half_length, self._center.y + self._half_width))
        corners.append(Vec2d(self._center.x - self._half_length, self._center.y + self._half_width))
        corners.append(Vec2d(self._center.x - self._half_length, self._center.y - self._half_width))
        return corners

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Determines whether a given point is in the box.

        :param Vec2d point: The point we wish to test for containment in the box
        :returns: True if the point is inside the box, False otherwise
        :rtype: bool
        """

        return abs(point.x - self._center.x) <= self._half_length + kMathEpsilon and \
               abs(point.y - self._center.y) <= self._half_width + kMathEpsilon

    def IsPointOnBoundary(self, point: Vec2d) -> bool:
        """
        Determines whether a given point is on the boundary of the box.

        :param Vec2d point: The point we wish to test for boundary membership
        :returns: True if the point is on the boundary of the box, False otherwise
        :rtype: bool
        """

        dx: float = abs(point.x - self._center.x)
        dy: float = abs(point.y - self._center.y)
        tag1 = abs(dx - self._half_length) <= kMathEpsilon and dy <= self._half_width + kMathEpsilon
        tag2 = abs(dy - self._half_width) <= kMathEpsilon and dx <= self._half_length + kMathEpsilon
        return tag1 or tag2

    def DistanceTo(self, *args) -> float:

        if isinstance([0], Vec2d):
            """
            Determines the distance between a point and the box.

            :param Vec2d point: The point whose distance to the box we wish to determine.
            :returns: The distance between the point and the box
            :rtype: float
            """

            point: Vec2d = args[0]
            dx: float = abs(point.x - self._center.x) - self._half_length
            dy: float = abs(point.y - self._center.y) - self._half_width
            if dx <= 0.0:
                return max(0.0, dy)
            if dy <= 0.0:
                return dx
            return math.hypot(dx, dy)

        elif isinstance([0], AABox2d):
            """
            Determines the distance between two boxes.

            :param AABox2d box: Another box.
            :returns: The distance between the two boxes
            :rtype: float
            """

            box: AABox2d = args[0]
            dx: float = abs(box.center_x - self._center.x) - box.half_length - self._half_length
            dy: float = abs(box.center_y - self._center.y) - box.half_width - self._half_width
            if dx <= 0.0:
                return max(0.0, dy)
            if dy <= 0.0:
                return dx
            return math.hypot(dx, dy)

        else:
            raise ValueError("Invalid arguments")

    def HasOverlap(self, box: 'AABox2d') -> bool:
        """
        Determines whether two boxes overlap.

        :param AABox2d box: Another box.
        :returns: True if the two boxes overlap, False otherwise
        :rtype: bool
        """

        return abs(box.center_x - self._center.x) <= box.half_length + self._half_length and \
               abs(box.center_y - self._center.y) <= box.half_width + self._half_width

    def Shift(self, shift_vec: Vec2d):
        """
        Shift the center of AABox by the input vector.

        :param Vec2d shift_vec: The vector by which we wish to shift the box
        """

        self._center += shift_vec

    def MergeFrom(self, *args) -> None:

        if isinstance(args[0], AABox2d):
            """
            Changes box to include another given box, as well as the current one.

            :param AABox2d other_box: Another box.
            """

            other_box: AABox2d = args[0]
            x1: float = min(self.min_x, other_box.min_x)
            x2: float = max(self.max_x, other_box.max_x)
            y1: float = min(self.min_y, other_box.min_y)
            y2: float = max(self.max_y, other_box.max_y)
            self._center = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0)
            self._length = x2 - x1
            self._width = y2 - y1
            self._half_length = self._length / 2.0
            self._half_width = self._width / 2.0

        elif isinstance(args[0], Vec2d):
            """
            Changes box to include a given point, as well as the current box.

            :param Vec2d other_point: Another point.
            """

            other_point: Vec2d = args[0]
            x1: float = min(self.min_x, other_point.x)
            x2: float = max(self.max_x, other_point.x)
            y1: float = min(self.min_y, other_point.y)
            y2: float = max(self.max_y, other_point.y)
            self._center = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0)
            self._length = x2 - x1
            self._width = y2 - y1
            self._half_length = self._length / 2.0
            self._half_width = self._width / 2.0

    def __str__(self) -> str:
        """
        Returns a string representation of the AABox2d.

        :returns: String representation of the AABox2d
        :rtype: str
        """

        return f"AABox2d(center = {self._center}, length = {self._length}, width = {self._width})"
