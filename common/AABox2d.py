from common.Vec2d import Vec2d
from typing import List

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

        elif len(args) == 2:
            """
            Creates an axes-aligned box from two opposite corners.

            :param Vec2d one_corner: One corner of the box
            :param Vec2d opposite_corner: The opposite corner to the first one
            """

            one_corner, opposite_corner = args
            self._center = Vec2d((one_corner.x + opposite_corner.x) / 2,
                                 (one_corner.y + opposite_corner.y) / 2)
            self._length = abs(one_corner.x - opposite_corner.x)
            self._width = abs(one_corner.y - opposite_corner.y)

        elif len(args) == 1 and isinstance(args[0], list):
            """
            Creates an axes-aligned box containing all points in a given vector.

            :param List[Vec2d] points: Vector of points to be included inside the box.
            """

            points = args[0]
            min_x = min(point.x for point in points)
            max_x = max(point.x for point in points)
            min_y = min(point.y for point in points)
            max_y = max(point.y for point in points)
            self._center = Vec2d((min_x + max_x) / 2, (min_y + max_y) / 2)
            self._length = max_x - min_x
            self._width = max_y - min_y

        self._half_length = self._length / 2
        self._half_width = self._width / 2

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

        return [
            Vec2d(self.min_x, self.min_y),
            Vec2d(self.min_x, self.max_y),
            Vec2d(self.max_x, self.max_y),
            Vec2d(self.max_x, self.min_y)
        ]

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Determines whether a given point is in the box.

        :param Vec2d point: The point we wish to test for containment in the box
        :returns: True if the point is inside the box, False otherwise
        :rtype: bool
        """

        return (self.min_x <= point.x <= self.max_x and
                self.min_y <= point.y <= self.max_y)

    def IsPointOnBoundary(self, point: Vec2d) -> bool:
        """
        Determines whether a given point is on the boundary of the box.

        :param Vec2d point: The point we wish to test for boundary membership
        :returns: True if the point is on the boundary of the box, False otherwise
        :rtype: bool
        """

        return (self.min_x == point.x or self.max_x == point.x or
                self.min_y == point.y or self.max_y == point.y)

    def DistanceTo(self, *args) -> float:

        if isinstance([0], Vec2d):
            """
            Determines the distance between a point and the box.

            :param Vec2d point: The point whose distance to the box we wish to determine.
            :returns: The distance between the point and the box
            :rtype: float
            """

            point: Vec2d = args[0]
            dx = max(self.min_x - point.x, 0, point.x - self.max_x)
            dy = max(self.min_y - point.y, 0, point.y - self.max_y)
            return (dx ** 2 + dy ** 2) ** 0.5

        elif isinstance([0], AABox2d):
            """
            Determines the distance between two boxes.

            :param AABox2d box: Another box.
            :returns: The distance between the two boxes
            :rtype: float
            """

            box: AABox2d = args[0]
            dx = max(self.min_x - box.max_x, 0, box.min_x - self.max_x)
            dy = max(self.min_y - box.max_y, 0, box.min_y - self.max_y)
            return (dx ** 2 + dy ** 2) ** 0.5

        else:
            raise ValueError("Invalid arguments")

    def HasOverlap(self, box: 'AABox2d') -> bool:
        """
        Determines whether two boxes overlap.

        :param AABox2d box: Another box.
        :returns: True if the two boxes overlap, False otherwise
        :rtype: bool
        """

        return not (self.max_x < box.min_x or self.min_x > box.max_x or
                    self.max_y < box.min_y or self.min_y > box.max_y)

    def Shift(self, shift_vec: Vec2d):
        """
        Shift the center of AABox by the input vector.

        :param Vec2d shift_vec: The vector by which we wish to shift the box
        """

        self._center = Vec2d(self._center.x + shift_vec.x,
                             self._center.y + shift_vec.y)

    def MergeFrom(self, *args) -> None:

        if isinstance(args[0], AABox2d):
            """
            Changes box to include another given box, as well as the current one.

            :param AABox2d other_box: Another box.
            """

            other_box: AABox2d = args[0]
            new_min_x = min(self.min_x, other_box.min_x)
            new_max_x = max(self.max_x, other_box.max_x)
            new_min_y = min(self.min_y, other_box.min_y)
            new_max_y = max(self.max_y, other_box.max_y)
            self._center = Vec2d((new_min_x + new_max_x) / 2, (new_min_y + new_max_y) / 2)
            self._length = new_max_x - new_min_x
            self._width = new_max_y - new_min_y
            self._half_length = self._length / 2
            self._half_width = self._width / 2

        elif isinstance(args[0], Vec2d):
            """
            Changes box to include a given point, as well as the current box.

            :param Vec2d other_point: Another point.
            """

            other_point: Vec2d = args[0]
            new_min_x = min(self.min_x, other_point.x)
            new_max_x = max(self.max_x, other_point.x)
            new_min_y = min(self.min_y, other_point.y)
            new_max_y = max(self.max_y, other_point.y)
            self._center = Vec2d((new_min_x + new_max_x) / 2, (new_min_y + new_max_y) / 2)
            self._length = new_max_x - new_min_x
            self._width = new_max_y - new_min_y
            self._half_length = self._length / 2
            self._half_width = self._width / 2

    def __str__(self) -> str:
        """
        Returns a string representation of the AABox2d.

        :returns: String representation of the AABox2d
        :rtype: str
        """

        return (f"AABox2d(center={self._center}, length={self._length}, width={self._width}, "
                f"half_length={self._half_length}, half_width={self._half_width})")
