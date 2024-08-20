from common.Vec2d import Vec2d, kMathEpsilon
from typing import List, Any
from math import hypot, cos, sin, fmod, pi
from common.LineSegment2d import LineSegment2d
from common.AABox2d import AABox2d
from common.Polygon2d import Polygon2d
from logging import Logger

logger: Logger = Logger("Box2d")

def NormalizeAngle(angle: float) -> float:
    """
    Normalize the angle to [-pi, pi]

    :param float angle: the angle to normalize
    :returns: the normalized angle
    :rtype: float
    """

    a: float = fmod(angle + pi, 2 * pi)
    if a < 0.0:
        a += 2 * pi
    return a - pi

def PtSegDistance(query_x: float, query_y: float, start_x: float, start_y: float, end_x: float, end_y: float, length: float):
    """
    Compute the distance between a point and a line segment

    :param float query_x: The x-coordinate of the point
    :param float query_y: The y-coordinate of the point
    :param float start_x: The x-coordinate of the start point of the line segment
    :param float start_y: The y-coordinate of the start point of the line segment
    :param float end_x: The x-coordinate of the end point of the line segment
    :param float end_y: The y-coordinate of the end point of the line segment
    :param float length: The length of the line segment
    :returns: The distance between the point and the line segment
    :rtype: float
    """

    x0 = query_x - start_x
    y0 = query_y - start_y
    dx = end_x - start_x
    dy = end_y - start_y
    proj = x0 * dx + y0 * dy
    
    if proj <= 0.0:
        return hypot(x0, y0)
    if proj >= length ** 2:
        return hypot(x0 - dx, y0 - dy)

    return abs(x0 * dy - y0 * dx) / length

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

        self._corners: List[Vec2d] = []
        self._max_x: float = float('-inf')
        self._min_x: float = float('inf')
        self._max_y: float = float('-inf')
        self._min_y: float = float('inf')

        if len(args) == 4:
            """
            Constructor which takes the center, heading, length and width.
            
            :param Vec2d center: The center of the rectangular bounding box.
            :param float heading: The angle between the x-axis and the heading-axis, measured counter-clockwise.
            :param float length: The size of the heading-axis.
            :param float width: The size of the axis perpendicular to the heading-axis.
            """

            center, heading, length, width = args
            self._center: Vec2d = center
            self._length: float = length
            self._width: float = width
            self._half_length: float = length / 2.0
            self._half_width: float = width / 2.0
            self._heading: float = heading
            self._cos_heading: float = cos(heading)
            self._sin_heading: float = sin(heading)
            assert self._length >= -kMathEpsilon
            assert self.width >= -kMathEpsilon
            self.InitCorners()

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
            self._length = front_length + back_length
            self._width = width
            self._half_length = length / 2.0
            self._half_width = width / 2.0
            self._heading = heading
            self._cos_heading = cos(heading)
            self._sin_heading = sin(heading)
            assert self._length >= -kMathEpsilon
            assert self._width >= -kMathEpsilon
            delta_length: float = (front_length - back_length) / 2.0
            self._center = Vec2d(point.x + self._cos_heading * delta_length, point.y + self._sin_heading * delta_length)
            self.InitCorners()

        if len(args) == 2:
            """
            Constructor which takes the heading-axis and the width of the box
            
            :param LineSegment2d axis: The heading-axis
            :param float width: The width of the box, which is taken perpendicularly to the heading direction.
            """
            
            axis, width = args
            self._center = axis.center
            self._length = axis.length
            self._width = width
            self._half_length = axis.length / 2.0
            self._half_width = width / 2.0
            self._heading = axis.heading
            self._cos_heading = axis.cos_heading
            self._sin_heading = axis.sin_heading
            assert self._length >= -kMathEpsilon
            assert self._width >= -kMathEpsilon
            self.InitCorners()

        if len(args) == 1:
            """
            Constructor which takes an AABox2d (axes-aligned box).

            :param AABox2d aabox: The input AABox2d.
            """

            aabox: AABox2d = args[0]
            self._center = aabox.center
            self._length = aabox.length
            self._width = aabox.width
            self._half_length = aabox.half_length
            self._half_width = aabox.half_width
            self._heading = 0.0
            self._cos_heading = 1.0
            self._sin_heading = 0.0
            assert self._length >= -kMathEpsilon
            assert self._width >= -kMathEpsilon

    @staticmethod
    def CreateAABox(one_corner: Vec2d, opposite_corner: Vec2d) -> 'Box2d':
        """
        Creates an axes-aligned Box2d from two opposite corners
   
        :param Vec2d one_corner: One of the corners
        :param Vec2d opposite_corner: The opposite corner to the first one
        :returns: An axes-aligned Box2d
        :rtype: Box2d
        """

        x1: float = min(one_corner.x, opposite_corner.x)
        x2: float = max(one_corner.x, opposite_corner.x)
        y1: float = min(one_corner.y, opposite_corner.y)
        y2: float = max(one_corner.y, opposite_corner.y)
        return Box2d(Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0), 0.0, x2 - x1, y2 - y1)

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

        return self._corners

    def IsPointIn(self, point: Vec2d) -> bool:
        """
        Tests points for membership in the box
        
        :param Vec2d point: A point that we wish to test for membership in the box
        :returns: True iff the point is contained in the box
        :rtype: bool
        """

        x0: float = point.x - self._center.x
        y0: float = point.y - self._center.y
        dx: float = abs(x0 * self._cos_heading + y0 * self._sin_heading)
        dy: float = abs(-x0 * self._sin_heading + y0 * self._cos_heading)
        return (dx <= self._half_length + kMathEpsilon) and (dy <= self._half_width + kMathEpsilon)

    def IsPointOnBoundary(self, point: Vec2d) -> bool:
        """
        Tests points for membership in the boundary of the box
        
        :param Vec2d point: A point that we wish to test for membership in the boundary
        :returns: True iff the point is a boundary point of the box
        :rtype: bool
        """

        x0: float = point.x - self._center.x
        y0: float = point.y - self._center.y
        dx: float = abs(x0 * self._cos_heading + y0 * self._sin_heading)
        dy: float = abs(x0 * self._sin_heading - y0 * self._cos_heading)
        return (abs(dx - self._half_length) <= kMathEpsilon and dy <= self._half_width + kMathEpsilon) \
            or (abs(dy - self._half_width) <= kMathEpsilon and dx <= self._half_length + kMathEpsilon)

    def CrossProd(start_point: Vec2d, end_point_1: Vec2d, end_point_2: Vec2d) -> float:
        """
        CrossProd util function

        :param Vec2d start_point: The start point
        :param Vec2d end_point_1: The first end point
        :param Vec2d end_point_2: The second end point
        :returns: The cross product of the vectors
        :rtype: float
        """

        return (end_point_1 - start_point).CrossProd(end_point_2 - start_point)

    def DistanceTo(self, param: Any) -> float:

        if isinstance(param, Vec2d):
            """
            Determines the distance between the box and a given point

            :param Vec2d point: The point whose distance to the box we wish to compute
            :returns: A distance
            :rtype: float
            """

            point: Vec2d = param
            x0: float = point.x - self._center.x
            y0: float = point.y - self._center.y
            dx: float = abs(x0 * self._cos_heading + y0 * self._sin_heading) - self._half_length
            dy: float = abs(x0 * self._sin_heading - y0 * self._cos_heading) - self._half_width
            if dx <= 0:
                return max(0.0, dy)
            if dy <= 0:
                return dx
            return hypot(dx, dy)

        elif isinstance(param, LineSegment2d):
            """
            Determines the distance between the box and a given line segment

            :param LineSegment2d line_segment: The line segment whose distance to the box we compute
            :returns: A distance
            :rtype: float 
            """

            line_segment: LineSegment2d = param
            if line_segment.length <= kMathEpsilon:
                return self.DistanceTo(line_segment.start)
            ref_x1: float = line_segment.start.x - self._center.x
            ref_y1: float = line_segment.start.y - self._center.y
            x1: float = ref_x1 * self._cos_heading + ref_y1 * self._sin_heading
            y1: float = ref_x1 * self._sin_heading - ref_y1 * self._cos_heading
            box_x: float = self._half_length
            box_y: float = self._half_width
            gx1: float = 1 if x1 >= box_x else -1 if x1 <= -box_x else 0
            gy1: float = 1 if y1 >= box_y else -1 if y1 <= -box_y else 0
            if gx1 == 0 and gy1 == 0:
                return 0.0
            ref_x2: float = line_segment.end.x - self._center.x
            ref_y2: float = line_segment.end.y - self._center.y
            x2: float = ref_x2 * self._cos_heading + ref_y2 * self._sin_heading
            y2: float = ref_x2 * self._sin_heading - ref_y2 * self._cos_heading
            gx2: float = 1 if x2 >= box_x else -1 if x2 <= -box_x else 0
            gy2: float = 1 if y2 >= box_y else -1 if y2 <= -box_y else 0
            if gx2 == 0 and gy2 == 0:
                return 0.0
            if gx1 < 0 or (gx1 == 0 and gx2 < 0):
                x1 = -x1
                gx1 = -gx1
                x2 = -x2
                gx2 = -gx2
            if gy1 < 0 or (gy1 == 0 and gy2 < 0):
                y1 = -y1
                gy1 = -gy1
                y2 = -y2
                gy2 = -gy2
            if gx1 < gy1 or (gx1 == gy1 and gx2 == gy2):
                x1, y1 = y1, x1
                gx1, gy1 = gy1, gx1
                x2, y2 = y2, x2
                gx2, gy2 = gy2, gx2
                box_x, box_y = box_y, box_x
            if gx1 == 1 and gy1 == 1:
                if gx2 * 3 + gy2 == 4:
                    return PtSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length())
                if gx2 * 3 + gy2 == 3:
                    return x2 - box_x if x1 > x2 else PtSegDistance(box_x, box_y, x1, y1, x2,
                                                                    y2, line_segment.length())
                if gx2 * 3 + gy2 == 2:
                    return PtSegDistance(box_x, -box_y, x1, y1, x2, y2, line_segment.length()) \
                           if x1 > x2 else PtSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length())
                if gx2 * 3 + gy2 == -1:
                    return 0.0 if self.CrossProd(Vec2d(x1, y1), Vec2d(x2, y2), Vec2d(box_x, -box_y)) >= 0.0 \
                           else PtSegDistance(box_x, -box_y, x1, y1, x2, y2, line_segment.length())
                if gx2 * 3 + gy2 == -4:
                    return PtSegDistance(box_x, -box_y, x1, y1, x2, y2, line_segment.length()) if self.CrossProd(
                           Vec2d(x1, y1), Vec2d(x2, y2), Vec2d(box_x, -box_y)) <= 0.0 else 0.0 \
                           if self.CrossProd(Vec2d(x1, y1), Vec2d(x2, y2), Vec2d(-box_x, box_y)) <= 0.0 else \
                           PtSegDistance(-box_x, box_y, x1, y1, x2, y2, line_segment.length())
            else:
                if gx2 * 3 + gy2 == 4:
                    return x1 - box_x if x1 < x2 else PtSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length())
                if gx2 * 3 + gy2 == 3:
                    return min(x1, x2) - box_x
                if gx2 * 3 + gy2 == 1 or gx2 * 3 + gy2 == -2:
                    return 0.0 if self.CrossProd(Vec2d(x1, y1), Vec2d(x2, y2), Vec2d(box_x, box_y)) <= 0.0 else \
                           PtSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length())
                if gx2 * 3 + gy2 == -3:
                    return 0.0
            logger.error(f"unimplemented state: '{gx1}' '{gy1}' '{gx2}' '{gy2}'")
            return 0.0

        elif isinstance(param, Box2d):
            """
            Determines the distance between two boxes

            :param Box2d box: The box whose distance to this box we want to compute
            :returns: A distance
            :rtype: float 
            """

            box: Box2d = param
            return Polygon2d(box).DistanceTo(self)
        
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

            line_segment: LineSegment2d = param
            if line_segment.length <= kMathEpsilon:
                return self.IsPointIn(line_segment.start)
            if (max(line_segment.start.x, line_segment.end.x) < self.min_x) or \
               (min(line_segment.start.x, line_segment.end.x) > self.max_x) or \
               (max(line_segment.start.y, line_segment.end.y) < self.min_y) or \
               (min(line_segment.start.y, line_segment.end.y) > self.max_y):
                return False
            # Construct coordinate system with origin point as left_bottom corner of
            # Box2d, y axis along direction of heading.
            x_axis: Vec2d = Vec2d(self._sin_heading, -self._cos_heading)
            y_axis: Vec2d = Vec2d(self._cos_heading, self._sin_heading)
            # corners_[2] is the left bottom point of the box.
            start_v: Vec2d = Vec2d(line_segment.start - self._corners[2])
            # "start_point" is the start point of "line_segment" mapped in the new
            # coordinate system.
            start_point: Vec2d = Vec2d(start_v.InnerProd(x_axis), start_v.InnerProd(y_axis))
            # Check if "start_point" is inside the box.
            if self.is_inside_rectangle(start_point):
                return True
            # Check if "end_point" is inside the box.
            end_v: Vec2d = Vec2d(line_segment.end - self._corners[2])
            end_point: Vec2d = Vec2d(end_v.InnerProd(x_axis), end_v.InnerProd(y_axis))
            if self.is_inside_rectangle(end_point):
                return True
            # Exclude the case when the 2 points of "line_segment" are at the same side
            # of rectangle.
            if start_point.x < 0.0 and end_point.x < 0.0:
                return False
            if start_point.y < 0.0 and end_point.y < 0.0:
                return False
            if start_point.x > self._width and end_point.x > self._width:
                return False
            if start_point.y > self._length and end_point.y > self._length:
                return False
            # Check if "line_segment" intersects with box.
            line_direction: Vec2d = line_segment.end - line_segment.start
            normal_vec: Vec2d = Vec2d(line_direction.y, -line_direction.x)
            p1: Vec2d = self._center - line_segment.start
            diagonal_vec: Vec2d = self._center - self._corners[0]
            # if project_p1 < projection of diagonal, "line_segment" intersects with box.
            project_p1: float = abs(p1.InnerProd(normal_vec))
            if abs(diagonal_vec.InnerProd(normal_vec)) >= project_p1:
                return True
            diagonal_vec = self._center - self._corners[1]
            if abs(diagonal_vec.InnerProd(normal_vec)) >= project_p1:
                return True
            return False

        elif isinstance(param, Box2d):
            """
            Determines whether these two boxes overlap

            :param Box2d box: The other box
            :returns: True if they overlap
            :rtype: bool
            """

            box: Box2d = param
            if box.max_x < self.min_x or box.min_x > self.max_x or box.max_y < self.min_y or box.min_y > self.max_y:
                return False
            
            shift_x: float = box.center_x - self._center.x
            shift_y: float = box.center_y - self._center.y

            dx1: float = self._cos_heading * self._half_length
            dy1: float = self._sin_heading * self._half_length
            dx2: float = self._sin_heading * self._half_width
            dy2: float = -self._cos_heading * self._half_width
            dx3: float = box.cos_heading * box.half_length
            dy3: float = box.sin_heading * box.half_length
            dx4: float = box.sin_heading * box.half_width
            dy4: float = -box.cos_heading * box.half_width

            condition1 = (abs(shift_x * self._cos_heading + shift_y * self._sin_heading) <= 
                         abs(dx3 * self._cos_heading + dy3 * self._sin_heading) + 
                         abs(dx4 * self._cos_heading + dy4 * self._sin_heading) + 
                         self._half_length)
    
            condition2 = (abs(shift_x * self._sin_heading - shift_y * self._cos_heading) <=
                         abs(dx3 * self._sin_heading - dy3 * self._cos_heading) +
                         abs(dx4 * self._sin_heading - dy4 * self._cos_heading) +
                         self._half_width)
    
            condition3 = (abs(shift_x * box.cos_heading + shift_y * box.sin_heading) <=
                         abs(dx1 * box.cos_heading + dy1 * box.sin_heading) +
                         abs(dx2 * box.cos_heading + dy2 * box.sin_heading) +
                         box.half_length)
    
            condition4 = (abs(shift_x * box.sin_heading - shift_y * box.cos_heading) <=
                         abs(dx1 * box.sin_heading - dy1 * box.cos_heading) +
                         abs(dx2 * box.sin_heading - dy2 * box.cos_heading) +
                         box.half_width)

            return condition1 and condition2 and condition3 and condition4

        else:

            raise ValueError("Unknown param type")

    def GetAABox(self) -> AABox2d:
        """
        Gets the smallest axes-aligned box containing the current one
        
        :returns: An axes-aligned box
        :rtype: AABox2d
        """

        dx1: float = abs(self._cos_heading * self._half_length)
        dy1: float = abs(self._sin_heading * self._half_length)
        dx2: float = abs(self._sin_heading * self._half_width)
        dy2: float = abs(self._cos_heading * self._half_width)
        return AABox2d(self._center, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0)

    def RotateFromCenter(self, rotate_angle: float) -> None:
        """
        Rotate from center.

        :param float rotate_angle: Angle to rotate.
        """
        
        self._heading = NormalizeAngle(self._heading + rotate_angle)
        self._cos_heading = cos(self._heading)
        self._sin_heading = sin(self._heading)
        self.InitCorners()

    def Shift(self, shift_vec: Vec2d) -> None:
        """
        Shifts this box by a given vector

        :param Vec2d shift_vec: The vector determining the shift
        """

        self._center += shift_vec
        for i in range(4):
            self._corners[i] += shift_vec
        for corner in self._corners:
            self._max_x = max(corner.x, self._max_x)
            self._min_x = min(corner.x, self._min_x)
            self._max_y = max(corner.y, self._max_y)
            self._min_y = min(corner.y, self._min_y)

    def LongitudinalExtend(self, extension_length: float) -> None:
        """
        Extend the box longitudinally

        :param float extension_length: the length to extend
        """

        self._length += extension_length
        self._half_length += extension_length / 2.0
        self.InitCorners()

    def LateralExtend(self, extension_length: float) -> None:
        """
        Extend the box laterally

        :param float extension_length: the length to extend
        """
        
        self._width += extension_length
        self._half_width += extension_length / 2.0
        self.InitCorners()

    def __str__(self) -> str:
        """
        Debug string representation

        :returns: A string representation of class Box2d
        :rtype: str
        """

        return f"box2d: (center: {self._center}, heading: {self._heading}, length: {self._length}, width: {self._width}"

    def InitCorners(self) -> None:
        """
        Initialize the corners of the box
        """

        dx1: float = self._cos_heading * self._half_length
        dy1: float = self._sin_heading * self._half_length
        dx2: float = self._sin_heading * self._half_width
        dy2: float = -self._cos_heading * self._half_width

        self._corners.clear()
        self._corners.append(Vec2d(self._center.x + dx1 + dx2, self._center.y + dy1 + dy2))
        self._corners.append(Vec2d(self._center.x + dx1 - dx2, self._center.y + dy1 - dy2))
        self._corners.append(Vec2d(self._center.x - dx1 - dx2, self._center.y - dy1 - dy2))
        self._corners.append(Vec2d(self._center.x - dx1 + dx2, self._center.y - dy1 + dy2))

        for corner in self._corners:
            self._max_x = max(corner.x, self._max_x)
            self._min_x = min(corner.x, self._min_x)
            self._max_y = max(corner.y, self._max_y)
            self._min_y = min(corner.y, self._min_y)

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
