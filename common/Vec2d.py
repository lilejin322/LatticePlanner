import math

kMathEpsilon: float = 1e-10

class Vec2d:
    """
    Implements a class of 2-dimensional vectors.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0):
        """
        Constructor which takes x- and y-coordinates.
        Default values are 0.0 for both.

        :param float x: x-coordinate, default=0.0
        :param float y: y-coordinate, default=0.0
        """
        
        self._x: float = x
        self._y: float = y

    @staticmethod
    def CreateUnitVec2d(angle: float):
        """
        Creates a unit-vector with a given angle to the positive x semi-axis

        :param float angle: angle in radians
        """

        return Vec2d(math.cos(angle), math.sin(angle))

    @property
    def x(self) -> float:
        """
        Getter for x component

        :returns: x component
        :rtype: float
        """

        return self._x

    @x.setter
    def set_x(self, value: float) -> None:
        """
        Setter for x component

        :param float value: new x component tos set
        """

        self._x: float = value

    @property
    def y(self):
        """
        Getter for y component

        :returns: y component
        :rtype: float
        """

        return self._y

    @y.setter
    def set_y(self, value: float) -> None:
        """
        Setter for y component

        :param float value: new y component to set
        """

        self._y = value

    def Length(self) -> float:
        """
        Gets the length of the vector

        :returns: length of the vector
        :rtype: float
        """

        return math.sqrt(self.x_ ** 2 + self.y_ ** 2)

    def LengthSquare(self) -> float:
        """
        Gets the squared length of the vector

        :returns: squared length of the vector
        :rtype: float
        """

        return self.x_ ** 2 + self.y_ ** 2

    def Angle(self) -> float:
        """
        Gets the angle between the vector and the positive x semi-axis

        :returns: angle in radians
        :rtype: float
        """

        return math.atan2(self.y_, self.x_)

    def Normalize(self) -> None:
        """
        Returns the unit vector that is co-linear with this vector
        """

        length = self.length()
        if length > 0:
            self.x_ /= length
            self.y_ /= length

    def DistanceTo(self, other: 'Vec2d') -> float:
        """
        Returns the distance to the given vector

        :param Vec2d other: other vector
        :returns: distance to the other vector
        :rtype: float
        """

        return math.sqrt((self.x_ - other.x_) ** 2 + (self.y_ - other.y_) ** 2)

    def DistanceSquareTo(self, other: 'Vec2d') -> float:
        """
        Returns the squared distance to the given vector

        :param Vec2d other: other vector
        :returns: squared distance to the other vector
        :rtype: float
        """

        return (self.x_ - other.x_) ** 2 + (self.y_ - other.y_) ** 2

    def CrossProd(self, other: 'Vec2d') -> float:
        """
        Returns the "cross" product between these two Vec2d (non-standard).

        :param Vec2d other: other vector
        :returns: cross product
        :rtype: float
        """

        return self.x_ * other.y_ - self.y_ * other.x_

    def InnerProd(self, other: 'Vec2d') -> float:
        """
        Returns the inner product between these two Vec2d.

        :param Vec2d other: other vector
        :returns: inner product
        :rtype: float
        """

        return self.x_ * other.x_ + self.y_ * other.y_

    def rotate(self, angle: float) -> 'Vec2d':
        """
        rotate the vector by angle.

        :param float angle: angle in radians
        :returns: rotated vector
        :rtype: Vec2d
        """

        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        return Vec2d(self.x_ * cos_angle - self.y_ * sin_angle,
                     self.x_ * sin_angle + self.y_ * cos_angle)

    def SelfRotate(self, angle: float) -> None:
        """
        rotate the vector itself by angle.

        :param float angle: angle in radians
        """

        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        new_x = self.x_ * cos_angle - self.y_ * sin_angle
        new_y = self.x_ * sin_angle + self.y_ * cos_angle
        self.x_ = new_x
        self.y_ = new_y

    def __add__(self, other: 'Vec2d') -> 'Vec2d':
        """
        Sums two Vec2d, Overrides the '+' operator.

        :param Vec2d other: other vector
        :returns: sum of the two vectors
        :rtype: Vec2d
        """

        return Vec2d(self.x_ + other.x_, self.y_ + other.y_)

    def __sub__(self, other: 'Vec2d') -> 'Vec2d':
        """
        Subtracts two Vec2d, Overrides the '-' operator.

        :param Vec2d other: other vector
        :returns: difference of the two vectors
        :rtype: Vec2d
        """

        return Vec2d(self.x_ - other.x_, self.y_ - other.y_)

    def __mul__(self, ratio: float) -> 'Vec2d':
        """
        Multiplies Vec2d by a scalar, Overrides the '*' operator.

        :param float ratio: scalar
        :returns: scaled vector
        :rtype: Vec2d
        """

        return Vec2d(self.x_ * ratio, self.y_ * ratio)

    def __truediv__(self, ratio: float) -> 'Vec2d':
        """
        Divides Vec2d by a scalar, Overrides the '/' operator.

        :param float ratio: scalar
        :returns: scaled vector
        :rtype: Vec2d
        """

        return Vec2d(self.x_ / ratio, self.y_ / ratio)

    def __iadd__(self, other: 'Vec2d') -> 'Vec2d':
        """
        Sums another Vec2d to the current one, Overrides the '+=' operator.

        :param Vec2d other: other vector
        :returns: sum of the two vectors
        :rtype: Vec2d
        """

        self.x_ += other.x_
        self.y_ += other.y_
        return self

    def __isub__(self, other: 'Vec2d') -> 'Vec2d':
        """
        Subtracts another Vec2d to the current one, Overrides the '-=' operator.

        :param Vec2d other: other vector
        :returns: difference of the two vectors
        :rtype: Vec2d
        """

        self.x_ -= other.x_
        self.y_ -= other.y_
        return self

    def __imul__(self, ratio: float) -> 'Vec2d':
        """
        Multiplies this Vec2d by a scalar, Overrides the '*=' operator.

        :param float ratio: scalar
        :returns: scaled vector
        :rtype: Vec2d
        """

        self.x_ *= ratio
        self.y_ *= ratio
        return self

    def __itruediv__(self, ratio: float) -> 'Vec2d':
        """
        Divides this Vec2d by a scalar, Overrides the '/=' operator.

        :param float ratio: scalar
        :returns: scaled vector
        :rtype: Vec2d
        """

        self.x_ /= ratio
        self.y_ /= ratio
        return self

    def __eq__(self, other: 'Vec2d') -> bool:
        """
        Compares two Vec2d, Overrides the '==' operator.

        :param Vec2d other: other vector
        :returns: True if the two vectors are equal, False otherwise
        :rtype: bool
        """

        return self.x_ == other.x_ and self.y_ == other.y_

    def __str__(self) -> str:
        """
        Returns a human-readable string representing this object

        :returns: human-readable string
        :rtype: str
        """

        return f"Vec2d(x: {self.x_}, y: {self.y_})"
