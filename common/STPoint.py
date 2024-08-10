from common.Vec2d import Vec2d
from copy import deepcopy

class STPoint(Vec2d):
    """
    STPoint class
    x-axis: t; y-axis: s.
    """

    def __init__(self, *args):
        """
        Constructor
        """

        if len(args) == 1 and isinstance(args[0], Vec2d):
            """
            :param Vec2d args[0]: Vec2d object
            """

            self.__dict__ = deepcopy(args[0].__dict)

        elif len(args) == 2:
            """
            :param float s: s-coordinate
            :param float t: t-coordinate
            """

            s, t = args
            super().__init__(t, s)
        else:
            raise ValueError("Invalid arguments")

    @property
    @staticmethod
    def x():

        raise AttributeError("x attribute in class STPoint is deleted")

    @property
    @staticmethod
    def y():

        raise AttributeError("y attribute in class STPoint is deleted")

    @property
    def s(self) -> float:
        """
        get s-coordinate

        :return: s-coordinate
        :rtype: float
        """

        return self._y

    @property
    def t(self) -> float:
        """
        get t-coordinate

        :return: t-coordinate
        :rtype: float
        """

        return self._x

    @s.setter
    def s(self, s: float) -> None:
        """
        s setter

        :param float s: s-coordinate
        """

        self._y = s
    
    @t.setter
    def t(self, t: float) -> None:
        """
        t setter

        :param float t: t-coordinate
        """

        self._x = t

    def __str__(self) -> str:
        """
        Returns a human-readable string representing this object

        :returns: human-readable string
        :rtype: str
        """

        return f"STPoint(s: {self._y}, t: {self._x})"
