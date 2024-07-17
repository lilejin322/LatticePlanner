from common.MapPathPoint import MapPathPoint

class ReferencePoint(MapPathPoint):
    """
    Reference point class
    """
    def __init__(self, x=0.0, y=0.0, heading=0.0, kappa=0.0, dkappa=0.0):
        """
        Constructor

        :param x: x coordinate
        :param y: y coordinate
        :param heading: heading angle
        :param kappa: curvature
        :param dkappa: derivative of curvature
        """

        self._x = x
        self._y = y
        self._heading = heading
        self._kappa = kappa
        self._dkappa = dkappa

    def x(self):
        return self._x

    def y(self):
        return self._y

    def heading(self):
        return self._heading

    def kappa(self):
        return self._kappa

    def dkappa(self):
        return self._dkappa
