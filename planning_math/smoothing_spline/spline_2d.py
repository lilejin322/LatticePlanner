"""
2D smoothing spline aligned with spline_2d.cc.
"""
from __future__ import annotations
import bisect
from typing import List, Tuple
import numpy as np
from planning_math.smoothing_spline.spline_2d_seg import Spline2dSeg

class Spline2d:
    """
    A class representing a 2D smoothing spline defined by a series of spline segments.
    The spline is defined over a set of knot points (t_knots) and consists of multiple spline segments, each represented by a Spline2dSeg object.
    """
    _t_knots: List[float]
    _spline_order: int
    _splines: List[Spline2dSeg]

    def __init__(self, t_knots: List[float], order: int) -> None:
        """
        Initialize the 2D smoothing spline with the given knot points and spline order.

        :param List[float] t_knots: The knot points defining the intervals for the spline segments.
        :param int order: The order of the spline segments (degree of the polynomial).
        :returns: None
        """
        self._t_knots = list(t_knots)
        self._spline_order = order
        self._splines: List[Spline2dSeg] = []
        if len(self._t_knots) > 1:
            for _ in range(1, len(self._t_knots)):
                self._splines.append(Spline2dSeg(order))

    def __call__(self, t: float) -> Tuple[float, float]:
        """
        Evaluate the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the spline.
        :returns: The (x, y) coordinates of the spline at the given parameter value.
        :rtype: Tuple[float, float]
        """
        if not self._splines:
            return 0.0, 0.0
        index = self.find_index(t)
        return self._splines[index](t - self._t_knots[index])

    def x(self, t: float) -> float:
        """
        Evaluate the x-component of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the spline.
        :returns: The x-coordinate of the spline at the given parameter value.
        :rtype: float
        """
        if not self._splines:
            return 0.0
        index = self.find_index(t)
        return self._splines[index].x(t - self._t_knots[index])

    def y(self, t: float) -> float:
        """
        Evaluate the y-component of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the spline.
        :returns: The y-coordinate of the spline at the given parameter value.
        :rtype: float
        """
        if not self._splines:
            return 0.0
        index = self.find_index(t)
        return self._splines[index].y(t - self._t_knots[index])

    def derivative_x(self, t: float) -> float:
        """
        Evaluate the x-component of the first derivative of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the derivative.
        :returns: The x-component of the first derivative of the spline at the given parameter value.
        :rtype: float
        """
        index = self.find_index(t)
        return self._splines[index].derivative_x(t - self._t_knots[index])

    def derivative_y(self, t: float) -> float:
        """
        Evaluate the y-component of the first derivative of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the derivative.
        :returns: The y-component of the first derivative of the spline at the given parameter value.
        :rtype: float
        """
        index = self.find_index(t)
        return self._splines[index].derivative_y(t - self._t_knots[index])

    def second_derivative_x(self, t: float) -> float:
        """
        Evaluate the x-component of the second derivative of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the derivative.
        :returns: The x-component of the second derivative of the spline at the given parameter value.
        :rtype: float
        """
        index = self.find_index(t)
        return self._splines[index].second_derivative_x(t - self._t_knots[index])

    def second_derivative_y(self, t: float) -> float:
        """
        Evaluate the y-component of the second derivative of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the derivative.
        :returns: The y-component of the second derivative of the spline at the given parameter value.
        :rtype: float
        """
        index = self.find_index(t)
        return self._splines[index].second_derivative_y(t - self._t_knots[index])

    def third_derivative_x(self, t: float) -> float:
        """
        Evaluate the x-component of the third derivative of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the derivative.
        :returns: The x-component of the third derivative of the spline at the given parameter value.
        :rtype: float
        """
        index = self.find_index(t)
        return self._splines[index].third_derivative_x(t - self._t_knots[index])

    def third_derivative_y(self, t: float) -> float:
        """
        Evaluate the y-component of the third derivative of the spline at a given parameter value.

        :param float t: The parameter value at which to evaluate the derivative.
        :returns: The y-component of the third derivative of the spline at the given parameter value.
        :rtype: float
        """
        index = self.find_index(t)
        return self._splines[index].third_derivative_y(t - self._t_knots[index])

    def set_splines(self, params: np.ndarray, order: int) -> bool:
        """
        Set the parameters for the spline segments based on the provided parameter array and spline order.

        :param np.ndarray params: A 2D array containing the parameters for the spline segments.
        :param int order: The order of the spline segments (degree of the polynomial).
        :returns: True if the parameters were set successfully, False otherwise.
        :rtype: bool
        """
        num_params = order + 1
        expected = 2 * (len(self._t_knots) - 1) * num_params
        if params.shape[0] != expected:
            return False
        for i, seg in enumerate(self._splines):
            x_param = [float(params[2 * i * num_params + j, 0]) for j in range(num_params)]
            y_param = [float(params[(2 * i + 1) * num_params + j, 0]) for j in range(num_params)]
            seg.set_params(x_param, y_param)
        self._spline_order = order
        return True

    @property
    def t_knots(self) -> List[float]:
        """
        Get the knot points defining the intervals for the spline segments.

        :returns: A list of knot points.
        :rtype: List[float]
        """
        return self._t_knots

    @property
    def spline_order(self) -> int:
        """
        Get the order of the spline segments.

        :returns: The order of the spline segments.
        :rtype: int
        """
        return self._spline_order

    def find_index(self, t: float) -> int:
        """
        Find the index of the spline segment corresponding to the given parameter value.
        
        :param float t: The parameter value for which to find the corresponding spline segment index.
        :returns: The index of the spline segment corresponding to the given parameter value.
        :rtype: int
        """
        upper = bisect.bisect_right(self._t_knots, t, 1, len(self._t_knots))
        return min(len(self._t_knots) - 1, upper) - 1
