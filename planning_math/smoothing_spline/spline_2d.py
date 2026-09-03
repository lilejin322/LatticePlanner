"""2D smoothing spline aligned with spline_2d.cc."""

from __future__ import annotations

import bisect
from typing import List, Tuple

import numpy as np

from planning_math.smoothing_spline.spline_2d_seg import Spline2dSeg


class Spline2d:
    def __init__(self, t_knots: List[float], order: int):
        self._t_knots = list(t_knots)
        self._spline_order = order
        self._splines: List[Spline2dSeg] = []
        if len(self._t_knots) > 1:
            for _ in range(1, len(self._t_knots)):
                self._splines.append(Spline2dSeg(order))

    def __call__(self, t: float) -> Tuple[float, float]:
        if not self._splines:
            return 0.0, 0.0
        index = self.find_index(t)
        return self._splines[index](t - self._t_knots[index])

    def x(self, t: float) -> float:
        if not self._splines:
            return 0.0
        index = self.find_index(t)
        return self._splines[index].x(t - self._t_knots[index])

    def y(self, t: float) -> float:
        if not self._splines:
            return 0.0
        index = self.find_index(t)
        return self._splines[index].y(t - self._t_knots[index])

    def derivative_x(self, t: float) -> float:
        index = self.find_index(t)
        return self._splines[index].derivative_x(t - self._t_knots[index])

    def derivative_y(self, t: float) -> float:
        index = self.find_index(t)
        return self._splines[index].derivative_y(t - self._t_knots[index])

    def second_derivative_x(self, t: float) -> float:
        index = self.find_index(t)
        return self._splines[index].second_derivative_x(t - self._t_knots[index])

    def second_derivative_y(self, t: float) -> float:
        index = self.find_index(t)
        return self._splines[index].second_derivative_y(t - self._t_knots[index])

    def third_derivative_x(self, t: float) -> float:
        index = self.find_index(t)
        return self._splines[index].third_derivative_x(t - self._t_knots[index])

    def third_derivative_y(self, t: float) -> float:
        index = self.find_index(t)
        return self._splines[index].third_derivative_y(t - self._t_knots[index])

    def set_splines(self, params: np.ndarray, order: int) -> bool:
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
        return self._t_knots

    @property
    def spline_order(self) -> int:
        return self._spline_order

    def find_index(self, t: float) -> int:
        upper = bisect.bisect_right(self._t_knots, t, 1, len(self._t_knots))
        return min(len(self._t_knots) - 1, upper) - 1
