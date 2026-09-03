"""Spline 2D kernel aligned with spline_2d_kernel.cc."""

from __future__ import annotations

import bisect
from typing import List, Tuple

import numpy as np

from planning_math.smoothing_spline.spline_seg_kernel import SplineSegKernel


class Spline2dKernel:
    def __init__(self, t_knots: List[float], spline_order: int):
        self._t_knots = list(t_knots)
        self._spline_order = spline_order
        self._total_params = (
            2 * (len(self._t_knots) - 1) * (1 + spline_order) if len(self._t_knots) > 1 else 0
        )
        self._kernel_matrix = np.zeros((self._total_params, self._total_params))
        self._offset = np.zeros((self._total_params, 1))

    def add_regularization(self, regularization_param: float) -> None:
        self._kernel_matrix += np.eye(self._total_params) * regularization_param

    def add_nth_derivative_kernel_matrix(self, n: int, weight: float) -> None:
        num_params = self._spline_order + 1
        seg_kernel = SplineSegKernel.instance()
        for i in range(len(self._t_knots) - 1):
            cur_kernel = seg_kernel.nth_derivative_kernel(
                n, num_params, self._t_knots[i + 1] - self._t_knots[i]
            ) * weight
            x_block = slice(2 * i * num_params, 2 * i * num_params + num_params)
            y_block = slice((2 * i + 1) * num_params, (2 * i + 1) * num_params + num_params)
            self._kernel_matrix[x_block, x_block] += cur_kernel
            self._kernel_matrix[y_block, y_block] += cur_kernel

    def add_second_order_derivative_matrix(self, weight: float) -> None:
        self.add_nth_derivative_kernel_matrix(2, weight)

    def add_third_order_derivative_matrix(self, weight: float) -> None:
        self.add_nth_derivative_kernel_matrix(3, weight)

    def kernel_matrix(self) -> np.ndarray:
        return self._kernel_matrix * 2.0

    @property
    def offset(self) -> np.ndarray:
        return self._offset

    def find_index(self, t: float) -> int:
        upper = bisect.bisect_right(self._t_knots, t, 1, len(self._t_knots))
        return min(len(self._t_knots) - 1, upper) - 1
