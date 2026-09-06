"""
Spline 2D kernel aligned with spline_2d_kernel.cc.
"""
from __future__ import annotations
import bisect
from typing import List
import numpy as np
from planning_math.smoothing_spline.spline_seg_kernel import SplineSegKernel

class Spline2dKernel:
    """
    Spline 2D kernel aligned with spline_2d_kernel.cc.
    """
    _t_knots: List[float]
    _spline_order: int
    _total_params: int
    _kernel_matrix: np.ndarray
    _offset: np.ndarray

    def __init__(self, t_knots: List[float], spline_order: int) -> None:
        """
        Constructor

        :param List[float] t_knots: The knot points for the spline.
        :param int spline_order: The order of the spline.
        :returns: None
        """
        self._t_knots = list(t_knots)
        self._spline_order = spline_order
        self._total_params = (
            2 * (len(self._t_knots) - 1) * (1 + spline_order) if len(self._t_knots) > 1 else 0
        )
        self._kernel_matrix = np.zeros((self._total_params, self._total_params))
        self._offset = np.zeros((self._total_params, 1))

    def add_regularization(self, regularization_param: float) -> None:
        """
        Add regularization to the kernel matrix.

        :param float regularization_param: The regularization parameter to add to the kernel matrix.
        :returns: None
        """
        self._kernel_matrix += np.eye(self._total_params) * regularization_param

    def add_nth_derivative_kernel_matrix(self, n: int, weight: float) -> None:
        """
        Add the nth derivative kernel matrix to the kernel matrix.

        :param int n: The order of the derivative to add to the kernel matrix.
        :param float weight: The weight to apply to the nth derivative kernel matrix.
        :returns: None
        """
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
        """
        Add the second-order derivative kernel matrix to the kernel matrix.

        :param float weight: The weight to apply to the second-order derivative kernel matrix.
        :returns: None
        """
        self.add_nth_derivative_kernel_matrix(2, weight)

    def add_third_order_derivative_matrix(self, weight: float) -> None:
        """
        Add the third-order derivative kernel matrix to the kernel matrix.

        :param float weight: The weight to apply to the third-order derivative kernel matrix.
        :returns: None
        """
        self.add_nth_derivative_kernel_matrix(3, weight)

    def kernel_matrix(self) -> np.ndarray:
        """
        Get the kernel matrix.

        :returns: The kernel matrix.
        :rtype: np.ndarray
        """
        return self._kernel_matrix * 2.0

    @property
    def offset(self) -> np.ndarray:
        """
        Get the offset vector.

        :returns: The offset vector.
        :rtype: np.ndarray
        """
        return self._offset

    def find_index(self, t: float) -> int:
        """
        Find the index of the knot point that is less than or equal to t.
        
        :param float t: The time coordinate to find the index for.
        :returns: The index of the knot point that is less than or equal to t.
        :rtype: int
        """
        upper = bisect.bisect_right(self._t_knots, t, 1, len(self._t_knots))
        return min(len(self._t_knots) - 1, upper) - 1
