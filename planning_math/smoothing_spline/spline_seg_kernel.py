"""
Spline segment kernel aligned with spline_seg_kernel.cc.
"""
from __future__ import annotations
import numpy as np

class SplineSegKernel:
    """
    Spline segment kernel aligned with spline_seg_kernel.cc, singleton pattern
    """
    _instance = None
    _reserved_order: int
    _kernel_fx: np.ndarray | None
    _kernel_derivative: np.ndarray | None
    _kernel_second_order_derivative: np.ndarray | None
    _kernel_third_order_derivative: np.ndarray | None

    def __init__(self, reserved_order: int = 5) -> None:
        """
        Constructor

        :param int reserved_order: The reserved order of the spline segment kernel.
        :returns: None
        """
        self._reserved_order = reserved_order
        self._kernel_fx: np.ndarray | None = None
        self._kernel_derivative: np.ndarray | None = None
        self._kernel_second_order_derivative: np.ndarray | None = None
        self._kernel_third_order_derivative: np.ndarray | None = None
        self._calculate_fx(reserved_order + 1)
        self._calculate_derivative(reserved_order + 1)
        self._calculate_second_order_derivative(reserved_order + 1)
        self._calculate_third_order_derivative(reserved_order + 1)

    @classmethod
    def instance(cls) -> "SplineSegKernel":
        """
        Get the singleton instance of SplineSegKernel.

        :returns: The singleton instance of SplineSegKernel.
        :rtype: SplineSegKernel
        """
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def nth_derivative_kernel(self, n: int, num_params: int, accumulated_x: float) -> np.ndarray:
        """
        Get the nth derivative kernel matrix.

        :param int n: The order of the derivative (1, 2, or 3).
        :param int num_params: The number of parameters.
        :param float accumulated_x: The accumulated x value.
        :returns: The nth derivative kernel matrix.
        :rtype: np.ndarray
        """
        if n == 1:
            return self.derivative_kernel(num_params, accumulated_x)
        if n == 2:
            return self.second_order_derivative_kernel(num_params, accumulated_x)
        if n == 3:
            return self.third_order_derivative_kernel(num_params, accumulated_x)
        return np.zeros((num_params, num_params))

    def derivative_kernel(self, num_params: int, accumulated_x: float) -> np.ndarray:
        """
        Get the first derivative kernel matrix.

        :param int num_params: The number of parameters.
        :param float accumulated_x: The accumulated x value.
        :returns: The first derivative kernel matrix.
        :rtype: np.ndarray
        """
        if num_params > self._reserved_order + 1:
            self._calculate_derivative(num_params)
        term_matrix = self._integrated_term_matrix(num_params, accumulated_x, "derivative")
        return self._kernel_derivative[:num_params, :num_params] * term_matrix

    def second_order_derivative_kernel(self, num_params: int, accumulated_x: float) -> np.ndarray:
        """
        Get the second derivative kernel matrix.
        
        :param int num_params: The number of parameters.
        :param float accumulated_x: The accumulated x value.
        :returns: The second derivative kernel matrix.
        :rtype: np.ndarray"""
        if num_params > self._reserved_order + 1:
            self._calculate_second_order_derivative(num_params)
        term_matrix = self._integrated_term_matrix(num_params, accumulated_x, "second_order")
        return self._kernel_second_order_derivative[:num_params, :num_params] * term_matrix

    def third_order_derivative_kernel(self, num_params: int, accumulated_x: float) -> np.ndarray:
        """
        Get the third derivative kernel matrix.
        
        :param int num_params: The number of parameters.
        :param float accumulated_x: The accumulated x value.
        :returns: The third derivative kernel matrix.
        :rtype: np.ndarray"""
        if num_params > self._reserved_order + 1:
            self._calculate_third_order_derivative(num_params)
        term_matrix = self._integrated_term_matrix(num_params, accumulated_x, "third_order")
        return self._kernel_third_order_derivative[:num_params, :num_params] * term_matrix

    @staticmethod
    def _integrated_term_matrix(num_params: int, x: float, kind: str) -> np.ndarray:
        """
        Calculate the integrated term matrix for the given kind of kernel.

        :param int num_params: The number of parameters.
        :param float x: The accumulated x value.
        :param str kind: The kind of kernel ("fx", "derivative", "second_order", or "third_order").
        :returns: The integrated term matrix.
        :rtype: np.ndarray
        """
        term_matrix = np.zeros((num_params, num_params))
        x_pow = [1.0] * (2 * num_params + 1)
        for i in range(1, 2 * num_params + 1):
            x_pow[i] = x_pow[i - 1] * x

        if kind == "fx":
            for r in range(num_params):
                for c in range(num_params):
                    term_matrix[r, c] = x_pow[r + c + 1]
        elif kind == "derivative":
            for r in range(1, num_params):
                for c in range(1, num_params):
                    term_matrix[r, c] = x_pow[r + c - 1]
        elif kind == "second_order":
            for r in range(2, num_params):
                for c in range(2, num_params):
                    term_matrix[r, c] = x_pow[r + c - 3]
        else:
            for r in range(3, num_params):
                for c in range(3, num_params):
                    term_matrix[r, c] = x_pow[r + c - 5]
        return term_matrix

    def _calculate_fx(self, num_params: int) -> None:
        """
        Calculate the kernel matrix for the function values.

        :param int num_params: The number of parameters.
        :returns: None
        """
        kernel = np.zeros((num_params, num_params))
        for r in range(num_params):
            for c in range(num_params):
                kernel[r, c] = 1.0 / (r + c + 1.0)
        self._kernel_fx = kernel

    def _calculate_derivative(self, num_params: int) -> None:
        """
        Calculate the kernel matrix for the first derivative.

        :param int num_params: The number of parameters.
        :returns: None
        """
        kernel = np.zeros((num_params, num_params))
        for r in range(1, num_params):
            for c in range(1, num_params):
                kernel[r, c] = r * c / (r + c - 1.0)
        self._kernel_derivative = kernel

    def _calculate_second_order_derivative(self, num_params: int) -> None:
        """
        Calculate the kernel matrix for the second order derivative.

        :param int num_params: The number of parameters.
        :returns: None
        """
        kernel = np.zeros((num_params, num_params))
        for r in range(2, num_params):
            for c in range(2, num_params):
                kernel[r, c] = (r * r - r) * (c * c - c) / (r + c - 3.0)
        self._kernel_second_order_derivative = kernel

    def _calculate_third_order_derivative(self, num_params: int) -> None:
        """
        Calculate the kernel matrix for the third order derivative.

        :param int num_params: The number of parameters.
        :returns: None
        """
        kernel = np.zeros((num_params, num_params))
        for r in range(3, num_params):
            for c in range(3, num_params):
                kernel[r, c] = (r * r - r) * (r - 2) * (c * c - c) * (c - 2) / (r + c - 5.0)
        self._kernel_third_order_derivative = kernel
