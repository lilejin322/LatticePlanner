"""Polynomial and 2D spline segment aligned with spline_2d_seg.cc."""

from __future__ import annotations

from typing import List, Tuple


class PolynomialXd:
    def __init__(self, coeffs: List[float]):
        self._coeffs = list(coeffs)

    def __call__(self, t: float) -> float:
        result = 0.0
        power = 1.0
        for coeff in self._coeffs:
            result += coeff * power
            power *= t
        return result

    @staticmethod
    def derived_from(poly: "PolynomialXd") -> "PolynomialXd":
        coeffs = poly._coeffs
        if len(coeffs) <= 1:
            return PolynomialXd([0.0])
        return PolynomialXd([coeffs[i] * i for i in range(1, len(coeffs))])


class Spline2dSeg:
    def __init__(self, order: int):
        self._spline_func_x = PolynomialXd([0.0] * (order + 1))
        self._spline_func_y = PolynomialXd([0.0] * (order + 1))
        self._refresh_derivatives()

    def set_params(self, x_param: List[float], y_param: List[float]) -> bool:
        if len(x_param) != len(y_param):
            return False
        self._spline_func_x = PolynomialXd(x_param)
        self._spline_func_y = PolynomialXd(y_param)
        self._refresh_derivatives()
        return True

    def _refresh_derivatives(self) -> None:
        self._derivative_x = PolynomialXd.derived_from(self._spline_func_x)
        self._derivative_y = PolynomialXd.derived_from(self._spline_func_y)
        self._second_derivative_x = PolynomialXd.derived_from(self._derivative_x)
        self._second_derivative_y = PolynomialXd.derived_from(self._derivative_y)
        self._third_derivative_x = PolynomialXd.derived_from(self._second_derivative_x)
        self._third_derivative_y = PolynomialXd.derived_from(self._second_derivative_y)

    def __call__(self, t: float) -> Tuple[float, float]:
        return self._spline_func_x(t), self._spline_func_y(t)

    def x(self, t: float) -> float:
        return self._spline_func_x(t)

    def y(self, t: float) -> float:
        return self._spline_func_y(t)

    def derivative_x(self, t: float) -> float:
        return self._derivative_x(t)

    def derivative_y(self, t: float) -> float:
        return self._derivative_y(t)

    def second_derivative_x(self, t: float) -> float:
        return self._second_derivative_x(t)

    def second_derivative_y(self, t: float) -> float:
        return self._second_derivative_y(t)

    def third_derivative_x(self, t: float) -> float:
        return self._third_derivative_x(t)

    def third_derivative_y(self, t: float) -> float:
        return self._third_derivative_y(t)
