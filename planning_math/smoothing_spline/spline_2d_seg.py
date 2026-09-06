"""
Polynomial and 2D spline segment aligned with spline_2d_seg.cc.
"""
from __future__ import annotations
from typing import List, Tuple

class PolynomialXd:
    """
    A class representing a polynomial in one variable with coefficients stored in a list.
    The polynomial is defined as:
        P(t) = coeffs[0] + coeffs[1] * t + coeffs[2] * t^2 + ... + coeffs[n] * t^n
    where n is the degree of the polynomial.
    """
    _coeffs: List[float]

    def __init__(self, coeffs: List[float]) -> None:
        """
        Initialize the polynomial with the given coefficients.

        :param List[float] coeffs: The coefficients of the polynomial, where coeffs[i] is the coefficient for t^i.
        :returns: None
        """
        self._coeffs = list(coeffs)

    def __call__(self, t: float) -> float:
        """
        Evaluate the polynomial at a given value of t.

        :param float t: The value at which to evaluate the polynomial.
        :returns: The value of the polynomial at t.
        :rtype: float
        """
        result = 0.0
        power = 1.0
        for coeff in self._coeffs:
            result += coeff * power
            power *= t
        return result

    @staticmethod
    def derived_from(poly: "PolynomialXd") -> "PolynomialXd":
        """
        Compute the derivative of the given polynomial.

        :param PolynomialXd poly: The polynomial to differentiate.
        :returns: A new PolynomialXd representing the derivative of the input polynomial.
        :rtype: PolynomialXd
        """
        coeffs = poly._coeffs
        if len(coeffs) <= 1:
            return PolynomialXd([0.0])
        return PolynomialXd([coeffs[i] * i for i in range(1, len(coeffs))])

class Spline2dSeg:
    """
    A class representing a 2D spline segment defined by two polynomials, one for the x-coordinate and one for the y-coordinate.
    """
    _spline_func_x: PolynomialXd
    _spline_func_y: PolynomialXd

    def __init__(self, order: int) -> None:
        """
        Initialize the 2D spline segment with the given order.

        :param int order: The order of the spline segment (degree of the polynomial).
        :returns: None
        """
        self._spline_func_x = PolynomialXd([0.0] * (order + 1))
        self._spline_func_y = PolynomialXd([0.0] * (order + 1))
        self._refresh_derivatives()

    def set_params(self, x_param: List[float], y_param: List[float]) -> bool:
        """
        Set the parameters for the spline segment.

        :param List[float] x_param: The coefficients for the x-coordinate polynomial.
        :param List[float] y_param: The coefficients for the y-coordinate polynomial.
        :returns: True if the parameters were set successfully, False otherwise.
        :rtype: bool
        """
        if len(x_param) != len(y_param):
            return False
        self._spline_func_x = PolynomialXd(x_param)
        self._spline_func_y = PolynomialXd(y_param)
        self._refresh_derivatives()
        return True

    def _refresh_derivatives(self) -> None:
        """
        Refresh the derivative polynomials based on the current spline functions.

        :returns: None
        """
        self._derivative_x = PolynomialXd.derived_from(self._spline_func_x)
        self._derivative_y = PolynomialXd.derived_from(self._spline_func_y)
        self._second_derivative_x = PolynomialXd.derived_from(self._derivative_x)
        self._second_derivative_y = PolynomialXd.derived_from(self._derivative_y)
        self._third_derivative_x = PolynomialXd.derived_from(self._second_derivative_x)
        self._third_derivative_y = PolynomialXd.derived_from(self._second_derivative_y)

    def __call__(self, t: float) -> Tuple[float, float]:
        """
        Evaluate the spline segment at a given value of t.
        
        :param float t: The value at which to evaluate the spline segment.
        :returns: A tuple (x, y) representing the coordinates of the spline segment at t.
        :rtype: Tuple[float, float]
        """
        return self._spline_func_x(t), self._spline_func_y(t)

    def x(self, t: float) -> float:
        """
        Evaluate the x-coordinate of the spline segment at a given value of t.
        
        :param float t: The value at which to evaluate the x-coordinate.
        :returns: The x-coordinate of the spline segment at t.
        :rtype: float
        """
        return self._spline_func_x(t)

    def y(self, t: float) -> float:
        """
        Evaluate the y-coordinate of the spline segment at a given value of t.
        
        :param float t: The value at which to evaluate the y-coordinate.
        :returns: The y-coordinate of the spline segment at t.
        :rtype: float
        """
        return self._spline_func_y(t)

    def derivative_x(self, t: float) -> float:
        """
        Evaluate the derivative of the x-coordinate polynomial at a given value of t.
        
        :param float t: The value at which to evaluate the derivative of the x-coordinate.
        :returns: The derivative of the x-coordinate polynomial at t.
        :rtype: float
        """
        return self._derivative_x(t)

    def derivative_y(self, t: float) -> float:
        """
        Evaluate the derivative of the y-coordinate polynomial at a given value of t.

        :param float t: The value at which to evaluate the derivative of the y-coordinate.
        :returns: The derivative of the y-coordinate polynomial at t.
        :rtype: float
        """
        return self._derivative_y(t)

    def second_derivative_x(self, t: float) -> float:
        """
        Evaluate the second derivative of the x-coordinate polynomial at a given value of t.

        :param float t: The value at which to evaluate the second derivative of the x-coordinate.
        :returns: The second derivative of the x-coordinate polynomial at t.
        :rtype: float
        """
        return self._second_derivative_x(t)

    def second_derivative_y(self, t: float) -> float:
        """
        Evaluate the second derivative of the y-coordinate polynomial at a given value of t.

        :param float t: The value at which to evaluate the second derivative of the y-coordinate.
        :returns: The second derivative of the y-coordinate polynomial at t.
        :rtype: float
        """
        return self._second_derivative_y(t)

    def third_derivative_x(self, t: float) -> float:
        """
        Evaluate the third derivative of the x-coordinate polynomial at a given value of t.

        :param float t: The value at which to evaluate the third derivative of the x-coordinate.
        :returns: The third derivative of the x-coordinate polynomial at t.
        :rtype: float
        """
        return self._third_derivative_x(t)

    def third_derivative_y(self, t: float) -> float:
        """
        Evaluate the third derivative of the y-coordinate polynomial at a given value of t.

        :param float t: The value at which to evaluate the third derivative of the y-coordinate.
        :returns: The third derivative of the y-coordinate polynomial at t.
        :rtype: float
        """
        return self._third_derivative_y(t)
