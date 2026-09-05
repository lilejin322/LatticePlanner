"""
Curve geometry helper submodule
"""

import math


def _divide_like_cpp(numerator: float, denominator: float) -> float:
    """
    C++'s `double` division never raises on a zero denominator -- it follows
    IEEE-754 and yields +-inf/nan. Python's `/` raises ZeroDivisionError on an
    exact 0.0 denominator, so that one case has to be emulated by hand; every
    other denominator (including a very small but nonzero one) is left to
    behave exactly like a normal float division, matching C++.
    """
    if denominator != 0.0:
        return numerator / denominator
    if numerator > 0.0:
        return math.inf
    if numerator < 0.0:
        return -math.inf
    return math.nan


def ComputeCurvature(dx: float, d2x: float, dy: float, d2y: float) -> float:
    a = dx * d2y - dy * d2x
    norm_square = dx * dx + dy * dy
    norm = norm_square ** 0.5
    b = norm * norm_square
    return _divide_like_cpp(a, b)


def ComputeCurvatureDerivative(
    dx: float, d2x: float, d3x: float, dy: float, d2y: float, d3y: float
) -> float:
    a = dx * d2y - dy * d2x
    b = dx * d3y - dy * d3x
    c = dx * d2x + dy * d2y
    d = dx * dx + dy * dy
    return _divide_like_cpp(b * d - 3.0 * a * c, d * d * d)
