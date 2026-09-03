"""
Curve geometry helper submodule
"""

def ComputeCurvature(dx: float, d2x: float, dy: float, d2y: float) -> float:
    a = dx * d2y - dy * d2x
    norm_square = dx * dx + dy * dy
    norm = norm_square ** 0.5
    b = norm * norm_square
    if abs(b) < 1e-12:
        return 0.0
    return a / b


def ComputeCurvatureDerivative(
    dx: float, d2x: float, d3x: float, dy: float, d2y: float, d3y: float
) -> float:
    a = dx * d2y - dy * d2x
    b = dx * d3y - dy * d3x
    c = dx * d2x + dy * d2y
    d = dx * dx + dy * dy
    if abs(d) < 1e-12:
        return 0.0
    return (b * d - 3.0 * a * c) / (d * d * d)
