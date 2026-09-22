"""
Shared 2D geometry and interpolation helper utils
"""
import math
from logging import Logger
from common.vec2d import Vec2d, kMathEpsilon

logger: Logger = Logger("geometry_utils")

__all__ = [
    "kMathEpsilon",
    "WrapAngle",
    "CrossProd",
    "NormalizeAngle",
    "AngleDiff",
    "value_or_zero",
    "lerp",
]

def WrapAngle(angle: float) -> float:
    """
    Wrap an angle to [0, 2*pi).
    
    :param float angle:
    :returns: the wrapped angle
    :rtype: float
    """
    new_angle = math.fmod(angle, math.pi * 2.0)
    return new_angle + math.pi * 2.0 if new_angle < 0 else new_angle

def CrossProd(start_point: Vec2d, end_point_1: Vec2d, 
              end_point_2: Vec2d) -> float:
    """
    Cross product of vectors (end_point_1 - start) x (end_point_2 - start).
    
    :param Vec2d start_point: 
    :param Vec2d end_point_1: 
    :param Vec2d end_point_2: 
    :returns: 
    :rtype: 
    """
    return (end_point_1 - start_point).CrossProd(end_point_2 - start_point)

def NormalizeAngle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi].
    
    :param float angle: 
    :returns: 
    :rtype: 
    """
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi

def AngleDiff(src: float, dst: float) -> float:
    """
    Signed shortest angle from src to dst.
    
    :param float src: 
    :param float dst: 
    :returns: 
    :rtype: 
    """
    return NormalizeAngle(dst - src)

def value_or_zero(value: float | None) -> float:
    """
    Return the protobuf default for an unset numeric scalar
    align to the upstream proto2 & cpp logic

    :param float | None value:
    :returns: the value, 0.0 if value is None
    :rtype: float
    """
    return 0.0 if value is None else value

def lerp(x0: float, t0: float, x1: float, t1: float, t: float) -> float:
    """
    Linear interpolation of x between (t0, x0) and (t1, x1), evaluated at t.

    :param float x0: the x0 value
    :param float t0: the t0 value
    :param float x1: the x1 value
    :param float t1: the t1 value
    :param float t: the t value
    :returns: the interpolated value
    :rtype: float
    """
    if abs(t1 - t0) <= 1.0e-6:
        logger.error("Input time difference is too small")
        return x0
    r = (t - t0) / (t1 - t0)
    x = x0 + r * (x1 - x0)
    return x
