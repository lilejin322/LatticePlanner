"""
Shared 2D geometry helper utils
"""
import math
from common.vec2d import Vec2d, kMathEpsilon

__all__ = [
    "kMathEpsilon",
    "WrapAngle",
    "CrossProd",
    "NormalizeAngle",
    "AngleDiff",
    "value_or_zero",
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
