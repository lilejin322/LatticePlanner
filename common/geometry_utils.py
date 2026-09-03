"""Shared 2D geometry helpers (no dependency on Box2d/Polygon2d/Path)."""

import math

from common.vec2d import Vec2d, kMathEpsilon

__all__ = [
    "kMathEpsilon",
    "WrapAngle",
    "CrossProd",
    "NormalizeAngle",
    "AngleDiff",
]


def WrapAngle(angle: float) -> float:
    """Wrap an angle to [0, 2*pi)."""
    new_angle = math.fmod(angle, math.pi * 2.0)
    return new_angle + math.pi * 2.0 if new_angle < 0 else new_angle


def CrossProd(start_point: Vec2d, end_point_1: Vec2d, end_point_2: Vec2d) -> float:
    """Cross product of vectors (end_point_1 - start) x (end_point_2 - start)."""
    return (end_point_1 - start_point).CrossProd(end_point_2 - start_point)


def NormalizeAngle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def AngleDiff(src: float, dst: float) -> float:
    """Signed shortest angle from src to dst."""
    return NormalizeAngle(dst - src)
