"""Spline 2D constraints aligned with spline_2d_constraint.cc."""

from __future__ import annotations

import bisect
import math
from typing import List, Sequence, Tuple

import numpy as np

from common.vec2d import Vec2d
from planning_math.smoothing_spline.affine_constraint import AffineConstraint


class Spline2dConstraint:
    def __init__(self, t_knots: List[float], spline_order: int):
        self._t_knots = list(t_knots)
        self._spline_order = spline_order
        self._total_param = 2 * (spline_order + 1) * max(0, len(t_knots) - 1)
        self._inequality_constraint = AffineConstraint(is_equality=False)
        self._equality_constraint = AffineConstraint(is_equality=True)

    def add_inequality_constraint(
        self, constraint_matrix: np.ndarray, constraint_boundary: np.ndarray
    ) -> bool:
        return self._inequality_constraint.add_constraint(constraint_matrix, constraint_boundary)

    def add_equality_constraint(
        self, constraint_matrix: np.ndarray, constraint_boundary: np.ndarray
    ) -> bool:
        return self._equality_constraint.add_constraint(constraint_matrix, constraint_boundary)

    @property
    def inequality_constraint(self) -> AffineConstraint:
        return self._inequality_constraint

    @property
    def equality_constraint(self) -> AffineConstraint:
        return self._equality_constraint

    def add_2d_boundary(
        self,
        t_coord: Sequence[float],
        angle: Sequence[float],
        ref_point: Sequence[Vec2d],
        longitudinal_bound: Sequence[float],
        lateral_bound: Sequence[float],
    ) -> bool:
        if not (
            len(t_coord) == len(angle) == len(ref_point) == len(lateral_bound) == len(longitudinal_bound)
        ):
            return False
        num_params = self._spline_order + 1
        affine_inequality = np.zeros((4 * len(t_coord), self._total_param))
        affine_boundary = np.zeros((4 * len(t_coord), 1))
        for i, t in enumerate(t_coord):
            d_lateral = self._sign_distance(ref_point[i], angle[i])
            d_longitudinal = self._sign_distance(ref_point[i], angle[i] - math.pi / 2.0)
            index = self._find_index(t)
            rel_t = t - self._t_knots[index]
            index_offset = 2 * index * num_params
            longi_coef = self._affine_coef(angle[i], rel_t)
            longitudinal_coef = self._affine_coef(angle[i] - math.pi / 2.0, rel_t)
            for j in range(2 * num_params):
                affine_inequality[4 * i, index_offset + j] = longi_coef[j]
                affine_inequality[4 * i + 1, index_offset + j] = -longi_coef[j]
                affine_inequality[4 * i + 2, index_offset + j] = longitudinal_coef[j]
                affine_inequality[4 * i + 3, index_offset + j] = -longitudinal_coef[j]
            affine_boundary[4 * i, 0] = d_lateral - lateral_bound[i]
            affine_boundary[4 * i + 1, 0] = -d_lateral - lateral_bound[i]
            affine_boundary[4 * i + 2, 0] = d_longitudinal - longitudinal_bound[i]
            affine_boundary[4 * i + 3, 0] = -d_longitudinal - longitudinal_bound[i]
        return self.add_inequality_constraint(affine_inequality, affine_boundary)

    def add_point_angle_constraint(self, t: float, angle: float) -> bool:
        num_params = self._spline_order + 1
        index = self._find_index(t)
        index_offset = index * 2 * num_params
        rel_t = t - self._t_knots[index]

        affine_equality = np.zeros((1, self._total_param))
        affine_boundary = np.zeros((1, 1))
        line_derivative_coef = self._affine_derivative_coef(angle, rel_t)
        for i, coef in enumerate(line_derivative_coef):
            affine_equality[0, i + index_offset] = coef
        if not self.add_equality_constraint(affine_equality, affine_boundary):
            return False

        affine_inequality = np.zeros((2, self._total_param))
        affine_inequality_boundary = np.zeros((2, 1))
        t_coef = self._derivative_coef(rel_t)
        normalized_angle = angle % (2.0 * math.pi)
        if normalized_angle < 0:
            normalized_angle += 2.0 * math.pi
        x_sign = -1 if (math.pi / 2.0) < normalized_angle < (math.pi * 1.5) else 1
        y_sign = -1 if normalized_angle >= math.pi else 1
        for i, coef in enumerate(t_coef):
            affine_inequality[0, i + index_offset] = coef * x_sign
            affine_inequality[1, i + index_offset + num_params] = coef * y_sign
        return self.add_inequality_constraint(affine_inequality, affine_inequality_boundary)

    def add_second_derivative_smooth_constraint(self) -> bool:
        if len(self._t_knots) < 3:
            return True
        num_params = self._spline_order + 1
        rows = 6 * (len(self._t_knots) - 2)
        affine_equality = np.zeros((rows, self._total_param))
        affine_boundary = np.zeros((rows, 1))
        for i in range(len(self._t_knots) - 2):
            rel_t = self._t_knots[i + 1] - self._t_knots[i]
            index_offset = 2 * i * num_params
            power_t = self._poly_coef(rel_t)
            derivative_t = self._derivative_coef(rel_t)
            second_derivative_t = self._second_derivative_coef(rel_t)
            for j in range(num_params):
                affine_equality[6 * i, j + index_offset] = power_t[j]
                affine_equality[6 * i + 1, j + index_offset] = derivative_t[j]
                affine_equality[6 * i + 2, j + index_offset] = second_derivative_t[j]
                affine_equality[6 * i + 3, j + index_offset + num_params] = power_t[j]
                affine_equality[6 * i + 4, j + index_offset + num_params] = derivative_t[j]
                affine_equality[6 * i + 5, j + index_offset + num_params] = second_derivative_t[j]
            affine_equality[6 * i, index_offset + 2 * num_params] = -1.0
            affine_equality[6 * i + 1, index_offset + 2 * num_params + 1] = -1.0
            affine_equality[6 * i + 2, index_offset + 2 * num_params + 2] = -2.0
            affine_equality[6 * i + 3, index_offset + 3 * num_params] = -1.0
            affine_equality[6 * i + 4, index_offset + 3 * num_params + 1] = -1.0
            affine_equality[6 * i + 5, index_offset + 3 * num_params + 2] = -2.0
        return self.add_equality_constraint(affine_equality, affine_boundary)

    def _find_index(self, t: float) -> int:
        upper = bisect.bisect_right(self._t_knots, t, 1, len(self._t_knots))
        return min(len(self._t_knots) - 1, upper) - 1

    def _poly_coef(self, t: float) -> List[float]:
        result = [1.0] * (self._spline_order + 1)
        for i in range(1, len(result)):
            result[i] = result[i - 1] * t
        return result

    def _derivative_coef(self, t: float) -> List[float]:
        result = [0.0] * (self._spline_order + 1)
        power_t = self._poly_coef(t)
        for i in range(1, len(result)):
            result[i] = power_t[i - 1] * i
        return result

    def _second_derivative_coef(self, t: float) -> List[float]:
        result = [0.0] * (self._spline_order + 1)
        power_t = self._poly_coef(t)
        for i in range(2, len(result)):
            result[i] = power_t[i - 2] * i * (i - 1)
        return result

    def _affine_coef(self, angle: float, t: float) -> List[float]:
        num_params = self._spline_order + 1
        result = [0.0] * (num_params * 2)
        x_coef = -math.sin(angle)
        y_coef = math.cos(angle)
        for i in range(num_params):
            result[i] = x_coef
            result[i + num_params] = y_coef
            x_coef *= t
            y_coef *= t
        return result

    def _affine_derivative_coef(self, angle: float, t: float) -> List[float]:
        num_params = self._spline_order + 1
        result = [0.0] * (num_params * 2)
        x_coef = -math.sin(angle)
        y_coef = math.cos(angle)
        power_t = self._poly_coef(t)
        for i in range(1, num_params):
            result[i] = x_coef * power_t[i - 1] * i
            result[i + num_params] = y_coef * power_t[i - 1] * i
        return result

    @staticmethod
    def _sign_distance(xy_point: Vec2d, angle: float) -> float:
        return xy_point.x * (-math.sin(angle)) + xy_point.y * math.cos(angle)
