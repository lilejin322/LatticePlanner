"""Affine constraint container aligned with Apollo AffineConstraint."""

from __future__ import annotations

import numpy as np


class AffineConstraint:
    def __init__(self, is_equality: bool = False):
        self._is_equality = is_equality
        self._constraint_matrix = np.zeros((0, 0))
        self._constraint_boundary = np.zeros((0, 1))

    def set_is_equality(self, is_equality: bool) -> None:
        self._is_equality = is_equality

    def add_constraint(self, constraint_matrix: np.ndarray, constraint_boundary: np.ndarray) -> bool:
        if constraint_matrix.size == 0:
            return True
        if self._constraint_matrix.size == 0:
            self._constraint_matrix = constraint_matrix
            self._constraint_boundary = constraint_boundary
            return True
        if constraint_matrix.shape[1] != self._constraint_matrix.shape[1]:
            return False
        self._constraint_matrix = np.vstack([self._constraint_matrix, constraint_matrix])
        self._constraint_boundary = np.vstack([self._constraint_boundary, constraint_boundary])
        return True

    @property
    def constraint_matrix(self) -> np.ndarray:
        return self._constraint_matrix

    @property
    def constraint_boundary(self) -> np.ndarray:
        return self._constraint_boundary

    @property
    def is_equality(self) -> bool:
        return self._is_equality
