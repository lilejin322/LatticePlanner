"""
Affine constraint container aligned with Apollo AffineConstraint.
"""
from __future__ import annotations
import numpy as np

class AffineConstraint:
    """
    Affine constraint container aligned with Apollo AffineConstraint.
    """
    _is_equality: bool
    _constraint_matrix: np.ndarray
    _constraint_boundary: np.ndarray

    def __init__(self, is_equality: bool = False) -> None:
        """
        Constructor

        :param bool is_equality: Whether the constraint is an equality constraint (True) or an inequality constraint (False).
        """
        self._is_equality = is_equality
        self._constraint_matrix = np.zeros((0, 0))
        self._constraint_boundary = np.zeros((0, 1))

    def set_is_equality(self, is_equality: bool) -> None:
        """
        Set whether the constraint is an equality constraint (True) or an inequality constraint (False).

        :param bool is_equality: Whether the constraint is an equality constraint (True) or an inequality constraint (False).
        :returns: None
        """
        self._is_equality = is_equality

    def add_constraint(self, constraint_matrix: np.ndarray, constraint_boundary: np.ndarray) -> bool:
        """
        Add a constraint to the affine constraint.

        :param np.ndarray constraint_matrix: The constraint matrix.
        :param np.ndarray constraint_boundary: The constraint boundary.
        :returns: True if the constraint was added successfully, False otherwise.
        :rtype: bool
        """
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
        """
        Get the constraint matrix.

        :returns: The constraint matrix.
        :rtype: np.ndarray
        """
        return self._constraint_matrix

    @property
    def constraint_boundary(self) -> np.ndarray:
        """
        Get the constraint boundary.

        :returns: The constraint boundary.
        :rtype: np.ndarray
        """
        return self._constraint_boundary

    @property
    def is_equality(self) -> bool:
        """
        Get whether the constraint is an equality constraint (True) or an inequality constraint (False).

        :returns: Whether the constraint is an equality constraint (True) or an inequality constraint (False).
        :rtype: bool
        """
        return self._is_equality
