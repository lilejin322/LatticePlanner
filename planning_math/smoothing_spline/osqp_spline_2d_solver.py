"""
OSQP-based 2D spline solver aligned with osqp_spline_2d_solver.cc.
"""
from __future__ import annotations
from typing import List
import numpy as np
from osqp import OSQP
import scipy.sparse as sparse
from planning_math.smoothing_spline.spline_2d import Spline2d
from planning_math.smoothing_spline.spline_2d_constraint import Spline2dConstraint
from planning_math.smoothing_spline.spline_2d_kernel import Spline2dKernel

class OsqpSpline2dSolver:
    """
    OSQP-based 2D spline solver aligned with osqp_spline_2d_solver.cc.
    """
    _t_knots: List[float]
    _order: int
    _spline: Spline2d
    _kernel: Spline2dKernel
    _constraint: Spline2dConstraint

    def __init__(self, t_knots: List[float], order: int) -> None:
        """
        Constructor

        :param List[float] t_knots: The knot points for the spline.
        :param int order: The order of the spline.
        :returns: None
        """
        self._t_knots = list(t_knots)
        self._order = order
        self._spline = Spline2d(t_knots, order)
        self._kernel = Spline2dKernel(t_knots, order)
        self._constraint = Spline2dConstraint(t_knots, order)

    def reset(self, t_knots: List[float], order: int) -> None:
        """
        Reset the solver with new knot points and order.
        
        :param List[float] t_knots: The new knot points for the spline.
        :param int order: The new order of the spline.
        :returns: None
        """
        self._t_knots = list(t_knots)
        self._order = order
        self._spline = Spline2d(t_knots, order)
        self._kernel = Spline2dKernel(t_knots, order)
        self._constraint = Spline2dConstraint(t_knots, order)

    @property
    def mutable_constraint(self) -> Spline2dConstraint:
        """
        Get the mutable constraint object for the spline solver.
        
        :returns: The mutable constraint object.
        :rtype: Spline2dConstraint
        """
        return self._constraint

    @property
    def mutable_kernel(self) -> Spline2dKernel:
        """
        Get the mutable kernel object for the spline solver.

        :returns: The mutable kernel object.
        :rtype: Spline2dKernel
        """
        return self._kernel

    @property
    def spline(self) -> Spline2d:
        """
        Get the spline object for the spline solver.

        :returns: The spline object.
        :rtype: Spline2d
        """
        return self._spline

    def solve(self) -> bool:
        """
        Solve the spline optimization problem using OSQP.

        :returns: True if the optimization was successful, False otherwise.
        :rtype: bool
        """
        p_mat = self._kernel.kernel_matrix()
        if p_mat.shape[0] == 0:
            return False

        inequality_matrix = self._constraint.inequality_constraint.constraint_matrix
        equality_matrix = self._constraint.equality_constraint.constraint_matrix
        if inequality_matrix.size == 0 and equality_matrix.size == 0:
            return False

        if inequality_matrix.size == 0:
            a_mat = equality_matrix
        elif equality_matrix.size == 0:
            a_mat = inequality_matrix
        else:
            a_mat = np.vstack([inequality_matrix, equality_matrix])

        q_vec = self._kernel.offset.reshape(-1)
        ineq_boundary = self._constraint.inequality_constraint.constraint_boundary
        eq_boundary = self._constraint.equality_constraint.constraint_boundary
        constraint_num = ineq_boundary.shape[0] + eq_boundary.shape[0]

        lower_bounds = np.zeros(constraint_num)
        upper_bounds = np.zeros(constraint_num)
        k_upper_limit = 1e9
        k_epsilon = 1e-9
        for i in range(constraint_num):
            if i < ineq_boundary.shape[0]:
                lower_bounds[i] = ineq_boundary[i, 0]
                upper_bounds[i] = k_upper_limit
            else:
                idx = i - ineq_boundary.shape[0]
                val = eq_boundary[idx, 0]
                lower_bounds[i] = val - k_epsilon
                upper_bounds[i] = val + k_epsilon

        solver = OSQP()
        solver.setup(
            P=sparse.csc_matrix(p_mat),
            q=q_vec,
            A=sparse.csc_matrix(a_mat),
            l=lower_bounds,
            u=upper_bounds,
            alpha=1.0,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=5000,
            polish=True,
            verbose=False,
        )
        result = solver.solve()
        solved_params = result.x.reshape(-1, 1)
        return self._spline.set_splines(solved_params, self._spline.spline_order)
