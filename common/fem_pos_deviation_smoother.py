"""
FEM-POS deviation smoother for reference line (OSQP)
"""

from __future__ import annotations
from typing import List, Sequence, Tuple
import osqp
import scipy.sparse as sparse


class FemPosDeviationSmoother:
    """
    Port of Apollo FemPosDeviationOsqpInterface without curvature constraints.
    """

    def __init__(
        self,
        weight_fem_pos_deviation: float = 1e10,
        weight_ref_deviation: float = 1.0,
        weight_path_length: float = 1.0,
        max_iter: int = 500,
    ):
        self.weight_fem_pos_deviation = weight_fem_pos_deviation
        self.weight_ref_deviation = weight_ref_deviation
        self.weight_path_length = weight_path_length
        self.max_iter = max_iter

    def Solve(
        self,
        raw_point2d: Sequence[Tuple[float, float]],
        bounds: Sequence[float],
    ) -> Tuple[bool, List[float], List[float]]:
        """
        Mirrors FemPosDeviationOsqpInterface::Solve() (fem_pos_deviation_osqp_interface.cc),
        which aborts (returns false) rather than substituting the raw, unsmoothed
        points -- callers must check the returned success flag instead of assuming
        opt_x/opt_y are always a valid smoothed result.
        """
        n = len(raw_point2d)
        if n < 3 or len(bounds) != n:
            return False, [], []

        import numpy as np

        num_vars = n * 2
        x_weight = self.weight_fem_pos_deviation
        y_weight = self.weight_path_length
        z_weight = self.weight_ref_deviation

        rows, cols, data = [], [], []
        for col in range(num_vars):
            point_index = col // 2
            is_x = col % 2 == 0
            ref = raw_point2d[point_index][0 if is_x else 1]

            def add_entry(row, value):
                rows.append(row)
                cols.append(col)
                data.append(value)

            if point_index == 0:
                add_entry(col, (x_weight + y_weight + z_weight) * 2.0)
            elif point_index == 1:
                add_entry(col - 2, (-2.0 * x_weight - y_weight) * 2.0)
                add_entry(col, (5.0 * x_weight + 2.0 * y_weight + z_weight) * 2.0)
            elif point_index == n - 2:
                add_entry(col - 4, x_weight * 2.0)
                add_entry(col - 2, (-4.0 * x_weight - y_weight) * 2.0)
                add_entry(col, (5.0 * x_weight + 2.0 * y_weight + z_weight) * 2.0)
            elif point_index == n - 1:
                add_entry(col - 4, x_weight * 2.0)
                add_entry(col - 2, (-2.0 * x_weight - y_weight) * 2.0)
                add_entry(col, (x_weight + y_weight + z_weight) * 2.0)
            else:
                add_entry(col - 4, x_weight * 2.0)
                add_entry(col - 2, (-4.0 * x_weight - y_weight) * 2.0)
                add_entry(col, (6.0 * x_weight + 2.0 * y_weight + z_weight) * 2.0)

        p_mat = sparse.csc_matrix((data, (rows, cols)), shape=(num_vars, num_vars))
        a_mat = sparse.eye(num_vars, format="csc")

        q = np.zeros(num_vars)
        for i, (x_ref, y_ref) in enumerate(raw_point2d):
            q[i * 2] = -2.0 * z_weight * x_ref
            q[i * 2 + 1] = -2.0 * z_weight * y_ref

        lower = np.zeros(num_vars)
        upper = np.zeros(num_vars)
        for i, (x_ref, y_ref) in enumerate(raw_point2d):
            bound = bounds[i]
            lower[i * 2] = x_ref - bound
            upper[i * 2] = x_ref + bound
            lower[i * 2 + 1] = y_ref - bound
            upper[i * 2 + 1] = y_ref + bound

        solver = osqp.OSQP()
        solver.setup(
            P=p_mat,
            q=q,
            A=a_mat,
            l=lower,
            u=upper,
            verbose=False,
            max_iter=self.max_iter,
            scaled_termination=True,
        )

        primal_warm_start = np.zeros(num_vars)
        for i, (x_ref, y_ref) in enumerate(raw_point2d):
            primal_warm_start[i * 2] = x_ref
            primal_warm_start[i * 2 + 1] = y_ref
        solver.warm_start(x=primal_warm_start)

        result = solver.solve()
        if result.info.status_val not in (1, 2):
            return False, [], []

        opt_x = [result.x[i * 2] for i in range(n)]
        opt_y = [result.x[i * 2 + 1] for i in range(n)]
        return True, opt_x, opt_y
