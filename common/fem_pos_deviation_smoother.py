"""
FEM-POS deviation smoother for reference line (OSQP)
"""
import osqp
import numpy as np
from logging import Logger
import scipy.sparse as sparse
from typing import List, Sequence, Tuple

logger = Logger("FemPosDeviationSmoother")

class FemPosDeviationSmoother:
    """
    Port of Apollo FemPosDeviationOsqpInterface without curvature constraints.
    This section has been significantly streamlined, as the lattice optimization relies solely on OSQP.
    """
    weight_fem_pos_deviation: float
    weight_ref_deviation: float
    weight_path_length: float
    max_iter: int

    def __init__(self, weight_fem_pos_deviation: float = 1e10, weight_ref_deviation: float = 1.0,
                 weight_path_length: float = 1.0, max_iter: int = 500) -> None:
        """
        Constructor

        :param float weight_fem_pos_deviation: weight of the FEM positional deviation term
        :param float weight_ref_deviation: weight of the reference-path deviation term
        :param float weight_path_length: weight of the path-length penalty term
        :param int max_iter: maximum number of optimization iterations
        """
        self.weight_fem_pos_deviation = weight_fem_pos_deviation
        self.weight_ref_deviation = weight_ref_deviation
        self.weight_path_length = weight_path_length
        self.max_iter = max_iter

    def Solve(self, raw_point2d: Sequence[Tuple[float, float]],
              bounds: Sequence[float]
              ) -> Tuple[bool, List[float], List[float]]:
        """
        Mirrors FemPosDeviationOsqpInterface::Solve() (fem_pos_deviation_osqp_interface.cc),
        which aborts (returns false) rather than substituting the raw, unsmoothed
        points -- callers must check the returned success flag instead of assuming
        opt_x/opt_y are always a valid smoothed result.

        :param Sequence[Tuple[float, float]]: the original raw points 2D
        :param Sequence[float] bounds: 
        :returns: 
        :rtype: Tuple[bool, List[float], List[float]]
        """
        n = len(raw_point2d)
        if n < 3 or len(bounds) != n:
            return False, [], []

        num_vars = n * 2
        x_weight = self.weight_fem_pos_deviation
        y_weight = self.weight_path_length
        z_weight = self.weight_ref_deviation

        rows, cols, data = [], [], []
        for col in range(num_vars):
            point_index = col // 2

            def add_entry(row: int, value: float) -> None:
                """
                Record one upper-triangle entry of the kernel at (row, col),
                where col comes from the enclosing loop. Values arrive already
                multiplied by 2.0, because OSQP's objective is (1/2) * x' * P * x,
                the same rescaling FemPosDeviationOsqpInterface::CalculateKernel does

                :param int row: the row index of the entry, never greater than col
                :param float value: the entry value, already scaled by 2.0
                """
                rows.append(row)
                cols.append(col)
                data.append(value)

            if point_index == 0:
                add_entry(col, (x_weight + y_weight + z_weight) * 2.0)
            elif point_index == 1:
                add_entry(col - 2, (-2.0 * x_weight - y_weight) * 2.0)
                # For three points the only second difference is p0 - 2*p1 + p2,
                # so p1 contributes 4 to the FEM diagonal instead of 4 + 1.
                fem_diagonal = 4.0 if n == 3 else 5.0
                add_entry(col, (fem_diagonal * x_weight + 2.0 * y_weight + z_weight) * 2.0)
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
