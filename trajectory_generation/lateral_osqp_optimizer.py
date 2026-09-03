from trajectory_generation.lateral_qp_optimizer import LateralQPOptimizer
from typing import List, Tuple, override
import config as config_module

from osqp import OSQP
from scipy import sparse
import numpy as np


class LateralOSQPOptimizer(LateralQPOptimizer):
    """
    LateralOSQPOptimizer class
    """

    def __init__(self):
        super().__init__()

    @override
    def Optimize(self, d_state: List[float], delta_s: float, d_bounds: List[Tuple[float, float]]) -> bool:
        self._delta_s = delta_s
        P_data: List[float] = []
        P_indices: List[int] = []
        P_indptr: List[int] = []
        self.CalculateKernel(d_bounds, P_data, P_indices, P_indptr)
        num_var: int = len(d_bounds)
        kNumParam: int = 3 * num_var
        kNumConstraint: int = kNumParam + 3 * (num_var - 1) + 3
        lower_bounds: List[float] = [-float("inf")] * kNumConstraint
        upper_bounds: List[float] = [float("inf")] * kNumConstraint

        prime_offset: int = num_var
        pprime_offset: int = 2 * num_var

        columns: List[List[Tuple[int, float]]] = [[] for _ in range(kNumParam)]

        constraint_index: int = 0

        for i in range(num_var - 1):
            columns[pprime_offset + i].append((constraint_index, -1.0))
            columns[pprime_offset + i + 1].append((constraint_index, 1.0))
            lower_bounds[constraint_index] = -config_module.FLAGS_lateral_third_order_derivative_max * delta_s
            upper_bounds[constraint_index] = config_module.FLAGS_lateral_third_order_derivative_max * delta_s
            constraint_index += 1

        for i in range(num_var - 1):
            columns[prime_offset + i].append((constraint_index, -1.0))
            columns[prime_offset + i + 1].append((constraint_index, 1.0))
            columns[pprime_offset + i].append((constraint_index, -0.5 * delta_s))
            columns[pprime_offset + i + 1].append((constraint_index, -0.5 * delta_s))
            lower_bounds[constraint_index] = 0.0
            upper_bounds[constraint_index] = 0.0
            constraint_index += 1

        for i in range(num_var - 1):
            columns[i].append((constraint_index, -1.0))
            columns[i + 1].append((constraint_index, 1.0))
            columns[prime_offset + i].append((constraint_index, -delta_s))
            columns[pprime_offset + i].append((constraint_index, -delta_s**2 / 3.0))
            columns[pprime_offset + i + 1].append((constraint_index, -delta_s**2 / 6.0))
            lower_bounds[constraint_index] = 0.0
            upper_bounds[constraint_index] = 0.0
            constraint_index += 1

        columns[0].append((constraint_index, 1.0))
        lower_bounds[constraint_index] = d_state[0]
        upper_bounds[constraint_index] = d_state[0]
        constraint_index += 1

        columns[prime_offset].append((constraint_index, 1.0))
        lower_bounds[constraint_index] = d_state[1]
        upper_bounds[constraint_index] = d_state[1]
        constraint_index += 1

        columns[pprime_offset].append((constraint_index, 1.0))
        lower_bounds[constraint_index] = d_state[2]
        upper_bounds[constraint_index] = d_state[2]
        constraint_index += 1

        LARGE_VALUE = 2.0

        for i in range(kNumParam):
            columns[i].append((constraint_index, 1.0))
            if i < num_var:
                lower_bounds[constraint_index] = d_bounds[i][0]
                upper_bounds[constraint_index] = d_bounds[i][1]
            else:
                lower_bounds[constraint_index] = -LARGE_VALUE
                upper_bounds[constraint_index] = LARGE_VALUE
            constraint_index += 1

        assert constraint_index == kNumConstraint

        A_data: List[float] = []
        A_indices: List[int] = []
        A_indptr: List[int] = []
        ind_p: int = 0
        for j in range(kNumParam):
            A_indptr.append(ind_p)
            for row_data_pair in columns[j]:
                A_data.append(row_data_pair[1])
                A_indices.append(row_data_pair[0])
                ind_p += 1
        A_indptr.append(ind_p)

        q: List[float] = [0.0] * kNumParam
        for i in range(kNumParam):
            if i < num_var:
                q[i] = -2.0 * config_module.FLAGS_weight_lateral_obstacle_distance * (d_bounds[i][0] + d_bounds[i][1])

        optimizer = OSQP()
        matrix_p = sparse.csc_matrix((P_data, P_indices, P_indptr), shape=(kNumParam, kNumParam))
        matrix_a = sparse.csc_matrix((A_data, A_indices, A_indptr), shape=(kNumConstraint, kNumParam))
        optimizer.setup(
            P=matrix_p,
            q=np.asarray(q, dtype=float),
            A=matrix_a,
            l=np.asarray(lower_bounds, dtype=float),
            u=np.asarray(upper_bounds, dtype=float),
            alpha=1.0,
            eps_abs=1.0e-05,
            eps_rel=1.0e-05,
            max_iter=5000,
            polish=True,
            verbose=config_module.FLAGS_enable_osqp_debug,
        )

        res = optimizer.solve()
        status = (res.info.status or "").lower()
        if res.x is None or "solved" not in status:
            self._opt_d = []
            self._opt_d_prime = []
            self._opt_d_pprime = []
            return False

        self._opt_d = res.x[:num_var].tolist()
        self._opt_d_prime = res.x[num_var : 2 * num_var].tolist()
        self._opt_d_pprime = res.x[2 * num_var : 3 * num_var].tolist()
        self._opt_d_prime[-1] = 0.0
        self._opt_d_pprime[-1] = 0.0
        return True

    @staticmethod
    def CalculateKernel(
        d_bounds: List[Tuple[float, float]], P_data: List[float], P_indices: List[int], P_indptr: List[int]
    ) -> None:
        kNumParam: int = 3 * len(d_bounds)
        P_data[:] = [0] * kNumParam
        P_indices[:] = [0] * kNumParam
        P_indptr[:] = [0] * (kNumParam + 1)

        for i in range(kNumParam):
            if i < len(d_bounds):
                P_data[i] = 2.0 * config_module.FLAGS_weight_lateral_offset + 2.0 * config_module.FLAGS_weight_lateral_obstacle_distance
            elif i < 2 * len(d_bounds):
                P_data[i] = 2.0 * config_module.FLAGS_weight_lateral_derivative
            else:
                P_data[i] = 2.0 * config_module.FLAGS_weight_lateral_second_order_derivative
            P_indices[i] = i
            P_indptr[i] = i

        P_indptr[kNumParam] = kNumParam


def CreateLateralOptimizer() -> LateralQPOptimizer:
    return LateralOSQPOptimizer()
