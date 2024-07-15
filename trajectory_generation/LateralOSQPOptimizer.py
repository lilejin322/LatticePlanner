from trajectory_generation.LateralQPOptimizer import LateralQPOptimizer
from typing import List, Tuple, override
from config import FLAGS_lateral_third_order_derivative_max, FLAGS_weight_lateral_obstacle_distance, FLAGS_enable_osqp_debug, \
                   FLAGS_weight_lateral_offset, FLAGS_weight_lateral_derivative, FLAGS_weight_lateral_second_order_derivative
from osqp import OSQP
from scipy import sparse

class LateralOSQPOptimizer(LateralQPOptimizer):
    """
    LateralOSQPOptimizer class
    """

    def __init__(self):
        """
        Constructor
        """

        super(LateralOSQPOptimizer, self).__init__()

    @override
    def Optimize(self, d_state: List[float], delta_s: float, d_bounds: List[Tuple[float, float]]) -> bool:
        """
        Optimize function, override the base class function

        :param List[float] d_state: d state
        :param float delta_s: delta s
        :param List[Tuple[float, float]] d_bounds: d bounds
        :returns: whether optimization is successful
        :rtype: bool
        """

        P_data: List[float] = []
        P_indices: List[int] = []
        P_indptr: List[int] = []
        self.CalculateKernel(d_bounds, P_data, P_indices, P_indptr)
        num_var: int = len(d_bounds)
        kNumParam: int = 3 * num_var
        kNumConstraint: int = kNumParam + 3 * (num_var - 1) + 3
        lower_bounds: List[float] = [-float('inf')] * kNumConstraint
        upper_bounds: List[float] = [float('inf')] * kNumConstraint

        prime_offset: int = num_var
        pprime_offset: int = 2 * num_var

        columns: List[List[Tuple[int, float]]] = [[] for _ in range(kNumParam)]

        constraint_index: int = 0

        # d_i+1'' - d_i''
        for i in range(num_var - 1):
            columns[pprime_offset + i].append((constraint_index, -1.0))
            columns[pprime_offset + i + 1].append((constraint_index, 1.0))

            lower_bounds[constraint_index] = -FLAGS_lateral_third_order_derivative_max * delta_s
            upper_bounds[constraint_index] = FLAGS_lateral_third_order_derivative_max * delta_s
            constraint_index += 1

        # d_i+1' - d_i' - 0.5 * ds * (d_i'' + d_i+1'')
        for i in range(num_var - 1):
            columns[prime_offset + i].append((constraint_index, -1.0))
            columns[prime_offset + i + 1].append((constraint_index, 1.0))
            columns[pprime_offset + i].append((constraint_index, -0.5 * delta_s))
            columns[pprime_offset + i + 1].append((constraint_index, -0.5 * delta_s))

            lower_bounds[constraint_index] = 0.0
            upper_bounds[constraint_index] = 0.0
            constraint_index += 1

        # d_i+1 - d_i - d_i' * ds - 1/3 * d_i'' * ds^2 - 1/6 * d_i+1'' * ds^2
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
        
        assert constraint_index == kNumConstraint, "Constraint index is not equal to kNumConstraint"

        # change affine_constraint to CSC format
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

        # offset
        q: List[float] = [0.0] * kNumParam
        for i in range(kNumParam):
            if i < num_var:
                q[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance * (d_bounds[i][0] + d_bounds[i][1])

        optimizer = OSQP()

        # Populate data
        # note that _indices is the row and _indptr is the column
        matrix_p = sparse.csc_matrix((P_data, (P_indices, P_indptr)), shape=(kNumParam, kNumParam))
        matrix_a = sparse.csc_matrix((A_data, (A_indices, A_indptr)), shape=(kNumConstraint, kNumParam))

        # Problem settings
        optimizer.setup(P=matrix_p, q=q, A=matrix_a, l=lower_bounds, u=upper_bounds, alpha=1.0, 
                        eps_abs=1.0e-05, eps_rel=1.0e-05, max_iter=5000, polish=True, verbose=FLAGS_enable_osqp_debug)
        
        # Solve Problem
        res = optimizer.solve()

        # extract primal results
        self._opt_d = res.x[:num_var].tolist()
        self._opt_d_prime = res.x[num_var:2*num_var].tolist()
        self._opt_d_pprime = res.x[2*num_var:3*num_var].tolist()
        self._opt_d_prime[-1] = 0.0
        self._opt_d_pprime[-1] = 0.0

        return True

    @staticmethod
    def CalculateKernel(d_bounds: List[Tuple[float, float]], P_data: List[float], P_indices: List[int], P_indptr: List[int]) -> None:
        """
        Calculate kernel

        :param List[Tuple[float, float]] d_bounds: d bounds
        :param List[float] P_data: P data
        :param List[int] P_indices: P indices
        :param List[int] P_indptr: P indptr
        """

        kNumParam: int = 3 * len(d_bounds)

        # initialize the length of the list
        P_data[:] = [0] * kNumParam
        P_indices[:] = [0] * kNumParam
        P_indptr[:] = [0] * (kNumParam + 1)

        for i in range(kNumParam):
            if i < len(d_bounds):
                P_data[i] = 2.0 * FLAGS_weight_lateral_offset + 2.0 * FLAGS_weight_lateral_obstacle_distance
            elif i < 2 * len(d_bounds):
                P_data[i] = 2.0 * FLAGS_weight_lateral_derivative
            else:
                P_data[i] = 2.0 * FLAGS_weight_lateral_second_order_derivative

            P_indices[i] = i
            P_indptr[i] = i

        P_indptr[kNumParam] = kNumParam
        assert len(P_data) == len(P_indices), "Length of P_data is not equal to length of P_indices"
