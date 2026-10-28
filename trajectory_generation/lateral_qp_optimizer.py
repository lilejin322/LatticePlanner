from abc import abstractmethod
from typing import List, Tuple
import config as config_module
from common.curve1d.piecewise_jerk_trajectory1d import PiecewiseJerkTrajectory1d

class LateralQPOptimizer:
    """
    LateralQPOptimizer class
    """

    def __init__(self):
        """
        Constructor
        """

        self._delta_s: float = config_module.FLAGS_default_delta_s_lateral_optimization

        self._opt_d: List[float] = []

        self._opt_d_prime: List[float] = []

        self._opt_d_pprime: List[float] = []
    
    @abstractmethod
    def Optimize(self, d_state: List[float], delta_s: float, d_bounds: List[Tuple[float, float]]) -> bool:
        """
        virtual function to be implemented in class LateralOSQPOptimizer

        :param List[float] d_state: d state
        :param float delta_s: delta s
        :param List[Tuple[float, float]] d_bounds: d bounds
        :returns: whether optimization is successful
        :rtype: bool
        """

        raise NotImplementedError

    def GetOptimalTrajectory(self) -> PiecewiseJerkTrajectory1d:
        """
        Get optimal trajectory

        :returns: optimal trajectory
        :rtype: PiecewiseJerkTrajectory1d
        """

        assert self._opt_d and self._opt_d_prime and self._opt_d_pprime, "Optimal trajectory data cannot be empty"

        optimal_trajectory: PiecewiseJerkTrajectory1d = PiecewiseJerkTrajectory1d(self._opt_d[0],
                                                                                  self._opt_d_prime[0],
                                                                                  self._opt_d_pprime[0])
        
        for i in range(1, len(self._opt_d)):
            j: float = (self._opt_d_pprime[i] - self._opt_d_pprime[i -1]) / self._delta_s
            optimal_trajectory.AppendSegment(j, self._delta_s)
        return optimal_trajectory
