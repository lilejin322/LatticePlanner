from abc import abstractmethod
from typing import List, Tuple
from config import FLAGS_default_delta_s_lateral_optimization
from common.trajectory1d.PiecewiseJerkTrajectory1d import PiecewiseJerkTrajectory1d
from common.FrenetFramePoint import FrenetFramePoint

class LateralQPOptimizer:
    """
    LateralQPOptimizer class
    """

    def __init__(self):
        """
        Constructor
        """

        self._delta_s: float = FLAGS_default_delta_s_lateral_optimization

        self._opt_d: List[float] = []

        self._opt_d_prime: List[float] = []

        self._opt_d_pprime: List[float] = []
    
    @abstractmethod
    def optimize(self, d_state: List[float], delta_s: float, d_bounds: List[Tuple[float, float]]) -> bool:
        """
        virtual function to be implemented in class LateralOSQPOptimizer

        :param List[float] d_state: d state
        :param float delta_s: delta s
        :param List[Tuple[float, float]] d_bounds: d bounds
        :returns: whether optimization is successful
        :rtype: bool
        """

        pass

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

    def GetFrenetFramePath(self) -> FrenetFramePoint:
        """
        Get frenet frame path

        :returns: frenet frame path
        :rtype: FrenetFramePoint
        """

        frenet_frame_path: List[FrenetFramePoint] = []
        accumulated_s: float = 0.0
        for i in range(len(self._opt_d)):
            frenet_frame_point: FrenetFramePoint = FrenetFramePoint()
            frenet_frame_point.set_s(accumulated_s)
            frenet_frame_point.set_l(self._opt_d[i])
            frenet_frame_point.set_dl(self._opt_d_prime[i])
            frenet_frame_point.set_ddl(self._opt_d_pprime[i])
            frenet_frame_path.append(frenet_frame_point)
            accumulated_s += self._delta_s
        return frenet_frame_path
