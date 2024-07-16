import heapq
from typing import List, Tuple
from common.curve1d.Curve1d import Curve1d
from protoclass.PathPoint import PathPoint
from behavior.CollisionChecker import CollisionChecker
from trajectory_generation.Trajectory1dGenerator import Trajectory1dGenerator
from TrajectoryCombiner import TrajectoryCombiner
from common.curve1d.ConstantDecelerationTrajectory1d import ConstantDecelerationTrajectory1d
from common.DiscretizedTrajectory import DiscretizedTrajectory
from config import FLAGS_trajectory_time_length

PairCost = Tuple[Tuple[Curve1d, Curve1d], float]

class BackupTrajectoryGenerator:
    """
    BackupTrajectoryGenerator class
    """

    def __init__(self, init_s: List[float], init_d: List[float], init_relative_time: float,
                 collision_checker: CollisionChecker,
                 trajectory1d_generator: Trajectory1dGenerator):
        """
        Constructor

        :param State init_s: initial s state
        :param State init_d: initial d state
        :param float init_relative_time: initial relative time
        :param CollisionChecker collision_checker: collision checker
        :param Trajectory1dGenerator trajectory1d_generator: trajectory1d generator
        """

        self.init_relative_time = init_relative_time
        self.collision_checker = collision_checker
        self.trajectory1d_generator = trajectory1d_generator
        self.trajectory_pair_pqueue: List[Tuple[float, Tuple[Curve1d, Curve1d]]] = []
        self.GenerateTrajectory1dPairs(init_s, init_d)

    def GenerateTrajectory1dPairs(self, init_s: List[float], init_d: List[float]) -> None:
        """
        Generate trajectory 1d pairs

        :param State init_s: initial s state
        :param State init_d: initial d state
        """

        lon_trajectories: List[Curve1d] = []
        dds_condidates: List[float] = [-0.1, -1.0, -2.0, -3.0, -4.0]
        for dds in dds_condidates:
            lon_trajectories.append(ConstantDecelerationTrajectory1d(init_s[0], init_s[1], dds))

        lat_trajectories: list[Curve1d] = []
        self.trajectory1d_generator.GenerateLateralTrajectoryBundle(lat_trajectories)

        def CostComparator(trajectory_pair: Tuple[Curve1d, Curve1d]) -> float:
            """
            Cost comparator

            :param Tuple[Curve1d, Curve1d] trajectory_pair: trajectory pair
            :returns: calculated cost priority
            :rtype: float
            """
        
            lon_trajectory = trajectory_pair[0]
            s_dot = lon_trajectory.Evaluate(1, FLAGS_trajectory_time_length)
            return s_dot
    
        for lon in lon_trajectories:
            for lat in lat_trajectories:
                # Note that in C++, the priority queue is a max heap, so we need to negate the cost
                heapq.heappush(self.trajectory_pair_pqueue, (-CostComparator(lon, lat), (lon, lat)))

    def GenerateTrajectory(self, discretized_ref_points: List[PathPoint]) -> DiscretizedTrajectory:
        """
        Generate trajectory

        :param List[PathPoint] discretized_ref_points: discretized reference points
        :returns: generated trajectory
        :rtype: DiscretizedTrajectory
        """

        while len(self.trajectory_pair_pqueue) > 1:
            # Note that trajectory_pair_pqueue is List[Tuple[weight, Tuple[Curve1d, Curve1d]]]
            top_pair: Tuple[Curve1d, Curve1d] = heapq.heappop(self.trajectory_pair_pqueue)[1]
            trajectory: DiscretizedTrajectory = TrajectoryCombiner.Combine(discretized_ref_points, top_pair[0],
                                                                           top_pair[1], self.init_relative_time)
            if not self.collision_checker.InCollision(trajectory):
                return trajectory
        top_pair: Tuple[Curve1d, Curve1d] = self.trajectory_pair_pqueue[0]
        return TrajectoryCombiner.Combine(discretized_ref_points, top_pair[0], top_pair[1], self.init_relative_time)
