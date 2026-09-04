import heapq
from typing import List, Tuple
from common.curve1d.curve1d import Curve1d
from protoclass.path_point import PathPoint
from behavior.collision_checker import CollisionChecker
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator
from trajectory_generation.trajectory_combiner import TrajectoryCombiner
from common.curve1d.constant_deceleration_trajectory1d import ConstantDecelerationTrajectory1d
from common.discretized_trajectory import DiscretizedTrajectory
import config as config_module

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
        self.trajectory_pair_pqueue: List[Tuple[float, int, Tuple[Curve1d, Curve1d]]] = []
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
            s_dot = lon_trajectory.Evaluate(1, config_module.FLAGS_trajectory_time_length)
            return s_dot
    
        sequence = 0
        for lon in lon_trajectories:
            for lat in lat_trajectories:
                # Match Apollo's queue priority: larger terminal speed first.
                heapq.heappush(self.trajectory_pair_pqueue, (-CostComparator((lon, lat)), sequence, (lon, lat)))
                sequence += 1

    def GenerateBestPair(
        self, discretized_ref_points: List[PathPoint]
    ) -> Tuple[Curve1d, Curve1d, DiscretizedTrajectory]:
        while len(self.trajectory_pair_pqueue) > 1:
            _, _, top_pair = heapq.heappop(self.trajectory_pair_pqueue)
            trajectory = TrajectoryCombiner.Combine(
                discretized_ref_points,
                top_pair[0],
                top_pair[1],
                self.init_relative_time,
            )
            if not self.collision_checker.InCollision(trajectory):
                return top_pair[0], top_pair[1], trajectory

        _, _, top_pair = self.trajectory_pair_pqueue[0]
        trajectory = TrajectoryCombiner.Combine(
            discretized_ref_points,
            top_pair[0],
            top_pair[1],
            self.init_relative_time,
        )
        return top_pair[0], top_pair[1], trajectory

    def GenerateTrajectory(self, discretized_ref_points: List[PathPoint]) -> DiscretizedTrajectory:
        """
        Generate trajectory

        :param List[PathPoint] discretized_ref_points: discretized reference points
        :returns: generated trajectory
        :rtype: DiscretizedTrajectory
        """

        _, _, trajectory = self.GenerateBestPair(discretized_ref_points)
        return trajectory
