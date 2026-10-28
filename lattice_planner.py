"""
Lattice planner aligned with modules/planning/planner/lattice/lattice_planner.cc
"""
import time
import config
from logging import Logger
from typing import List, Tuple
from path_matcher import PathMatcher
from cartesian_frenet_converter import CartesianFrenetConverter
from behavior.collision_checker import CollisionChecker
from behavior.path_time_graph import PathTimeGraph
from behavior.prediction_querier import PredictionQuerier
from common.constraint_checker import ConstraintChecker
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.path_point import PathPoint
from protoclass.planning_target import PlanningTarget
from protoclass.trajectory_point import TrajectoryPoint
from trajectory_generation.backup_trajectory_generator import BackupTrajectoryGenerator
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator
from trajectory_generation.trajectory_combiner import TrajectoryCombiner
from trajectory_generation.trajectory_evaluator import TrajectoryEvaluator

def ComputeInitFrenetState(matched_point: PathPoint,
                           cartesian_state: TrajectoryPoint) -> Tuple[List[float],List[float]]:
    """
    Compute the initial Frenet state based on the matched point and Cartesian state.

    :param PathPoint matched_point: the matched point on the reference line
    :param TrajectoryPoint cartesian_state: the Cartesian state
    :returns: the initial Frenet state
    :rtype: Tuple[List[float], List[float]]
    """
    return CartesianFrenetConverter.cartesian_to_frenet(
        matched_point.s,
        matched_point.x,
        matched_point.y,
        matched_point.theta,
        matched_point.kappa,
        matched_point.dkappa,
        cartesian_state.path_point.x,
        cartesian_state.path_point.y,
        cartesian_state.v,
        cartesian_state.a,
        cartesian_state.path_point.theta,
        cartesian_state.path_point.kappa,
    )

class LatticePlanner:
    """
    Pure lattice 1d-pair search + TrajectoryCombiner, matching Apollo lattice_planner.cc.
    """
    num_planning_cycles: int
    num_planning_succeeded_cycles: int

    def __init__(self) -> None:
        """
        Constructor
        """
        self.num_planning_cycles = 0
        self.num_planning_succeeded_cycles = 0
        self.logger = Logger("LatticePlanner")

    def PlanOnLanelet(self, planning_init_point: TrajectoryPoint, obstacles, reference_points,
                      lane_width, cruise_speed: float, planning_target: PlanningTarget = None):
        """Run one lattice cycle on a lanelet centerline.

        Sampling, costs, and constraint checks are unchanged. ``lane_width`` and
        ``reference_points`` come from the CommonRoad lanelet. Returns
        ``(ok, trajectory, trajectory_type)``.
        """
        start_time = time.time()
        current_time = start_time

        self.logger.debug(
            f"Number of planning cycles: {self.num_planning_cycles} "
            f"{self.num_planning_succeeded_cycles}"
        )
        self.num_planning_cycles += 1

        reference_line = reference_points
        matched_point = PathMatcher.MatchToPath(
            reference_line,
            planning_init_point.path_point.x,
            planning_init_point.path_point.y,
        )
        init_s, init_d = ComputeInitFrenetState(matched_point, planning_init_point)

        prediction_querier = PredictionQuerier(obstacles, reference_line)
        path_time_graph = PathTimeGraph(
            prediction_querier.GetObstacles(),
            reference_line,
            lane_width,
            init_s[0],
            init_s[0] + config.FLAGS_speed_lon_decision_horizon,
            0.0,
            config.FLAGS_trajectory_time_length,
            init_d,
        )

        if planning_target is None:
            planning_target = PlanningTarget(cruise_speed=cruise_speed)
        else:
            planning_target.cruise_speed = cruise_speed
        if planning_target.stop_point is not None:
            self.logger.debug(
                f"Planning target stop s: {planning_target.stop_point.s}, "
                f"Current ego s: {init_s[0]}"
            )

        trajectory1d_generator = Trajectory1dGenerator(
            init_s, init_d, path_time_graph, prediction_querier
        )
        lon_bundle, lat_bundle = trajectory1d_generator.GenerateTrajectoryBundles(
            planning_target
        )

        trajectory_evaluator = TrajectoryEvaluator(
            init_s,
            planning_target,
            lon_bundle,
            lat_bundle,
            path_time_graph,
            reference_line,
        )

        collision_checker = CollisionChecker(
            obstacles,
            init_s[0],
            init_d[0],
            reference_line,
            lane_width,
            path_time_graph,
        )

        combined_constraint_failure_count = 0
        collision_failure_count = 0
        num_lattice_traj = 0

        while trajectory_evaluator.has_more_trajectory_pairs():
            trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost()
            trajectory_pair = trajectory_evaluator.next_top_trajectory_pair()

            combined_trajectory = TrajectoryCombiner.Combine(
                reference_line,
                trajectory_pair[0],
                trajectory_pair[1],
                planning_init_point.relative_time,
            )

            result = ConstraintChecker.ValidTrajectory(combined_trajectory)
            if result != ConstraintChecker.Result.VALID:
                combined_constraint_failure_count += 1
                continue

            if collision_checker.InCollision(combined_trajectory):
                collision_failure_count += 1
                continue

            num_lattice_traj += 1
            chosen = combined_trajectory
            chosen_type = ADCTrajectory.TrajectoryType.NORMAL
            break

        self.logger.debug(f"Trajectory_Evaluation_Time = {time.time() - current_time}")
        self.logger.debug(f"Total_Lattice_Planning_Frame_Time = {time.time() - start_time}")

        if num_lattice_traj > 0:
            self.num_planning_succeeded_cycles += 1
            return True, chosen, chosen_type

        self.logger.error("Planning failed")
        if config.FLAGS_enable_backup_trajectory:
            self.logger.error("Use backup trajectory")
            backup_generator = BackupTrajectoryGenerator(
                init_s,
                init_d,
                planning_init_point.relative_time,
                collision_checker,
                trajectory1d_generator,
            )
            trajectory = backup_generator.GenerateTrajectory(reference_line)
            return True, trajectory, ADCTrajectory.TrajectoryType.PATH_FALLBACK

        return False, None, ADCTrajectory.TrajectoryType.UNKNOWN
