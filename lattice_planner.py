"""Lattice planner aligned with modules/planning/planner/lattice/lattice_planner.cc."""

import math
import time
from typing import List, Optional, Tuple

from path_matcher import PathMatcher
from cartesian_frenet_converter import CartesianFrenetConverter
from behavior.collision_checker import CollisionChecker
from behavior.path_time_graph import PathTimeGraph
from behavior.prediction_querier import PredictionQuerier
from common.constraint_checker import ConstraintChecker
from common.discretized_trajectory import DiscretizedTrajectory
from common.frame import Frame
from common.planning_context import PlanningContext
from reference_line.reference_line_info import ReferenceLineInfo
from reference_line.reference_point import ReferencePoint
import config
from logging import Logger
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from trajectory_generation.backup_trajectory_generator import BackupTrajectoryGenerator
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator
from trajectory_generation.trajectory_combiner import TrajectoryCombiner
from trajectory_generation.trajectory_evaluator import TrajectoryEvaluator


def ToDiscretizedReferenceLine(ref_points: List[ReferencePoint]) -> List[PathPoint]:
    s = 0.0
    path_points: List[PathPoint] = []
    for ref_point in ref_points:
        path_point = PathPoint(
            x=ref_point.x,
            y=ref_point.y,
            z=0.0,
            theta=ref_point.heading,
            kappa=ref_point.kappa,
            dkappa=ref_point.dkappa,
            ddkappa=0.0,
        )
        if path_points:
            dx = path_point.x - path_points[-1].x
            dy = path_point.y - path_points[-1].y
            s += math.sqrt(dx * dx + dy * dy)
        path_point.s = s
        path_points.append(path_point)
    return path_points


def ComputeInitFrenetState(
    matched_point: PathPoint, cartesian_state: TrajectoryPoint
) -> Tuple[List[float], List[float]]:
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
    """Pure lattice 1d-pair search + TrajectoryCombiner, matching Apollo lattice_planner.cc."""

    def __init__(self):
        self.num_planning_cycles = 0
        self.num_planning_succeeded_cycles = 0
        self.logger = Logger("LatticePlanner")

    def Plan(
        self,
        planning_start_point: TrajectoryPoint,
        frame: Frame,
        computed_trajectory: ADCTrajectory,
        planning_context: Optional[PlanningContext] = None,
    ) -> bool:
        del computed_trajectory, planning_context

        success_line_count = 0
        for index, reference_line_info in enumerate(frame.mutable_reference_line_info):
            if index != 0:
                reference_line_info.SetPriorityCost(config.FLAGS_cost_non_priority_reference_line)
            else:
                reference_line_info.SetPriorityCost(0.0)

            if self.PlanOnReferenceLine(planning_start_point, frame, reference_line_info):
                success_line_count += 1
            elif reference_line_info.IsChangeLanePath():
                self.logger.error(
                    f"Planner failed to change lane to {reference_line_info.Lanes().Id()}"
                )
            else:
                self.logger.error(f"Planner failed to {reference_line_info.Lanes().Id()}")

        return success_line_count > 0

    def PlanOnReferenceLine(
        self,
        planning_init_point: TrajectoryPoint,
        frame: Frame,
        reference_line_info: ReferenceLineInfo,
    ) -> bool:
        start_time = time.time()
        current_time = start_time

        self.logger.debug(
            f"Number of planning cycles: {self.num_planning_cycles} "
            f"{self.num_planning_succeeded_cycles}"
        )
        self.num_planning_cycles += 1

        reference_line_info.set_is_on_reference_line()
        reference_line = ToDiscretizedReferenceLine(
            reference_line_info.reference_line.reference_points
        )

        matched_point = PathMatcher.MatchToPath(
            reference_line,
            planning_init_point.path_point.x,
            planning_init_point.path_point.y,
        )
        init_s, init_d = ComputeInitFrenetState(matched_point, planning_init_point)

        prediction_querier = PredictionQuerier(frame.obstacles, reference_line)
        path_time_graph = PathTimeGraph(
            prediction_querier.GetObstacles(),
            reference_line,
            reference_line_info,
            init_s[0],
            init_s[0] + config.FLAGS_speed_lon_decision_horizon,
            0.0,
            config.FLAGS_trajectory_time_length,
            init_d,
        )

        speed_limit = reference_line_info.reference_line.GetSpeedLimitFromS(init_s[0])
        reference_line_info.SetLatticeCruiseSpeed(speed_limit)

        planning_target = reference_line_info.planning_target
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
            frame.obstacles,
            init_s[0],
            init_d[0],
            reference_line,
            reference_line_info,
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
            reference_line_info.SetTrajectory(combined_trajectory)
            reference_line_info.SetCost(
                reference_line_info.PriorityCost() + trajectory_pair_cost
            )
            reference_line_info.SetDrivable(True)
            reference_line_info.set_trajectory_type(ADCTrajectory.TrajectoryType.NORMAL)
            break

        self.logger.debug(f"Trajectory_Evaluation_Time = {time.time() - current_time}")
        self.logger.debug(f"Total_Lattice_Planning_Frame_Time = {time.time() - start_time}")

        if num_lattice_traj > 0:
            self.num_planning_succeeded_cycles += 1
            return True

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
            if not trajectory:
                self.logger.error("Backup trajectory generator returned an empty trajectory")
                reference_line_info.SetCost(float("inf"))
                reference_line_info.SetDrivable(False)
                return False
            reference_line_info.AddCost(config.FLAGS_backup_trajectory_cost)
            reference_line_info.SetTrajectory(trajectory)
            reference_line_info.SetDrivable(True)
            reference_line_info.set_trajectory_type(
                ADCTrajectory.TrajectoryType.PATH_FALLBACK
            )
            return True

        reference_line_info.SetCost(float("inf"))
        return False
