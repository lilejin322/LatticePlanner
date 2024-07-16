import math
import time
from typing import List, Optional, Tuple
from common.PathPoint import PathPoint
from common.ReferencePoint import ReferencePoint
from common.TrajectoryPoint import TrajectoryPoint
from CartesianFrenetConverter import CartesianFrenetConverter
from common.Frame import Frame 
from common.ReferenceLineInfo import ReferenceLineInfo
from common.ADCTrajectory import ADCTrajectory
from logging import Logger
from PathMatcher import PathMatcher
from config import FLAGS_speed_lon_decision_horizon, FLAGS_trajectory_time_length, FLAGS_enable_backup_trajectory, FLAGS_backup_trajectory_cost
from behavior.PathTimeGraph import PathTimeGraph
from behavior.CollisionChecker import CollisionChecker
from trajectory_generation.LatticeTrajectory1d import LatticeTrajectory1d
from trajectory_generation.Trajectory1dGenerator import Trajectory1dGenerator
from trajectory_generation.BackupTrajectoryGenerator import BackupTrajectoryGenerator
from behavior.PredictionQuerier import PredictionQuerier
from trajectory_generation.TrajectoryEvaluator import TrajectoryEvaluator
from common.DiscretizedTrajectory import DiscretizedTrajectory
from trajectory_generation.TrajectoryCombiner import TrajectoryCombiner
from common.ConstraintChecker import ConstraintChecker

def ToDiscretizedReferenceLine(ref_points: List[ReferencePoint]) -> List[PathPoint]:
    """
    Convert a list of reference points to a list of path points

    :param ref_points: list of reference points
    :returns: list of path points
    :rtype: List[PathPoint]
    """

    s = 0.0
    path_points = []
    for ref_point in ref_points:
        path_point = PathPoint(ref_point.x, ref_point.y, ref_point.heading,
                               ref_point.kappa, ref_point.dkappa)
        if path_points:
            dx = path_point.x - path_points[-1].x
            dy = path_point.y - path_points[-1].y
            s += math.sqrt(dx * dx + dy * dy)
        path_point.s = s
        path_points.append(path_point)
    return path_points

def ComputeInitFrenetState(matched_point: PathPoint, cartesian_state: TrajectoryPoint) -> Tuple[List[float], List[float]]:
    """
    Convert a matched point and a cartesian state to an initial Frenet state

    :param matched_point: matched point
    :param cartesian_state: cartesian state
    """

    rt_s, rt_d = CartesianFrenetConverter.cartesian_to_frenet(
        matched_point.s, matched_point.x, matched_point.y,
        matched_point.theta, matched_point.kappa, matched_point.dkappa,
        cartesian_state.path_point.x, cartesian_state.path_point.y,
        cartesian_state.v, cartesian_state.a,
        cartesian_state.path_point.theta,
        cartesian_state.path_point.kappa)
    return rt_s, rt_d


class LatticePlanner:

    def __init__(self):
        super().__init__()
        self.num_planning_cycles = 0
        self.num_planning_succeeded_cycles = 0
        self.logger = Logger("LatticePlanner")

    def Plan(self, planning_start_point: TrajectoryPoint, frame: Frame, computed_trajectory: ADCTrajectory) -> bool:
        """
        Plan a trajectory for the vehicle

        :param planning_start_point: the starting point for planning
        :param frame: the current frame
        :param computed_trajectory: the computed trajectory; we are confused about not using this parameter
        :returns: whether the planning was successful
        :rtype: bool
        """

        success_line_count = 0
        for index, reference_line_info in enumerate(frame.mutable_reference_line_info):
            if index != 0:
                reference_line_info.SetPriorityCost(1000.0)
            else:
                reference_line_info.SetPriorityCost(0.0)

            status = self.PlanOnReferenceLine(planning_start_point, frame, reference_line_info)
            if status != True:
                if reference_line_info.IsChangeLanePath():
                    self.logger.error(f"Planner failed to change lane to {reference_line_info.Lanes().Id()}")
                else:
                    self.logger.error(f"Planner failed to {reference_line_info.Lanes().Id()}")
            else:
                success_line_count += 1

        if success_line_count > 0:
            return True
        return False

    def PlanOnReferenceLine(self, planning_init_point: TrajectoryPoint, frame: Frame, reference_line_info: ReferenceLineInfo) -> bool:
        """
        Plan a trajectory on a reference line

        :param planning_init_point: the starting point for planning
        :param frame: the current frame
        :param reference_line_info: the returned reference line information
        :returns: status of the planning
        :rtype: bool
        """

        start_time = time.time()
        current_time = start_time

        self.logger.debug(f"Number of planning cycles: {self.num_planning_cycles} {self.num_planning_succeeded_cycles}")
        self.num_planning_cycles += 1

        reference_line_info.set_is_on_reference_line()
        # 1. obtain a reference line and transform it to the PathPoint format.
        reference_line: List[PathPoint] = ToDiscretizedReferenceLine(reference_line_info.reference_line().reference_points())

        # 2. compute the matched point of the init planning point on the reference
        # line.
        matched_point: PathPoint = PathMatcher.MatchToPath(reference_line, planning_init_point.path_point.x,
                                                           planning_init_point.path_point.y)


        # 3. according to the matched point, compute the init state in Frenet frame.
        init_s, init_d = ComputeInitFrenetState(matched_point, planning_init_point)
        self.logger.debug(f"ReferenceLine and Frenet Conversion Time: {time.time()-current_time}")
        current_time = time.time()

        prediction_querier = PredictionQuerier(frame.obstacles, reference_line)

        # 4. parse the decision and get the planning target.
        path_time_graph = PathTimeGraph(prediction_querier.GetObstacles(), reference_line, reference_line_info, init_s[0],
                                        init_s[0]+FLAGS_speed_lon_decision_horizon, 0.0, FLAGS_trajectory_time_length, init_d)

        speed_limit: float = reference_line_info.reference_line.GetSpeedLimitFromS(init_s[0])
        reference_line_info.SetLatticeCruiseSpeed(speed_limit)

        planning_target = reference_line_info.planning_target
        if planning_target.has_stop_point():
            self.logger.debug(f"Planning target stop s: {planning_target.stop_point().s}, Current ego s: {init_s[0]}")

        self.logger.debug(f"Decision_Time = {time.time()-current_time}")
        current_time = time.time()

        # 5. generate 1d trajectory bundle for longitudinal and lateral respectively.
        trajectory1d_generator = Trajectory1dGenerator(init_s, init_d, path_time_graph, prediction_querier)
        lon_trajectory1d_bundle, lat_trajectory1d_bundle = trajectory1d_generator.GenerateTrajectoryBundles(planning_target)

        self.logger.debug(f"Trajectory_Generation_Time = {time.time()-current_time}")
        current_time = time.time() 

        # 6. first, evaluate the feasibility of the 1d trajectories according to
        # dynamic constraints.
        # second, evaluate the feasible longitudinal and lateral trajectory pairs
        # and sort them according to the cost.

        trajectory_evaluator = TrajectoryEvaluator(init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
                                                        path_time_graph, reference_line)

        self.logger.debug(f"Trajectory_Evaluator__Construction_Time = {time.time()-current_time}")
        current_time = time.time()

        self.logger.debug(f"number of trajectory pairs = {trajectory_evaluator.num_of_trajectory_pairs()} number_lon_traj = {len(lon_trajectory1d_bundle)} number_lat_traj = {len(lat_trajectory1d_bundle)}")

        # Get instance of collision checker and constraint checker
        collision_checker = CollisionChecker(frame.obstacles, init_s[0],init_d[0], reference_line, reference_line_info, path_time_graph)

        # 7. always get the best pair of trajectories to combine; return the first
        # collision-free trajectory.

        constraint_failure_count: int = 0
        collision_failure_count: int = 0
        combined_constraint_failure_count: int = 0

        lon_vel_failure_count: int = 0
        lon_acc_failure_count: int = 0
        lon_jerk_failure_count: int = 0
        curvature_failure_count: int = 0
        lat_acc_failure_count: int = 0
        lat_jerk_failure_count: int = 0

        num_lattice_traj: int = 0

        while trajectory_evaluator.has_more_trajectory_pairs():
            trajectory_pair_cost: float = trajectory_evaluator.top_trajectory_pair_cost()
            trajectory_pair = trajectory_evaluator.next_top_trajectory_pair()

            # combine two 1d trajectories to one 2d trajectory
            combined_trajectory = TrajectoryCombiner.Combine(reference_line, trajectory_pair[0], trajectory_pair[1], planning_init_point.relative_time())

            # check longitudinal and lateral acceleration
            # considering trajectory curvatures
            result = ConstraintChecker.ValidTrajectory(combined_trajectory)
            if result != ConstraintChecker.Result.VALID:
                combined_constraint_failure_count += 1
                if result == ConstraintChecker.Result.LON_VELOCITY_OUT_OF_BOUND:
                    lon_vel_failure_count += 1
                elif result == ConstraintChecker.Result.LON_ACCELERATION_OUT_OF_BOUND:
                    lon_acc_failure_count += 1
                elif result == ConstraintChecker.Result.LON_JERK_OUT_OF_BOUND:
                    lon_jerk_failure_count += 1
                elif result == ConstraintChecker.Result.CURVATURE_OUT_OF_BOUND:
                    curvature_failure_count += 1
                elif result == ConstraintChecker.Result.LAT_ACCELERATION_OUT_OF_BOUND:
                    lat_acc_failure_count += 1
                elif result == ConstraintChecker.Result.LAT_JERK_OUT_OF_BOUND:
                    lat_jerk_failure_count += 1
                else:
                    # Intentional empty
                    pass
                continue
            
            # check collision with other obstacles
            if collision_checker.InCollision(combined_trajectory):
                collision_failure_count += 1
                continue

            # put combine trajectory into debug data
            combined_trajectory_points = combined_trajectory
            num_lattice_traj += 1
            reference_line_info.SetTrajectory(combined_trajectory)
            reference_line_info.SetCost(reference_line_info.PriorityCost() + trajectory_pair_cost)
            reference_line_info.SetDrivable(True)

            # Print rhe chosen end condition and start condition
            self.logger.debug(f"Starting Lon. State: s = {init_s[0]} ds = {init_s[1]}  dds = {init_s[2]}")
            # cast
            lattice_traj: LatticeTrajectory1d = trajectory_pair[0]
            if lattice_traj is None:
                self.logger.debug("Dynamically casting trajectory1d failed")

            if lattice_traj.has_target_position():
                self.logger.debug(f"Ending Lon. State: s = {lattice_traj.target_position} ds = {lattice_traj.target_velocity}  t = {lattice_traj.target_time}")

            self.logger.debug(f"InputPose XY: {planning_init_point.ShortDebugString()} S: ({init_s[0]}, {init_s[1]}, {init_s[2]}) L: ({init_d[0]}, {init_d[1]}, {init_d[2]})")

            self.logger.debug(f"Reference_line_priority_cost = {reference_line_info.PriorityCost()} Total_Trajectory_Cost = {trajectory_pair_cost}")
            self.logger.debug(f"OutputTrajectory: {[combined_trajectory_points[i].ShortDebugString for i in range(10)]}")

            break
        
        self.logger.debug(f"Trajectory_Evaluation_Time = {time.time()-current_time}")

        self.logger.debug(f"Step CombineTrajectory Succeeded")

        self.logger.debug(f"1d trajectory not valid for constraint [{constraint_failure_count}] times")
        self.logger.debug(f"Combined trajectory not valid for [{combined_constraint_failure_count}] times")
        self.logger.debug(f"Trajectory not valid for collision [{collision_failure_count}] times")
        self.logger.debug(f"Total_Lattice_Planning_Frame_Time = {time.time()-start_time}")

        if num_lattice_traj > 0:
            self.logger.debug(f"Planning succeeded")
            self.num_planning_succeeded_cycles += 1
            reference_line_info.SetDrivable(True)
            return True
        else:
            self.logger.error(f"Planning failed")
            if FLAGS_enable_backup_trajectory:
                self.logger.error(f"Use backup trajectory")
                BackupTrajectoryGenerator(init_s, init_d, planning_init_point.relative_time, collision_checker, trajectory1d_generator)
                trajectory: DiscretizedTrajectory = BackupTrajectoryGenerator.GenerateTrajectory(reference_line)

                reference_line_info.AddCost(FLAGS_backup_trajectory_cost)
                reference_line_info.SetTrajectory(trajectory)
                reference_line_info.SetDrivable(True)
                return True
            else:
                reference_line_info.SetCost(float("inf"))
            return False
