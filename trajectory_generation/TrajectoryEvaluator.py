from typing import List, Tuple
import heapq
import math
from common.curve1d.Curve1d import Curve1d
from common.PlanningTarget import PlanningTarget
from common.PathPoint import PathPoint
from behavior.PathTimeGraph import PathTimeGraph
from config import FLAGS_trajectory_time_length, FLAGS_trajectory_time_resolution, FLAGS_lattice_stop_buffer, \
                   FLAGS_speed_lon_decision_horizon, FLAGS_trajectory_space_resolution, FLAGS_weight_lon_objective, \
                   FLAGS_weight_lon_jerk, FLAGS_weight_lon_collision, FLAGS_weight_centripetal_acceleration, \
                   FLAGS_weight_lat_offset, FLAGS_weight_lat_comfort, FLAGS_lat_offset_bound, FLAGS_weight_opposite_side_offset, \
                   FLAGS_weight_same_side_offset, FLAGS_numerical_epsilon, FLAGS_longitudinal_jerk_upper_bound, FLAGS_weight_target_speed, \
                   FLAGS_weight_dist_travelled, FLAGS_lon_collision_cost_std, FLAGS_lon_collision_yield_buffer, FLAGS_lon_collision_overtake_buffer, \
                   FLAGS_comfort_acceleration_factor, FLAGS_longitudinal_acceleration_upper_bound, FLAGS_longitudinal_acceleration_lower_bound
from common import ConstraintChecker1d
from logging import Logger
import numpy as np
from PathMatcher import PathMatcher
from common.trajectory1d.PiecewiseAccelerationTrajectory1d import PiecewiseAccelerationTrajectory1d
from trajectory_generation import PiecewiseBrakingTrajectoryGenerator
from common.SpeedPoint import SpeedPoint

class TrajectoryEvaluator:
    """
    TrajectoryEvaluator class
    """

    def __init__(self, 
                 init_s: List[float],
                 planning_target: PlanningTarget,
                 lon_trajectories: List[Curve1d],
                 lat_trajectories: List[Curve1d],
                 path_time_graph: PathTimeGraph,
                 reference_line: List[PathPoint]):
        """
        Constructor

        :param List[float] init_s: initial s values
        :param PlanningTarget planning_target: planning target
        :param List[Curve1d] lon_trajectories: longitudinal trajectories
        :param List[Curve1d] lat_trajectories: lateral trajectories
        :param PathTimeGraph path_time_graph: path time graph
        :param List[PathPoint] reference_line: reference line
        """

        self.logger = Logger("TrajectoryEvaluator")
        self.init_s = init_s
        self.lon_trajectories = lon_trajectories
        self.lat_trajectories = lat_trajectories
        self.path_time_graph = path_time_graph
        self.reference_line = reference_line
        # Note that cost_queue is List[weight, Tuple[lon_trajectory, lat_trajectory]]
        self.cost_queue: List[Tuple[float, Tuple[Curve1d, Curve1d]]] = []
        start_time = 0.0
        end_time = FLAGS_trajectory_time_length
        self.path_time_intervals = self.path_time_graph.GetPathBlockingIntervals(
            start_time, end_time, FLAGS_trajectory_time_resolution)

        self.reference_s_dot = self.ComputeLongitudinalGuideVelocity(planning_target)

        # if we have a stop point along the reference line,
        # filter out the lon. trajectories that pass the stop point.
        stop_point = float('inf')
        if planning_target.has_stop_point:
            stop_point = planning_target.stop_point.s

        for lon_trajectory in lon_trajectories:
            lon_end_s: float = lon_trajectory.Evaluate(0, end_time)
            if init_s[0] < stop_point and lon_end_s + FLAGS_lattice_stop_buffer > stop_point:
                continue

            if not ConstraintChecker1d.IsValidLongitudinalTrajectory(lon_trajectory):
                continue

            for lat_trajectory in lat_trajectories:
                cost: float = self.Evaluate(planning_target, lon_trajectory, lat_trajectory)
                # in C++ the priority queue is a max heap, but the source code has overriden the operator to be a min heap
                # struct CostComparator
                #   : public std::binary_function<const PairCost&, const PairCost&, bool> {
                #  bool operator()(const PairCost& left, const PairCost& right) const {
                #   return left.second > right.second;
                #  }
                # };
                # we need to be very cautious about these things
                heapq.heappush(self.cost_queue, (cost, (lon_trajectory, lat_trajectory)))
        
        self.logger.debug(f"Number of valid 1d trajectory pairs: {len(self.cost_queue)}")
    
    def has_more_trajectory_pairs(self) -> bool:
        """
        Check if there are more trajectory pairs to evaluate

        :returns: True if there are more trajectory pairs to evaluate, False otherwise
        :rtype: bool
        """

        return len(self.cost_queue) > 0
    
    def num_of_trajectory_pairs(self) -> int:
        """
        Get the number of trajectory pairs to evaluate

        :returns: the number of trajectory pairs to evaluate
        :rtype: int
        """

        return len(self.cost_queue)
    
    def next_top_trajectory_pair(self) -> Tuple[Curve1d, Curve1d]:
        """
        Get the next top trajectory pair

        :returns: the next top trajectory pair
        :rtype: Tuple[Curve1d, Curve1d]
        """

        assert self.has_more_trajectory_pairs(), "No more trajectory pairs to evaluate"

        top = heapq.heappop(self.cost_queue)[1]

        return top
    
    def top_trajectory_pair_cost(self) -> float:
        """
        Get the cost of the top trajectory pair

        :returns: the cost of the top trajectory pair
        :rtype: float
        """

        cost = self.cost_queue[0][0]
        # in C++ the priority queue is a max heap, so we need to negate the cost
        return -cost
    
    def Evaluate(self, planning_target: PlanningTarget, lon_trajectory: Curve1d, lat_trajectory: Curve1d,
                cost_components = List[float]) -> float:
        """
        Evaluate the trajectory pair

        :param PlanningTarget planning_target: the planning target
        :param Curve1d lon_trajectory: the longitudinal trajectory
        :param Curve1d lat_trajectory: the lateral trajectory
        :param List[float] cost_components: the cost components
        :returns: the cost of the trajectory pair
        :rtype: float
        """

        # Costs:
        # 1. Cost of missing the objective, e.g., cruise, stop, etc.
        # 2. Cost of longitudinal jerk
        # 3. Cost of longitudinal collision
        # 4. Cost of lateral offset
        # 5. Cost of lateral comfort

        # Longitudinal costs
        lon_objective_cost: float = self.LonObjectiveCost(lon_trajectory, planning_target, self.reference_s_dot)

        lon_jerk_cost: float = self.LonComfortCost(lon_trajectory)

        lon_collision_cost: float = self.LonCollisionCost(lon_trajectory)

        centripetal_acc_cost: float = self.CentripetalAccelerationCost(lon_trajectory)

        # decides the longitudinal evaluation horizon for lateral trajectories
        evaluation_horizon: float = min(FLAGS_speed_lon_decision_horizon,
                                        lon_trajectory.Evaluate(0, lon_trajectory.ParamLength))
        s_values: List[float] = [s for s in np.arange(0.0, evaluation_horizon, FLAGS_trajectory_space_resolution)]

        # Lateral costs
        lat_offset_cost: float = self.LatOffSetCost(lat_trajectory, s_values)

        lat_comfort_cost: float = self.LatComfortCost(lon_trajectory, lat_trajectory)

        if cost_components is not None:
            cost_components.append(lon_objective_cost)
            cost_components.append(lon_jerk_cost)
            cost_components.append(lon_collision_cost)
            cost_components.append(lat_offset_cost)
        
        total_cost = (lon_objective_cost * FLAGS_weight_lon_objective +
                      lon_jerk_cost * FLAGS_weight_lon_jerk +
                      lon_collision_cost * FLAGS_weight_lon_collision +
                      centripetal_acc_cost * FLAGS_weight_centripetal_acceleration +
                      lat_offset_cost * FLAGS_weight_lat_offset +
                      lat_comfort_cost * FLAGS_weight_lat_comfort)
        return total_cost

    @staticmethod
    def LatOffSetCost(lat_trajectory: Curve1d, s_values: List[float]) -> float:
        """
        Compute the lateral offset cost

        :param Curve1d lat_trajectory: the lateral trajectory
        :param List[float] s_values: the s values
        :returns: the lateral offset cost
        :rtype: float
        """

        lat_offset_start: float = lat_trajectory.Evaluate(0, 0.0)
        cost_sqr_sum: float = 0.0
        cost_abs_sum: float = 0.0
        for s in s_values:
            lat_offset: float = lat_trajectory.Evaluate(0, s)
            cost: float = lat_offset / FLAGS_lat_offset_bound
            if lat_offset * lat_offset_start < 0.0:
                cost_sqr_sum += cost * cost * FLAGS_weight_opposite_side_offset
                cost_abs_sum += abs(cost) * FLAGS_weight_opposite_side_offset
            else:
                cost_sqr_sum += cost * cost * FLAGS_weight_same_side_offset
                cost_abs_sum += abs(cost) * FLAGS_weight_same_side_offset
        return cost_sqr_sum / (cost_abs_sum + FLAGS_numerical_epsilon)

    def LatComfortCost(self, lon_trajectory: Curve1d, lat_trajectory: Curve1d) -> float:
        """
        Compute the lateral comfort cost

        :param Curve1d lon_trajectory: the longitudinal trajectory
        :param Curve1d lat_trajectory: the lateral trajectory
        :returns: the lateral comfort cost
        :rtype: float
        """

        max_cost: float = 0.0
        t: float = 0.0
        while t < FLAGS_trajectory_time_length:
            s: float = lon_trajectory.Evaluate(0, t)
            s_dot: float = lon_trajectory.Evaluate(1, t)
            s_dotdot: float = lon_trajectory.Evaluate(2, t)

            relative_s: float = s - self.init_s[0]
            l_prime: float = lat_trajectory.Evaluate(1, relative_s)
            l_primeprime: float = lat_trajectory.Evaluate(2, relative_s)
            cost: float = l_primeprime * s_dot * s_dot + l_prime * s_dotdot
            max_cost = max(max_cost, abs(cost))
            t += FLAGS_trajectory_time_resolution
        return max_cost

    def LonComfortCost(self, lon_trajectory: Curve1d) -> float:
        """
        Compute the longitudinal comfort cost

        :param Curve1d lon_trajectory: the longitudinal trajectory
        :returns: the longitudinal comfort cost
        :rtype: float
        """

        cost_sqr_sum: float = 0.0
        cost_abs_sum: float = 0.0
        t: float = 0.0
        while t < FLAGS_trajectory_time_length:
            jerk: float = lon_trajectory.Evaluate(3, t)
            cost: float = jerk / FLAGS_longitudinal_jerk_upper_bound
            cost_sqr_sum += cost * cost
            cost_abs_sum += abs(cost)
            t += FLAGS_trajectory_time_resolution
        return cost_sqr_sum / (cost_abs_sum + FLAGS_numerical_epsilon)

    @staticmethod
    def LonObjectiveCost(lon_trajectory: Curve1d, planning_target: PlanningTarget, ref_s_dots: List[float]) -> float:
        """
        Compute the longitudinal objective cost

        :param Curve1d lon_trajectory: the longitudinal trajectory
        :param PlanningTarget planning_target: the planning target
        :param List[float] ref_s_dots: the reference s dots
        :returns: the longitudinal objective cost
        :rtype: float
        """

        t_max: float = lon_trajectory.ParamLength
        dist_s: float = lon_trajectory.Evaluate(0, t_max) - lon_trajectory.Evaluate(0, 0.0)

        speed_cost_sqr_sum: float = 0.0
        speed_cost_weight_sum: float = 0.0
        for i in range(len(ref_s_dots)):
            t: float = float(i) * FLAGS_trajectory_time_resolution
            cost: float = ref_s_dots[i] - lon_trajectory.Evaluate(1, t)
            speed_cost_sqr_sum += t * t * abs(cost)
            speed_cost_weight_sum += t * t
        
        speed_cost: float = speed_cost_sqr_sum / (speed_cost_weight_sum + FLAGS_numerical_epsilon)
        dist_travelled_cost: float = 1.0 / (1.0 + dist_s)
        return (speed_cost * FLAGS_weight_target_speed + dist_travelled_cost * FLAGS_weight_dist_travelled) / (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled)

    # TODO(all): consider putting pointer of reference_line_info and frame
    # while constructing trajectory evaluator
    def LonCollisionCost(self, lon_trajectory: Curve1d) -> float:
        """
        Compute the longitudinal collision cost

        :param Curve1d lon_trajectory: the longitudinal trajectory
        :returns: the longitudinal collision cost
        :rtype: float
        """

        cost_sqr_sum: float = 0.0
        cost_abs_sum: float = 0.0
        for i in range(len(self.path_time_intervals)):
            pt_interval: List[Tuple[float, float]] = self.path_time_intervals[i]
            if not pt_interval:
                continue
            t: float = float(i) * FLAGS_trajectory_time_resolution
            traj_s: float = lon_trajectory.Evaluate(0, t)
            sigma: float = FLAGS_lon_collision_cost_std
            for m in pt_interval:
                dist: float = 0.0
                if traj_s < m[0] - FLAGS_lon_collision_yield_buffer:
                    dist = m[0] - FLAGS_lon_collision_yield_buffer - traj_s
                elif traj_s > m[1] + FLAGS_lon_collision_overtake_buffer:
                    dist = traj_s - m[1] - FLAGS_lon_collision_overtake_buffer
                cost: float = math.exp(-dist * dist / (2.0 * sigma * sigma))
                
                cost_sqr_sum += cost * cost
                cost_abs_sum += cost

        return cost_sqr_sum / (cost_abs_sum + FLAGS_numerical_epsilon)

    def CentripetalAccelerationCost(self, lon_trajectory: Curve1d) -> float:
        """
        Compute the centripetal acceleration cost

        :param Curve1d lon_trajectory: the longitudinal trajectory
        :returns: the centripetal acceleration cost
        :rtype: float
        """

        centripetal_acc_sum: float = 0.0
        centripetal_acc_sqr_sum: float = 0.0
        t: float = 0.0
        while t < FLAGS_trajectory_time_length:
            s: float = lon_trajectory.Evaluate(0, t)
            v: float = lon_trajectory.Evaluate(1, t)
            ref_point: PathPoint = PathMatcher.MatchToPathS(self.reference_line, s)
            assert ref_point.has_kappa, "Reference point does not have kappa"
            centripetal_acc: float = v * v * ref_point.kappa
            centripetal_acc_sum += abs(centripetal_acc)
            centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc
            t += FLAGS_trajectory_time_resolution
        return centripetal_acc_sqr_sum / (centripetal_acc_sum + FLAGS_numerical_epsilon)

    def ComputeLongitudinalGuideVelocity(self, planning_target: PlanningTarget) -> List[float]:
        """
        Compute the longitudinal guide velocity

        :param PlanningTarget planning_target: the planning target
        :returns: the longitudinal guide velocity
        :rtype: List[float]
        """

        reference_s_dot: List[float] = []

        cruise_v: float = planning_target.cruise_speed

        if not planning_target.has_stop_point:
            lon_traj: PiecewiseAccelerationTrajectory1d = PiecewiseAccelerationTrajectory1d(self.init_s[0], cruise_v)
            lon_traj.AppendSegment(0.0, FLAGS_trajectory_time_length + FLAGS_numerical_epsilon)

            t: float = 0.0
            while t < FLAGS_trajectory_time_length:
                reference_s_dot.append(lon_traj.Evaluate(1, t))
                t += FLAGS_trajectory_time_resolution
        else:
            dist_s: float = planning_target.stop_point.s - self.init_s[0]
            if dist_s < FLAGS_numerical_epsilon:
                lon_traj: PiecewiseAccelerationTrajectory1d = PiecewiseAccelerationTrajectory1d(self.init_s[0], 0.0)
                lon_traj.AppendSegment(0.0, FLAGS_trajectory_time_length + FLAGS_numerical_epsilon)

                t: float = 0.0
                while t < FLAGS_trajectory_time_length:
                    reference_s_dot.append(lon_traj.Evaluate(1, t))
                    t += FLAGS_trajectory_time_resolution
                return reference_s_dot
            
            a_comfort: float = FLAGS_longitudinal_acceleration_upper_bound * FLAGS_comfort_acceleration_factor
            d_comfort: float = -FLAGS_longitudinal_acceleration_lower_bound * FLAGS_comfort_acceleration_factor

            lon_ref_trajectory: Curve1d = PiecewiseBrakingTrajectoryGenerator.Generate(
                                               planning_target.stop_point.s, self.init_s[0],
                                               planning_target.cruise_speed, self.init_s[1], a_comfort, d_comfort,
                                               FLAGS_trajectory_time_length + FLAGS_numerical_epsilon)
    
            t: float = 0.0
            while t < FLAGS_trajectory_time_length:
                reference_s_dot.append(lon_ref_trajectory.Evaluate(1, t))
                t += FLAGS_trajectory_time_resolution
        
        return reference_s_dot
    
    def InterpolateDenseStPoints(self, st_points: List[SpeedPoint], t: float) -> Tuple[bool, float]:
        """
        Interpolate dense st points

        :param List[SpeedPoint] st_points: the speed points
        :param float t: the time
        :returns: a tuple of state and traj_s
        :rtype: Tuple[bool, float]
        """

        assert len(st_points) > 1, "st_points size must be greater than 1"
        if t < st_points[0].t or t > st_points[-1].t:
            self.logger.error("AutoTuning InterpolateDenseStPoints Error")
            return False, None
        for i in range(1, len(st_points)):
            if t <= st_points[i].t:
                return True, st_points[i].t
        return False, None
