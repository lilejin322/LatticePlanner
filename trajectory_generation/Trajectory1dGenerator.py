from typing import List, Tuple
from common.curve1d.Curve1d import Curve1d
from common.curve1d.QuarticPolynomialCurve1d import QuarticPolynomialCurve1d
from common.curve1d.QuinticPolynomialCurve1d import QuinticPolynomialCurve1d
from behavior.PathTimeGraph import PathTimeGraph
from behavior.PredictionQuerier import PredictionQuerier
from common.PlanningTarget import PlanningTarget
from logging import Logger
from config import FLAGS_lateral_optimization, FLAGS_max_s_lateral_optimization, FLAGS_default_delta_s_lateral_optimization
from trajectory_generation.EndConditionSampler import EndConditionSampler
from trajectory_generation.LateralOSQPOptimizer import LateralOSQPOptimizer
from common.trajectory1d.PiecewiseJerkTrajectory1d import PiecewiseJerkTrajectory1d
from trajectory_generation.LatticeTrajectory1d import LatticeTrajectory1d

# A common function for trajectory bundles generation with 
# a given initial state and end conditions

class Trajectory1dGenerator:
    """
    Trajectory1dGenerator class  
    """

    def __init__(self, lon_init_state: List[float], lat_init_state: List[float], path_time_graph: PathTimeGraph, prediction_querier: PredictionQuerier):
        """
        Constructor

        :param List[float] lon_init_state: initial state for longitudinal motion
        :param List[float] lat_init_state: initial state for lateral motion
        :param PathTimeGraph path_time_graph: path time graph
        :param PredictionQuerier prediction_querier: prediction querier
        """

        self.init_lon_state = lon_init_state
        self.init_lat_state = lat_init_state
        self.end_condition_sampler: EndConditionSampler = EndConditionSampler(lon_init_state, lat_init_state, path_time_graph, prediction_querier)
        self.path_time_graph = path_time_graph
        self.logger = Logger("Trajectory1dGenerator")

    def GenerateTrajectoryBundles(self, planning_target: PlanningTarget, lon_trajectory_bundle: List[Curve1d], lat_trajectory_bundle: List[Curve1d]) -> None:
        """
        Generate trajectory bundles

        :param PlanningTarget planning_target: planning target
        :param List[Curve1d] lon_trajectory_bundle: longitudinal trajectory bundle
        :param List[Curve1d] lat_trajectory_bundle: lateral trajectory bundle
        """

        self.GenerateLongitudinalTrajectoryBundle(planning_target, lon_trajectory_bundle)
        self.GenerateLateralTrajectoryBundle(lat_trajectory_bundle)

    def GenerateSpeedProfilesForCruising(self, target_speed: float, lon_trajectory_bundle: List[Curve1d]) -> None:
        """
        Generate speed profiles for cruising

        :param float target_speed: target speed
        :param List[Curve1d] lon_trajectory_bundle: longitudinal trajectory bundle
        """

        self.logger.debug(f"cruise speed is {target_speed}")
        end_conditions: List[Tuple[List[float], float]] = self.end_condition_sampler.SampleLonEndConditionsForCruising(target_speed)
        if not end_conditions:
            return

        # For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
        # "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
        # invoke the common function to generate trajectory bundles.

        self.GenerateTrajectory1DBundle(self.init_lon_state, end_conditions, lon_trajectory_bundle, order=4)

    def GenerateSpeedProfilesForStopping(self, stop_point: float, lon_trajectory_bundle: List[Curve1d]) -> None:
        """
        Generate speed profiles for stopping

        :param float stop_point: stop point
        :param List[Curve1d] lon_trajectory_bundle: longitudinal trajectory bundle
        """

        self.logger.debug(f"stop point is {stop_point}")
        end_conditions: List[Tuple[List[float], float]] = self.end_condition_sampler.SampleLonEndConditionsForStopping(stop_point)
        if not end_conditions:
            return

        # Use the common function to generate trajectory bundles.
        self.GenerateTrajectory1DBundle(self.init_lon_state, end_conditions, lon_trajectory_bundle, order=5)

    def GenerateSpeedProfilesForPathTimeObstacles(self, lon_trajectory_bundle: List[Curve1d]) -> None:
        """
        Generate speed profiles for path-time obstacles

        :param List[Curve1d] lon_trajectory_bundle: longitudinal trajectory bundle
        """

        end_conditions: List[Tuple[List[float], float]] = self.end_condition_sampler.SampleLonEndConditionsForPathTimePoints()
        if not end_conditions:
            return
        
        # Use the common function to generate trajectory bundles.

        self.GenerateTrajectory1DBundle(self.init_lon_state, end_conditions, lon_trajectory_bundle, order=5)

    def GenerateLongitudinalTrajectoryBundle(self, planning_target: PlanningTarget, lon_trajectory_bundle: List[Curve1d]) -> None:
        """
        Generate longitudinal trajectory bundle

        :param PlanningTarget planning_target: planning target
        :param List[Curve1d] lon_trajectory_bundle: longitudinal trajectory bundle
        """

        # cruising trajectories are planned regardlessly.
        self.GenerateSpeedProfilesForCruising(planning_target.cruise_speed, lon_trajectory_bundle)
        self.GenerateSpeedProfilesForPathTimeObstacles(lon_trajectory_bundle)

        if planning_target.has_stop_point:
            self.GenerateSpeedProfilesForStopping(planning_target.stop_point.s, lon_trajectory_bundle)

    def GenerateLateralTrajectoryBundle(self, lat_trajectory_bundle: List[Curve1d]) -> None:
        """
        Generate lateral trajectory bundle

        :param List[Curve1d] lat_trajectory_bundle: lateral trajectory bundle
        """

        if not FLAGS_lateral_optimization:
            end_conditions: List[Tuple[List[float], float]] = self.end_condition_sampler.SampleLatEndConditions()

            # Use the common function to generate trajectory bundles.
            self.GenerateTrajectory1DBundle(self.init_lat_state, end_conditions, lat_trajectory_bundle, order=5)
        else:
            s_min: float = self.init_lon_state[0]
            s_max: float = s_min + FLAGS_max_s_lateral_optimization

            delta_s: float = FLAGS_default_delta_s_lateral_optimization

            lateral_boudns: List[Tuple[float, float]] = self.path_time_graph.GetLateralBounds(s_min, s_max, delta_s)

            # LateralTrajectoryOptimizer lateral_optimizer
            lateral_optimizer = LateralOSQPOptimizer()

            lateral_optimizer.Optimize(self.init_lat_state, delta_s, lateral_boudns)

            lateral_trajectory = lateral_optimizer.GetOptimalTrajectory()

            lat_trajectory_bundle.append(PiecewiseJerkTrajectory1d(lateral_trajectory))

    @staticmethod
    def GenerateTrajectory1DBundle(init_state: List[float], end_conditions: List[Tuple[List[float], float]], trajectory_bundle: List[Curve1d], order: int) -> None:
        """
        Generate trajectory bundle

        :param List[float] init_state: initial state
        :param List[Tuple[List[float], float]] end_conditions: end conditions
        :param List[Curve1d] trajectory_bundle: trajectory bundle
        """

        assert trajectory_bundle is not None, "Trajectory bundle cannot be None"

        if not end_conditions:
            return
        # In Python no need to reserve memory for List[]

        for end_condition in end_conditions:
            if order == 4:
                trajectory1d: LatticeTrajectory1d = LatticeTrajectory1d(QuarticPolynomialCurve1d(
                    init_state, [end_condition[0][1], end_condition[0][2]], end_condition[1]))
            elif order == 5:
                trajectory1d: LatticeTrajectory1d = LatticeTrajectory1d(QuinticPolynomialCurve1d(
                    init_state, end_condition[0], end_condition[1]))
            else:
                raise ValueError(f"Unsupported order: {order}")
        
            trajectory1d.set_target_velocity(end_condition[0][1])
            trajectory1d.set_target_time(end_condition[1])
            if order == 5:
                trajectory1d.set_target_position(end_condition[0][0])
            trajectory_bundle.append(trajectory1d)
