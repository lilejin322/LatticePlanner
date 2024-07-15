from common.STPoint import STPoint
from typing import List, Tuple
from behavior.PathTimeGraph import PathTimeGraph
from behavior.PredictionQuerier import PredictionQuerier
from behavior.FeasibleRegion import FeasibleRegion
from config import FLAGS_num_velocity_sample, FLAGS_polynomial_minimal_param, FLAGS_trajectory_time_length, \
                   FLAGS_min_velocity_sample_gap, FLAGS_numerical_epsilon, FLAGS_time_min_density, FLAGS_default_lon_buffer, \
                   FLAGS_num_sample_follow_per_timestamp, FRONT_EDGE_TO_CENTER

class SamplePoint:
    """
    SamplePoint class
    """

    def __init__(self, path_time_point: STPoint, ref_v: float):
        """
        Constructor
        """
        self.path_time_point = path_time_point
        self.ref_v = ref_v

State = List[float]
Condition = Tuple[State, float]

class EndConditionSampler:
    """
    EndConditionSampler class
    Input: planning objective, vehicle kinematic/dynamic constraints,
    Output: sampled ending 1 dimensional states with corresponding time duration.
    """

    def __init__(self, init_s: List[float], init_d: List[float], path_time_graph: PathTimeGraph,
                 prediction_querier: PredictionQuerier):
        """
        Constructor
        """
        self.init_s = init_s
        self.init_d = init_d
        self.path_time_graph = path_time_graph
        self.prediction_querier = prediction_querier
        self.feasible_region = FeasibleRegion(init_s)

    @staticmethod
    def SampleLatEndConditions() -> List[Tuple[List[float], float]]:
        """
        Sample lateral end conditions

        :returns: sampled lateral end conditions
        :rtype: List[Tuple[List[float], float]]
        """
        
        end_d_conditions: List[Tuple[List[float], float]] = []
        end_d_candidates = [0.0, -0.5, 0.5]
        end_s_candidates = [10.0, 20.0, 40.0, 80.0]

        for s in end_s_candidates:
            for d in end_d_candidates:
                end_d_state = [d, 0.0, 0.0]
                end_d_conditions.append((end_d_state, s))
        
        return end_d_conditions
    
    def SampleLonEndConditionsForCruising(self, ref_cruise_speed: float) -> List[Tuple[List[float], float]]:
        """
        Sample longitudinal end conditions for cruising

        :param float ref_cruise_speed: reference cruising speed
        :returns: sampled longitudinal end conditions for cruising
        :rtype: List[Tuple[List[float], float]]
        """
        
        assert FLAGS_num_velocity_sample > 1, "Number of velocity samples must be greater than 1"

        # time interval is one second plus the last one 0.01
        num_of_time_samples: int = 9
        time_samples: List[float] = [float('inf')] * num_of_time_samples
        for i in range(1, num_of_time_samples):
            ratio: float = float(i) / float(num_of_time_samples - 1)
            time_samples[i] = FLAGS_trajectory_time_length * ratio
        time_samples[0] = FLAGS_polynomial_minimal_param

        end_s_conditions: List[Tuple[List[float], float]] = []
        for time in time_samples:
            v_upper: float = min(self.feasible_region.VUpper(time), ref_cruise_speed)
            v_lower: float = self.feasible_region.VLower(time)

            lower_end_s: List[float] = [0.0, v_lower, 0.0]
            end_s_conditions.append((lower_end_s, time))

            upper_end_s: List[float] = [0.0, v_upper, 0.0]
            end_s_conditions.append((upper_end_s, time))

            v_range: float = v_upper - v_lower
            # number of sample velocities
            num_of_mid_points: int = min(FLAGS_num_velocity_sample - 2, int(v_range / FLAGS_min_velocity_sample_gap))

            if num_of_mid_points > 0:
                velocity_seg: float = v_range / float(num_of_mid_points + 1)
                for i in range(1, num_of_mid_points + 1):
                    end_s: List[float] = [0.0, v_lower + velocity_seg * float(i), 0.0]
                    end_s_conditions.append((end_s, time))
        
        return end_s_conditions

    def SampleLonEndConditionsForStopping(self, ref_stop_point: float) -> List[Tuple[List[float], float]]:
        """
        Sample longitudinal end conditions for stopping

        :param float ref_stop_point: reference stopping point
        :returns: sampled longitudinal end conditions for stopping
        :rtype: List[Tuple[List[float], float]]
        """

        # time interval is one second plus the last one 0.01
        num_of_time_samples: int = 9
        time_samples: List[float] = [float('inf')] * num_of_time_samples
        for i in range(1, num_of_time_samples):
            ratio: float = float(i) / float(num_of_time_samples - 1)
            time_samples[i] = FLAGS_trajectory_time_length * ratio
        time_samples[0] = FLAGS_polynomial_minimal_param

        end_s_conditions: List[Tuple[List[float], float]] = []
        for time in time_samples:
            end_s: List[float] = [max(self.init_s[0], ref_stop_point), 0.0, 0.0]
            end_s_conditions.append((end_s, time))
        return end_s_conditions

    def SampleLonEndConditionsForPathTimePoints(self) -> List[Tuple[List[float], float]]:
        """
        Sample longitudinal end conditions for path time points

        :returns: sampled longitudinal end conditions for path time points
        :rtype: List[Tuple[List[float], float]]
        """
        
        end_s_conditions: List[Tuple[List[float], float]] = []

        sample_points = self.QueryPathTimeObstacleSamplePoints()
        for sample_point in sample_points:
            if sample_point.path_time_point.t < FLAGS_polynomial_minimal_param:
                continue
            s: float = sample_point.path_time_point.s
            v: float = sample_point.ref_v
            t: float = sample_point.path_time_point.t
            if s > self.feasible_region.SUpper(t) or s < self.feasible_region.SLower(t):
                continue
            end_state: List[float] = [s, v, 0.0]
            end_s_conditions.append((end_state, t))
        return end_s_conditions

    def QueryPathTimeObstacleSamplePoints(self) -> List[SamplePoint]:
        """
        Query path time obstacle sample points

        :returns: path time obstacle sample points
        :rtype: List[SamplePoint]
        """

        sample_points: List[SamplePoint] = []
        for path_time_obstacle in self.path_time_graph.GetPathTimeObstacles():
            obstacle_id: str = path_time_obstacle.id
            self.QueryFollowPathTimePoints(obstacle_id, sample_points)
            self.QueryOvertakePathTimePoints(obstacle_id, sample_points)
        return sample_points

    def QueryFollowPathTimePoints(self, obstacle_id: str, sample_points: List[SamplePoint]) -> None:
        """
        Query follow path time points

        :param str obstacle_id: obstacle id
        :param List[SamplePoint] sample_points: sample points
        """

        follow_path_time_points: List[STPoint] = self.path_time_graph.GetObstacleSurroundingPoints(obstacle_id,
                                                                                    -FLAGS_numerical_epsilon,
                                                                                    FLAGS_time_min_density)
        
        for path_time_point in follow_path_time_points:
            v: float = self.prediction_querier.ProjectVelocityAlongReferenceLine(obstacle_id, path_time_point.s, path_time_point.t)
            # Generate candidate s
            s_upper: float = path_time_point.s - FRONT_EDGE_TO_CENTER
            s_lower: float = s_upper - FLAGS_default_lon_buffer
            assert FLAGS_num_sample_follow_per_timestamp >= 2, "Number of sample follow per timestamp must be greater than 2"
            s_gap: float = FLAGS_default_lon_buffer / float(FLAGS_num_sample_follow_per_timestamp - 1)
            for i in range(FLAGS_num_sample_follow_per_timestamp):
                s: float = s_lower + s_gap * i
                sample_point: SamplePoint = SamplePoint(path_time_point, v)
                sample_point.path_time_point.s = s
                sample_points.append(sample_point)

    def QueryOvertakePathTimePoints(self, obstacle_id: str, sample_points: List[SamplePoint]) -> None: 
        """
        Query overtake path time points

        :param str obstacle_id: obstacle id
        :param List[SamplePoint] sample_points: sample points
        """
    
        over_take_path_time_points: List[STPoint] = self.path_time_graph.GetObstacleSurroundingPoints(obstacle_id,
                                                                                                      FLAGS_numerical_epsilon,
                                                                                                      FLAGS_time_min_density)

        for path_time_point in over_take_path_time_points:
            v: float = self.prediction_querier.ProjectVelocityAlongReferenceLine(obstacle_id, path_time_point.s, path_time_point.t)
            sample_point: SamplePoint = SamplePoint(path_time_point, v)
            sample_point.path_time_point.s = path_time_point.s + FLAGS_default_lon_buffer
            sample_points.append(sample_point)
