"""
Collision checker submodule
"""
from typing import List
import config as config_module
from math import fabs
from logging import Logger
from common.discretized_trajectory import DiscretizedTrajectory
from behavior.path_time_graph import PathTimeGraph
from protoclass.path_point import PathPoint
from path_matcher import PathMatcher
from lattice_motion import ego_occupancy, rectangles_overlap

class CollisionChecker:
    """
    CollisionChecker class
    """

    def __init__(self, obstacles, ego_vehicle_s: float, ego_vehicle_d: float,
                 discretized_reference_line: List[PathPoint], lane_width,
                 path_time_graph: PathTimeGraph) -> None:
        """
        ``lane_width(s)`` returns ``(ok, left, right)`` from the lanelet.
        """
        self.lane_width = lane_width
        self.path_time_graph = path_time_graph
        self.predicted_bounding_rectangles = []
        self.logger = Logger("CollisionChecker")
        self.BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d, discretized_reference_line)

    @staticmethod
    def StaticInCollision(obstacles, ego_trajectory: DiscretizedTrajectory, ego_length: float,
                          ego_width: float, ego_back_edge_to_center: float) -> bool:
        for i in range(ego_trajectory.NumOfPoints()):
            ego_point = ego_trajectory.TrajectoryPointAt(i)
            relative_time = ego_point.relative_time or 0.0
            ego_theta = ego_point.path_point.theta or 0.0
            ego_box = ego_occupancy(
                ego_point.path_point.x,
                ego_point.path_point.y,
                ego_theta,
                ego_length,
                ego_width,
                ego_back_edge_to_center,
            )
            for obstacle in obstacles:
                other = obstacle.occupancy_at(relative_time)
                if rectangles_overlap(ego_box, other):
                    return True
        return False

    def InCollision(self, discretized_trajectory: DiscretizedTrajectory) -> bool:
        """
        Check if the ego vehicle is in collision with the predicted bounding rectangles.

        :param DiscretizedTrajectory discretized_trajectory: Ego vehicle trajectory
        :returns: True if the ego vehicle is in collision with the predicted bounding rectangles
        :rtype: bool
        """
        if discretized_trajectory.NumOfPoints() > len(self.predicted_bounding_rectangles):
            raise ValueError("Number of trajectory points exceeds the number of predicted bounding rectangles.")

        ego_length = config_module.EGO_VEHICLE_LENGTH
        ego_width = config_module.EGO_VEHICLE_WIDTH

        for i in range(discretized_trajectory.NumOfPoints()):

            trajectory_point = discretized_trajectory.TrajectoryPointAt(i)
            ego_theta = trajectory_point.path_point.theta

            ego_box = ego_occupancy(
                trajectory_point.path_point.x,
                trajectory_point.path_point.y,
                ego_theta or 0.0,
                ego_length,
                ego_width,
                config_module.EGO_BACK_EDGE_TO_CENTER,
            )
            for obstacle_box in self.predicted_bounding_rectangles[i]:
                if rectangles_overlap(ego_box, obstacle_box):
                    return True
        return False

    def BuildPredictedEnvironment(self, obstacles, ego_vehicle_s: float, ego_vehicle_d: float,
                                  discretized_reference_line: List[PathPoint]) -> None:
        """
        Build predicted environment.

        :param List[Obstacle] obstacles: List of obstacles
        :param float ego_vehicle_s: Ego vehicle s-coordinate
        :param float ego_vehicle_d: Ego vehicle d-coordinate
        :param List[PathPoint] discretized_reference_line: Discretized reference line
        """
        assert not self.predicted_bounding_rectangles, "Predicted bounding rectangles should be empty before building the environment."

        # If the ego vehicle is in lane,
        # then, ignore all obstacles from the same lane.
        ego_vehicle_in_lane: bool = self.IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d)
        obstacles_considered = []

        for obstacle in obstacles:
            if ego_vehicle_in_lane and (
                self.IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s, discretized_reference_line)
                or not self.path_time_graph.IsObstacleInGraph(obstacle.id)
            ):
                continue
            obstacles_considered.append(obstacle)

        relative_time: float = 0.0
        while relative_time < config_module.FLAGS_trajectory_time_length:
            predicted_env = []
            for obstacle in obstacles_considered:
                predicted_env.append(
                    obstacle.occupancy_at(
                        relative_time,
                        length_extra=2.0 * config_module.FLAGS_lon_collision_buffer,
                        width_extra=2.0 * config_module.FLAGS_lat_collision_buffer,
                    )
                )

            self.predicted_bounding_rectangles.append(predicted_env)
            relative_time += config_module.FLAGS_trajectory_time_resolution

    def IsEgoVehicleInLane(self, ego_vehicle_s: float, ego_vehicle_d: float) -> bool:
        """
        Check if the ego vehicle is in lane.

        :param float ego_vehicle_s: Ego vehicle s-coordinate
        :param float ego_vehicle_d: Ego vehicle d-coordinate
        :returns: True if the ego vehicle is in lane, False otherwise
        :rtype: bool
        """
        left_width: float = config_module.FLAGS_default_reference_line_width * 0.5
        right_width: float = config_module.FLAGS_default_reference_line_width * 0.5
        ok, lane_left_width, lane_right_width = self.lane_width(ego_vehicle_s)
        if ok:
            left_width = lane_left_width
            right_width = lane_right_width
        return ego_vehicle_d < left_width and ego_vehicle_d > -right_width

    def IsObstacleBehindEgoVehicle(self, obstacle, ego_vehicle_s: float,
                                   discretized_reference_line: List[PathPoint]) -> bool:
        half_lane_width = config_module.FLAGS_default_reference_line_width * 0.5
        pose = obstacle.pose_at(0.0)
        obstacle_reference_line_position = PathMatcher.GetPathFrenetCoordinate(
            discretized_reference_line, pose.x, pose.y)

        if (obstacle_reference_line_position[0] < ego_vehicle_s and
                fabs(obstacle_reference_line_position[1]) < half_lane_width):
            self.logger.warning(f"Ignore obstacle [{obstacle.id}]")
            return True
        return False
