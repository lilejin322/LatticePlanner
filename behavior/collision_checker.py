"""
Collision checker submodule
"""
from typing import List
from math import cos, sin, fabs, atan2, pi
from common.box2d import Box2d
from common.vec2d import Vec2d
from common.obstacle import Obstacle
from reference_line.reference_line_info import ReferenceLineInfo
from common.discretized_trajectory import DiscretizedTrajectory
from behavior.path_time_graph import PathTimeGraph
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.path_point import PathPoint
from logging import Logger
from path_matcher import PathMatcher
import config as config_module

class CollisionChecker:
    """
    CollisionChecker class
    """

    def __init__(self, obstacles: List[Obstacle], ego_vehicle_s: float, ego_vehicle_d: float,
                 discretized_reference_line: List[PathPoint], reference_line_info: ReferenceLineInfo,
                 path_time_graph: PathTimeGraph):
        """
        Constructor

        :param List[Obstacle] obstacles: List of obstacles
        :param float ego_vehicle_s: Ego vehicle s-coordinate
        :param float ego_vehicle_d: Ego vehicle d-coordinate
        :param List[PathPoint] discretized_reference_line: Discretized reference line
        :param ReferenceLineInfo reference_line_info: Reference line information
        :param PathTimeGraph path_time_graph: Path time graph
        """

        self.reference_line_info = reference_line_info
        self.path_time_graph = path_time_graph
        self.predicted_bounding_rectangles = []
        self.logger = Logger("CollisionChecker")
        self.BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d, discretized_reference_line)

    @staticmethod
    def StaticInCollision(obstacles: List[Obstacle], ego_trajectory: DiscretizedTrajectory, ego_length: float,
                          ego_width: float, ego_back_edge_to_center: float) -> bool:
        """
        Check if the ego vehicle is in collision with the obstacles.

        :param List[Obstacle] obstacles: List of obstacles
        :param DiscretizedTrajectory ego_trajectory: Ego vehicle trajectory
        :param float ego_length: Ego vehicle length
        :param float ego_width: Ego vehicle width
        :param float ego_back_edge_to_center: Ego vehicle back edge to center distance
        :returns: True if the ego vehicle is in collision with the obstacles, False otherwise
        :rtype: bool
        """

        for i in range(ego_trajectory.NumOfPoints()):
            ego_point = ego_trajectory.TrajectoryPointAt(i)
            relative_time = ego_point.relative_time
            ego_theta = ego_point.path_point.theta
            
            ego_box = Box2d(Vec2d(ego_point.path_point.x, ego_point.path_point.y), ego_theta, ego_length, ego_width)

            # correct the inconsistency of reference point and center point
            # TODO(all): move the logic before constructing the ego_box
            shift_distance = ego_length / 2.0 - ego_back_edge_to_center
            ego_box.Shift(Vec2d(shift_distance * cos(ego_theta), shift_distance * sin(ego_theta)))

            for obstacle in obstacles:
                obstacle_point = obstacle.GetPointAtTime(relative_time)
                obstacle_box = obstacle.GetBoundingBox(obstacle_point)

                # Check whether overlap exists
                if ego_box.HasOverlap(obstacle_box):
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

            ego_box = Box2d(Vec2d(trajectory_point.path_point.x, trajectory_point.path_point.y), ego_theta, ego_length, ego_width)

            shift_distance = ego_length / 2.0 - config_module.EGO_BACK_EDGE_TO_CENTER
            ego_box.Shift(Vec2d(shift_distance * cos(ego_theta), shift_distance * sin(ego_theta)))

            for obstacle_box in self.predicted_bounding_rectangles[i]:
                if ego_box.HasOverlap(obstacle_box):
                    return True
        return False

    def BuildPredictedEnvironment(self, obstacles: List[Obstacle], ego_vehicle_s: float, ego_vehicle_d: float,
                                  discretized_reference_line: List[PathPoint]):
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
            if obstacle.IsVirtual():
                continue
            if ego_vehicle_in_lane and (self.IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s, discretized_reference_line)
                                        or not self.path_time_graph.IsObstacleInGraph(obstacle.Id())):
                continue
            obstacles_considered.append(obstacle)

        relative_time: float = 0.0
        while relative_time < config_module.FLAGS_trajectory_time_length:
            predicted_env = []
            for obstacle in obstacles_considered:
                # If an obstacle has no trajectory, 
                # then, ignore all obstacles from the same lane.
                point = obstacle.GetPointAtTime(relative_time)
                obs_box = obstacle.GetBoundingBox(point)

                obs_box.LongitudinalExtend(2.0 * config_module.FLAGS_lon_collision_buffer)
                obs_box.LateralExtend(2.0 * config_module.FLAGS_lat_collision_buffer)
                predicted_env.append(obs_box)

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
        ok, lane_left_width, lane_right_width = self.reference_line_info.reference_line.GetLaneWidth(ego_vehicle_s)
        if ok:
            left_width = lane_left_width
            right_width = lane_right_width
        return ego_vehicle_d < left_width and ego_vehicle_d > -right_width

    def IsObstacleBehindEgoVehicle(self, obstacle: Obstacle, ego_vehicle_s: float,
                                   discretized_reference_line: List[PathPoint]) -> bool:
        """
        Check if the obstacle is behind the ego vehicle.

        :param Obstacle obstacle: Obstacle
        :param float ego_vehicle_s: Ego vehicle s-coordinate
        :param List[PathPoint] discretized_reference_line: Discretized reference line
        :returns: True if the obstacle is behind the ego vehicle, False otherwise
        :rtype: bool
        """

        half_lane_width = config_module.FLAGS_default_reference_line_width * 0.5
        point: TrajectoryPoint = obstacle.GetPointAtTime(0.0)
        obstacle_reference_line_position = PathMatcher.GetPathFrenetCoordinate(
            discretized_reference_line, point.path_point.x, point.path_point.y)

        if (obstacle_reference_line_position[0] < ego_vehicle_s and
                fabs(obstacle_reference_line_position[1]) < half_lane_width):
            self.logger.warning(f"Ignore obstacle [{obstacle.Id()}]")
            return True
        return False
