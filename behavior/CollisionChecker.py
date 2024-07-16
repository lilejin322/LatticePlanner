from typing import List
from math import cos, sin, fabs, atan2, pi
from common.Obstacle import Obstacle
from protoclass.PathPoint import PathPoint
from common.ReferenceLineInfo import ReferenceLineInfo
from behavior.PathTimeGraph import PathTimeGraph
from common.DiscretizedTrajectory import DiscretizedTrajectory
from protoclass.TrajectoryPoint import TrajectoryPoint
from logging import Logger
from PathMatcher import PathMatcher
from shapely.geometry import box
from shapely.affinity import translate, rotate
from config import EGO_VEHICLE_LENGTH, EGO_VEHICLE_WIDTH, EGO_BACK_EDGE_TO_CENTER, FLAGS_trajectory_time_length, \
                   FLAGS_lon_collision_buffer, FLAGS_lat_collision_buffer, FLAGS_trajectory_time_resolution, \
                   FLAGS_default_reference_line_width

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
        self.BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d, discretized_reference_line)
        self.logger = Logger("CollisionChecker")

    @staticmethod
    def StaticInCollision(obstacles: List[Obstacle], ego_trajectory: DiscretizedTrajectory, ego_length: float,
                          ego_width: float, ego_back_edge_to_center: float) -> bool:
        """
        Check if the ego vehicle is in collision with the obstacles

        :param List[Obstacle] obstacles: List of obstacles
        :param DiscretizedTrajectory ego_trajectory: Ego vehicle trajectory
        :param float ego_length: Ego vehicle length
        :param float ego_width: Ego vehicle width
        :param float ego_back_edge_to_center: Ego vehicle back edge to center distance
        :returns: True if the ego vehicle is in collision with the obstacles, False otherwise
        :rtype: bool
        """

        for i in range(ego_trajectory.NumOfPoints):
            ego_point = ego_trajectory.TrajectoryPointAt(i)
            relative_time = ego_point.relative_time()
            ego_theta = ego_point.path_point.theta
            
            # create ego vehicle bounding box
            ego_box = box(
                ego_point.path_point.x - ego_length / 2.0,
                ego_point.path_point.y - ego_width / 2.0,
                ego_point.path_point.x + ego_length / 2.0,
                ego_point.path_point.y + ego_width / 2.0
            )

            # rotate the bounding box to theta, note that the rotation is using radians
            ego_box = rotate(ego_box, ego_theta, origin='centroid', use_radians=True)

            # correct the inconsistency of reference point and center point
            # TODO(all): move the logic before constructing the ego_box
            shift_distance = ego_length / 2.0 - ego_back_edge_to_center
            shift_vec_x = shift_distance * cos(ego_theta)
            shift_vec_y = shift_distance * sin(ego_theta)
            ego_box = translate(ego_box, xoff=shift_vec_x, yoff=shift_vec_y)

            for obstacle in obstacles:
                obstacle_point = obstacle.GetPointAtTime(relative_time)
                obstacle_box = obstacle.GetBoundingBox(obstacle_point)

                # Check whether overlap exists
                if ego_box.intersects(obstacle_box):
                    return True
        return False

    def InCollision(self, discretized_trajectory: DiscretizedTrajectory) -> bool:
        """
        Check if the ego vehicle is in collision with the predicted bounding rectangles

        :param DiscretizedTrajectory discretized_trajectory: Ego vehicle trajectory
        :returns: True if the ego vehicle is in collision with the predicted bounding rectangles
        :rtype: bool
        """

        if discretized_trajectory.NumOfPoints > len(self.predicted_bounding_rectangles):
            raise ValueError("Number of trajectory points exceeds the number of predicted bounding rectangles.")

        ego_length = EGO_VEHICLE_LENGTH
        ego_width = EGO_VEHICLE_WIDTH

        for i in range(discretized_trajectory.NumOfPoints):

            trajectory_point = discretized_trajectory.TrajectoryPointAt(i)
            ego_theta = trajectory_point.path_point.theta

            # create ego vehicle bounding box
            ego_box = box(
                trajectory_point.path_point.x - ego_length / 2.0,
                trajectory_point.path_point.y - ego_width / 2.0,
                trajectory_point.path_point.x + ego_length / 2.0,
                trajectory_point.path_point.y + ego_width / 2.0
            )
            # rotate the bounding box to theta
            ego_box = rotate(ego_box, ego_theta, origin='centroid', use_radians=True)

            shift_distance = ego_length / 2.0 - EGO_BACK_EDGE_TO_CENTER
            shift_vec_x = shift_distance * cos(ego_theta)
            shift_vec_y = shift_distance * sin(ego_theta)
            ego_box = translate(ego_box, xoff=shift_vec_x, yoff=shift_vec_y)

            for obstacle_box in self.predicted_bounding_rectangles[i]:
                if ego_box.intersects(obstacle_box):
                    return True
        return False

    def BuildPredictedEnvironment(self, obstacles: List[Obstacle], ego_vehicle_s: float, ego_vehicle_d: float,
                                  discretized_reference_line: List[PathPoint]):
        """
        Build predicted environment

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
                                        or not self.path_time_graph.IsObstacleInGraph(obstacle.Id)):
                continue
            obstacles_considered.append(obstacle)

        relative_time: float = 0.0
        while relative_time < FLAGS_trajectory_time_length:
            predicted_env = []
            for obstacle in obstacles_considered:
                # If an obstacle has no trajectory, 
                # then, ignore all obstacles from the same lane.
                point = obstacle.GetPointAtTime(relative_time)
                obs_box = obstacle.GetBoundingBox(point)

                # get the 4 vertices of the obstacle box and calculate the angle
                vertices = list(obs_box.exterior.coords)[:-1] 
                p1 = vertices[0]
                p2 = vertices[1]
                delta_x = p2[0] - p1[0]
                delta_y = p2[1] - p1[1]
                # get the rotation angle in radians
                obs_theta = atan2(delta_y, delta_x)
                # Normalize the angle to [0, π)
                obs_theta = obs_theta % pi
                rot_box = rotate(obs_box, -obs_theta, origin='centroid', use_radians=True)
                # get the min_x, min_y, max_x, max_y of the rotated box
                min_x, min_y, max_x, max_y = rot_box.bounds
                # extend the box
                # we have confirmed the source code in apollo Box2d
                # (lon, lat) <-> (x, y) <-> (length, width) corresponding to shapely
                exd_box = box(min_x - FLAGS_lon_collision_buffer, min_y - FLAGS_lat_collision_buffer,
                              max_x + FLAGS_lon_collision_buffer, max_y + FLAGS_lat_collision_buffer)
                # rotate the obs_box to the correct angle
                rot_exd_box = rotate(exd_box, obs_theta, origin='centroid', use_radians=True)

                predicted_env.append(rot_exd_box)

            self.predicted_bounding_rectangles.append(predicted_env)
            relative_time += FLAGS_trajectory_time_resolution

    def IsEgoVehicleInLane(self, ego_vehicle_s: float, ego_vehicle_d: float) -> bool:
        """
        Check if the ego vehicle is in lane

        :param float ego_vehicle_s: Ego vehicle s-coordinate
        :param float ego_vehicle_d: Ego vehicle d-coordinate
        :returns: True if the ego vehicle is in lane, False otherwise
        :rtype: bool
        """

        left_width: float = FLAGS_default_reference_line_width * 0.5
        right_width: float = FLAGS_default_reference_line_width * 0.5
        self.reference_line_info.reference_line.GetLaneWidth(ego_vehicle_s, left_width, right_width)
        return ego_vehicle_d < left_width and ego_vehicle_d > -right_width

    def IsObstacleBehindEgoVehicle(self, obstacle: Obstacle, ego_vehicle_s: float,
                                   discretized_reference_line: List[PathPoint]) -> bool:
        """
        Check if the obstacle is behind the ego vehicle

        :param Obstacle obstacle: Obstacle
        :param float ego_vehicle_s: Ego vehicle s-coordinate
        :param List[PathPoint] discretized_reference_line: Discretized reference line
        :returns: True if the obstacle is behind the ego vehicle, False otherwise
        :rtype: bool
        """

        half_lane_width = FLAGS_default_reference_line_width * 0.5
        point: TrajectoryPoint = obstacle.GetPointAtTime(0.0)
        obstacle_reference_line_position = PathMatcher.GetPathFrenetCoordinate(
            discretized_reference_line, point.path_point.x, point.path_point.y)

        if (obstacle_reference_line_position[0] < ego_vehicle_s and
                fabs(obstacle_reference_line_position[1]) < half_lane_width):
            self.logger.warning(f"Ignore obstacle [{obstacle.Id}]")
            return True
        return False
