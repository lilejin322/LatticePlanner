from typing import List, Tuple, Dict
import numpy as np
from common.Obstacle import Obstacle
from common.ReferenceLineInfo import ReferenceLineInfo
from common.PathPoint import PathPoint
from PathMatcher import PathMatcher
from common.SLBoundary import SLBoundary
from common.STBoundary import STBoundary
from logging import Logger
from config import FLAGS_default_reference_line_width, FLAGS_trajectory_time_length

class PathTimeGraph:
    """
    PathTimeGraph is a graph that represents the feasible path-time space for a vehicle.
    """

    def __init__(self, obstacles: List[Obstacle], discretized_ref_points: List[PathPoint],
                 reference_line_info: ReferenceLineInfo, s_start: float, s_end: float,
                 t_start: float, t_end: float, init_d: List[float]):
        """
        Construct a PathTimeGraph with obstacles, reference line info, and graph ranges.

        :param List[Obstacle] obstacles: a list of obstacles.
        :param List[PathPoint] discretized_ref_points: discretized reference points.
        :param ReferenceLineInfo reference_line_info: reference line information.
        :param float s_start: start s value of the graph.
        :param float s_end: end s value of the graph.
        :param float t_start: start time of the graph.
        :param float t_end: end time of the graph.
        :param List[float] init_d: initial d values.
        """

        assert s_start < s_end, "s_start must be less than s_end"
        assert t_start < t_end, "t_start must be less than t_end"

        self.path_range_ = (s_start, s_end)
        self.time_range_ = (t_start, t_end)
        self.reference_line_info = reference_line_info
        self.init_d_ = init_d
        self.logger = Logger("PathTimeGraph")

        self.path_time_obstacle_map: List[STBoundary] = {}
        self.path_time_obstacles: List[STBoundary] = []
        self.static_obs_sl_boundaries: List[SLBoundary] = []

        self.SetupObstacles(obstacles, discretized_ref_points)

    def ComputeObstacleBoundary(self, vertices, discretized_ref_points: List[PathPoint]):
        """
        Compute the boundary of the obstacle in the s-l coordinate.

        :param List[Vec2d] vertices: vertices of the obstacle.
        :param List[PathPoint] discretized_ref_points: discretized reference points.
        :return: the boundary of the obstacle in the s-l coordinate.
        :rtype: SLBoundary
        """
    
        start_s = float('inf')
        end_s = float('-inf')
        start_l = float('inf')
        end_l = float('-inf')

        for point in vertices:
            sl_point = PathMatcher.GetPathFrenetCoordinate(discretized_ref_points, point.x, point.y)
            start_s = min(start_s, sl_point[0])
            end_s = max(end_s, sl_point[0])
            start_l = min(start_l, sl_point[1])
            end_l = max(end_l, sl_point[1])

        sl_boundary = SLBoundary()
        sl_boundary.set_start_s(start_s)
        sl_boundary.set_end_s(end_s)
        sl_boundary.set_start_l(start_l)
        sl_boundary.set_end_l(end_l)

        return sl_boundary

    def SetUpObstacles(self, obstacles: List[Obstacle], discretized_ref_points: List[PathPoint]):
        """
        Set up obstacles for the PathTimeGraph.

        :param List[Obstacle] obstacles: a list of obstacles.
        :param List[PathPoint] discretized_ref_points: discretized reference points.
        """
        for obstacle in obstacles:
            if obstacle.Is_virtual:
                continue
            if not obstacle.HasTrajectory:
                self.SetStaticObstacle(obstacle, discretized_ref_points)
            else:
                self.SetDynamicObstacle(obstacle, discretized_ref_points)

        self.static_obs_sl_boundaries.sort(key=lambda sl: sl.start_s())

        for path_time_obstacle in self.path_time_obstacle_map:
            self.path_time_obstacles.append(path_time_obstacle[1])

    def SetStaticObstacle(self, obstacle: Obstacle, discretized_ref_points: List[PathPoint]):
        """
        Set a static obstacle for the PathTimeGraph.

        :param Obstacle obstacle: a static obstacle.
        :param List[PathPoint] discretized_ref_points: discretized reference points.
        """

        polygon = obstacle.PerceptionPolygon()

        obstacle_id: str = obstacle.Id
        sl_boundary: SLBoundary = self.ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points)

        left_width: float = FLAGS_default_reference_line_width * 0.5
        right_width: float = FLAGS_default_reference_line_width * 0.5
        left_width, right_width = self.reference_line_info.reference_line.GetLaneWidth(sl_boundary.start_s)
        if (sl_boundary.start_s() > self.path_range_[1] or
                sl_boundary.end_s() < self.path_range_[0] or
                sl_boundary.start_l() > left_width or
                sl_boundary.end_l() < -right_width):
            self.logger.debug(f"Obstacle [{obstacle_id}] is out of range.")
            return

        self.path_time_obstacle_map[obstacle_id].set_id(obstacle_id)
        self.path_time_obstacle_map[obstacle_id].set_bottom_left_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0))
        self.path_time_obstacle_map[obstacle_id].set_bottom_right_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.start_s(), FLAGS_trajectory_time_length))
        self.path_time_obstacle_map[obstacle_id].set_upper_left_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0))
        self.path_time_obstacle_map[obstacle_id].set_upper_right_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.end_s(), FLAGS_trajectory_time_length))
        self.static_obs_sl_boundaries.append(sl_boundary)
        self.logger.debug(f"ST-Graph mapping static obstacle: {obstacle_id}, start_s: {sl_boundary.start_s()}, "
              f"end_s: {sl_boundary.end_s()}, start_l: {sl_boundary.start_l()}, end_l: {sl_boundary.end_l()}")

    def SetDynamicObstacle():
        pass




    def get_path_time_obstacles(self) -> List['STBoundary']:
        return self.path_time_obstacles

    def get_path_time_obstacle(self, obstacle_id: str, path_time_obstacle: 'STBoundary') -> bool:
        if obstacle_id in self.path_time_obstacle_map:
            path_time_obstacle = self.path_time_obstacle_map[obstacle_id]
            return True
        return False

    def get_path_blocking_intervals(self, t: float) -> List[Tuple[float, float]]:
        pass

    def get_path_blocking_intervals(self, t_start: float, t_end: float, t_resolution: float) -> List[List[Tuple[float, float]]]:
        pass

    def get_path_range(self) -> Tuple[float, float]:
        return self.path_range

    def get_time_range(self) -> Tuple[float, float]:
        return self.time_range

    def get_obstacle_surrounding_points(self, obstacle_id: str, s_dist: float, t_density: float) -> List['STPoint']:
        pass

    def is_obstacle_in_graph(self, obstacle_id: str) -> bool:
        return obstacle_id in self.path_time_obstacle_map

    def get_lateral_bounds(self, s_start: float, s_end: float, s_resolution: float) -> List[Tuple[float, float]]:
        pass

    def setup_obstacles(self, obstacles: List['Obstacle'], discretized_ref_points: List['PathPoint']):
        pass

    def set_path_time_point(self, obstacle_id: str, s: float, t: float) -> 'STPoint':
        pass

    def set_dynamic_obstacle(self, obstacle: 'Obstacle', discretized_ref_points: List['PathPoint']):
        pass

    def update_lateral_bounds_by_obstacle(self, sl_boundary: 'SLBoundary', discretized_path: List[float],
                                          s_start: float, s_end: float, bounds: List[Tuple[float, float]]):
        pass






class STPoint:
    pass

class Vec2d:
    pass
