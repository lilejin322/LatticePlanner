from typing import List, Tuple, Dict
from common.Obstacle import Obstacle
from common.ReferenceLineInfo import ReferenceLineInfo
from common.PathPoint import PathPoint
from PathMatcher import PathMatcher
from common.SLBoundary import SLBoundary
from common.STBoundary import STBoundary
from common.STPoint import STPoint
from logging import Logger
from config import FLAGS_default_reference_line_width, FLAGS_trajectory_time_length, FLAGS_trajectory_time_resolution,\
                   FLAGS_numerical_epsilon, EGO_VEHICLE_WIDTH, FLAGS_bound_buffer
from common.TrajectoryPoint import TrajectoryPoint
from shapely.geometry import Polygon
from bisect import bisect_left, bisect_right
from common.Vec2d import Vec2d

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
        self._init_d = init_d
        self.logger = Logger("PathTimeGraph")

        self.path_time_obstacle_map: Dict[str, STBoundary] = {}
        self.path_time_obstacles: List[STBoundary] = []
        self.static_obs_sl_boundaries: List[SLBoundary] = []

        self.SetUpObstacles(obstacles, discretized_ref_points)

    @staticmethod
    def ComputeObstacleBoundary(vertices: List[Vec2d], discretized_ref_points: List[PathPoint]) -> SLBoundary:
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

    def SetUpObstacles(self, obstacles: List[Obstacle], discretized_ref_points: List[PathPoint]) -> None:
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

    def SetStaticObstacle(self, obstacle: Obstacle, discretized_ref_points: List[PathPoint]) -> None:
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
        if (sl_boundary.start_s > self.path_range_[1] or
                sl_boundary.end_s < self.path_range_[0] or
                sl_boundary.start_l > left_width or
                sl_boundary.end_l < -right_width):
            self.logger.debug(f"Obstacle [{obstacle_id}] is out of range.")
            return

        self.path_time_obstacle_map[obstacle_id] = STBoundary(obstacle_id)
        self.path_time_obstacle_map[obstacle_id].set_bottom_left_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.start_s, 0.0))
        self.path_time_obstacle_map[obstacle_id].set_bottom_right_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.start_s, FLAGS_trajectory_time_length))
        self.path_time_obstacle_map[obstacle_id].set_upper_left_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.end_s, 0.0))
        self.path_time_obstacle_map[obstacle_id].set_upper_right_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.end_s, FLAGS_trajectory_time_length))
        self.static_obs_sl_boundaries.append(sl_boundary)
        self.logger.debug(f"ST-Graph mapping static obstacle: {obstacle_id}, start_s: {sl_boundary.start_s}, "
                          f"end_s: {sl_boundary.end_s}, start_l: {sl_boundary.start_l}, end_l: {sl_boundary.end_l}")

    def SetDynamicObstacle(self, obstacle: Obstacle, discretized_ref_points: List[PathPoint]):
        """
        Set a dynamic obstacle for the PathTimeGraph.

        :param Obstacle obstacle: a dynamic obstacle.
        :param List[PathPoint] discretized_ref_points: discretized reference points.
        """

        relative_time: float = self.time_range_[0]
        while relative_time < self.time_range_[1]:
            point: TrajectoryPoint = obstacle.GetPointAtTime(relative_time)
            box: Polygon = obstacle.GetBoundingBox(point)
            sl_boundary: SLBoundary = self.ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points)

            left_width: float = FLAGS_default_reference_line_width * 0.5
            right_width: float = FLAGS_default_reference_line_width * 0.5
            left_width, right_width = self.reference_line_info.reference_line.GetLaneWidth(sl_boundary.start_s)

            # The obstacle is not shown on the region to be considered.
            if (sl_boundary.start_s > self.path_range_[1] or
                sl_boundary.end_s < self.path_range_[0] or
                sl_boundary.start_l > left_width or
                sl_boundary.end_l < -right_width):
                if obstacle.Id in self.path_time_obstacle_map:
                    break
                relative_time += FLAGS_trajectory_time_resolution
                continue

            if obstacle.Id not in self.path_time_obstacle_map:
                self.path_time_obstacle_map[obstacle.Id] = STBoundary(obstacle.Id)

                self.path_time_obstacle_map[obstacle.Id].set_bottom_left_point(self.SetPathTimePoint(
                    obstacle.Id, sl_boundary.start_s, relative_time))
                self.path_time_obstacle_map[obstacle.Id].set_upper_left_point(self.SetPathTimePoint(
                    obstacle.Id, sl_boundary.end_s, relative_time))

            self.path_time_obstacle_map[obstacle.Id].set_bottom_right_point(self.SetPathTimePoint(
                obstacle.Id, sl_boundary.start_s, relative_time))
            self.path_time_obstacle_map[obstacle.Id].upper_right_point = self.SetPathTimePoint(
                obstacle.Id, sl_boundary.end_s, relative_time)
            relative_time += FLAGS_trajectory_time_resolution

    @staticmethod
    def SetPathTimePoint(obstacle_id: str, s: float, t: float) -> STPoint:
        """
        Set a path-time point for the PathTimeGraph.

        :param str obstacle_id: the obstacle ID.
        :param float s: the s value.
        :param float t: the time value.
        :returns: the path-time point.
        :rtype: STPoint
        """

        path_time_point: STPoint = STPoint(s, t)
        return path_time_point

    def GetPathTimeObstacles(self) -> List[STBoundary]:
        """
        Get the path-time obstacles.
        """

        return self.path_time_obstacles

    def GetPathTimeObstacle(self, obstacle_id: str) -> Tuple[bool, STBoundary]:
        """
        Get the path-time obstacle by the obstacle ID.

        :param str obstacle_id: the obstacle ID.
        :returns: a tuple of a boolean value and the path-time obstacle.
        :rtype: Tuple[bool, STBoundary]
        """

        if obstacle_id in self.path_time_obstacle_map:
            return True, self.path_time_obstacle_map[obstacle_id]
        return False, None

    def GetPathBlockingIntervalsT(self, t: float) -> List[Tuple[float, float]]:
        """
        Get the path blocking intervals.

        :param float t: the time value.
        :returns: the path blocking intervals.
        :rtype: List[Tuple[float, float]]
        """

        assert self.time_range_[0] <= t <= self.time_range_[1], "Time t is out of the time range."
        intervals: List[Tuple[float, float]] = []
        for pt_obstacle in self.path_time_obstacles:
            if t > pt_obstacle.max_t or t < pt_obstacle.min_t:
                continue
            s_upper: float = self.lerp(pt_obstacle.upper_left_point.s, pt_obstacle.upper_left_point.t,
                                       pt_obstacle.upper_right_point.s, pt_obstacle.upper_right_point.t, t)
            s_lower: float = self.lerp(pt_obstacle.bottom_left_point.s, pt_obstacle.bottom_left_point.t,
                                       pt_obstacle.bottom_right_point.s, pt_obstacle.bottom_right_point.t, t)
            intervals.append((s_lower, s_upper))
        return intervals

    def lerp(self, x0: float, t0: float, x1: float, t1: float, t: float) -> float:
        """
        Linear interpolation.

        :param float x0: the x0 value.
        :param float t0: the t0 value.
        :param float x1: the x1 value.
        :param float t1: the t1 value.
        :param float t: the t value.
        :returns: the interpolated value.
        :rtype: float
        """

        if abs(t1 - t0) <= 1.0e-6:
            self.logger.error("Input time difference is too small")
            return x0
        r = (t - t0) / (t1 - t0)
        x = x0 + r * (x1 - x0)
        return x

    def GetPathBlockingIntervals(self, t_start: float, t_end: float,
                                 t_resolution: float) -> List[List[Tuple[float, float]]]:
        """
        Get the path blocking intervals.

        :param float t_start: the start time value.
        :param float t_end: the end time value.
        :param float t_resolution: the time resolution.
        :returns: the path blocking intervals.
        :rtype: List[List[Tuple[float, float]]]
        """

        intervals: List[List[Tuple[float, float]]] = []
        t = t_start
        while t <= t_end:
            intervals.append(self.GetPathBlockingIntervalsT(t))
            t += t_resolution
        return intervals

    def get_path_range(self) -> Tuple[float, float]:
        return self.path_range_

    def get_time_range(self) -> Tuple[float, float]:
        return self.time_range_

    def GetObstacleSurroundingPoints(self, obstacle_id: str, s_dist: float, t_min_density: float) -> List[STPoint]:
        """
        Get the surrounding points of the obstacle.

        :param str obstacle_id: the obstacle ID.
        :param float s_dist: the s distance.
        :param float t_min_density: the minimum time density.
        :returns: the surrounding points of the obstacle.
        :rtype: List[STPoint]
        """

        assert t_min_density > 0.0, "t_min_density must be greater than 0.0"
        pt_pairs: List[STPoint] = []
        if obstacle_id not in self.path_time_obstacle_map:
            return pt_pairs

        pt_obstacle: STBoundary = self.path_time_obstacle_map[obstacle_id]

        s0: float = 0.0
        s1: float = 0.0

        t0: float = 0.0
        t1: float = 0.0

        if s_dist > 0.0:
            s0 = pt_obstacle.upper_left_point.s
            s1 = pt_obstacle.upper_right_point.s

            t0 = pt_obstacle.upper_left_point.t
            t1 = pt_obstacle.upper_right_point.t
        else:
            s0 = pt_obstacle.bottom_left_point.s
            s1 = pt_obstacle.bottom_right_point.s

            t0 = pt_obstacle.bottom_left_point.t
            t1 = pt_obstacle.bottom_right_point.t

        time_gap: float = t1 - t0

        assert time_gap > -FLAGS_numerical_epsilon, "Time gap is negative"
        time_gap = abs(time_gap)

        num_sections: int = int(time_gap / t_min_density) + 1
        t_interval: float = time_gap / num_sections

        for i in range(num_sections + 1):
            t = t_interval * i + t0
            s = self.lerp(s0, t0, s1, t1, t) + s_dist

            ptt = STPoint(s, t)
            pt_pairs.append(ptt)

        return pt_pairs

    def IsObstacleInGraph(self, obstacle_id: str) -> bool:
        """
        Check if the obstacle is in the graph.

        :param str obstacle_id: the obstacle ID.
        :returns: True if the obstacle is in the graph, False otherwise.
        :rtype: bool
        """

        return obstacle_id in self.path_time_obstacle_map

    def GetLateralBounds(self, s_start: float, s_end: float, s_resolution: float) -> List[Tuple[float, float]]:
        """
        Get the lateral bounds.

        :param float s_start: the start s value.
        :param float s_end: the end s value.
        :param float s_resolution: the s resolution.
        :returns: the lateral bounds.
        :rtype: List[Tuple[float, float]]
        """
        assert s_start < s_end, "s_start must be less than s_end"
        assert s_resolution > FLAGS_numerical_epsilon, "s_resolution must be greater than numerical epsilon"
        bounds: List[Tuple[float, float]] = []
        discretized_path: List[float] = []
        s_range: float = s_end - s_start
        s_curr: float = s_start
        num_bound: int = int(s_range / s_resolution)

        ego_width: float = EGO_VEHICLE_WIDTH

        # Initialize bounds by reference line width
        for _ in range(num_bound):
            left_width: float = FLAGS_default_reference_line_width / 2.0
            right_width: float = FLAGS_default_reference_line_width / 2.0
            left_width, right_width = self.reference_line_info.reference_line.GetLaneWidth(s_curr)
            ego_d_lower: float = self._init_d[0] - ego_width / 2.0
            ego_d_upper: float = self._init_d[0] + ego_width / 2.0
            bounds.append(min(-right_width, ego_d_lower - FLAGS_bound_buffer),
                          max(left_width, ego_d_upper + FLAGS_bound_buffer))
            discretized_path.append(s_curr)
            s_curr += s_resolution
        
        for static_sl_boundary in self.static_obs_sl_boundaries:
            self.UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start, s_end, bounds)

        for i in range(len(bounds)):
            bounds[i] = (bounds[i][0] + ego_width / 2.0, bounds[i][1] - ego_width / 2.0)
            if bounds[i][0] >= bounds[i][1]:
                bounds[i] = (0.0, 0.0)
        return bounds

    @staticmethod
    def UpdateLateralBoundsByObstacle(sl_boundary: SLBoundary, discretized_path: List[float],
                                      s_start: float, s_end: float, bounds: List[Tuple[float, float]]) -> None:
        """
        Update the lateral bounds by the obstacle.

        :param SLBoundary sl_boundary: the s-l boundary of the obstacle.
        :param List[float] discretized_path: the discretized path.
        :param float s_start: the start s value.
        :param float s_end: the end s value.
        :param List[Tuple[float, float]] bounds: the lateral bounds.
        """

        if sl_boundary.start_s > s_end or sl_boundary.end_s < s_start:
            return
        start_iter: int = bisect_left(discretized_path, sl_boundary.start_s())
        end_iter: int = bisect_right(discretized_path, sl_boundary.end_s())
        start_index = start_iter - discretized_path[0]
        end_index = end_iter - discretized_path[0]
        if sl_boundary.end_l > -FLAGS_numerical_epsilon and sl_boundary.start_l < FLAGS_numerical_epsilon:
            for i in range(start_index, end_index):
                bounds[i] = (-FLAGS_numerical_epsilon, FLAGS_numerical_epsilon)
            return
        if sl_boundary.end_l < FLAGS_numerical_epsilon:
            for i in range(start_index, min(end_index + 1, len(bounds))):
                bounds[i][0] = max(bounds[i][0], sl_boundary.end_l + FLAGS_bound_buffer)
            return
        if sl_boundary.start_l > -FLAGS_numerical_epsilon:
            for i in range(start_index, min(end_index + 1, len(bounds))):
                bounds[i][1] = min(bounds[i][1], sl_boundary.start_l - FLAGS_bound_buffer)
            return
