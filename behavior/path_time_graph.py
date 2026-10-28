"""
Path time graph submodule
"""
from typing import List, Tuple, Dict
import config as config_module
from common.st_point import STPoint
from common.geometry_utils import lerp
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.sl_boundary import SLBoundary
from path_matcher import PathMatcher
from logging import Logger
from bisect import bisect_left, bisect_right

class PathTimeGraph:
    """
    PathTimeGraph is a graph that represents the feasible path-time space for a vehicle.
    """

    def __init__(self, obstacles: list, discretized_ref_points: List[PathPoint],
                 lane_width, s_start: float, s_end: float,
                 t_start: float, t_end: float, init_d: List[float]) -> None:
        """
        Construct a PathTimeGraph.

        ``lane_width(s)`` returns ``(ok, left, right)`` from the lanelet.
        """
        assert s_start < s_end, "s_start must be less than s_end"
        assert t_start < t_end, "t_start must be less than t_end"

        self._path_range = (s_start, s_end)
        self._time_range = (t_start, t_end)
        self.lane_width = lane_width
        self._init_d = init_d
        self.logger = Logger("PathTimeGraph")
        self.path_time_obstacle_map: Dict[str, "_STRegion"] = {}
        self.path_time_obstacles: List["_STRegion"] = []
        self.static_obs_sl_boundaries: List[SLBoundary] = []
        self.SetUpObstacles(obstacles, discretized_ref_points)

    def _half_widths(self, station: float) -> Tuple[float, float]:
        left = config_module.FLAGS_default_reference_line_width * 0.5
        right = left
        ok, queried_left, queried_right = self.lane_width(station)
        if ok:
            left = queried_left
            right = queried_right
        return left, right

    @staticmethod
    def ComputeObstacleBoundary(vertices, discretized_ref_points: List[PathPoint]) -> SLBoundary:
        """
        Compute the boundary of the obstacle in the s-l coordinate.

        :param List[Vec2d] vertices: vertices of the obstacle
        :param List[PathPoint] discretized_ref_points: discretized reference points
        :return: the boundary of the obstacle in the s-l coordinate
        :rtype: SLBoundary
        """
        start_s = float('inf')
        end_s = float('-inf')
        start_l = float('inf')
        end_l = float('-inf')

        for point in vertices:
            sl_point = PathMatcher.GetPathFrenetCoordinate(
                discretized_ref_points, float(point[0]), float(point[1])
            )
            start_s = min(start_s, sl_point[0])
            end_s = max(end_s, sl_point[0])
            start_l = min(start_l, sl_point[1])
            end_l = max(end_l, sl_point[1])

        return SLBoundary(start_s=start_s, end_s=end_s, start_l=start_l, end_l=end_l)

    def SetUpObstacles(self, obstacles: list, discretized_ref_points: List[PathPoint]) -> None:
        """
        Set up obstacles for the PathTimeGraph.

        :param list obstacles: a list of obstacles
        :param List[PathPoint] discretized_ref_points: discretized reference points
        """
        for obstacle in obstacles:
            if not obstacle.moving():
                self.SetStaticObstacle(obstacle, discretized_ref_points)
            else:
                self.SetDynamicObstacle(obstacle, discretized_ref_points)

        self.static_obs_sl_boundaries.sort(key=lambda sl: sl.start_s)

        for path_time_obstacle in self.path_time_obstacle_map.values():
            self.path_time_obstacles.append(path_time_obstacle)

    def SetStaticObstacle(self, obstacle, discretized_ref_points: List[PathPoint]) -> None:
        """
        Set a static obstacle for the PathTimeGraph.

        :param Obstacle obstacle: a static obstacle
        :param List[PathPoint] discretized_ref_points: discretized reference points
        """
        obstacle_id: str = obstacle.id
        sl_boundary: SLBoundary = self.ComputeObstacleBoundary(
            obstacle.corners_at(0.0), discretized_ref_points
        )

        left_width, right_width = self._half_widths(sl_boundary.start_s)
        if (sl_boundary.start_s > self._path_range[1] or
                sl_boundary.end_s < self._path_range[0] or
                sl_boundary.start_l > left_width or
                sl_boundary.end_l < -right_width):
            self.logger.debug(f"Obstacle [{obstacle_id}] is out of range.")
            return

        self.path_time_obstacle_map[obstacle_id] = _STRegion(obstacle_id)
        self.path_time_obstacle_map[obstacle_id].set_bottom_left_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.start_s, 0.0))
        self.path_time_obstacle_map[obstacle_id].set_bottom_right_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.start_s, config_module.FLAGS_trajectory_time_length))
        self.path_time_obstacle_map[obstacle_id].set_upper_left_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.end_s, 0.0))
        self.path_time_obstacle_map[obstacle_id].set_upper_right_point(
            self.SetPathTimePoint(obstacle_id, sl_boundary.end_s, config_module.FLAGS_trajectory_time_length))
        self.static_obs_sl_boundaries.append(sl_boundary)
        self.logger.debug(f"ST-Graph mapping static obstacle: {obstacle_id}, start_s: {sl_boundary.start_s}, "
                          f"end_s: {sl_boundary.end_s}, start_l: {sl_boundary.start_l}, end_l: {sl_boundary.end_l}")

    def SetDynamicObstacle(self, obstacle, discretized_ref_points: List[PathPoint]) -> None:
        """
        Set a dynamic obstacle for the PathTimeGraph.

        :param Obstacle obstacle: a dynamic obstacle
        :param List[PathPoint] discretized_ref_points: discretized reference points
        """
        relative_time: float = self._time_range[0]
        while relative_time < self._time_range[1]:
            sl_boundary: SLBoundary = self.ComputeObstacleBoundary(
                obstacle.corners_at(relative_time), discretized_ref_points
            )

            left_width, right_width = self._half_widths(sl_boundary.start_s)

            # The obstacle is not shown on the region to be considered.
            if (sl_boundary.start_s > self._path_range[1] or
                sl_boundary.end_s < self._path_range[0] or
                sl_boundary.start_l > left_width or
                sl_boundary.end_l < -right_width):
                if obstacle.id in self.path_time_obstacle_map:
                    break
                relative_time += config_module.FLAGS_trajectory_time_resolution
                continue

            if obstacle.id not in self.path_time_obstacle_map:
                self.path_time_obstacle_map[obstacle.id] = _STRegion(obstacle.id)

                self.path_time_obstacle_map[obstacle.id].set_bottom_left_point(self.SetPathTimePoint(
                    obstacle.id, sl_boundary.start_s, relative_time))
                self.path_time_obstacle_map[obstacle.id].set_upper_left_point(self.SetPathTimePoint(
                    obstacle.id, sl_boundary.end_s, relative_time))

            self.path_time_obstacle_map[obstacle.id].set_bottom_right_point(self.SetPathTimePoint(
                obstacle.id, sl_boundary.start_s, relative_time))
            self.path_time_obstacle_map[obstacle.id].set_upper_right_point(self.SetPathTimePoint(
                obstacle.id, sl_boundary.end_s, relative_time))
            relative_time += config_module.FLAGS_trajectory_time_resolution

    @staticmethod
    def SetPathTimePoint(obstacle_id: str, s: float, t: float) -> STPoint:
        """
        Set a path-time point for the PathTimeGraph.

        :param str obstacle_id: the obstacle ID
        :param float s: the s value
        :param float t: the time value
        :returns: the path-time point
        :rtype: STPoint
        """
        path_time_point: STPoint = STPoint(s, t)
        return path_time_point

    def GetPathTimeObstacles(self) -> list:
        """
        Get the path-time obstacles.

        :returns: the list of path-time obstacles
        :rtype: list
        """
        return self.path_time_obstacles

    def GetPathTimeObstacle(self, obstacle_id: str):
        """
        Get the path-time obstacle by the obstacle ID.

        :param str obstacle_id: the obstacle ID
        :returns: a tuple of a boolean value and the path-time obstacle
        :rtype: Tuple[bool, STBoundary]
        """
        if obstacle_id in self.path_time_obstacle_map:
            return True, self.path_time_obstacle_map[obstacle_id]
        return False, None

    def GetPathBlockingIntervalsT(self, t: float) -> List[Tuple[float, float]]:
        """
        Get the path blocking intervals.

        :param float t: the time value
        :returns: the path blocking intervals
        :rtype: List[Tuple[float, float]]
        """
        assert self._time_range[0] <= t <= self._time_range[1], "Time t is out of the time range."
        intervals: List[Tuple[float, float]] = []
        for pt_obstacle in self.path_time_obstacles:
            if t > pt_obstacle.max_t or t < pt_obstacle.min_t:
                continue
            s_upper: float = lerp(pt_obstacle.upper_left_point.s, pt_obstacle.upper_left_point.t,
                                       pt_obstacle.upper_right_point.s, pt_obstacle.upper_right_point.t, t)
            s_lower: float = lerp(pt_obstacle.bottom_left_point.s, pt_obstacle.bottom_left_point.t,
                                       pt_obstacle.bottom_right_point.s, pt_obstacle.bottom_right_point.t, t)
            intervals.append((s_lower, s_upper))
        return intervals

    def GetPathBlockingIntervals(self, t_start: float, t_end: float,
                                 t_resolution: float) -> List[List[Tuple[float, float]]]:
        """
        Get the path blocking intervals.

        :param float t_start: the start time value
        :param float t_end: the end time value
        :param float t_resolution: the time resolution
        :returns: the path blocking intervals
        :rtype: List[List[Tuple[float, float]]]
        """
        intervals: List[List[Tuple[float, float]]] = []
        t = t_start
        while t <= t_end:
            intervals.append(self.GetPathBlockingIntervalsT(t))
            t += t_resolution
        return intervals

    def get_path_range(self) -> Tuple[float, float]:
        """
        Get the path range, from s_start to s_end

        :returns: the path range
        :rtype: Tuple[float, float]
        """
        return self._path_range

    def get_time_range(self) -> Tuple[float, float]:
        """
        Get the time range, from t_start to t_end

        :returns: the time range
        :rtype: Tuple[float, float]
        """
        return self._time_range

    def GetObstacleSurroundingPoints(self, obstacle_id: str, s_dist: float, t_min_density: float) -> List[STPoint]:
        """
        Get the surrounding points of the obstacle.

        :param str obstacle_id: the obstacle ID
        :param float s_dist: the s distance
        :param float t_min_density: the minimum time density
        :returns: the surrounding points of the obstacle
        :rtype: List[STPoint]
        """
        assert t_min_density > 0.0, "t_min_density must be greater than 0.0"
        pt_pairs: List[STPoint] = []
        if obstacle_id not in self.path_time_obstacle_map:
            return pt_pairs

        pt_obstacle = self.path_time_obstacle_map[obstacle_id]

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

        assert time_gap > -config_module.FLAGS_numerical_epsilon, "Time gap is negative"
        time_gap = abs(time_gap)

        num_sections: int = int(time_gap / t_min_density) + 1
        t_interval: float = time_gap / num_sections

        for i in range(num_sections + 1):
            t = t_interval * i + t0
            s = lerp(s0, t0, s1, t1, t) + s_dist

            ptt = STPoint(s, t)
            pt_pairs.append(ptt)

        return pt_pairs

    def IsObstacleInGraph(self, obstacle_id: str) -> bool:
        """
        Check if the obstacle is in the graph.

        :param str obstacle_id: the obstacle ID
        :returns: True if the obstacle is in the graph, False otherwise
        :rtype: bool
        """
        return obstacle_id in self.path_time_obstacle_map

    def GetLateralBounds(self, s_start: float, s_end: float, s_resolution: float) -> List[Tuple[float, float]]:
        """
        Get the lateral bounds.

        :param float s_start: the start s value
        :param float s_end: the end s value
        :param float s_resolution: the s resolution
        :returns: the lateral bounds
        :rtype: List[Tuple[float, float]]
        """
        assert s_start < s_end, "s_start must be less than s_end"
        assert s_resolution > config_module.FLAGS_numerical_epsilon, "s_resolution must be greater than numerical epsilon"
        bounds: List[Tuple[float, float]] = []
        discretized_path: List[float] = []
        s_range: float = s_end - s_start
        s_curr: float = s_start
        num_bound: int = int(s_range / s_resolution)

        ego_width: float = config_module.EGO_VEHICLE_WIDTH

        # Initialize bounds by reference line width
        for _ in range(num_bound):
            left_width, right_width = self._half_widths(s_curr)
            ego_d_lower: float = self._init_d[0] - ego_width / 2.0
            ego_d_upper: float = self._init_d[0] + ego_width / 2.0
            bounds.append((min(-right_width, ego_d_lower - config_module.FLAGS_bound_buffer),
                           max(left_width, ego_d_upper + config_module.FLAGS_bound_buffer)))
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

        :param SLBoundary sl_boundary: the s-l boundary of the obstacle
        :param List[float] discretized_path: the discretized path
        :param float s_start: the start s value
        :param float s_end: the end s value
        :param List[Tuple[float, float]] bounds: the lateral bounds
        """
        if sl_boundary.start_s > s_end or sl_boundary.end_s < s_start:
            return
        start_index: int = bisect_left(discretized_path, sl_boundary.start_s)
        end_index: int = bisect_right(discretized_path, sl_boundary.start_s)
        if sl_boundary.end_l > -config_module.FLAGS_numerical_epsilon and sl_boundary.start_l < config_module.FLAGS_numerical_epsilon:
            for i in range(start_index, end_index):
                bounds[i] = (-config_module.FLAGS_numerical_epsilon, config_module.FLAGS_numerical_epsilon)
            return
        if sl_boundary.end_l < config_module.FLAGS_numerical_epsilon:
            for i in range(start_index, min(end_index + 1, len(bounds))):
                bounds[i] = (max(bounds[i][0], sl_boundary.end_l + config_module.FLAGS_nudge_buffer), bounds[i][1])
            return
        if sl_boundary.start_l > -config_module.FLAGS_numerical_epsilon:
            for i in range(start_index, min(end_index + 1, len(bounds))):
                bounds[i] = (bounds[i][0], min(bounds[i][1], sl_boundary.start_l - config_module.FLAGS_nudge_buffer))
            return


class _STRegion:
    """The four corners the sampler interpolates. Not a polygon."""

    def __init__(self, region_id: str):
        self._id = region_id
        self.upper_left_point = None
        self.upper_right_point = None
        self.bottom_left_point = None
        self.bottom_right_point = None
        self._min_t = float("inf")
        self._max_t = float("-inf")

    def id(self) -> str:
        return self._id

    def _note(self, point: STPoint) -> None:
        self._min_t = min(self._min_t, point.t)
        self._max_t = max(self._max_t, point.t)

    def set_upper_left_point(self, point: STPoint) -> None:
        self.upper_left_point = point
        self._note(point)

    def set_upper_right_point(self, point: STPoint) -> None:
        self.upper_right_point = point
        self._note(point)

    def set_bottom_left_point(self, point: STPoint) -> None:
        self.bottom_left_point = point
        self._note(point)

    def set_bottom_right_point(self, point: STPoint) -> None:
        self.bottom_right_point = point
        self._note(point)

    @property
    def min_t(self) -> float:
        return self._min_t

    @property
    def max_t(self) -> float:
        return self._max_t
