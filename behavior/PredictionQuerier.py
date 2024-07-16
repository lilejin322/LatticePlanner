from typing import List, Dict
from common.Obstacle import Obstacle
from protoclass.PathPoint import PathPoint
from logging import Logger
from bisect import bisect
from math import cos, sin
from PathMatcher import PathMatcher

class PredictionQuerier:
    """
    PredictionQuerier is a class that provides an interface to query obstacle prediction results.
    """

    def __init__(self, obstacles: List[Obstacle], reference_line: List[PathPoint]):
        """
        Constructor

        :param obstacles: a list of obstacles
        :param reference_line: a list of path points that represent the reference line
        """

        self.reference_line = reference_line
        self.id_obstacle_map: Dict[str, Obstacle] = {}
        self.obstacles: List[Obstacle] = []
        self.logger = Logger("PredictionQuerier")

        for obstacle in obstacles:
            if obstacle.Id not in self.id_obstacle_map:
                self.id_obstacle_map[obstacle.Id] = obstacle
                self.obstacles.append(obstacle)
            else:
                self.logger.warning(f"Duplicated obstacle found [{obstacle.Id}]")

    def GetObstacles(self) -> List[Obstacle]:
        """
        Get list of obstacles

        :returns: a list of obstacles
        :rtype: List[Obstacle]
        """

        return self.obstacles

    def ProjectVelocityAlongReferenceLine(self, obstacle_id: str, s: float, t: float) -> float:
        """
        Project velocity of an obstacle along the reference line

        :param obstacle_id: id of the obstacle
        :param s: longitudinal distance along the reference line
        :param t: time to query the velocity
        :returns: velocity of the obstacle along the reference line
        :rtype: float
        """

        assert obstacle_id in self.id_obstacle_map, f"Obstacle with id {obstacle_id} not found"
        trajectory = self.id_obstacle_map[obstacle_id].Trajectory()
        num_traj_point = len(trajectory.trajectory_point)
        if num_traj_point < 2:
            return 0.0

        if t < trajectory.trajectory_point[0].relative_time or t > trajectory.trajectory_point[num_traj_point - 1].relative_time:
            return 0.0

        matched_index = bisect.bisect_left([p.relative_time for p in trajectory.trajectory_point], t)
        matched_it = trajectory.trajectory_point[matched_index]
        v: float = matched_it.v
        theta: float = matched_it.path_point.theta
        v_x: float = v * cos(theta)
        v_y: float = v * sin(theta)

        obstacle_point_on_ref_line = PathMatcher.MatchToPathS(self.reference_line, s)

        ref_theta = obstacle_point_on_ref_line.theta

        return cos(ref_theta) * v_x + sin(ref_theta) * v_y
