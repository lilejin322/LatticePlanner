"""
Prediction querier submodule
"""
from typing import List
from logging import Logger
from math import cos, sin
from path_matcher import PathMatcher
from protoclass.path_point import PathPoint

class PredictionQuerier:
    """
    PredictionQuerier is a class that provides an interface to query obstacle prediction results.
    """

    def __init__(self, obstacles, reference_line: List[PathPoint]) -> None:
        """
        Constructor

        :param obstacles: a list of obstacles
        :param reference_line: a list of path points that represent the reference line
        """

        self.reference_line = reference_line
        self.id_obstacle_map = {}
        self.obstacles = []
        self.logger = Logger("PredictionQuerier")

        for obstacle in obstacles:
            if obstacle.id not in self.id_obstacle_map:
                self.id_obstacle_map[obstacle.id] = obstacle
                self.obstacles.append(obstacle)
            else:
                self.logger.warning(f"Duplicated obstacle found [{obstacle.id}]")

    def GetObstacles(self):
        """
        Get list of obstacles.

        :returns: a list of obstacles
        :rtype: List[Obstacle]
        """
        return self.obstacles

    def ProjectVelocityAlongReferenceLine(self, obstacle_id: str, s: float, t: float) -> float:
        """
        Project velocity of an obstacle along the reference line.

        :param obstacle_id: id of the obstacle
        :param s: longitudinal distance along the reference line
        :param t: time to query the velocity
        :returns: velocity of the obstacle along the reference line
        :rtype: float
        """
        assert obstacle_id in self.id_obstacle_map, f"Obstacle with id {obstacle_id} not found"
        mover = self.id_obstacle_map[obstacle_id]
        if not mover.moving():
            return 0.0
        if t < mover.poses[0].t or t > mover.poses[-1].t:
            return 0.0
        pose = mover.pose_at(t)
        v = pose.v
        theta = pose.theta
        v_x: float = v * cos(theta)
        v_y: float = v * sin(theta)

        obstacle_point_on_ref_line = PathMatcher.MatchToPathS(self.reference_line, s)

        ref_theta = obstacle_point_on_ref_line.theta

        return cos(ref_theta) * v_x + sin(ref_theta) * v_y
