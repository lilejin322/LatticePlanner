"""
Prediction querier submodule
"""
from typing import List
from logging import Logger
from math import cos, sin
from path_matcher import PathMatcher
from protoclass.path_point import PathPoint
from lattice_motion import is_moving, state_in_horizon

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
            if obstacle.obstacle_id not in self.id_obstacle_map:
                self.id_obstacle_map[obstacle.obstacle_id] = obstacle
                self.obstacles.append(obstacle)
            else:
                self.logger.warning(f"Duplicated obstacle found [{obstacle.obstacle_id}]")

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
        obstacle = self.id_obstacle_map[obstacle_id]
        if not is_moving(obstacle):
            return 0.0
        pose = state_in_horizon(obstacle, t)
        if pose is None:
            return 0.0
        v = pose.velocity
        theta = pose.orientation
        v_x: float = v * cos(theta)
        v_y: float = v * sin(theta)

        obstacle_point_on_ref_line = PathMatcher.MatchToPathS(self.reference_line, s)

        ref_theta = obstacle_point_on_ref_line.theta

        return cos(ref_theta) * v_x + sin(ref_theta) * v_y
