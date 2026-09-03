"""Utilities aligned with path_decider_obstacle_utils.cc."""

from __future__ import annotations

from common.obstacle import Obstacle
import config as config_module


def IsWithinPathDeciderScopeObstacle(obstacle: Obstacle) -> bool:
    if obstacle.IsVirtual():
        return False
    if (
        obstacle.HasLongitudinalDecision()
        and obstacle.HasLateralDecision()
        and obstacle.IsIgnore()
    ):
        return False
    if not obstacle.IsStatic() or (obstacle.speed or 0.0) > config_module.FLAGS_static_obstacle_speed_threshold:
        return False
    return True
