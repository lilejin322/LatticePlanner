from typing import List, Tuple, Dict
from collections import OrderedDict
from protoclass.DecisionResult import MainStop, ObjectDecisionType, ObjectStop
from common.NudgeInfo import NudgeInfo
from common.Obstacle import Obstacle
from protoclass.PerceptionObstacle import PerceptionObstacle
from common.STBoundary import STBoundary
from common.ReferenceLine import ReferenceLine
from protoclass.SLBoundary import SLBoundary
from logging import Logger
from protoclass.PointENU import PointENU
from config import FRONT_EDGE_TO_CENTER

logger = Logger("PathDecision")

class PathDecision:
    """
    PathDecision represents all obstacle decisions on one path.
    """

    def __init__(self):
        """
        Constructor
        """

        self._obstacles = OrderedDict()   # the cpp source code is IndexedList<string, Obstacle>
        self._main_stop: MainStop = None
        self._stop_reference_line_s: float = float('inf')
        # nudge info
        self._nudge_info: NudgeInfo = None

    def AddObstacle(self, obstacle: Obstacle) -> Obstacle:
        """
        Add an obstacle by id to dict

        :param Obstacle obstacle: the obstacle to add
        :returns: the obstacle added
        :rtype: Obstacle
        """

        self._obstacles[obstacle.Id] = obstacle
        return self._obstacles[obstacle.Id]

    @property
    def obstacles(self) -> OrderedDict:
        """
        Get all obstacles

        :returns: all obstacles
        :rtype: OrderedDict
        """

        return self._obstacles

    def AddLateralDecision(self, tag: str, object_id: str, decision: ObjectDecisionType) -> bool:
        """
        Add lateral decision

        :param str tag: the tag
        :param str object_id: the object id
        :param ObjectDecisionType decision: the decision
        :returns: True if added, False otherwise
        :rtype: bool
        """

        obstacle: Obstacle = self._obstacles.get(object_id)
        if obstacle is None:
            logger.error("failed to find obstacle")
            return False
        obstacle.AddLateralDecision(tag, decision)
        return True

    def AddLongitudinalDecision(self, tag: str, object_id: str, decision: ObjectDecisionType) -> bool:
        """
        Add longitudinal decision

        :param str tag: the tag
        :param str object_id: the object id
        :param ObjectDecisionType decision: the decision
        :returns: True if added, False otherwise
        :rtype: bool
        """

        obstacle: Obstacle = self._obstacles.get(object_id)
        if obstacle is None:
            logger.error("failed to find obstacle")
            return False
        obstacle.AddLongitudinalDecision(tag, decision)
        return True

    def Find(self, object_id: str) -> Obstacle:
        """
        Find an obstacle by id

        :param str object_id: the id of the obstacle
        :returns: the obstacle found
        :rtype: Obstacle
        """

        return self._obstacles[object_id]

    def FindPerceptionObstacle(self, perception_obstacle_id: str) -> PerceptionObstacle:
        """
        Find a perception obstacle by id

        :param str perception_obstacle_id: the id of the perception obstacle
        :returns: the perception obstacle found
        :rtype: PerceptionObstacle
        """

        for _, obstacle in self._obstacles.items():
            if str(obstacle.Perception.id) == perception_obstacle_id:
                return obstacle.Perception

        return None

    def SetSTBoundary(self, id: str, boundary: STBoundary) -> None:
        """
        Set the ST boundary

        :param str id: the id of the boundary
        :param STBoundary boundary: the boundary to set
        """

        obstacle: Obstacle = self._obstacles.get(id)
        if obstacle is None:
            logger.error(f"Failed to find obstacle : {id}")
            return
        else:
            obstacle.set_path_st_boundary(boundary)

    def EraseStBoundaries(self) -> None:
        """
        Erase ST boundaries
        """

        for _, obstacle in self._obstacles.items():
            obstacle.EraseStBoundary()

    @property
    def main_stop(self) -> MainStop:
        """
        Get the main stop decision

        :returns: the main stop decision
        :rtype: MainStop
        """

        return self._main_stop
    
    @property
    def stop_reference_line_s(self) -> float:
        """
        Get the stop reference line s

        :returns: the stop reference line s
        :rtype: float
        """

        return self._stop_reference_line_s

    def MergeWithMainStop(self, obj_stop: ObjectStop, obj_id: str, reference_line: ReferenceLine, adc_sl_boundary: SLBoundary) -> bool:
        """
        Merge with main stop
        """

        stop_point: PointENU = obj_stop.stop_point
        _, stop_line_sl = reference_line.XYToSL(stop_point)

        stop_line_s: float = stop_line_sl.s
        if stop_line_s < 0.0 or stop_line_s > reference_line.Length():
            logger.error(f"Ignore object: {obj_id} fence route_s[{stop_line_s}] not in range [0, {reference_line.Length()}]")
            return False
        
        # check stop_line_s vs adc_s, ignore if it is further way than main stop
        stop_line_s = max(stop_line_s, adc_sl_boundary.end_s - FRONT_EDGE_TO_CENTER)

        if stop_line_s >= self._stop_reference_line_s:
            logger.error(f"stop point is farther than current main stop point.")
            return False
        
        self._main_stop = MainStop(reason_code = obj_stop.reason_code,
                                   reason = f"stop by {obj_id}",
                                   stop_point = PointENU(x=obj_stop.stop_point.x, y=obj_stop.stop_point.y),
                                   stop_heading = obj_stop.stop_heading)
        self._stop_reference_line_s = stop_line_s
        logger.debug(f"main stop obstacle id: {obj_id}, stop_line_s: {stop_line_s}, stop_point: ({obj_stop.stop_point.x}, {obj_stop.stop_point.y}), stop_heading: {obj_stop.stop_heading}")
        return True

    @property
    def nudge_info(self) -> NudgeInfo:
        """
        Get the nudge info

        :returns: the nudge info
        :rtype: NudgeInfo
        """

        return self._nudge_info
