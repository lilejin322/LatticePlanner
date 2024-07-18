from protoclass.TrajectoryPoint import TrajectoryPoint
from typing import List, Tuple
from protoclass.DecisionResult import ObjectDecisionType, ObjectIgnore, ObjectStop, ObjectFollow, \
                                      ObjectYield, ObjectOvertake, ObjectNudge, ObjectAvoid, ObjectSidePass
from common.SLBoundary import SLBoundary
from common.STBoundary import STBoundary
from common.Vec2d import Vec2d
import math
from bisect import bisect_left
from logging import Logger
import hashlib
import struct
from config import EGO_VEHICLE_WIDTH, FRONT_EDGE_TO_CENTER

logger = Logger("Obstacle")
kStBoundaryDeltaS: float = 0.2;        # meters
kStBoundarySparseDeltaS: float = 1.0   # meters
kStBoundaryDeltaT: float = 0.05        # seconds

class BoundaryType:
    pass

class PerceptionObstacle:
    pass

class Priority:
    pass

class Polygon2d:
    pass

class Trajectory:
    pass

class PredictionObstacles:
    pass

class Box2d:
    pass

class Obstacle:
    """
    class Obstacle
    This is the class that associates an Obstacle with its path
    properties. An obstacle's path properties relative to a path.
    The `s` and `l` values are examples of path properties.
    The decision of an obstacle is also associated with a path.
 
    The decisions have two categories: lateral decision and longitudinal
    decision.
    Lateral decision includes: nudge, ignore.
    Lateral decision safety priority: nudge > ignore.
    Longitudinal decision includes: stop, yield, follow, overtake, ignore.
    Decision safety priorities order: stop > yield >= follow > overtake > ignore
 
    Ignore decision belongs to both lateral decision and longitudinal decision,
    and it has the lowest priority.
    """

    s_longitudinal_decision_safety_sorter = {ObjectIgnore: 0,
                                             ObjectOvertake: 100,
                                             ObjectFollow: 300,
                                             ObjectYield: 400,
                                             ObjectStop: 500}

    s_lateral_decision_safety_sorter = {ObjectIgnore: 0, ObjectNudge: 100}

    def __init__(self, id: str = "", 
                 perception_obstacle: PerceptionObstacle=None, 
                 obstacle_priority: Priority=None, 
                 is_static: bool = False,
                 trajectory: Trajectory=None):
        """
        Constructor
        """

        self._id: str = id
        self._perception_obstacle: PerceptionObstacle = perception_obstacle
        self._obstacle_priority: Priority = obstacle_priority
        self._perception_id: int = perception_obstacle.id
        self._perception_bounding_box: Box2d = Box2d([perception_obstacle.position.x,
                                                     perception_obstacle.position.y],
                                                     perception_obstacle.theta,
                                                     perception_obstacle.length,
                                                     perception_obstacle.width)
        self._perception_polygon: Polygon2d = None
        self._decisions: List[ObjectDecisionType] = []
        self._decider_tags: List[str] = []
        self._sl_boundary: SLBoundary = None
        self._reference_line_st_boundary: STBoundary = None
        self._path_st_boundary: STBoundary = None
        self._lateral_decision: ObjectDecisionType = None
        self._longitudinal_decision: ObjectDecisionType = None
        # for keep_clear usage only
        self._is_blocking_obstacle: bool = False
        self._is_lane_blocking: bool = False
        self._is_lane_change_blocking: bool = False
        self._is_caution_level_obstacle: bool = (obstacle_priority == ObstaclePriority.CAUTION)
        self._min_radius_stop_distance: float = -1.0
        self._path_st_boundary_initialized: float = False
        self.polygon_points: List[Vec2d] = []
        if FLAGS_use_navigation_mode or perception_obstacle.polygon_point_size <= 2:
            self._perception_bounding_box.GetAllCorners(self.polygon_points)
        else:
            assert perception_obstacle.polygon_point_size > 2, f"object {self._id} has less than 3 polygon points"
            for point in perception_obstacle.polygon_point:
                self.polygon_points.append(Vec2d(point.x, point.y))
        assert Polygon2d.ComputeConvexHull(self.polygon_points, self._perception_polygon), f"object {self.id} polygon is not a valid convex hull.\n {perception_obstacle}"
        self._is_static: bool = is_static or (obstacle_priority == ObstaclePriority.IGNORE)
        self._is_virtual: bool = perception_obstacle.id < 0
        self._speed: float = math.hypot(perception_obstacle.velocity.x, perception_obstacle.velocity.y)
        self._trajectory: list = trajectory if trajectory is not None else []
        if trajectory is not None:
            trajectory_points = trajectory.mutable_trajectory.point
            cumulative_s: float = 0.0
            if len(trajectory_points) > 0:
                trajectory_points[0].mutable_path_point.set_s(0.0)
            for i in range(1, len(trajectory_points)):
                prev = trajectory_points[i - 1]
                cur = trajectory_points[i]
                if prev.relative_time >= cur.relative_time:
                    raise ValueError(f"prediction time is not increasing. current point: {cur} previous point: {prev}")
                cumulative_s += DistanceXY(prev.path_point, cur.path_point)
                trajectory_points[i].mutable_path_point.set_s(cumulative_s)

    @property
    def Id(self) -> str:
        """
        Getter for id

        :returns: Obstacle id
        :rtype: str
        """

        return self._id

    def SetId(self, id: str) -> None:
        """
        Setter for id

        :param str id: Obstacle id
        """

        self._id = id

    @property
    def speed(self) -> float:
        """
        Getter for speed

        :returns: Obstacle speed
        :rtype: float
        """

        return self._speed

    def PerceptionId(self) -> int:
        """
        Getter for perception id

        :returns: Perception id
        :rtype: int
        """

        return self._perception_id

    @property
    def IsStatic(self) -> bool:
        """
        Getter for is_static

        :returns: True if obstacle is static, False otherwise
        :rtype: bool
        """

        return self._is_static

    @property
    def IsVirtual(self) -> bool:
        """
        Getter for is_virtual

        :returns: True if obstacle is virtual, False otherwise
        :rtype: bool
        """

        return self._is_virtual

    def GetPointAtTime(self, time: float) -> TrajectoryPoint:
        """
        Get the point at a given time

        :param float time: Time
        :returns: Trajectory point at the given time
        :rtype: TrajectoryPoint
        """

        points: list = self._trajectory.trajectory_point
        if len(points) < 2:
            point = TrajectoryPoint()
            point.mutable_path_point.set_x(self._perception_obstacle.position.x)
            point.muyable_path_point.set_y(self._perception_obstacle.position.y)
            point.mutable_path_point.set_z(self._perception_obstacle.position.z)
            point.mutable_path_point.set_theta(self._perception_obstacle.theta)
            point.mutable_path_point.set_s(0.0)
            point.mutable_path_point.set_kappa(0.0)
            point.mutable_path_point.set_dkappa(0.0)
            point.mutable_path_point.set_ddkappa(0.0)
            point.set_v(0.0)
            point.set_a(0.0)
            point.set_relative_time(0.0)
            return point
        else:
            times = [point.relative_time for point in points]
            index = bisect_left(times, relative_time)
            if index == 0:
                return points[0]
            elif index == len(points):
                return points[-1]
            else:
                return InterpolateUsingLinearApproximation(points[index - 1], points[index], relative_time)

    def GetBoundingBox(self, point: TrajectoryPoint) -> Box2d:
        """
        Get the bounding box at a given point

        :param TrajectoryPoint point: Trajectory point
        :returns: Bounding box
        :rtype: Box2d
        """

        return Box2d([point.path_point.x, point.path_point.y], point.path_point.theta,
                     self._perception_obstacle.length, self._perception_obstacle.width)

    @property
    def PerceptionBoundingBox(self) -> Box2d:
        """
        Getter for perception bounding box

        :returns: Perception bounding box
        """

        return self._perception_bounding_box

    @property
    def PerceptionPolygon(self) -> Polygon2d:
        """
        Getter for perception polygon

        :returns: Perception polygon
        :rtype: Polygon2d
        """

        return self._perception_polygon

    @property
    def Trajectory(self) -> Trajectory:
        """
        Getter for trajectory

        :returns: Obstacle trajectory
        :rtype: Trajectory
        """

        return self._trajectory

    @property
    def HasTrajectory(self) -> bool:
        """
        Check if obstacle has trajectory

        :returns: True if obstacle has trajectory, False otherwise
        :rtype: bool
        """

        return bool(self._trajectory)

    @property
    def Perception(self) -> PerceptionObstacle:
        """
        Getter for perception obstacle

        :returns: Perception obstacle
        :rtype: PerceptionObstacle
        """

        return self._perception_obstacle

    @staticmethod
    def CreateObstacles(predictions: PredictionObstacles) -> List['Obstacle']:
        """
        This is a helper function that can create obstacles from prediction
        data.  The original prediction may have multiple trajectories for each
        obstacle. But this function will create one obstacle for each trajectory.

        :param PredictionObstacles predictions: The prediction results
        :returns: obstacles The output obstacles saved in a list of unique_ptr.
        :rtype: List[Obstacle]
        """

        obstacles: List[Obstacle] = []
        for prediction_obstacle in predictions.prediction_obstacle:
            if not Obstacle.IsValidObstacle(prediction_obstacle.perception_obstacle):
                logger.error(f"Invalid perception obstacle: {prediction_obstacle.perception_obstacle}")
                continue
            perception_id = str(prediction_obstacle.perception_obstacle.id)
            if not prediction_obstacle.trajectory:
                obstacles.append(Obstacle(perception_id, prediction_obstacle.perception_obstacle,
                                          prediction_obstacle.priority.priority, prediction_obstacle.is_static))
                continue
            trajectory_index = 0
            for trajectory in prediction_obstacle.trajectory:
                is_valid_trajectory: bool = True
                for point in trajectory.trajectory_point:
                    if not Obstacle.IsValidTrajectoryPoint(point):
                        logger.error(f"obj:{perception_id} TrajectoryPoint: {trajectory} is NOT valid.")
                        is_valid_trajectory = False
                        break
                if not is_valid_trajectory:
                    continue
                obstacle_id: str = f"{perception_id}_{trajectory_index}"
                obstacles.append(Obstacle(obstacle_id, prediction_obstacle.perception_obstacle,
                                          trajectory, prediction_obstacle.priority.priority,
                                          prediction_obstacle.is_static))
                trajectory_index += 1
        return obstacles

    @staticmethod
    def CreateStaticVirtualObstacles(id: str, obstacle_box: Box2d) -> 'Obstacle':
        """
        Create static virtual obstacles

        :param str id: Obstacle id
        :param Box2d obstacle_box: Obstacle bounding box
        :returns: the generated Obstacle
        :rtype: Obstacle
        """

        # create a "virtual" perception_obstacle
        perception_obstacle = PerceptionObstacle()
        # simulator needs a valid integer
        negative_id = int(hashlib.md5(id.encode()).hexdigest(), 16)
        # set the first bit to 1 so negative_id became negative number
        negative_id |= (0x1 << 31)
        negative_id_signed = struct.unpack('i', struct.pack('I', negative_id & 0xFFFFFFFF))[0]
        perception_obstacle.set_id(negative_id_signed)
        perception_obstacle.mutable_position.set_x(obstacle_box.center.x)
        perception_obstacle.mutable_position.set_y(obstacle_box.center.y)
        perception_obstacle.set_theta(obstacle_box.heading)
        perception_obstacle.mutable_velocity.set_x(0)
        perception_obstacle.mutable_velocity.set_y(0)
        perception_obstacle.set_length(obstacle_box.length)
        perception_obstacle.set_width(obstacle_box.width)
        perception_obstacle.set_height(FLAGS_virtual_stop_wall_height)
        perception_obstacle.set_type(PerceptionObstacle.UNKNOWN_UNMOVABLE)
        perception_obstacle.set_tracking_time(1.0)

        corner_points: List[Vec2d] = []
        obstacle_box.GetAllCorners(corner_points)
        for corner_point in corner_points:
            point = perception_obstacle.add_polygon_point()
            point.set_x(corner_point.x)
            point.set_y(corner_point.y)
        obstacle = Obstacle(id, perception_obstacle, ObstaclePriority.NORMAL, True)
        obstacle._is_virtual = True
        return obstacle

    @staticmethod
    def IsValidPerceptionObstacle(obstacle: PerceptionObstacle) -> bool:
        """
        Check if perception obstacle is valid

        :param PerceptionObstacle obstacle: Perception obstacle
        :returns: True if perception obstacle is valid, False otherwise
        :rtype: bool
        """

        if obstacle.length <= 0.0:
            logger.error(f"invalid obstacle length: {obstacle.length}")
            return False
        if obstacle.width <= 0.0:
            logger.error(f"invalid obstacle width: {obstacle.width}")
            return False
        if obstacle.height <= 0.0:
            logger.error(f"invalid obstacle height: {obstacle.height}")
            return False
        if obstacle.has_velocity:
            if math.isnan(obstacle.velocity.x) or math.isnan(obstacle.velocity.y):
                logger.error(f"invalid obstacle velocity: {obstacle.velocity}")
                return False
        for pt in obstacle.polygon_point:
            if math.isnan(pt.x) or math.isnan(pt.y):
                logger.error(f"invalid obstacle polygon point: {pt}")
                return False
        return True

    @staticmethod
    def IsValidTrajectoryPoint(point: TrajectoryPoint) -> bool:
        """
        Check if trajectory point is valid

        :param TrajectoryPoint point: Trajectory point
        :returns: True if trajectory point is valid, False otherwise
        :rtype: bool
        """

        if not point.has_path_point:
            return False
        if math.isnan(point.path_point.x) or math.isnan(point.path_point.y) or math.isnan(point.path_point.z):
            return False
        if math.isnan(point.path_point.kappa) or math.isnan(point.path_point.s):
            return False
        if math.isnan(point.path_point.dkappa) or math.isnan(point.path_point.ddkappa):
            return False
        if math.isnan(point.v) or math.isnan(point.a) or math.isnan(point.relative_time):
            return False
        return True

    @property
    def IsCautionLevelObstacle(self) -> bool:
        """
        Getter for is_caution_level_obstacle

        :returns: True if obstacle is caution level, False otherwise
        :rtype: bool
        """

        return self._is_caution_level_obstacle

    def LateralDecision(self) -> ObjectDecisionType:
        """
        Lateral decision is one of {Nudge, Ignore}

        :returns: the merged lateral decision
        :rtype: ObjectDecisionType
        """

        raise NotImplementedError

    def LongitudinalDecision(self) -> ObjectDecisionType:
        """
        Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}

        :returns: the merged longitudinal decision
        :rtype: ObjectDecisionType
        """

        raise NotImplementedError

    def DebugString(self) -> str:
        """
        Debug string representation

        :returns: Debug string representation
        :rtype: str
        """

        raise NotImplementedError

    def PrintPolygonCurve(self) -> None:
        """
        Print polygon curve
        """

        raise NotImplementedError

    def PerceptionSLBoundary(self) -> SLBoundary:
        """

        """

        raise NotImplementedError

    def reference_line_st_boundary(self) -> STBoundary:
        """

        """

        raise NotImplementedError

    def path_st_boundary(self) -> STBoundary:
        """

        """
        
        raise NotImplementedError

    def decider_tags(self) -> List[str]:
        """

        """
        
        raise NotImplementedError

    def decisions(self) -> ObjectDecisionType:
        """

        """

        raise NotImplementedError

    def AddLongitudinalDecision(self, decider_tag: str, decision: ObjectDecisionType) -> None:
        """
        Add a longitudinal decision
        """

        raise NotImplementedError

    def AddLateralDecision(self, decider_tag: str, decision: ObjectDecisionType) -> None:
        """
        Add a lateral decision
        """

        raise NotImplementedError

    def HasLateralDecision(self) -> bool:
        """
        Check if obstacle has lateral decision

        :returns: True if obstacle has lateral decision, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def set_path_st_boundary(self, boundary: STBoundary) -> None:
        """
        Set path ST boundary

        :param STBoundary boundary: Path ST boundary
        """

        raise NotImplementedError

    @property
    def is_path_st_boundary_initialized(self) -> bool:
        """
        Check if path ST boundary is initialized

        :returns: True if path ST boundary is initialized, False otherwise
        :rtype: bool
        """

        return self._path_st_boundary_initialized

    def SetStBoundaryType(self, type: BoundaryType) -> None:
        """
        Set ST boundary type

        :param BoundaryType type: ST boundary type
        """

        raise NotImplementedError

    def EraseStBoundary(self) -> None:
        """
        Erase ST boundary
        """

        raise NotImplementedError

    def EraseDecision(self) -> None:
        """
        Erase decision
        """

        raise NotImplementedError

    def SetReferenceLineStBoundary(self, boundary: STBoundary) -> None:
        """
        Set reference line ST boundary

        :param STBoundary boundary: Reference line ST boundary
        """

        raise NotImplementedError

    def SetReferenceLineStBoundaryType(self, type: BoundaryType) -> None:
        """
        Set reference line ST boundary type

        :param BoundaryType type: Reference line ST boundary type
        """

        raise NotImplementedError

    def EraseReferenceLineStBoundary(self) -> None:
        """
        Erase reference line ST boundary
        """

        raise NotImplementedError

    @property
    def HasLongitudinalDecision(self) -> bool:
        """
        Check if obstacle has longitudinal decision

        :returns: True if obstacle has longitudinal decision, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def HasNonIgnoreDecision(self) -> bool:
        """
        Check if obstacle has non-ignore decision

        :returns: True if obstacle has non-ignore decision, False otherwise
        :rtype: bool
        """
        
        raise NotImplementedError

    def MinRadiusStopDistance(self) -> float:
        """
        Calculate stop distance with the obstacle using the ADC's minimum
        turning radius

        :param VehicleParam vehicle_param: Vehicle parameters
        :returns: Minimum radius stop distance
        :rtype: float
        """

        if self._min_radius_stop_distance > 0:
            return self._min_radius_stop_distance
        stop_distance_buffer: float = 0.5
        min_turn_radius: float = MinSafeTurnRadius
        latteral_diff: float = EGO_VEHICLE_WIDTH / 2.0 + max(abs(self._sl_boundary.start_l), abs(self._sl_boundary.end_l))
        kEpison: float = 1e-5
        latteral_diff: float = min(latteral_diff, min_turn_radius - kEpison)
        stop_distance: float = math.sqrt(abs(min_turn_radius**2 - (min_turn_radius - latteral_diff)**2)) + stop_distance_buffer
        stop_distance -= FRONT_EDGE_TO_CENTER
        stop_distance = min(stop_distance, FLAGS_max_stop_distance_obstacle)
        stop_distance = max(stop_distance, FLAGS_min_stop_distance_obstacle)
        return stop_distance

    def IsIgnore(self) -> bool:
        """
        Check if this object can be safely ignored.
        The object will be ignored if the lateral decision is ignore and the
        longitudinal decision is ignore
        
        returns: longitudinal_decision_ == ignore && lateral_decision == ignore.
        rtype: bool
        """

        raise NotImplementedError

    def IsLongitudinalIgnore(self) -> bool:
        """
        Check if this object can be safely ignored in longitudinal decision

        :returns: True if object can be safely ignored in longitudinal decision,
                  False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def IsLateralIgnore(self) -> bool:
        """
        Check if this object can be safely ignored in lateral decision

        :returns: True if object can be safely ignored in lateral decision,
                  False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def BuildReferenceLineStBoundary(self, reference_line: ReferenceLine, adc_start_s: float) -> None:
        """
        Build reference line ST boundary
        """

        half_adc_width: float = EGO_VEHICLE_WIDTH / 2
        if self._is_static or not self._trajectory.trajectory_point:
            point_pairs: List[Tuple[STPoint, STPoint]] = []
            start_s: float = self._sl_boundary.start_s
            end_s: float = self._sl_boundary.end_s
            if end_s - start_s < kStBoundaryDeltaS:
                end_s = start_s + kStBoundaryDeltaS
            if not reference_line.IsBlockRoad(self._perception_bounding_box, half_adc_width):
                return
            point_pairs.append((STPoint(start_s - adc_start_s, 0.0), STPoint(end_s - adc_start_s, 0.0)))
            point_pairs.append((STPoint(start_s - adc_start_s, FLAGS_st_max_t), STPoint(end_s - adc_start_s, FLAGS_st_max_t)))
            self._reference_line_st_boundary = STBoundary(point_pairs)
        else:
            if self.BuildTrajectoryStBoundary(reference_line, adc_start_s, self._reference_line_st_boundary):
                logger.debug(f"Found st_boundary for obstacle {self._id}")
                logger.debug(f"st_boundary: min_t = {self._reference_line_st_boundary.min_t}, max_t = {self._reference_line_st_boundary.max_t}, min_s = {self._reference_line_st_boundary.min_s}, max_s = {self._reference_line_st_boundary.max_s}")
            else:
                logger.debug(f"No st_boundary for obstacle {self._id}")

    def SetPerceptionSlBoundary(self, sl_boundary: SLBoundary) -> None:
        """
        Set perception SL boundary

        :param SLBoundary sl_boundary: Perception SL boundary
        """
        
        self._sl_boundary = sl_boundary

    @staticmethod
    def IsLongitudinalDecision(decision: ObjectDecisionType) -> bool:
        """
        check if an ObjectDecisionType is a longitudinal decision.

        :param ObjectDecisionType decision: Object decision type
        :returns: True if decision is longitudinal, False otherwise
        :rtype: bool
        """
        
        raise NotImplementedError

    @staticmethod
    def IsLateralDecision(decision: ObjectDecisionType) -> bool:
        """
        check if an ObjectDecisionType is a lateral decision.

        :param ObjectDecisionType decision: Object decision type
        :returns: True if decision is lateral, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def SetBlockingObstacle(self, blocking: bool) -> None:
        """
        Set blocking obstacle

        :param bool blocking: Blocking obstacle
        """

        self._is_blocking_obstacle = blocking

    @property
    def IsBlockingObstacle(self) -> bool:
        """
        Getter for is_blocking_obstacle

        :returns: True if obstacle is blocking, False otherwise
        :rtype: bool
        """

        return self._is_blocking_obstacle

    def IsLaneBlocking(self) -> bool:
        """
        IsLaneBlocking is only meaningful when IsStatic() == true.

        :returns: True if obstacle is lane blocking, False otherwise
        :rtype: bool
        """

        return self._is_lane_blocking

    def CheckLaneBlocking(self, reference_line: ReferenceLine) -> None:
        """
        Check lane blocking

        :param ReferenceLine reference_line: Reference line
        """
    
        raise NotImplementedError

    def IsLaneChangeBlocking(self) -> bool:
        """
        Getter for is_lane_change_blocking

        :returns: True if obstacle is lane change blocking, False otherwise
        :rtype: bool
        """

        return self._is_lane_change_blocking

    def SetLaneChangeBlocking(self, is_distance_clear: bool) -> None:
        """
        Set lane change blocking

        :param bool is_distance_clear: Is distance clear
        """

        raise NotImplementedError

    def GetObstacleTrajectoryPolygon(self, point: TrajectoryPoint) -> Polygon2d:
        """
        Get obstacle trajectory polygon

        :param TrajectoryPoint point: Trajectory point
        :returns: Obstacle trajectory polygon
        :rtype: Polygon2d
        """

        raise NotImplementedError

    def BuildTrajectoryStBoundary(self, reference_line: ReferenceLine, adc_start_s: float) -> Tuple[bool, STBoundary]:
        """
        Build trajectory ST boundary

        :param ReferenceLine reference_line: Reference line
        :param float adc_start_s: ADC start s
        :returns: Tuple of success bool and the built trajectory ST boundary
        :rtype: Tuple[bool, STBoundary]
        """

        raise NotImplementedError
    
    def IsValidObstacle(self, perception_obstacle: PerceptionObstacle) -> bool:
        """
        Check if perception obstacle is valid

        :param PerceptionObstacle perception_obstacle: Perception obstacle
        :returns: True if perception obstacle is valid, False otherwise
        :rtype: bool
        """

        raise NotImplementedError
