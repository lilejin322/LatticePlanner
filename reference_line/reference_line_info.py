import math
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from logging import Logger
from typing import List
import config as config_module
from common.box2d import Box2d
from common.discretized_trajectory import DiscretizedTrajectory
from common.obstacle import Obstacle
from common.path import AngleDiff
from common.path_data import PathData
from common.path_decision import PathDecision
from common.route_segments import RouteSegments
from common.speed_data import SpeedData
from common.vec2d import Vec2d
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.debug import Debug, PlanningData
from protoclass.decision_result import ObjectDecisionType, ObjectIgnore
from protoclass.lattice_structure import PlanningTarget
from protoclass.path_point import PathPoint
from protoclass.sl_boundary import SLBoundary
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState
from reference_line import ReferenceLine

logger = Logger("ReferenceLineInfo")


class ReferenceLineInfo:
    """
    ReferenceLineInfo holds all data for one reference line.
    """



    def __init__(self, vehicle_state: VehicleState=None, adc_planning_point: TrajectoryPoint=None, reference_line: ReferenceLine=None, segments: RouteSegments=None, share_map: bool=False):
        """
        Constructor

        ``share_map`` keeps the caller's reference line and route segments
        instead of deep-copying them. The closed-loop sim builds one road
        and reuses it every cycle; copying a sampled curve is most of the
        cycle cost. Callers that shrink the line or add speed limits must
        leave this off so those edits stay on their own copy.
        """

        self._vehicle_state = deepcopy(vehicle_state)
        self._adc_planning_point = deepcopy(adc_planning_point)
        if share_map:
            self._reference_line = reference_line
            self._lanes: RouteSegments = segments
        else:
            self._reference_line = deepcopy(reference_line)
            self._lanes = deepcopy(segments)
        # this is the number that measures the goodness of this reference line.
        # The lower the better.
        self._cost: float = 0.0
        self._is_drivable: bool = True
        self._path_decision: PathDecision = PathDecision()
        self._blocking_obstacle: Obstacle = None
        self._path_data: PathData = None
        self._speed_data: SpeedData = None
        self._discretized_trajectory: DiscretizedTrajectory = None
        # SL boundary of the planning start box relative to the reference line.
        self._adc_sl_boundary: SLBoundary = None
        self._is_on_reference_line: bool = False
        self._priority_cost: float = 0.0
        self._planning_target: PlanningTarget = PlanningTarget(cruise_speed=config_module.FLAGS_default_cruise_speed)
        self._trajectory_type = ADCTrajectory.TrajectoryType.UNKNOWN
        self._cruise_speed = 0.0
        self._base_cruise_speed = 0.0
        self._index: int = 0
        self._debug = Debug(planning_data=PlanningData(path=[]))

    def Init(self, obstacles: List[Obstacle], target_speed: float = None) -> bool:
        """

        returns: True if successful, False otherwise
        rtype: bool
        """

        if target_speed is None:
            target_speed = config_module.FLAGS_default_cruise_speed

        # stiching point
        path_point = self._adc_planning_point.path_point
        position: Vec2d = Vec2d(path_point.x, path_point.y)
        vec_to_center: Vec2d = Vec2d((config_module.FRONT_EDGE_TO_CENTER - config_module.BACK_EDGE_TO_CENTER) / 2.0, (config_module.LEFT_EDGE_TO_CENTER - config_module.RIGHT_EDGE_TO_CENTER) / 2.0)
        center: Vec2d = position + vec_to_center.rotate(path_point.theta)
        box: Box2d = Box2d(center, path_point.theta, config_module.EGO_VEHICLE_LENGTH, config_module.EGO_VEHICLE_WIDTH)
        self._adc_sl_boundary = SLBoundary()
        if not self._reference_line.GetSLBoundary(box, self._adc_sl_boundary):
            logger.error(f"Failed to get ADC boundary from box: {box}")
            return False

        if self._adc_sl_boundary.end_s < 0 or self._adc_sl_boundary.start_s > self._reference_line.Length():
            logger.warning(f"Vehicle SL {self._adc_sl_boundary} is not on reference line: [0, {self._reference_line.Length()}]")

        kOutOfReferenceLineL = 14.0;       # in meters
        if self._adc_sl_boundary.start_l > kOutOfReferenceLineL or self._adc_sl_boundary.end_l < -kOutOfReferenceLineL:
            logger.error(f"Ego vehicle is too far away from reference line. self._adc_sl_boundary.start_l: {self._adc_sl_boundary.start_l}, self._adc_sl_boundary.end_l: {self._adc_sl_boundary.end_l}")
            return False
        self._is_on_reference_line = self._reference_line.IsOnLane(self._adc_sl_boundary)
        if not self.AddObstacles(obstacles):
            logger.error(f"Failed to add obstacles to reference line")
            return False

        map_path = self._reference_line.map_path
        for speed_bump in getattr(map_path, "speed_bump_overlaps", []):
            # -1 and + 1.0 are added to make sure it can be sampled.
            self._reference_line.AddSpeedLimit(speed_bump.start_s - 1.0, speed_bump.end_s + 1.0, config_module.FLAGS_speed_bump_speed_limit)

        self.SetCruiseSpeed(target_speed)
        self.SetLatticeCruiseSpeed(target_speed)
        return True

    def AddObstacles(self, obstacles: List[Obstacle]) -> bool:
        """
        Add obstacles to the reference line info

        param List[Obstacle] obstacles: List of obstacles to add
        returns: True if successful, False otherwise
        rtype: bool
        """

        if isinstance(obstacles, dict):
            obstacles = list(obstacles.values())

        if config_module.FLAGS_use_multi_thread_to_add_obstacles:
            with ThreadPoolExecutor() as executor:
                futures = {executor.submit(self.AddObstacle, obstacle): obstacle for obstacle in obstacles}
                for future in as_completed(futures):
                    if not future.result():
                        logger.error("Fail to add obstacles.")
                        return False
        else:
            for obstacle in obstacles:
                if not self.AddObstacle(obstacle):
                    logger.error(f"Failed to add obstacle {obstacle.Id()}.")
                    return False

        return True

    def AddObstacle(self, obstacle: Obstacle) -> Obstacle:
        """
        Add an obstacle to the reference line info
        AddObstacle is thread safe        

        param Obstacle obstacle: Obstacle to add
        returns: The added obstacle
        rtype: Obstacle
        """

        if obstacle is None:
            logger.error("The provided obstacle is empty")
            return None
        mutable_obstacle: Obstacle = self.path_decision.AddObstacle(obstacle)
        if not mutable_obstacle:
            logger.error(f"Failed to add obstacle {obstacle.Id()}")
            return None

        perception_sl = SLBoundary()
        tag = self._reference_line.GetSLBoundary(obstacle.PerceptionBoundingBox(), perception_sl)
        if not tag:
            logger.error(f"Failed to get SL boundary for obstacle {obstacle.Id()}")
            return mutable_obstacle
        mutable_obstacle.SetPerceptionSlBoundary(perception_sl)
        mutable_obstacle.CheckLaneBlocking(self._reference_line)
        if mutable_obstacle.IsLaneBlocking():
            logger.debug(f"Obstacle {obstacle.Id()} is lane blocking.")
        else:
            logger.debug(f"Obstacle {obstacle.Id()} is NOT lane blocking.")

        if self.IsIrrelevantObstacle(mutable_obstacle):
            ignore = ObjectDecisionType()
            ignore.object_tag = ObjectIgnore()
            self.path_decision.AddLateralDecision("reference_line_filter", obstacle.Id(), ignore)
            self.path_decision.AddLongitudinalDecision("reference_line_filter", obstacle.Id(), ignore)
            logger.debug(f"NO build reference line st boundary. id: {obstacle.Id()}")
        else:
            logger.debug(f"Build reference line st boundary. id: {obstacle.Id()}")
            mutable_obstacle.BuildReferenceLineStBoundary(self._reference_line, self._adc_sl_boundary.start_s)
            st_boundary = mutable_obstacle.reference_line_st_boundary()
            if st_boundary and not st_boundary.IsEmpty():
                logger.debug(f"Reference line st boundary: t[{st_boundary.min_t}, {st_boundary.max_t}] s[{st_boundary.min_s}, {st_boundary.max_s}]")

        return mutable_obstacle

    @property
    def vehicle_state(self) -> VehicleState:
        """
        Get the vehicle state

        returns: The vehicle state
        rtype: VehicleState
        """

        return self._vehicle_state

    @property
    def path_decision(self) -> PathDecision:
        """
        Get the path decision

        returns: The path decision
        rtype: PathDecision
        """

        return self._path_decision

    @property
    def reference_line(self) -> ReferenceLine:
        """
        Get the reference line

        returns: The reference line
        rtype: ReferenceLine
        """

        return self._reference_line



    def SetTrajectory(self, trajectory: DiscretizedTrajectory) -> None:
        """
        Set the trajectory

        :param DiscretizedTrajectory trajectory: The trajectory to set
        """

        self._discretized_trajectory = trajectory

    @property
    def trajectory(self) -> DiscretizedTrajectory:
        """
        Get the trajectory

        returns: The trajectory
        rtype: DiscretizedTrajectory
        """

        return self._discretized_trajectory

    def Cost(self) -> float:
        """
        """

        return self._cost

    def AddCost(self, cost: float) -> None:
        """
        Add cost to the current cost

        :param float cost: The cost to add
        """

        self._cost += cost

    def SetCost(self, cost: float) -> None:
        """
        Set the cost

        :param float cost: The cost to set
        """

        self._cost = cost

    def PriorityCost(self) -> float:
        """
        Get the priority cost

        returns: The priority cost
        rtype: float
        """

        return self._priority_cost

    def SetPriorityCost(self, cost: float) -> None:
        """

        :param float cost: The cost to set
        """

        self._priority_cost = cost


    def SetLatticeCruiseSpeed(self, speed: float) -> None:
        """
        Set the lattice cruise speed
        For lattice planner'speed planning target

        :param float speed: The speed to set
        """

        self._planning_target.cruise_speed = speed

    @property
    def planning_target(self) -> PlanningTarget:
        """
        Get the planning target
        For lattice planner'speed planning target
        """

        return self._planning_target

    def SetCruiseSpeed(self, speed: float) -> None:
        """
        Set the cruise speed

        :param float speed: The speed to set
        """

        self._cruise_speed = speed
        self._base_cruise_speed = speed







    @property
    def path_data(self) -> PathData:
        """
        Get the path data

        :returns: The path data
        :rtype: PathData
        """

        return self._path_data

    def SetPathData(self, path_data: PathData) -> None:
        self._path_data = path_data

    def SetSpeedData(self, speed_data: SpeedData) -> None:
        self._speed_data = speed_data

    @property
    def debug(self):
        if self._debug.planning_data is None:
            self._debug.planning_data = PlanningData()
        return self._debug

    @property
    def speed_data(self) -> SpeedData:
        """
        Get the speed data

        :returns: The speed data
        :rtype: SpeedData
        """

        return self._speed_data

    def CombinePathAndSpeedProfile(self, relative_time: float, start_s: float, discretized_trajectory: DiscretizedTrajectory) -> bool:
        """
        aggregate final result together by some configuration
        """

        assert discretized_trajectory is not None, "discretized_trajectory is None"
        # use varied resolution to reduce data load but also provide enough data
        # point for control module
        kDenseTimeResoltuion = config_module.FLAGS_trajectory_time_min_interval
        kSparseTimeResolution = config_module.FLAGS_trajectory_time_max_interval
        kDenseTimeSec = config_module.FLAGS_trajectory_time_high_density_period

        if not self._path_data.discretized_path:
            logger.error("path data is empty.")
            return False

        if not self._speed_data:
            logger.error("speed profile is empty")
            return False

        cur_rel_time = 0.0
        while cur_rel_time < self._speed_data.TotalTime():
            tag, speed_point = self._speed_data.EvaluateByTime(cur_rel_time)
            if not tag:
                logger.error(f"Fail to get speed point with relative time {cur_rel_time}")
                return False
            
            if speed_point.s > self._path_data.discretized_path.Length():
                break

            path_point: PathPoint = self._path_data.GetPathPointWithPathS(speed_point.s)
            path_point.s += start_s

            trajectory_point: TrajectoryPoint = TrajectoryPoint(path_point=deepcopy(path_point), v=speed_point.v, a=speed_point.a,
                                                                relative_time=speed_point.t + relative_time)
            discretized_trajectory.AppendTrajectoryPoint(trajectory_point)

            cur_rel_time += (kDenseTimeResoltuion if cur_rel_time < kDenseTimeSec else kSparseTimeResolution)

        if self._path_data.is_reverse_path():
            for trajectory_point in discretized_trajectory:
                trajectory_point.v = -trajectory_point.v
                trajectory_point.a = -trajectory_point.a
            logger.info("reversed path")
            discretized_trajectory.SetIsReversed(True)
        
        return True

    @staticmethod
    def AdjustTrajectoryWhichStartsFromCurrentPos(planning_start_point: TrajectoryPoint,
                                                  trajectory: List[TrajectoryPoint]) -> tuple:
        """
        adjust trajectory if it starts from cur_vehicle postion rather planning
        init point from upstream

        :param TrajectoryPoint planning_start_point: The planning start point
        :param List[TrajectoryPoint] trajectory: The trajectory to adjust
        :returns: (bool, The adjusted trajectory)
        :rtype: Tuple[bool, DiscretizedTrajectory]
        """

        # TODO(all): It is a brutal way to insert the planning init point, one elegant
        # way would be bypassing trajectory stitching logics somehow, or use planing
        # init point from trajectory stitching to compute the trajectory at the very
        # start

        # find insert index by check heading
        kMaxAngleDiff: float = math.pi / 2.0

        start_point_heading: float = planning_start_point.path_point.theta
        start_point_x: float = planning_start_point.path_point.x
        start_point_y: float = planning_start_point.path_point.y
        start_point_relative_time: float = planning_start_point.relative_time

        insert_idx: int = -1
        for i, traj in enumerate(trajectory):
            # skip trajectory_points early than planning_start_point
            if traj.relative_time <= start_point_relative_time:
                continue

            cur_point_x: float = traj.path_point.x
            cur_point_y: float = traj.path_point.y
            tracking_heading: float = math.atan2(cur_point_y - start_point_y, cur_point_x - start_point_x)
            if abs(AngleDiff(start_point_heading, tracking_heading)) < kMaxAngleDiff:
                insert_idx = i
                break
        
        if insert_idx == -1:
            logger.error(f"All points are behind of planning init point")
            return False, None

        cut_trajectory = DiscretizedTrajectory(trajectory)
        cut_trajectory = cut_trajectory[insert_idx:]
        cut_trajectory.insert(0, planning_start_point)

        # In class TrajectoryStitcher, the stitched point which is also the planning
        # init point is supposed have one planning_cycle_time ahead respect to
        # current timestamp as its relative time. So the relative timelines
        # of planning init point and the trajectory which start from current
        # position(relative time = 0) are the same. Therefore any conflicts on the
        # relative time including the one below should return false and inspected its
        # cause.
        if len(cut_trajectory) > 1 and cut_trajectory[0].relative_time >= cut_trajectory[1].relative_time:
            logger.error(f"planning init point relative_time[{cut_trajectory[0].relative_time}] larger than its next point's relative_time[{cut_trajectory[1].relative_time}]")
            return False, None
        
        # In class TrajectoryStitcher, the planing_init_point is set to have s as 0,
        # so adjustment is needed to be done on the other points
        accumulated_s: float = 0.0
        for i in range(1, len(cut_trajectory)):
            pre_path_point: PathPoint = cut_trajectory[i - 1].path_point
            cur_path_point: PathPoint = cut_trajectory[i].path_point
            accumulated_s += math.sqrt((cur_path_point.x - pre_path_point.x) ** 2 + (cur_path_point.y - pre_path_point.y) ** 2)
            cur_path_point.s = accumulated_s

        # reevaluate relative_time to make delta t the same
        adjusted_trajectory = DiscretizedTrajectory()
        adjusted_trajectory.clear()
        # use varied resolution to reduce data load but also provide enough data
        # point for control module
        kDenseTimeResoltuion: float = config_module.FLAGS_trajectory_time_min_interval
        kSparseTimeResolution: float = config_module.FLAGS_trajectory_time_max_interval
        kDenseTimeSec: float = config_module.FLAGS_trajectory_time_high_density_period
        cur_rel_time = cut_trajectory[0].relative_time
        while cur_rel_time <= cut_trajectory[-1].relative_time:

            adjusted_trajectory.AppendTrajectoryPoint(cut_trajectory.Evaluate(cur_rel_time))
            cur_rel_time += (kDenseTimeResoltuion if cur_rel_time < kDenseTimeSec else kSparseTimeResolution)

        return True, adjusted_trajectory

    
    
    def IsChangeLanePath(self) -> bool:
        """
        Check if the current reference line is a change lane reference line, i.e.,
        ADC's current position is not on this reference line.

        :returns: True if it is a change lane reference line, otherwise False.
        :rtype: bool
        """

        return bool(self._lanes is not None and not self._lanes.IsOnSegment())


    def SetDrivable(self, drivable: bool) -> None:
        """
        Set if the vehicle can drive following this reference line
        A planner need to set this value to true if the reference line is OK

        :param bool drivable: The value to set
        """

        self._is_drivable = drivable
    
    def IsDrivable(self) -> bool:
        """
        Get if the vehicle can drive following this reference line

        :returns: True if the vehicle can drive following this reference line, otherwise False.
        :rtype: bool
        """

        return self._is_drivable
    

    def Lanes(self) -> RouteSegments:
        """
        Get the lanes

        :returns: The lanes
        :rtype: RouteSegments
        """

        return self._lanes
    












    def GetBlockingObstacle(self) -> Obstacle:
        """
        Get the blocking obstacle

        :returns: The blocking obstacle
        :rtype: Obstacle
        """

        return self._blocking_obstacle

    def SetBlockingObstacle(self, blocking_obstacle_id: str) -> None:
        """
        Set the blocking obstacle

        :param str blocking_obstacle_id: The obstacle to set
        """

        self._blocking_obstacle = self._path_decision.Find(blocking_obstacle_id)



    def set_is_on_reference_line(self) -> None:
        """
        Set the path is on reference line
        """

        self._is_on_reference_line = True

    def GetPriority(self) -> int:
        """
        Get the priority

        :returns: The priority
        :rtype: int
        """

        return self._reference_line.GetPriority()

    def SetPriority(self, priority: int) -> None:
        """
        Set the priority

        :param int priority: The priority to set
        """

        self._reference_line.SetPriority(priority)

    def set_trajectory_type(self, trajectory_type: ADCTrajectory.TrajectoryType) -> None:
        """
        Set the trajectory type

        :param ADCTrajectory.TrajectoryType trajectory_type: The trajectory type to set
        """

        self._trajectory_type = trajectory_type

    @property
    def trajectory_type(self) -> ADCTrajectory.TrajectoryType:
        """
        Get the trajectory type

        :returns: The trajectory type
        :rtype: ADCTrajectory.TrajectoryType
        """

        return self._trajectory_type

    def IsIrrelevantObstacle(self, obstacle: Obstacle) -> bool:
        """
        Check if the obstacle is irrelevant

        :param Obstacle obstacle: The obstacle to check
        :returns: True if the obstacle is irrelevant, otherwise False.
        :rtype: bool
        """

        if obstacle.IsCautionLevelObstacle():
            return False

        # if adc is on the road, and obstacle behind adc, ignore
        obstacle_boundary = obstacle.PerceptionSLBoundary()
        if obstacle_boundary.end_s > self._reference_line.Length():
            return True
        if self._is_on_reference_line and (not self.IsChangeLanePath()) \
                                      and (obstacle_boundary.end_s < self._adc_sl_boundary.end_s) \
                                      and (self._reference_line.IsOnLane(obstacle_boundary) or obstacle_boundary.end_s < 0.0):
            # if obstacle is far backward
            return True

        return False






    

    def __copy__(self):
        """
        Disallow copy operation
        """

        raise NotImplementedError("Copy operation is not allowed")

    def __deepcopy__(self, memo):
        """
        Disallow deep copy operation
        """

        raise NotImplementedError("Deep copy operation is not allowed")

    @property
    def index(self) -> int:
        """
        Get the index

        :returns: The index
        :rtype: int
        """

        return self._index

    def set_index(self, index: int) -> None:
        """
        Set the index

        :param int index: The index to set
        """

        self._index = index
