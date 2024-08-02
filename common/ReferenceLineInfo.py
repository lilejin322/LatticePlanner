from typing import List, Dict, Tuple
from enum import Enum
from protoclass.TrajectoryPoint import TrajectoryPoint
from common.ReferenceLine import ReferenceLine
from common.RouteSegments import RouteSegments
from common.Obstacle import Obstacle
from common.DiscretizedTrajectory import DiscretizedTrajectory
from protoclass.SLBoundary import SLBoundary
from protoclass.ADCTrajectory import ADCTrajectory, EngageAdvice
from protoclass.Lane import Lane
from common.Path import PathOverlap
from protoclass.SLBoundary import SLPoint
from protoclass.DecisionResult import DecisionResult, VehicleSignal, ObjectDecisions, ObjectDecisionType, ObjectIgnore, ChangeLaneType, \
                                      MainDecision, MainStop, MainCruise, MainEmergencyStop, EmergencyStopCruiseToStop, ObjectDecision, \
                                      ObjectAvoid, StopReasonCode, MainMissionComplete, ObjectStop
from protoclass.VehicleState import VehicleState
from common.PathData import PathData
from common.PathDecision import PathDecision
from common.SpeedData import SpeedData
from protoclass.LatencyStats import LatencyStats
from common.PathBoundary import PathBoundary
from common.PlanningContext import PlanningContext
from protoclass.RSSInfo import RSSInfo
from protoclass.PathPoint import PathPoint
from protoclass.lattice_structure import StopPoint, PlanningTarget
from common.StGraphData import StGraphData
from common.LaneInfo import LaneInfo
from config import FRONT_EDGE_TO_CENTER, BACK_EDGE_TO_CENTER, LEFT_EDGE_TO_CENTER, RIGHT_EDGE_TO_CENTER, \
                   EGO_VEHICLE_LENGTH, EGO_VEHICLE_WIDTH, FLAGS_speed_bump_speed_limit, FLAGS_default_cruise_speed, \
                   FLAGS_use_multi_thread_to_add_obstacles, FLAGS_trajectory_time_min_interval, \
                   FLAGS_trajectory_time_max_interval, FLAGS_trajectory_time_high_density_period, \
                   FLAGS_passed_destination_threshold, FLAGS_destination_check_distance, FLAGS_turn_signal_distance, \
                   FLAGS_obstacle_lon_ignore_buffer
from common.Vec2d import Vec2d
from common.Box2d import Box2d
from logging import Logger
from common.ReferencePoint import ReferencePoint
from copy import deepcopy, copy
from concurrent.futures import ThreadPoolExecutor, as_completed
import math
from protoclass.PointENU import PointENU

logger = Logger("ReferenceLineInfo")

def WithinOverlap(overlap: PathOverlap, s: float) -> bool:
    """
    Check if the s is within the overlap

    :param PathOverlap overlap: The overlap
    :param float s: The s value
    :returns: True if the s is within the overlap, otherwise False.
    :rtype: bool
    """

    kEpsilon: float = 1e-2
    return overlap.start_s - kEpsilon <= s <= overlap.end_s + kEpsilon

class ReferenceLineInfo:
    """
    ReferenceLineInfo holds all data for one reference line.
    """

    class LaneType(Enum):

        LeftForward = 0
        LeftReverse = 1
        RightForward = 2
        RightReverse = 3

    class OverlapType(Enum):
        """
        different types of overlaps that can be handled by different scenarios.
        """

        CLEAR_AREA = 1
        CROSSWALK = 2
        OBSTACLE = 3
        PNC_JUNCTION = 4
        SIGNAL = 5
        STOP_SIGN = 6
        YIELD_SIGN = 7
        JUNCTION = 8

    def __init__(self, vehicle_state: VehicleState=None, adc_planning_point: TrajectoryPoint=None, reference_line: ReferenceLine=None, segments: RouteSegments=None):
        """
        Constructor
        """

        self._vehicle_state = vehicle_state
        self._adc_planning_point = adc_planning_point
        self._reference_line = reference_line
        self._lanes: RouteSegments = segments
        # this is the number that measures the goodness of this reference line.
        # The lower the better.
        self._cost: float = 0.0
        self._is_drivable: bool = True
        self._path_decision: PathDecision = None
        self._blocking_obstacle: Obstacle = None
        self._candidate_path_boundaries: List[PathBoundary] = []
        self._candidate_path_data: List[PathData] = []
        self._path_data: PathData = None
        self._fallback_path_data: PathData = None
        self._speed_data: SpeedData = None
        self._discretized_trajectory: DiscretizedTrajectory = None
        self._rss_info: RSSInfo = None
        # SL boundary of stitching point (starting point of plan trajectory)
        # relative to the reference line
        self._adc_sl_boundary: SLBoundary = None
        self._latency_stats: LatencyStats = None
        self._is_on_reference_line: bool = False
        self._is_path_lane_borrow: bool = False
        self._status: ADCTrajectory.RightOfWayStatus = ADCTrajectory.RightOfWayStatus.UNPROTECTED
        self._offset_to_other_reference_line: float = 0.0
        self._priority_cost: float = 0.0
        self._planning_target: PlanningTarget = None
        self._trajectory_type = ADCTrajectory.TrajectoryType.UNKNOWN
        # Overlaps encountered in the first time along the reference line in front of
        # the vehicle
        self._first_encounter_overlaps: List[Tuple[ReferenceLineInfo.OverlapType, PathOverlap]] = []
        # Data generated by speed_bounds_decider for constructing st_graph for
        # different st optimizer
        self._st_graph_data: StGraphData = None
        self._vehicle_signal: VehicleSignal = None
        self._cruise_speed = 0.0
        self._base_cruise_speed = 0.0
        self._path_reusable = False
        self._junction_right_of_way_map: Dict[str, bool] = {}

    def Init(self, obstacles: List[Obstacle], target_speed: float) -> bool:
        """

        returns: True if successful, False otherwise
        rtype: bool
        """

        # stiching point
        path_point = self._adc_planning_point.path_point
        position: Vec2d = Vec2d(path_point.x, path_point.y)
        vec_to_center: Vec2d = Vec2d((FRONT_EDGE_TO_CENTER - BACK_EDGE_TO_CENTER) / 2.0, (LEFT_EDGE_TO_CENTER - RIGHT_EDGE_TO_CENTER) / 2.0)
        center: Vec2d = position + vec_to_center.rotate(path_point.theta)
        box: Box2d = Box2d(center, path_point.theta, EGO_VEHICLE_LENGTH, EGO_VEHICLE_WIDTH)
        # realtime vehicle position
        vehicle_position: Vec2d = Vec2d(self._vehicle_state.x, self._vehicle_state.y)
        vehicle_center: Vec2d = Vec2d(vehicle_position + vec_to_center.rotate(self._vehicle_state.heading))
        vehicle_box: Box2d = Box2d(vehicle_center, self._vehicle_state.heading, EGO_VEHICLE_LENGTH, EGO_VEHICLE_WIDTH)
        if not self._reference_line.GetSLBoundary(box, self._adc_sl_boundary):
            logger.error(f"Failed to get ADC boundary from box: {box}")
            return False
        
        self.InitFirstOverlaps()

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
        for speed_bump in map_path.speed_bump_overlaps:
            # -1 and + 1.0 are added to make sure it can be sampled.
            self._reference_line.AddSpeedLimit(speed_bump.start_s - 1.0, speed_bump.end_s + 1.0, FLAGS_speed_bump_speed_limit)

        self.SetCruiseSpeed(target_speed)
        # set lattice planning target speed limit;
        self.SetLatticeCruiseSpeed(target_speed)

        self._vehicle_signal = None

        return True

    def AddObstacles(self, obstacles: List[Obstacle]) -> bool:
        """
        Add obstacles to the reference line info

        param List[Obstacle] obstacles: List of obstacles to add
        returns: True if successful, False otherwise
        rtype: bool
        """

        if FLAGS_use_multi_thread_to_add_obstacles:
            with ThreadPoolExecutor() as executor:
                futures = {executor.submit(self.AddObstacle, obstacle): obstacle for obstacle in obstacles}
                for future in as_completed(futures):
                    if not future.result():
                        logger.error("Fail to add obstacles.")
                        return False
        else:
            for obstacle in obstacles:
                if not self.AddObstacle(obstacle):
                    logger.error(f"Failed to add obstacle {obstacle.Id}.")
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
            logger.error(f"Failed to add obstacle {obstacle.Id}")
            return None

        perception_sl = SLBoundary()
        tag = self._reference_line.GetSLBoundary(obstacle.PerceptionPolygon, perception_sl)
        if not tag:
            logger.error(f"Failed to get SL boundary for obstacle {obstacle.Id}")
            return mutable_obstacle
        mutable_obstacle.SetPerceptionSlBoundary(perception_sl)
        mutable_obstacle.CheckLaneBlocking(self._reference_line)
        if mutable_obstacle.IsLaneBlocking():
            logger.debug(f"Obstacle {obstacle.Id} is lane blocking.")
        else:
            logger.debug(f"Obstacle {obstacle.Id} is NOT lane blocking.")
        
        if self.IsIrrelevantObstacle(mutable_obstacle):
            ignore = ObjectDecisionType()
            ignore.object_tag = ObjectIgnore()
            self.path_decision.AddLateralDecision("reference_line_filter", obstacle.Id, ignore)
            self.path_decision.AddLongitudinalDecision("reference_line_filter", obstacle.Id, ignore)
            logger.debug(f"NO build reference line st boundary. id: {obstacle.Id}")
        else:
            logger.debug(f"Build reference line st boundary. id: {obstacle.Id}")
            mutable_obstacle.BuildReferenceLineStBoundary(self._reference_line, self._adc_sl_boundary.start_s)
            logger.debug(f"Reference line st boundary: t[{mutable_obstacle.reference_line_st_boundary().min_t}, {mutable_obstacle.reference_line_st_boundary().max_t}] s[{mutable_obstacle.reference_line_st_boundary().min_s}, {mutable_obstacle.reference_line_st_boundary().max_s}]")

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
    
    def SDistanceToDestination(self) -> float:
        """
        Get the distance to the destination

        returns: The distance to the destination
        rtype: float
        """

        res: float = float('inf')
        dest = self._path_decision.Find(FLAGS_destination_obstacle_id)
        if not dest:
            return res
        if not dest.LongitudinalDecision.has_stop():
            return res
        if not self._reference_line.IsOnLane(dest.PerceptionBoundingBox.center):
            return res
        stop_s = dest.PerceptionSLBoundary.start_s + dest.LongitudinalDecision.stop().distance_s
        return stop_s - self._adc_sl_boundary.end_s

    def ReachedDestination(self) -> bool:
        """
        Judge if the vehicle has reached the destination

        returns: True if the vehicle has reached the destination, otherwise False.
        rtype: bool
        """

        distance_destination: float = self.SDistanceToDestination()
        return distance_destination <= FLAGS_passed_destination_threshold
    
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
    
    @property
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
    
    @property
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
    
    def SetLatticeStopPoint(self, stop_point: StopPoint) -> None:
        """
        Set the lattice stop point
        For lattice planner'speed planning target

        :param StopPoint stop_point: The stop point to set
        """

        self._planning_target.stop_point = deepcopy(stop_point)
    
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
    
    def LimitCruiseSpeed(self, speed: float) -> None:
        """
        Limit the cruise speed based on the "base_cruise_speed_". If the new
        setting speed > "base_cruise_speed_", it will be ignored.
        
        :param float speed: The new speed.
        """

        if self._base_cruise_speed <= speed:
            return
        self._cruise_speed = speed
    
    def GetBaseCruiseSpeed(self) -> float:
        """
        Get the base cruise speed
        """

        return self._base_cruise_speed if self._base_cruise_speed > 0.0 else FLAGS_default_cruise_speed
    
    def GetCruiseSpeed(self) -> float:
        """
        Get the cruise speed
        """

        return self._cruise_speed if self._cruise_speed > 0.0 else FLAGS_default_cruise_speed
    
    def LocateLaneInfo(self, s: float) -> LaneInfo:
        """
        Locate the lane info based on the s

        :param float s: The s value
        :returns: The lane info
        :rtype: LaneInfo
        """

        lanes: List[LaneInfo] = self._reference_line.GetLaneFromS(s)
        if not lanes:
            logger.warning(f"cannot get any lane using s: {s}.")
            return None
        
        return lanes[0]

    def GetNeighborLaneInfo(self, lane_type: LaneType, s: float) -> Tuple[bool, str, float]:
        """
        Get the neighbor lane info

        :param LaneType lane_type: The lane type
        :param float s: The s value
        :returns: The neighbor lane info, (bool, str lane_id, float lane_width)
        :rtype: Tuple[bool, str, float]
        """

        lane_info = self.LocateLaneInfo(s)
        if lane_info is None:
            return False, "", float("inf")
        
        if lane_type == self.LaneType.LeftForward:
            if not lane_info.lane.left_neighbor_forward_lane_id:
                return False, "", float("inf")
            lane_id = lane_info.lane.left_neighbor_forward_lane_id[0]
        
        elif lane_type == self.LaneType.LeftReverse:
            if not lane_info.lane.left_neighbor_reverse_lane_id:
                return False, "", float("inf")
            lane_id = lane_info.lane.left_neighbor_reverse_lane_id[0]

        elif lane_type == self.LaneType.RightForward:
            if not lane_info.lane.right_neighbor_forward_lane_id:
                return False, "", float("inf")
            lane_id = lane_info.lane.right_neighbor_forward_lane_id[0]
        
        elif lane_type == self.LaneType.RightReverse:
            if not lane_info.lane.right_neighbor_reverse_lane_id:
                return False, "", float("inf")
            lane_id = lane_info.lane.right_neighbor_reverse_lane_id[0]

        else:
            raise ValueError("Invalid LaneType")

        neighbor_lane = hdmap.HDMapUtil.BaseMapPtr().GetLaneById(lane_id)
        if neighbor_lane is None:
            return False, "", float("inf")
        
        ref_point: ReferencePoint = self._reference_line.GetReferencePoint(s)

        tag, neighbor_s, neighbor_l = neighbor_lane.GetProjection(Vec2d(ref_point.x, ref_point.y))
        if not tag:
            return False, "", float("inf")
        
        lane_width = neighbor_lane.GetWidth(neighbor_s)
        return True, lane_id, lane_width

    def IsStartFrom(self, previous_reference_line_info: 'ReferenceLineInfo') -> bool:
        """
        check if current reference line is started from another reference
        line info line. The method is to check if the start point of current
        reference line is on previous reference line info.

        :returns: True if current reference line starts on previous reference
        line, otherwise False.
        """

        if not self._reference_line.reference_points:
            return False
        start_point = self._reference_line.reference_points[0]
        prev_reference_line = previous_reference_line_info.reference_line
        _, sl_point = prev_reference_line.XYToSL(start_point)
        return previous_reference_line_info.reference_line.IsOnLane(sl_point)

    @property
    def latency_stats(self) -> LatencyStats:
        """
        Get the latency stats

        :returns: The latency stats
        :rtype: LatencyStats
        """

        return self._latency_stats

    @property
    def path_data(self) -> PathData:
        """
        Get the path data

        :returns: The path data
        :rtype: PathData
        """

        return self._path_data

    @property
    def fallback_path_data(self) -> PathData:
        """
        Get the fallback path data

        :returns: The fallback path data
        :rtype: PathData
        """

        return self._fallback_path_data

    @property
    def speed_data(self) -> SpeedData:
        """
        Get the speed data

        :returns: The speed data
        :rtype: SpeedData
        """

        return self._speed_data

    @property
    def rss_info(self) -> RSSInfo:
        """
        Get the rss info

        :returns: The rss info
        :rtype: RSSInfo
        """

        return self._rss_info
    
    def CombinePathAndSpeedProfile(self, relative_time: float, start_s: float, discretized_trajectory: DiscretizedTrajectory) -> bool:
        """
        aggregate final result together by some configuration
        """

        assert discretized_trajectory is not None, "discretized_trajectory is None"
        # use varied resolution to reduce data load but also provide enough data
        # point for control module
        kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval
        kSparseTimeResolution = FLAGS_trajectory_time_max_interval
        kDenseTimeSec = FLAGS_trajectory_time_high_density_period

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

    def AdjustTrajectoryWhichStartsFromCurrentPos(planning_start_point: TrajectoryPoint,
                                                  trajectory: List[TrajectoryPoint]) -> Tuple[bool, DiscretizedTrajectory]:
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
            if abs(AngleDiff(start_point_heading, tracking_heading) < kMaxAngleDiff):
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
        kDenseTimeResoltuion: float = FLAGS_trajectory_time_min_interval
        kSparseTimeResolution: float = FLAGS_trajectory_time_max_interval
        kDenseTimeSec: float = FLAGS_trajectory_time_high_density_period
        cur_rel_time = cut_trajectory[0].relative_time
        while cur_rel_time <= cut_trajectory[-1].relative_time:

            adjusted_trajectory.AppendTrajectoryPoint(adjusted_trajectory.Evaluate(cur_rel_time))
            cur_rel_time += (kDenseTimeResoltuion if cur_rel_time < kDenseTimeSec else kSparseTimeResolution)

        return True, adjusted_trajectory

    def AdcSlBoundary(self) -> SLBoundary:
        """

        """

        return self._adc_sl_boundary
    
    def PathSpeedDebugString(self) -> str:
        """
        Get the path speed debug string

        :returns: The path speed debug string
        :rtype: str
        """

        return f"path_data: {self._path_data}, speed_data: {self._speed_data}"
    
    def IsChangeLanePath(self) -> bool:
        """
        Check if the current reference line is a change lane reference line, i.e.,
        ADC's current position is not on this reference line.

        :returns: True if it is a change lane reference line, otherwise False.
        :rtype: bool
        """

        return not self.Lanes.IsOnSegment

    def IsNeighborLanePath(self) -> bool:
        """
        Check if the current reference line is the neighbor of the vehicle
        current position

        :returns: True if it is a neighbor lane reference line, otherwise False.
        :rtype: bool
        """

        return self.Lanes.IsNeighborSegment

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
    
    def ExportEngageAdvice(self, planning_context: PlanningContext) -> EngageAdvice:
        """
        Export engage advice

        :param PlanningContext planning_context: The planning context
        :returns: The engage advice
        :rtype: EngageAdvice
        """

        prev_advice = EngageAdvice()
        kMaxAngleDiff: float = math.pi / 6.0

        engage: bool = False
        if not self.IsDrivable():
            prev_advice.reason = "Reference line not drivable"
        elif not self._is_on_reference_line:
            scenario_type = planning_context.planning_status.scenario.scenario_type
            if scenario_type == "PARK_AND_GO" or self.IsChangeLanePath():
                # note: when self._is_on_reference_line is FALSE
                #  (1) always engage while in PARK_AND_GO scenario
                #  (2) engage when "ChangeLanePath" is picked as Drivable ref line
                #  where most likely ADC not OnLane yet
                engage = True
            else:
                prev_advice.reason = "Not on reference line"
        else:
            # check heading
            ref_point = self._reference_line.GetReferencePoint(self._adc_sl_boundary.end_s)
            if AngleDiff(self._vehicle_state.heading, ref_point.heading) < kMaxAngleDiff:
                engage = True
            else:
                prev_advice.reason = "Vehicle heading is not aligned"
        
        if engage:
            if self._vehicle_state.driving_mode != VehicleState.DrivingMode.COMPLETE_AUTO_DRIVE:
                # READY_TO_ENGAGE when in non-AUTO mode
                prev_advice.advice = EngageAdvice.Advice.READY_TO_ENGAGE
            else:
                # KEEP_ENGAGED when in AUTO mode
                prev_advice.advice = EngageAdvice.Advice.KEEP_ENGAGED
            prev_advice.reason = ""
        else:
            if prev_advice.advice != EngageAdvice.Advice.DISALLOW_ENGAGE:
                prev_advice.advice = EngageAdvice.Advice.PREPARE_DISENGAGE
        
        engage_advice: EngageAdvice = deepcopy(prev_advice)
        return engage_advice

    @property
    def Lanes(self) -> RouteSegments:
        """
        Get the lanes

        :returns: The lanes
        :rtype: RouteSegments
        """

        return self._lanes
    
    def TargetLaneId(self) -> List[str]:
        """
        Get the target lane id

        :returns: The target lane id
        :rtype: List[str]
        """

        lane_ids: List[str] = []
        for lane_seg in self._lanes:
            lane_ids.append(lane_seg.lane.id)
        return lane_ids

    def ExportDecision(self, planning_context: PlanningContext) -> Tuple[DecisionResult, PlanningContext]:
        """
        Export decision

        :param PlanningContext planning_context: The planning context
        :returns: (DecisionResult decision_result, PlanningContext planning_context)
        :rtype: Tuple[DecisionResult, PlanningContext]
        """

        decision_result = self.MakeDecision(planning_context)
        vehicle_signal = self.ExportVehicleSignal(decision_result)
        main_decision: MainDecision = decision_result.main_decision
        if isinstance(main_decision.task, MainStop):
            main_decision.task.change_lane_type = self.Lanes.PreviousAction()
        elif isinstance(main_decision.task, MainCruise):
            main_decision.task.change_lane_type = self.Lanes.PreviousAction()
        return decision_result, planning_context

    def SetJunctionRightOfWay(self, junction_s: float, is_protected: bool) -> None:
        """
        Set the junction right of way

        :param float junction_s: The s value of the junction
        :param bool is_protected: True if the junction is protected, otherwise False.
        """

        for overlap in self._reference_line.map_path.junction_overlaps:
            if WithinOverlap(overlap, junction_s):
                self._junction_right_of_way_map[overlap.object_id] = is_protected

    def GetRightOfWayStatus(self) -> ADCTrajectory.RightOfWayStatus:
        """
        Get the right of way status

        :returns: The right of way status
        :rtype: ADCTrajectory.RightOfWayStatus
        """

        for overlap in self._reference_line.map_path.junction_overlaps:
            if overlap.end_s < self._adc_sl_boundary.start_s:
                self._junction_right_of_way_map.pop(overlap.object_id)
            elif WithinOverlap(overlap, self._adc_sl_boundary.end_s):
                is_protected: bool = self._junction_right_of_way_map.get(overlap.object_id)
                if is_protected:
                    return ADCTrajectory.RightOfWayStatus.PROTECTED
    
        return ADCTrajectory.RightOfWayStatus.UNPROTECTED

    def GetPathTurnType(self, s: float) -> Lane.LaneTurn:
        """
        Get the path turn type

        :param float s: The s value
        :returns: The path turn type
        :rtype: Lane.LaneTurn
        """

        forward_buffer: float = 20.0
        route_s: float = 0.0
        for seg in self.Lanes:
            if route_s > s + forward_buffer:
                break
            route_s += seg.end_s - seg.start_s
            if route_s < s:
                continue
            turn_type = seg.lane.turn
            if turn_type == Lane.LaneTurn.LEFT_TURN or \
               turn_type == Lane.LaneTurn.RIGHT_TURN or \
               turn_type == Lane.LaneTurn.U_TURN:
                return turn_type
    
        return Lane.LaneTurn.NO_TURN

    def GetIntersectionRightofWayStatus(self, pnc_junction_overlap: PathOverlap) -> bool:
        """
        Get the intersection right of way status

        :param PathOverlap pnc_junction_overlap: The pnc junction overlap
        :returns: the bool result
        :rtype: bool
        """

        if self.GetPathTurnType(pnc_junction_overlap.start_s) != Lane.LaneTurn.NO_TURN:
            return False
        
        # TODO(all): iterate exits of intersection to check/compare speed-limit
        return True

    @property
    def OffsetToOtherReferenceLine(self) -> float:
        """
        Get the offset to other reference line

        :returns: The offset to other reference line
        :rtype: float
        """

        return self._offset_to_other_reference_line
    
    def SetOffsetToOtherReferenceLine(self, offset: float) -> None:
        """
        Set the offset to other reference line

        :param float offset: The offset to set
        """

        self._offset_to_other_reference_line = offset
    
    def GetCandidatePathBoundaries(self) -> List[PathBoundary]:
        """
        Get the candidate path boundaries

        :returns: The candidate path boundaries
        :rtype: List[PathBoundary]
        """

        return self._candidate_path_boundaries
    
    def SetCandidatePathBoundaries(self, candidate_path_boundaries: List[PathBoundary]) -> None:
        """
        Set the candidate path boundaries

        :param PathBoundary candidate_path_boundaries: The candidate path boundaries to set
        """

        self._candidate_path_boundaries = candidate_path_boundaries

    def GetCandidatePathData(self) -> List[PathData]:
        """
        Get the candidate path data

        :returns: The candidate path data
        :rtype: List[PathData]
        """

        return self._candidate_path_data
    
    def SetCandidatePathData(self, candidate_path_data: List[PathData]) -> None:
        """
        Set the candidate path data

        :param List[PathData] candidate_path_data: The candidate path data to set
        """

        self._candidate_path_data = candidate_path_data
    
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

    def is_path_lane_borrow(self) -> bool:
        """
        Check if the path is lane borrow

        :returns: True if the path is lane borrow, otherwise False.
        :rtype: bool
        """

        return self._is_path_lane_borrow
    
    def set_is_path_lane_borrow(self, is_path_lane_borrow: bool) -> None:
        """
        Set the path is lane borrow

        :param bool is_path_lane_borrow: The value to set
        """

        self._is_path_lane_borrow = is_path_lane_borrow

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

    @property
    def st_graph_data(self) -> StGraphData:
        """
        Get the st graph data
        
        :returns: The st graph data
        :rtype: StGraphData
        """

        return self._st_graph_data
    
    def FirstEncounteredOverlaps(self) -> List[Tuple[OverlapType, PathOverlap]]:
        """
        Get the first encountered overlaps

        :returns: The first encountered overlaps
        :rtype: List[Tuple[OverlapType, PathOverlap]]
        """

        return self._first_encounter_overlaps
    
    def GetPnCJunction(self, s: float) -> Tuple[int, PathOverlap]:
        """
        Get the pnc junction

        :param float s: The s value
        :returns: (int, PathOverlap pnc_junction_overlap)
        :rtype: Tuple[int, PathOverlap]
        """

        pnc_junction_overlaps: List[PathOverlap] = self.reference_line.map_path.pnc_junction_overlaps

        kError: float = 1.0   # meter
        for overlap in pnc_junction_overlaps:
            if s >= overlap.start_s - kError and s <= overlap.end_s + kError:
                pnc_junction_overlap = deepcopy(overlap)
                return 1, pnc_junction_overlap

        return 0, None

    def GetJunction(self, s: float) -> Tuple[int, PathOverlap]:
        """
        Get the junction

        :param float s: The s value
        :returns: (int, PathOverlap junction_overlap)
        :rtype: Tuple[int, PathOverlap]
        """

        junction_overlaps: List[PathOverlap] = self._reference_line.map_path.junction_overlaps

        kError: float = 1.0   # meter
        for overlap in junction_overlaps:
            if s >= overlap.start_s - kError and s <= overlap.end_s + kError:
                junction_overlap = deepcopy(overlap)
                return 1, junction_overlap
        
        return 0, None

    def GetAllStopDecisionSLPoint(self) -> List[SLPoint]:
        """
        Get all stop decision sl point

        :returns: The stop decision sl point
        :rtype: List[SLPoint]
        """

        result: List[SLPoint] = []
        for obstacle in self._path_decision.obstacles:
            object_decision: ObjectDecisionType = obstacle.LongitudinalDecision()
            if not isinstance(object_decision.object_tag, ObjectStop):
                continue
            stop_point: PointENU = object_decision.object_tag.stop_point
            _, stop_line_sl = self._reference_line.XYToSL(stop_point)
            if stop_line_sl.s <= 0 or stop_line_sl.s >= self._reference_line.Length():
                continue
            result.append(stop_line_sl)
        
        # sort by s
        if result:
            result.sort(key=lambda sl_point: sl_point.s)

        return result

    def SetTurnSignal(self, turn_signal: VehicleSignal.TurnSignal) -> None:
        """
        Set the turn signal

        :param VehicleSignal.TurnSignal turn_signal: The turn signal to set
        """

        self._vehicle_signal.turn_signal = turn_signal

    def SetEmergencyLight(self) -> None:
        """
        Set the emergency light
        """

        self._vehicle_signal.emergency_light = True

    def set_path_reusable(self, path_reusable: bool) -> None:
        """
        Set the path reusable

        :param bool path_reusable: The value to set
        """

        self._path_reusable = path_reusable
    
    @property
    def path_reusable(self) -> bool:
        """
        Get the path reusable

        :returns: The path reusable
        :rtype: bool
        """

        return self._path_reusable
    
    def GetOverlapOnReferenceLine(self, overlap_id: str, overlap_type: OverlapType) -> PathOverlap:
        """
        Get the overlap on reference line

        :param str overlap_id: The overlap id
        :param OverlapType overlap_type: The overlap type
        :returns: The overlap
        :rtype: PathOverlap
        """

        if overlap_type == ReferenceLineInfo.OverlapType.SIGNAL:
            # traffic_light_overlap
            traffic_light_overlaps: List[PathOverlap] = self._reference_line.map_path.signal_overlaps
            for traffic_light_overlap in traffic_light_overlaps:
                if traffic_light_overlap.object_id == overlap_id:
                    return traffic_light_overlap

        elif overlap_type == ReferenceLineInfo.OverlapType.STOP_SIGN:
            # stop_sign_overlap
            stop_sign_overlaps: List[PathOverlap] = self._reference_line.map_path.stop_sign_overlaps
            for stop_sign_overlap in stop_sign_overlaps:
                if stop_sign_overlap.object_id == overlap_id:
                    return stop_sign_overlap

        elif overlap_type == ReferenceLineInfo.OverlapType.PNC_JUNCTION:
            # pnc_junction_overlap
            pnc_junction_overlaps: List[PathOverlap] = self._reference_line.map_path.pnc_junction_overlaps
            for pnc_junction_overlap in pnc_junction_overlaps:
                if pnc_junction_overlap.object_id == overlap_id:
                    return pnc_junction_overlap

        elif overlap_type == ReferenceLineInfo.OverlapType.YIELD_SIGN:        
            # yield_sign_overlap
            yield_sign_overlaps: List[PathOverlap] = self._reference_line.map_path.yield_sign_overlaps
            for yield_sign_overlap in yield_sign_overlaps:
                if yield_sign_overlap.object_id == overlap_id:
                    return yield_sign_overlap
        
        elif overlap_type == ReferenceLineInfo.OverlapType.JUNCTION:
            # junction_overlap
            junction_overlaps: List[PathOverlap] = self._reference_line.map_path.junction_overlaps
            for junction_overlap in junction_overlaps:
                if junction_overlap.object_id == overlap_id:
                    return junction_overlap

        return None

    def InitFirstOverlaps(self) -> None:
        """
        Init the first overlaps
        """

        map_path = self._reference_line.map_path
        # clear_zone
        tag, clear_area_overlap = self.GetFirstOverlap(map_path.clear_area_overlaps)
        if tag:
            self._first_encounter_overlaps.append(self.OverlapType.CLEAR_AREA, clear_area_overlap)

        # crosswalk
        tag, crosswalk_overlap = self.GetFirstOverlap(map_path.crosswalk_overlaps)
        if tag:
            self._first_encounter_overlaps.append(self.OverlapType.CROSSWALK, crosswalk_overlap)

        # pnc_junction
        tag, pnc_junction_overlap = self.GetFirstOverlap(map_path.pnc_junction_overlaps)
        if tag:
            self._first_encounter_overlaps.append(self.OverlapType.PNC_JUNCTION, pnc_junction_overlap)
        
        # signal
        tag, signal_overlap = self.GetFirstOverlap(map_path.signal_overlaps)
        if tag:
            self._first_encounter_overlaps.append(self.OverlapType.SIGNAL, signal_overlap)
        
        # stop_sign
        tag, stop_sign_overlap = self.GetFirstOverlap(map_path.stop_sign_overlaps)
        if tag:
            self._first_encounter_overlaps.append(self.OverlapType.STOP_SIGN, stop_sign_overlap)
        
        # yield_sign
        tag, yield_sign_overlap = self.GetFirstOverlap(map_path.yield_sign_overlaps)
        if tag:
            self._first_encounter_overlaps.append(self.OverlapType.YIELD_SIGN, yield_sign_overlap)
        
        # sort by start_s
        if self._first_encounter_overlaps:
            self._first_encounter_overlaps.sort(key=lambda x: x[1].start_s)

    def CheckChangeLane(self) -> bool:
        """
        Check if the vehicle is changing lane

        * This function is weird, cpp source has no implementation but has a declaration
        :returns: True if the vehicle is changing lane, otherwise False.
        :rtype: bool
        """

        raise NotImplementedError

    def SetTurnSignalBasedOnLaneTurnType(self, vehicle_signal: VehicleSignal) -> None:
        """
        Set the turn signal based on lane turn type

        :param VehicleSignal vehicle_signal: The vehicle signal to set
        """

        if vehicle_signal.turn_signal is not None and vehicle_signal.turn_signal != VehicleSignal.TurnSignal.TURN_NONE:
            return
        
        vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_NONE

        # Set turn signal based on lane-change.
        if self.IsChangeLanePath():
            if self.Lanes.PreviousAction() == ChangeLaneType.LEFT:
                vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_LEFT
            elif self.Lanes.PreviousAction() == ChangeLaneType.RIGHT:
                vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_RIGHT
            return
        
        # Set turn signal based on lane-borrow.
        if "left" in self._path_data.path_label:
            vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_LEFT
            return
        if "right" in self._path_data.path_label:
            vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_RIGHT
            return

        # Set turn signal based on lane's turn type.
        route_s: float = 0.0
        adc_s: float = self._adc_sl_boundary.end_s
        for seg in self.Lanes:
            if route_s > adc_s + FLAGS_turn_signal_distance:
                break
            route_s += seg.end_s - seg.start_s
            if route_s < adc_s:
                continue
            turn = seg.lane.turn
            if turn == Lane.LaneTurn.LEFT_TURN:
                vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_LEFT
                break
            elif turn == Lane.LaneTurn.RIGHT_TURN:
                vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_RIGHT
                break
            elif turn == Lane.LaneTurn.U_TURN:
                # check left or right by geometry.
                start_xy = PointFactory.ToVec2d(seg.lane.GetSmoothPoint(seg.start_s))
                middle_xy = PointFactory.ToVec2d(seg.lane.GetSmoothPoint((seg.start_s + seg.end_s) / 2.0))
                end_xy = PointFactory.ToVec2d(seg.lane.GetSmoothPoint(seg.end_s))
                start_to_middle = middle_xy - start_xy
                start_to_end = end_xy - start_xy
                if start_to_middle.CrossProd(start_to_end) < 0:
                    vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_RIGHT
                else:
                    vehicle_signal.turn_signal = VehicleSignal.TurnSignal.TURN_LEFT
                break

    def ExportVehicleSignal(self, decision_result: DecisionResult) -> VehicleSignal:
        """
        Export the vehicle signal

        :param DecisionResult decision_result: The decision result
        :returns: The vehicle signal
        :rtype: VehicleSignal
        """

        decision_result.vehicle_signal = copy(self._vehicle_signal)
        self.SetTurnSignalBasedOnLaneTurnType(decision_result.vehicle_signal)
        return decision_result.vehicle_signal

    def IsIrrelevantObstacle(self, obstacle: Obstacle) -> bool:
        """
        Check if the obstacle is irrelevant

        :param Obstacle obstacle: The obstacle to check
        :returns: True if the obstacle is irrelevant, otherwise False.
        :rtype: bool
        """

        if obstacle.IsCautionLevelObstacle:
            return False
        
        # if adc is on the road, and obstacle behind adc, ignore
        obstacle_boundary = obstacle.PerceptionSLBoundary()
        if obstacle_boundary.end_s > self._reference_line.Length():
            return True
        if self._is_on_reference_line and (not self.IsChangeLanePath()) \
                                      and (self._adc_sl_boundary.end_s - obstacle_boundary.end_s > FLAGS_obstacle_lon_ignore_buffer) \
                                      and (self._reference_line.IsOnLane(obstacle_boundary) or obstacle_boundary.end_s < 0.0):
            # if obstacle is far backward
            return True

        return False

    def MakeDecision(self, planning_context: PlanningContext) -> Tuple[DecisionResult, PlanningContext]:
        """
        Make decision

        :returns: (DecisionResult decision_result, PlanningContext planning_context)
        :rtype: Tuple[DecisionResult, PlanningContext]
        """
            
        # cruise by default
        decision_result = DecisionResult(main_decision=MainDecision(task=MainCruise))

        # check stop decision
        error_code: int = self.MakeMainStopDecision(decision_result)
        if error_code < 0:
            decision_result = self.MakeEStopDecision()
        self.MakeMainMissionCompleteDecision(decision_result, planning_context)
        self.SetObjectDecisions(decision_result)

    def MakeMainStopDecision(self, decision_result: DecisionResult) -> int:
        """
        Make main stop decision

        :param DecisionResult decision_result: The decision result obj to be modified
        :returns: the error code
        :rtype: int
        """

        min_stop_line_s: float = float("inf")
        stop_obstacle: Obstacle = None
        stop_decision: ObjectStop = None

        for obstacle in self._path_decision.obstacles:
            object_decision = obstacle.LongitudinalDecision()
            if not object_decision.stop:
                continue
            
            stop_point:PointENU = object_decision.stop.stop_point
            _, stop_line_sl = self._reference_line.XYToSL(stop_point)

            stop_line_s: float = stop_line_sl.s
            if stop_line_s < 0 or stop_line_s > self._reference_line.Length():
                logger.error(f"Ignore object: {obstacle.Id} fence route_s [{stop_line_s}] not in range [0, {self._reference_line.Length()}]")
                continue

            # check stop_line_s vs adc_s
            if stop_line_s < min_stop_line_s:
                min_stop_line_s = stop_line_s
                stop_obstacle = obstacle
                stop_decision = object_decision.stop

        if stop_obstacle is not None:
            decision_result.main_decision.task = MainStop()
            main_stop: MainStop = decision_result.main_decision.task
            main_stop.reason_code = stop_decision.reason_code
            main_stop.reason = "stop by " + stop_obstacle.Id
            main_stop.stop_point.x = stop_decision.stop_point.x 
            main_stop.stop_point.y = stop_decision.stop_point.y
            main_stop.stop_heading = stop_decision.stop_heading
            logger.debug(f" main stop obstacle id:{stop_obstacle.Id} stop_line_s:{min_stop_line_s} stop_point: ({stop_decision.stop_point.x}, {stop_decision.stop_point.y}) stop_heading: {stop_decision.stop_heading}")

            return 1
    
        return 0

    def MakeMainMissionCompleteDecision(self, decision_result: DecisionResult, planning_context: PlanningContext) -> None:
        """
        Make main mission complete decision

        :returns: (DecisionResult decision_result, PlanningContext planning_context)
        :rtype: Tuple[DecisionResult, PlanningContext]
        """

        if not isinstance(decision_result.main_decision.task, MainStop):
            return
        main_stop: MainStop = decision_result.main_decision.task
        if main_stop.reason_code != StopReasonCode.STOP_REASON_DESTINATION or main_stop.reason_code != StopReasonCode.STOP_REASON_PULL_OVER:
            return
        distance_destination: float = self.SDistanceToDestination()
        if distance_destination > FLAGS_destination_check_distance:
            return
        
        decision_result.main_decision.task = MainMissionComplete()
        mission_complete: MainMissionComplete = decision_result.main_decision.task
        if self.ReachedDestination():
            planning_context.planning_status.destination.has_passed_destination = True
        else:
            mission_complete.stop_point = deepcopy(main_stop.stop_point)
            mission_complete.stop_heading = main_stop.stop_heading

    def MakeEStopDecision(self) -> DecisionResult:
        """
        Make E stop decision

        :returns: The decision result
        :rtype: DecisionResult
        """

        decision_result = DecisionResult(main_decision=MainDecision(task=MainEmergencyStop()))
        main_estop: MainEmergencyStop = decision_result.main_decision.task
        main_estop.reason_code = MainEmergencyStop.ReasonCode.ESTOP_REASON_INTERNAL_ERR
        main_estop.reason = "estop reason to be added"
        main_estop.task = EmergencyStopCruiseToStop()
        
        # set object decisions
        object_decisions: ObjectDecisions = decision_result.object_decision
        for obstacle in self._path_decision.obstacles:
            object_decision = ObjectDecision(id==obstacle.Id, perception_id=obstacle.PerceptionId,
                                             object_decision=[ObjectDecisionType(object_tag=ObjectAvoid())])
            object_decisions.decision.append(object_decision)
        
        return decision_result

    def SetObjectDecisions(self, decision_result: DecisionResult) -> None:
        """
        Set object decisions

        :returns: The object decisions
        :rtype: ObjectDecisions
        """

        object_decisions: ObjectDecisions = decision_result.object_decision
        for obstacle in self._path_decision.obstacles:
            if not obstacle.HasNonIgnoreDecision():
                continue
            object_decision = ObjectDecision(id=obstacle.Id, perception_id=obstacle.PerceptionId)
            if obstacle.HasLateralDecision() and (not obstacle.IsLateralIgnore()):
                object_decision.object_decision.append(deepcopy(obstacle.LateralDecision()))
            if obstacle.HasLongitudinalDecision() and (not obstacle.IsLongitudinalIgnore()):
                object_decision.object_decision.append(deepcopy(obstacle.LongitudinalDecision()))
            object_decisions.decision.append(object_decision)

    def AddObstacleHelper(self, obstacle: Obstacle) -> bool:
        """
        Add obstacle helper

        :param Obstacle obstacle: The obstacle to add
        :returns: True if successful, False otherwise
        :rtype: bool
        """

        return self.AddObstacle(obstacle) is not None
    
    def GetFirstOverlap(self, path_overlaps: List[PathOverlap]) -> Tuple[bool, PathOverlap]:
        """
        Get the first overlap

        :param List[PathOverlap] path_overlaps: The path overlaps
        :returns: (bool, The first overlap)
        :rtype: Tuple[bool, PathOverlap]
        """

        start_s: float = self._adc_sl_boundary.end_s
        kMaxOverlapRange: float = 500.0
        overlap_min_s: float = kMaxOverlapRange

        overlap_min_s_iter = path_overlaps[-1]
        for iter in path_overlaps:
            if iter.end_s < start_s:
                continue
            if overlap_min_s > iter.start_s:
                overlap_min_s_iter = iter
                overlap_min_s = iter.start_s

        # Ensure that the path_overlaps is not empty.
        if overlap_min_s_iter is not None:
            path_overlap = overlap_min_s_iter

        return (overlap_min_s < kMaxOverlapRange, path_overlap)

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
