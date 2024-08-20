from protoclass.TrajectoryPoint import TrajectoryPoint
from typing import List, Dict
from protoclass.ADCTrajectory import ADCTrajectory
from common.Box2d import Box2d
from common.ReferenceLineInfo import ReferenceLineInfo
from common.PlanningContext import PlanningContext
from common.Obstacle import Obstacle
from protoclass.PredictionObstacles import PredictionObstacles
from logging import Logger
from protoclass.PlanningStatus import ReroutingStatus, LaneFollowCommand
from protoclass.Pose import Pose
from protoclass.PointENU import PointENU
from protoclass.LaneWaypoint import LaneWaypoint
from config import FLAGS_use_navigation_mode
from common.ReferenceLine import ReferenceLine
from common.RouteSegments import RouteSegments
from common.Vec2d import Vec2d
from protoclass.VehicleState import VehicleState
from config import FLAGS_default_cruise_speed, FLAGS_virtual_stop_wall_length
from common.ReferencePoint import ReferencePoint
from common.LaneInfo import LaneInfo

logger = Logger("Frame")

class Frame:
    """
    Frame holds all data for one planning cycle.
    """

    _pad_msg_driving_action: DrivingAction = None
    """in C++ code, this is a static member variable"""

    def __init__(self, *args):

        if len(args) == 1:
            """
            Constructor1

            :param int sequence_num: Sequence number
            """

            sequence_num: int = args[0]
            self._sequence_num = sequence_num
            self._monitor_logger_buffer = MonitorMessageItem.PLANNING
        
        elif len(args) == 5:
            """
            Constructor2

            :param int sequence_num: Sequence number
            :param LocalView local_view: Local view
            :param TrajectoryPoint planning_start_point: Planning start point
            :param VehicleState vehicle_state: Vehicle state
            :param ReferenceLineProvider reference_line_provider: Reference line provider
            """

            sequence_num: int = args[0]
            local_view: LocalView = args[1]
            planning_start_point: TrajectoryPoint = args[2]
            vehicle_state: VehicleState = args[3]
            reference_line_provider: RefereneceLineProvider = args[4]
            self._sequence_num = sequence_num
            self._local_view = local_view
            self._planning_start_point = planning_start_point
            self._vehicle_state = vehicle_state
            self._reference_line_provider = reference_line_provider
            self._monitor_logger_buffer = MonitorMessageItem.PLANNING

        elif len(args) == 4:
            """
            Constructor3

            :param int sequence_num: Sequence number
            :param LocalView local_view: Local view
            :param TrajectoryPoint planning_start_point: Planning start point
            :param VehicleState vehicle_state: Vehicle state
            """

            sequence_num: int = args[0]
            local_view: LocalView = args[1]
            planning_start_point: TrajectoryPoint = args[2]
            vehicle_state: VehicleState = args[3]
            self.__init__(sequence_num, local_view, planning_start_point, vehicle_state, None)

        self._reference_line_info: List[ReferenceLineInfo] = []
        self._is_near_destination = False
        self._drive_reference_line_info: ReferenceLineInfo = None
        """the reference line info that the vehicle finally choose to drive on"""
        self._obstacles: List[Obstacles] = []
        self._traffic_lights: Dict[str, TrafficLight] = {}
        self._current_frame_planned_trajectory = None
        """current frame published trajectory"""
        self._current_frame_planned_path = None
        """current frame path for future possible speed fallback"""
        self._future_route_waypoints: List[LaneWaypoint] = []
        self._open_space_info: OpenSpaceInfo = None
        self._hdmap = None

    @property
    def PlanningStartPoint(self) -> TrajectoryPoint:
        """
        Get planning start point

        :return TrajectoryPoint: Planning start point
        :rtype: TrajectoryPoint
        """

        return self._planning_start_point

    def Init(self, vehicle_state_provider: VehicleStateProvider, reference_lines: List[ReferenceLine], segments: List[RouteSegments], 
             future_route_waypoints: List[LaneWaypoint], ego_info: EgoInfo) -> Status:
        """
        Init frame

        :param VehicleStateProvider vehicle_state_provider: Vehicle state provider
        :param List[ReferenceLine] reference_lines: Reference lines
        :param List[RouteSegments] segments: Route segments
        :param List[LaneWaypoint] future_route_waypoints: Future route waypoints
        :param EgoInfo ego_info: Ego info
        :returns: status
        :rtype: Status
        """
        
        raise NotImplementedError

    def CreateReferenceLineInfo(self, reference_lines: List[ReferenceLine], segments: List[RouteSegments]) -> bool:
        """
        Create reference line info

        :param List[ReferenceLine] reference_lines: Reference lines
        :param List[RouteSegments] segments: Route segments
        :returns: True if success, False otherwise
        :rtype: bool
        """

        self._reference_line_info.clear()
        if len(reference_lines) == 0:
            return True
        ref_line_iter = iter(reference_lines)
        segments_iter = iter(segments)
        ref_line_index = 0
        while True:
            try:
                ref_line = next(ref_line_iter)
                segment = next(segments_iter)
            except StopIteration:
                break
            if segment.StopForDestination:
                self._is_near_destination = True
            self._reference_line_info.append(ReferenceLineInfo(self._vehicle_state, self._planning_start_point, ref_line, segment))
            self._reference_line_info[-1].set_index(ref_line_index)
            ref_line_index += 1
        
        if len(self._reference_line_info) == 2:
            xy_point: Vec2d = Vec2d(x=self._vehicle_state.x, y=self._vehicle_state.y)
            tag, first_sl = self._reference_line_info[0].reference_line.XYToSL(xy_point)
            if not tag:
                return False
            tag, second_sl = self._reference_line_info[-1].reference_line.XYToSL(xy_point)
            if not tag:
                return False
            offset: float = first_sl.l - second_sl.l
            self._reference_line_info[0].SetOffsetToOtherReferenceLine(offset)
            self._reference_line_info[-1].SetOffsetToOtherReferenceLine(-offset)
        target_speed: float = FLAGS_default_cruise_speed
        if self._local_view.planning_command.target_speed is not None:
            target_speed = self._local_view.planning_command.target_speed
        has_valid_reference_line: bool = False
        ref_line_index: int = 0
        for ref_info in self._reference_line_info[:]:
            if ref_info.Init(self.obstacles, target_speed):
                self._reference_line_info.remove(ref_info)
            else:
                has_valid_reference_line = True
                ref_info.set_index(ref_line_index)
                logger.info(f"get referenceline: index: {ref_info.index}, id: {ref_info.id}, key: {ref_info.key}")
                ref_line_index += 1
        if not has_valid_reference_line:
            logger.info("No valid reference line")
        return True

    def InitFrameData(self, vehicle_state_provider: VehicleStateProvider, ego_info: EgoInfo) -> Status:
        """
        Init frame data

        :param VehicleStateProvider vehicle_state_provider: Vehicle state provider
        :param EgoInfo ego_info: Ego info
        :returns: status
        :rtype: Status
        """

        raise NotImplementedError

    def InitForOpenSpace(self, vehicle_state_provider: VehicleStateProvider, ego_info: EgoInfo) -> Status:
        """
        Init for open space

        :param VehicleStateProvider vehicle_state_provider: Vehicle state provider
        :param EgoInfo ego_info: Ego info
        :returns: status
        :rtype: Status
        """
        
        raise NotImplementedError

    def FindCollisionObstacle(self, ego_info: EgoInfo) -> Obstacle:
        """
        Find an obstacle that collides with ADC (Autonomous Driving Car) if
        such obstacle exists.

        : param EgoInfo ego_info: Ego info
        : returns: the obstacle if such obstacle exists, otherwise None
        : rtype: Obstacle
        """

        raise NotImplementedError

    def CreateStaticVirtualObstacle(self, id: str, box: Box2d) -> Obstacle:
        """
        Create static virtual obstacle

        :param str id: Obstacle id
        :param Box2d box: Box
        :returns: Obstacle
        :rtype: Obstacle
        """

        raise NotImplementedError

    def AddObstacle(self, obstacle: Obstacle) -> None:
        """
        Add obstacle

        :param Obstacle obstacle: Obstacle
        """

        raise NotImplementedError

    def ReadTrafficLights(self) -> None:
        """
        Read traffic lights
        """

        raise NotImplementedError

    def ReadPadMsgDrivingAction(self) -> None:
        """
        Read pad message driving action
        """

        raise NotImplementedError

    def ResetPadMsgDrivingAction(self) -> None:
        """
        Reset pad message driving action
        """

        raise NotImplementedError

    @property
    def SequenceNum(self) -> int:
        """
        Get sequence number

        :returns: Sequence number
        :rtype: int
        """

        return self._sequence_num

    def __str__(self) -> str:
        """
        Get string representation

        :returns: Debug string representation
        :rtype: str
        """

        raise NotImplementedError

    def ComputedTrajectory(self) -> PublishableTrajectory:
        """
        Get computed trajectory

        :returns: Computed trajectory
        :rtype: PublishableTrajectory
        """

        raise NotImplementedError

    def RecordInputDebug(self, debug: Debug) -> None:
        """
        Record input debug

        :param Debug debug: Debug
        """
        
        raise NotImplementedError

    @property
    def reference_line_info(self) -> List[ReferenceLineInfo]:
        """
        Get reference line info

        :returns: Reference line info
        :rtype: List[ReferenceLineInfo]
        """

        return self._reference_line_info

    def Find(self, id: str) -> Obstacle:
        """
        Find obstacle by id

        :param str id: Obstacle id
        :returns: Obstacle
        :rtype: Obstacle
        """

        raise NotImplementedError

    def FindDriveReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Find drive reference line info

        :returns: Drive reference line info
        :rtype: ReferenceLineInfo
        """

        raise NotImplementedError

    def FindTargetReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Find target reference line info

        :returns: Target reference line info
        :rtype: ReferenceLineInfo
        """

        raise NotImplementedError

    def FindFailedReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Find failed reference line info

        :returns: Failed reference line info
        :rtype: ReferenceLineInfo
        """

        raise NotImplementedError

    @property
    def DriveReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Get drive reference line info

        :returns: Drive reference line info
        :rtype: ReferenceLineInfo
        """

        return self._drive_reference_line_info

    @property
    def obstacles(self) -> List[Obstacle]:
        """
        Get obstacles

        :returns: Obstacles
        :rtype: List[Obstacle]
        """

        return self._obstacles

    def CreateStopObstacle(self, *args) -> Obstacle:
        """
        create static virtual object with lane width, mainly used for virtual stop wall
        """

        if isinstance(args[0], ReferenceLineInfo):
            """
            CreateStopObstacle_1
            
            :param ReferenceLineInfo reference_line_info: Reference line info
            :param str obstacle_id: Obstacle id
            :param float obstacle_s: Obstacle s
            """

            reference_line_info: ReferenceLineInfo = args[0]
            obstacle_id: str = args[1]
            obstacle_s: float = args[2]
            if reference_line_info is None:
                logger.error("reference_line_info is None")
                return None
            reference_line: ReferenceLine = reference_line_info.reference_line
            box_center_s: float = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0
            box_center: ReferencePoint = reference_line.GetReferencePoint(box_center_s)
            heading: float = reference_line.GetReferencePoint(obstacle_s).heading
            kStopWallWidth: float = 4.0
            stop_wall_box: Box2d = Box2d(box_center, heading, FLAGS_virtual_stop_wall_length, kStopWallWidth)

            return self.CreateStaticVirtualObstacle(obstacle_id, stop_wall_box)

        if isinstance(args[0], str):
            """
            CreateStopObstacle_2

            :param str obstacle_id: Obstacle id
            :param str lane_id: Lane id
            :param float lane_s: Lane s
            """

            obstacle_id: str = args[0]
            lane_id: str = args[1]
            lane_s: float = args[2]
            if self._hdmap is None:
                logger.error("Invalid HD Map.")
                return None
            lane: LaneInfo = None
            if self._reference_line_provider is None:
                lane = self._hdmap.GetLaneById(lane_id)
            else:
                lane = self._reference_line_provider.GetLaneById(lane_id)
            if lane is None:
                logger.error(f"Failed to find lane {lane_id}")
                return None
            
            dest_lane_s: float = max(0.0, lane_s)
            dest_point = lane.GetSmoothPoint(dest_lane_s)

            lane_left_width, lane_right_width, _ = lane.GetWidth(dest_lane_s)
            stop_wall_box: Box2d = Box2d(Vec2d(dest_point.x, dest_point.y), lane.Heading(dest_lane_s),
                                         FLAGS_virtual_stop_wall_length, lane_left_width + lane_right_width)

            return self.CreateStaticVirtualObstacle(obstacle_id, stop_wall_box)

    def CreateStaticObstacle(self, reference_line_info: ReferenceLineInfo, obstacle_id: str,
                             obstacle_start_s: float, obstacle_end_s: float) -> Obstacle:
        """
        Create static obstacle

        :param ReferenceLineInfo reference_line_info: Reference line info
        :param str obstacle_id: Obstacle id
        :param float obstacle_start_s: Obstacle start s
        :param float obstacle_end_s: Obstacle end s
        :returns: Obstacle
        :rtype: Obstacle
        """

        raise NotImplementedError

    def Rerouting(self, planning_context: PlanningContext) -> bool:
        """
        Rerouting process

        :param PlanningContext planning_context: Planning context
        :returns: Rerouting result
        :rtype: bool
        """

        if FLAGS_use_navigation_mode:
            logger.error("Rerouting not supported in navigation mode")
            return False
        if self._local_view.planning_command is None:
            logger.error("No previous routing available")
            return False
        if self._hdmap is None:
            logger.error("Invalid HD Map.")
            return False
        rerouting: ReroutingStatus = planning_context.planning_status.rerouting
        rerouting.need_rerouting = True
        lane_follow_command: LaneFollowCommand = rerouting.lane_follow_command
        if len(self._future_route_waypoints) < 1:
            logger.error("Failed to find future waypoints")
            return False
        for i in range(len(self._future_route_waypoints) - 1):
            waypoint: Pose = Pose(position=PointENU(x=self._future_route_waypoints[i].pose.x,
                                                    y=self._future_route_waypoints[i].pose.y),
                                                    heading=self._future_route_waypoints[i].heading)
            lane_follow_command.way_point.append(waypoint)
        end_pose: Pose = lane_follow_command.end_pose
        end_pose.position.x = self._future_route_waypoints[-1].pose.x
        end_pose.position.y = self._future_route_waypoints[-1].pose.y
        end_pose.heading = self._future_route_waypoints[-1].heading

        self._monitor_logger_buffer.INFO("Planning send Routing request")
        return True

    @property
    def vehicle_state(self) -> VehicleState:
        """
        Get vehicle state

        :returns: Vehicle state
        :rtype: VehicleState
        """

        return self._vehicle_state

    @staticmethod
    def AlignPredictionTime(planning_start_time: float, prediction_obstacles: PredictionObstacles) -> None:
        """
        Align prediction time

        :param float planning_start_time: Planning start time
        :param PredictionObstacles prediction_obstacles: Prediction obstacles
        """

        raise NotImplementedError

    def set_current_frame_planned_trajectory(self, current_frame_planned_trajectory: ADCTrajectory) -> None:
        """
        Set current frame planned trajectory

        :param ADCTrajectory current_frame_planned_trajectory: Current frame planned trajectory
        """

        self._current_frame_planned_trajectory = current_frame_planned_trajectory

    @property
    def current_frame_planned_trajectory(self) -> ADCTrajectory:
        """
        Get current frame planned trajectory
        """

        return self._current_frame_planned_trajectory

    def set_current_frame_planned_path(self, current_frame_planned_path: DiscrtizedPath) -> None:
        """
        Set current frame planned path

        :param DiscrtizedPath current_frame_planned_path: Current frame planned path
        """

        self._current_frame_planned_path = current_frame_planned_path

    @property
    def current_frame_planned_path(self) -> DiscrtizedPath:
        """
        Get current frame planned path

        :returns: Current frame planned path
        :rtype: DiscrtizedPath
        """

        return self._current_frame_planned_path

    @property
    def is_near_destination(self) -> bool:
        """
        Get is near destination

        :returns: Is near destination
        :rtype: bool
        """

        return self._is_near_destination

    def UpdateReferenceLinePriority(self, id_to_priority: Dict[str, int]) -> None:
        """
        Adjust reference line priority according to actual road conditions

        :param Dict[str, int] id_to_priority: lane id and reference line priority mapping relationship
        """

        for id, priority in id_to_priority.items():
            ref_line_info_itr = next(
                (ref_line_info for ref_line_info in self._reference_line_info 
                 if ref_line_info.Lanes.Id() == id), None)
            if ref_line_info_itr is not None:
                ref_line_info_itr.SetPriority(priority)

    @property
    def local_view(self) -> LocalView:
        """
        Get local view

        :returns: Local view
        :rtype: LocalView
        """

        return self._local_view

    @property
    def GetObstacleList(self) -> IndexedObstacles:
        """
        Get obstacle list，这个函数我认为可以删掉在py里面，先放在这

        :returns: Obstacle list
        :rtype: IndexedObstacles
        """

        return self._obstacles

    @property
    def open_space_info(self) -> OpenSpaceInfo:
        """
        Get open space info

        :returns: Open space info
        :rtype: OpenSpaceInfo
        """

        return self._open_space_info

    def GetSignal(self, traffic_light_id: str) -> TrafficLight:
        """
        Get signal

        :param str traffic_light_id: Traffic light id
        :returns: Signal
        :rtype: TrafficLight
        """

        raise NotImplementedError

    @property
    def GetPadMsgDrivingAction(self) -> DrivingAction:
        """
        Get pad message driving action

        :returns: Pad message driving action
        :rtype: DrivingAction
        """

        return self._pad_msg_driving_action

class FrameHistory(IndexedQueue):
    """
    Frame history
    """

    def __init__(self):
        """
        Constructor
        """

        raise NotImplementedError
