from protoclass.TrajectoryPoint import TrajectoryPoint
from typing import List, Dict
from protoclass.ADCTrajectory import ADCTrajectory
from common.Box2d import Box2d
from common.ReferenceLineInfo import ReferenceLineInfo
from common.PlanningContext import PlanningContext
from common.Obstacle import Obstacle
from protoclass.PredictionObstacles import PredictionObstacles

class Frame:
    """
    Frame holds all data for one planning cycle.
    """

    _pad_msg_driving_action: DrivingAction = None
    """in C++ code, this is a static member variable"""

    def __init__(self, sequence_num: int, local_view: LocalView, planning_start_point: TrajectoryPoint, 
                 vehicle_state: VehicleState, reference_line_provider: ReferenceLineProvider):
        """
        Constructor

        :param LocalView local_view: Local view
        :param TrajectoryPoint planning_start_point: Planning start point
        :param VehicleState vehicle_state: Vehicle state
        :param ReferenceLineProvider reference_line_provider: Reference line provider
        """

        self._sequence_num = sequence_num
        self._local_view = local_view
        self._planning_start_point = planning_start_point
        self._vehicle_state = vehicle_state
        self._reference_line_provider = reference_line_provider
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
        self._monitor_logger_buffer: MonitorLogBuffer = None
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

        raise NotImplementedError

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

    def CreateStopObstacle(self, reference_line_info: ReferenceLineInfo, obstacle_id: str, obstacle_s: float) -> Obstacle:
        """
        Create stop obstacle

        :param ReferenceLineInfo reference_line_info: Reference line info
        :param str obstacle_id: Obstacle id
        :param float obstacle_s: Obstacle s
        :returns: Obstacle
        :rtype: Obstacle
        """

        raise NotImplementedError

    def CreateStopObstacle2(self, obstacle_id: str, lane_id: str, lane_s: float) -> Obstacle:
        """
        要注意，这块重载要着重小心重写
        """

        raise NotImplementedError
        

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

        raise NotImplementedError

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

        raise NotImplementedError

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
