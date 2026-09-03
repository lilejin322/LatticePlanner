from protoclass.trajectory_point import TrajectoryPoint
from typing import List, Dict, Optional, Any
from protoclass.adc_trajectory import ADCTrajectory, Point3D
from common.box2d import Box2d
from reference_line.reference_line_info import ReferenceLineInfo
from common.planning_context import PlanningContext
from common.obstacle import Obstacle
from protoclass.prediction_obstacles import PredictionObstacles
from logging import Logger
from protoclass.planning_status import ReroutingStatus, LaneFollowCommand
from protoclass.pose import Pose
from protoclass.point_enu import PointENU
from protoclass.lane_waypoint import LaneWaypoint
import config as config_module
from reference_line import ReferenceLine
from common.route_segments import RouteSegments
from common.vec2d import Vec2d
from protoclass.vehicle_state import VehicleState
from reference_line.reference_point import ReferencePoint
from common.lane_info import LaneInfo
from protoclass.sl_boundary import SLPoint
from common.line_segment2d import LineSegment2d
from protoclass.header import ErrorCode
from common.status import Status
from common.polygon2d import Polygon2d
from protoclass.debug import Debug
from dataclasses import dataclass, field
from copy import deepcopy
from datetime import datetime
import math
from enum import Enum
from protoclass.header import Header
from protoclass.chassis import Chassis
from protoclass.localization_estimate import LocalizationEstimate
from protoclass.traffic_light_detection import TrafficLightDetection, TrafficLight
from protoclass.planning_internal import MapMsg
from protoclass.stories import Stories
from protoclass.planning_command import PlanningCommand
from protoclass.routing import RoutingResponse
from common.discretized_path import DiscretizedPath
from common.vehicle_state_provider import VehicleStateProvider
from reference_line.reference_line_provider import ReferenceLineProvider
from common.hd_map import HDMapUtil

logger = Logger("Frame")


class _VehicleStateUtil:
    @staticmethod
    def IsVehicleStateValid(vehicle_state: VehicleState) -> bool:
        return vehicle_state is not None and vehicle_state.x is not None and vehicle_state.y is not None


util = _VehicleStateUtil()


@dataclass
class OpenSpaceInfo:
    """Minimal placeholder for Apollo open-space frame data."""


@dataclass
class EgoInfo:
    ego_box: Box2d


class _NoOpMonitor:
    PLANNING = "PLANNING"

    def ERROR(self, *_args, **_kwargs):
        return None


if "MonitorMessageItem" not in globals():
    MonitorMessageItem = _NoOpMonitor()

class DrivingAction(Enum):
    """Matches apollo.planning.DrivingAction in pad_msg.proto."""

    FOLLOW = 0
    CHANGE_LEFT = 1
    CHANGE_RIGHT = 2
    PULL_OVER = 3
    STOP = 4
    RESUME_CRUISE = 5
    NONE = 100

@dataclass
class PadMessage:
    """
    PadMessage class, oriented from protobuf message
    """

    header: Optional[Header] = None
    """control mode, set mode according to low level definition"""
    action: Optional[int] = None
    """action in the driving_mode"""

@dataclass
class PerceptionEdgeInfo:
    """
    PerceptionEdgeInfo class, oriented from protobuf message
    """

    header: Optional[Header] = None
    is_useable: Optional[bool] = False
    edge_point: List[Point3D] = field(default_factory=list)
    edge_relative_point: List[Point3D] = field(default_factory=list)
    delta_s: Optional[float] = 0.2
    edge_length: Optional[float] = None
    is_smoother_succ: Optional[bool] = False
    is_cross_localization: Optional[bool] = False

@dataclass
class LocalView:
    """
    LocalView contains all necessary data as planning input
    """

    prediction_obstacles: Optional[PredictionObstacles] = None
    chassis: Optional[Chassis] = None
    localization_estimate: Optional[LocalizationEstimate] = None
    traffic_light: Optional[TrafficLightDetection] = None
    relative_map: Optional[MapMsg] = None
    pad_msg: Optional[PadMessage] = None
    stories: Optional[Stories] = None
    planning_command: Optional[PlanningCommand] = None
    routing: Optional[RoutingResponse] = None
    end_lane_way_point: Optional[LaneWaypoint] = None
    perception_road_edge: Optional[PerceptionEdgeInfo] = None

class Frame:
    """
    Frame holds all data for one planning cycle.
    """

    _pad_msg_driving_action: DrivingAction = DrivingAction.NONE
    """in C++ code, this is a static member variable"""

    def __init__(self, *args):

        if len(args) == 1:
            """
            Constructor1

            :param int sequence_num: Sequence number
            """

            sequence_num: int = args[0]
            self._sequence_num = sequence_num
            self._monitor_logger_buffer = MonitorMessageItem
        
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
            reference_line_provider: ReferenceLineProvider = args[4]
            self._sequence_num = sequence_num
            self._local_view = local_view
            self._planning_start_point = planning_start_point
            self._vehicle_state = vehicle_state
            self._reference_line_provider = reference_line_provider
            self._monitor_logger_buffer = MonitorMessageItem

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
        self._obstacles: Dict[str, Obstacle] = {}
        self._traffic_lights: Dict[str, TrafficLight] = {}
        self._current_frame_planned_trajectory = None
        """current frame published trajectory"""
        self._current_frame_planned_path = None
        """current frame path for future possible speed fallback"""
        self._future_route_waypoints: List[LaneWaypoint] = []
        self._open_space_info: OpenSpaceInfo = None
        self._hdmap = None
        self._planning_context = PlanningContext()

    @property
    def planning_context(self) -> PlanningContext:
        return self._planning_context

    @property
    def hdmap(self):
        return self._hdmap

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
        TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.

        :param VehicleStateProvider vehicle_state_provider: Vehicle state provider
        :param List[ReferenceLine] reference_lines: Reference lines
        :param List[RouteSegments] segments: Route segments
        :param List[LaneWaypoint] future_route_waypoints: Future route waypoints
        :param EgoInfo ego_info: Ego info
        :returns: status
        :rtype: Status
        """
        
        status = self.InitFrameData(vehicle_state_provider, ego_info)
        if not status.ok():
            logger.error(f"Failed to init frame {status}")
            return status
        if not self.CreateReferenceLineInfo(reference_lines, segments):
            msg = "Failed to init reference line info."
            logger.error(msg)
            return Status(ErrorCode.PLANNING_ERROR, msg)
        self._future_route_waypoints = future_route_waypoints
        return Status.OK()

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
            if segment.StopForDestination():
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
        target_speed: float = config_module.FLAGS_default_cruise_speed
        if (
            self._local_view.planning_command is not None
            and self._local_view.planning_command.target_speed is not None
        ):
            target_speed = self._local_view.planning_command.target_speed
        has_valid_reference_line: bool = False
        ref_line_index: int = 0
        for ref_info in self._reference_line_info[:]:
            if ref_info.Init(self.obstacles, target_speed):
                has_valid_reference_line = True
                ref_info.set_index(ref_line_index)
                logger.info(
                    f"get referenceline: index: {ref_info.index}, id: {ref_info.Lanes().Id()}, key: {ref_info.Lanes().Id()}"
                )
                ref_line_index += 1
            else:
                self._reference_line_info.remove(ref_info)
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

        self._hdmap = HDMapUtil.BaseMapPtr()
        assert self.hdmap is not None, "HDMap is null"

        self._vehicle_state = vehicle_state_provider.vehicle_state
        if not util.IsVehicleStateValid(self._vehicle_state):
            logger.error("Adc init point is not set")
            return Status(ErrorCode.PLANNING_ERROR, "Adc init point is not set")

        logger.debug(f"Enabled align prediction time? : {config_module.FLAGS_align_prediction_time}")

        if config_module.FLAGS_align_prediction_time:
            prediction = deepcopy(self._local_view.prediction_obstacles)
            self.AlignPredictionTime(self._vehicle_state.timestamp, prediction)
            self._local_view.prediction_obstacles = prediction

        for obstacle in Obstacle.CreateObstacles(self._local_view.prediction_obstacles):
            self.AddObstacle(obstacle)

        if self._planning_start_point.v < 1e-3:
            collision_obstacle = self.FindCollisionObstacle(ego_info)
            if collision_obstacle is not None:
                msg = f"Found collision with obstacle: {collision_obstacle.Id()}"
                logger.error(msg)
                self._monitor_logger_buffer.ERROR(msg)
                return Status(ErrorCode.PLANNING_ERROR, msg)

        self.ReadTrafficLights()

        self.ReadPadMsgDrivingAction()

        return Status.OK()

    def InitForOpenSpace(self, vehicle_state_provider: VehicleStateProvider, ego_info: EgoInfo) -> Status:
        """
        Init for open space

        :param VehicleStateProvider vehicle_state_provider: Vehicle state provider
        :param EgoInfo ego_info: Ego info
        :returns: status
        :rtype: Status
        """
        
        return self.InitFrameData(vehicle_state_provider, ego_info)

    def FindCollisionObstacle(self, ego_info: EgoInfo) -> Obstacle:
        """
        Find an obstacle that collides with ADC (Autonomous Driving Car) if
        such obstacle exists.

        : param EgoInfo ego_info: Ego info
        : returns: the obstacle if such obstacle exists, otherwise None
        : rtype: Obstacle
        """

        if not self._obstacles.items():
            return None
        adc_polygon = Polygon2d(ego_info.ego_box)
        for _, obstacle in self._obstacles.items():
            if obstacle.IsVirtual():
                continue

            obstacle_polygon = obstacle.PerceptionPolygon()
            if obstacle_polygon.HasOverlap(adc_polygon):
                return obstacle
        return None

    def CreateStaticVirtualObstacle(self, id: str, box: Box2d) -> Obstacle:
        """
        Create static virtual obstacle

        :param str id: Obstacle id
        :param Box2d box: Box
        :returns: Obstacle
        :rtype: Obstacle
        """

        obj = self._obstacles.get(id)
        if obj is not None:
            logger.warning(f"obstacle {id} already exist.")
            return obj
        new_obj = Obstacle.CreateStaticVirtualObstacles(id, box)
        if new_obj is None:
            logger.error(f"Failed to create obstacle {id}")
            return None
        self._obstacles[id] = new_obj 
        return new_obj

    def AddObstacle(self, obstacle: Obstacle) -> None:
        """
        Add obstacle

        :param Obstacle obstacle: Obstacle
        """

        if obstacle is None:
            logger.error("Obstacle is None")
            return
        
        if obstacle.Id() in self._obstacles:
            raise KeyError(f"Obstacle with ID {obstacle.Id()} already exists.")
        
        self._obstacles[obstacle.Id()] = obstacle

    def ReadTrafficLights(self) -> None:
        """
        Read traffic lights
        """

        self._traffic_lights.clear()
        traffic_light_detection = self._local_view.traffic_light
        if traffic_light_detection is None:
            return
        delay: float = traffic_light_detection.header.timestamp_sec - datetime.now().timestamp()
        if delay > config_module.FLAGS_signal_expire_time_sec:
            logger.debug(f"traffic signals msg is expired, delay = {delay} seconds.")
            return
        for traffic_light in traffic_light_detection.traffic_light:
            self._traffic_lights[traffic_light.id] = traffic_light

    def ReadPadMsgDrivingAction(self) -> None:
        """
        Read pad message driving action
        """

        if self._local_view.pad_msg is not None:
            if self._local_view.pad_msg.action is not None:
                self._pad_msg_driving_action = self._local_view.pad_msg.action

    def ResetPadMsgDrivingAction(self) -> None:
        """
        Reset pad message driving action
        """

        self._pad_msg_driving_action = DrivingAction.NONE

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

        return f"Frame: {self._sequence_num}"

    def ComputedTrajectory(self) -> Any:
        """
        Get computed trajectory

        :returns: Computed trajectory
        :rtype: PublishableTrajectory
        """

        return self._current_frame_planned_trajectory

    def RecordInputDebug(self, debug: Debug) -> None:
        """
        Record input debug

        :param Debug debug: Debug
        """

        if debug is None:
            logger.error("Skip record input into debug")
            return
        planning_debug_data = debug.planning_data
        planning_debug_data.adc_position = self._local_view.localization_estimate
        planning_debug_data.chassis = self._local_view.chassis

        if not config_module.FLAGS_use_navigation_mode and self._local_view.routing is not None:
            planning_debug_data.routing = self._local_view.routing

        planning_debug_data.prediction_header = self._local_view.prediction_obstacles.header

    @property
    def reference_line_info(self) -> List[ReferenceLineInfo]:
        """
        Get reference line info

        :returns: Reference line info
        :rtype: List[ReferenceLineInfo]
        """

        return self._reference_line_info

    @property
    def mutable_reference_line_info(self) -> List[ReferenceLineInfo]:
        """
        Apollo-compatible mutable reference line info accessor.
        """

        return self._reference_line_info

    def Find(self, id: str) -> Obstacle:
        """
        Find obstacle by id

        :param str id: Obstacle id
        :returns: Obstacle
        :rtype: Obstacle
        """

        return self._obstacles.get(id)

    def FindDriveReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Find drive reference line info

        :returns: Drive reference line info
        :rtype: ReferenceLineInfo
        """

        min_cost: float = math.inf
        self._drive_reference_line_info = None
        for reference_line_info in self._reference_line_info:
            if reference_line_info.IsDrivable() and reference_line_info.Cost() < min_cost:
                self._drive_reference_line_info = reference_line_info
                min_cost = reference_line_info.Cost()
        return self._drive_reference_line_info

    def FindTargetReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Find target reference line info

        :returns: Target reference line info
        :rtype: ReferenceLineInfo
        """

        target_reference_line_info = None
        for reference_line_info in self._reference_line_info:
            if reference_line_info.IsChangeLanePath():
                return reference_line_info
            target_reference_line_info = reference_line_info
        return target_reference_line_info

    def FindFailedReferenceLineInfo(self) -> ReferenceLineInfo:
        """
        Find failed reference line info

        :returns: Failed reference line info
        :rtype: ReferenceLineInfo
        """

        for reference_line_info in self._reference_line_info:
            # Find the unsuccessful lane-change path
            if not reference_line_info.IsDrivable() and reference_line_info.IsChangeLanePath():
                return reference_line_info
        return None

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

        return list(self._obstacles.values())

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
            :returns: Obstacle
            :rtype: Obstacle
            """

            reference_line_info: ReferenceLineInfo = args[0]
            obstacle_id: str = args[1]
            obstacle_s: float = args[2]
            if reference_line_info is None:
                logger.error("reference_line_info is None")
                return None
            reference_line: ReferenceLine = reference_line_info.reference_line
            box_center_s: float = obstacle_s + config_module.FLAGS_virtual_stop_wall_length / 2.0
            box_center: ReferencePoint = reference_line.GetReferencePoint(box_center_s)
            heading: float = reference_line.GetReferencePoint(obstacle_s).heading
            kStopWallWidth: float = 4.0
            stop_wall_box: Box2d = Box2d(box_center, heading, config_module.FLAGS_virtual_stop_wall_length, kStopWallWidth)

            return self.CreateStaticVirtualObstacle(obstacle_id, stop_wall_box)

        if isinstance(args[0], str):
            """
            CreateStopObstacle_2

            :param str obstacle_id: Obstacle id
            :param str lane_id: Lane id
            :param float lane_s: Lane s
            :returns: Obstacle
            :rtype: Obstacle
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
                                         config_module.FLAGS_virtual_stop_wall_length, lane_left_width + lane_right_width)

            return self.CreateStaticVirtualObstacle(obstacle_id, stop_wall_box)

    def CreateStaticObstacle(self, reference_line_info: ReferenceLineInfo, obstacle_id: str,
                             obstacle_start_s: float, obstacle_end_s: float) -> Obstacle:
        """
        create static virtual object with lane width

        :param ReferenceLineInfo reference_line_info: Reference line info
        :param str obstacle_id: Obstacle id
        :param float obstacle_start_s: Obstacle start s
        :param float obstacle_end_s: Obstacle end s
        :returns: Obstacle
        :rtype: Obstacle
        """

        if reference_line_info is None:
            logger.error("reference_line_info is None")
            return None
        
        reference_line: ReferenceLine = reference_line_info.reference_line

        # start_xy
        sl_point: SLPoint = SLPoint(s=obstacle_start_s, l=0.0)
        tag, obstacle_start_xy = reference_line.SLToXY(sl_point)
        if not tag:
            logger.error(f"Failed to get start_xy from sl: {sl_point}")
            return None
        
        # end_xy
        sl_point = SLPoint(s=obstacle_end_s, l=0.0)
        tag, obstacle_end_xy = reference_line.SLToXY(sl_point)
        if not tag:
            logger.error(f"Failed to get end_xy from sl: {sl_point}")
            return None
        
        tag, left_lane_width, right_lane_width = reference_line.GetLaneWidth(obstacle_start_s)
        if not tag:
            logger.error(f"Failed to get lane width at s[{obstacle_start_s}]")
            return None
        
        obstacle_box: Box2d = Box2d(LineSegment2d(obstacle_start_xy, obstacle_end_xy), left_lane_width + right_lane_width)

        return self.CreateStaticVirtualObstacle(obstacle_id, obstacle_box)

    def Rerouting(self, planning_context: PlanningContext) -> bool:
        """
        Rerouting process

        :param PlanningContext planning_context: Planning context
        :returns: Rerouting result
        :rtype: bool
        """

        if config_module.FLAGS_use_navigation_mode:
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

        if not prediction_obstacles or not prediction_obstacles.header or \
            not prediction_obstacles.header.timestamp_sec:
            return

        prediction_header_time: float = prediction_obstacles.header.timestamp_sec
        for obstacle in prediction_obstacles.prediction_obstacle:
            for trajectory in obstacle.trajectory:
                for point in trajectory.trajectory_point:
                    point.relative_time = prediction_header_time + point.relative_time - planning_start_time
                
                first_non_negative = 0
                while (first_non_negative < len(trajectory.trajectory_point) and
                       trajectory.trajectory_point[first_non_negative].relative_time < 0):
                    first_non_negative += 1
                if first_non_negative:
                    del trajectory.trajectory_point[:first_non_negative]

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

    def set_current_frame_planned_path(self, current_frame_planned_path: DiscretizedPath) -> None:
        """
        Set current frame planned path

        :param DiscrtizedPath current_frame_planned_path: Current frame planned path
        """

        self._current_frame_planned_path = current_frame_planned_path

    @property
    def current_frame_planned_path(self) -> DiscretizedPath:
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

    def ApplyTrafficRules(self, traffic_decider=None, planning_context=None) -> None:
        """Run traffic rules and populate planning_target.stop_point for lattice."""
        from traffic_rules.traffic_decider import TrafficDecider

        decider = traffic_decider or TrafficDecider(planning_context)
        decider.Init(planning_context=planning_context)
        for ref_info in self._reference_line_info:
            decider.Execute(self, ref_info)

    def ApplyPathDecider(self, path_decider=None, planning_context=None) -> None:
        """Run path decider for static obstacle nudge/stop decisions."""
        from common.path_decider import PathDecider

        for ref_info in self._reference_line_info:
            if ref_info.path_data is None:
                continue
            decider = path_decider or PathDecider(ref_info)
            decider.Execute(ref_info, planning_context)

    def UpdateReferenceLinePriority(self, id_to_priority: Dict[str, int]) -> None:
        """
        Adjust reference line priority according to actual road conditions

        :param Dict[str, int] id_to_priority: lane id and reference line priority mapping relationship
        """

        for id, priority in id_to_priority.items():
            ref_line_info_itr = next(
                (ref_line_info for ref_line_info in self._reference_line_info 
                 if ref_line_info.Lanes().Id() == id), None)
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

    def GetObstacleList(self) -> List[Obstacle]:
        """
        Get obstacle list

        :returns: Obstacle list
        :rtype: List[Obstacle]
        """

        return list(self._obstacles.values())

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

        result = self._traffic_lights.get(traffic_light_id)
        if result is None:
            return TrafficLight(id=traffic_light_id, color=TrafficLight.Color.UNKNOWN, confidence=0.0, tracking_time=0.0)
        return result

    def GetPadMsgDrivingAction(self) -> DrivingAction:
        """
        Get pad message driving action

        :returns: Pad message driving action
        :rtype: DrivingAction
        """

        return self._pad_msg_driving_action
