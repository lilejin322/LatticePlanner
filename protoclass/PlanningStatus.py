from dataclasses import dataclass
from typing import Optional, List
from enum import Enum
from protoclass.PointENU import PointENU
from protoclass.Header import Header
from protoclass.Pose import Pose

@dataclass
class ChangeLaneStatus:
    """
    ChangeLaneStatus class, oriented from protobuf message
    """

    class ChangeLaneStatusEnum(Enum):

        IN_CHANGE_LANE = 1              # during change lane state
        CHANGE_LANE_FAILED = 2          # change lane failed
        CHANGE_LANE_FINISHED = 3        # change lane finished

    status: Optional[ChangeLaneStatusEnum] = None
    # the id of the route segment that the vehicle is driving on
    path_id: Optional[str] = None
    # the time stamp when the state started.
    timestamp: Optional[float] = None

@dataclass
class CreepDeciderStatus:
    """
    CreepDeciderStatus class, oriented from protobuf message
    """

    # Counter for creep clearance
    creep_clear_counter: Optional[int] = None

@dataclass
class StopTime:
    """
    StopTime class, oriented from protobuf message
    """

    # Obstacle id stopping the vehicle
    obstacle_id: Optional[str] = None
    # the timestamp when start stopping for the crosswalk
    stop_timestamp_sec: Optional[float] = None

@dataclass
class CrosswalkStatus:
    """
    CrosswalkStatus class, oriented from protobuf message
    """

    # Id of crosswalk
    crosswalk_id: Optional[str] = None
    # the timestamp when start stopping for the crosswalk
    stop_time: List[StopTime] = None
    # Crosswalks already passed or finished
    finished_crosswalk: List[str] = None

@dataclass
class DestinationStatus:
    """
    DestinationStatus class, oriented from protobuf message
    """

    # If vehicle has passed destination
    has_passed_destination: Optional[bool] = False

@dataclass
class EmergencyStopStatus:
    """
    EmergencyStopStatus class, oriented from protobuf message
    """

    # Stop fence point for emergency stop
    stop_fence_point: Optional[PointENU] = None

@dataclass
class OpenSpaceStatus:
    """
    OpenSpaceStatus class, oriented from protobuf message
    """

    # History of partitioned trajectories index
    partitioned_trajectories_index_history: List[str] = None
    # If position has been initialized.
    position_init: Optional[bool] = False

@dataclass
class ParkAndGoStatus:
    """
    ParkAndGoStatus class, oriented from protobuf message
    """

    # Initial position of vehicle
    adc_init_position: Optional[PointENU] = None
    # Initial heading of vehicle
    adc_init_heading: Optional[float] = None
    # If current stage is "ParkAndGoStageCheck"
    in_check_stage: Optional[bool] = None
    # Mapped point on reference line near intial position of vehicle, which is
    # used as end pose of openspace algorithm.
    adc_adjust_end_pose: Optional[PointENU] = None

@dataclass
class PathDeciderStatus:
    """
    PathDeciderStatus class, oriented from protobuf message
    """

    # Cycle counter of front static obstacle's existance
    front_static_obstacle_cycle_counter: Optional[int] = 0
    # If vehicle is in lane borrow scenario
    is_in_path_lane_borrow_scenario: Optional[bool] = False
    # Blocking obstacle id in front of the vehicle
    front_static_obstacle_id: Optional[str] = ""
    # If vehicle left borrow
    left_borrow: Optional[bool] = False
    # If vehicle right borrow
    right_borrow: Optional[bool] = False

@dataclass
class PullOverStatus:
    """
    PullOverStatus class, oriented from protobuf message
    """

    class PullOverType(Enum):

        PULL_OVER = 1                          # pull-over upon destination arrival
        EMERGENCY_PULL_OVER = 2                # emergency pull-over

    # Current pullover type
    pull_over_type: Optional[PullOverType] = None
    # Indicate if current path is pull over path
    plan_pull_over_path: Optional[bool] = False
    # Position of pull over
    position: Optional[PointENU] = None
    # Heading of pull over
    theta: Optional[float] = None
    # Front length for pull over region
    length_front: Optional[float] = None
    # Back length for pull over region
    length_back: Optional[float] = None
    # Left width for pull over region
    width_left: Optional[float] = None
    # Right width for pull over region
    width_right: Optional[float] = None

@dataclass
class LaneSegment:
    """
    LaneSegment class, oriented from protobuf message
    """

    # lane id which this LaneSegment belongs to.
    id: Optional[str] = None
    # Start s of this LaneSegment on the lane.
    start_s: Optional[float] = None
    # End s of this LaneSegment on the lane.
    end_s: Optional[float] = None

@dataclass
class LaneFollowCommand:
    """
    LaneFollowCommand class, oriented from protobuf message
    """

    header: Optional[Header] = None
    # Unique identification for command.
    command_id: Optional[int] = -1
    # If the start pose is set as the first point of "way_point".
    is_start_pose_set: Optional[bool] = False
    # The points between "start_pose" and "end_pose".
    way_point: List[Pose] = None
    # End pose of the lane follow command, must be given.
    end_pose: Pose = None
    # The lane segments which should not be passed by.
    blacklisted_lane: List[LaneSegment] = None
    # The road which should not be passed by.
    blacklisted_road: List[str] = None
    # Expected speed when executing this command. If "target_speed" > maximum
    # speed of the vehicle, use maximum speed of the vehicle instead. If it is
    # not given, the default target speed of system will be used.
    target_speed: Optional[float] = None

@dataclass
class ReroutingStatus:
    """
    ReroutingStatus class, oriented from protobuf message
    """

    # Time of last rerouting
    last_rerouting_time: Optional[float] = None
    # If rerouting need to be done
    need_rerouting: Optional[bool] = False
    # Last received planning command
    lane_follow_command: Optional[LaneFollowCommand] = None

@dataclass
class SpeedDeciderStatus:
    """
    SpeedDeciderStatus class, oriented from protobuf message
    """

    # Time of stopping at pedestrian
    pedestrian_stop_time: List[StopTime] = None

@dataclass
class ScenarioStatus:
    """
    ScenarioStatus class, oriented from protobuf message
    """

    # Current scenario type
    scenario_type: Optional[str] = None
    # Current stage type
    stage_type: Optional[str] = None

@dataclass
class StopSignStatus:
    """
    StopSignStatus class, oriented from protobuf message
    """

    # Id of current stop sign overlap 
    current_stop_sign_overlap_id: Optional[str] = None
    # Id of just finished stop sign overlap
    done_stop_sign_overlap_id: Optional[str] = None
    # Obstacles which the vehicle should stop for
    wait_for_obstacle_id: List[str] = None

@dataclass
class TrafficLightStatus:
    """
    TrafficLightStatus class, oriented from protobuf message
    """

    # Overlap id of current traffic light
    current_traffic_light_overlap_id: List[str] = None
    # Overlap id of traffic light which is already done
    done_traffic_light_overlap_id: List[str] = None

@dataclass
class YieldSignStatus:
    """
    YieldSignStatus class, oriented from protobuf message
    """

    # Overlap id of current yield sign
    current_yield_sign_overlap_id: List[str] = None
    # Overlap id of current yield sign which is already passed
    done_yield_sign_overlap_id: List[str] = None
    # Obstacles which the vehicle should stop for
    wait_for_obstacle_id: List[str] = None

@dataclass
class LaneFollowStatus:
    """
    LaneFollowStatus class, oriented from protobuf message
    """

    # Is lane follow ok
    lane_follow_block: Optional[bool] = False
    # Blocking obstacle id in front of the vehicle
    block_obstacle_id: Optional[str] = ""
    # Last block time
    last_block_timestamp: Optional[float] = 0
    # Duration of block
    block_duration: Optional[float] = 0

@dataclass
class LaneBorrowStatus:
    """
    LaneBorrowStatus class, oriented from protobuf message
    """

    # Is lane borrow ok
    lane_borrow_block: Optional[bool] = False
    # Blocking obstacle id in front of the vehicle
    block_obstacle_id: Optional[str] = ""
    # Last block time
    last_block_timestamp: Optional[float] = 0
    # Duration of block
    block_duration: Optional[float] = 0

@dataclass
class PlanningStatus:
    """
    PlanningStatus class, oriented from protobuf message

    note: please keep this one as minimal as possible. do NOT pollute it.
    """

    change_lane: Optional[ChangeLaneStatus] = None
    creep_decider: Optional[CreepDeciderStatus] = None
    crosswalk: Optional[CrosswalkStatus] = None
    destination: Optional[DestinationStatus] = None
    emergency_stop: Optional[EmergencyStopStatus] = None
    open_space: Optional[OpenSpaceStatus] = None
    park_and_go: Optional[ParkAndGoStatus] = None
    path_decider: Optional[PathDeciderStatus] = None
    pull_over: Optional[PullOverStatus] = None
    rerouting: Optional[ReroutingStatus] = None
    scenario: Optional[ScenarioStatus] = None
    speed_decider: Optional[SpeedDeciderStatus] = None
    stop_sign: Optional[StopSignStatus] = None
    traffic_light: Optional[TrafficLightStatus] = None
    yield_sign: Optional[YieldSignStatus] = None
    lane_follow: Optional[LaneFollowStatus] = None
    lane_borrow: Optional[LaneBorrowStatus] = None
