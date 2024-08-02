from dataclasses import dataclass, field
from typing import Optional, Union, List
from enum import Enum
from protoclass.PointENU import PointENU

@dataclass
class ObjectIgnore:
  """
  ObjectIgnore class, oriented from protobuf message
  """

  pass

class StopReasonCode(Enum):
  """
  StopReasonCode class, oriented from protobuf message
  """

  STOP_REASON_HEAD_VEHICLE = 1
  STOP_REASON_DESTINATION = 2
  STOP_REASON_PEDESTRIAN = 3
  STOP_REASON_OBSTACLE = 4
  STOP_REASON_PREPARKING = 5
  # only for red signal
  STOP_REASON_SIGNAL = 100
  STOP_REASON_STOP_SIGN = 101
  STOP_REASON_YIELD_SIGN = 102
  STOP_REASON_CLEAR_ZONE = 103
  STOP_REASON_CROSSWALK = 104
  STOP_REASON_CREEPER = 105
  # end of the reference_line
  STOP_REASON_REFERENCE_END = 106
  # yellow signal
  STOP_REASON_YELLOW_SIGNAL = 107
  # pull over
  STOP_REASON_PULL_OVER = 108
  STOP_REASON_SIDEPASS_SAFETY = 109
  STOP_REASON_PRE_OPEN_SPACE_STOP = 200
  STOP_REASON_LANE_CHANGE_URGENCY = 201
  STOP_REASON_EMERGENCY = 202

@dataclass
class ObjectStop:
  """
  ObjectStop class, oriented from protobuf message
  """

  reason_code: Optional[StopReasonCode] = None
  # in meters
  distance_s: Optional[float] = None
  # When stopped, the front center of vehicle should be at this point.
  stop_point: Optional[PointENU] = None
  # When stopped, the heading of the vehicle should be stop_heading.
  stop_heading: Optional[float] = None
  wait_for_obstacle: Optional[List[str]] = field(default_factory=list)

@dataclass
class ObjectNudge:
  """
  ObjectNudge class, oriented from protobuf message
  dodge the obstacle in lateral direction when driving
  """

  class Type(Enum):
    """
    Type class, oriented from protobuf message
    """

    # drive from the left side to nudge a static obstacle
    LEFT_NUDGE = 1
    # drive from the right side to nudge a static obstacle
    RIGHT_NUDGE = 2
    # drive from the left side to nudge a dynamic obstacle
    DYNAMIC_LEFT_NUDGE = 3
    # drive from the right side to nudge a dynamic obstacle
    DYNAMIC_RIGHT_NUDGE = 4

  type: Optional[Type] = None
  # minimum lateral distance in meters. positive if type = LEFT_NUDGE
  # negative if type = RIGHT_NUDGE
  distance_l: Optional[float] = None

@dataclass
class ObjectYield:
  """
  ObjectYield class, oriented from protobuf message
  """

  # minimum longitudinal distance in meters
  distance_s: Optional[float] = None
  fence_point: Optional[PointENU] = None
  fence_heading: Optional[float] = None
  # minimum time buffer required after the
  # obstacle reaches the intersect point.
  time_buffer: Optional[float] = None

@dataclass
class ObjectFollow:
  """
  ObjectFollow class, oriented from protobuf message
  """

  # minimum longitudinal distance in meters
  distance_s: Optional[float] = None
  fence_point: Optional[PointENU] = None
  fence_heading: Optional[float] = None

@dataclass
class ObjectOvertake:
  """
  ObjectOvertake class, oriented from protobuf message
  """

  # minimum longitudinal distance in meters
  distance_s: Optional[float] = None
  fence_point: Optional[PointENU] = None
  fence_heading: Optional[float] = None
  # minimum time buffer required before the
  # obstacle reaches the intersect point.
  time_buffer: Optional[float] = None

@dataclass
class ObjectSidePass:
  """
  ObjectSidePass class, oriented from protobuf message
  """

  class Type(Enum):
    """
    Type class, oriented from protobuf message
    """

    LEFT = 1
    RIGHT = 2

  type: Optional[Type] = None

@dataclass
class ObjectAvoid:
  """
  ObjectAvoid class, oriented from protobuf message
  unified object decision while estop
  """

  pass

@dataclass
class ObjectStatic:
  """
  ObjectStatic class, oriented from protobuf message
  """

  pass

@dataclass
class ObjectDynamic:
  """
  ObjectDynamic class, oriented from protobuf message
  """

  pass

@dataclass
class ObjectMotionType:
  """
  ObjectMotionType class, oriented from protobuf message
  """

  motion_tag: Optional[Union[ObjectStatic, ObjectDynamic]] = None

@dataclass
class ObjectDecisionType:
  """
  ObjectDecisionType class, oriented from protobuf message
  """

  object_tag: Optional[Union[ObjectIgnore, ObjectStop, ObjectFollow, ObjectYield, ObjectOvertake,
                             ObjectNudge, ObjectAvoid, ObjectSidePass]] = None

@dataclass
class ObjectStatus:
  """
  ObjectStatus class, oriented from protobuf message
  """

  motion_type: Optional[ObjectMotionType] = None
  decision_type: Optional[ObjectDecisionType] = None

@dataclass
class ObjectDecision:
  """
  ObjectDecision class, oriented from protobuf message
  """

  id: Optional[str] = None
  perception_id: Optional[int] = None
  object_decision: Optional[List[ObjectDecisionType]] = field(default_factory=list)

@dataclass
class ObjectDecisions:
  """
  ObjectDecisions class, oriented from protobuf message
  """

  decision: Optional[List[ObjectDecision]] = field(default_factory=list)

class ChangeLaneType(Enum):
  """
  ChangeLaneType class, oriented from protobuf message
  """

  FORWARD = 0
  LEFT = 1
  RIGHT = 2

@dataclass
class MainCruise:
  """
  MainCruise class, oriented from protobuf message
  cruise current lane
  """

  change_lane_type: Optional[ChangeLaneType] = None

@dataclass
class MainStop:
  """
  MainStop class, oriented from protobuf message
  """

  reason_code: Optional[StopReasonCode] = None
  reason: Optional[str] = None
  # When stopped, the front center of vehicle should be at this point.
  stop_point: Optional[PointENU] = None
  # When stopped, the heading of the vehicle should be stop_heading.
  stop_heading: Optional[float] = None
  change_lane_type: Optional[ChangeLaneType] = None

@dataclass
class EmergencyStopHardBrake:
  """
  EmergencyStopHardBrake class, oriented from protobuf message
  hard brake
  """

  pass

@dataclass
class EmergencyStopCruiseToStop:
  """
  EmergencyStopCruiseToStop class, oriented from protobuf message
  cruise to stop
  """

  pass

@dataclass
class MainEmergencyStop:
  """
  MainEmergencyStop class, oriented from protobuf message
  Unexpected event happened, human driver is required to take over
  """

  class ReasonCode(Enum):
    """
    ReasonCode class, oriented from protobuf message
    """

    ESTOP_REASON_INTERNAL_ERR = 1
    ESTOP_REASON_COLLISION = 2
    ESTOP_REASON_ST_FIND_PATH = 3
    ESTOP_REASON_ST_MAKE_DECISION = 4
    ESTOP_REASON_SENSOR_ERROR = 5

  reason_code: Optional[ReasonCode] = None
  reason: Optional[str] = None
  task: Optional[Union[EmergencyStopHardBrake, EmergencyStopCruiseToStop]] = None

@dataclass
class TargetLane:
  """
  TargetLane class, oriented from protobuf message
  """

  # lane id
  id: Optional[str] = None
  # in meters
  start_s: Optional[float] = None
  # in meters
  end_s: Optional[float] = None
  # in m/s
  speed_limit: Optional[float] = None

@dataclass
class MainChangeLane:
  """
  MainChangeLane class, oriented from protobuf message
  This message is deprecated
  """

  class Type(Enum):
    """
    Type class, oriented from protobuf message
    """

    LEFT = 1
    RIGHT = 2

  type: Optional[Type] = None
  default_lane: Optional[List[TargetLane]] = field(default_factory=list)
  default_lane_stop: Optional[MainStop] = None
  target_lane_stop: Optional[MainStop] = None

@dataclass
class MainMissionComplete:
  """
  MainMissionComplete class, oriented from protobuf message
  """

  # arrived at routing destination
  # When stopped, the front center of vehicle should be at this point.
  stop_point: Optional[PointENU] = None
  # When stopped, the heading of the vehicle should be stop_heading.
  stop_heading: Optional[float] = None

@dataclass
class MainNotReady:
  """
  MainNotReady class, oriented from protobuf message
  """

  # decision system is not ready.
  # e.g. wait for routing data.
  reason: Optional[str] = None

@dataclass
class MainParking:
  """
  MainParking class, oriented from protobuf message
  """

  class ParkingStatus(Enum):
    """
    ParkingStatus class, oriented from protobuf message
    """

    # TODO(QiL): implement and expand to more enums
    IN_PARKING = 1

  status: Optional[ParkingStatus] = None

@dataclass
class MainDecision:
  """
  MainDecision class, oriented from protobuf message
  """

  task: Optional[Union[MainCruise, MainStop, MainEmergencyStop, MainChangeLane, MainMissionComplete, MainNotReady, MainParking]] = None
  target_lane: Optional[List[TargetLane]] = None

@dataclass
class VehicleSignal:
  """
  VehicleSignal class, oriented from protobuf message
  """

  class TurnSignal(Enum):
    """
    TurnSignal class, oriented from protobuf message
    """

    TURN_NONE = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    TURN_HAZARD_WARNING = 3

  turn_signal: Optional[TurnSignal] = None
  high_beam: Optional[bool] = None
  low_beam: Optional[bool] = None
  horn: Optional[bool] = None
  emergency_light: Optional[bool] = None

@dataclass
class DecisionResult:
  """
  DecisionResult class, oriented from protobuf message
  """

  main_decision: Optional[MainDecision] = None
  object_decision: Optional[ObjectDecisions] = None
  vehicle_signal: Optional[VehicleSignal] = None
