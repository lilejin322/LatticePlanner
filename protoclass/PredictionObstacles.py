from dataclasses import dataclass, field
from typing import List, Optional
from protoclass.Header import Header, ErrorCode
from enum import Enum
from protoclass.ADCTrajectory import Point3D
from protoclass.PerceptionObstacle import PerceptionObstacle
from protoclass.TrajectoryPoint import TrajectoryPoint
from protoclass.Lane import Lane

@dataclass
class JunctionExit:
    """
    JunctionExit class, oriented from protobuf message
    """

    exit_lane_id: Optional[str] = None
    exit_position: Optional[Point3D] = None
    exit_heading: Optional[float] = None
    exit_width: Optional[float] = None

@dataclass
class LaneFeature:
    """
    LaneFeature class, oriented from protobuf message
    """

    lane_id: Optional[str] = None
    lane_turn_type: Optional[int] = None
    lane_s: Optional[float] = None
    lane_l: Optional[float] = None
    angle_diff: Optional[float] = None
    dist_to_left_boundary: Optional[float] = None
    dist_to_right_boundary: Optional[float] = None
    lane_heading: Optional[float] = None
    lane_type: Optional[Lane.LaneType] = None

@dataclass
class JunctionFeature:
    """
    JunctionFeature class, oriented from protobuf message
    """

    junction_id: Optional[str] = None
    junction_range: Optional[float] = None
    enter_lane: Optional[LaneFeature] = None
    junction_exit: List[JunctionExit] = field(default_factory=list)
    junction_mlp_feature: List[float] = field(default_factory=list)
    junction_mlp_label: List[int] = field(default_factory=list)  # dim is number of masks, i.e. 12
    junction_mlp_probability: List[float] = field(default_factory=list)
    start_lane_id: List[str] = field(default_factory=list)

@dataclass
class PredictionPathPoint:
    """
    PredictionPathPoint class, oriented from protobuf message
    """

    x: float
    y: float
    velocity_heading: Optional[float] = None

@dataclass
class PredictionTrajectoryPoint:
    """
    PredictionTrajectoryPoint class, oriented from protobuf message
    """

    path_point: PredictionPathPoint
    timestamp: float

@dataclass
class ObstacleInteractiveTag:
    """
    Obstacle interactive tag class, oriented from protobuf message
    """

    class InteractiveTag(Enum):

        INTERACTION = 1
        NONINTERACTION = 2

    interactive_tag: Optional[InteractiveTag] = InteractiveTag.NONINTERACTION

@dataclass
class ObstaclePriority:
    """
    Obstacle priority class, oriented from protobuf message
    """

    class Priority(Enum):

        CAUTION = 1
        NORMAL = 2
        IGNORE = 3

    priority: Optional[Priority] = Priority.NORMAL

@dataclass
class Trajectory:
    """
    Trajectory class, oriented from protobuf message
    """

    probability: Optional[float] = None               # probability of this trajectory
    trajectory_point: List[TrajectoryPoint] = field(default_factory=list)

@dataclass
class Feature:
    """
    Feature class, oriented from protobuf message
    """

    # Obstacle ID
    id: Optional[int] = None

    # Obstacle features
    polygon_point: List[Point3D] = field(default_factory=list)
    position: Optional[Point3D] = None
    front_position: Optional[Point3D] = None
    velocity: Optional[Point3D] = None
    raw_velocity: Optional[Point3D] = None
    acceleration: Optional[Point3D] = None
    velocity_heading: Optional[float] = None
    speed: Optional[float] = None
    acc: Optional[float] = None
    theta: Optional[float] = None
    length: Optional[float] = None
    width: Optional[float] = None
    height: Optional[float] = None
    tracking_time: Optional[float] = None
    timestamp: Optional[float] = None

    # obstacle type-specific features
    lane: Optional[Lane] = None
    junction_feature: Optional[JunctionFeature] = None

    # Obstacle tracked features
    t_position: Optional[Point3D] = None
    t_velocity: Optional[Point3D] = None
    t_velocity_heading: Optional[float] = None
    t_speed: Optional[float] = None
    t_acceleration: Optional[Point3D] = None
    t_acc: Optional[float] = None

    is_still: Optional[bool] = field(default_factory=lambda: False)
    type: Optional[int] = None  # Use PerceptionObstacle.Type values
    label_update_time_delta: Optional[float] = None

    priority: Optional[ObstaclePriority] = None
    interactive_tag: Optional[ObstacleInteractiveTag] = None

    is_near_junction: Optional[bool] = field(default_factory=lambda: False)

    # Obstacle ground-truth labels:
    future_trajectory_points: List[PredictionTrajectoryPoint] = field(default_factory=list)

    # Obstacle short-term predicted trajectory points
    short_term_predicted_trajectory_points: List[TrajectoryPoint] = field(default_factory=list)

    # Obstacle predicted trajectories
    predicted_trajectory: List[Trajectory] = field(default_factory=list)

    # ADC trajectory at the same frame, and ADC trajectory timestamp
    adc_trajectory_point: List[TrajectoryPoint] = field(default_factory=list)
    adc_timestamp: Optional[float] = None
    adc_localization: Optional[PerceptionObstacle] = None
    
    # Surrounding lanes
    surrounding_lane_id: List[str] = field(default_factory=list)
    within_lane_id: List[str] = field(default_factory=list)

@dataclass
class Scenario:
    """
    Scenario class, oriented from protobuf message
    """

    class Type(Enum):

        UNKNOWN = 0
        CRUISE = 1000
        CRUISE_URBAN = 1001
        CRUISE_HIGHWAY = 1002
        JUNCTION = 2000
        JUNCTION_TRAFFIC_LIGHT = 2001
        JUNCTION_STOP_SIGN = 2002

    type: Optional[Type] = Type.UNKNOWN
    junction_id: Optional[str] = None

@dataclass
class Intent:
    """
    self driving car intent, oriented from protobuf message
    """

    class Type(Enum):

        UNKNOWN = 0
        STOP = 1
        CRUISE = 2
        CHANGE_LANE = 3

    type: Optional[Type] = Type.UNKNOWN

@dataclass
class ObstacleIntent:
    """
    estimated obstacle intent, oriented from protobuf message
    """

    class ObstacleIntentType(Enum):

        UNKNOWN = 0
        STOP = 1
        STATIONARY = 2
        MOVING = 3
        CHANGE_LANE = 4
        LOW_ACCELERATION = 5
        HIGH_ACCELERATION = 6
        LOW_DECELERATION = 7
        HIGH_DECELERATION = 8

    type: Optional[ObstacleIntentType] = ObstacleIntentType.UNKNOWN

@dataclass
class PredictionObstacle:
    """
    PredictionObstacle class, oriented from protobuf message
    """

    perception_obstacle: Optional[PerceptionObstacle] = None
    timestamp: Optional[float] = None                           # GPS time in seconds
    # the length of the time for this prediction (e.g. 10s)
    predicted_period: Optional[float] = None
    # can have multiple trajectories per obstacle
    trajectory: List[Trajectory] = field(default_factory=list)

    # estimated obstacle intent
    intent: Optional[ObstacleIntent] = None

    priority: Optional[ObstaclePriority] = None

    interactive_tag: Optional[ObstacleInteractiveTag] = None

    is_static: Optional[bool] = field(default_factory=lambda: False)
    
    # Feature history latest -> earliest sequence
    feature: List[Feature] = field(default_factory=list)

@dataclass
class PredictionObstacles:
    """
    PredictionObstacles class, oriented from protobuf message
    """

    # timestamp is included in header
    header: Optional[Header] = None

    # make prediction for multiple obstacles
    prediction_obstacle: List[PredictionObstacle] = field(default_factory=list)

    # perception error code
    perception_error_code: Optional[ErrorCode] = None

    # start timestamp
    start_timestamp: Optional[float] = None

    # end timestamp
    end_timestamp: Optional[float] = None

    # self driving car intent
    intent: Optional[Intent] = None

    # Scenario
    scenario: Optional[Scenario] = None
