from dataclasses import dataclass, field
from typing import Optional, List
from enum import Enum
from protoclass.ADCTrajectory import Point3D
from protoclass.Trajectory import Trajectory
import math

@dataclass
class BBox2D:
    """
    BBox2D class, oriented from protobuf message
    """

    xmin: Optional[float] = None             # in pixels
    ymin: Optional[float] = None             # in pixels
    xmax: Optional[float] = None             # in pixels
    ymax: Optional[float] = None             # in pixels

@dataclass
class SensorMeasurement:
    """
    SensorMeasurement class, oriented from protobuf message
    """

    sensor_id: Optional[str] = None
    id: Optional[int] = None
    position: Optional[Point3D] = None
    theta: Optional[float] = None
    length: Optional[float] = None
    width: Optional[float] = None
    height: Optional[float] = None
    velocity: Optional[Point3D] = None
    type: Optional['PerceptionObstacle'.Type] = None
    sub_type: Optional['PerceptionObstacle'.SubType] = None
    timestamp: Optional[float] = None
    box: Optional[BBox2D] = None             # only for camera measurements

@dataclass
class LightStatus:
    """
    LightStatus class, oriented from protobuf message
    """

    brake_visible: Optional[float] = None
    brake_switch_on: Optional[float] = None
    left_turn_visible: Optional[float] = None
    left_turn_switch_on: Optional[float] = None
    right_turn_visible: Optional[float] = None
    right_turn_switch_on: Optional[float] = None

@dataclass
class DebugMessage:
    """
    DebugMessage class, oriented from protobuf message
    """

    trajectory: List[Trajectory] = field(default_factory=list)

@dataclass
class V2XInformation:
    """
    V2XInformation class, oriented from protobuf message
    """
    class V2XType(Enum):

        NONE = 0
        ZOMBIES_CAR = 1
        BLIND_ZONE = 2

    v2x_type: List[V2XType] = field(default_factory=list)

@dataclass
class PerceptionObstacle:
    """
    perceptionObsacle class, oriented from protobuf message
    """

    id: Optional[int] = None                 # obstacle ID.
    # obstacle position in the world coordinate system.
    position: Optional[Point3D] = None
    theta: Optional[float] = None            # heading in the world coordinate system.
    velocity: Optional[Point3D] = None       # obstacle velocity.
    # Size of obstacle bounding box.
    length: Optional[float] = None           # obstacle length.
    width: Optional[float] = None            # obstacle width.
    height: Optional[float] = None           # obstacle height.
    polygon_point: List[Point3D] = field(default_factory=list)     # obstacle corner points.
    # duration of an obstacle since detection in s.
    tracking_time: Optional[float] = None

    class Type(Enum):

        UNKNOWN = 0
        UNKNOWN_MOVABLE = 1
        UNKNOWN_UNMOVABLE = 2
        PEDESTRIAN = 3                       # Pedestrian, usually determined by moving behavior.
        BICYCLE = 4                          # bike, motor bike
        VEHICLE = 5                          # Passenger car or truck.
    
    type: Optional[Type] = None              # obstacle type
    timestamp: Optional[float] = None        # GPS time in seconds.
    
    # Just for offline debugging, will not fill this field on board.
    # Format: [x0, y0, z0, x1, y1, z1...]
    point_cloud: List[float] = field(default_factory=list)
    
    confidence: Optional[float] = None
    
    class ConfidenceType(Enum):

        CONFIDENCE_UNKNOWN = 0
        CONFIDENCE_CNN = 1
        CONFIDENCE_RADAR = 2
    
    confidence_type: Optional[ConfidenceType] = None
    # trajectory of object.
    drops: List[Point3D] = field(default_factory=list)
    
    # The following fields are new added in Apollo 4.0
    acceleration: Optional[Point3D] = None   # obstacle acceleration

    # a stable obstacle point in the world coordinate system
    # position defined above is the obstacle bounding box ground center
    anchor_point: Optional[Point3D] = None
    bbox2d: Optional[BBox2D] = None
    
    class SubType(Enum):

        ST_UNKNOWN = 0
        ST_UNKNOWN_MOVABLE = 1
        ST_UNKNOWN_UNMOVABLE = 2
        ST_CAR = 3
        ST_VAN = 4
        ST_TRUCK = 5
        ST_BUS = 6
        ST_CYCLIST = 7
        ST_MOTORCYCLIST = 8
        ST_TRICYCLIST = 9
        ST_PEDESTRIAN = 10
        ST_TRAFFICCONE = 11
        ST_SMALLMOT = 12
        ST_BIGMOT = 13
        ST_NONMOT = 14
    
    sub_type: Optional[SubType] = None       # obstacle sub_type
    measurements: List[SensorMeasurement] = field(default_factory=list)     # sensor measurements

    # orthogonal distance between obstacle lowest point and ground plane
    height_above_ground: Optional[float] = field(default_factory=lambda: math.nan)

    # position covariance which is a row-majored 3x3 matrix
    position_covariance: List[float] = field(default_factory=list)
    # velocity covariance which is a row-majored 3x3 matrix
    velocity_covariance: List[float] = field(default_factory=list)
    # acceleration covariance which is a row-majored 3x3 matrix
    acceleration_covariance: List[float] = field(default_factory=list)

    # lights of vehicles
    light_status: Optional[LightStatus] = None

    # Debug Message
    msg: Optional[DebugMessage] = None
    
    class Source(Enum):

        HOST_VEHICLE = 0
        V2X = 1
    
    source: Optional[Source] = field(default_factory=lambda: PerceptionObstacle.Source.HOST_VEHICLE)
    v2x_info: Optional[V2XInformation] = None
    
    class SemanticType(Enum):

        SM_UNKNOWN = 0
        SM_IGNORE = 1
        SM_GROUND = 2
        SM_OBJECT = 3
        SM_CURB = 4
        SM_VEGETATION = 5
        SM_FENCE = 6
        SM_NOISE = 7
        SM_WALL = 8
        SM_MAX_OBJECT_SEMANTIC_LABEL = 9
    
    semantic_type: Optional[SemanticType] = None
