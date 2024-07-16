from dataclasses import dataclass, field
from typing import Optional, List
from protoclass.PathPoint import PathPoint
from protoclass.TrajectoryPoint import TrajectoryPoint
from protoclass.Header import Header
from protoclass.Debug import Debug
from protoclass.DecisionResult import DecisionResult
import math
from enum import Enum

class GearPosition(Enum):
    """
    GearPosition class, oriented from protobuf message
    """
    
    GEAR_NEUTRAL = 0
    GEAR_DRIVE = 1
    GEAR_REVERSE = 2
    GEAR_PARKING = 3
    GEAR_LOW = 4
    GEAR_INVALID = 5
    GEAR_NONE = 6

@dataclass
class EStop:
    """
    EStop class, oriented from protobuf message
    """

    is_estop: Optional[bool] = None
    reason: Optional[str] = None

@dataclass
class TaskStats:
    """
    TaskStats class, oriented from protobuf message
    """

    name: Optional[str] = None
    time_ms: Optional[float] = None

@dataclass
class LatencyStats:
    """
    LatencyStats class, oriented from protobuf message
    """

    total_time_ms: Optional[float] = None
    task_stats: List[TaskStats] = field(default_factory=list)
    init_frame_time_ms: Optional[float] = None

@dataclass
class EngageAdvice:
    """
    EngageAdvice class, oriented from protobuf message
    This is the engage advice that published by critical runtime modules.
    """

    class Advice(Enum):
        """
        Advice class, oriented from protobuf message
        """
        
        UNKNOWN = 0
        DISALLOW_ENGAGE = 1
        READY_TO_ENGAGE = 2
        KEEP_ENGAGED = 3
        PREPARE_DISENGAGE = 4

    advice: Optional[Advice] = field(default=Advice.DISALLOW_ENGAGE)
    reason: Optional[str] = None

@dataclass
class Id:
    """
    Id class, oriented from protobuf message
    Global unique ids for all objects (include lanes, junctions, overlaps, etc).
    """

    id: Optional[str] = None

@dataclass
class Point:
    """
    Point class, oriented from protobuf message
    """

    x: Optional[float] = None
    y: Optional[float] = None

@dataclass
class LocationPose:
    """
    LocationPose class, oriented from protobuf message
    """

    vehice_location: Optional[Point] = None
    left_lane_boundary_point: Optional[Point] = None
    right_lane_boundary_point: Optional[Point] = None

@dataclass
class RSSInfo:
    """
    RSSInfo class, oriented from protobuf message
    """

    is_rss_safe: Optional[bool] = None
    cur_dist_lon: Optional[float] = None
    rss_safe_dist_lon: Optional[float] = None
    acc_lon_range_minimum: Optional[float] = None
    acc_lon_range_maximum: Optional[float] = None
    acc_lat_left_range_minimum: Optional[float] = None
    acc_lat_left_range_maximum: Optional[float] = None
    acc_lat_right_range_minimum: Optional[float] = None
    acc_lat_right_range_maximum: Optional[float] = None

@dataclass
class Point3D:
    """
    Point3D class, oriented from protobuf message
    A general 3D point. Its meaning and units depend on context, and must be
    explained in comments.
    """

    x: Optional[float] = field(default=math.nan)
    y: Optional[float] = field(default=math.nan)
    z: Optional[float] = field(default=math.nan)

@dataclass
class Polygon:
    """
    Polygon class, oriented from protobuf message
    A general polygon, points are counter clockwise
    """

    point: List[Point3D] = field(default_factory=list)

@dataclass
class CriticalRegion:
    """
    CriticalRegion class, oriented from protobuf message
    the region where planning cares most
    """

    region: List[Polygon] = field(default_factory=list)

@dataclass
class ADCTrajectory:
    """
    ADCTrajectory class, oriented from protobuf message
    """

    header: Optional[Header] = None
    # in meters
    total_path_length: Optional[float] = None
    # in seconds
    total_path_time: Optional[float] = None
    estop: Optional[EStop] = None
    debug: Optional[Debug] = None
    # is_replan == true mean replan triggered
    is_replan: Optional[bool] = field(default=False)
    # Specify trajectory gear
    gear: Optional[GearPosition] = None
    # path data + speed data
    trajectory_point: List[TrajectoryPoint] = field(default_factory=list)
    # path point without speed info
    path_point: List[PathPoint] = field(default_factory=list)
    decision: Optional[DecisionResult] = None
    latency_stats: Optional[LatencyStats] = None
    # the routing used for current planning result
    routing_header: Optional[Header] = None
    
    class RightOfWayStatus(Enum):
        """
        RightOfWayStatus class, oriented from protobuf message
        """

        UNPROTECTED = 0
        PROTECTED = 1
    
    right_of_way_status: Optional[RightOfWayStatus] = None
    # lane id along current reference line
    lane_id: List[Id] = field(default_factory=list)
    # set the engage advice for based on current planning result.
    engage_advice: Optional[EngageAdvice] = None
    # critical region will be empty when planning is NOT sure which region is critical
    # critical regions may or may not overlap
    critical_region: Optional[CriticalRegion] = None
    
    class TrajectoryType(Enum):
        """
        TrajectoryType class, oriented from protobuf message
        """

        UNKNOWN = 0
        NORMAL = 1
        PATH_FALLBACK = 2
        SPEED_FALLBACK = 3
        PATH_REUSED = 4
        OPEN_SPACE = 5
    
    trajectory_type: Optional[TrajectoryType] = TrajectoryType.UNKNOWN
    replan_reason: Optional[str] = None
    # lane id along target reference line
    target_lane_id: List[Id] = field(default_factory=list)
    # complete dead end flag
    car_in_dead_end: Optional[bool] = None
    # vehicle location pose
    location_pose: Optional[LocationPose] = None
    # output related to RSS
    rss_info: Optional[RSSInfo] = None
