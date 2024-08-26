from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import Enum
from protoclass.SpeedPoint import SpeedPoint
from protoclass.SLBoundary import SLPoint, SLBoundary
from protoclass.DecisionResult import ObjectDecisionType
from protoclass.Header import Header
from protoclass.LocalizationEstimate import LocalizationEstimate
from protoclass.PathPoint import Path

@dataclass
class StGraphBoundaryDebug:
    """
    StGraphBoundaryDebug class, oriented from protobuf message
    """

    class StBoundaryType(Enum):

        ST_BOUNDARY_TYPE_UNKNOWN = 1
        ST_BOUNDARY_TYPE_STOP = 2
        ST_BOUNDARY_TYPE_FOLLOW = 3
        ST_BOUNDARY_TYPE_YIELD = 4
        ST_BOUNDARY_TYPE_OVERTAKE = 5
        ST_BOUNDARY_TYPE_KEEP_CLEAR = 6
        ST_BOUNDARY_TYPE_DRIVABLE_REGION = 7

    name: Optional[str] = None
    point: List[SpeedPoint] = field(default_factory=list)
    type: Optional[StBoundaryType] = None

@dataclass
class STGraphDebug:
    """
    STGraphDebug class, oriented from protobuf message
    """

    @dataclass
    class STGraphSpeedConstraint:
        """
        STGraphSpeedConstraint class, oriented from protobuf message
        """

        t: List[float] = field(default_factory=list)
        lower_bound: List[float] = field(default_factory=list)
        upper_bound: List[float] = field(default_factory=list)

    @dataclass
    class STGraphKernelCuiseRef:
        """
        STGraphKernelCuiseRef class, oriented from protobuf
        """

        t: List[float] = field(default_factory=list)
        cruise_line_s: List[float] = field(default_factory=list)

    @dataclass
    class STGraphKernelFollowRef:
        """
        STGraphKernelFollowRef class, oriented from protobuf
        """

        t: List[float] = field(default_factory=list)
        follow_line_s: List[float] = field(default_factory=list)

    name: Optional[str] = None
    boundary: List[StGraphBoundaryDebug] = field(default_factory=list)
    speed_limit: List[SpeedPoint] = field(default_factory=list)
    speed_profile: List[SpeedPoint] = field(default_factory=list)
    speed_constraint: Optional[STGraphSpeedConstraint] = None
    kernel_cruise_ref: Optional[STGraphKernelCuiseRef] = None
    kernel_follow_ref: Optional[STGraphKernelFollowRef] = None

@dataclass
class SLFrameDebug:
    """
    SLFrameDebug class, oriented from protobuf message
    """

    name: Optional[str] = None
    sampled_s: List[float] = field(default_factory=list)
    static_obstacle_lower_bound: List[float] = field(default_factory=list)
    dynamic_obstacle_lower_bound: List[float] = field(default_factory=list)
    static_obstacle_upper_bound: List[float] = field(default_factory=list)
    dynamic_obstacle_upper_bound: List[float] = field(default_factory=list)
    map_lower_bound: List[float] = field(default_factory=list)
    map_upper_bound: List[float] = field(default_factory=list)
    sl_path: List[SLPoint] = field(default_factory=list)
    aggregated_boundary_s: List[float] = field(default_factory=list)
    aggregated_boundary_low: List[float] = field(default_factory=list)
    aggregated_boundary_high: List[float] = field(default_factory=list)

@dataclass
class TrafficLight:
    """
    TrafficLight class, oriented from protobuf message
    """

    class Color(Enum):

        UNKNOWN = 0
        RED = 1
        YELLOW = 2
        GREEN = 3
        BLACK = 4

    color: Optional[Color] = None
    id: Optional[str] = None
    """Traffic light string-ID in the map data"""
    confidence: Optional[float] = 1.0
    """Confidence about the detected results, between 0 and 1"""
    tracking_time: Optional[float] = None
    """Duration of the traffic light since detected"""
    blink: Optional[bool] = None
    """Is the traffic light blinking"""
    remaining_time: Optional[float] = None
    """v2x traffic light remaining time"""

@dataclass
class SignalLightDebug:
    """
    SignalLightDebug class, oriented from protobuf message
    """

    @dataclass
    class SignalDebug:
        """
        SignalDebug class, oriented from protobuf message
        """

        light_id: Optional[str] = None
        color: Optional[TrafficLight.Color] = None
        light_stop_s: Optional[float] = None
        adc_stop_deceleration: Optional[float] = None
        is_stop_wall_created: Optional[bool] = None

    adc_speed: Optional[float] = None
    adc_front_s: Optional[float] = None
    signal: List[SignalDebug] = field(default_factory=list)

@dataclass
class DecisionTag:
    """
    DecisionTag class, oriented from protobuf message
    """

    decider_tag: Optional[str] = None
    decision: Optional[ObjectDecisionType] = None

@dataclass
class ObstacleDebug:
    """
    ObstacleDebug class, oriented from protobuf message
    """

    id: Optional[str] = None
    sl_boundary: Optional[SLBoundary] = None
    decision_tag: List[DecisionTag] = field(default_factory=list)
    vertices_x_coords: List[float] = field(default_factory=list)
    vertices_y_coords: List[float] = field(default_factory=list)

@dataclass
class ReferenceLineDebug:
    """
    ReferenceLineDebug class, oriented from protobuf message
    """

    id: Optional[str] = None
    length: Optional[float] = None
    cost: Optional[float] = None
    is_change_lane_path: Optional[bool] = None
    is_drivable: Optional[bool] = None
    is_protected: Optional[bool] = None
    is_offroad: Optional[bool] = None
    minimum_boundary: Optional[float] = None
    average_kappa: Optional[float] = None
    average_dkappa: Optional[float] = None
    kappa_rms: Optional[float] = None
    dkappa_rms: Optional[float] = None
    kappa_max_abs: Optional[float] = None
    dkappa_max_abs: Optional[float] = None
    average_offset: Optional[float] = None

@dataclass
class SampleLayerDebug:
    """
    SampleLayerDebug class, oriented from protobuf message
    """

    sl_point: List[SLPoint] = field(default_factory=list)

@dataclass
class DpPolyGraphDebug:
    """
    DpPolyGraphDebug class, oriented from protobuf message
    """

    sample_layer: List[SampleLayerDebug] = field(default_factory=list)
    min_cost_point: List[SLPoint] = field(default_factory=list)

@dataclass
class LatticeStPixel:
    """
    LatticeStPixel class, oriented from protobuf message
    """

    s: Optional[int] = None
    t: Optional[int] = None
    r: Optional[int] = None
    g: Optional[int] = None
    b: Optional[int] = None

@dataclass
class LatticeStTraining:
    """
    LatticeStTraining class, oriented from protobuf message
    """

    pixel: List[LatticeStPixel] = field(default_factory=list)
    timestamp: Optional[float] = None
    annotation: Optional[str] = None
    num_s_grids: Optional[int] = None
    num_t_grids: Optional[int] = None
    s_resolution: Optional[float] = None
    t_resolution: Optional[float] = None

@dataclass
class NavigationPath:
    """
    NavigationPath class, oriented from protobuf message
    """

    path: Optional[Path] = None
    path_priority: Optional[int] = None
    """highest = 0 which can directly reach destination; change lane indicator"""

@dataclass
class MapMsg:
    """
    The map message in transmission format.
    """

    header: Optional[Header] = None
    hdmap: Optional[Map] = None 
    """
    Coordination: FLU
    x: Forward
    y: Left
    z: Up
    """
    navigation_path: Dict[str, NavigationPath] = field(default_factory=dict)
    """
    key: type string; the lane id in Map
    value: Navigation path; the reference line of the lane
    """
    lane_marker: Optional[LaneMarkers] = None
    """Lane marker information from perception"""
    localization: Optional[LocalizationEstimate] = None
    """Localization"""
