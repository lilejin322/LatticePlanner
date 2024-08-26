from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import Enum
from protoclass.SpeedPoint import SpeedPoint
from protoclass.SLBoundary import SLPoint, SLBoundary
from protoclass.DecisionResult import ObjectDecisionType
from protoclass.Header import Header
from protoclass.LocalizationEstimate import LocalizationEstimate
from protoclass.PathPoint import Path
from protoclass.PerceptionObstacle import LaneMarkers
from protoclass.TrajectoryPoint import TrajectoryPoint
from protoclass.PointENU import PointENU
from protoclass.Trajectory import Trajectory
import math

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

@dataclass
class CostComponents:
    """
    CostComponents class, oriented from protobuf message
    """

    cost_component: List[float] = field(default_factory=list)

@dataclass
class AutoTuningTrainingData:
    """
    AutoTuningTrainingData class, oriented from protobuf message
    """

    teacher_component: Optional[CostComponents] = None
    student_component: Optional[CostComponents] = None

@dataclass
class Car:
    """
    Car class, oriented from protobuf message
    """

    label: Optional[str] = None
    hide_label_in_legend: Optional[bool] = False
    
    x: Optional[float] = None
    y: Optional[float] = None
    heading: Optional[float] = None
    color: Optional[str] = None

@dataclass
class Point2D:
    """
    A general 2D point. Its meaning and units depend on context, and must be
    explained in comments.
    """

    x: Optional[float] = math.nan
    y: Optional[float] = math.nan

@dataclass
class Polygon:
    """
    Polygon class, oriented from protobuf message
    """

    label: Optional[str] = None
    hide_label_in_legend: Optional[bool] = False
    point: List[Point2D] = field(default_factory=list)

    # If the 'color' property is undefined, a random one will be assigned.
    # See http://www.chartjs.org/docs/latest/charts/line.html
    # for all supported properties from chart.js
    properties: Dict[str, str] = field(default_factory=dict)

@dataclass
class Line:
    """
    Line class, oriented from protobuf message
    """

    label: Optional[str] = None
    hide_label_in_legend: Optional[bool] = False
    point: List[Point2D] = field(default_factory=list)

    """
    If the 'color' property is undefined, a random one will be assigned.
    See http://www.chartjs.org/docs/latest/charts/line.html
    for all supported properties from chart.js
    """
    properties: Dict[str, str] = field(default_factory=dict)

@dataclass
class Options:
    """
    Options class, oriented from protobuf message
    """

    @dataclass
    class Axis:

        min: Optional[float] = None
        max: Optional[float] = None
        label_string: Optional[str] = None

        window_size: Optional[float] = None
        """size of the axis of your graph which is then divided into measuring grades"""

        step_size: Optional[float] = None
        """size of the smaller measuring grades in the axis found between two larger measuring grades"""

        mid_value: Optional[float] = None
        """midpoint taken within the dataset. If it is not specified, we will calculate it for you."""

    legend_display: Optional[bool] = True
    x: Optional[Axis] = None
    y: Optional[Axis] = None

    aspect_ratio: Optional[float] = None
    """This is the aspect ratio (width/height) of the entire chart."""

    sync_xy_window_size: Optional[bool] = False
    """
    Same window size for x-Axis and y-Axis. It is
    effective only if x/y window_size is NOT set.
    """

@dataclass
class Chart:
    """
    Chart class, oriented from protobuf message
    """

    title: Optional[str] = None
    options: Optional[Options] = None

    # Data sets
    line: List[Line] = field(default_factory=list)
    polygon: List[Polygon] = field(default_factory=list)
    car: List[Car] = field(default_factory=list)

@dataclass
class ScenarioDebug:
    """
    ScenarioDebug class, oriented from protobuf message
    """

    msg: Optional[str] = None
    scenario_plugin_type: Optional[str] = None
    stage_plugin_type: Optional[str] = None

@dataclass
class Trajectories:
    """
    Trajectories class, oriented from protobuf message
    """

    trajectory: List[Trajectory] = field(default_factory=list)

@dataclass
class VehicleMotionPoint:
    """
    VehicleMotionPoint class, oriented from protobuf message
    """

    trajectory_point: Optional[TrajectoryPoint] = None
    """trajectory point"""
    steer: Optional[float] = None
    """The angle between vehicle front wheel and vehicle longitudinal axis"""

@dataclass
class VehicleMotion:
    """
    VehicleMotion class, oriented from protobuf message
    """

    name: Optional[str] = None
    vehicle_motion_point: List[VehicleMotionPoint] = field(default_factory=list)

@dataclass
class OpenSpaceDebug:
    """
    OpenSpaceDebug class, oriented from protobuf message
    """

    trajectories: Optional[Trajectories] = None
    warm_start_trajectory: Optional[VehicleMotion] = None
    smoothed_trajectory: Optional[VehicleMotion] = None
    warm_start_dual_lambda: List[float] = field(default_factory=list)
    warm_start_dual_miu: List[float] = field(default_factory=list)
    optimized_dual_lambda: List[float] = field(default_factory=list)
    optimized_dual_miu: List[float] = field(default_factory=list)
    xy_boundary: List[float] = field(default_factory=list)
    obstacles: List[ObstacleDebug] = field(default_factory=list)
    roi_shift_point: Optional[TrajectoryPoint] = None
    end_point: Optional[TrajectoryPoint] = None
    partitioned_trajectories: Optional[Trajectories] = None
    chosen_trajectory: Optional[Trajectories] = None
    is_fallback_trajectory: Optional[bool] = None
    fallback_trajectory: Optional[Trajectories] = None
    trajectory_stitching_point: Optional[TrajectoryPoint] = None
    future_collision_point: Optional[TrajectoryPoint] = None
    time_latency: Optional[float] = 0.0
    origin_point: Optional[PointENU] = None
    origin_heading_rad: Optional[float] = None

@dataclass
class SmootherDebug:
    """
    SmootherDebug class, oriented from protobuf message
    """

    class SmootherType(Enum):

        SMOOTHER_NONE = 1
        SMOOTHER_CLOSE_STOP = 2

    is_smoothed: Optional[bool] = None

    type: SmootherType = SmootherType.SMOOTHER_NONE
    reason: Optional[str] = None

class PullOverDebug:
    """
    PullOverDebug class, oriented from protobuf message
    """

    position: Optional[PointENU] = None
    theta: Optional[float] = None
    length_front: Optional[float] = None
    length_back: Optional[float] = None
    width_left: Optional[float] = None
    width_right: Optional[float] = None

@dataclass
class HybridModelDebug:
    """
    HybridModelDebug class, oriented from protobuf message
    """

    using_learning_model_output: Optional[bool] = False
    learning_model_output_usage_ratio: Optional[float] = None
    learning_model_output_fail_reason: Optional[str] = None
    evaluated_path_reference: Optional[Path] = None
