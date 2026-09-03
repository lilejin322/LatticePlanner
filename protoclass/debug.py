"""
Debug submodule
"""
from dataclasses import dataclass, field
from typing import List, Optional
from protoclass.header import Header
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.localization_estimate import LocalizationEstimate
from protoclass.chassis import Chassis
from protoclass.routing import RoutingResponse
from protoclass.path_point import Path
from protoclass.speed_point import SpeedPlan
from protoclass.planning_internal import STGraphDebug, SLFrameDebug, SignalLightDebug, ObstacleDebug, ReferenceLineDebug, DpPolyGraphDebug, \
                                         LatticeStTraining, MapMsg, AutoTuningTrainingData, Chart, ScenarioDebug, OpenSpaceDebug, SmootherDebug, \
                                         PullOverDebug, HybridModelDebug

@dataclass
class PlanningData:
    """
    PlanningData class, oriented from protobuf message
    """

    adc_position: Optional[LocalizationEstimate] = None
    chassis: Optional[Chassis] = None
    routing: Optional[RoutingResponse] = None
    init_point: Optional[TrajectoryPoint] = None
    path: List[Path] = field(default_factory=list)
    speed_plan: List[SpeedPlan] = field(default_factory=list)
    st_graph: List[STGraphDebug] = field(default_factory=list)
    sl_frame: List[SLFrameDebug] = field(default_factory=list)
    prediction_header: Optional[Header] = None
    signal_light: Optional[SignalLightDebug] = None
    obstacle: List[ObstacleDebug] = field(default_factory=list)
    reference_line: List[ReferenceLineDebug] = field(default_factory=list)
    dp_poly_graph: Optional[DpPolyGraphDebug] = None
    lattice_st_image: Optional[LatticeStTraining] = None
    relative_map: Optional[MapMsg] = None
    auto_tuning_training_data: Optional[AutoTuningTrainingData] = None
    front_clear_distance: Optional[float] = None
    chart: List[Chart] = field(default_factory=list)
    scenario: Optional[ScenarioDebug] = None
    open_space: Optional[OpenSpaceDebug] = None
    smoother: Optional[SmootherDebug] = None
    pull_over: Optional[PullOverDebug] = None
    hybrid_model: Optional[HybridModelDebug] = None

@dataclass
class Debug:
    """
    Debug class, oriented from protobuf message
    """

    planning_data: Optional[PlanningData] = None
