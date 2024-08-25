from dataclasses import dataclass
from typing import List, Optional
from protoclass.Header import Header
from protoclass.TrajectoryPoint import TrajectoryPoint
from protoclass.LocalizationEstimate import LocalizationEstimate

@dataclass
class PlanningData:
    """
    PlanningData class, oriented from protobuf message
    """

    adc_position: Optional[LocalizationEstimate] = None
    chassis: Optional[Chassis] = None
    routing: Optional[RoutingResponse] = None
    init_point: Optional[TrajectoryPoint] = None
    path: List[Path] = None
    speed_plan: List[SpeedPlan] = None
    st_graph: List[STGraphDebug] = None
    sl_frame: List[SLFrameDebug] = None
    prediction_header: Optional[Header] = None
    signal_light: Optional[SignalLightDebug] = None
    obstacle: List[ObstacleDebug] = None
    reference_line: List[ReferenceLineDebug] = None
    dp_poly_graph: Optional[DpPolyGraphDebug] = None
    lattice_st_image: Optional[LatticeStTraining] = None
    relative_map: Optional[MapMsg] = None
    auto_tuning_training_data: Optional[AutoTuningTrainingData] = None
    front_clear_distance: Optional[float] = None
    chart: List[Chart] = None
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
