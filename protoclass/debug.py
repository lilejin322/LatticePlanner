"""
Debug submodule
"""
from dataclasses import dataclass, field
from typing import Any, List, Optional
from protoclass.header import Header
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.path_point import Path
from protoclass.speed_point import SpeedPlan


@dataclass
class PlanningData:
    """
    PlanningData class, oriented from protobuf message
    """

    adc_position: Optional[Any] = None
    chassis: Optional[Any] = None
    routing: Optional[Any] = None
    init_point: Optional[TrajectoryPoint] = None
    path: List[Path] = field(default_factory=list)
    speed_plan: List[SpeedPlan] = field(default_factory=list)
    st_graph: List[Any] = field(default_factory=list)
    sl_frame: List[Any] = field(default_factory=list)
    prediction_header: Optional[Header] = None
    signal_light: Optional[Any] = None
    obstacle: List[Any] = field(default_factory=list)
    reference_line: List[Any] = field(default_factory=list)
    dp_poly_graph: Optional[Any] = None
    lattice_st_image: Optional[Any] = None
    relative_map: Optional[Any] = None
    auto_tuning_training_data: Optional[Any] = None
    front_clear_distance: Optional[float] = None
    chart: List[Any] = field(default_factory=list)
    scenario: Optional[Any] = None
    open_space: Optional[Any] = None
    smoother: Optional[Any] = None
    pull_over: Optional[Any] = None
    hybrid_model: Optional[Any] = None

@dataclass
class Debug:
    """
    Debug class, oriented from protobuf message
    """

    planning_data: Optional[PlanningData] = None
