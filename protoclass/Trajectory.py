from protoclass.TrajectoryPoint import TrajectoryPoint
from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class Trajectory:
    """
    Trajectory class, oriented from protobuf message
    """

    probability: Optional[float] = None
    trajectory_point: List[TrajectoryPoint] = field(default_factory=list)
