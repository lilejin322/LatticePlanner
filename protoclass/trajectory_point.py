"""
Trajectory Point submodule
"""
from dataclasses import dataclass, field
from typing import Optional
from protoclass.path_point import PathPoint

@dataclass
class TrajectoryPoint:
    """
    TrajectoryPoint class, oriented from protobuf message
    """

    # path point
    path_point: Optional[PathPoint] = field(default_factory=PathPoint)
    # linear velocity
    v: Optional[float] = None  # in [m/s]
    # linear acceleration
    a: Optional[float] = None
    # relative time from beginning of the trajectory
    relative_time: Optional[float] = None
    # longitudinal jerk
    da: Optional[float] = None
    # The angle between vehicle front wheel and vehicle longitudinal axis
    steer: Optional[float] = None
