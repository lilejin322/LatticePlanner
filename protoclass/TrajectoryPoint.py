from dataclasses import dataclass
from typing import Optional
from common.PathPoint import PathPoint
from protoclass.GaussianInfo import GaussianInfo

@dataclass
class TrajectoryPoint:
    # path point
    path_point: Optional[PathPoint] = None
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
    # Gaussian probability information
    gaussian_info: Optional[GaussianInfo] = None
