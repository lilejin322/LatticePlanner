from dataclasses import dataclass
from typing import Optional

@dataclass
class PathPoint:
    """
    PathPoint class, oriented from protobuf message
    """

    # coordinates
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None

    # direction on the x-y plane
    theta: Optional[float] = None
    # curvature on the x-y planning
    kappa: Optional[float] = None
    # accumulated distance from beginning of the path
    s: Optional[float] = None
    # derivative of kappa w.r.t s.
    dkappa: Optional[float] = None
    # derivative of derivative of kappa w.r.t s.
    ddkappa: Optional[float] = None
    # The lane ID where the path point is on
    lane_id: Optional[str] = None

    # derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
    x_derivative: Optional[float] = None
    y_derivative: Optional[float] = None
