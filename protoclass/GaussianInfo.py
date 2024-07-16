from dataclasses import dataclass
from typing import Optional

@dataclass
class GaussianInfo:
    """
    GaussianInfo class, oriented from protobuf message
    """
    # Information of gaussian distribution
    sigma_x: Optional[float] = None
    sigma_y: Optional[float] = None
    correlation: Optional[float] = None

    # Information of representative uncertainty area
    area_probability: Optional[float] = None
    ellipse_a: Optional[float] = None
    ellipse_b: Optional[float] = None
    theta_a: Optional[float] = None
