from dataclasses import dataclass
from typing import Optional

@dataclass
class SpeedPoint:
    """
    SpeedPoint class, oriented from protobuf message
    """

    s: Optional[float] = None
    t: Optional[float] = None
    v: Optional[float] = None     # speed (m/s)
    a: Optional[float] = None     # acceleration (m/s^2)
    da: Optional[float] = None    # jerk (m/s^3)
