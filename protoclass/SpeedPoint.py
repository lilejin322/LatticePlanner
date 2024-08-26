from dataclasses import dataclass, field
from typing import Optional, List

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

@dataclass
class SpeedPlan:
    """
    SpeedPlan class, oriented from protobuf message
    """

    name: Optional[str] = None
    speed_point: List[SpeedPoint] = field(default_factory=list)
