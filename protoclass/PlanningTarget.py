from dataclasses import dataclass, field
from typing import Optional
from enum import Enum

@dataclass
class StopPoint:
    """
    StopPoint class, oriented from protobuf message
    """

    class StopPointType(Enum):
        """
        StopPointType class, oriented from protobuf message
        """

        HARD = 0
        SOFT = 1

    s: Optional[float] = None
    type: Optional[StopPointType] = field(default=StopPointType.HARD)

@dataclass
class PlanningTarget:
    """
    PlanningTarget class, oriented from protobuf message
    """

    stop_point: Optional[StopPoint] = None
    cruise_speed: Optional[float] = None
