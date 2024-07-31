from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

@dataclass
class StopPoint:
    """
    StopPoint class, oriented from protobuf message
    """

    class Type(Enum):

        HARD = 0
        SOFT = 1

    s: Optional[float] = None
    type: Optional[Type] = field(default=Type.HARD)

@dataclass
class PlanningTarget:
    """
    PlanningTarget class, oriented from protobuf message
    """

    stop_point: Optional[StopPoint] = None
    cruise_speed: Optional[float] = None
