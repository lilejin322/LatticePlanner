from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

@dataclass
class EngageAdvice:
    """
    EngageAdvice class, oriented from protobuf message

    This is the engage advice that published by critical runtime modules.
    """

    class Advice(Enum):
        UNKNOWN = 0
        DISALLOW_ENGAGE = 1
        READY_TO_ENGAGE = 2
        KEEP_ENGAGED = 3
        PREPARE_DISENGAGE = 4

    advice: Optional[Advice] = field(default=Advice.DISALLOW_ENGAGE)
    reason: Optional[str] = None
