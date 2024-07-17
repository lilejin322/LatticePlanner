from dataclasses import dataclass
from typing import Optional

@dataclass
class FrenetFramePoint:
    """
    FrenetFramePoint class, oriented from protobuf message
    """

    s: Optional[float] = None
    l: Optional[float] = None
    dl: Optional[float] = None
    ddl: Optional[float] = None
