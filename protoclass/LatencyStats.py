from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class TaskStats:
    """
    TaskStats class, oriented from protobuf message
    """

    name: Optional[str] = None
    time_ms: Optional[float] = None

@dataclass
class LatencyStats:
    """
    LatencyStats class, oriented from protobuf message
    """

    total_time_ms: Optional[float] = None
    task_stats: List[TaskStats] = field(default_factory=list)
    init_frame_time_ms: Optional[float] = None
