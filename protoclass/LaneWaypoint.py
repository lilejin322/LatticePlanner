from dataclasses import dataclass
from typing import Optional
from protoclass.PointENU import PointENU

@dataclass
class LaneWaypoint:
    """
    LaneWaypoint class, oriented from protobuf message
    """

    id: Optional[str] = None
    s: Optional[float] = None
    pose: Optional[PointENU] = None
    # When the developer selects a point on the dreamview route editing
    # the direction can be specified by dragging the mouse
    # dreamview calculates the heading based on this to support construct lane way point with heading
    heading: Optional[float] = None
