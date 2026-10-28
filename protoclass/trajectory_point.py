"""One timed sample of a planned trajectory."""
from dataclasses import dataclass, field
from typing import Optional
from protoclass.path_point import PathPoint


@dataclass
class TrajectoryPoint:
    """State passed into one lattice cycle and written by the combiner."""

    path_point: Optional[PathPoint] = field(default_factory=PathPoint)
    v: Optional[float] = None
    a: Optional[float] = None
    relative_time: Optional[float] = None
