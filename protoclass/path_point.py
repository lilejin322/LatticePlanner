"""Path point on a reference line or a planned trajectory."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class PathPoint:
    """Cartesian sample the lattice sampler and combiner read."""

    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None
    kappa: Optional[float] = None
    s: Optional[float] = None
    dkappa: Optional[float] = None
