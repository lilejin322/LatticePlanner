"""Obstacle extent in Frenet coordinates."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class SLBoundary:
    """Longitudinal and lateral extent of one obstacle against the reference line.

    start_s <= end_s. Left of the reference line is positive.
    """

    start_s: Optional[float] = None
    end_s: Optional[float] = None
    start_l: Optional[float] = None
    end_l: Optional[float] = None
