"""Cruise or stop target for one lattice cycle."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class StopPoint:
    """Station where longitudinal sampling should stop."""

    s: Optional[float] = None


@dataclass
class PlanningTarget:
    """Stop station, if any, and the cruise speed the evaluator tracks."""

    stop_point: Optional[StopPoint] = None
    cruise_speed: Optional[float] = None
