"""Trajectory type labels produced by one lattice cycle."""

from enum import Enum


class TrajectoryType(Enum):
    UNKNOWN = 0
    NORMAL = 1
    PATH_FALLBACK = 2
