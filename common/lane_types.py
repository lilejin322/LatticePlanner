"""Lane-related datatypes shared by map path and reference line code."""

from dataclasses import dataclass
from enum import Enum
from typing import List

from common.lane_info import LaneInfo

__all__ = [
    "InterpolatedIndex",
    "RoadType",
    "SpeedLimit",
    "LaneSegment",
]


@dataclass
class InterpolatedIndex:
    id: int = 0
    offset: float = 0.0


class RoadType(Enum):
    UNKNOWN = 0
    HIGHWAY = 1
    CITY_ROAD = 2
    PARK = 3


@dataclass
class SpeedLimit:
    """Speed limit segment along a reference line (m/s)."""

    start_s: float = 0.0
    end_s: float = 0.0
    speed_limit: float = 0.0


@dataclass
class LaneSegment:
    lane: LaneInfo = None
    start_s: float = 0.0
    end_s: float = 0.0

    def __post_init__(self):
        # Apollo's LaneSegment allows a null lane for placeholder segments.
        return None

    def Length(self) -> float:
        return self.end_s - self.start_s

    @staticmethod
    def Join(segments: List["LaneSegment"]) -> None:
        k_segment_delta = 0.5
        k = 0
        i = 0
        while i < len(segments):
            j = i
            while j + 1 < len(segments) and segments[i].lane == segments[j + 1].lane:
                j += 1

            segment_k = segments[k]
            segment_k.lane = segments[i].lane
            segment_k.start_s = segments[i].start_s
            segment_k.end_s = segments[j].end_s
            if segment_k.start_s < k_segment_delta:
                segment_k.start_s = 0.0
            if segment_k.end_s + k_segment_delta >= segment_k.lane.total_length:
                segment_k.end_s = segment_k.lane.total_length

            i = j + 1
            k += 1

        segments[:] = segments[:k]

    def __str__(self) -> str:
        if not self.lane:
            return "(lane is null)"
        return f"id = {self.lane.id.id}  start_s = {self.start_s}  end_s = {self.end_s}"
