"""
Results returned by map queries.

Author: Lejin Li, Kyushu University & National Institute of Informatics, Japan
Contact: li.lejin.147@s.kyushu-u.ac.jp
"""

from typing import Tuple
from dataclasses import dataclass, field
from shapely.geometry import LineString


@dataclass
class PositionEstimate:
    """
    A point's position on the map: its nearest lane and the distance along it.

    Returned by ``MapService.get_position``. Both fields are None when the map
    has no lanes.
    """
    lane_id: str | None = None
    """The lane whose complete centerline is nearest to the point, of any lane type."""
    s: float | None = None
    """Metres along that centerline to the point's projection, clamped to its ends;
    usable with ``get_lane_coord_and_heading``."""


@dataclass(frozen=True)
class LaneSignalOverlap:
    """
    A signal association and its longitudinal interval on one lane.

    Missing protobuf coordinates remain None, rather than becoming zero.
    """
    overlap_id: str
    """The map overlap holding both the lane and the signal."""
    lane_id: str
    """The lane the signal controls."""
    signal_id: str
    """The controlling signal."""
    start_s: float | None
    """Start of the interval on the lane in metres, as stored in the map; None if missing."""
    end_s: float | None
    """End of the interval on the lane in metres, as stored in the map; None if missing."""


@dataclass(frozen=True)
class LaneStopLine:
    """
    A signal stop-line candidate matched to explicit lane overlap intervals.

    Geometry is the complete original stop line, not clipped to the lane.
    distance is the minimum distance to the supporting lane intervals in metres.
    """
    lane_id: str
    """The lane the stop line was matched to."""
    signal_id: str
    """The signal the stop line belongs to."""
    stop_line_index: int
    """Position of the stop line in the signal's ``stop_line`` list."""
    geometry: LineString
    """The complete stop line, which may span several lanes."""
    overlaps: tuple[LaneSignalOverlap, ...]
    """The lane's overlap intervals with the signal that lie within the tolerance."""
    distance: float
    """Metres from the stop line to the nearest of those intervals."""


PROHIBITING_MARKINGS = frozenset({"SOLID_WHITE", "SOLID_YELLOW", "DOUBLE_YELLOW", "CURB"})
"""``LaneBoundaryType.Type`` names that a vehicle may not cross."""


@dataclass(frozen=True, order=True)
class LaneMarking:
    """
    A stretch of one lane boundary whose markings do not change.

    Adjacent lanes describe a shared line separately, each as its own boundary.
    Markings order by lane, side, and position; geometry is not compared.
    """
    lane_id: str
    """The lane whose boundary this is."""
    side: str
    """``"left"`` or ``"right"`` of the lane, in its driving direction."""
    start_s: float
    """Start along the boundary curve, in metres from its first point."""
    end_s: float
    """End along the boundary curve, in metres from its first point."""
    types: Tuple[str, ...]
    """``LaneBoundaryType.Type`` names, such as ``("SOLID_WHITE",)``."""
    virtual: bool
    """Whether the boundary does not exist in the real world, as inside junctions."""
    geometry: LineString = field(compare=False, repr=False)
    """The stretch of the boundary curve from ``start_s`` to ``end_s``; ignored in comparisons."""

    @property
    def prohibits_crossing(self) -> bool:
        """
        Whether a vehicle may not cross this marking: a real solid line, double
        yellow line, or curb. Virtual boundaries and dotted or UNKNOWN markings
        do not prohibit crossing; with several types, any prohibiting one does.

        :rtype: bool
        """
        return not self.virtual and not PROHIBITING_MARKINGS.isdisjoint(self.types)
