"""
Apollo map loading and queries, backed by one explicitly configured service.

Re-implemented using shapely STRtree rather than the Apollo's original KDTree, will be faster.

Author: Lejin Li, Kyushu University & National Institute of Informatics, Japan
Contact: li.lejin.147@s.kyushu-u.ac.jp
"""

from .map_service import MapService
from .types import LaneMarking, LaneSignalOverlap, LaneStopLine, PositionEstimate

__all__ = ["MapService", "PositionEstimate", "LaneMarking", "LaneSignalOverlap", "LaneStopLine"]
