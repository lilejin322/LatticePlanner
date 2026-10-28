"""Identity of the reference line the vehicle is following.

Lattice asks whether the vehicle is already on this segment. An empty segment
is not on-segment, which is the same default the old route passage used.
"""

from copy import deepcopy

kSegmentationEpsilon: float = 0.2


class RouteSegments(list):
    def __init__(self, segments=None):
        super().__init__(segments or [])
        self._is_on_segment = False
        self._id = ""

    def __deepcopy__(self, memo):
        copied = RouteSegments(deepcopy(list(self), memo))
        copied._is_on_segment = self._is_on_segment
        copied._id = self._id
        memo[id(self)] = copied
        return copied

    def IsOnSegment(self) -> bool:
        return self._is_on_segment

    def SetIsOnSegment(self, on_segment: bool) -> None:
        self._is_on_segment = on_segment

    def Id(self) -> str:
        return self._id

    def SetId(self, id: str) -> None:
        self._id = id
