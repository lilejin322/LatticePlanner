"""
Common geometry conversion and spatial indexing for map queries and relationship analysis.

Author: Lejin Li, Kyushu University & National Institute of Informatics, Japan
Contact: li.lejin.147@s.kyushu-u.ac.jp
"""

import numpy as np
from bisect import bisect_right
from itertools import accumulate
from math import atan2, ceil, hypot, pi
from google.protobuf.message import Message
from shapely import STRtree
from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import substring
from typing import Callable, Generic, Iterable, List, Mapping, NamedTuple, Optional, Tuple, TypeVar

T = TypeVar("T")


def curve_to_line(curve: Message) -> LineString:
    """
    Read every line segment, omitting repeated segment endpoints.

    :param Message curve: An ``apollo.hdmap.Curve`` message.
    :returns: A line connecting all points, with consecutive duplicates removed.
    :rtype: LineString
    :raises ValueError: If fewer than two points remain after deduplication.
    """
    points: List[Tuple[float, float]] = []
    for segment in curve.segment:
        for point in segment.line_segment.point:
            coordinate = (point.x, point.y)
            if not points or points[-1] != coordinate:
                points.append(coordinate)
    if len(points) < 2:
        raise ValueError("A map curve must contain at least two distinct points")
    return LineString(points)


def polygon_to_shape(polygon: Message) -> Optional[Polygon]:
    """
    Read a junction's or crosswalk's polygon.

    :param Message polygon: An ``apollo.hdmap.Polygon`` message.
    :returns: The polygon, or None if it has no points.
    :rtype: Optional[Polygon]
    :raises ValueError: If the polygon has fewer than three points.
    """
    points: List[Tuple[float, float]] = [(point.x, point.y) for point in polygon.point]
    if not points:
        return None
    if len(points) < 3:
        raise ValueError("A map polygon must contain at least three points")
    return Polygon(points)


def curves_to_lines(curves: Iterable[Message]) -> Optional[MultiLineString]:
    """
    Read several curves, such as a signal's or stop sign's stop lines, as one geometry.

    :param Iterable[Message] curves: ``apollo.hdmap.Curve`` messages.
    :returns: The lines, or None if there are no curves.
    :rtype: Optional[MultiLineString]
    :raises ValueError: If a curve has fewer than two distinct points.
    """
    lines: List[LineString] = [curve_to_line(curve) for curve in curves]
    return MultiLineString(lines) if lines else None


def split_boundary(boundary: Message) -> List[Tuple[float, float, Message, LineString]]:
    """
    Split a lane boundary's curve where its marking changes.

    Each ``LaneBoundaryType`` piece runs from its ``s`` to the next piece's
    ``s``, or to the end of the curve; ``s`` is clamped to the curve's
    geometric length. A stretch before the first piece has no marking.

    :param Message boundary: An ``apollo.hdmap.LaneBoundary`` message.
    :returns: (start s, end s, LaneBoundaryType piece, line) for each piece of
        positive length; none if the boundary has no curve or no pieces.
    :rtype: List[Tuple[float, float, Message, LineString]]
    :raises ValueError: If the curve has fewer than two distinct points.
    """
    if not boundary.curve.segment or not boundary.boundary_type:
        return []
    line: LineString = curve_to_line(boundary.curve)
    # LaneBoundaryType messages, in order of their start s along the curve.
    pieces: List[Message] = sorted(boundary.boundary_type, key=lambda piece: piece.s)
    starts: List[float] = [min(max(piece.s, 0.0), line.length) for piece in pieces]
    ends: List[float] = starts[1:] + [line.length]
    # Most boundaries have one piece covering the whole curve; substring is slow.
    return [(start, end, piece, line if (start, end) == (0.0, line.length) else substring(line, start, end))
            for start, end, piece in zip(starts, ends, pieces) if end > start]


class SpatialIndex(NamedTuple, Generic[T]):
    """
    An STRtree over map geometries and the item each belongs to, in tree order.

    Items are object IDs, or records such as ``LaneMarking``; they must be
    orderable, since equally near ones are returned in item order.
    """
    items: List[T]
    """Object IDs or records, in the same order as the tree's geometries."""
    tree: STRtree
    """Spatial index whose geometry indices correspond to positions in items."""

    @classmethod
    def build(cls, kind: str, objects: Mapping[str, Message],
              geometry: Callable[[Message], Optional[BaseGeometry]]) -> "SpatialIndex[str]":
        """
        Index the geometry of every object, in mapping order.

        :param str kind: Object kind, for error messages.
        :param Mapping[str, Message] objects: Objects by ID.
        :param Callable[[Message], Optional[BaseGeometry]] geometry: Returns an object's geometry, or None to leave it out.
        :returns: The index of the IDs of the objects that have a geometry.
        :rtype: SpatialIndex[str]
        :raises ValueError: Naming the first object whose geometry is malformed.
        """
        ids, geometries = [], []
        for object_id, obj in objects.items():
            try:
                shape: Optional[BaseGeometry] = geometry(obj)
            except ValueError as error:
                raise ValueError(f"invalid geometry for {kind} {object_id!r}: {error}") from error
            if shape is not None:
                ids.append(object_id)
                geometries.append(shape)
        return cls(ids, STRtree(geometries))

    def within(self, geometry: BaseGeometry, distance: float) -> List[T]:
        """
        Return the items whose geometry lies within ``distance`` of ``geometry``.

        :param BaseGeometry geometry: Query geometry in map coordinates, such as
            a point or a vehicle footprint.
        :param float distance: Search radius in metres; 0 means intersecting,
            and a polygon containing the query geometry is at distance 0.
        :returns: Items nearest first, equally near ones in item order.
        :rtype: List[T]
        """
        geometries = self.tree.geometries
        found = sorted((geometries[index].distance(geometry), self.items[index])
                       for index in self.tree.query(geometry, predicate="dwithin", distance=distance))
        return [item for _, item in found]

    def nearest(self, point: Point) -> Optional[T]:
        """
        Return the item whose geometry is nearest to ``point``.

        :param Point point: Query position in map coordinates.
        :returns: The nearest item, the earliest in index order among equally
            near ones (such as coincident lanes), or None for an empty index.
        :rtype: Optional[T]
        """
        indices = self.tree.query_nearest(point)
        return self.items[min(indices)] if len(indices) else None


def chord_heading(line: LineString, start: float, end: float) -> float:
    """
    Return the direction of the chord between two positions along a line.

    Over a few metres a chord follows the line's course, unlike one densely
    sampled segment, whose direction a few centimetres can change by degrees.

    :param LineString line: The line.
    :param float start: Metres along the line from its first point to the chord's start.
    :param float end: Metres to the chord's end, greater than ``start``.
    :returns: Direction in radians within ``[-pi, pi]``.
    :rtype: float
    """
    a, b = line.interpolate(start), line.interpolate(end)
    return atan2(b.y - a.y, b.x - a.x)


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [-pi, pi).

    :param float angle: Angle in radians.
    :returns: The equivalent angle in radians within [-pi, pi).
    :rtype: float
    """
    return (angle + pi) % (2 * pi) - pi


class CenterlineProfile:
    """
    Cumulative s and segment headings of a centerline, for heading queries.

    Each segment's direction is anchored at its midpoint, and headings are
    interpolated between neighbouring midpoints. They are continuous and match
    a smooth curve's tangent at both segment midpoints and vertices. Before the
    first and after the last midpoint, the end segment's direction is used.
    Apollo's LaneInfo::Heading interpolates between vertices instead, which
    turns ahead of the curve, so results are not identical to Apollo's.

    :param LineString line: The lane centerline used to compute cumulative distances
                            and segment headings.
    """
    line: LineString
    """The centerline the profile describes."""
    cumulative_s: List[float]
    """Metres from the first point to each vertex; one more entry than there are segments."""
    headings: List[float]
    """Direction of each segment in radians, from its start vertex to its end vertex."""
    midpoint_s: List[float]
    """Metres from the first point to each segment's midpoint, where its heading is anchored."""

    def __init__(self, line: LineString) -> None:
        """
        Constructor

        :param LineString line: The lane centerline used to compute cumulative distances
                                and segment headings.
        """
        coords: List[Tuple[float, float]] = list(line.coords)
        pairs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = list(zip(coords, coords[1:]))
        lengths: List[float] = [hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in pairs]
        self.line: LineString = line
        self.cumulative_s: List[float] = [0.0, *accumulate(lengths)]
        self.headings: List[float] = [atan2(y2 - y1, x2 - x1) for (x1, y1), (x2, y2) in pairs]
        self.midpoint_s: List[float] = [start + length / 2 for start, length in zip(self.cumulative_s, lengths)]

    def heading_at(self, s: float) -> float:
        """
        Get the heading at s, interpolated between neighbouring segment midpoints.

        Between two midpoints the heading turns linearly with s, the shorter
        way round; before the first and after the last midpoint it is the end
        segment's direction, so any s, even outside the line, has a heading.
        The line must have no zero-length segments, as ``curve_to_line``
        guarantees.

        :param float s: Metres along the line from its first point.
        :returns: Heading in radians within ``[-pi, pi)``.
        :rtype: float
        """
        index: int = bisect_right(self.midpoint_s, s)
        if index == 0:
            return self.headings[0]
        if index == len(self.headings):
            return self.headings[-1]
        start, end = self.midpoint_s[index - 1], self.midpoint_s[index]
        before: float = self.headings[index - 1]
        turn: float = normalize_angle(self.headings[index] - before)
        return normalize_angle(before + turn * (s - start) / (end - start))


REPLAY_TOLERANCE = 0.05
"""Metres a replayed path may deviate from lane geometry when vertex noise is dropped."""


def turn_curvatures(line: LineString) -> List[Tuple[float, float, float]]:
    """
    Return ``(low, high, curvature)`` for every interval between neighbouring
    segment midpoints of ``line``, under the heading model of
    ``CenterlineProfile``: the heading turns linearly between midpoints, so
    each interval has a constant absolute curvature. Intervals without a turn
    are omitted.

    :param LineString line: A line without zero-length segments.
    :returns: Turning intervals as ``(low, high, curvature)`` tuples, where
        ``low`` and ``high`` are distances along the line in metres and
        ``curvature`` is the absolute heading change per metre. Returns an
        empty list if the line has no turns.
    :rtype: List[Tuple[float, float, float]]
    """
    profile = CenterlineProfile(line)
    spans = []
    for index in range(1, len(profile.midpoint_s)):
        low, high = profile.midpoint_s[index - 1:index + 1]
        if high <= low:
            continue
        turn = abs(normalize_angle(profile.headings[index] - profile.headings[index - 1]))
        if turn:
            spans.append((low, high, turn / (high - low)))
    return spans


def replay_line(points: Iterable[Tuple[float, ...]]) -> Optional[LineString]:
    """
    The line a vehicle replays through ``points``: vertex noise within
    ``REPLAY_TOLERANCE`` dropped. None if fewer than two distinct points remain.

    :param Iterable[Tuple[float, ...]] points: Ordered points with at least two coordinates each. Only
        the first two coordinates (X and Y) are used; consecutive duplicate XY coordinates are removed.
    :returns: A line through the input XY coordinates, simplified within
        ``REPLAY_TOLERANCE``, or ``None`` if fewer than two coordinates remain
        after removing consecutive duplicates.
    :rtype: LineString | None
    """
    coords: List[Tuple[float, float]] = []
    for point in points:
        xy = (point[0], point[1])
        if not coords or coords[-1] != xy:
            coords.append(xy)
    if len(coords) < 2:
        return None
    return LineString(coords).simplify(REPLAY_TOLERANCE, preserve_topology=False)


SMOOTHING_SIGMA = 2.0
"""Metres: standard deviation of the Gaussian that rounds a replayed path's corners."""
SMOOTHING_STEP = 0.5
"""Largest metres between the vertices of a smoothed replay path."""


def smooth_polyline(points: Iterable[Tuple[float, ...]], sigma: float = SMOOTHING_SIGMA,
                    step: float = SMOOTHING_STEP) -> Tuple[np.ndarray, np.ndarray]:
    """
    Resample a polyline every ``step`` metres or less and smooth it with a
    Gaussian of ``sigma`` metres along its length, keeping both end points.

    Coarse maps join long straight segments at sharp vertices; a vehicle that
    follows such a polyline changes direction instantly at every vertex while
    its heading model turns gradually, so pose and motion disagree. The
    smoothed line has its curvature spread over a few metres around each
    vertex, so headings derived from it match the direction of motion.
    Straight stretches stay straight; the ends are extended along their end
    segments before filtering, then any residual end offset is faded out over
    ``3 * sigma``.

    :param Iterable[Tuple[float, ...]] points: Ordered polyline vertices, each
        containing at least X and Y in metres. Additional coordinates are
        ignored, and consecutive duplicate XY coordinates are removed.
    :param float sigma: Gaussian smoothing standard deviation in metres
        along the polyline. Nonpositive values disable smoothing but retain
        resampling.
    :param float step: Maximum resampling interval in metres along the input
        polyline. Must be positive.
    :returns: The smoothed vertices, shape (n, 2), and for each the distance
        along the input polyline it was resampled at.
    :rtype: Tuple[np.ndarray, np.ndarray]
    """
    raw = np.array([(point[0], point[1]) for point in points], dtype=float)
    if len(raw) > 1:
        raw = raw[np.r_[True, np.hypot(*np.diff(raw, axis=0).T) > 0]]
    if len(raw) < 2:
        return raw, np.zeros(len(raw))
    along = np.r_[0.0, np.cumsum(np.hypot(*np.diff(raw, axis=0).T))]
    length = along[-1]
    count = max(2, ceil(length / step))
    s = np.linspace(0.0, length, count + 1)
    s[-1] = length
    x, y = np.interp(s, along, raw[:, 0]), np.interp(s, along, raw[:, 1])
    if sigma <= 0:
        return np.column_stack([x, y]), s
    spacing = length / count
    reach = ceil(3 * sigma / spacing)
    offsets = np.arange(-reach, reach + 1) * spacing
    kernel = np.exp(-0.5 * (offsets / sigma) ** 2)
    kernel /= kernel.sum()
    first = (raw[1] - raw[0]) / np.hypot(*(raw[1] - raw[0]))
    last = (raw[-1] - raw[-2]) / np.hypot(*(raw[-1] - raw[-2]))
    before = np.arange(reach, 0, -1) * spacing
    after = np.arange(1, reach + 1) * spacing
    smoothed = []
    for values, start, end in ((x, first[0], last[0]), (y, first[1], last[1])):
        padded = np.r_[values[0] - before * start, values, values[-1] + after * end]
        smoothed.append(np.convolve(padded, kernel, mode="valid"))
    # Fade any end offset out: c(s) = a * r0(s) + b * r1(s) with c(0), c(L) exact.
    fade = 3 * sigma
    r0, r1 = np.clip(1 - s / fade, 0, 1), np.clip(1 - (length - s) / fade, 0, 1)
    k0, k1 = r1[0], r0[-1]
    determinant = 1 - k0 * k1
    for values, result in zip((x, y), smoothed):
        e0, e1 = result[0] - values[0], result[-1] - values[-1]
        a = (e0 - k0 * e1) / determinant
        b = (e1 - k1 * e0) / determinant
        result -= a * r0 + b * r1
    return np.column_stack(smoothed), s


def max_replay_curvature(points: Iterable[Tuple[float, ...]]) -> float:
    """
    The largest curvature a vehicle replaying ``points`` would drive, in 1/m:
    noise dropped as in ``replay_line``, corners rounded by ``smooth_polyline``.

    :param Iterable[Tuple[float, ...]] points: Ordered polyline vertices, each
        containing at least X and Y in metres. Additional coordinates are
        ignored, and consecutive duplicate XY coordinates are removed.
    :returns: Maximum absolute curvature of the simplified and smoothed
        replay path, in inverse metres. Returns ``0.0`` if no usable line
        remains, fewer than three smoothed vertices remain, or there are
        no turns.
    :rtype: float
    """
    line = replay_line(points)
    if line is None or line.length == 0:
        return 0.0
    vertices, _ = smooth_polyline(line.coords)
    if len(vertices) < 3:
        return 0.0
    return max((curvature for _, _, curvature in turn_curvatures(LineString(vertices))), default=0.0)
