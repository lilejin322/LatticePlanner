"""
Derived map relationships; all source objects belong to MapService.

Author: Lejin Li, Kyushu University & National Institute of Informatics, Japan
Contact: li.lejin.147@s.kyushu-u.ac.jp
"""

from collections import defaultdict
from functools import cached_property
from math import atan2, cos, degrees, hypot, isfinite, sin
import networkx as nx
from google.protobuf.message import Message
from shapely import make_valid
from shapely.geometry import LineString, MultiPoint, Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import substring
from typing import (TYPE_CHECKING, Callable, Iterable, Iterator, List,
                                  Mapping, NamedTuple, Optional, Set, Tuple, Dict)

from .geometry import chord_heading, curve_to_line, normalize_angle, polygon_to_shape
from .types import LaneSignalOverlap, LaneStopLine
from ..utils import require_nonnegative

if TYPE_CHECKING:
    from .map_service import MapService


SAME_APPROACH = 45.0
"""Signals whose controlled lanes enter within this many degrees share an approach."""
OPPOSING_APPROACH = 135.0
"""Signals whose controlled lanes enter more than this many degrees apart are opposing."""
LEFT_TURN = 45.0
"""A movement turning left by more than this many degrees, or any U-turn, yields to oncoming traffic."""
DIRECTION_CHORD = 5.0
"""Metres over which lane directions are measured, as chords rather than single segments."""
CROSSING_CLEARANCE = 1.0
"""Metres from the ends of two movements within which meeting is a merge or diverge, not a crossing."""


def _overlap_ids(obj: Message) -> Set[str]:
    """
    Return the overlap IDs a map object refers to.

    ``overlap_id`` holds ``apollo.hdmap.Id`` messages; their strings are
    returned as a set, so repeated references count once.

    :param Message obj: A lane, junction, signal, or other object with ``overlap_id``.
    :returns: Overlap IDs.
    :rtype: Set[str]
    """
    return {overlap.id for overlap in obj.overlap_id}


def forward_links(lane_id: str, lane: Message,
                  lanes: Mapping[str, Message]) -> Iterator[Tuple[str, str]]:
    """
    Yield the forward (from, to) lane pairs that one lane declares.

    A lane links each known predecessor to itself and itself to each known
    successor; unknown IDs are skipped, as in Apollo's ``GraphCreator::AddEdge``.

    :param str lane_id: The lane's ID.
    :param Message lane: The ``apollo.hdmap.Lane`` message.
    :param Mapping[str, Message] lanes: All lanes by ID.
    :returns: (from lane ID, to lane ID) pairs, predecessors first; a pair may
        repeat if both lanes declare it.
    :rtype: Iterator[Tuple[str, str]]
    """
    for other in lane.predecessor_id:
        if other.id in lanes:
            yield other.id, lane_id
    for other in lane.successor_id:
        if other.id in lanes:
            yield lane_id, other.id


class _Movement(NamedTuple):
    """
    One path of a signal's traffic, judged inside its junction.
    """
    yields: bool
    """Whether it turns left or makes a U-turn, and so yields to oncoming traffic."""
    line: Optional[BaseGeometry]
    """Its course after the signal, inside the junction polygon if it has one, as one line."""


def _cross_inside(first: Optional[BaseGeometry], second: Optional[BaseGeometry]) -> bool:
    """
    Whether two movement lines cross away from their ends.

    Only meeting farther than ``CROSSING_CLEARANCE`` from where either line
    starts or ends counts, so merges, diverges, and lines that barely touch near
    an end do not. Shared stretches, as of movements meeting head-on, count
    wherever they extend beyond that clearance.

    :param Optional[BaseGeometry] first: One movement's line, or None if it has none.
    :param Optional[BaseGeometry] second: The other movement's line, or None.
    :returns: True if they cross; False if either is None.
    :rtype: bool
    """
    if first is None or second is None:
        return False
    hit = first.intersection(second)
    if hit.is_empty:
        return False
    ends = MultiPoint([coords for line in (first, second) for part in getattr(line, "geoms", [line])
                       for coords in (part.coords[0], part.coords[-1])])
    return not hit.difference(ends.buffer(CROSSING_CLEARANCE)).is_empty


def _meet(left: Iterable[Tuple[str, BaseGeometry]],
          right: Iterable[Tuple[str, BaseGeometry]]) -> bool:
    """
    Whether a lane geometry on one side intersects that of a different lane on the other.

    :param Iterable[Tuple[str, BaseGeometry] left: (lane ID, geometry) pairs.
    :param Iterable[Tuple[str, BaseGeometry] right: (lane ID, geometry) pairs.
    :returns: Whether meet result
    :rtype: bool
    """
    right = list(right)
    return any(a != b and x.intersects(y) for a, x in left for b, y in right)


class MapRelations:
    """
    Lazy overlap and signal analysis.

    Signal EQ/NE labels model a junction with circular lights from the lanes
    each signal controls; Apollo maps carry no signal phases, so they are
    inferred, not read.
    """

    def __init__(self, service: "MapService") -> None:
        """
        Bind the relations to one loaded map.

        :param MapService service: The service whose tables are analysed; every
            relationship is computed from them on first access.
        """
        self._service = service

    def _at_junction(self, objects: Mapping[str, Message]) -> Dict[str, List[str]]:
        """
        Group objects by the junctions they share an overlap ID with.

        :param Mapping[str, Message] objects: Objects by ID, such as lanes or signals.
        :returns: Sorted object IDs for every junction; empty lists included.
        :rtype: Dict[str, List[str]]
        """
        by_overlap = defaultdict(list)
        for object_id, obj in objects.items():
            for overlap_id in _overlap_ids(obj):
                by_overlap[overlap_id].append(object_id)
        result = {}
        for junction_id, junction in self._service.junction_table.items():
            matches = {
                object_id for overlap_id in _overlap_ids(junction)
                for object_id in by_overlap[overlap_id]
            }
            result[junction_id] = sorted(matches)
        return result

    @cached_property
    def signals_at_junction(self) -> Dict[str, List[str]]:
        """
        Signals sharing an overlap with a junction or controlling its lanes.

        Some maps (e.g. the Sunnyvale loops) have no signal-junction overlaps,
        so a signal also joins each junction containing a lane it controls.

        :returns: Sorted signal IDs by junction ID; every junction included,
            with an empty list if it has no signals.
        :rtype: Dict[str, List[str]]
        """
        result = {junction_id: set(signals) for junction_id, signals
                  in self._at_junction(self._service.signal_table).items()}
        # Overlapping lanes, not road membership: a few controlled lanes in
        # sunnyvale_loop overlap a junction without belonging to its road.
        for junction_id, lanes in self.lanes_overlapping_junction.items():
            for lane_id in lanes:
                result[junction_id].update(self.signals_for_lane[lane_id])
        return {junction_id: sorted(signals) for junction_id, signals in result.items()}

    @cached_property
    def lanes_overlapping_junction(self) -> Dict[str, List[str]]:
        """
        Lanes sharing an overlap with each junction, even only at one end.

        Junction polygons often cover the first or last few metres of the lanes
        entering and leaving them, so these include such lanes, not just the
        lanes crossing the junction.

        :returns: Sorted lane IDs by junction ID; every junction included,
            with an empty list if no lane overlaps it.
        :rtype: Dict[str, List[str]]
        """
        return self._at_junction(self._service.lane_table)

    @cached_property
    def lane_signal_overlaps(self) -> Dict[str, Tuple[LaneSignalOverlap, ...]]:
        """
        Resolve each lane's overlaps directly, without a junction prerequisite.

        As in Apollo's LaneInfo, follow lane.overlap_id into the overlap table.
        Require typed lane/signal participants and existing signal objects;
        dangling references or unrelated overlap objects cannot create a link.

        :returns: One record per overlap and signal by lane ID, ordered by
            overlap ID, signal ID, and interval; every lane included, with an
            empty tuple if no signal overlaps it.
        :rtype: Dict[str, Tuple[LaneSignalOverlap, ...]]
        """
        result = {}
        for lane_id, lane in self._service.lane_table.items():
            records = set()
            for overlap_id in sorted(_overlap_ids(lane)):
                overlap = self._service.overlap_table.get(overlap_id)
                if overlap is None:
                    continue
                lane_objects = [obj for obj in overlap.object
                                if obj.id.id == lane_id and obj.HasField("lane_overlap_info")]
                signal_ids = {obj.id.id for obj in overlap.object
                              if obj.HasField("signal_overlap_info")
                              and obj.id.id in self._service.signal_table}
                for obj in lane_objects:
                    info = obj.lane_overlap_info
                    for signal_id in signal_ids:
                        records.add(LaneSignalOverlap(
                            overlap_id, lane_id, signal_id,
                            info.start_s if info.HasField("start_s") else None,
                            info.end_s if info.HasField("end_s") else None,
                        ))
            result[lane_id] = tuple(sorted(records, key=lambda record: (
                record.overlap_id, record.signal_id,
                (record.start_s is not None, record.start_s),
                (record.end_s is not None, record.end_s),
            )))
        return result

    @cached_property
    def signal_lane_overlaps(self) -> Dict[str, Tuple[LaneSignalOverlap, ...]]:
        """
        The records of ``lane_signal_overlaps``, grouped by signal instead of lane.

        :returns: Records by signal ID, ordered by lane ID; every signal included.
        :rtype: Dict[str, Tuple[LaneSignalOverlap, ...]]
        """
        result = {signal_id: [] for signal_id in self._service.signal_table}
        for lane_id in sorted(self.lane_signal_overlaps):
            for record in self.lane_signal_overlaps[lane_id]:
                result[record.signal_id].append(record)
        return {key: tuple(value) for key, value in result.items()}

    @cached_property
    def lanes_controlled_by_signal(self) -> Dict[str, List[str]]:
        """
        The lanes each signal overlaps, often only short entry pieces of its movements.

        :returns: Sorted lane IDs by signal ID; every signal included.
        :rtype: Dict[str, List[str]]
        """
        return {signal_id: sorted({record.lane_id for record in records})
                for signal_id, records in self.signal_lane_overlaps.items()}

    @cached_property
    def signals_for_lane(self) -> Dict[str, List[str]]:
        """
        The signals overlapping each lane; lanes after a controlled entry piece have none.

        :returns: Sorted signal IDs by lane ID; every lane included.
        :rtype: Dict[str, List[str]]
        """
        return {lane_id: sorted({record.signal_id for record in records})
                for lane_id, records in self.lane_signal_overlaps.items()}

    def get_signal_stop_lines(self, signal_id: str, lane_id: str,
                              max_distance: float) -> Tuple[LaneStopLine, ...]:
        """
        Match a signal's stop-line curves to the lane's overlap intervals.

        Apollo has no lane ID on Signal.stop_line. The explicit lane-signal
        association is authoritative; matching individual curves is geometric,
        so every curve within max_distance is returned, never a guessed one.

        :param str signal_id: Signal ID.
        :param str lane_id: Lane ID.
        :param float max_distance: Largest distance in metres between a stop
            line and the lane's centerline over one of its overlap intervals.
        :returns: One record per matching stop line, in the signal's stop-line
            order; empty if they share no usable interval or the signal has no
            stop line.
        :rtype: Tuple[LaneStopLine, ...]
        :raises KeyError: If the map has no such signal or lane.
        :raises ValueError: If ``max_distance`` is negative or not finite, or a
            stop line of the signal is malformed.
        """
        require_nonnegative(max_distance=max_distance)
        signal = self._service.get_signal_by_id(signal_id)
        self._service.get_lane_by_id(lane_id)  # Unknown IDs are errors, even if unassociated.
        overlaps = [record for record in self.lane_signal_overlaps[lane_id]
                    if record.signal_id == signal_id]
        if not overlaps or not signal.stop_line:
            return ()

        centerline = self._service.get_lane_central_curve_by_id(lane_id)
        intervals = []
        for record in overlaps:
            start, end = record.start_s, record.end_s
            if (start is None or end is None or not isfinite(start) or not isfinite(end)
                    or start < 0 or end < start or start > centerline.length):
                continue
            # Clip an interval's end to the available centerline geometry.
            intervals.append((record, substring(centerline, start, min(end, centerline.length))))
        if not intervals:
            return ()

        result = []
        for index, curve in enumerate(signal.stop_line):
            try:
                geometry = curve_to_line(curve)
            except ValueError as error:
                raise ValueError(f"Invalid stop_line[{index}] for signal {signal_id!r}") from error
            if not geometry.is_valid:
                raise ValueError(f"Invalid stop_line[{index}] for signal {signal_id!r}")
            matches = [(record, geometry.distance(section)) for record, section in intervals]
            matches = [(record, distance) for record, distance in matches if distance <= max_distance]
            if matches:
                result.append(LaneStopLine(
                    lane_id=lane_id,
                    signal_id=signal_id,
                    stop_line_index=index,
                    geometry=geometry,
                    overlaps=tuple(record for record, _ in matches),
                    distance=min(distance for _, distance in matches),
                ))
        return tuple(result)

    def get_lane_stop_lines(self, lane_id: str, max_distance: float) -> Tuple[LaneStopLine, ...]:
        """
        Match the stop lines of every signal overlapping a lane.

        :param str lane_id: Lane ID.
        :param float max_distance: As in ``get_signal_stop_lines``.
        :returns: Matches ordered by signal ID, then by each signal's stop-line order.
        :rtype: Tuple[LaneStopLine, ...]
        :raises KeyError: If the map has no such lane.
        :raises ValueError: If ``max_distance`` is negative or not finite, or a
            matched signal's stop line is malformed.
        """
        # Validate even when the lane has no signals and nothing is matched.
        require_nonnegative(max_distance=max_distance)
        return tuple(
            match
            for signal_id in self.signals_for_lane[lane_id]
            for match in self.get_signal_stop_lines(signal_id, lane_id, max_distance)
        )

    def is_conflict_lanes(self, lane_ids1: List[str], lane_ids2: List[str]) -> bool:
        """
        Whether any lane of one group meets a different lane of the other.

        Complete centerlines count, and so does a touch at a shared end point.

        :param List[str] lane_ids1: Lane IDs of the first group.
        :param List[str] lane_ids2: Lane IDs of the second group.
        :rtype: bool
        :raises KeyError: If either group names an unknown lane.
        """
        curve = self._service.get_lane_central_curve_by_id
        return _meet(((lane, curve(lane)) for lane in lane_ids1), ((lane, curve(lane)) for lane in lane_ids2))

    @cached_property
    def forward_successors(self) -> Dict[str, List[str]]:
        """
        Each lane's forward successors, as in ``routing_graph``, without needing it.

        Built from the lane table with ``forward_links``, so it also works for
        maps loaded with ``load_only=True``, which have no routing graph.

        :returns: Successor lane IDs by lane ID, without duplicates, in the
            order the links are declared; lanes without successors are absent.
        :rtype: Dict[str, List[str]]
        """
        lanes = self._service.lane_table
        successors: Dict[str, Dict[str, None]] = defaultdict(dict)
        for lane_id, lane in lanes.items():
            for source, target in forward_links(lane_id, lane, lanes):
                successors[source][target] = None
        return {lane_id: list(targets) for lane_id, targets in successors.items()}

    def movement_paths(self, signal_id: str, junction_id: str) -> List[Tuple[str, ...]]:
        """
        Return the lane paths a signal's traffic takes through a junction.

        Signals may control only a short entry piece of each lane, so each
        controlled lane is followed forward through every successor that still
        overlaps the junction, until the path leaves it.

        :param str signal_id: Signal ID.
        :param str junction_id: A junction the signal belongs to.
        :returns: Paths of lane IDs, each starting with a controlled lane.
        :rtype: List[Tuple[str, ...]]
        """
        inside = set(self.lanes_overlapping_junction[junction_id])
        paths = []

        def extend(path: tuple[str, ...]) -> None:
            successors = [lane for lane in self.forward_successors.get(path[-1], ())
                          if lane in inside and lane not in path]
            if not successors:
                paths.append(path)
            for successor in successors:
                extend(path + (successor,))

        for lane_id in self.lanes_controlled_by_signal[signal_id]:
            extend((lane_id,))
        return paths

    @cached_property
    def signal_graph(self) -> nx.Graph:
        """
        Signals at a shared junction joined by ``relation`` ``"EQ"`` (same colour)
        or ``"NE"`` (never green together), as at a junction with circular lights:

        - controlled lanes that are the same, or enter from the same approach: EQ;
        - crossing approaches: NE;
        - opposing approaches: EQ, as left turns yield to oncoming traffic, unless
          movements cross where neither turns left (a skewed junction): NE.

        NE evidence takes precedence across junctions. EQ candidates merge
        groups only if no NE pair would end up inside a group; same-lane and
        same-approach candidates take priority over opposing approaches.
        Rejected EQ candidates become NE so configuration generation can
        separate their phases. Edges retain all original junction evidence.

        :returns: The signal Graph
        :rtype: nx.Graph
        """
        graph = nx.Graph()
        graph.add_nodes_from(sorted(self._service.signal_table))
        evidence = defaultdict(list)
        for junction_id, signals in sorted(self.signals_at_junction.items()):
            area = self._junction_areas.get(junction_id)
            movements: Dict[str, List[_Movement]] = {}

            def movements_of(signal: str) -> List[_Movement]:
                if signal not in movements:
                    movements[signal] = [self._movement(signal, path, area)
                                         for path in self.movement_paths(signal, junction_id)]
                return movements[signal]

            for index, left in enumerate(signals):
                for right in signals[index + 1:]:
                    relation = self._relation(left, right, movements_of)
                    if relation is not None:
                        evidence[left, right].append((junction_id, left, right, relation))
        for (left, right), records in sorted(evidence.items()):
            graph.add_edge(left, right,
                           relation="NE" if any(record[3] == "NE" for record in records) else "EQ",
                           junctions=tuple(record[0] for record in records),
                           evidence=tuple(records))
        self._resolve_signal_groups(graph)
        return graph

    def _resolve_signal_groups(self, graph: nx.Graph) -> None:
        """
        Resolve inferred EQ preferences into deterministic groups respecting every NE edge.

        EQ edges are tried in priority order, identical controlled lanes first,
        then the same approach, then opposing approaches; each merges two
        groups unless that would put an NE pair inside one. EQ edges left
        between groups become NE.

        :param nx.Graph graph: Edges with ``relation``; modified in place.
        """
        def priority(pair: tuple[str, str]) -> tuple[int, float, str, str]:
            left, right = pair
            if self.lanes_controlled_by_signal[left] == self.lanes_controlled_by_signal[right]:
                return 0, 0.0, left, right
            between = abs(degrees(normalize_angle(self._approach(left) - self._approach(right))))
            # Quantize ranking noise so global rotation preserves tie-breaking.
            return (1 if between < SAME_APPROACH else 2,
                    round(min(between, 180 - between), 9), left, right)

        groups = {signal: {signal} for signal in graph}
        candidates = sorted(((a, b) for a, b, r in graph.edges(data="relation") if r == "EQ"),
                            key=priority)
        for left, right in candidates:
            first, second = groups[left], groups[right]
            if first is second:
                continue
            if any(graph.has_edge(a, b) and graph[a][b]["relation"] == "NE"
                   for a in first for b in second):
                continue
            merged = first | second
            for signal in merged:
                groups[signal] = merged
        for left, right, data in graph.edges(data=True):
            if data["relation"] == "EQ" and groups[left] is not groups[right]:
                data["relation"] = "NE"

    def _relation(self, left: str, right: str,
                  movements_of: Callable[[str], List[_Movement]]) -> Optional[str]:
        """
        Infer one junction's relation between two of its signals.

        :param str left: A signal ID.
        :param str right: Another signal ID at the same junction.
        :param movements_of: Returns a signal's judged movements at this junction.
        :returns: ``"EQ"`` for the same lanes or approach, or opposing approaches
            whose straight movements do not cross; ``"NE"`` for crossing
            approaches, or opposing ones whose straight movements cross; None if
            either signal controls no lanes or has no known approach.
        :rtype: Optional[str]
        """
        left_lanes = self.lanes_controlled_by_signal[left]
        right_lanes = self.lanes_controlled_by_signal[right]
        if left_lanes and left_lanes == right_lanes:
            return "EQ"
        if not left_lanes or not right_lanes:
            return None
        left_direction, right_direction = self._approach(left), self._approach(right)
        if left_direction is None or right_direction is None:
            return None
        between = abs(degrees(normalize_angle(left_direction - right_direction)))
        if between < SAME_APPROACH:
            return "EQ"
        if between <= OPPOSING_APPROACH:
            return "NE"
        unyielding = any(not p.yields and not q.yields and _cross_inside(p.line, q.line)
                         for p in movements_of(left) for q in movements_of(right))
        return "NE" if unyielding else "EQ"

    def _heading(self, lane_id: str, s: float) -> float:
        """
        Return a lane's direction over a ``DIRECTION_CHORD`` chord from s.

        Near the lane's end the chord is shifted back to fit, rather than shortened.

        :param str lane_id: Lane ID.
        :param float s: Metres along the centerline where the chord starts.
        :returns: Direction in radians.
        :rtype: float
        """
        line = self._service.get_lane_central_curve_by_id(lane_id)
        end = min(max(s, 0.0) + DIRECTION_CHORD, line.length)
        return chord_heading(line, max(end - DIRECTION_CHORD, 0.0), end)

    def _path_line(self, signal_id: str, path: Tuple[str, ...]) -> LineString:
        """
        Return a movement's course from its signal along all its lanes, as one line.

        :param str signal_id: The signal; its overlap position starts the course.
        :param Tuple[str, ...] path: Lane IDs from ``movement_paths``.
        :returns: The course, repeated points removed.
        :rtype: LineString
        """
        coords = []
        for index, lane_id in enumerate(path):
            line = self._service.get_lane_central_curve_by_id(lane_id)
            if index == 0 and self._signal_entries[signal_id, lane_id] > 0:
                line = substring(line, self._signal_entries[signal_id, lane_id], line.length)
            for point in getattr(line, "coords", ()):
                if not coords or coords[-1] != point:
                    coords.append(point)
        return LineString(coords if len(coords) > 1 else coords * 2)

    @cached_property
    def _signal_entries(self) -> Dict[Tuple[str, str], float]:
        """
        Earliest finite signal overlap position on each controlled lane.

        This is where the signal starts governing traffic on the lane: approach
        directions are measured from it, and movements start there. Only
        ``start_s`` values within the lane's geometric length count.

        :returns: Metres along the lane by (signal ID, lane ID); pairs whose
            overlaps all lack a usable ``start_s`` are absent, which leaves
            that signal's approach unknown.
        :rtype: Dict[Tuple[str, str], float]
        """
        entries = {}
        for signal_id, records in self.signal_lane_overlaps.items():
            for record in records:
                start = record.start_s
                if start is None or not isfinite(start):
                    continue
                length = self._service.get_lane_central_curve_by_id(record.lane_id).length
                if 0 <= start <= length:
                    key = signal_id, record.lane_id
                    entries[key] = min(entries.get(key, start), start)
        return entries

    def _approach(self, signal_id: str) -> Optional[float]:
        """
        Return the mean direction of a signal's controlled lanes at its overlaps.

        :param str signal_id: Signal ID.
        :returns: Direction in radians, or None if the signal controls no lanes,
            a lane lacks a usable overlap position, or the directions cancel out.
        :rtype: Optional[float]
        """
        lanes = self.lanes_controlled_by_signal[signal_id]
        if not lanes or any((signal_id, lane) not in self._signal_entries for lane in lanes):
            return None
        directions = [self._heading(lane, self._signal_entries[signal_id, lane]) for lane in lanes]
        x, y = sum(map(cos, directions)), sum(map(sin, directions))
        return None if hypot(x, y) < 1e-9 * len(lanes) else atan2(y, x)

    @cached_property
    def _junction_areas(self) -> Dict[str, Optional[BaseGeometry]]:
        """
        Each junction's polygon, made valid; None where it has no usable polygon.

        Movements are judged inside these areas. A self-intersecting polygon is
        repaired with ``make_valid`` rather than rejected; a missing one, or one
        with fewer than three points, gives None, and that junction's movements
        are judged whole.

        :returns: Area by junction ID; every junction included.
        :rtype: Dict[str, Optional[BaseGeometry]]
        """
        areas = {}
        for junction_id, junction in self._service.junction_table.items():
            try:
                area = polygon_to_shape(junction.polygon)
            except ValueError:
                area = None
            areas[junction_id] = area if area is None or area.is_valid else make_valid(area)
        return areas

    def _movement(self, signal_id: str, path: Tuple[str, ...],
                  area: Optional[BaseGeometry]) -> _Movement:
        """
        Judge one movement inside its junction.

        Only its course after the signal counts, and only inside the junction
        polygon when there is one: exit lanes overlap a junction where they
        start but may run far beyond it. The turn compares chords along the
        whole course, across lane joins, at the signal and where it leaves the
        junction, so no single short segment decides it.

        :param str signal_id: The signal whose traffic takes the path; its
            overlap position starts the movement.
        :param Tuple[str, ...] path: Lane IDs from ``movement_paths``.
        :param Optional[BaseGeometry] area: The junction's area from
            ``_junction_areas``, or None to judge the whole course.
        :returns: Whether it yields as a left turn or U-turn, and its line
            inside the junction; a course of zero length neither yields nor
            has a line.
        :rtype: _Movement
        """
        course = self._path_line(signal_id, path)
        inside, leave = course, course.length
        if area is not None and course.length > 0:
            clipped = course.intersection(area)
            ends = [course.project(Point(coords)) for part in getattr(clipped, "geoms", [clipped])
                    for coords in getattr(part, "coords", ())]
            if ends and max(ends) > 0:  # Otherwise it never enters the polygon: judge it whole.
                inside, leave = clipped, max(ends)
        if leave == 0:
            return _Movement(False, None)
        reach = min(DIRECTION_CHORD, leave)
        turn = degrees(normalize_angle(chord_heading(course, leave - reach, leave)
                                       - chord_heading(course, 0.0, min(DIRECTION_CHORD, course.length))))
        return _Movement(turn > LEFT_TURN or turn < -OPPOSING_APPROACH, inside)

    def get_signals_wrt(self, signal_id: str) -> List[Tuple[str, str]]:
        """
        Return the signals related to a signal in ``signal_graph``.

        :param str signal_id: Signal ID.
        :returns: (other signal ID, ``"EQ"`` or ``"NE"``) pairs.
        :rtype: List[Tuple[str, str]]
        :raises KeyError: If the map has no such signal.
        """
        if signal_id not in self._service.signal_table:
            raise KeyError(signal_id)
        return [(other, data["relation"])
                for other, data in self.signal_graph[signal_id].items()]
