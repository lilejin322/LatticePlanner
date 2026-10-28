"""
Re-implementation the map service using shapely STRtree for retrival.
Although apollo's source code using KDTree for searching, but the python version
has performance issue, so a better way is to use the STRtree instead.

Author: Lejin Li, Kyushu University & National Institute of Informatics, Japan
Contact: li.lejin.147@s.kyushu-u.ac.jp
"""

import networkx as nx
from functools import cached_property
from math import isfinite
from pathlib import Path
from threading import Lock
from typing import Dict, List, NamedTuple, Optional, Tuple
from google.protobuf.message import Message
from shapely import STRtree
from shapely.geometry import LineString, Point
from shapely.geometry.base import BaseGeometry

from .geometry import (
    CenterlineProfile, SpatialIndex, curve_to_line, curves_to_lines, polygon_to_shape,
    split_boundary,
)
from .relations import MapRelations, forward_links
from .types import LaneMarking, LaneSignalOverlap, LaneStopLine, PositionEstimate
from ..message.descriptor import DescriptorBundle
from ..utils import require_nonnegative


class _Snapshot(NamedTuple):
    """
    One map file's shared data: its tables, and the indexed view once built.
    """
    tables: "MapService"
    """Shared decoded tables and lazy relations, without spatial indexes or a routing graph."""
    indexed: Optional["MapService"] = None
    """Reusable view with spatial indexes and a routing graph; None until a full load succeeds."""

_snapshots: Dict[Tuple[type, Path, Path], _Snapshot] = {}
"""File snapshots by service class, resolved map path, and descriptor path."""
_snapshots_lock = Lock()
"""Serializes loads and evictions. Cache hits read ``_snapshots`` without it:
an entry is only ever replaced by a complete, more indexed one."""


def _resolve(path: str | Path) -> Path:
    """
    Resolve path util function

    :param str | Path: Path to resolve. May be provided as a string or ``Path`` object.
    :returns: The expanded and resolved absolute path
    :rtype: Path
    """
    return Path(path).expanduser().resolve()


def _read_map(path: Path, descriptor_path: Path) -> Message:
    """
    Decode a binary Apollo map file using the given protobuf descriptor.

    The map file is read directly from disk and parsed as an
    ``apollo.hdmap.Map`` message without caching.

    :param Path path: Path to the binary Apollo map file
    :param Path: descriptor_path: Path to the protobuf descriptor set used to
        resolve the ``apollo.hdmap.Map`` message type
    :returns: The decoded Apollo HD map protobuf message
    :rtype: Message
    """
    return DescriptorBundle(descriptor_path).message("apollo.hdmap.Map").FromString(path.read_bytes())


def _validate_geometry(geometry: BaseGeometry, name: str = "geometry",
                       kind: type = BaseGeometry) -> None:
    """
    Validate that the given value is a non-empty Shapely geometry with finite coordinates.

    :param BaseGeometry geometry: Geometry to validate.
    :param str name: The argument's name, for error messages.
    :param type kind: The Shapely type required, such as ``Point``.
    :raises TypeError: If ``geometry`` is not of that type.
    :raises ValueError: If ``geometry`` is empty or has non-finite coordinates.
    """
    if not isinstance(geometry, kind):
        raise TypeError(f"{name} must be a shapely {kind.__name__}, not {type(geometry).__name__}")
    if geometry.is_empty:
        raise ValueError(f"{name} must not be empty")
    if not all(isfinite(bound) for bound in geometry.bounds):
        raise ValueError(f"{name} must have finite coordinates")


def is_allowed_to_cross(boundary: Message) -> bool:
    """
    Determine whether a lane boundary may be crossed based on its marking types.

    A boundary is considered crossable only when it has at least one boundary
    marking and every marking is dotted yellow or dotted white. Missing,
    empty, or unsupported boundary markings are treated as non-crossable.

    :param Message boundary: Apollo lane boundary protobuf message to inspect.
    :returns: ``True`` if the boundary may be crossed, otherwise ``False``.
    :rtype: bool
    """
    # Missing markings do not establish permission to cross the boundary.
    if not boundary.boundary_type:
        return False
    for boundary_type in boundary.boundary_type:
        # DOTTED_YELLOW = 1
        # DOTTED_WHITE  = 2
        if not boundary_type.types or boundary_type.types[0] not in [1, 2]:
            return False

    return True


class MapService:
    """
    Own map objects, spatial indexes, and topology for one loaded map.

    File loads share a cached snapshot keyed by map and descriptor paths.
    Each service can switch maps independently. Treat shared protobuf objects,
    tables, graphs, and derived relations as read-only. Pass ``load_only=True``
    to access tables without spatial indexes or routing graphs.

    :param str | Path descriptor_path: Path to the protobuf descriptor set used to decode map messages.
    :param str | Path | None map_path: Binary map file to load immediately; None leaves the service empty.
    :param bool load_only: If True, load only object tables without building spatial indexes or the routing graph.
    """
    descriptor_path: Path
    """Resolved path to the protobuf descriptor set used to decode map objects."""
    map_path: Optional[Path]
    """Resolved map file path; None before loading or after loading from a protobuf message."""
    lane_table: Dict[str, Message]
    """Lane protobuf messages keyed by lane ID."""
    junction_table: Dict[str, Message]
    """Junction protobuf messages keyed by junction ID."""
    crosswalk_table: Dict[str, Message]
    """Crosswalk protobuf messages keyed by crosswalk ID."""
    signal_table: Dict[str, Message]
    """Traffic signal protobuf messages keyed by signal ID."""
    stop_sign_table: Dict[str, Message]
    """Stop sign protobuf messages keyed by stop sign ID."""
    overlap_table: Dict[str, Message]
    """Overlap protobuf messages keyed by overlap ID."""
    routing_graph: Optional[nx.DiGraph]
    """Directed lane topology with F/L/R direction_type edges; None until the graph is built."""

    FLAGS_min_length_for_lane_change: float = 1.0
    """Minimum source lane length in meters required to add lane-change edges."""

    def __init__(
        self, descriptor_path: str | Path, *,
        map_path: str | Path | None = None,
        load_only: bool = False) -> None:
        """
        Constructor

        :param str | Path descriptor_path: Path to the protobuf descriptor set used to decode map messages.
        :param str | Path | None map_path: Binary map file to load immediately; None leaves the service empty.
        :param bool load_only: If True, load only object tables without building spatial indexes or the routing graph.
        """
        self.descriptor_path = DescriptorBundle(descriptor_path).descriptor_path
        self._reset()
        if map_path is not None:
            self.load_map_from_file(map_path, load_only=load_only)

    def _reset(self) -> None:
        """
        Clear this service's map path, object tables, indexes, graph, and derived caches.
        """
        self.__dict__.pop("relations", None)
        self.map_path: str | Path | None = None
        self.lane_table: Dict[str, Message] = dict()
        self.junction_table: Dict[str, Message] = dict()
        self.crosswalk_table: Dict[str, Message] = dict()
        self.signal_table: Dict[str, Message] = dict()
        self.stop_sign_table: Dict[str, Message] = dict()
        self.overlap_table: Dict[str, Message] = dict()

        # Lane, junction, crosswalk, signal, and stop-sign indexes by kind; None
        # until build_spatial_indexes().
        self._indexes: Optional[Dict[str, SpatialIndex]] = None

        # None until build_routing_graph(), as the indexes are for spatial queries.
        self.routing_graph: Optional[nx.DiGraph] = None
        self._centerlines: Dict[str, LineString] = dict()
        self._centerline_profiles: Dict[str, CenterlineProfile] = dict()

    def load_map_from_file(self, filename: str | Path, load_only: bool = False) -> None:
        """
        Reuse a file snapshot; a failed load leaves this service unchanged.

        :param str | Path filename: Path to the binary Apollo map file to load using this service's descriptor set.
        :param bool load_only: If True, expose only object tables without spatial indexes or a routing graph.
        """
        self._adopt_map(self._snapshot(_resolve(filename), load_only))

    def _snapshot(self, path: Path, load_only: bool) -> "MapService":
        """
        Return the shared snapshot of a map file, reading it on first use.

        Path aliases share one snapshot, and the file is read at most once until
        ``clear_cache``. A load is published only when every step succeeded, so
        a failed first load caches nothing and the next caller reads again.

        :param Path path: Resolved absolute path to the binary Apollo map file.
        :param bool load_only: If True, return the tables-only view; otherwise return the indexed view.
        :returns: Shared map service snapshot with the requested level of initialization.
        :rtype: MapService
        """
        key = (type(self), path, self.descriptor_path)
        cached = _snapshots.get(key)
        if cached is None or (not load_only and cached.indexed is None):
            with _snapshots_lock:
                cached = _snapshots.get(key)
                if cached is None:
                    tables = type(self)(self.descriptor_path)
                    tables._populate(_read_map(path, self.descriptor_path), load_only=True)
                    tables.map_path = path
                    # Bind relations to the snapshot, never to a caller that can switch maps.
                    _ = tables.relations
                    cached = _Snapshot(tables)
                if not load_only and cached.indexed is None:
                    indexed = type(self)(self.descriptor_path)
                    indexed._adopt_map(cached.tables)
                    indexed.build_spatial_indexes()
                    indexed.build_routing_graph()
                    cached = cached._replace(indexed=indexed)
                _snapshots[key] = cached
        return cached.tables if load_only else cached.indexed

    def _adopt_map(self, snapshot: "MapService") -> None:
        """
        Adopt the snapshot's map state by sharing its objects and derived data.

        :param MapService snapshot: Prepared service whose map state replaces this service's current state.
        """
        # Drop relations bound to the previous map if the snapshot has none yet.
        self.__dict__.pop("relations", None)
        self.__dict__.update(snapshot.__dict__)

    @classmethod
    def clear_cache(
        cls, map_path: str | Path | None = None, *,
        descriptor_path: str | Path | None = None) -> None:
        """
        Evict matching file snapshots without changing existing services.

        With no filters, evict all snapshots for this class. An eviction waits
        for loads in progress, so nothing read before it stays cached. Reload a
        service afterwards to read changed files; descriptor bundles retain
        their own independent immutable-file cache.

        :param str | Path | None map_path: Restrict eviction to this resolved map file path; None matches all map paths.
        :param str | Path | None descriptor_path: Restrict eviction to this resolved descriptor path; None matches all descriptor paths.
        """
        path = None if map_path is None else _resolve(map_path)
        schema = None if descriptor_path is None else _resolve(descriptor_path)
        with _snapshots_lock:
            for key in [key for key in _snapshots if key[0] is cls
                        and (path is None or key[1] == path) and (schema is None or key[2] == schema)]:
                del _snapshots[key]

    def load_map_from_proto(self, map: Message, load_only: bool = False) -> None:
        """
        Replace this service's map; a failed load leaves it unchanged.

        :param Message map: Apollo HD map message created using this service's descriptor set.
        :param bool load_only: If True, load only object tables without building spatial indexes or the routing graph.
        :raises TypeError: If map is not an apollo.hdmap.Map instance from this service's descriptor set.
        """
        expected = DescriptorBundle(self.descriptor_path).message("apollo.hdmap.Map")
        if not isinstance(map, expected):
            raise TypeError("map must use the service's descriptor_path schema")
        staged = type(self)(self.descriptor_path)
        staged._populate(map, load_only)
        # Retained relations must keep this map even when the caller reloads.
        _ = staged.relations
        self._adopt_map(staged)

    def _populate(self, map: Message, load_only: bool) -> None:
        """
        Populate object tables by ID and optionally build the spatial indexes and routing graph.

        :param Message map: Decoded Apollo HD map message whose objects populate this service's tables.
        :param bool load_only: If True, populate only object tables; otherwise also build the spatial indexes and routing graph.
        """
        obj_table_map = [
            (map.lane, self.lane_table),
            (map.junction, self.junction_table),
            (map.crosswalk, self.crosswalk_table),
            (map.signal, self.signal_table),
            (map.stop_sign, self.stop_sign_table),
            (map.overlap, self.overlap_table),
        ]
        for objects, table_name in obj_table_map:
            for obj in objects:
                table_name[obj.id.id] = obj

        if not load_only:
            self.build_spatial_indexes()
            self.build_routing_graph()

    def get_lanes(self, point: Optional[Point] = None, distance=0.0) -> List[str]:
        """
        Get lane IDs: all lanes in map order when ``distance`` is 0 (``point`` is
        then ignored), otherwise the lanes within ``distance`` metres of ``point``,
        nearest first and equally near ones by ID.

        :param Optional[Point] point: Query position in map coordinates; required
            when ``distance`` is positive.
        :param float distance: Search radius in metres from ``point`` to each lane's
            centerline; 0 lists every lane.
        :returns: Lane IDs, in map order or nearest first.
        :rtype: List[str]
        :raises ValueError: If ``distance`` is negative or not finite, or ``point``
            has non-finite coordinates.
        :raises TypeError: If ``distance`` is positive and ``point`` is not a Point.
        :raises RuntimeError: If ``distance`` is positive and the map was loaded
            with ``load_only=True``, which builds no spatial indexes.
        """
        return self._query("lane", self.lane_table, point, distance)

    def get_junctions(self, point: Optional[Point] = None, distance=0.0) -> List[str]:
        """
        Get junction IDs, as ``get_lanes`` does for lanes: all junctions in map
        order, or those whose polygon lies within ``distance`` metres of
        ``point``, nearest first. A junction containing ``point`` is at distance 0.

        :param Optional[Point] point: Query position; required when ``distance`` is positive.
        :param float distance: Search radius in metres; 0 lists every junction.
        :returns: Junction IDs, in map order or nearest first.
        :rtype: List[str]
        :raises ValueError, TypeError, RuntimeError: As ``get_lanes``.
        """
        return self._query("junction", self.junction_table, point, distance)

    def get_crosswalks(self, point: Optional[Point] = None, distance=0.0) -> List[str]:
        """
        Get crosswalk IDs: all in map order, or those whose polygon lies within
        ``distance`` metres of ``point``, nearest first, as ``get_junctions``.

        :param Optional[Point] point: Query position; required when ``distance`` is positive.
        :param float distance: Search radius in metres; 0 lists every crosswalk.
        :returns: Crosswalk IDs, in map order or nearest first.
        :rtype: List[str]
        :raises ValueError, TypeError, RuntimeError: As ``get_lanes``.
        """
        return self._query("crosswalk", self.crosswalk_table, point, distance)

    def get_signals(self, point: Optional[Point] = None, distance=0.0) -> List[str]:
        """
        Get signal IDs: all in map order, or those with a stop line within
        ``distance`` metres of ``point``, nearest first. As in Apollo, signals are
        located by their stop lines; a signal without one is never found nearby.

        :param Optional[Point] point: Query position; required when ``distance`` is positive.
        :param float distance: Search radius in metres; 0 lists every signal.
        :returns: Signal IDs, in map order or nearest first.
        :rtype: List[str]
        :raises ValueError, TypeError, RuntimeError: As ``get_lanes``.
        """
        return self._query("signal", self.signal_table, point, distance)

    def get_stop_signs(self, point: Optional[Point] = None, distance=0.0) -> List[str]:
        """
        Get stop-sign IDs: all in map order, or those with a stop line within
        ``distance`` metres of ``point``, nearest first, as ``get_signals``.

        :param Optional[Point] point: Query position; required when ``distance`` is positive.
        :param float distance: Search radius in metres; 0 lists every stop sign.
        :returns: Stop-sign IDs, in map order or nearest first.
        :rtype: List[str]
        :raises ValueError, TypeError, RuntimeError: As ``get_lanes``.
        """
        return self._query("stop sign", self.stop_sign_table, point, distance)

    def _query(self, kind: str, table: Dict[str, Message], point: Optional[Point],
               distance: float) -> List[str]:
        """
        All IDs of ``table`` in map order, or those of the ``kind`` index near ``point``.

        Shared by ``get_lanes``, ``get_junctions``, ``get_crosswalks``,
        ``get_signals``, and ``get_stop_signs``.

        :param str kind: Index to search: ``"lane"``, ``"junction"``,
            ``"crosswalk"``, ``"signal"``, or ``"stop sign"``.
        :param Dict[str, Message] table: Objects of that kind by ID, listed
            when ``distance`` is 0.
        :param Optional[Point] point: Query position in map coordinates; ignored
            when ``distance`` is 0.
        :param float distance: Search radius in metres; 0 lists every ID.
        :returns: IDs in map order, or within ``distance`` nearest first,
            equally near ones by ID.
        :rtype: List[str]
        :raises ValueError: If ``distance`` is negative or not finite, or ``point``
            has non-finite coordinates.
        :raises TypeError: If ``distance`` is positive and ``point`` is not a Point.
        :raises RuntimeError: If ``distance`` is positive and the map was loaded
            with ``load_only=True``, which builds no spatial indexes.
        """
        require_nonnegative(distance=distance)
        if distance == 0.0:
            return list(table)
        _validate_geometry(point, "point", Point)
        return self._index(kind).within(point, distance)

    def get_lane_markings(self, geometry: BaseGeometry, distance: float = 0.0) -> List[LaneMarking]:
        """
        Get the lane marking pieces near a geometry, such as a vehicle footprint.

        Every lane boundary is split where its marking changes. A footprint
        overlapping a marking whose ``prohibits_crossing`` is true crosses a
        line the vehicle may not cross. Unlike ``get_lanes``, ``distance=0``
        means intersecting, not everything.

        :param BaseGeometry geometry: Query geometry in map coordinates.
        :param float distance: Search radius in metres; 0 finds markings that
            intersect or touch ``geometry``.
        :returns: Markings nearest first; equally near ones by lane, side, and s.
        :rtype: List[LaneMarking]
        :raises ValueError: If ``distance`` is negative or not finite, or
            ``geometry`` is empty or has non-finite coordinates.
        :raises TypeError: If ``geometry`` is not a Shapely geometry.
        :raises RuntimeError: If the map was loaded with ``load_only=True``,
            which builds no spatial indexes.
        """
        require_nonnegative(distance=distance)
        _validate_geometry(geometry)
        return self._index("lane marking").within(geometry, distance)

    def get_lane_by_id(self, lane_id: str) -> Message:
        """
        Get a lane's protobuf message.

        The message is shared with every service using the same cached map;
        treat it as read-only.

        :param str lane_id: Lane ID.
        :returns: The ``apollo.hdmap.Lane`` message.
        :rtype: Message
        :raises KeyError: If the map has no such lane.
        """
        return self.lane_table[lane_id]

    def get_lane_central_curve_by_id(self, lane_id: str) -> LineString:
        """
        Get a lane's complete centerline as a Shapely line.

        Every line segment of ``central_curve`` is joined in order, omitting
        repeated endpoints. Its length is the geometric arc length used by ``s``
        in ``get_position`` and ``get_lane_coord_and_heading``, which may differ
        from ``Lane.length``. Each line is built once per loaded map.

        :param str lane_id: Lane ID.
        :returns: The centerline in map coordinates.
        :rtype: LineString
        :raises KeyError: If the map has no such lane.
        :raises ValueError: If the centerline has fewer than two distinct points.
        """
        # Built once per loaded map. Shapely geometries are immutable, so
        # handles sharing a cached snapshot may also share these lines.
        line = self._centerlines.get(lane_id)
        if line is None:
            line = curve_to_line(self.get_lane_by_id(lane_id).central_curve)
            self._centerlines[lane_id] = line
        return line

    def get_junction_by_id(self, junction_id: str) -> Message:
        """
        Get a junction's protobuf message.

        The message is shared with every service using the same cached map;
        treat it as read-only.

        :param str junction_id: Junction ID.
        :returns: The ``apollo.hdmap.Junction`` message.
        :rtype: Message
        :raises KeyError: If the map has no such junction.
        """
        return self.junction_table[junction_id]

    def get_crosswalk_by_id(self, crosswalk_id: str) -> Message:
        """
        Get a crosswalk's protobuf message.

        The message is shared with every service using the same cached map;
        treat it as read-only.

        :param str crosswalk_id: Crosswalk ID.
        :returns: The ``apollo.hdmap.Crosswalk`` message.
        :rtype: Message
        :raises KeyError: If the map has no such crosswalk.
        """
        return self.crosswalk_table[crosswalk_id]

    def get_signal_by_id(self, signal_id: str) -> Message:
        """
        Get a signal's protobuf message.

        The message is shared with every service using the same cached map;
        treat it as read-only.

        :param str signal_id: Signal ID.
        :returns: The ``apollo.hdmap.Signal`` message.
        :rtype: Message
        :raises KeyError: If the map has no such signal.
        """
        return self.signal_table[signal_id]

    def get_stop_sign_by_id(self, stop_sign_id: str) -> Message:
        """
        Get a stop sign's protobuf message.

        The message is shared with every service using the same cached map;
        treat it as read-only.

        :param str stop_sign_id: Stop sign ID.
        :returns: The ``apollo.hdmap.StopSign`` message.
        :rtype: Message
        :raises KeyError: If the map has no such stop sign.
        """
        return self.stop_sign_table[stop_sign_id]

    def get_overlap_by_id(self, overlap_id: str) -> Message:
        """
        Get a overlap's protobuf message.

        The message is shared with every service using the same cached map;
        treat it as read-only.

        :param str overlap_id: Overlap ID.
        :returns: The ``apollo.hdmap.Overlap`` message.
        :rtype: Message
        :raises KeyError: If the map has no such overlap.
        """
        return self.overlap_table[overlap_id]

    def build_routing_graph(self) -> None:
        """
        Build ``routing_graph``, the lanes' explicit topology, after Apollo's
        ``modules/routing/topo_creator/graph_creator.cc``.

        Every lane is a node. A forward (``F``) edge joins A to B when A lists
        B as a successor or B lists A as a predecessor. A lane at least
        ``FLAGS_min_length_for_lane_change`` long gets a left (``L``) or right
        (``R``) edge to each forward neighbour on that side whose boundary
        ``is_allowed_to_cross``. IDs missing from the lane table are skipped,
        as in ``GraphCreator::AddEdge``. Edges carry only ``direction_type``.

        Unlike Apollo's graph, this one keeps every lane type and every U-turn,
        and also follows predecessor links; ``RouteSpace`` filters lanes and
        adds costs for each agent's rules.
        """
        self.routing_graph = nx.DiGraph()
        self.routing_graph.add_nodes_from(self.lane_table)

        def known(ids) -> List[str]:
            # Skip unknown IDs instead of adding nodes, as in GraphCreator::AddEdge.
            return [x.id for x in ids if x.id in self.lane_table]

        for lane_id, lane in self.lane_table.items():
            self.routing_graph.add_edges_from(forward_links(lane_id, lane, self.lane_table), direction_type='F')

            if lane.length < MapService.FLAGS_min_length_for_lane_change:
                continue

            # Lane changes cross the source lane's own boundary.
            if lane.HasField('left_boundary') and is_allowed_to_cross(lane.left_boundary):
                for pid in known(lane.left_neighbor_forward_lane_id):
                    self.routing_graph.add_edge(lane_id, pid, direction_type='L')
            if lane.HasField('right_boundary') and is_allowed_to_cross(lane.right_boundary):
                for pid in known(lane.right_neighbor_forward_lane_id):
                    self.routing_graph.add_edge(lane_id, pid, direction_type='R')

    def build_spatial_indexes(self) -> None:
        """
        Build the STRtree indexes for position queries, as Apollo's HDMapImpl
        does: complete lane centerlines, junction and crosswalk polygons, and
        signal and stop-sign stop lines; and lane boundaries split into marking
        pieces, for ``get_lane_markings``.

        GEOS measures the exact distance to every candidate's whole geometry,
        and coincident lanes keep separate entries. Objects other than lanes may
        lack their geometry; they are left out, so position queries never
        return them.

        :raises ValueError: If a lane centerline or boundary curve, polygon, or
            stop line is malformed.
        """
        self._indexes = {
            "lane": SpatialIndex.build("lane", self.lane_table,
                                       lambda lane: self.get_lane_central_curve_by_id(lane.id.id)),
            "junction": SpatialIndex.build("junction", self.junction_table,
                                           lambda junction: polygon_to_shape(junction.polygon)),
            "crosswalk": SpatialIndex.build("crosswalk", self.crosswalk_table,
                                            lambda crosswalk: polygon_to_shape(crosswalk.polygon)),
            "signal": SpatialIndex.build("signal", self.signal_table,
                                         lambda signal: curves_to_lines(signal.stop_line)),
            "stop sign": SpatialIndex.build("stop sign", self.stop_sign_table,
                                            lambda stop_sign: curves_to_lines(stop_sign.stop_line)),
        }
        markings = self._lane_markings()
        self._indexes["lane marking"] = SpatialIndex(markings, STRtree([m.geometry for m in markings]))

    def _lane_markings(self) -> List[LaneMarking]:
        """
        Split every lane boundary into marking pieces, in lane table order.

        :returns: The list of LaneMarking
        :rtype: List[LaneMarking]
        :raises ValueError: Naming the lane whose boundary curve is malformed.
        """
        names = DescriptorBundle(self.descriptor_path).message(
            "apollo.hdmap.LaneBoundaryType").DESCRIPTOR.enum_types_by_name["Type"].values_by_number
        markings = []
        for lane_id, lane in self.lane_table.items():
            for side, boundary in (("left", lane.left_boundary), ("right", lane.right_boundary)):
                try:
                    pieces = split_boundary(boundary)
                except ValueError as error:
                    raise ValueError(f"invalid {side} boundary of lane {lane_id!r}: {error}") from error
                markings.extend(
                    LaneMarking(lane_id, side, start, end,
                                tuple(names[t].name if t in names else str(t) for t in piece.types),
                                boundary.virtual, line)
                    for start, end, piece, line in pieces)
        return markings

    @property
    def lane_index(self) -> Optional[STRtree]:
        """
        The spatial index of complete lane centerlines.

        :returns: The STRtree, or None if the map was loaded with ``load_only=True``.
        :rtype: Optional[STRtree]
        """
        return None if self._indexes is None else self._indexes["lane"].tree

    def get_lane_coord_and_heading(self, lane_id: str,
                                   s: float) -> Tuple[Point, float]:
        """
        Return the centerline point and heading at geometric arc length s.

        s uses the same centreline distance as get_position(), not Lane.length.
        Headings interpolate between segment midpoints; see CenterlineProfile.

        :param str lane_id: Lane ID.
        :param float s: Metres along the lane's complete centerline, within
            ``[0, centerline length]``.
        :returns: The point on the centerline in map coordinates, and the lane
            heading there in radians within ``[-pi, pi)``.
        :rtype: Tuple[Point, float]
        :raises KeyError: If the map has no such lane.
        :raises ValueError: If ``s`` is outside the centerline or not finite.
        """
        profile = self._centerline_profile(lane_id)
        length = profile.line.length
        # Negative s would make Shapely interpolate backwards from the end.
        if not 0 <= s <= length:
            raise ValueError(f"s={s} is outside lane {lane_id!r} [0, {length}]")

        ip = profile.line.interpolate(s)
        return Point(ip.x, ip.y), profile.heading_at(s)

    def _centerline_profile(self, lane_id: str) -> CenterlineProfile:
        """
        Return a lane's centerline profile for heading queries, built once per loaded map.

        :param str lane_id: Lane ID.
        :returns: Cumulative s and midpoint-interpolated headings of the
            lane's complete centerline.
        :rtype: CenterlineProfile
        :raises KeyError: If the map has no such lane.
        :raises ValueError: If the centerline has fewer than two distinct points.
        """
        # Derived from immutable lane geometry, so handles sharing a cached
        # snapshot may also share these lazily built profiles.
        profile = self._centerline_profiles.get(lane_id)
        if profile is None:
            profile = CenterlineProfile(self.get_lane_central_curve_by_id(lane_id))
            self._centerline_profiles[lane_id] = profile
        return profile

    def get_nearest_lane(self, point: Point) -> str:
        """
        Get the lane whose complete centerline is nearest to a point.

        Lanes of every type are considered. Equally near lanes, such as
        coincident ones, resolve to the earliest in map order.

        :param Point point: Query position in map coordinates.
        :returns: The nearest lane ID.
        :rtype: str
        :raises TypeError: If ``point`` is not a Shapely Point.
        :raises ValueError: If ``point`` is empty or has non-finite coordinates,
            or the map has no lanes.
        :raises RuntimeError: If the map was loaded with ``load_only=True``,
            which builds no spatial indexes.
        """
        _validate_geometry(point, "point", Point)
        lane_id = self._index("lane").nearest(point)
        if lane_id is None:
            raise ValueError("Map has no lanes")
        return lane_id

    def _index(self, kind: str) -> SpatialIndex:
        """
        Return one of the indexes built by ``build_spatial_indexes``.

        :param str kind: ``"lane"``, ``"junction"``, ``"crosswalk"``,
            ``"signal"``, ``"stop sign"``, or ``"lane marking"``.
        :returns: The index of that kind.
        :rtype: SpatialIndex
        :raises RuntimeError: If the map was loaded with ``load_only=True``,
            which builds no spatial indexes.
        """
        if self._indexes is None:
            raise RuntimeError("Spatial indexes are unavailable; load with load_only=False")
        return self._indexes[kind]

    def get_position(self, point: Point) -> PositionEstimate:
        """
        Project a point onto the nearest lane's complete centerline.

        The lane is chosen as by ``get_nearest_lane``, among lanes of every
        type. ``s`` is the unrounded geometric distance in metres along that
        centerline to the closest point, clamped to its ends, so it can be
        passed to ``get_lane_coord_and_heading``.

        :param Point point: Query position in map coordinates.
        :returns: The lane ID and ``s``; both None if the map has no lanes.
        :rtype: PositionEstimate
        :raises TypeError: If ``point`` is not a Shapely Point.
        :raises ValueError: If ``point`` is empty or has non-finite coordinates.
        :raises RuntimeError: If the map was loaded with ``load_only=True``,
            which builds no spatial indexes.
        """
        _validate_geometry(point, "point", Point)
        self._index("lane")
        if not self.lane_table:
            return PositionEstimate()
        lane_id = self.get_nearest_lane(point)
        return PositionEstimate(lane_id, self.get_lane_central_curve_by_id(lane_id).project(point))

    @cached_property
    def relations(self) -> MapRelations:
        """
        Lazy overlap and signal relationships for this map.

        Covers lanes overlapping each junction, lane-signal overlaps and
        their intervals, signals controlling each lane, signal stop lines
        matched to lanes, and the signal EQ/NE graph. Each relationship is
        computed on first access and needs no spatial index. Handles loaded
        from the same file share one instance; loading another map replaces it.

        :returns: The relationships of the currently loaded map.
        :rtype: MapRelations
        """
        return MapRelations(self)

    def get_signals_wrt(self, signal_id: str) -> List[Tuple[str, str]]:
        """
        Get the signals related to a signal at the same junction, with their relation.

        ``"EQ"`` signals show the same colour; ``"NE"`` signals are never green
        together. As at a junction with circular lights, signals whose
        controlled lanes enter from the same or the opposing approach are EQ,
        left turns and U-turns yielding to oncoming traffic, and signals of
        crossing approaches are NE. Opposing signals are NE instead if their
        movements cross where neither turns left, as at a skewed junction.
        Apollo maps carry no signal phases, so this is inferred from lane
        geometry at signal overlaps, not read; signals controlling no lanes
        are unrelated. Missing or invalid overlap positions leave approaches
        unknown. Conflicting inferred EQ preferences become NE when necessary
        to keep each synchronous group free of NE constraints.

        :param str signal_id: Signal ID.
        :returns: (other signal ID, ``"EQ"`` or ``"NE"``) pairs.
        :rtype: List[Tuple[str, str]]
        :raises KeyError: If the map has no such signal.
        """
        return self.relations.get_signals_wrt(signal_id)

    def get_lanes_controlled_by_signal(self, signal_id: str) -> List[str]:
        """
        Return lane IDs associated with the signal by explicit overlaps.

        A lane is controlled when one overlap holds both the lane and the
        signal. These are only the lanes the signal overlaps, often short entry
        pieces; ``relations.movement_paths`` follows them through a junction.

        :param str signal_id: Signal ID.
        :returns: Controlled lane IDs, sorted; empty if the signal overlaps no lane.
        :rtype: List[str]
        :raises KeyError: If the map has no such signal.
        """
        return list(self.relations.lanes_controlled_by_signal[signal_id])

    def get_signals_for_lane(self, lane_id: str) -> List[str]:
        """
        Return signal IDs associated with the lane by explicit overlaps.

        A signal controls the lane when one overlap holds both. Lanes after a
        controlled entry piece are not associated, even though its signal
        governs the traffic on them.

        :param str lane_id: Lane ID.
        :returns: Signal IDs, sorted; empty if no signal overlaps the lane.
        :rtype: List[str]
        :raises KeyError: If the map has no such lane.
        """
        return list(self.relations.signals_for_lane[lane_id])

    def get_signal_lane_overlaps(self, signal_id: str) -> Tuple[LaneSignalOverlap, ...]:
        """
        Return all lane intervals associated with a signal, in lane metres.

        Each record is one overlap between the signal and one lane, with the
        interval's ``start_s`` and ``end_s`` as stored in the map; missing
        values stay None. Separate overlaps on the same lane are all kept.
        These ``s`` values come from the map, not from the centerline geometry
        used by ``get_position``, so they may differ slightly.

        :param str signal_id: Signal ID.
        :returns: Records ordered by lane ID, then overlap ID and interval;
            empty if the signal overlaps no lane.
        :rtype: Tuple[LaneSignalOverlap, ...]
        :raises KeyError: If the map has no such signal.
        """
        return self.relations.signal_lane_overlaps[signal_id]

    def get_lane_signal_overlaps(self, lane_id: str) -> Tuple[LaneSignalOverlap, ...]:
        """
        Return all signal intervals on a lane, retaining separate overlaps.

        The lane's own overlap references are followed; only overlaps holding
        this lane and an existing signal count. Records are as in
        ``get_signal_lane_overlaps``: ``s`` values as stored in the map, None
        where missing.

        :param str lane_id: Lane ID.
        :returns: Records ordered by overlap ID, signal ID, and interval;
            empty if no signal overlaps the lane.
        :rtype: Tuple[LaneSignalOverlap, ...]
        :raises KeyError: If the map has no such lane.
        """
        return self.relations.lane_signal_overlaps[lane_id]

    def get_signal_stop_lines(self, signal_id: str, lane_id: str, *,
                              max_distance: float = 1.0) -> Tuple[LaneStopLine, ...]:
        """
        Match a signal's stop lines to this lane's overlap intervals.

        max_distance is a geometric tolerance in metres, not a search radius
        around the vehicle. Multiple candidates are retained; no nearest-only
        fallback is used. The returned geometry is the full signal stop line.

        Apollo stores no lane ID on a stop line, so each of the signal's stop
        lines is kept when it lies within ``max_distance`` of the stretch of
        this lane's centerline covered by one of their overlap intervals.
        Intervals without both ends, or outside the centerline, are skipped;
        an interval running past the centerline's end is clipped to it.

        :param str signal_id: Signal ID.
        :param str lane_id: Lane ID.
        :param float max_distance: Largest distance in metres between a stop
            line and an overlap interval of the lane.
        :returns: One record per matching stop line, in the signal's stop-line
            order; empty if the signal and lane share no usable overlap
            interval or the signal has no stop line.
        :rtype: Tuple[LaneStopLine, ...]
        :raises KeyError: If the map has no such signal or lane.
        :raises ValueError: If ``max_distance`` is negative or not finite, or a
            stop line of the signal is malformed.
        """
        return self.relations.get_signal_stop_lines(signal_id, lane_id, max_distance)

    def get_lane_stop_lines(self, lane_id: str, *,
                            max_distance: float = 1.0) -> Tuple[LaneStopLine, ...]:
        """
        Return matching stop lines from every signal associated with a lane.

        Stop-sign stop lines are not included. Shared geometry keeps separate
        records for each signal so that signal provenance is preserved.

        Equivalent to ``get_signal_stop_lines`` for each signal of
        ``get_signals_for_lane``. Only signals overlapping this lane count, so
        a lane following a controlled entry piece usually has none.

        :param str lane_id: Lane ID.
        :param float max_distance: As in ``get_signal_stop_lines``.
        :returns: Matching stop lines, ordered by signal ID, then by each
            signal's stop-line order.
        :rtype: Tuple[LaneStopLine, ...]
        :raises KeyError: If the map has no such lane.
        :raises ValueError: If ``max_distance`` is negative or not finite, even
            when the lane has no signal, or a matched signal's stop line is
            malformed.
        """
        return self.relations.get_lane_stop_lines(lane_id, max_distance)

    def is_conflict_lanes(self, lane_ids1: List[str],
                          lane_ids2: List[str]) -> bool:
        """
        Whether any lane of one group meets a different lane of the other.

        Lanes meet when their complete centerlines intersect, including a mere
        touch where they merge or diverge at a shared end point. A lane listed
        in both groups does not conflict with itself.

        :param List[str] lane_ids1: Lane IDs of the first group.
        :param List[str] lane_ids2: Lane IDs of the second group.
        :returns: True if some pair of different lanes intersects.
        :rtype: bool
        :raises KeyError: If either group names an unknown lane.
        """
        return self.relations.is_conflict_lanes(lane_ids1, lane_ids2)
