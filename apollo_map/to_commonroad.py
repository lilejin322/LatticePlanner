"""Convert a bundled Apollo HD map into a CommonRoad scenario.

The lanelet construction follows EVITA's ``map_converter.py``
(https://github.com/fabiocavaleri/EVITA). Maps are read with this repo's
descriptor-based loader.
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx
import numpy as np
from shapely.geometry import LineString, Point

from commonroad.common.common_lanelet import LineMarking, StopLine
from commonroad.common.common_scenario import FileInformation
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.util import FileFormat
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LaneletType
from commonroad.scenario.scenario import Scenario, ScenarioID, Tag
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
    TrafficLightDirection,
    TrafficLightState,
)
from commonroad.scenario.traffic_sign import TrafficSign, TrafficSignElement, TrafficSignIDGermany

from .map.map_service import MapService

APOLLO_TO_CR_TYPE = {
    "CITY_DRIVING": LaneletType.URBAN,
    "BIKING": LaneletType.BICYCLE_LANE,
    "SIDEWALK": LaneletType.SIDEWALK,
    "PARKING": LaneletType.PARKING,
    "SHOULDER": LaneletType.SHOULDER,
    "NONE": None,
}

SIGNAL_DIRECTION = {
    "ARROW_STRAIGHT": TrafficLightDirection.STRAIGHT,
    "ARROW_FORWARD": TrafficLightDirection.STRAIGHT,
    "ARROW_LEFT": TrafficLightDirection.LEFT,
    "ARROW_LEFT_STRAIGHT": TrafficLightDirection.LEFT_STRAIGHT,
    "ARROW_LEFT_AND_FORWARD": TrafficLightDirection.LEFT_STRAIGHT,
    "ARROW_RIGHT": TrafficLightDirection.RIGHT,
    "ARROW_STRAIGHT_RIGHT": TrafficLightDirection.STRAIGHT_RIGHT,
    "ARROW_RIGHT_AND_FORWARD": TrafficLightDirection.STRAIGHT_RIGHT,
    "ARROW_LEFT_RIGHT": TrafficLightDirection.LEFT_RIGHT,
}


def enum_name(message, field: str) -> str:
    """Name of a protobuf enum field on a dynamically built message."""
    value = getattr(message, field)
    enum_type = message.DESCRIPTOR.fields_by_name[field].enum_type
    if enum_type is None:
        return str(value)
    found = enum_type.values_by_number.get(int(value))
    return found.name if found is not None else str(value)


def curve_to_points(curve) -> List[Tuple[float, float]]:
    """Polyline vertices from an Apollo curve, with the EVITA 3-point densify."""
    points: List[Tuple[float, float]] = []
    for segment in curve.segment:
        if segment.HasField("line_segment"):
            points.extend((point.x, point.y) for point in segment.line_segment.point)
    if len(points) == 3:
        p0, p1, p2 = points
        mid01 = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        mid12 = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
        points = [p0, mid01, p1, mid12, p2]
    if len(points) == 1:
        points = [points[0], points[0]]
    return points


def _ids(id_list, lane_id_map: Dict[str, int]) -> Optional[List[int]]:
    if not id_list:
        return None
    converted = [lane_id_map[item.id] for item in id_list if item.id in lane_id_map]
    return converted or None


def _adjacent(forward: Optional[Sequence[int]], reverse: Optional[Sequence[int]]) -> Tuple[Optional[int], bool]:
    if forward:
        return forward[0], True
    if reverse:
        return reverse[0], False
    return None, False


def _numeric_id(raw_id: str, offset: int) -> int:
    if "_" in raw_id:
        return int(raw_id.split("_")[-1]) + offset
    return int(raw_id) + offset


class ApolloParsedMap:
    """Lane, signal, and junction tables the EVITA converter reads off MapParser."""

    def __init__(self, service: MapService) -> None:
        self.lanes = dict(service.lane_table)
        self.signals = dict(service.signal_table)
        self.stop_signs = dict(service.stop_sign_table)
        self.junctions = dict(service.junction_table)
        self._signals_at_junction: Dict[str, List[str]] = defaultdict(list)
        self._lanes_at_junction: Dict[str, List[str]] = defaultdict(list)
        self._lanes_controlled_by_signal: Dict[str, List[str]] = defaultdict(list)
        self._signal_relations = nx.Graph()
        self._lane_nx = nx.DiGraph()
        self._parse_relations()
        self._parse_signal_relations()
        self._parse_lane_relations()

    def get_lanes(self) -> List[str]:
        return list(self.lanes)

    def get_lane_by_id(self, lane_id: str):
        return self.lanes[lane_id]

    def get_signals(self) -> List[str]:
        return list(self.signals)

    def get_signal_by_id(self, signal_id: str):
        return self.signals[signal_id]

    def get_stop_signs(self) -> List[str]:
        return list(self.stop_signs)

    def get_stop_sign_by_id(self, stop_sign_id: str):
        return self.stop_signs[stop_sign_id]

    def get_signals_wrt(self, signal_id: str) -> List[Tuple[str, str]]:
        result = []
        for _left, right, data in self._signal_relations.edges(signal_id, data=True):
            result.append((right, data["v"]))
        return result

    def get_lanes_not_in_junction(self) -> set:
        lanes = set(self.lanes)
        for junction_id in self._lanes_at_junction:
            lanes -= set(self._lanes_at_junction[junction_id])
        return lanes

    def reachable_from(self, lane_id: str, depth: int = 5) -> List[List[str]]:
        if depth == 1:
            return [[lane_id]]
        result: List[List[str]] = []
        for _left, right in self._lane_nx.edges(lane_id):
            result.append([lane_id, right])
            for path in self.reachable_from(right, depth - 1):
                result.append([lane_id] + path)
        return result

    def central_curve(self, lane_id: str) -> LineString:
        lane = self.lanes[lane_id]
        points = lane.central_curve.segment[0].line_segment.point
        return LineString([(point.x, point.y) for point in points])

    def _overlaps(self, left, right) -> bool:
        left_ids = {item.id for item in left.overlap_id}
        right_ids = {item.id for item in right.overlap_id}
        return bool(left_ids & right_ids)

    def _conflict_lanes(self, left_ids: Iterable[str], right_ids: Iterable[str]) -> bool:
        for left_id in left_ids:
            for right_id in right_ids:
                if left_id == right_id:
                    continue
                if self.central_curve(left_id).intersects(self.central_curve(right_id)):
                    return True
        return False

    def _parse_relations(self) -> None:
        for signal_id, signal in self.signals.items():
            for junction_id, junction in self.junctions.items():
                if self._overlaps(signal, junction):
                    self._signals_at_junction[junction_id].append(signal_id)
        for lane_id, lane in self.lanes.items():
            for junction_id, junction in self.junctions.items():
                if self._overlaps(lane, junction):
                    self._lanes_at_junction[junction_id].append(lane_id)
        for junction_id in self.junctions:
            for signal_id in self._signals_at_junction[junction_id]:
                for lane_id in self._lanes_at_junction[junction_id]:
                    if self._overlaps(self.signals[signal_id], self.lanes[lane_id]):
                        self._lanes_controlled_by_signal[signal_id].append(lane_id)

    def _parse_signal_relations(self) -> None:
        graph = nx.Graph()
        for junction_id in self.junctions:
            signal_ids = self._signals_at_junction[junction_id]
            for left_id in signal_ids:
                graph.add_node(left_id)
                for right_id in signal_ids:
                    if left_id == right_id:
                        continue
                    left_lanes = self._lanes_controlled_by_signal[left_id]
                    right_lanes = self._lanes_controlled_by_signal[right_id]
                    if left_lanes == right_lanes:
                        graph.add_edge(left_id, right_id, v="EQ")
                    elif self._conflict_lanes(left_lanes, right_lanes):
                        graph.add_edge(left_id, right_id, v="NE")
        self._signal_relations = graph

    def _parse_lane_relations(self) -> None:
        graph = nx.DiGraph()
        curves = {lane_id: self.central_curve(lane_id) for lane_id in self.lanes}
        for lane_id, line in curves.items():
            graph.add_node(lane_id)
            start = Point(line.coords[0])
            end = Point(line.coords[-1])
            for other_id, other in curves.items():
                if lane_id == other_id:
                    continue
                other_start = Point(other.coords[0])
                other_end = Point(other.coords[-1])
                if start.distance(other_end) < 0.001:
                    graph.add_edge(other_id, lane_id)
                elif end.distance(other_start) < 0.001:
                    graph.add_edge(lane_id, other_id)
        self._lane_nx = graph


def _lanelets_touching_stop_line(network: LaneletNetwork, stop_line_points: np.ndarray) -> List[int]:
    if stop_line_points is None or len(stop_line_points) == 0:
        return []
    stop_line = LineString(stop_line_points)
    return [
        lanelet.lanelet_id
        for lanelet in network.lanelets
        if stop_line.intersects(LineString(lanelet.center_vertices))
    ]


def _crop_stop_line(lanelet: Lanelet, stop_line_points: np.ndarray) -> np.ndarray:
    midpoint = LineString(stop_line_points).interpolate(0.5, normalized=True)
    left_bound = LineString(lanelet.left_vertices)
    right_bound = LineString(lanelet.right_vertices)
    left = left_bound.interpolate(left_bound.project(midpoint))
    right = right_bound.interpolate(right_bound.project(midpoint))
    return np.array([np.array(left.coords[0]), np.array(right.coords[0])])


def convert_map(
    map_name_or_path,
    output_dir: str | Path,
    *,
    filename: str = "base_map.bin",
    map_id: int = 1,
) -> Path:
    """Write a CommonRoad XML plus the EVITA id and reachability JSON files.

    ``map_name_or_path`` is a bundled name such as ``borregas_ave`` or a path
    to a binary Apollo map. Returns the XML path.
    """
    from apollo_map import load_map, map_file

    source = Path(map_name_or_path)
    if source.is_file():
        service = load_map(source, load_only=True)
        map_name = source.parent.name
    else:
        service = load_map(map_name_or_path, filename=filename, load_only=True)
        map_name = str(map_name_or_path)
        source = map_file(map_name, filename)

    parsed = ApolloParsedMap(service)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario = Scenario(
        dt=0.1,
        scenario_id=ScenarioID(country_id="DEU", map_name=map_name, map_id=map_id),
        file_information=FileInformation(
            author="Fabio Cavaleri",
            affiliation="USI-AIT-NII",
            source="BaiduApollo",
        ),
        tags={Tag.URBAN},
    )
    network = LaneletNetwork()
    lane_id_map = {lane_id: index for index, lane_id in enumerate(parsed.get_lanes())}
    (output_dir / f"{map_name}_lane_ids_map.json").write_text(json.dumps(lane_id_map, indent=2))

    for lane_id in parsed.get_lanes():
        lane = parsed.get_lane_by_id(lane_id)
        center = curve_to_points(lane.central_curve)
        left = curve_to_points(lane.left_boundary.curve)
        right = curve_to_points(lane.right_boundary.curve)
        if len(center) < 2 or len(left) < 2 or len(right) < 2:
            continue
        left_adjacent, left_same = _adjacent(
            _ids(lane.left_neighbor_forward_lane_id, lane_id_map),
            _ids(lane.left_neighbor_reverse_lane_id, lane_id_map),
        )
        right_adjacent, right_same = _adjacent(
            _ids(lane.right_neighbor_forward_lane_id, lane_id_map),
            _ids(lane.right_neighbor_reverse_lane_id, lane_id_map),
        )
        lanelet = Lanelet(
            lanelet_id=lane_id_map[lane_id],
            left_vertices=np.array(left),
            right_vertices=np.array(right),
            center_vertices=np.array(center),
            predecessor=_ids(lane.predecessor_id, lane_id_map),
            successor=_ids(lane.successor_id, lane_id_map),
            adjacent_left=left_adjacent,
            adjacent_left_same_direction=left_same,
            adjacent_right=right_adjacent,
            adjacent_right_same_direction=right_same,
        )
        cr_type = APOLLO_TO_CR_TYPE.get(enum_name(lane, "type"))
        if cr_type is not None:
            lanelet.lanelet_type.add(cr_type)
        network.add_lanelet(lanelet)
    scenario.add_objects(network)

    for stop_id in parsed.get_stop_signs():
        stop_sign = parsed.get_stop_sign_by_id(stop_id)
        if not stop_sign.stop_line or not stop_sign.stop_line[0].segment:
            continue
        points = np.array(curve_to_points(stop_sign.stop_line[0]))
        if len(points) < 2:
            continue
        sign_id = _numeric_id(stop_id, 5000)
        lanelet_ids = _lanelets_touching_stop_line(network, points)
        for lanelet_id in lanelet_ids:
            lanelet = network.find_lanelet_by_id(lanelet_id)
            cropped = _crop_stop_line(lanelet, points)
            lanelet.stop_line = StopLine(
                cropped[0],
                cropped[-1],
                LineMarking.SOLID_SOLID,
                traffic_sign_ref={sign_id},
            )
            lanelet.traffic_signs = {sign_id}
        position = stop_sign.stop_line[0].segment[0].line_segment.point[0]
        scenario.add_objects(
            TrafficSign(
                sign_id,
                [TrafficSignElement(TrafficSignIDGermany.STOP)],
                set(lanelet_ids),
                np.array([position.x, position.y]),
            ),
            set(lanelet_ids),
        )

    cycle = TrafficLightCycle(
        cycle_elements=[
            TrafficLightCycleElement(state=TrafficLightState.RED, duration=1000),
        ]
    )
    signal_ids: Dict[str, int] = {}
    signal_relations: Dict[str, List[Tuple[str, str]]] = {}
    for signal_id in parsed.get_signals():
        signal = parsed.get_signal_by_id(signal_id)
        if not signal.subsignal:
            continue
        subsignal = signal.subsignal[0]
        x, y = subsignal.location.x, subsignal.location.y
        direction = SIGNAL_DIRECTION.get(enum_name(subsignal, "type"), TrafficLightDirection.ALL)
        light_id = _numeric_id(signal_id, 10000)
        signal_ids[signal_id] = light_id
        light = TrafficLight(
            traffic_light_id=light_id,
            position=np.array([x, y]),
            direction=direction,
            traffic_light_cycle=cycle,
        )
        stop_points = []
        for stop_line in signal.stop_line:
            stop_points.extend(curve_to_points(stop_line))
        stop_points = np.array(stop_points) if stop_points else np.empty((0, 2))
        lanelet_ids = _lanelets_touching_stop_line(network, stop_points)
        if lanelet_ids:
            scenario.add_objects(light, set(lanelet_ids))
            for lanelet_id in lanelet_ids:
                lanelet = network.find_lanelet_by_id(lanelet_id)
                cropped = _crop_stop_line(lanelet, stop_points)
                lanelet.stop_line = StopLine(
                    cropped[0],
                    cropped[-1],
                    LineMarking.SOLID_SOLID,
                    traffic_light_ref={light_id},
                )
        signal_relations[signal_id] = parsed.get_signals_wrt(signal_id)

    (output_dir / f"{map_name}_signals_ids_map.json").write_text(json.dumps(signal_ids, indent=2))
    (output_dir / f"{map_name}_signals_relations_map.json").write_text(
        json.dumps(signal_relations, indent=2)
    )

    outside = [lane_id_map[lane_id] for lane_id in parsed.get_lanes_not_in_junction()]
    (output_dir / f"{map_name}_lanes_not_in_junction.json").write_text(json.dumps(outside, indent=2))

    reachable = {}
    feasible = {}
    for lane_id in parsed.get_lanes():
        paths = parsed.reachable_from(lane_id, 10)
        reached = {lane_id}
        for path in paths:
            reached.update(path)
        cr_id = lane_id_map[lane_id]
        reachable[str(cr_id)] = [lane_id_map[item] for item in reached]
        feasible[str(cr_id)] = [[cr_id]] + [
            [lane_id_map[item] for item in path] for path in paths
        ]
    (output_dir / f"{map_name}_rechable_lanelets.json").write_text(json.dumps(reachable, indent=2))
    (output_dir / f"{map_name}_feasible_paths.json").write_text(json.dumps(feasible, indent=2))

    xml_path = output_dir / f"{map_name}.xml"
    CommonRoadFileWriter(
        scenario,
        PlanningProblemSet(),
        file_format=FileFormat.XML,
    ).write_to_file(str(xml_path), OverwriteExistingFile.ALWAYS)
    return xml_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Convert an Apollo HD map to CommonRoad XML")
    parser.add_argument("map_name", help="Bundled map name or path to a base_map.bin")
    parser.add_argument(
        "-o",
        "--output",
        default="scripts/output/commonroad",
        help="Directory for the XML and id JSON files",
    )
    parser.add_argument("--filename", default="base_map.bin", help="File inside a bundled map directory")
    args = parser.parse_args(argv)
    xml_path = convert_map(args.map_name, args.output, filename=args.filename)
    print(xml_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
