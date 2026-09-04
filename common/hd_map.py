import os
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

from common.lane_info import Id, LaneInfo, OverlapInfo
from common.vec2d import Vec2d
from common.geometry_utils import AngleDiff
from protoclass.lane import Lane
from protoclass.overlap import Overlap
from protoclass.point_enu import PointENU
import config as config_module


def _load_map_pb2():
    from generated.hdmap.modules.map.proto import map_pb2
    return map_pb2


def _parse_map_file(map_filename: str):
    map_pb2 = _load_map_pb2()
    map_proto = map_pb2.Map()
    with open(map_filename, "rb") as file_in:
        try:
            map_proto.ParseFromString(file_in.read())
            return map_proto
        except Exception:
            pass
    import google.protobuf.text_format as text_format
    with open(map_filename, "r") as file_in:
        text_format.Merge(file_in.read(), map_proto)
    return map_proto


def BaseMapFile(map_dir: Optional[str] = None) -> Optional[str]:
    search_dir = map_dir or config_module.FLAGS_map_dir
    if not os.path.isabs(search_dir):
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        search_dir = os.path.join(repo_root, search_dir)
    for name in config_module.FLAGS_base_map_filename.split("|"):
        candidate = os.path.join(search_dir, name)
        if os.path.isfile(candidate):
            return candidate
    return None


def _id_to_str(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if hasattr(value, "id"):
        return _id_to_str(value.id)
    return str(value)


def MakeMapId(id_str: Any) -> Id:
    return Id(_id_to_str(id_str))


@dataclass
class RoadInfo:
    road: Any

    @property
    def id(self):
        return getattr(self.road, "id", None)

    @property
    def type(self):
        return getattr(self.road, "type", None)


@dataclass
class HDMap:
    """Apollo-style in-memory HDMap facade for Python dataclass map objects."""

    _lanes: Dict[str, LaneInfo] = field(default_factory=dict)
    _overlaps: Dict[str, OverlapInfo] = field(default_factory=dict)
    _roads: Dict[str, RoadInfo] = field(default_factory=dict)
    _signals: Dict[str, Any] = field(default_factory=dict)
    _junctions: Dict[str, Any] = field(default_factory=dict)
    _crosswalks: Dict[str, Any] = field(default_factory=dict)
    _stop_signs: Dict[str, Any] = field(default_factory=dict)
    _yield_signs: Dict[str, Any] = field(default_factory=dict)
    _clear_areas: Dict[str, Any] = field(default_factory=dict)
    _speed_bumps: Dict[str, Any] = field(default_factory=dict)
    _parking_spaces: Dict[str, Any] = field(default_factory=dict)
    _pnc_junctions: Dict[str, Any] = field(default_factory=dict)
    _rsus: Dict[str, Any] = field(default_factory=dict)

    def LoadMapFromProto(self, map_proto: Any) -> int:
        self.Clear()
        for lane in getattr(map_proto, "lane", []) or []:
            self.AddLane(lane)
        for overlap in getattr(map_proto, "overlap", []) or []:
            self.AddOverlap(overlap)
        for road in getattr(map_proto, "road", []) or []:
            self.AddRoad(road)
        for road_info in self._roads.values():
            road_id = _id_to_str(road_info.id)
            for section in getattr(road_info.road, "section", []) or []:
                section_id = _id_to_str(getattr(section, "id", None))
                for lane_id in getattr(section, "lane_id", []) or []:
                    lane_id_str = _id_to_str(lane_id)
                    lane_info = self._lanes.get(lane_id_str)
                    if lane_info is None:
                        raise ValueError(
                            f"Road {road_id} references unknown lane {lane_id_str}"
                        )
                    lane_info.set_road_id(road_id)
                    lane_info.set_section_id(section_id)
        for attr, table in (
            ("signal", self._signals),
            ("junction", self._junctions),
            ("crosswalk", self._crosswalks),
            ("stop_sign", self._stop_signs),
            ("yield_", self._yield_signs),
            ("clear_area", self._clear_areas),
            ("speed_bump", self._speed_bumps),
            ("parking_space", self._parking_spaces),
            ("pnc_junction", self._pnc_junctions),
            ("rsu", self._rsus),
        ):
            for obj in getattr(map_proto, attr, []) or []:
                table[_id_to_str(getattr(obj, "id", None))] = obj
        for lane in self._lanes.values():
            lane.PostProcess(self)
        return 0

    def LoadMapFromFile(self, map_filename: str) -> int:
        map_proto = _parse_map_file(map_filename)
        return self.LoadMapFromProto(map_proto)

    def Clear(self) -> None:
        self._lanes.clear()
        self._overlaps.clear()
        self._roads.clear()
        self._signals.clear()
        self._junctions.clear()
        self._crosswalks.clear()
        self._stop_signs.clear()
        self._yield_signs.clear()
        self._clear_areas.clear()
        self._speed_bumps.clear()
        self._parking_spaces.clear()
        self._pnc_junctions.clear()
        self._rsus.clear()

    def AddLane(self, lane: Any) -> LaneInfo:
        lane_info = lane if isinstance(lane, LaneInfo) else LaneInfo(lane)
        self._lanes[_id_to_str(lane_info.id)] = lane_info
        return lane_info

    def AddOverlap(self, overlap: Any) -> OverlapInfo:
        overlap_info = overlap if isinstance(overlap, OverlapInfo) else OverlapInfo(overlap)
        self._overlaps[_id_to_str(overlap_info.id)] = overlap_info
        return overlap_info

    def AddRoad(self, road: Any) -> RoadInfo:
        road_info = road if isinstance(road, RoadInfo) else RoadInfo(road)
        self._roads[_id_to_str(road_info.id)] = road_info
        return road_info

    def GetLaneById(self, id_value: Any) -> Optional[LaneInfo]:
        return self._lanes.get(_id_to_str(id_value))

    def GetOverlapById(self, id_value: Any) -> Optional[OverlapInfo]:
        return self._overlaps.get(_id_to_str(id_value))

    def GetRoadById(self, id_value: Any) -> Optional[RoadInfo]:
        return self._roads.get(_id_to_str(id_value))

    def GetSignalById(self, id_value: Any):
        return self._signals.get(_id_to_str(id_value))

    def GetJunctionById(self, id_value: Any):
        return self._junctions.get(_id_to_str(id_value))

    def GetCrosswalkById(self, id_value: Any):
        return self._crosswalks.get(_id_to_str(id_value))

    def GetStopSignById(self, id_value: Any):
        return self._stop_signs.get(_id_to_str(id_value))

    def GetYieldSignById(self, id_value: Any):
        return self._yield_signs.get(_id_to_str(id_value))

    def GetClearAreaById(self, id_value: Any):
        return self._clear_areas.get(_id_to_str(id_value))

    def GetSpeedBumpById(self, id_value: Any):
        return self._speed_bumps.get(_id_to_str(id_value))

    def GetParkingSpaceById(self, id_value: Any):
        return self._parking_spaces.get(_id_to_str(id_value))

    def GetPncJunctionById(self, id_value: Any):
        return self._pnc_junctions.get(_id_to_str(id_value))

    def GetRSUById(self, id_value: Any):
        return self._rsus.get(_id_to_str(id_value))

    def GetLanes(self, point: PointENU, distance: float) -> List[LaneInfo]:
        xy = Vec2d(point.x, point.y)
        lanes = []
        for lane in self._lanes.values():
            lane_distance, _, _, _ = lane.DistanceTo(xy)
            if lane_distance <= distance:
                lanes.append(lane)
        return lanes

    def GetNearestLane(self, point: PointENU) -> Tuple[bool, Optional[LaneInfo], float, float]:
        xy = Vec2d(point.x, point.y)
        best_lane = None
        best_s = 0.0
        best_l = 0.0
        best_distance = float("inf")
        for lane in self._lanes.values():
            distance, _, s_offset, s_offset_index = lane.DistanceTo(xy)
            if s_offset_index < 0:
                continue
            if distance < best_distance:
                segment = lane.segments[min(s_offset_index, len(lane.segments) - 1)]
                best_lane = lane
                best_s = s_offset
                best_l = segment.unit_direction.CrossProd(xy - segment.start)
                best_distance = distance
        return best_lane is not None, best_lane, best_s, best_l

    def GetLanesWithHeading(
        self,
        point: PointENU,
        distance: float,
        central_heading: float,
        max_heading_difference: float,
    ) -> List[LaneInfo]:
        xy = Vec2d(point.x, point.y)
        lanes = []
        for lane in self.GetLanes(point, distance):
            lane_distance, _, _, s_offset_index = lane.DistanceTo(xy)
            if s_offset_index < 0 or lane_distance > distance:
                continue
            heading = lane.headings[min(s_offset_index, len(lane.headings) - 1)]
            if abs(AngleDiff(heading, central_heading)) <= max_heading_difference:
                lanes.append(lane)
        return lanes

    def GetNearestLaneWithHeading(
        self,
        point: PointENU,
        distance: float,
        central_heading: float,
        max_heading_difference: float,
    ) -> Tuple[bool, Optional[LaneInfo], float, float]:
        xy = Vec2d(point.x, point.y)
        best_lane = None
        best_s = 0.0
        best_s_offset_index = 0
        best_distance = distance
        for lane in self.GetLanesWithHeading(point, distance, central_heading, max_heading_difference):
            lane_distance, _, s_offset, s_offset_index = lane.DistanceTo(xy)
            if lane_distance < best_distance:
                best_distance = lane_distance
                best_lane = lane
                best_s = s_offset
                best_s_offset_index = s_offset_index
        if best_lane is None:
            return False, None, 0.0, 0.0
        segment = best_lane.segments[min(best_s_offset_index, len(best_lane.segments) - 1)]
        best_l = segment.unit_direction.CrossProd(xy - segment.start)
        return True, best_lane, best_s, best_l

    def GetRoads(self, point: PointENU, distance: float) -> List[RoadInfo]:
        roads = []
        road_ids = set()
        for lane in self.GetLanes(point, distance):
            road_id = lane.road_id
            if not road_id or road_id in road_ids:
                continue
            road = self.GetRoadById(road_id)
            assert road is not None, f"Unknown road id referenced by lane: {road_id}"
            road_ids.add(road_id)
            roads.append(road)
        return roads


class HDMapUtil:
    _base_map = HDMap()

    @staticmethod
    def BaseMapPtr(relative_map=None) -> HDMap:
        return HDMapUtil._base_map

    @staticmethod
    def BaseMap() -> HDMap:
        return HDMapUtil._base_map

    @staticmethod
    def SetBaseMap(hdmap: HDMap) -> None:
        HDMapUtil._base_map = hdmap

    @staticmethod
    def ReloadMaps(map_dir: Optional[str] = None) -> bool:
        map_file = BaseMapFile(map_dir)
        if map_file is None:
            return False
        hdmap = HDMap()
        if hdmap.LoadMapFromFile(map_file) != 0:
            return False
        HDMapUtil._base_map = hdmap
        return True


def CreateMap(map_file_path: str) -> HDMap:
    hdmap = HDMap()
    if hdmap.LoadMapFromFile(map_file_path) != 0:
        raise RuntimeError(f"Failed to load map from {map_file_path}")
    return hdmap
