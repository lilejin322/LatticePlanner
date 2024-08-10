from typing import List, Tuple
from protoclass.PointENU import PointENU
from common.Vec2d import Vec2d
from common.LineSegment2d import LineSegment2d
from common.Box2d import Box2d
from protoclass.Lane import Lane, Curve, LineSegment
from logging import Logger
from config import FLAGS_half_vehicle_width
from bisect import bisect_left
from ReferenceLine import kMathEpsilon
from PathMatcher import PathMatcher
from dataclasses import dataclass
from common.AABox2d import AABox2d
import math
from protoclass.Overlap import Overlap, ObjectOverlapInfo
from common.LaneSegmentKDTree import AABoxKDTreeParams

logger = Logger("LaneInfo")

kEpsilon: float = 0.1
"""Margin for comparation"""

kDuplicatedPointsEpsilon: float = 1e-7
"""Minimum distance to remove duplicated points."""

@dataclass
class Id:
    """
    Id class
    """

    id: str = ""

class OverlapInfo:
    """
    a wrapper class for Overlap
    """

    def __init__(self, overlap: Overlap):
        """
        Constructor

        :param Overlap overlap: An instance of the Overlap class.
        """

        self._overlap: Overlap = overlap

    @property
    def id(self) -> Id:
        """
        Get the id from the overlap object.

        :returns: The id of the overlap.
        :rtype: Id
        """

        return self._overlap.id

    @property
    def overlap(self) -> Overlap:
        """
        Get the overlap object.

        :returns: The overlap object.
        :rtype: Overlap
        """

        return self._overlap

    def GetObjectOverlapInfo(self, id: Id) -> ObjectOverlapInfo:
        """
        Get the ObjectOverlapInfo corresponding to a given id.

        :param Id id: The id for which to retrieve the ObjectOverlapInfo
        :returns: The ObjectOverlapInfo object or None if not found
        :rtype: ObjectOverlapInfo
        """

        for obj in self._overlap.objects:
            if obj.id.id == self.id.id:
                return obj
        return None

def PointFromVec2d(point: Vec2d) -> PointENU:
    """
    Convert Vec2d to PointENU

    :param Vec2d point: the input point
    :returns: the output point
    :rtype: PointENU
    """

    return PointENU(x = point.x, y = point.y)

def RemoveDuplicates(points: List[Vec2d]) -> None:
    """
    Remove duplicates from the list of points

    :param List[Vec2d] points: the list of points
    """

    if len(points) == 0:
        return
    count: int = 0
    limit: float = kDuplicatedPointsEpsilon ** 2
    for point in points:
        if count == 0 or point.DistanceSquareTo(points[count - 1]) > limit:
            points[count] = point
            count += 1
    points[:] = points[:count]

def PointsFromCurve(input_curve: Curve) -> List[Vec2d]:
    """
    Convert curve to points

    :param Curve input_curve: the input curve
    :returns: the points
    :rtype: List[Vec2d]
    """

    points: List[Vec2d] = []
    for curve in input_curve.segment:
        if isinstance(curve.curve_type, LineSegment):
            for point in curve.curve_type.point:
                points.append(Vec2d(point.x, point.y))
        else:
            logger.error(f"Can not handle curve type.")
    RemoveDuplicates(points)
    return points

class LaneInfo:

    def __init__(self, lane: Lane):
        """
        Constructor
        """

        self._lane: Lane = lane
        self._road_id: str = ""
        self._section_id: str = ""
        self._points: List[Vec2d] = []
        self._unit_directions: List[Vec2d] = []
        self._headings: List[float] = []
        self._segments: List[LineSegment2d] = []
        self._accumulated_s: List[float] = []
        self._overlap_ids = []
        self._overlaps: List[OverlapInfo] = []
        self._cross_lanes: List[OverlapInfo] = []
        self._signals: List[OverlapInfo] = []
        self._yield_signs: List[OverlapInfo] = []
        self._stop_signs: List[OverlapInfo] = []
        self._crosswalks: List[OverlapInfo] = []
        self._junctions: List[OverlapInfo] = []
        self._clear_areas: List[OverlapInfo] = []
        self._speed_bumps: List[OverlapInfo] = []
        self._parking_spaces: List[OverlapInfo] = []
        self._pnc_junctions: List[OverlapInfo] = []
        self._total_length: float = 0.0
        self._sampled_left_width: List[Tuple[float, float]] = []
        self._sampled_right_width: List[Tuple[float, float]] = []
        self._sampled_left_road_width: List[Tuple[float, float]] = []
        self._sampled_right_road_width: List[Tuple[float, float]] = []
        self._segment_box_list: List[Tuple[AABox2d, LaneInfo, LineSegment2d, int]] = []
        self._lane_segment_kdtree = None
        self.Init()

    @property
    def id(self) -> str:
        """
        Get the id of the lane

        :returns: the id of the lane
        :rtype: str
        """

        return self._lane.id

    @property
    def road_id(self) -> str:
        """
        Get the road id of the lane

        :returns: the road id of the lane
        :rtype: str
        """

        return self._road_id

    @property
    def section_id(self) -> str:
        """
        Get the section id of the lane

        :returns: the section id of the lane
        :rtype: str
        """

        return self._section_id

    @property
    def lane(self) -> Lane:
        """
        Get the lane object

        :returns: the lane object
        :rtype: Lane
        """

        return self._lane

    @property
    def points(self) -> List[Vec2d]:
        """
        Get the points of the lane

        :returns: the points of the lane
        :rtype: List[Vec2d]
        """

        return self._points

    @property
    def unit_directions(self) -> List[Vec2d]:
        """
        Get the unit directions of the lane

        :returns: the unit directions of the lane
        :rtype: List[Vec2d]
        """

        return self._unit_directions

    def Heading(self, s: float) -> float:
        
        if len(self._accumulated_s) == 0:
            return 0.0
        kEpsilon: float = 0.001
        if s + kEpsilon < self._accumulated_s[0]:
            logger.warning(f"s: {s} should be >= {self._accumulated_s[0]}")
            return self._headings[0]
        if s - kEpsilon > self._accumulated_s[-1]:
            logger.warning(f"s: {s} should be <= {self._accumulated_s[-1]}")
            return self._headings[-1]
        
        index: int = bisect_left(self._accumulated_s, s)
        if index == 0 or (index < len(self._accumulated_s) and self._accumulated_s[index] - s <= kMathEpsilon):
            return self._headings[index]
        return PathMatcher.slerp(self._headings[index - 1], self._accumulated_s[index - 1],
                                 self._headings[index], self._accumulated_s[index], s)

    def Curvature(self, s: float) -> float:

        if len(self._points) < 2:
            logger.error(f"Not engough points to compute curvature.")
            return 0.0
        kEpsilon: float = 0.001
        if s + kEpsilon < self._accumulated_s[0]:
            logger.error(f"s: {s} should be >= {self._accumulated_s[0]}")
            return 0.0
        if s > self._accumulated_s[-1] + kEpsilon:
            logger.error(f"s: {s} should be <= {self._accumulated_s[-1]}")
            return 0.0
        
        index: int = bisect_left(self._accumulated_s, s)
        if index == len(self._accumulated_s):
            logger.debug(f"Reach the end of lane.")
            return 0.0
        if index == 0:
            logger.debug(f"Reach the beginning of lane.")
            return 0.0
        return (self._headings[index] - self._headings[index - 1]) / (self._accumulated_s[index] - self._accumulated_s[index - 1] + kEpsilon)

    @property
    def headings(self) -> List[float]:
        """
        Get the headings of the lane

        :returns: the headings of the lane
        :rtype: List[float]
        """

        return self._headings

    @property
    def segments(self) -> List[LineSegment2d]:
        """
        Get the segments of the lane

        :returns: the segments of the lane
        :rtype: List[LineSegment2d]
        """

        return self._segments

    @property
    def accumulate_s(self) -> List[float]:
        """
        Get the accumulated s of the lane

        :returns: the accumulated s of the lane
        :rtype: List[float]
        """

        return self._accumulated_s

    @property
    def overlaps(self) -> List[OverlapInfo]:
        """
        Get the overlaps of the lane

        :returns: the overlaps of the lane
        :rtype: List[OverlapInfo]
        """

        return self._overlaps

    @property
    def cross_lanes(self) -> List[OverlapInfo]:
        """
        Get the cross lanes of the lane

        :returns: the cross lanes of the lane
        :rtype: List[OverlapInfo]
        """

        return self._cross_lanes

    @property
    def signals(self) -> List[OverlapInfo]:
        """
        Get the signals of the lane

        :returns: the signals of the lane
        :rtype: List[OverlapInfo]
        """

        return self._signals

    @property
    def yield_signs(self) -> List[OverlapInfo]:
        """
        Get the yield signs of the lane

        :returns: the yield signs of the lane
        :rtype: List[OverlapInfo]
        """

        return self._yield_signs

    @property
    def stop_signs(self) -> List[OverlapInfo]:
        """
        Get the stop signs of the lane
        
        :returns: the stop signs of the lane
        :rtype: List[OverlapInfo]
        """

        return self._stop_signs

    @property
    def crosswalks(self) -> List[OverlapInfo]:
        """
        Get the crosswalks of the lane

        :returns: the crosswalks of the lane
        :rtype: List[OverlapInfo]
        """

        return self._crosswalks

    @property
    def junctions(self) -> List[OverlapInfo]:
        """
        Get the junctions of the lane

        :returns: the junctions of the lane
        :rtype: List[OverlapInfo]
        """

        return self._junctions

    @property
    def clear_areas(self) -> List[OverlapInfo]:
        """
        Get the clear areas of the lane

        :returns: the clear areas of the lane
        :rtype: List[OverlapInfo]
        """

        return self._clear_areas

    @property
    def speed_bumps(self) -> List[OverlapInfo]:
        """
        Get the speed bumps of the lane

        :returns: the speed bumps of the lane
        :rtype: List[OverlapInfo]
        """

        return self._speed_bumps

    @property
    def parking_spaces(self) -> List[OverlapInfo]:
        """
        Get the parking spaces of the lane

        :returns: the parking spaces of the lane
        :rtype: List[OverlapInfo]
        """

        return self._parking_spaces

    @property
    def pnc_junctions(self) -> List[OverlapInfo]:
        """
        Get the pnc junctions of the lane

        :returns: the pnc junctions of the lane
        :rtype: List[OverlapInfo]
        """

        return self._pnc_junctions

    @property
    def total_length(self) -> float:
        """
        Get the total length of the lane

        :returns: the total length of the lane
        :rtype: float
        """

        return self._total_length

    @property
    def sampled_left_width(self) -> List[Tuple[float, float]]:
        """
        Get the sampled left width of the lane

        :returns: the sampled left width of the lane
        :rtype: List[Tuple[float, float]]
        """

        return self._sampled_left_width

    @property
    def sampled_right_width(self) -> List[Tuple[float, float]]:
        """
        Get the sampled right width of the lane

        :returns: the sampled right width of the lane
        :rtype: List[Tuple[float, float]]
        """

        return self._sampled_right_width

    def GetWidth(self, s: float) -> Tuple[float, float, float]:
        """

        :returns: left_width, right_width, width
        :rtype: Tuple[float, float, float]
        """

        left_width: float = self.GetWidthFromSample(self._sampled_left_width, s)
        right_width: float = self.GetWidthFromSample(self._sampled_right_width, s)
        return left_width, right_width, left_width + right_width

    def GetEffectiveWidth(self, s: float) -> float:
        """
        Get the effective width of the lane
        """

        left_width, right_width, _ = self.GetWidth(s)
        return 2.0 * min(left_width, right_width)

    @property
    def sampled_left_road_width(self) -> List[Tuple[float, float]]:
        """
        Get the sampled left road width of the lane

        :returns: the sampled left road width of the lane
        :rtype: List[Tuple[float, float]]
        """

        return self._sampled_left_road_width

    @property
    def sampled_right_road_width(self) -> List[Tuple[float, float]]:
        """
        Get the sampled right road width of the lane

        :returns: the sampled right road width of the lane
        :rtype: List[Tuple[float, float]]
        """

        return self._sampled_right_road_width

    def GetRoadWidth(self, s: float) -> Tuple[float, float, float]:
        """

        :returns: left_road_width, right_road_width, road_width
        :rtype: Tuple[float, float, float]
        """

        left_width: float = self.GetWidthFromSample(self._sampled_left_road_width, s)
        right_width: float = self.GetWidthFromSample(self._sampled_right_road_width, s)
        return left_width, right_width, left_width + right_width

    def IsOnLane(self, *args) -> bool:
        
        if isinstance(args[0], Vec2d):
            """

            :param Vec2d point: the point to be checked
            :returns: whether the point is on the lane
            :rtype: bool
            """

            point: Vec2d = args[0]
            tag, accumulate_s, lateral = self.GetProjection(point)
            if not tag:
                return False
            
            if accumulate_s > self.total_length + kEpsilon or accumulate_s + kEpsilon < 0.0:
                return False
            
            left_width, right_width, _ = self.GetWidth(accumulate_s)
            if lateral < left_width and lateral > -right_width:
                return True
            return False

        elif isinstance(args[0], Box2d):
            """

            :param Box2d box: the box to be checked
            :returns: whether the box is on the lane
            :rtype: bool
            """

            box: Box2d = args[0]
            corners: List[Vec2d] = box.GetAllCorners()
            for corner in corners:
                if not self.IsOnLane(corner):
                    return False
            return True

    def GetSmoothPoint(self, s: float) -> PointENU:
        """
        Get the smooth point of the lane at s

        :param float s: the s value
        :returns: the smooth point of the lane at s
        :rtype: PointENU
        """

        point: PointENU = PointENU()
        if len(self._points) < 2:
            return point
        if s <= 0.0:
            return PointFromVec2d(self._points[0])
        
        if s >= self.total_length:
            return PointFromVec2d(self._points[-1])
        
        index: int = bisect_left(self._accumulated_s, s)
        if index == len(self._accumulated_s):
            return point
        delta_s: float = self._accumulated_s[index] - s
        if delta_s < kMathEpsilon:
            return PointFromVec2d(self._points[index])
        
        smooth_point = self._points[index] - self._unit_directions[index - 1] * delta_s

        return PointFromVec2d(smooth_point)

    def DistanceTo(self, point: Vec2d) -> Tuple[float, Vec2d, float, int]:
        """
        Get the distance to the lane

        :param Vec2d point: the point to be checked
        :returns: the distance, th map_point, the s_offset, the s_offset_index
        :rtype: Tuple[float, Vec2d, float, int]
        """

        segment_box = self._lane_segment_kdtree.GetNearestObject(point)
        if segment_box is None:
            return float('inf'), Vec2d(), float('inf'), -1
        index: int = segment_box.id()
        distance, map_point = self._segments[index].DistanceTo(point)
        s_offset_index: int = index
        s_offset = self._accumulated_s[index] + self._segments[index].start.DistanceTo(map_point)
        return distance, map_point, s_offset, s_offset_index

    def GetNearestPoint(self, point: Vec2d) -> Tuple[PointENU, float]:
        """
        Get the nearest point on the lane

        :param Vec2d point: the point to be checked
        :returns: the nearest point on the lane and the distance
        :rtype: Tuple[PointENU, float]
        """

        segment_box = self._lane_segment_kdtree.GetNearestObject(point)
        if segment_box is None:
            return PointENU(), float("inf")
        index: int = segment_box.id()
        distance, nearest_point = self._segments[index].DistanceTo(point)

        return PointFromVec2d(nearest_point), distance

    def GetProjection(self, point: Vec2d, heading: float=None) -> Tuple[bool, float, float]:
        """
        Get the projection of the point on the lane

        :param Vec2d point: the point to be projected
        :param float heading: the heading of the point
        :returns: whether the projection is successful, float accumulated_s, float lateral
        :rtype: Tuple[bool, float, float]
        """
        
        if len(self._segments) == 0:
            return False, float("inf"), float("inf")
        min_dist: float = float("inf")
        seg_num: int = len(self._segments)
        min_index: int = 0
        for i in range(seg_num):
            if heading is not None:
                if abs(AngleDiff(self._segments[i].heading(), heading)) >= math.pi / 2.0:
                    continue
            distance: float = self._segments[i].DistanceSquareTo(point)
            if distance < min_dist:
                min_index = i
                min_dist = distance
        min_dist = math.sqrt(min_dist)
        nearest_seg = self._segments[min_index]
        prod = nearest_seg.ProductOntoUnit(point)
        proj = nearest_seg.ProjectOntoUnit(point)
        if min_index == 0:
            accumulated_s = min(proj, nearest_seg.length())
            if proj < 0:
                lateral = prod
            else:
                lateral = min_dist if prod > 0.0 else -min_dist
        elif min_index == seg_num - 1:
            accumulated_s = self._accumulated_s[min_index] + max(0.0, proj)
            if proj > 0:
                lateral = prod
            else:
                lateral = min_dist if prod > 0.0 else -min_dist
        else:
            accumulated_s = self._accumulated_s[min_index] + max(0.0, min(proj, nearest_seg.length()))
            lateral = min_dist if prod > 0.0 else -min_dist
        return True, accumulated_s, lateral

    def Init(self) -> None:
        """
        Init method
        """

        self._points = PointsFromCurve(self._lane.central_curve)
        assert len(self._points) >= 2, "Lane should have at least two points."
        self._segments.clear()
        self._accumulated_s.clear()
        self._unit_directions.clear()
        self._headings.clear()

        s: float = 0
        for i in range(len(self._points) - 1):
           self._segments.append(LineSegment2d(self._points[i], self._points[i + 1]))
           self._accumulated_s.append(s)
           self._unit_directions.append(self._segments[-1].unit_direction)
           s += self._segments[-1].length()

        self._accumulated_s.append(s)
        self._total_length = s
        assert len(self._unit_directions) > 0, "LaneInfo._unit_directions should not be empty."
        self._unit_directions.append(self._unit_directions[-1])
        for direction in self._unit_directions:
            self._headings.append(direction.Angle())
        for overlap_id in self._lane.overlap_id:
            self._overlap_ids.append(overlap_id.id)
        assert len(self._segments) > 0, "LaneInfo._segments should not be empty."

        self._sampled_left_width.clear()
        self._sampled_right_width.clear()
        for sample in self._lane.left_sample:
            self._sampled_left_width.append((sample.s, sample.width))
        for sample in self._lane.right_sample:
            self._sampled_right_width.append((sample.s, sample.width))
        
        if self._lane.type is not None:
            if self._lane.type == Lane.LaneType.CITY_DRIVING:
                for p in self._sampled_left_width:
                    if p[1] < FLAGS_half_vehicle_width:
                        logger.error(f"_lane[id = {self._lane.id.id}]. _sampled_left_width[{p[1]}] is too small. It should be larger than half vehicle width[{FLAGS_half_vehicle_width}].")
                for p in self._sampled_right_width:
                    if p[1] < FLAGS_half_vehicle_width:
                        logger.error(f"_lane[id = {self._lane.id.id}]. _sampled_right_width[{p[1]}] is too small. It should be larger than half vehicle width[{FLAGS_half_vehicle_width}].")
            elif self._lane.type == Lane.LaneType.NONE:
                logger.error(f"_lane[id = {self._lane.id.id}] type is NONE.")
        else:
            logger.error(f"_lane[id = {self._lane.id.id}] has NO type.")
        
        self._sampled_left_road_width.clear()
        self._sampled_right_road_width.clear()
        for sample in self._lane.left_road_sample:
            self._sampled_left_road_width.append((sample.s, sample.width))
        for sample in self._lane.right_road_sample:
            self._sampled_right_road_width.append((sample.s, sample.width))
        
        self.CreateKDTree()

    def PostProcess(self, map_instance: HDMapImpl):
        """
        Post process method
        """

        self.UpdateOverlaps(map_instance)

    def UpdateOverlaps(self, map_instance: HDMapImpl):
        """
        Update overlaps method
        """

        for overlap_id in self._overlap_ids:
            overlap = map_instance.GetOverlapById(MakeMapId(overlap_id))
            if overlap is None:
                continue
            self._overlaps.append(overlap)
            for obj in overlap.overlap.object:
                object_id = obj.id.id
                if object_id == self._lane.id.id:
                    continue
                object_map_id = MakeMapId(object_id)
                if map_instance.GetLaneById(object_map_id) is not None:
                    self._cross_lanes.append(overlap)
                if map_instance.GetSignalById(object_map_id) is not None:
                    self._signals.append(overlap)
                if map_instance.GetYieldSignById(object_map_id) is not None:
                    self._yield_signs.append(overlap)
                if map_instance.GetStopSignById(object_map_id) is not None:
                    self._stop_signs.append(overlap)
                if map_instance.GetCrosswalkById(object_map_id) is not None:
                    self._crosswalks.append(overlap)
                if map_instance.GetJunctionById(object_map_id) is not None:
                    self._junctions.append(overlap)
                if map_instance.GetClearAreaById(object_map_id) is not None:
                    self._clear_areas.append(overlap)
                if map_instance.GetSpeedBumpById(object_map_id) is not None:
                    self._speed_bumps.append(overlap)
                if map_instance.GetParkingSpaceById(object_map_id) is not None:
                    self._parking_spaces.append(overlap)
                if map_instance.GetPncJunctionById(object_map_id) is not None:
                    self._pnc_junctions.append(overlap)

    def GetWidthFromSample(self, samples: List[Tuple[float, float]], s: float) -> float:
        """
        Get the width from the sample

        :param List[Tuple[float, float]] samples: the samples
        :param float s: the s value
        :returns: the width
        :rtype: float
        """
        
        if len(samples) == 0:
            return 0.0
        if s <= samples[0][0]:
            return samples[0][1]
        if s >= samples[-1][0]:
            return samples[-1][1]
        low: int = 0
        high: int = len(samples)
        while low + 1 < high:
            mid: int = (low + high) >> 1
            if samples[mid][0] <= s:
                low = mid
            else:
                high = mid
        sample1: Tuple[float, float] = samples[low]
        sample2 = Tuple[float, float] = samples[high]
        ratio: float = (sample2[0] - s) / (sample2[0] - sample1[0])
        return sample1[1] * ratio + sample2[1] * (1.0 - ratio)

    def CreateKDTree(self) -> None:
        """
        Create the kd tree
        """

        params = AABoxKDTreeParams()
        params.max_leaf_dimension = 5.0
        params.max_leaf_size = 16

        self._segment_box_list.clear()
        for id in range(len(self._segments)):
            segment = self._segments[id]
            self._segment_box_list.append((AABox2d(segment.start, segment.end), self, segment, id))

        self._lane_segment_kdtree = LaneSegmentKDTree(self._segment_box_list, params)

    def set_road_id(self, road_id: str) -> None:
        """
        Set the road id of the lane

        :param str road_id: the road id of the lane
        """

        self._road_id = road_id

    def set_section_id(self, section_id: str) -> None:
        """
        Set the section id of the lane

        :param str section_id: the section id of the lane
        """

        self._section_id = section_id
