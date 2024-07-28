from typing import List, Tuple, Any, Dict
from dataclasses import dataclass
import math
from common.MapPathPoint import MapPathPoint
from common.ReferenceLine import LaneSegment, InterpolatedIndex, kMathEpsilon
from common.Vec2d import Vec2d
from common.LineSegment2d import LineSegment2d
from common.Box2d import Box2d
from common.MapPathPoint import LaneWaypoint
from collections import defaultdict
from bisect import bisect_left, bisect_right

kSampleDistance: float = 0.25

def FindLaneSegment(p1: MapPathPoint, p2: MapPathPoint) -> Tuple[bool, LaneSegment]:
    """
    Find lane segment

    :param MapPathPoint p1: the first point
    :param MapPathPoint p2: the second point
    :returns: (bool, lane_segment)
    :rtype: Tuple[bool, LaneSegment]
    """

    for wp1 in p1.lane_waypoints:
        if wp1.lane is None:
            continue
        for wp2 in p2.lane_waypoints:
            if wp2.lane is None:
                continue
            if wp1.lane['id'] == wp2.lane['id'] and wp1.s < wp2.s:
                return (True, LaneSegment(wp1.lane, wp1.s, wp2.s))
    return (False, None)

@dataclass
class PathApproximation:
    # Define properties as per the original PathApproximation structure
    pass

@dataclass
class PathOverlap:
    """
    PathOverlap class
    """

    object_id: str = ""
    start_s: float = 0.0
    end_s: float = 0.0

    def __str__(self) -> str:
        """
        Return string representation of PathOverlap

        :returns: String representation of PathOverlap
        :rtype: str
        """

        return f"PathOverlap(object_id = {self.object_id}, start_s = {self.start_s}, end_s = {self.end_s})"

class Path:
    """
    Path class
    """

    def __init__(self, *args):

        self._accumulated_s = []
        self._segments = []
        self._unit_directions = []
        self._lane_segments_to_next_point = []
        self._lane_left_width = []
        self._lane_right_width = []
        self._road_left_width = []
        self._road_right_width = []
        self._last_point_index = []
        if len(args) == 1:

            if isinstance(args[0][0], MapPathPoint):
                """
                Constructor with path_points

                :param List[MapPathPoint] path_points: Path points
                """

                path_points: List[MapPathPoint] = args[0]
                self._path_points = path_points
                self.Init()
            
            elif isinstance(args[0][0], LaneSegment):
                """
                Constructor with lane_segments
                
                :param List[LaneSegment] lane_segments: Lane segments
                """

                lane_segments: List[LaneSegment] = args[0]
                self._lane_segments = lane_segments
                for segment in self._lane_segments:
                    points = MapPathPoint.GetPointsFromLane(segment.lane, segment.start_s, segment.end_s)
                    self._path_points.extend(points)
                MapPathPoint.RemoveDuplicates(self._path_points)
                assert len(self._path_points) >= 2
                self.Init()

            else:
                raise ValueError("Invalid parameter type")

        elif len(args) == 2:
            """
            Constructor with path_points and lane_segments

            :param List[MapPathPoint] path_points: Path points
            :param List[LaneSegment] lane_segments: Lane segments
            """

            path_points, lane_segments = args
            self._path_points = path_points
            self._lane_segments = lane_segments
            self.Init()

        elif len(args) == 3:
            """
            Constructor with path_points, lane_segments and max_approximation_error

            :param List[MapPathPoint] path_points: Path points
            :param List[LaneSegment] lane_segments: Lane segments
            :param float max_approximation_error: Max approximation error
            """

            path_points, lane_segments, max_approximation_error = args
            self._path_points = path_points
            self._lane_segments = lane_segments
            self.Init()
            if max_approximation_error > 0.0:
                self._use_path_approximation = True
                self._approximation = PathApproximation(self, max_approximation_error)

        else:
            raise ValueError("Invalid number of parameters")

    def Init(self) -> None:
        """
        Init method
        """

        self.InitPoints()
        self.InitLaneSegments()
        self.InitPointIndex()
        self.InitWidth()
        self.InitOverlaps()

    def InitPoints(self) -> None:
        """
        Init points method
        """

        self._num_points = len(self._path_points)
        assert self._num_points >= 2

        self._accumulated_s.clear()
        self._segments.clear()
        self._unit_directions.clear()

        s: float = 0.0
        for i in range (self._num_points):
            self._accumulated_s.append(s)
            if i + 1 >= self._num_points:
                heading = self._path_points[i] - self._path_points[i - 1]
                heading.Normalize()
            else:
                self._segments.append(LineSegment2d(self._path_points[i], self._path_points[i + 1]))
                heading = self._path_points[i + 1] - self._path_points[i]
                heading_length: float = heading.Length()
                # TODO(All): use heading.length when all adjacent lanes are guarantee to
                # be connected.
                s += heading_length
                # Normalize "heading".
                if heading_length > 0.0:
                    heading /= heading_length
            self._unit_directions.append(heading)
        self._length = s
        self._num_sample_points = int(self._length / kSampleDistance) + 1
        self._num_segments = self._num_points - 1

        assert len(self._accumulated_s) == int(self._num_points), "accumulated_s size is not equal to num_points"
        assert len(self._unit_directions) == int(self._num_points), "unit_directions size is not equal to num_points"
        assert len(self._segments) == int(self._num_segments), "segments size is not equal to num_segments"

    def InitLaneSegments(self) -> None:
        """
        Init lane segments method
        """
        
        if not self._lane_segments:
            for i in range(self._num_points - 1):
                tag, lane_segment = FindLaneSegment(self._path_points[i], self._path_points[i + 1])
                if tag:
                    self._lane_segments.append(lane_segment)
        LaneSegment.Join(self._lane_segments)
        if not self._lane_segments:
            return
        self._lane_accumulated_s = self._lane_accumulated_s[:len(self._lane_segments)]
        self._lane_accumulated_s[0] = self._lane_segments[0].Length()
        for i in range(1, len(self._lane_segments)):
            self._lane_accumulated_s[i] = self._lane_accumulated_s[i - 1] + self._lane_segments[i].Length()
        
        self._lane_segments_to_next_point.clear()
        for i in range(self._num_points - 1):
            tag, lane_segment = FindLaneSegment(self._path_points[i], self._path_points[i + 1])
            if tag:
                self._lane_segments_to_next_point.append(lane_segment)
            else:
                self._lane_segments_to_next_point.append(LaneSegment())
        assert len(self._lane_segments_to_next_point) == int(self._num_segments), "self._lane_segments_to_next_point size is not equal to num_segments"

    def InitWidth(self) -> None:
        """
        Init width method
        """

        self._lane_left_width.clear()
        self._lane_right_width.clear()
        self._road_left_width.clear()
        self._road_right_width.clear()

        sample_s: float = 0.0
        segment_end_s: float = -1.0
        segment_start_s: float = -1.0
        waypoint_s: float = 0.0
        left_width: float = 0.0
        right_width: float = 0.0
        cur_waypoint: LaneWaypoint = None
        is_reach_to_end: bool = False
        path_point_index: int = 0
        for i in range (self._num_sample_points):
            # Find the segment at the position of "sample_s".
            while segment_end_s < sample_s and (not is_reach_to_end):
                cur_point = self._path_points[path_point_index]
                cur_waypoint = cur_point.lane_waypoints[0]
                assert cur_waypoint.lane, f"cur_point.lane is empty dict, cur_point = {cur_point}"
                segment_start_s = self._lane_accumulated_s[path_point_index]
                segment_end_s = segment_start_s + self._segments[path_point_index].length()
                path_point_index += 1
                if path_point_index >= self._num_points:
                    is_reach_to_end = True
            
            # Find the width of the way point at the position of "sample_s"
            waypoint_s = cur_waypoint.s + sample_s - segment_start_s
            left_width, right_width = cur_waypoint.lane.GetWidth(waypoint_s)
            self._lane_left_width.append(left_width - cur_waypoint.l)
            self._lane_right_width.append(right_width + cur_waypoint.l)
            left_width, right_width = cur_waypoint.lane.GetRoadWidth(waypoint_s)
            self._road_left_width.append(left_width - cur_waypoint.l)
            self._road_right_width.append(right_width + cur_waypoint.l)
            sample_s += kSampleDistance

        # Check the width array size.
        num_sample_points = self._num_sample_points
        assert len(self._lane_left_width) == num_sample_points
        assert len(self._lane_right_width) == num_sample_points
        assert len(self._road_left_width) == num_sample_points
        assert len(self._road_right_width) == num_sample_points

    def InitPointIndex(self) -> None:
        """
        Init point index method
        """
        
        self._last_point_index.clear()
        s: float = 0.0
        last_index: int = 0
        for i in range(self._num_sample_points):
            while last_index + 1 < self._num_points and self._accumulated_s[last_index + 1] <= s:
                last_index += 1
            self._last_point_index.append(last_index)
            s += kSampleDistance
        assert len(self._last_point_index) == self._num_sample_points

    def InitOverlaps(self) -> None:
        """
        Init overlaps method
        """

        self._lane_overlaps = self.GetAllOverlaps("lane")
        self._signal_overlaps = self.GetAllOverlaps("signal")
        self._yield_sign_overlaps = self.GetAllOverlaps("yield")
        self._stop_sign_overlaps = self.GetAllOverlaps("stop")
        self._crosswalk_overlaps = self.GetAllOverlaps("crosswalk")
        self._junction_overlaps = self.GetAllOverlaps("junction")
        self._pnc_junction_overlaps = self.GetAllOverlaps("pnc_junction")
        self._clear_area_overlaps = self.GetAllOverlaps("clear_area")
        self._speed_bump_overlaps = self.GetAllOverlaps("speed_bump")
        self._parking_space_overlaps = self.GetAllOverlaps("parking_space")

    def GetSmoothPoint(self, param: Any) -> MapPathPoint:

        if isinstance(param, InterpolatedIndex):
            """
            Return smooth coordinate by interpolated index.

            :param InterpolatedIndex index: Interpolated index
            :returns: Smooth coordinate
            :rtype: MapPathPoint
            """

            index: InterpolatedIndex = param
            assert index.id >= 0 and index.id < self._num_points

            ref_point = self._path_points[index.id]
            if abs(index.offset) > kMathEpsilon:
                delta:Vec2d = self._unit_directions[index.id] * index.offset
                point: MapPathPoint = MapPathPoint(Vec2d(ref_point.x() + delta.x(), ref_point.y() + delta.y()), ref_point.heading)
                if index.id < self._num_segments and ref_point.lane_waypoints:
                    lane_segment: LaneSegment = self._lane_segments_to_next_point[index.id]
                    ref_lane_waypoint: LaneWaypoint = ref_point.lane_waypoints[0]
                    if lane_segment.lane:
                        for lane_waypoint in ref_point.lane_waypoints:
                            if lane_waypoint.lane['id'] == lane_segment.lane['id']:
                                ref_lane_waypoint = lane_waypoint
                                break
                        point.add_lane_waypoint(LaneWaypoint(lane_segment.lane, lane_segment.start_s + index.offset, ref_lane_waypoint.l))
                if not point.lane_waypoints and ref_point.lane_waypoints:
                    point.add_lane_waypoint(ref_point.lane_waypoints[0])
                return point
            else:
                return ref_point

        elif isinstance(param, float):
            """
            Return smooth coordinate by accumulate_s.

            :param float s: Accumulate s
            :returns: Smooth coordinate
            :rtype: MapPathPoint
            """

            s: float = param
            return self.GetSmoothPoint(self.GetIndexFromS(s))

        else:

            raise ValueError("Invalid parameter type")

    def GetSFromIndex(self, index: InterpolatedIndex) -> float:
        """
        Compute accumulate s value of the index.

        :param InterpolatedIndex index: Interpolated index
        :returns: Accumulate s value
        :rtype: float
        """

        if index.id < 0:
            return 0.0
        if index.id >= self._num_points:
            return self._length
        return self._accumulated_s[index.id] + index.offset

    def GetIndexFromS(self, s: float) -> InterpolatedIndex:
        """
        Compute interpolated index by accumulate_s.

        :param float s: Accumulate s
        :returns: Interpolated index
        :rtype: InterpolatedIndex
        """
        
        if s <= 0.0:
            return InterpolatedIndex(0, 0.0)
        assert self._num_points > 0, "self._num_points is less than 0"
        if s >= self._length:
            return InterpolatedIndex(self._num_points - 1, 0.0)
        sample_id: int = s // kSampleDistance
        if sample_id >= self._num_sample_points:
            return InterpolatedIndex(self._num_points - 1, 0.0)
        next_sample_id: int = sample_id + 1
        low: int = self._last_point_index[sample_id]
        high: int = min(self._num_points, self._last_point_index[next_sample_id] + 1) if next_sample_id < self._num_sample_points else self._num_points
        while low + 1 < high:
            mid: int = (low + high) >> 1
            if self._accumulated_s[mid] <= s:
                low = mid
            else:
                high = mid
        return InterpolatedIndex(low, s - self._accumulated_s[low])

    def GetLaneIndexFromS(self, s: float) -> InterpolatedIndex:
        """
        get the index of the lane from s by accumulate_s

        :param float s: Accumulate s
        :returns: Interpolated index
        :rtype: InterpolatedIndex
        """

        if s <= 0.0:
            return InterpolatedIndex(0, 0.0)
        assert self._num_points > 0, "self._num_points is less than 0"
        if s >= self._length:
            return InterpolatedIndex(int(len(self._lane_segments) - 1), self._lane_segments[-1].Length())
        
        index: int = bisect_left(self._lane_accumulated_s, s)
        if index == len(self._lane_accumulated_s):
            return InterpolatedIndex(len(self._lane_segments) - 1, self._lane_segments[-1].Length())
        if index == 0:
            return InterpolatedIndex(index, s)
        else:
            return InterpolatedIndex(index, s - self._lane_accumulated_s[index - 1])

    def GetLaneSegments(self, start_s: float, end_s: float) -> List[LaneSegment]:
        """
        Get lane segments from start_s to end_s.

        :param float start_s: Start s
        :param float end_s: End s
        :returns: List of lane segments
        :rtype: List[LaneSegment]
        """

        lanes: List[LaneSegment] = []
        if start_s + kMathEpsilon > end_s:
            return lanes
        
        start_index = self.GetLaneIndexFromS(start_s)
        if start_index.offset + kMathEpsilon >= self._lane_segments[start_index.id].Length():
            start_index.id += 1
            start_index.offset = 0
        
        num_lanes: int = len(self._lane_segments)
        if start_index.id >= num_lanes:
            return lanes
        
        lanes.append(LaneSegment(self._lane_segments[start_index.id].lane, start_index.offset, self._lane_segments[start_index.id].Length()))
        
        end_index = self.GetLaneIndexFromS(end_s)
        for i in range(start_index.id, end_index.id):
            if i >= num_lanes:
                break
            lanes.append(self._lane_segments[i])
        
        if end_index.offset >= kMathEpsilon:
            lanes.append(LaneSegment(self._lane_segments[end_index.id].lane, 0, end_index.offset))
        
        return lanes

    def GetNearestPoint(self, point: Vec2d) -> Tuple[bool, float, float, float]:
        """
        Get nearest point on path. The conventional cpp code is using pointer to modify the value
        in python, we return the new value instead 

        :param Vec2d point: Point
        :returns: (bool, float accumulate_s, float lateral, float min_distance)
        :rtype: Tuple[bool, float, float, float]
        """

        tag, accumulate_s, lateral, min_distance = self.GetProjection(point)
        if not tag:
            return False, accumulate_s, lateral, min_distance

        if accumulate_s < 0.0:
            accumulate_s = 0.0
            min_distance = point.DistanceTo(self._path_points[0])
        elif accumulate_s > self._length:
            accumulate_s = self._length
            min_distance = point.DistanceTo(self._path_points[-1])

        return True, accumulate_s, lateral, min_distance

    def GetProjectionWithHueristicParams(self, point: Vec2d, hueristic_start_s: float, hueristic_end_s: float) -> Tuple[bool, float, float, float]:
        """
        Get projection with heuristic params

        :param Vec2d point: Point
        :param float hueristic_start_s: Hueristic start s
        :param float hueristic_end_s: Hueristic end s
        :returns: (bool, float accumulate_s, float lateral, float min_distance)
        :rtype: Tuple[bool, float, float, float]
        """
        
        if not self._segments:
            return False, None, None, 0.0
        assert self._num_points >= 2, "num_points is less than 2"
        min_distance: float = float("inf")

        start_interpolation_index: int = self.GetIndexFromS(hueristic_start_s).id
        end_interpolation_index: int = int(min(self._num_segments, self.GetIndexFromS(hueristic_end_s).id + 1))
        min_index: int = start_interpolation_index
        for i in range(start_interpolation_index, end_interpolation_index):
            distance: float = self._segments[i].DistanceSquareTo(point)
            if distance < min_distance:
                min_index = i
                min_distance = distance
        min_distance = math.sqrt(min_distance)
        nearest_seg = self._segments[min_index]
        prod = nearest_seg.ProductOntoUnit(point)
        proj = nearest_seg.ProjectOntoUnit(point)
        if min_index == 0:
            accumulate_s = min(proj, nearest_seg.length())
            if proj < 0:
                lateral = prod
            else:
                lateral = (1 if prod > 0.0 else -1) * min_distance
        elif min_index == self._num_segments - 1:
            accumulate_s = self._accumulated_s[min_index] + max(0.0, proj)
            if proj > 0:
                lateral = prod
            else:
                lateral = (1 if prod > 0.0 else -1) * min_distance
        else:
            accumulate_s = self._accumulated_s[min_index] + max(0.0, min(proj, nearest_seg.length()))
            lateral = (1 if prod > 0.0 else -1) * min_distance
        return True, accumulate_s, lateral, min_distance

    def GetProjection(self, point: Vec2d, heading: float = None) -> Tuple[bool, float, float, float]:
        """
        Get projection

        :param Vec2d point: Point
        :param float heading: Heading, if None, will not do heading check
        :returns: (bool, float accumulate_s, float lateral, float min_distance)
        :rtype: Tuple[bool, float, float, float]
        """

        if not self._segments:
            return False, 0.0, 0.0, 0.0
        if self._use_path_approximation:
            return self._approximation.GetProjection(point, heading=None)
        assert self._num_points >= 2, "num_points is less than 2"
        min_distance = float("inf")
        min_index: int = 0
        for i in range(self._num_points):
            if heading is not None and (abs(AngleDiff(self._segments[i].heading(), heading)) >= math.pi / 2.0):
                continue
            distance: float = self._segments[i].DistanceSquareTo(point)
            if distance < min_distance:
                min_index = i
                min_distance = distance
        min_distance = math.sqrt(min_distance)
        nearest_seg = self._segments[min_index]
        prod = nearest_seg.ProductOntoUnit(point)
        proj = nearest_seg.ProjectOntoUnit(point)
        if min_index == 0:
            accumulate_s = min(proj, nearest_seg.length())
            if proj < 0:
                lateral = prod
            else:
                lateral = (1 if prod > 0.0 else -1) * min_distance
        elif min_index == self._num_segments - 1:
            accumulate_s = self._accumulated_s[min_index] + max(0.0, proj)
            if proj > 0:
                lateral = prod
            else:
                lateral = (1 if prod > 0.0 else -1) * min_distance
        else:
            accumulate_s = self._accumulated_s[min_index] + max(0.0, min(proj, nearest_seg.length()))
            lateral = (1 if prod > 0.0 else -1) * min_distance
        return True, accumulate_s, lateral, min_distance

    def GetProjectionWithWarmStartS(self, point: Vec2d, accumulate_s: float) -> Tuple[bool, float, float]:
        """
        Get projection with warm start s

        :param Vec2d point: Point
        :param float accumulate_s: old accumulate s
        :returns: (bool, float accumulate_s, float lateral)
        :rtype: Tuple[bool, float, float]
        """
        
        if not self._segments:
            return False, None, None
        if accumulate_s < 0.0:
            accumulate_s = 0.0
        elif accumulate_s > self._length:
            accumulate_s = self._length
        
        assert self._num_points >= 2, "num_points is less than 2"
        warm_start_s = accumulate_s
        # Find the segment at the position of "accumulate_s".
        left_index: int = 0
        right_index: int = self._num_segments
        mid_index: int = 0
        # Find the segment with projection of the given point on it.
        while right_index > left_index + 1:
            mid_index = self.FindIndex(left_index, right_index, warm_start_s)
            segment = self._segments[mid_index]
            start_point = segment.start()
            delta_x: float = point.x - start_point.x
            delta_y: float = point.y - start_point.y
            unit_direction = segment.unit_direction
            proj: float = delta_x * unit_direction.x + delta_y * unit_direction.y
            accumulate_s = self._accumulated_s[mid_index] + proj
            lateral = unit_direction.x * delta_y - unit_direction.y * delta_x
            if proj > 0.0:
                if proj < segment.length():
                    return True, accumulate_s, lateral
                if mid_index == right_index:
                    accumulate_s = self._accumulated_s[mid_index]
                    return True, accumulate_s, lateral
                left_index = mid_index
            else:
                if mid_index == left_index:
                    accumulate_s = self._accumulated_s[mid_index]
                    return True, accumulate_s, lateral
                if abs(proj) < self._segments[mid_index - 1].length():
                    return True, accumulate_s, lateral
                right_index = mid_index - 1
            warm_start_s = segment.length() + proj
        return True, accumulate_s, lateral

    def GetHeadingAlongPath(self, point: Vec2d) -> Tuple[bool, float]:
        """
        Get heading along path

        :param Vec2d point: Point
        :returns: (bool, float heading)
        :rtype: Tuple[bool, float]
        """
        
        s: float = 0.0
        l: float = 0.0
        tag, s, l, _ = self.GetProjection(point)
        if tag:
            heading = self.GetSmoothPoint(s).heading
            return True, heading
        return False, None

    @property
    def num_points(self) -> int:
        """
        Return number of points

        :returns: Number of points
        :rtype: int
        """

        return self._num_points

    @property
    def num_segments(self) -> int:
        """
        Return number of segments

        :returns: Number of segments
        :rtype: int
        """

        return self._num_segments

    @property
    def path_points(self) -> List[MapPathPoint]:
        """
        Return path points

        :returns: Path points
        :rtype: List[MapPathPoint]
        """

        return self._path_points

    @property
    def lane_segments(self) -> List[LaneSegment]:
        """
        Return lane segments

        :returns: Lane segments
        :rtype: List[LaneSegment]
        """

        return self._lane_segments
    
    @property
    def lane_segments_to_next_point(self) -> List[LaneSegment]:
        """
        Return lane segments to next point

        :returns: Lane segments to next point
        :rtype: List[LaneSegment]
        """

        return self._lane_segments_to_next_point
    
    @property
    def unit_directions(self) -> List[Vec2d]:
        """
        Return unit directions

        :returns: Unit directions
        :rtype: List[Vec2d]
        """

        return self._unit_directions

    @property
    def accumulated_s(self) -> List[float]:
        """
        Return accumulated s

        :returns: Accumulated s
        :rtype: List[float]
        """

        return self._accumulated_s

    @property
    def segments(self) -> List[LineSegment2d]:
        """
        Return segments

        :returns: Segments
        :rtype: List[LineSegment2d]
        """

        return self._segments

    @property
    def approximation(self) -> PathApproximation:
        """
        Return approximation

        :returns: Approximation
        :rtype: PathApproximation
        """

        return self._approximation
    
    @property
    def length(self) -> float:
        """
        Return length

        :returns: Length
        :rtype: float
        """

        return self._length

    def NextLaneOverlap(self, s: float) -> PathOverlap:
        """
        Get next lane overlap

        :param float s: s
        :returns: Path overlap
        :rtype: PathOverlap
        """

        index = bisect_right([overlap.start_s for overlap in self._lane_overlaps], s)
        if index == len(self._lane_overlaps):
            return None
        else:
            return self._lane_overlaps[index]

    @property
    def lane_overlaps(self) -> List[PathOverlap]:
        """
        Return lane overlaps

        :returns: Lane overlaps
        :rtype: List[PathOverlap]
        """

        return self._lane_overlaps

    @property
    def signal_overlaps(self) -> List[PathOverlap]:
        """
        Return signal overlaps

        :returns: Signal overlaps
        :rtype: List[PathOverlap]
        """

        return self._signal_overlaps
    
    @property
    def yield_sign_overlaps(self) -> List[PathOverlap]:
        """
        Return yield sign overlaps

        :returns: Yield sign overlaps
        :rtype: List[PathOverlap]
        """

        return self._yield_sign_overlaps
    
    @property
    def stop_sign_overlaps(self) -> List[PathOverlap]:
        """
        Return stop sign overlaps

        :returns: Stop sign overlaps
        :rtype: List[PathOverlap]
        """

        return self._stop_sign_overlaps
    
    @property
    def crosswalk_overlaps(self) -> List[PathOverlap]:
        """
        Return crosswalk overlaps

        :returns: Crosswalk overlaps
        :rtype: List[PathOverlap]
        """

        return self._crosswalk_overlaps
    
    @property
    def junction_overlaps(self) -> List[PathOverlap]:
        """
        Return junction overlaps

        :returns: Junction overlaps
        :rtype: List[PathOverlap]
        """

        return self._junction_overlaps

    @property
    def pnc_junction_overlaps(self) -> List[PathOverlap]:
        """
        Return pnc junction overlaps

        :returns: PNC junction overlaps
        :rtype: List[PathOverlap]
        """

        return self._pnc_junction_overlaps

    @property
    def clear_area_overlaps(self) -> List[PathOverlap]:
        """
        Return clear area overlaps

        :returns: Clear area overlaps
        :rtype: List[PathOverlap]
        """

        return self._clear_area_overlaps

    @property
    def speed_bump_overlaps(self) -> List[PathOverlap]:
        """
        Return speed bump overlaps

        :returns: Speed bump overlaps
        :rtype: List[PathOverlap]
        """

        return self._speed_bump_overlaps

    @property
    def parking_space_overlaps(self) -> List[PathOverlap]:
        """
        Return parking space overlaps

        :returns: Parking space overlaps
        :rtype: List[PathOverlap]
        """

        return self._parking_space_overlaps
    
    @property
    def dead_end_overlaps(self) -> List[PathOverlap]:
        """
        Return dead end overlaps

        :returns: Dead end overlaps
        :rtype: List[PathOverlap]
        """

        return self._dead_end_overlaps

    def GetLaneLeftWidth(self, s: float) -> float:
        """
        Get lane left width

        :param float s: s
        :returns: Lane left width
        :rtype: float
        """

        return self.GetSample(self._lane_left_width, s)

    def GetLaneRightWidth(self, s: float) -> float:
        """
        Get lane right width

        :param float s: s
        :returns: Lane right width
        :rtype: float
        """
        
        return self.GetSample(self._lane_right_width, s)

    def GetLaneWidth(self, s: float) -> Tuple[bool, float, float]:
        """
        Get lane width

        :param float s: s
        :returns: (bool, float lane_left_width, float lane_right_width)
        :rtype: Tuple[bool, float, float]
        """
        
        if s < 0.0 or s > self._length:
            return False, 0.0, 0.0
        lane_left_width = self.GetLaneLeftWidth(s)
        lane_right_width = self.GetLaneRightWidth(s)
        return True, lane_left_width, lane_right_width

    def GetRoadLeftWidth(self, s: float) -> float:
        """
        Get road left width

        :param float s: s
        :returns: Road left width
        :rtype: float
        """

        return self.GetSample(self._road_left_width, s)

    def GetRoadRightWidth(self, s: float) -> float:
        """
        Get road right width

        :param float s: s
        :returns: Road right width
        :rtype: float
        """

        return self.GetSample(self._road_right_width, s)

    def GetRoadWidth(self, s: float) -> Tuple[bool, float, float]:
        """
        Get road width

        :param float s: s
        :returns: (bool, float road_left_width, float road_right_width)
        :rtype: Tuple[bool, float, float]
        """

        if s < 0.0 or s > self._length:
            return False, 0.0, 0.0
        road_left_width = self.GetRoadLeftWidth(s)
        road_right_width = self.GetRoadRightWidth(s)
        return True, road_left_width, road_right_width
    
    def IsOnPath(self, point: Vec2d) -> bool:
        """
        Check if on path

        :param Vec2d point: Point
        :returns: True if on path, False otherwise
        :rtype: bool
        """

        accumulate_s: float = 0.0
        lateral: float = 0.0
        tag, accumulate_s, lateral, _ = self.GetProjection(point)
        if not tag:
            return False
        tag, lane_left_width, lane_right_width = self.GetLaneWidth(accumulate_s)
        if not tag:
            return False
        if lateral < lane_left_width and lateral > -lane_right_width:
            return True
        return False

    def OverlapWith(self, box: Box2d, width: float) -> bool:
        """
        Check overlap with box

        :param Box2d box: Box
        :param float width: Width
        :returns: True if overlap, False otherwise
        :rtype: bool
        """

        if self._use_path_approximation:
            return self._approximation.OverlapWith(box, width)
        center: Vec2d = box.center
        radius_sqr: float = (box.diagonal / 2.0 + width) ** 2 + kMathEpsilon
        for segment in self._segments:
            if segment.DistanceSquareTo(center) > radius_sqr:
                continue
            if box.DistanceTo(segment) <= width + kMathEpsilon:
                return True
        return False

    def __str__(self) -> str:
        """
        Return string representation of Path

        :returns: String representation of Path
        :rtype: str
        """

        return f"num_points: {self._num_points}, points: {self._path_points}, numlane_segments = {len(self._lane_segments)}, lane_segments: {self._lane_segments}"

    def GetSample(self, samples: List[float], s: float) -> float:
        """
        Get sample

        :param List[float] samples: Samples
        :param float s: s
        :returns: Sample
        :rtype: float
        """

        if not samples:
            return 0.0
        if s <= 0.0:
            return samples[0]
        idx: int = s // kSampleDistance
        if idx >= self._num_sample_points - 1:
            return samples[-1]
        ratio: float = s - idx * kSampleDistance / kSampleDistance
        return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio

    def GetAllOverlaps(self, type: str) -> List[PathOverlap]:
        """
        Get all overlaps by type

        NOTE: This part has been modified, the cpp source code using callback function
        Here we manipulate the lane structure as dict()

        :param str type: Overlap Object Type
        :returns: List of path overlaps
        :rtype: List[PathOverlap]
        """
        overlaps: List[PathOverlap] = []

        overlaps_by_id: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
        s: float = 0.0
        for lane_segment in self._lane_segments:
            if not lane_segment.lane:
                continue
            for overlap in lane_segment.lane.get('overlaps'):
                overlap_info = overlap.get('overlap_info')
                if not overlap_info:
                    continue
                lane_overlap_info = overlap_info.get('lane_overlap_info')
                if lane_overlap_info.get('start_s') <= lane_segment.end_s and lane_overlap_info.get('end_s') >= lane_segment.start_s:
                    ref_s = s - lane_segment.start_s
                    adjusted_start_s = max(lane_overlap_info.get('start_s'), lane_segment.start_s) + ref_s
                    adjusted_end_s = min(lane_overlap_info.get('end_s'), lane_segment.end_s) + ref_s
                    for object in overlap.get('object'):
                        if object.get('id') != lane_segment.lane.get('id') and object.get('type') == type:
                            overlaps_by_id[object.get('id')].append((adjusted_start_s, adjusted_end_s))
            s += lane_segment.end_s - lane_segment.start_s

        for object_id, segments in overlaps_by_id.items():
            segments.sort()

            kMinOverlapDistanceGap: float = 1.5   # in meters
            for segment in segments:
                if overlaps and overlaps[-1].object_id == object_id and segment[0] - overlaps[-1].end_s <= kMinOverlapDistanceGap:
                    overlaps[-1].end_s = max(overlaps[-1].end_s, segment[1])
                else:
                    overlaps.append(PathOverlap(object_id, segment[0], segment[1]))

        overlaps.sort(key=lambda overlap: overlap.start_s)
        return overlaps

    def FindIndex(self, left_index: int, right_index: int, target_s: float) -> int:
        """
        Find the segment index with binary search

        :param int left_index: Left index
        :param int right_index: Right index
        :param float target_s: Target s
        :returns: mid_index
        :rtype: int
        """

        while right_index > left_index + 1:
            mid_index = (left_index + right_index) >> 1
            if self._accumulated_s[mid_index] < target_s:
                left_index = mid_index
                continue
            if self._accumulated_s[mid_index - 1] > target_s:
                right_index = mid_index - 1
                continue
            return mid_index
