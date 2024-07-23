from typing import List, Tuple, Any
from dataclasses import dataclass
import math
from common.MapPathPoint import MapPathPoint
from common.ReferenceLine import LaneSegment, InterpolatedIndex
from common.Vec2d import Vec2d
from common.LineSegment2d import LineSegment2d
from common.Box2d import Box2d

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
        

    def InitPointIndex(self) -> None:
        """
        Init point index method
        """
        
        raise NotImplementedError

    def InitOverlaps(self) -> None:
        """
        Init overlaps method
        """

        raise NotImplementedError

    def GetSmoothPoint(self, param: Any) -> MapPathPoint:

        if isinstance(param, InterpolatedIndex):
            """
            Return smooth coordinate by interpolated index.

            :param InterpolatedIndex index: Interpolated index
            :returns: Smooth coordinate
            :rtype: MapPathPoint
            """

            index: InterpolatedIndex = param
            raise NotImplementedError
        
        elif isinstance(param, float):
            """
            Return smooth coordinate by accumulate_s.

            :param float s: Accumulate s
            :returns: Smooth coordinate
            :rtype: MapPathPoint
            """

            s: float = param
            raise NotImplementedError

        else:

            raise ValueError("Invalid parameter type")

    def GetSFromIndex(self, index: InterpolatedIndex) -> float:
        """
        Compute accumulate s value of the index.

        :param InterpolatedIndex index: Interpolated index
        :returns: Accumulate s value
        :rtype: float
        """

        raise NotImplementedError

    def GetIndexFromS(self, s: float) -> InterpolatedIndex:
        """
        Compute interpolated index by accumulate_s.

        :param float s: Accumulate s
        :returns: Interpolated index
        :rtype: InterpolatedIndex
        """
        
        raise NotImplementedError

    def GetLaneIndexFromS(self, s: float) -> InterpolatedIndex:
        """
        get the index of the lane from s by accumulate_s

        :param float s: Accumulate s
        :returns: Interpolated index
        :rtype: InterpolatedIndex
        """

        raise NotImplementedError

    def GetLaneSegments(self, start_s: float, end_s: float) -> List[LaneSegment]:
        """
        Get lane segments from start_s to end_s.

        :param float start_s: Start s
        :param float end_s: End s
        :returns: List of lane segments
        :rtype: List[LaneSegment]
        """

        raise NotImplementedError

    def GetNearestPoint(self, point: Vec2d) -> Tuple[bool, float, float, float]:
        """
        Get nearest point on path.

        :param Vec2d point: Point
        :returns: (bool, float accumulate_s, float lateral, float distance)
        :rtype: Tuple[bool, float, float, float]
        """

        raise NotImplementedError

    def GetProjectionWithHueristicParams(self, point: Vec2d, hueristic_start_s: float, hueristic_end_s: float) -> Tuple[bool, float, float, float]:
        """
        Get projection with heuristic params

        :param Vec2d point: Point
        :param float hueristic_start_s: Hueristic start s
        :param float hueristic_end_s: Hueristic end s
        :returns: (bool, float accumulate_s, float lateral, float min_distance)
        :rtype: Tuple[bool, float, float, float]
        """
        
        raise NotImplementedError

    def GetProjection(self, point: Vec2d, heading: float = None) -> Tuple[bool, float, float, float]:
        """
        Get projection

        :param Vec2d point: Point
        :param float heading: Heading
        :returns: (bool, float accumulate_s, float lateral, float distance)
        :rtype: Tuple[bool, float, float, float]
        """
        
        raise NotImplementedError

    def GetProjectionWithWarmStartS(self, point: Vec2d) -> Tuple[bool, float, float]:
        """
        Get projection with warm start s

        :param Vec2d point: Point
        :returns: (bool, float accumulate_s, float lateral)
        :rtype: Tuple[bool, float, float]
        """
        
        raise NotImplementedError

    def GetHeadingAlongPath(self, point: Vec2d) -> Tuple[bool, float]:
        """
        Get heading along path

        :param Vec2d point: Point
        :returns: (bool, float heading)
        :rtype: Tuple[bool, float]
        """
        
        raise NotImplementedError

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

    def NextLaneOverlap(s: float) -> PathOverlap:
        """
        Get next lane overlap

        :param float s: s
        :returns: Path overlap
        :rtype: PathOverlap
        """

        raise NotImplementedError

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

        raise NotImplementedError

    def GetLaneRightWidth(self, s: float) -> float:
        """
        Get lane right width

        :param float s: s
        :returns: Lane right width
        :rtype: float
        """
        
        raise NotImplementedError

    def GetLaneWidth(self, s: float) -> Tuple[bool, float, float]:
        """
        Get lane width

        :param float s: s
        :returns: (bool, float lane_left_width, float lane_right_width)
        :rtype: Tuple[bool, float, float]
        """
        
        raise NotImplementedError

    def GetRoadLeftWidth(self, s: float) -> float:
        """
        Get road left width

        :param float s: s
        :returns: Road left width
        :rtype: float
        """

        raise NotImplementedError

    def GetRoadRightWidth(self, s: float) -> float:
        """
        Get road right width

        :param float s: s
        :returns: Road right width
        :rtype: float
        """

        raise NotImplementedError

    def GetRoadWidth(self, s: float) -> Tuple[bool, float, float]:
        """
        Get road width

        :param float s: s
        :returns: (bool, float road_left_width, float road_right_width)
        :rtype: Tuple[bool, float, float]
        """

        raise NotImplementedError
    
    def IsOnPath(self, point: Vec2d) -> bool:
        """
        Check if on path

        :param Vec2d point: Point
        :returns: True if on path, False otherwise
        :rtype: bool
        """

        raise NotImplementedError

    def OverlapWith(self, box: Box2d, width: float) -> bool:
        """
        Check overlap with box

        :param Box2d box: Box
        :param float width: Width
        :returns: True if overlap, False otherwise
        :rtype: bool
        """
        
        raise NotImplementedError

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

        raise NotImplementedError


    def GetAllOverlaps(self, get_overlaps_from_lane) -> List[PathOverlap]:
        """
        Get all overlaps

        # NOTE: This part is a bit difficult to tackle
        :param ***:
        :returns: List of path overlaps
        :rtype: List[PathOverlap]
        """

        raise NotImplementedError

    def FindIndex(self, left_index: int, right_index: int, target_s: float) -> int:
        """
        Find index

        :param int left_index: Left index
        :param int right_index: Right index
        :param float target_s: Target s
        :returns: mid_index
        :rtype: int
        """
        
        raise NotImplementedError
