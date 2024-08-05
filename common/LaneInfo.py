from typing import List, Tuple
from protoclass.PointENU import PointENU
from common.Vec2d import Vec2d
from common.LineSegment2d import LineSegment2d
from common.Box2d import Box2d

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
        self._segment_box_list = []
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
        
        raise NotImplementedError

    def Curvature(self, s: float) -> float:

        raise NotImplementedError

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

        raise NotImplementedError

    def GetEffectiveWidth(self, s: float) -> float:
        """
        Get the effective width of the lane
        """

        raise NotImplementedError

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

        raise NotImplementedError

    def IsOnLane(self, *args) -> bool:
        
        if isinstance(args[0], Vec2d):
            """

            :param Vec2d point: the point to be checked
            """

            point: Vec2d = args[0]
            raise NotImplementedError

        elif isinstance(args[0], Box2d):
            """

            :param Box2d box: the box to be checked
            """

            box: Box2d = args[0]
            raise NotImplementedError

    def GetSmoothPoint(self, s: float) -> PointENU:
        """
        Get the smooth point of the lane at s

        :param float s: the s value
        :returns: the smooth point of the lane at s
        :rtype: PointENU
        """

        raise NotImplementedError

    def DistanceTo(self, *args) -> float:
        """
        Get the distance to the lane
        """

        raise NotImplementedError

    def GetNearestPoint(self, point: Vec2d) -> Tuple[PointENU, float]:
        """
        Get the nearest point on the lane

        :param Vec2d point: the point to be checked
        :returns: the nearest point on the lane and the distance
        :rtype: Tuple[PointENU, float]
        """

        raise NotImplementedError

    def GetProjection(self, point: Vec2d, heading: float=None) -> Tuple[bool, float, float]:
        """
        Get the projection of the point on the lane

        :param Vec2d point: the point to be projected
        :param float heading: the heading of the point
        :returns: whether the projection is successful, float accumulated_s, float lateral
        :rtype: Tuple[bool, float, float]
        """
        
        raise NotImplementedError

    def Init(self) -> None:
       """
       Init method
       """

       raise NotImplementedError

    def PostProcess(self, map_instance: HDMapImpl):
        """
        Post process method
        """

        raise NotImplementedError

    def UpdateOverlaps(self, map_instance: HDMapImpl):
        """
        Update overlaps method
        """
        
        raise NotImplementedError

    def GetWidthFromSample(self, samples: List[Tuple[float, float]], s: float) -> float:
        """
        Get the width from the sample
        """
        
        raise NotImplementedError

    def CreateKDTree(self) -> None:
        """
        Create the kd tree
        """

        raise NotImplementedError

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
