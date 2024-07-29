from dataclasses import dataclass, field
from typing import List, Optional, Union
from protoclass.PointENU import PointENU
from enum import Enum

@dataclass
class LineSegment:
    """
    Straight line segment.
    """

    point: List[PointENU] = field(default_factory=list)

@dataclass
class CurveSegment:
    """
    Generalization of a line.
    """

    curve_type: Optional[Union[LineSegment]] = None  # oneof curve_type
    s: Optional[float] = None                        # start position (s-coordinate)
    start_position: Optional[PointENU] = None
    heading: Optional[float] = None                  # start orientation
    length: Optional[float] = None

@dataclass
class Curve:
    """
    An object similar to a line but that need not be straight.
    """

    segment: List[CurveSegment] = field(default_factory=list)

@dataclass
class LaneBoundaryType:
    """
    LaneBoundaryType class, oriented from protobuf message
    """

    class LaneBoundaryTypeEnum(Enum):
        
        UNKNOWN = 0
        DOTTED_YELLOW = 1
        DOTTED_WHITE = 2
        SOLID_YELLOW = 3
        SOLID_WHITE = 4
        DOUBLE_YELLOW = 5
        CURB = 6

    s: Optional[float] = None                        # Offset relative to the starting point of boundary
    types: List[LaneBoundaryTypeEnum] = field(default_factory=list)      # support multiple types

@dataclass
class LaneBoundary:
    """
    Lane boundary class, oriented from protobuf message
    """

    curve: Optional[Curve] = None
    length: Optional[float] = None
    virtual: Optional[bool] = None                   # indicate whether the lane boundary exists in real world
    boundary_type: List[LaneBoundaryType] = field(default_factory=list)  # in ascending order of s

@dataclass
class LaneSampleAssociation:
    """
    Association between central point to closest boundary.
    """

    s: Optional[float] = None
    width: Optional[float] = None

@dataclass
class Lane:
    """
    Lane class, oriented from protobuf message
    
    A lane is part of a roadway, that is designated for use by a single line of
    vehicles.
    Most public roads (include highways) have more than two lanes.
    """

    @dataclass
    class Id:
        """
        Global unique ids for all objects (include lanes, junctions, overlaps, etc).
        """

        id: Optional[str] = None

    id: Optional[Id] = None

    # Central lane as reference trajectory, not necessary to be the geometry
    # central.
    central_curve: Optional[Curve] = None

    # Lane boundary curve.
    left_boundary: Optional[LaneBoundary] = None
    right_boundary: Optional[LaneBoundary] = None

    # in meters.
    length: Optional[float] = None

    # Speed limit of the lane, in meters per second.
    speed_limit: Optional[float] = None

    overlap_id: List[Id] = field(default_factory=list)

    # All lanes can be driving into (or from).
    predecessor_id: List[Id] = field(default_factory=list)
    successor_id: List[Id] = field(default_factory=list)

    # Neighbor lanes on the same direction.
    left_neighbor_forward_lane_id: List[Id] = field(default_factory=list)
    right_neighbor_forward_lane_id: List[Id] = field(default_factory=list)

    class LaneType(Enum):

        NONE = 1
        CITY_DRIVING = 2
        BIKING = 3
        SIDEWALK = 4
        PARKING = 5
        SHOULDER = 6

    type: Optional[LaneType] = None

    class LaneTurn(Enum):

        NO_TURN = 1
        LEFT_TURN = 2
        RIGHT_TURN = 3
        U_TURN = 4

    turn: Optional[LaneTurn] = None

    left_neighbor_reverse_lane_id: List[Id] = field(default_factory=list)
    right_neighbor_reverse_lane_id: List[Id] = field(default_factory=list)

    junction_id: Optional[Id] = None

    # Association between central point to closest boundary.
    left_sample: List[LaneSampleAssociation] = field(default_factory=list)
    right_sample: List[LaneSampleAssociation] = field(default_factory=list)

    class LaneDirection(Enum):

        FORWARD = 1
        BACKWARD = 2
        BIDIRECTION = 3

    direction: Optional[LaneDirection] = None

    # Association between central point to closest road boundary.
    left_road_sample: List[LaneSampleAssociation] = field(default_factory=list)
    right_road_sample: List[LaneSampleAssociation] = field(default_factory=list)

    self_reverse_lane_id: List[Id] = field(default_factory=list)
