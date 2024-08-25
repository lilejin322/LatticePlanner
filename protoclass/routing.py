from dataclasses import dataclass, field
from typing import List, Optional
from protoclass.Header import Header, StatusPb
from protoclass.LaneWaypoint import LaneWaypoint
from protoclass.PlanningStatus import LaneSegment
from protoclass.DecisionResult import ChangeLaneType

@dataclass
class RoutingRequest:
    """
    RoutingRequest class, oriented from protobuf message
    """

    header: Optional[Header] = None
    waypoint: List[LaneWaypoint] = field(default_factory=list)
    """
    at least two points. The first is start point, the end is final point.
    The routing must go through each point in waypoint.
    """
    blacklisted_lane: List[LaneSegment] = field(default_factory=list)
    blacklisted_road: List[str] = field(default_factory=list)
    broadcast: bool = True
    is_start_pose_set: bool = False
    """ If the start pose is set as the first point of "way_point"."""

@dataclass
class Passage:
    """
    Passage class, oriented from protobuf message
    """

    segment: List[LaneSegment] = field(default_factory=list)
    can_exit: Optional[bool] = None
    change_lane_type: ChangeLaneType = ChangeLaneType.FORWARD

@dataclass
class RoadSegment:
    """
    RoadSegment class, oriented from protobuf message
    """

    id: Optional[str] = None
    passage: List[Passage] = field(default_factory=list)

@dataclass
class Measurement:
    """
    Measurement class, oriented from protobuf message
    """

    distance: Optional[float] = None

@dataclass
class RoutingResponse:
    """
    RoutingResponse class, oriented from protobuf message
    """

    header: Optional[Header] = None
    road: List[RoadSegment] = field(default_factory=list)
    measurement: Optional[Measurement] = None
    routing_request: Optional[RoutingRequest] = None
    map_version: Optional[bytes] = None
    """the map version which is used to build road graph"""
    status: Optional[StatusPb] = None
