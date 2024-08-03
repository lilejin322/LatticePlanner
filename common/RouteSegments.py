from common.ReferenceLine import LaneSegment
from protoclass.DecisionResult import ChangeLaneType
from typing import Tuple, Union, Any
from protoclass.SLBoundary import SLPoint
from common.MapPathPoint import LaneWaypoint
from protoclass.PointENU import PointENU
from common.Vec2d import Vec2d
from copy import deepcopy
from logging import Logger
import math
from common.Path import AngleDiff

logger = Logger("RouteSegments")

kSegmentationEpsilon: float = 0.5   # Minimum error in lane segmentation.

def DistanceXY(u: Any, v: Any) -> float:
    """
    Distance XY

    :param Any u: U
    :param Any v: V
    :returns: Distance XY
    :rtype: float
    """

    return math.hypot(u.x - v.x, u.y - v.y)

class RouteSegments(list):
    """
    RouteSegments class, inherit from list

    This class is a representation of the Passage type in routing.proto.
    It is extended from a passage region, but keeps some properties of the
    passage, such as the last end LaneWaypoint of the original passage region
    (route_end_waypoint), whether the passage can lead to another passage in
    routing (can_exit_).
    This class contains the original data that can be used to generate
    hdmap::Path.
    """

    def __init__(self):
        """
        Constructor
        """

        self._route_end_waypoint: LaneWaypoint = None
        # whether this segment can lead to another passage region in routing
        self._can_exit: bool = False
        # Indicates whether the vehicle is on current RouteSegment.
        self._is_on_segment: bool = False
        # Indicates whether current routeSegment is the neighbor of vehicle routeSegment.
        self._is_neighbor: bool = False
        self._next_action: ChangeLaneType = ChangeLaneType.FORWARD
        self._previous_action: ChangeLaneType = ChangeLaneType.FORWARD
        self._id: str = ""
        # Whether the vehicle should stop for destination. In a routing that has
        # loops, the adc may pass by destination many times, but it only need to stop
        # for destination  in the last loop.
        self._stop_for_destination: bool = False

    @property
    def NextAction(self) -> ChangeLaneType:
        """
        Get the next change lane action need to take by the vehicle, if the vehicle
        is on this RouteSegments.
        --- If the vehicle does not need to change lane, then change_lane_type ==
        routing::FORWARD;
        --- If the vehicle need to change to left lane according to routing, then
        change_lane_type_ == routing::LEFT;
        --- If the vehicle need to change to right lane according to routing, then
        change_lane_type_ == routing::RIGHT;
        """

        return self._next_action
    
    def SetNextAction(self, action: ChangeLaneType) -> None:
        """
        Set the next action

        :param ChangeLaneType action: action to set
        """

        self._next_action = action
    
    @property
    def PreviousAction(self) -> ChangeLaneType:
        """
        Get the previous change lane action need to take by the vehicle to reach
        current segment, if the vehicle is not on this RouteSegments.
        If the vehicle is already on this segment, or does not need to change lane
        to reach this segment, then change_lane_type = routing::FORWARD;
        If the vehicle need to change to left to reach this segment, then
        change_lane_type_ =  routing::LEFT;
        If the vehicle need to change to right to reach this segment, then
        change_lane_type_ = routing::RIGHT;
        """

        return self._previous_action
    
    def SetPreviousAction(self, action: ChangeLaneType) -> None:
        """
        Set the previous action

        :param ChangeLaneType action: action to set
        """

        self._previous_action = action

    @property
    def CanExit(self) -> bool:
        """
        Whether the passage region that generate this route segment can lead to
        another passage region in route.

        :returns: whether can exit
        :rtype: bool
        """

        return self._can_exit
    
    def SetCanExit(self, can_exit: bool) -> None:
        """
        Set whether can exit

        :param bool can_exit: whether can exit
        """

        self._can_exit = can_exit

    def GetProjection(self, *args) -> Tuple[bool, SLPoint, LaneWaypoint]:
        """
        Project a point to this route segment.

        waypoint: return the LaneWaypoint, which has lane and lane_s on the
        route segment.
        sl_point.s: return the longitudinal s relative to the route segment.
        sl_point.l: return the lateral distance relative to the route segment.
        """
        
        if len(args) == 1 and isinstance(args[0], PointENU):
            """
            :param PointENU point_enu: a map point, or point, which is a Vec2d point
            :returns: (false if error happened or projected outside of the lane segments,
                       SLPoint sl_point, LaneWaypoint waypoint)
            :rtype: Tuple[bool, SLPoint, LaneWaypoint]
            """

            point_enu: PointENU = args[0]
            return self.GetProjection(Vec2d(point_enu.x, point_enu.y))

        elif len(args) == 1 and isinstance(args[0], Vec2d):
            """
            :param Vec2d point: a Vec2d point
            :returns: (false if error happened or projected outside of the lane segments,
                       SLPoint sl_point, LaneWaypoint waypoint)
            :rtype: Tuple[bool, SLPoint, LaneWaypoint]
            """

            point: Vec2d = args[0]
            heading = None
            return self.GetProjection(point, heading)

        elif len(args) == 2 and isinstance(args[0], Vec2d) and isinstance(args[1], (float, int)):
            """
            :param Vec2d point: a Vec2d point
            :param float heading: heading of the point
            :returns: (false if error happened or projected outside of the lane segments,
                       SLPoint sl_point, LaneWaypoint waypoint)
            :rtype: Tuple[bool, SLPoint, LaneWaypoint]
            """

            point: Vec2d = args[0]
            heading: Union[float, None] = args[1]
            min_l: float = float("inf")
            accumulated_s: float = 0.0
            has_projection: bool = False
            sl_point: SLPoint = SLPoint()
            waypoint: LaneWaypoint = LaneWaypoint()
            for segment in self:
                tag, lane_s, lane_l = segment.lane.GetProjection(point, heading)
                if not tag:
                    logger.error(f"Failed to get projection from point {point} on lane {segment.lane.id}")
                    return False, None, None
                if lane_s < segment.start_s - kSegmentationEpsilon or lane_s > segment.end_s + kSegmentationEpsilon:
                    continue
                if abs(lane_l) < min_l:
                    has_projection = True
                    lane_s = max(lane_s, segment.start_s)
                    lane_s = min(lane_s, segment.end_s)
                    min_l = abs(lane_l)
                    sl_point.l=lane_l
                    sl_point.s=lane_s - segment.start_s + accumulated_s
                    waypoint.lane = segment.lane
                    waypoint.s = lane_s
                accumulated_s += segment.end_s - segment.start_s

            return has_projection, sl_point, waypoint

        else:

            raise ValueError("Invalid arguments for GetProjection()")

    def GetWaypoint(self, s: float) -> Tuple[bool, LaneWaypoint]:
        """
        Get the LaneWaypoint on the route segment by the longitudinal s.

        :param float s: longitudinal s
        :returns: (bool tag, LaneWaypoint waypoint)
        :rtype: Tuple[bool, LaneWaypoint]
        """

        accumulated_s: float = 0.0
        has_projection: bool = False
        waypoint: LaneWaypoint = LaneWaypoint()
        for segment in self:
            if accumulated_s - kSegmentationEpsilon < s < accumulated_s + segment.end_s - segment.start_s + kSegmentationEpsilon:
                waypoint.lane = segment.lane
                waypoint.s = s - accumulated_s + segment.start_s
                if waypoint.s < segment.start_s:
                    waypoint.s = segment.start_s
                elif waypoint.s > segment.end_s:
                    waypoint.s = segment.end_s
                has_projection = True
                break

            accumulated_s += segment.end_s - segment.start_s

        return has_projection, waypoint

    def CanDriveFrom(self, waypoint: LaneWaypoint) -> bool:
        """
        Check whether the map allows a vehicle can reach current
        RouteSegment from a point on a lane (LaneWaypoint).

        :param LaneWaypoint waypoint: the start waypoint
        :returns: true if the map allows a vehicle to drive from waypoint to
        current RouteSegment. Otherwise false.
        :rtype: bool
        """

        point = waypoint.lane.GetSmoothPoint(waypoint.s)

        # 0 if waypoint is on segment, ok
        if self.IsWaypointOnSegment(waypoint):
            return True
        
        # 1. should have valid projection
        has_projection, route_sl, segment_waypoint = self.GetProjection(point)
        if not has_projection:
            logger.error(f"No projection from waypoint: {waypoint}")
            return False
        kMaxLaneWidth: float = 10.0
        if abs(route_sl.l) > 2 * kMaxLaneWidth:
            return False
        
        # 2. heading should be the same
        waypoint_heading = waypoint.lane.Heading(waypoint.s)
        segment_heading = segment_waypoint.lane.Heading(segment_waypoint.s)
        heading_diff = AngleDiff(waypoint_heading, segment_heading)
        if abs(heading_diff) > math.pi / 2.0:
            logger.debug(f"Angle diff too large: {heading_diff}")
            return False

        # 3. the waypoint and the projected lane should not be separated apart.
        waypoint_left_width, waypoint_right_width = waypoint.lane.GetWidth(waypoint.s)
        segment_left_width, segment_right_width = segment_waypoint.lane.GetWidth(segment_waypoint.s)
        segment_projected_point = segment_waypoint.lane.GetSmoothPoint(segment_waypoint.s)
        dist: float = DistanceXY(point, segment_projected_point)
        kLaneSeparationDistance: float = 0.3
        if route_sl.l < 0:
            # waypoint at right side
            if dist > waypoint_left_width + segment_right_width + kLaneSeparationDistance:
                logger.error(f"waypoint is too far to reach: {dist}")
                return False
        else:
            # waypoint at left side
            if dist > waypoint_right_width + segment_left_width + kLaneSeparationDistance:
                logger.error(f"waypoint is too far to reach: {dist}")
                return False

        return True

    @property
    def RouteEndWaypoint(self) -> LaneWaypoint:
        """
        This is the point that is the end of the original passage in routing.
        It is used to check if the vehicle is out of current routing.
        The LaneWaypoint.lane is nullptr if the end of the passage is not on the
        RouteSegment.

        :returns: the route end way point
        :rtype: LaneWaypoint
        """

        return self._route_end_waypoint

    def SetRouteEndWaypoint(self, waypoint: LaneWaypoint) -> None:
        """
        Set the route end waypoint

        :param LaneWaypoint waypoint: the route end waypoint
        """

        self._route_end_waypoint = waypoint

    def Stitch(self, other: 'RouteSegments') -> bool:
        """
        Stitch current route segments with the other route segment.
        Example 1
        this:   |--------A-----x-----B------|
        other:                 |-----B------x--------C-------|
        Result: |--------A-----x-----B------x--------C-------|
        In the above example, A-B is current route segments, and B-C is the other
        route segments. We update current route segments to A-B-C.
        Example 2
        this:                  |-----A------x--------B-------|
        other:  |--------C-----x-----A------|
        Result: |--------C-----x-----A------x--------B-------|
        In the above example, A-B is current route segments, and C-A is the other
        route segments. We update current route segments to C-A-B

        :returns: false if these two reference line cannot be stitched
        :rtype: bool
        """

        first_waypoint: LaneWaypoint = self.FirstWayPoint()
        has_overlap: bool = self.IsWaypointOnSegment(other.FirstWayPoint)
        if other.IsWaypointOnSegment(first_waypoint):
            for segment in other:
                if self.WithinLaneSegment(segment,  first_waypoint):
                    break
            if segment is not None:
                self[0].start_s = min(self[0].start_s, segment.start_s)
                self[0].end_s = max(self[0].end_s, segment.end_s)
                self[:0] = other[:other.index(segment)]
                has_overlap = True

        last_waypoint: LaneWaypoint = self.LastWayPoint()
        if other.IsWaypointOnSegment(last_waypoint):
            for segment in reversed(other):
                if self.WithinLaneSegment(segment, last_waypoint):
                    break
            if segment is not None:
                self[-1].start_s = min(self[-1].start_s, segment.start_s)
                self[-1].end_s = max(self[-1].end_s, segment.end_s)
                self.extend(other[other.index(segment) + 1:])
                has_overlap = True

        return has_overlap

    def Shrink(self,*args) -> bool:
        """
        Shrink method
        """

        if len(args) == 3 and isinstance(args[0], Vec2d):
            """
            :param Vec2d point: point to shrink
            :param float look_backward: look backward distance
            :param float look_forward: look forward distance
            """

            point: Vec2d = args[0]
            look_backward: float = args[1]
            look_forward: float = args[2]

            tag, sl_point, waypoint = self.GetProjection(point)
            if not tag:
                logger.error(f"failed to project {point} to segment")
                return False
            return self.Shrink(sl_point.s, look_backward, look_forward)

        if len(args) == 3 and isinstance(args[0], (float, int)):
            """
            :param float s: s to shrink
            :param float look_backward: look backward distance
            :param float look_forward: look forward distance
            """

            s: float = args[0]
            look_backward: float = args[1]
            look_forward: float = args[2]

            tag, waypoint = self.GetWaypoint(s)
            if not tag:
                return False
            return self.Shrink(s, waypoint, look_backward, look_forward)

        if len(args) == 4:
            """
            :param float s: s to shrink
            :param LaneWaypoint waypoint: waypoint to shrink
            :param float look_backward: look backward distance
            :param float look_forward: look forward distance
            """

            s: float = args[0]
            waypoint: LaneWaypoint = args[1]
            look_backward: float = args[2]
            look_forward: float = args[3]

            acc_s: float = 0.0
            index: int = 0
            while index != len(self) and acc_s + self[index].Length() < s - look_backward:
                acc_s += self[index].Length()
                index += 1
            if index == len(self):
                return True
            self[index].start_s = max(self[index].start_s, s - look_backward - acc_s + self[index].start_s)
            if self[index].Length < kSegmentationEpsilon:
                index += 1
            del self[:index]
            
            index: int = 0
            acc_s: float = 0.0
            while index != len(self) and self.WithinLaneSegment(self[index], waypoint):
                index += 1
            if index == len(self):
                return True
            acc_s = self[index].end_s - waypoint.s
            if acc_s >= look_forward:
                self[index].end_s = waypoint.s + look_forward
                index += 1
                del self[index:]
                return True
            index += 1
            while index != len(self) and acc_s + self[index].Length() < look_forward:
                acc_s += self[index].Length()
                index += 1
            if index == len(self):
                return True
            self[index].end_s = min(self[index].end_s, look_forward - acc_s + self[index].start_s)
            del self[index + 1:]
            return True

        else:

            raise ValueError("Invalid arguments for Shrink()")

    @property
    def IsOnSegment(self) -> bool:
        """
        Check if the vehicle is on this RouteSegments.

        :returns: whether the vehicle is on this RouteSegments
        :rtype: bool
        """

        return self._is_on_segment
    
    def SetIsOnSegment(self, on_segment: bool) -> None:
        """
        Set whether the vehicle is on this RouteSegments

        :param bool on_segment: whether the vehicle is on this RouteSegments
        """
        
        self._is_on_segment = on_segment

    @property
    def IsNeighborSegment(self) -> bool:
        """
        Check if the vehicle is on the neighbor segment of this RouteSegments.

        :returns: whether the vehicle is on the neighbor segment of this RouteSegments
        :rtype: bool
        """

        return self._is_neighbor
    
    def SetIsNeighborSegment(self, is_neighbor: bool) -> None:
        """
        Set whether the vehicle is on the neighbor segment of this RouteSegments

        :param bool is_neighbor: whether the vehicle is on the neighbor segment of this RouteSegments
        """

        self._is_neighbor = is_neighbor
    
    def SetId(self, id: str) -> None:
        """
        Set the id

        :param str id: id to set
        """

        self._id = id

    @property
    def Id(self) -> str:
        """
        Get the id

        :returns: id
        :rtype: str
        """

        return self._id
    
    def FirstWayPoint(self) -> LaneWaypoint:
        """
        Get the first waypoint from the lane segments.

        :returns: the first waypoint
        :rtype: LaneWaypoint
        """

        return LaneWaypoint(self[0].lane, self[0].start_s, 0.0)

    def LastWayPoint(self) -> LaneWaypoint:
        """
        Get the last waypoint from the lane segments.

        :returns: the last waypoint
        :rtype: LaneWaypoint
        """

        return LaneWaypoint(self[-1].lane, self[-1].end_s, 0.0)

    def IsWaypointOnSegment(self, waypoint: LaneWaypoint) -> bool:
        """
        Check if a waypoint is on segment

        :param LaneWaypoint waypoint: the waypoint to check
        :returns: whether the waypoint is on segment
        :rtype: bool
        """

        for segment in self:
            if self.WithinLaneSegment(segment, waypoint):
                return True
        return False

    def IsConnectedSegment(self, other: 'RouteSegments') -> bool:
        """
        Check if we can reach the other segment from current segment just
        by following lane.
        
        :param RouteSegments other: Another route segment
        :returns: whether the other segment is connected to this segment
        :rtype: bool
        """

        if len(self) == 0 or len(other) == 0:
            return False
        if self.IsWaypointOnSegment(other.FirstWayPoint()):
            return True
        if self.IsWaypointOnSegment(other.LastWayPoint()):
            return True
        if other.IsWaypointOnSegment(self.FirstWayPoint()):
            return True
        if other.IsWaypointOnSegment(self.LastWayPoint()):
            return True
        return False

    @property
    def StopForDestination(self) -> bool:
        """
        Stop for destination

        :returns: whether stop for destination
        :rtype: bool
        """

        return self._stop_for_destination

    def SetStopForDestination(self, stop_for_destination: bool) -> None:
        """
        Set stop for destination

        :param bool stop_for_destination: whether stop for destination
        """

        self._stop_for_destination = stop_for_destination
    
    def SetProperties(self, other: 'RouteSegments') -> None:
        """
        Copy the properties of other segments to current one
        """

        self._route_end_waypoint = deepcopy(other.RouteEndWaypoint)
        self._can_exit = deepcopy(other.CanExit)
        self._is_on_segment = deepcopy(other.IsOnSegment)
        self._next_action = deepcopy(other.NextAction)
        self._previous_action = deepcopy(other.PreviousAction)
        self._id = deepcopy(other.Id)
        self._stop_for_destination = deepcopy(other.StopForDestination)

    def WithinLaneSegment(self, lane_segment: LaneSegment, waypoint: LaneWaypoint) -> bool:
        """
        Check if the waypoint is within the lane segment

        :param LaneSegment lane_segment: lane segment
        :param LaneWaypoint waypoint: waypoint
        :returns: whether the waypoint is within the lane segment
        :rtype: bool
        """

        if waypoint.lane is None:
            return False
        tag1: bool = lane_segment.lane.id == waypoint.lane.id
        tag2: bool = lane_segment.start_s - kSegmentationEpsilon <= waypoint.s
        tag3: bool = lane_segment.end_s + kSegmentationEpsilon >= waypoint.s
        return  tag1 and tag2 and tag3

    @staticmethod
    def Length(segments: 'RouteSegments') -> float:
        """
        Get the length of the route segments
        """

        s: float = 0.0
        for seg in segments:
            s += seg.Length()
        return s

    def __str__(self) -> str:
        """
        Get the string representation of the route segments

        :returns: string representation of the route segments
        :rtype: str
        """

        return ";".join(str(element) for element in self) + ";"
