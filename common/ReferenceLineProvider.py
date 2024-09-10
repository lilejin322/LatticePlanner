import threading
import queue
import concurrent.futures
from typing import List, Tuple, Set
from common.LaneInfo import LaneInfo, Id

class ReferenceLineProvider:
    """
    The class of ReferenceLineProvider.
    It provides smoothed reference line to planning.
    """

    def __init__(self, vehicle_state_provider=None, reference_line_config=None, relative_map=None):
        """
        Constructor

        :param VehicleStateProvider vehicle_state_provider: The vehicle state provider
        :param ReferenceLineConfig reference_line_config: The reference line configuration
        :param MapMsg relative_map: The relative map
        """

        self._is_initialized = False
        self._is_stop = False
        self._smoother = None
        self._smoother_config = None
        self._pnc_map_mutex = threading.Lock()
        self._pnc_map_list = []
        self._current_pnc_map = None
        self._relative_map = relative_map
        self._vehicle_state_mutex = threading.Lock()
        self._vehicle_state = None
        self._routing_mutex = threading.Lock()
        self._planning_command = None
        self._has_planning_command = False
        self._is_new_command = False
        self._reference_lines_mutex = threading.Lock()
        self._reference_lines = []
        self._route_segments = []
        self._last_calculation_time = 0.0
        self._reference_line_history = queue.Queue()
        self._route_segments_history = queue.Queue()
        self._task_future = None
        self._is_reference_line_updated = True
        self._vehicle_state_provider = vehicle_state_provider

    def UpdatePlanningCommand(self, command: PlanningCommand) -> bool:
        """
        Update when new PlanningCommand is received.

        :param PlanningCommand command: The new PlanningCommand
        :returns: True if no error occurs.
        :rtype: bool
        """

        with self._routing_mutex:
            self._planning_command = command
            self._is_new_command = True
        return True

    def UpdateVehicleState(self, vehicle_state: VehicleState) -> None:
        """
        Update when new VehicleState is received.

        :param VehicleState vehicle_state: The new VehicleState
        """

        with self._vehicle_state_mutex:
            self._vehicle_state = vehicle_state

    def Start(self) -> bool:
        """
        Start the ReferenceLineProvider.
        
        :returns: success or not
        :rtype: bool
        """

        self._is_stop = False
        self._task_future = concurrent.futures.ThreadPoolExecutor().submit(self.generate_thread)

    def Stop(self) -> None:
        """
        Stop the ReferenceLineProvider.
        """

        self._is_stop = True
        if self._task_future:
            self._task_future.result()  # Wait for the task to complete

    def Reset(self) -> None:
        """
        Reset the ReferenceLineProvider.
        """

        self._is_initialized = False

    def GetReferenceLines(self) -> Tuple[List[ReferenceLine], List[RouteSegments]]:
        """
        Get the reference lines and route segments.

        :returns: The reference lines and route segments
        :rtype: Tuple[List[ReferenceLine], List[RouteSegments]]
        """

        with self._reference_lines_mutex:
            return self._reference_lines, self._route_segments

    def LastTimeDelay(self) -> float:
        """
        Get the last calculation time delay.

        :returns: The last calculation time delay
        :rtype: float
        """

        return self._last_calculation_time

    def FutureRouteWaypoints(self) -> List[LaneWaypoint]:
        """
        Get the future route waypoints.

        :returns: The future route waypoints
        :rtype: List[LaneWaypoint]
        """

        raise NotImplementedError

    def UpdatedReferenceLine(self) -> bool:
        """
        Check if the reference line is updated.

        :returns: True if the reference line is updated
        :rtype: bool
        """

        return self._is_reference_line_updated.load()

    def GetEndLaneWayPoint(self, end_point: LaneWaypoint) -> None:
        """
        Get the end lane waypoint.

        :param LaneWaypoint end_point: The end lane waypoint
        :returns: The end lane waypoint
        :rtype: LaneWaypoint
        """

        raise NotImplementedError

    def GetLaneById(self, id: Id) -> LaneInfo:
        """
        Get the lane by id.

        :param Id id: The id of the lane
        :returns: The lane
        :rtype: LaneInfo
        """

        raise NotImplementedError

    def CreateReferenceLine(self) -> Tuple[bool, List[ReferenceLine], List[RouteSegments]]:
        """
        Use LaneFollowMap to create reference line and the corresponding
        segments based on routing and current position. This is a thread safe
        function.

        :returns: true if !reference_lines.empty() && reference_lines.size() == segments.size()
                  and the corresponding reference lines and route segments
        :rtype: Tuple[bool, List[ReferenceLine], List[RouteSegments]]
        """

        raise NotImplementedError

    def UpdateReferenceLine(self, reference_lines: List[ReferenceLine], route_segments: List[RouteSegments]) -> None:
        """
        store the computed reference line. This function can avoid
        unnecessary copy if the reference lines are the same.

        :param List[ReferenceLine] reference_lines: The reference lines
        :param List[RouteSegments] route_segments: The route segments
        """

        with self._reference_lines_mutex:
            self._reference_lines = reference_lines
            self._route_segments = route_segments
            self._is_reference_line_updated = True

    def GenerateThread(self) -> None:
        """
        Generate thread
        """
        
        raise NotImplementedError

    def IsValidReferenceLine(self) -> None:
        """
        Check if the reference line is valid.
        """

        raise NotImplementedError

    def PrioritizeChangeLane(self, route_segments: List[RouteSegments]):
        """
        Prioritize change lane.

        :param RouteSegments route_segments: The route segments
        """

        raise NotImplementedError

    def CreateRouteSegments(self, vehicle_state: VehicleState) -> Tuple[bool, List[RouteSegments]]:
        """
        Create route segments.

        :param VehicleState vehicle_state: The vehicle state
        :returns: True if the route segments are created successfully and the route segments
        :rtype: Tuple[bool, List[RouteSegments]]
        """
        
        raise NotImplementedError

    def IsReferenceLineSmoothValid(self, raw: ReferenceLine, smoothed: ReferenceLine) -> bool:
        """
        Check if the smoothed reference line is valid.

        :param ReferenceLine raw: The raw reference line
        :param ReferenceLine smoothed: The smoothed reference line
        :returns: True if the smoothed reference line is valid
        :rtype: bool
        """

        raise NotImplementedError

    def SmoothReferenceLine(self, raw_reference_line: ReferenceLine) -> Tuple[bool, ReferenceLine]:
        """
        Smooth the reference line.

        :param ReferenceLine raw_reference_line: The raw reference line
        :returns: True if the reference line is smoothed and the smoothed reference line
        :rtype: Tuple[bool, ReferenceLine]
        """

        raise NotImplementedError

    def SmoothPrefixedReferenceLine(self, prefix_ref: ReferenceLine, raw_ref: ReferenceLine) -> Tuple[bool, ReferenceLine]:
        """
        Smooth the prefixed reference line.

        :param ReferenceLine prefix_ref: The prefixed reference line
        :param ReferenceLine raw_ref: The raw reference line
        :returns: True if the reference line is smoothed and the smoothed reference line
        :rtype: Tuple[bool, ReferenceLine]
        """

        raise NotImplementedError

    def GetAnchorPoints(self, reference_line: ReferenceLine) -> List[AnchorPoint]:
        """
        Get the anchor points.

        :param ReferenceLine reference_line: The reference line
        :returns: The anchor points
        :rtype: List[AnchorPoint]
        """

        raise NotImplementedError

    def SmoothRouteSegment(self, segments: RouteSegments, reference_line: ReferenceLine) -> bool:
        """
        Smooth the route segment.

        :param RouteSegments segments: The route segments
        :param ReferenceLine reference_line: The reference line
        :returns: True if the route segment is smoothed
        :rtype: bool
        """

        raise NotImplementedError

    def ExtendReferenceLine(self, state: VehicleState) -> Tuple[bool, RouteSegments, ReferenceLine]:
        """
        This function creates a smoothed forward reference line
        based on the given segments.

        :param VehicleState state: The vehicle state
        :returns: True if the reference line is extended, the route segments and the reference line
        :rtype: Tuple[bool, RouteSegments, ReferenceLine]
        """

        raise NotImplementedError

    def GetAnchorPoint(self, reference_line: ReferenceLine, s: float) -> AnchorPoint:
        """
        Get the anchor point.

        :param ReferenceLine reference_line: The reference line
        :param float s: The s value
        :returns: The anchor point
        :rtype: AnchorPoint
        """

        raise NotImplementedError

    def GetReferenceLinesFromRelativeMap(self) -> Tuple[List[ReferenceLine], List[RouteSegments]]:
        """
        Get the reference lines from the relative map.

        :returns: The reference lines and route segments
        :rtype: Tuple[List[ReferenceLine], List[RouteSegments]]
        """

        raise NotImplementedError

    def GetNearestWayPointFromNavigationPath(self, state: VehicleState, navigation_lane_ids: Set[str]) -> LaneWaypoint:
        """
        This function get adc lane info from navigation path and map
        by vehicle state.

        :param VehicleState state: The vehicle state
        :param Set[str] navigation_lane_ids: The navigation lane ids
        :returns: The nearest waypoint
        :rtype: LaneWaypoint
        """

        raise NotImplementedError

    def Shrink(self, sl: SLPoint) -> Tuple[bool, ReferenceLine, RouteSegments]:
        """
        Shrink the reference line.

        :param SLPoint sl: The SL point
        :returns: True if the reference line is shrunk, the reference line and the route segments
        :rtype: Tuple[bool, ReferenceLine, RouteSegments]
        """

        raise NotImplementedError
