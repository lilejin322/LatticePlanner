from typing import List, Dict, Tuple
from enum import Enum
from protoclass.TrajectoryPoint import TrajectoryPoint
from common.ReferenceLine import ReferenceLine
from common.RouteSegments import RouteSegments
from common.Obstacle import Obstacle
from common.DiscretizedTrajectory import DiscretizedTrajectory
from protoclass.SLBoundary import SLBoundary
from protoclass.ADCTrajectory import ADCTrajectory
from protoclass.Lane import Lane
from common.Path import PathOverlap
from protoclass.SLBoundary import SLPoint
from protoclass.DecisionResult import DecisionResult, VehicleSignal, ObjectDecisions
from protoclass.VehicleState import VehicleState
from common.PathData import PathData
from common.PathDecision import PathDecision
from common.SpeedData import SpeedData
from protoclass.LatencyStats import LatencyStats
from protoclass.EngageAdvice import EngageAdvice
from common.PathBoundary import PathBoundary
from common.PlanningContext import PlanningContext
from protoclass.RSSInfo import RSSInfo
from protoclass.lattice_structure import StopPoint, PlanningTarget
from common.StGraphData import StGraphData
from common.LaneInfo import LaneInfo

class ReferenceLineInfo:
    """
    ReferenceLineInfo holds all data for one reference line.
    """

    class LaneType(Enum):

        LeftForward = 0
        LeftReverse = 1
        RightForward = 2
        RightReverse = 3

    class OverlapType(Enum):
        """
        different types of overlaps that can be handled by different scenarios.
        """

        CLEAR_AREA = 1
        CROSSWALK = 2
        OBSTACLE = 3
        PNC_JUNCTION = 4
        SIGNAL = 5
        STOP_SIGN = 6
        YIELD_SIGN = 7
        JUNCTION = 8

    def __init__(self, vehicle_state: VehicleState=None, adc_planning_point: TrajectoryPoint=None, reference_line: ReferenceLine=None, segments: RouteSegments=None):
        """
        Constructor
        """

        self._vehicle_state = vehicle_state
        self._adc_planning_point = adc_planning_point
        self._reference_line = reference_line
        self._lanes: RouteSegments = segments
        # this is the number that measures the goodness of this reference line.
        # The lower the better.
        self._cost: float = 0.0
        self._is_drivable: bool = True
        self._path_decision: PathDecision = None
        self._blocking_obstacle: Obstacle = None
        self._candidate_path_boundaries: List[PathBoundary] = []
        self._candidate_path_data: List[PathData] = []
        self._path_data: PathData = None
        self._fallback_path_data: PathData = None
        self._speed_data: SpeedData = None
        self._discretized_trajectory: DiscretizedTrajectory = None
        self._rss_info: RSSInfo = None
        # SL boundary of stitching point (starting point of plan trajectory)
        # relative to the reference line
        self._adc_sl_boundary: SLBoundary = None
        self._latency_stats: LatencyStats = None
        self._is_on_reference_line: bool = False
        self._is_path_lane_borrow: bool = False
        self._status: ADCTrajectory.RightOfWayStatus = ADCTrajectory.RightOfWayStatus.UNPROTECTED
        self._offset_to_other_reference_line: float = 0.0
        self._priority_cost: float = 0.0
        self._planning_target: PlanningTarget = None
        self._trajectory_type = ADCTrajectory.TrajectoryType.UNKNOWN
        # Overlaps encountered in the first time along the reference line in front of
        # the vehicle
        self._first_encounter_overlaps: List[Tuple[ReferenceLineInfo.OverlapType, PathOverlap]] = []
        # Data generated by speed_bounds_decider for constructing st_graph for
        # different st optimizer
        self._st_graph_data: StGraphData = None
        self._vehicle_signal = None
        self._cruise_speed = 0.0
        self._base_cruise_speed = 0.0
        self._path_reusable = False
        self._junction_right_of_way_map: Dict = {}

    def Init(self, obstacles: List[Obstacle], target_speed: float) -> bool:
        """

        returns: True if successful, False otherwise
        rtype: bool
        """

        raise NotImplementedError        

    def AddObstacles(self, obstacles: List[Obstacle]) -> bool:
        """
        Add obstacles to the reference line info

        param List[Obstacle] obstacles: List of obstacles to add
        returns: True if successful, False otherwise
        rtype: bool
        """

        raise NotImplementedError
    
    def AddObstacle(self, obstacle: Obstacle) -> Obstacle:
        """
        Add an obstacle to the reference line info
        
        (Note this part in cpp is weird using pointer, should take a deeper look)
        param Obstacle obstacle: Obstacle to add
        returns: The added obstacle
        rtype: Obstacle
        """

        raise NotImplementedError


    @property
    def vehicle_state(self) -> VehicleState:
        """
        Get the vehicle state

        returns: The vehicle state
        rtype: VehicleState
        """

        return self._vehicle_state
    
    def path_decision(self) -> PathDecision:
        """
        """

        raise NotImplementedError
    
    def reference_line(self) -> ReferenceLine:
        """
        """

        raise NotImplementedError
    
    def mutable_reference_line(self) -> ReferenceLine:
        """
        """

        raise NotImplementedError
    
    def SDistanceToDestination(self) -> float:
        """
        """

        raise NotImplementedError
    
    def ReachedDestination(self) -> bool:
        """
        """

        raise NotImplementedError
    
    def SetTrajectory(self, trajectory: DiscretizedTrajectory) -> None:
        """
        """

        raise NotImplementedError
    
    def trajectory(self) -> DiscretizedTrajectory:
        """
        """

        raise NotImplementedError
    
    @property
    def Cost(self) -> float:
        """
        """

        return self._cost
    
    def AddCost(self, cost: float) -> None:
        """
        Add cost to the current cost

        :param float cost: The cost to add
        """

        self._cost += cost
    
    def SetCost(self, cost: float) -> None:
        """
        Set the cost

        :param float cost: The cost to set
        """

        self._cost = cost
    
    @property
    def PriorityCost(self) -> float:
        """
        Get the priority cost

        returns: The priority cost
        rtype: float
        """

        return self._priority_cost
    
    def SetPriorityCost(self, cost: float) -> None:
        """

        :param float cost: The cost to set
        """

        self._priority_cost = cost
    
    def SetLatticeStopPoint(stop_point: StopPoint) -> None:
        """
        Set the lattice stop point
        For lattice planner'speed planning target

        :param StopPoint stop_point: The stop point to set
        """

        raise NotImplementedError
    
    def SetLatticeCruiseSpeed(speed: float) -> None:
        """
        Set the lattice cruise speed
        For lattice planner'speed planning target

        :param float speed: The speed to set
        """

        raise NotImplementedError

    @property
    def planning_target(self) -> PlanningTarget:
        """
        Get the planning target
        For lattice planner'speed planning target
        """

        return self._planning_target
    
    def SetCruiseSpeed(self, speed: float) -> None:
        """
        Set the cruise speed

        :param float speed: The speed to set
        """

        self._cruise_speed = speed
        self._base_cruise_speed = speed
    
    def LimitCruiseSpeed(self, speed: float) -> None:
        """
        Limit the cruise speed based on the "base_cruise_speed_". If the new
        setting speed > "base_cruise_speed_", it will be ignored.
        
        :param float speed: The new speed.
        """

        raise NotImplementedError
    
    def GetBaseCruiseSpeed(self) -> float:
        """
        Get the base cruise speed
        """

        raise NotImplementedError
    
    def GetCruiseSpeed(self) -> float:
        """
        Get the cruise speed
        """

        raise NotImplementedError
    
    def LocateLaneInfo(s: float) -> LaneInfo:
        """
        Locate the lane info based on the s

        :param float s: The s value
        :returns: The lane info
        :rtype: LaneInfo
        """

        raise NotImplementedError
    
    def GetNeighborLaneInfo(lane_type: LaneType, s: float) -> Tuple[bool, str, float]:
        """
        Get the neighbor lane info

        :param LaneType lane_type: The lane type
        :param float s: The s value
        :returns: The neighbor lane info, (bool, str lane_id, float lane_width)
        :rtype: Tuple[bool, str, float]
        """

        raise NotImplementedError
    
    def IsStartFrom(previous_reference_line_info: 'ReferenceLineInfo') -> bool:
        """
        check if current reference line is started from another reference
        line info line. The method is to check if the start point of current
        reference line is on previous reference line info.

        :returns: True if current reference line starts on previous reference
        line, otherwise False.
        """

        raise NotImplementedError
    
    @property
    def latency_stats(self) -> LatencyStats:
        """
        Get the latency stats

        :returns: The latency stats
        :rtype: LatencyStats
        """

        return self._latency_stats
    
    def path_data(self) -> PathData:
        """
        Get the path data

        :returns: The path data
        :rtype: PathData
        """

        raise NotImplementedError
    
    def fallback_path_data(self) -> PathData:
        """
        Get the fallback path data

        :returns: The fallback path data
        :rtype: PathData
        """

        raise NotImplementedError
    
    def speed_data(self) -> SpeedData:
        """
        Get the speed data

        :returns: The speed data
        :rtype: SpeedData
        """

        raise NotImplementedError
    
    def rss_info(self) -> RSSInfo:
        """
        Get the rss info

        :returns: The rss info
        :rtype: RSSInfo
        """

        raise NotImplementedError
    
    def CombinePathAndSpeedProfile(relative_time: float, start_s: float, discretized_trajectory: DiscretizedTrajectory) -> bool:
        """
        aggregate final result together by some configuration
        """

        raise NotImplementedError
    
    def AdjustTrajectoryWhichStartsFromCurrentPos(planning_start_point: TrajectoryPoint,
                                                  trajectory: List[TrajectoryPoint]) -> DiscretizedTrajectory:
        """
        adjust trajectory if it starts from cur_vehicle postion rather planning
        init point from upstream

        :param TrajectoryPoint planning_start_point: The planning start point
        :param List[TrajectoryPoint] trajectory: The trajectory to adjust
        :returns: The adjusted trajectory
        :rtype: DiscretizedTrajectory
        """

        raise NotImplementedError
    
    def AdcSlBoundary(self) -> SLBoundary:
        """

        """

        raise NotImplementedError
    
    def PathSpeedDebugString(self) -> str:
        """

        """

        raise NotImplementedError
    
    def IsChangeLanePath(self) -> bool:
        """
        Check if the current reference line is a change lane reference line, i.e.,
        ADC's current position is not on this reference line.

        :returns: True if it is a change lane reference line, otherwise False.
        :rtype: bool
        """

        raise NotImplementedError
    
    def IsNeighborLanePath(self) -> bool:
        """
        Check if the current reference line is the neighbor of the vehicle
        current position

        :returns: True if it is a neighbor lane reference line, otherwise False.
        :rtype: bool
        """

        raise NotImplementedError
    
    def SetDrivable(self, drivable: bool) -> None:
        """
        Set if the vehicle can drive following this reference line
        A planner need to set this value to true if the reference line is OK

        :param bool drivable: The value to set
        """

        raise NotImplementedError
    
    def IsDrivable(self) -> bool:
        """
        
        :returns: True if the vehicle can drive following this reference line, otherwise False.
        :rtype: bool
        """

        raise NotImplementedError
    
    def ExportEngageAdvice(self) -> Tuple[EngageAdvice, PlanningContext]:
        """

        """

        raise NotImplementedError
    
    def Lanes(self) -> RouteSegments:
        """

        """

        raise NotImplementedError
    
    def TargetLaneId(self) -> List[str]:
        """

        """

        raise NotImplementedError
    
    def ExportDecision(self) -> Tuple[DecisionResult, PlanningContext]:
        """


        :returns: (DecisionResult decision_result, PlanningContext planning_context)
        :rtype: Tuple[DecisionResult, PlanningContext]
        """

        raise NotImplementedError
    
    def SetJunctionRightOfWay(self, junction_s: float, is_protected: bool) -> None:
        """
        
        """
        
        raise NotImplementedError
    
    def GetRightOfWayStatus(self) -> ADCTrajectory.RightOfWayStatus:
        """
        
        """

        raise NotImplementedError
    
    def GetPathTurnType(self, s: float) -> Lane.LaneTurn:
        """

        """

        raise NotImplementedError
    
    def GetIntersectionRightofWayStatus(self, pnc_junction_overlap: PathOverlap) -> bool:
        """

        """

        raise NotImplementedError

    @property
    def OffsetToOtherReferenceLine(self) -> float:
        """
        Get the offset to other reference line

        :returns: The offset to other reference line
        :rtype: float
        """

        return self._offset_to_other_reference_line
    
    def SetOffsetToOtherReferenceLine(self, offset: float) -> None:
        """
        Set the offset to other reference line

        :param float offset: The offset to set
        """

        self._offset_to_other_reference_line = offset
    
    def GetCandidatePathBoundaries(self) -> List[PathBoundary]:
        """
        Get the candidate path boundaries

        :returns: The candidate path boundaries
        :rtype: List[PathBoundary]
        """

        raise NotImplementedError
    
    def SetCandidatePathBoundaries(self, candidate_path_boundaries: List[PathBoundary]) -> None:
        """
        Set the candidate path boundaries

        :param PathBoundary candidate_path_boundaries: The candidate path boundaries to set
        """

        raise NotImplementedError
    
    def GetCandidatePathData(self) -> List[PathData]:
        """
        Get the candidate path data

        :returns: The candidate path data
        :rtype: List[PathData]
        """

        raise NotImplementedError
    
    def SetCandidatePathData(self, candidate_path_data: List[PathData]) -> None:
        """
        Set the candidate path data

        :param List[PathData] candidate_path_data: The candidate path data to set
        """

        raise NotImplementedError
    
    def GetBlockingObstacle(self) -> Obstacle:
        """
        Get the blocking obstacle

        :returns: The blocking obstacle
        :rtype: Obstacle
        """

        return self._blocking_obstacle
    
    def SetBlockingObstacle(self, blocking_obstacle_id: str) -> None:
        """
        Set the blocking obstacle

        :param str blocking_obstacle_id: The obstacle to set
        """

        raise NotImplementedError
    
    def is_path_lane_borrow(self) -> bool:
        """
        Check if the path is lane borrow

        :returns: True if the path is lane borrow, otherwise False.
        :rtype: bool
        """

        return self._is_path_lane_borrow
    
    def set_is_path_lane_borrow(self, is_path_lane_borrow: bool) -> None:
        """
        Set the path is lane borrow

        :param bool is_path_lane_borrow: The value to set
        """

        self._is_path_lane_borrow = is_path_lane_borrow

    def set_is_on_reference_line(self) -> None:
        """
        Set the path is on reference line
        """

        self._is_on_reference_line = True
    
    def GetPriority(self) -> int:
        """
        Get the priority

        :returns: The priority
        :rtype: int
        """

        return self._reference_line.GetPriority()
    
    def SetPriority(self, priority: int) -> None:
        """
        Set the priority

        :param int priority: The priority to set
        """

        self._reference_line.SetPriority(priority)
    
    def set_trajectory_type(self, trajectory_type: ADCTrajectory.TrajectoryType) -> None:
        """
        Set the trajectory type

        :param ADCTrajectory.TrajectoryType trajectory_type: The trajectory type to set
        """

        self._trajectory_type = trajectory_type
    
    @property
    def trajectory_type(self) -> ADCTrajectory.TrajectoryType:
        """
        Get the trajectory type

        :returns: The trajectory type
        :rtype: ADCTrajectory.TrajectoryType
        """
            
        return self._trajectory_type

    def st_graph_data(self) -> StGraphData:
        """
        Get the st graph data
        
        :returns: The st graph data
        :rtype: StGraphData
        """

        return self._st_graph_data
    
    def FirstEncounteredOverlaps(self) -> List[Tuple[OverlapType, PathOverlap]]:
        """
        Get the first encountered overlaps

        :returns: The first encountered overlaps
        :rtype: List[Tuple[OverlapType, PathOverlap]]
        """

        return self._first_encounter_overlaps
    
    def GetPnCJunction(self, s: float) -> Tuple[int, PathOverlap]:
        """
        Get the pnc junction

        :param float s: The s value
        :returns: (int, PathOverlap pnc_junction_overlap)
        :rtype: Tuple[int, PathOverlap]
        """

        raise NotImplementedError
    
    def GetJunction(self, s: float) -> Tuple[int, PathOverlap]:
        """
        Get the junction

        :param float s: The s value
        :returns: (int, PathOverlap junction_overlap)
        :rtype: Tuple[int, PathOverlap]
        """

        raise NotImplementedError
    
    def GetAllStopDecisionSLPoint(self) -> List[SLPoint]:
        """
        Get all stop decision sl point

        :returns: The stop decision sl point
        :rtype: List[SLPoint]
        """

        raise NotImplementedError
    
    def SetTurnSignal(self, turn_signal: VehicleSignal.TurnSignal) -> None:
        """
        Set the turn signal

        :param VehicleSignal.TurnSignal turn_signal: The turn signal to set
        """

        raise NotImplementedError
    
    def SetEmergencyLight(self) -> None:
        """
        Set the emergency light
        """

        raise NotImplementedError
    
    def set_path_reusable(self, path_reusable: bool) -> None:
        """
        Set the path reusable

        :param bool path_reusable: The value to set
        """

        self._path_reusable = path_reusable
    
    @property
    def path_reusable(self) -> bool:
        """
        Get the path reusable

        :returns: The path reusable
        :rtype: bool
        """

        return self._path_reusable
    
    def GetOverlapOnReferenceLine(self, overlap_id: str, overlap_type: OverlapType) -> PathOverlap:
        """
        Get the overlap on reference line

        :param str overlap_id: The overlap id
        :param OverlapType overlap_type: The overlap type
        :returns: The overlap
        :rtype: PathOverlap
        """

        raise NotImplementedError
    
    def InitFirstOverlaps(self) -> None:
        """
        Init the first overlaps
        """

        raise NotImplementedError
    
    def CheckChangeLane(self) -> bool:
        """
        Check if the vehicle is changing lane

        :returns: True if the vehicle is changing lane, otherwise False.
        :rtype: bool
        """

        raise NotImplementedError

    def SetTurnSignalBasedOnLaneTurnType(self) -> VehicleSignal:
        """
        Set the turn signal based on lane turn type

        :returns: The turn signal
        :rtype: VehicleSignal
        """

        raise NotImplementedError
    
    def ExportVehicleSignal(self) -> VehicleSignal:
        """
        Export the vehicle signal

        :returns: The vehicle signal
        :rtype: VehicleSignal
        """

        raise NotImplementedError
    
    def IsIrrelevantObstacle(obstacle: Obstacle) -> bool:
        """
        Check if the obstacle is irrelevant

        :param Obstacle obstacle: The obstacle to check
        :returns: True if the obstacle is irrelevant, otherwise False.
        :rtype: bool
        """

        raise NotImplementedError
    
    def MakeDecision(self) -> Tuple[DecisionResult, PlanningContext]:
        """
        Make decision

        :returns: (DecisionResult decision_result, PlanningContext planning_context)
        :rtype: Tuple[DecisionResult, PlanningContext]
        """
            
        raise NotImplementedError
    
    def MakeMainStopDecision(self) -> Tuple[int, DecisionResult]:
        """
        Make main stop decision

        :returns: (int, DecisionResult decision_result)
        :rtype: Tuple[int, DecisionResult]
        """

        raise NotImplementedError
    
    def MakeMainMissionCompleteDecision(self) -> Tuple[DecisionResult, PlanningContext]:
        """
        Make main mission complete decision

        :returns: (DecisionResult decision_result, PlanningContext planning_context)
        :rtype: Tuple[DecisionResult, PlanningContext]
        """

        raise NotImplementedError

    def MakeEStopDecision(self) -> DecisionResult:
        """
        Make E stop decision

        :returns: The decision result
        :rtype: DecisionResult
        """

        raise NotImplementedError
    
    def SetObjectDecisions(self) -> ObjectDecisions:
        """
        Set object decisions

        :returns: The object decisions
        :rtype: ObjectDecisions
        """

        raise NotImplementedError
    
    def AddObstacleHelper(self, obstacle: Obstacle) -> bool:
        """
        Add obstacle helper

        :param Obstacle obstacle: The obstacle to add
        :returns: True if successful, False otherwise
        :rtype: bool
        """

        raise NotImplementedError
    
    def GetFirstOverlap(self, path_overlaps: List[PathOverlap]) -> PathOverlap:
        """
        Get the first overlap

        :param List[PathOverlap] path_overlaps: The path overlaps
        :returns: The first overlap
        :rtype: PathOverlap
        """

        raise NotImplementedError

    def DISALLOW_COPY_AND_ASSIGN(self, ReferenceLineInfo):
        """
        This function is weird, the python should be implemented as follow template

        class ReferenceLineInfo:
            def __init__(self):
                pass

            def __copy__(self):
                raise NotImplementedError("Copy operation is not allowed")

            def __deepcopy__(self, memo):
                raise NotImplementedError("Deep copy operation is not allowed")

            def __setattr__(self, key, value):
                if key in self.__dict__:
                    raise AttributeError("Assignment operation is not allowed")
                super().__setattr__(key, value)
        """

        raise NotImplementedError
