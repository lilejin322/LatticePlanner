from protoclass.planning_status import (
    ChangeLaneStatus,
    CreepDeciderStatus,
    CrosswalkStatus,
    DestinationStatus,
    EmergencyStopStatus,
    LaneBorrowStatus,
    LaneFollowCommand,
    LaneFollowStatus,
    OpenSpaceStatus,
    ParkAndGoStatus,
    PathDeciderStatus,
    PlanningStatus,
    PullOverStatus,
    ReroutingStatus,
    ScenarioStatus,
    SpeedDeciderStatus,
    StopSignStatus,
    TrafficLightStatus,
    YieldSignStatus,
)

class PlanningContext:
    """
    PlanningContext is the runtime context in planning. It is
    persistent across multiple frames.
    """

    def __init__(self) -> None:
        """
        Constructor
        """

        self.Init()
    
    def Clear(self) -> None:
        """
        Clear the context
        """

        self.Init()
    
    def Init(self) -> None:
        """
        Initialize the context
        """

        self._planning_status = PlanningStatus(
            change_lane=ChangeLaneStatus(),
            creep_decider=CreepDeciderStatus(),
            crosswalk=CrosswalkStatus(),
            destination=DestinationStatus(),
            emergency_stop=EmergencyStopStatus(),
            open_space=OpenSpaceStatus(),
            park_and_go=ParkAndGoStatus(),
            path_decider=PathDeciderStatus(),
            pull_over=PullOverStatus(),
            rerouting=ReroutingStatus(lane_follow_command=LaneFollowCommand()),
            scenario=ScenarioStatus(),
            speed_decider=SpeedDeciderStatus(),
            stop_sign=StopSignStatus(),
            traffic_light=TrafficLightStatus(),
            yield_sign=YieldSignStatus(),
            lane_follow=LaneFollowStatus(),
            lane_borrow=LaneBorrowStatus(),
        )

    def mutable_planning_status(self) -> PlanningStatus:
        return self._planning_status
    
    @property
    def planning_status(self) -> PlanningStatus:
        """
        Get the planning status
        please put all status info inside PlanningStatus for easy maintenance.
        do NOT create new struct at this level.

        :returns: Planning status
        :rtype: PlanningStatus
        """

        return self._planning_status
