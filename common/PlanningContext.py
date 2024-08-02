from protoclass.PlanningStatus import PlanningStatus

class PlanningContext:
    """
    PlanningContext is the runtime context in planning. It is
    persistent across multiple frames.
    """

    def __init__(self) -> None:
        """
        Constructor
        """

        self._planning_status: PlanningStatus = PlanningStatus()
    
    def Clear(self) -> None:
        """
        Clear the context
        """

        self.__init__()
    
    def Init(self) -> None:
        """
        Initialize the context
        """

        pass
    
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
