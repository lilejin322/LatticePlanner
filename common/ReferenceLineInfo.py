class ReferenceLineInfo:

    def __init__(self):
        self.reference_line = []
        self.planning_target = None

    def SetPriorityCost(self, cost):
        pass

    def IsChangeLanePath(self):
        return False

    def Lanes(self):
        return self

    def Id(self):
        return "lane_id"

    def set_is_on_reference_line(self):
        pass

    def SetLatticeCruiseSpeed(self, speed):
        pass

    def planning_target(self):
        return self.planning_target

    def SetTrajectory(self, trajectory):
        pass

    def SetCost(self, cost):
        pass

    def SetDrivable(self, drivable):
        pass
