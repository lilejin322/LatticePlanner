from dataclasses import dataclass, field
from typing import List


@dataclass
class NudgeInfo:
    """
    Nudge info, this class has not been implemented yet

    Maybe no need
    """

    left_nudge_obstacle_ids: List[str] = field(default_factory=list)
    right_nudge_obstacle_ids: List[str] = field(default_factory=list)

    def Clear(self) -> None:
        self.left_nudge_obstacle_ids.clear()
        self.right_nudge_obstacle_ids.clear()

    def AddLeftNudgeObstacle(self, obstacle_id: str) -> None:
        if obstacle_id not in self.left_nudge_obstacle_ids:
            self.left_nudge_obstacle_ids.append(obstacle_id)

    def AddRightNudgeObstacle(self, obstacle_id: str) -> None:
        if obstacle_id not in self.right_nudge_obstacle_ids:
            self.right_nudge_obstacle_ids.append(obstacle_id)
