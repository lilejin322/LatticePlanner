"""One planning cycle: the reference line being followed and its obstacles."""

from typing import Dict, List

from common.obstacle import Obstacle
from protoclass.prediction_obstacles import PredictionObstacles
from reference_line.reference_line_info import ReferenceLineInfo


class Frame:
    """Holds the reference line and obstacles for one lattice cycle."""

    def __init__(self, sequence_num: int = 0):
        self._sequence_num = sequence_num
        self._reference_line_info: List[ReferenceLineInfo] = []
        self._obstacles: Dict[str, Obstacle] = {}

    @property
    def reference_line_info(self) -> List[ReferenceLineInfo]:
        return self._reference_line_info

    @property
    def mutable_reference_line_info(self) -> List[ReferenceLineInfo]:
        return self._reference_line_info

    @property
    def obstacles(self) -> List[Obstacle]:
        return list(self._obstacles.values())

    @staticmethod
    def AlignPredictionTime(planning_start_time: float, prediction_obstacles: PredictionObstacles) -> None:
        """Shift prediction points so time zero matches the planning start.

        Points that land before the start are dropped. This matches the sampled
        alignment the lattice prediction querier expects.
        """
        if (
            not prediction_obstacles
            or not prediction_obstacles.header
            or prediction_obstacles.header.timestamp_sec is None
        ):
            return

        prediction_header_time = prediction_obstacles.header.timestamp_sec
        for obstacle in prediction_obstacles.prediction_obstacle:
            for trajectory in obstacle.trajectory:
                for point in trajectory.trajectory_point:
                    point.relative_time = (
                        prediction_header_time + point.relative_time - planning_start_time
                    )
                first_non_negative = 0
                while (
                    first_non_negative < len(trajectory.trajectory_point)
                    and trajectory.trajectory_point[first_non_negative].relative_time < 0
                ):
                    first_non_negative += 1
                if first_non_negative:
                    del trajectory.trajectory_point[:first_non_negative]
