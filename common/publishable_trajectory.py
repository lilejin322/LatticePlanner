"""Publishable trajectory aligned with publishable_trajectory.cc."""

from __future__ import annotations

from typing import List

from common.discretized_trajectory import DiscretizedTrajectory
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.trajectory_point import TrajectoryPoint


class PublishableTrajectory(DiscretizedTrajectory):
    def __init__(self, header_time: float, discretized_trajectory: DiscretizedTrajectory):
        super().__init__(list(discretized_trajectory))
        self._header_time = header_time

    @property
    def header_time(self) -> float:
        return self._header_time

    def __getitem__(self, index):
        if isinstance(index, slice):
            result = PublishableTrajectory.__new__(PublishableTrajectory)
            result.data = self.data[index]
            result._header_time = self._header_time
            return result
        return super().__getitem__(index)

    def PrependTrajectoryPoints(self, trajectory_points: List[TrajectoryPoint]) -> None:
        self[:0] = list(trajectory_points)

    def PopulateTrajectoryProtobuf(self, trajectory_pb: ADCTrajectory) -> None:
        if trajectory_pb.header is None:
            from protoclass.header import Header

            trajectory_pb.header = Header()
        trajectory_pb.header.timestamp_sec = self._header_time
        trajectory_pb.trajectory_point = list(self)
        if self:
            last_tp = self[-1]
            trajectory_pb.total_path_length = last_tp.path_point.s
            trajectory_pb.total_path_time = last_tp.relative_time
