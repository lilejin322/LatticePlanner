"""Reference path taken from a CommonRoad lanelet.

Orientations, curvatures, and station come from commonroad-io's polyline
helpers and ``Lanelet.distance``. Lane width at a station comes from
``Lanelet.interpolate_position``, which is the distance from the center point
to the left and right boundaries at that station.
"""

from __future__ import annotations

import copy
from typing import Callable, Sequence, Tuple

import numpy as np

from commonroad.geometry.polyline_util import (
    compute_polyline_curvatures,
    compute_polyline_orientations,
)
from commonroad.scenario.lanelet import Lanelet
from protoclass.path_point import PathPoint

LaneWidth = Callable[[float], Tuple[bool, float, float]]


class LaneletPath:
    """Path points and lane width for one lattice cycle."""

    def __init__(self, lanelet: Lanelet, points: list, speed_limit: float):
        self.lanelet = lanelet
        self.points = points
        self.speed_limit = float(speed_limit)

    def lane_width(self, station: float) -> Tuple[bool, float, float]:
        length = float(self.lanelet.distance[-1])
        if station < 0.0 or station > length:
            return False, 0.0, 0.0
        # interpolate_position indexes the next vertex, so the exact end is
        # the last center point rather than a segment past the polyline.
        if station >= length - 1e-9:
            center = self.lanelet.center_vertices[-1]
            left = self.lanelet.left_vertices[-1]
            right = self.lanelet.right_vertices[-1]
        else:
            center, right, left, _index = self.lanelet.interpolate_position(float(station))
        return (
            True,
            float(np.hypot(left[0] - center[0], left[1] - center[1])),
            float(np.hypot(right[0] - center[0], right[1] - center[1])),
        )

    def speed_at(self, _station: float) -> float:
        return self.speed_limit


def path_from_lanelet(lanelet: Lanelet, speed_limit: float) -> LaneletPath:
    """Sample the lanelet centerline into the points the lattice combiner matches."""
    center = np.asarray(lanelet.center_vertices, dtype=float)
    if len(center) < 2:
        raise ValueError("lanelet centerline needs at least two points")
    stations = np.asarray(lanelet.distance, dtype=float)
    headings = compute_polyline_orientations(center)
    if len(center) >= 3:
        curvature = compute_polyline_curvatures(center)
    else:
        curvature = np.zeros(len(center))
    points = []
    for index, (xy, station, heading, kappa) in enumerate(
        zip(center, stations, headings, curvature)
    ):
        if index + 1 < len(center):
            step = float(stations[index + 1] - station)
            dkappa = (float(curvature[index + 1]) - float(kappa)) / step if step > 1e-6 else 0.0
        else:
            dkappa = 0.0
        points.append(
            PathPoint(
                x=float(xy[0]),
                y=float(xy[1]),
                z=0.0,
                theta=float(heading),
                kappa=float(kappa),
                s=float(station),
                dkappa=dkappa,
                ddkappa=0.0,
            )
        )
    return LaneletPath(lanelet, points, speed_limit)


def straight_lanelet(length: float, left_width: float, right_width: float, step: float = 1.0) -> Lanelet:
    """A straight lanelet for tests. Boundaries are offset from the centerline."""
    count = max(int(length / step) + 1, 2)
    stations = np.linspace(0.0, float(length), count)
    center = np.column_stack((stations, np.zeros(count)))
    left = np.column_stack((stations, np.full(count, float(left_width))))
    right = np.column_stack((stations, np.full(count, -float(right_width))))
    return Lanelet(left, center, right, lanelet_id=1)


def translated_lanelet(lanelet: Lanelet, origin_xy: Sequence[float]) -> Lanelet:
    """Copy ``lanelet`` into a frame whose origin is ``origin_xy``."""
    local = copy.deepcopy(lanelet)
    local.translate_rotate(np.asarray([-float(origin_xy[0]), -float(origin_xy[1])], dtype=float), 0.0)
    local.distance = None
    return local


def merged_lanelet(chain: Sequence[Lanelet]) -> Lanelet:
    """Join an already-connected successor chain with ``Lanelet.merge_lanelets``."""
    if not chain:
        raise ValueError("lanelet chain is empty")
    merged = copy.deepcopy(chain[0])
    for nxt in chain[1:]:
        merged = Lanelet.merge_lanelets(merged, copy.deepcopy(nxt))
    return merged
