"""Predicted rectangles for the lattice ST graph and collision check.

The shape is a CommonRoad ``RectOccupancy``. A mover is either one pose or a
timed sequence. The lattice sampler still projects those corners onto the
lanelet; this file does not change the polynomials or the costs.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import shapely

from common.geometry_utils import NormalizeAngle, lerp
from commonroad.common.util import make_valid_orientation
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy


@dataclass
class Pose:
    t: float
    x: float
    y: float
    theta: float
    v: float


class Mover:
    def __init__(self, mover_id: str, length: float, width: float, poses: Sequence[Pose]):
        if not poses:
            raise ValueError("a mover needs at least one pose")
        self.mover_id = mover_id
        self.length = float(length)
        self.width = float(width)
        self.poses = list(poses)

    @property
    def id(self) -> str:
        return self.mover_id

    def moving(self) -> bool:
        return len(self.poses) >= 2

    def pose_at(self, t: float) -> Pose:
        poses = self.poses
        if len(poses) == 1 or t <= poses[0].t:
            return poses[0]
        if t >= poses[-1].t:
            return poses[-1]
        index = bisect_left([pose.t for pose in poses], t)
        left = poses[index - 1]
        right = poses[index]
        if abs(right.t - left.t) < 1e-9:
            return right
        return Pose(
            t=t,
            x=lerp(left.x, left.t, right.x, right.t, t),
            y=lerp(left.y, left.t, right.y, right.t, t),
            theta=_angle_lerp(left.theta, left.t, right.theta, right.t, t),
            v=lerp(left.v, left.t, right.v, right.t, t),
        )

    def corners_at(self, t: float) -> List[Tuple[float, float]]:
        vertices = self.occupancy_at(t).vertices
        return list(vertices[:-1])

    def occupancy_at(self, t: float, length_extra: float = 0.0, width_extra: float = 0.0) -> RectOccupancy:
        pose = self.pose_at(t)
        return _rect(pose.x, pose.y, pose.theta, self.length + length_extra, self.width + width_extra)


def ego_occupancy(x: float, y: float, theta: float, length: float, width: float, back_edge_to_center: float) -> RectOccupancy:
    """Rectangle centered on the geometric center, not the rear axle."""
    shift = length / 2.0 - back_edge_to_center
    heading = theta or 0.0
    return _rect(
        x + shift * _cos(heading),
        y + shift * _sin(heading),
        heading,
        length,
        width,
    )


def rectangles_overlap(left: RectOccupancy, right: RectOccupancy) -> bool:
    return left.shapely_object.intersects(right.shapely_object)


def mover_from_actor(actor) -> Mover:
    import config

    speed = 0.0 if actor.is_static else actor.v
    if actor.is_static or abs(speed) < 1e-3:
        return Mover(
            actor.actor_id,
            actor.length,
            actor.width,
            [Pose(0.0, actor.x, actor.y, actor.theta, 0.0)],
        )
    dt = config.FLAGS_trajectory_time_resolution
    horizon = config.FLAGS_trajectory_time_length
    poses = []
    t = 0.0
    while t <= horizon + 1e-9:
        poses.append(
            Pose(
                t,
                actor.x + speed * _cos(actor.theta) * t,
                actor.y + speed * _sin(actor.theta) * t,
                actor.theta,
                speed,
            )
        )
        t += dt
    return Mover(actor.actor_id, actor.length, actor.width, poses)


def _rect(x: float, y: float, theta: float, length: float, width: float) -> RectOccupancy:
    return RectOccupancy(
        rect_center=shapely.Point(float(x), float(y)),
        width=float(width),
        length=float(length),
        orientation=make_valid_orientation(float(theta or 0.0)),
    )


def _angle_lerp(a0: float, t0: float, a1: float, t1: float, t: float) -> float:
    if abs(t1 - t0) <= 1e-10:
        return NormalizeAngle(a0)
    start = NormalizeAngle(a0)
    delta = NormalizeAngle(a1) - start
    if delta > 3.141592653589793:
        delta -= 2.0 * 3.141592653589793
    elif delta < -3.141592653589793:
        delta += 2.0 * 3.141592653589793
    return NormalizeAngle(start + (t - t0) / (t1 - t0) * delta)


def _cos(theta: float) -> float:
    import math
    return math.cos(theta or 0.0)


def _sin(theta: float) -> float:
    import math
    return math.sin(theta or 0.0)
