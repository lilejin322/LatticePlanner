"""Closed-loop roads taken from the converted Apollo map.

The geometry is the Borregas Ave CommonRoad file written by the existing
converter. Lanelets are not created here. An actor's ``x`` is station along
the chosen centerline and ``y`` is the lateral offset, positive to the left.
Speed stays constant. This module does not manage traffic.
"""

from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from scripts.closed_loop_sim import Actor
from scripts.commonroad_drive import (
    _longest_lanelet,
    load_scenario,
    occupancy_clearance,
)

_MAP_NAME = "borregas_ave"
_CACHE: dict = {}


@dataclass
class OfficialFrame:
    scenario: object
    lanelet: object
    center: np.ndarray
    stations: np.ndarray
    origin_x: float
    origin_y: float
    lanelet_ids: List[int]
    neighbor_xy: Optional[np.ndarray]
    length: float


@dataclass
class _Placed:
    actor: Actor
    s0: float
    lateral: float
    speed: float
    static: bool


@dataclass
class OfficialDrive:
    frame: OfficialFrame
    previous_map: object
    placed: List[_Placed]
    sync_fn: object
    collision_fn: object

    def close(self) -> None:
        return None

    def annotate(self, result):
        frame = self.frame
        stations = []
        laterals = []
        for sample in result.ego_trace:
            station, lateral = project_station(frame, sample.x, sample.y)
            stations.append(station)
            laterals.append(lateral)
        if stations:
            result.ego_x0 = stations[0]
            result.ego_x1 = stations[-1]
            result.max_abs_y = max(abs(value) for value in laterals)
        result.ref_x = [float(point[0]) for point in frame.center]
        result.ref_y = [float(point[1]) for point in frame.center]
        if frame.neighbor_xy is not None and len(frame.neighbor_xy):
            result.other_ref_x = [float(point[0]) for point in frame.neighbor_xy]
            result.other_ref_y = [float(point[1]) for point in frame.neighbor_xy]
        result.origin_x = frame.origin_x
        result.origin_y = frame.origin_y
        result.lanelet_ids = list(frame.lanelet_ids)
        result.commonroad_scenario = frame.scenario
        return result


def open_official_drive(
    role: str,
    ego: Actor,
    actors: Sequence[Actor],
    *,
    horizon_s: float,
) -> OfficialDrive:
    """Snap ``ego`` onto the official lanelet and follow actors along it.

    ``role`` is ``through``, the longest loaded lanelet.
    """
    frame = official_frame(role)
    previous = None
    placed = []
    for actor in actors:
        placed.append(
            _Placed(
                actor=actor,
                s0=float(actor.x),
                lateral=float(actor.y),
                speed=0.0 if actor.is_static else float(actor.v),
                static=bool(actor.is_static) or abs(actor.v) < 1e-3,
            )
        )
        _put(frame, actor, placed[-1].s0, placed[-1].lateral, 0.0 if placed[-1].static else placed[-1].speed)
        actor.replay = True
        actor.is_static = placed[-1].static
    _put(frame, ego, float(ego.x), float(ego.y), float(ego.v))

    def sync_fn(t: float) -> None:
        for item in placed:
            station = item.s0 if item.static else item.s0 + item.speed * t
            speed = 0.0 if item.static else item.speed
            if station >= frame.length:
                station = frame.length
                speed = 0.0
            _put(frame, item.actor, station, item.lateral, speed)
            item.actor.replay = True
            item.actor.is_static = item.static

    obstacles = _commonroad_actors(placed, frame, horizon_s)

    def collision_fn(_planner_obstacles, executed, t: float):
        return occupancy_clearance(
            executed,
            obstacles,
            t0=t,
            scenario_dt=float(frame.scenario.dt),
            origin=np.array([frame.origin_x, frame.origin_y]),
        )

    return OfficialDrive(
        frame=frame,
        previous_map=previous,
        placed=placed,
        sync_fn=sync_fn,
        collision_fn=collision_fn,
    )


def official_frame(role: str) -> OfficialFrame:
    if role != "through":
        raise ValueError(f"unknown official-map role {role!r}")
    cached = _CACHE.get(role)
    if cached is not None:
        return cached
    scenario, _problems = _official_scenario()
    lanelet = _longest_lanelet(scenario.lanelet_network)
    frame = _build_frame(scenario, lanelet)
    _CACHE[role] = frame
    return frame


def project_station(frame: OfficialFrame, x: float, y: float) -> Tuple[float, float]:
    best_distance = None
    best = (0.0, 0.0)
    center = frame.center
    stations = frame.stations
    for index in range(len(center) - 1):
        ax, ay = center[index]
        bx, by = center[index + 1]
        dx = float(bx - ax)
        dy = float(by - ay)
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-12:
            continue
        raw = ((x - ax) * dx + (y - ay) * dy) / length_sq
        ratio = min(max(raw, 0.0), 1.0)
        px = ax + ratio * dx
        py = ay + ratio * dy
        distance = (x - px) ** 2 + (y - py) ** 2
        if best_distance is None or distance < best_distance:
            cross = dx * (y - ay) - dy * (x - ax)
            lateral = cross / math.sqrt(length_sq)
            station = float(stations[index] + ratio * (stations[index + 1] - stations[index]))
            best_distance = distance
            best = (station, lateral)
    return best


def _official_scenario():
    if "scenario" in _CACHE:
        return _CACHE["scenario"]
    from apollo_map.to_commonroad import convert_map

    with tempfile.TemporaryDirectory() as output_dir:
        scenario, problems = load_scenario(convert_map(_MAP_NAME, output_dir))
    _CACHE["scenario"] = (scenario, problems)
    return _CACHE["scenario"]


def _build_frame(scenario, lanelet) -> OfficialFrame:
    from lanelet_path import translated_lanelet

    origin = np.asarray(lanelet.center_vertices[0], dtype=float)
    local = translated_lanelet(lanelet, origin)
    center = np.asarray(local.center_vertices, dtype=float)
    stations = np.asarray(local.distance, dtype=float)
    return OfficialFrame(
        scenario=scenario,
        lanelet=local,
        center=center,
        stations=stations,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        lanelet_ids=[lanelet.lanelet_id],
        neighbor_xy=None,
        length=float(stations[-1]),
    )


def _put(frame: OfficialFrame, actor: Actor, station: float, lateral: float, speed: float) -> None:
    x, y, theta = pose_on_center(frame, station, lateral)
    actor.x = x
    actor.y = y
    actor.theta = theta
    actor.kappa = 0.0
    actor.v = speed


def pose_on_center(frame: OfficialFrame, station: float, lateral: float) -> Tuple[float, float, float]:
    length = float(frame.stations[-1])
    station = min(max(float(station), 0.0), length)
    center = frame.center
    if station >= length - 1e-9:
        point = center[-1]
        delta = center[-1] - center[-2]
    else:
        point, _right, _left, index = frame.lanelet.interpolate_position(station)
        nxt = min(index + 1, len(center) - 1)
        delta = center[nxt] - center[index]
    theta = math.atan2(float(delta[1]), float(delta[0]))
    x = float(point[0]) + lateral * -math.sin(theta)
    y = float(point[1]) + lateral * math.cos(theta)
    return x, y, theta


def _commonroad_actors(placed: Sequence[_Placed], frame: OfficialFrame, horizon_s: float):
    from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
    from commonroad.prediction.prediction import TrajectoryPrediction
    from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
    from commonroad.scenario.state import CustomState, InitialState
    from commonroad.scenario.trajectory import Trajectory

    dt = float(frame.scenario.dt)
    last_step = max(1, int(math.floor(horizon_s / dt + 0.5)))
    obstacles = []
    for index, item in enumerate(placed):
        shape = RectObstacleShape(width=float(item.actor.width), length=float(item.actor.length))
        poses = [_world_pose(frame, item, step * dt) for step in range(last_step + 1)]
        x0, y0, theta0, speed0 = poses[0]
        initial = InitialState(
            time_step=0,
            position=np.array([x0, y0]),
            orientation=theta0,
            velocity=speed0,
        )
        obstacle_id = 1_000_000 + index
        if item.static:
            obstacles.append(StaticObstacle(obstacle_id, ObstacleType.CAR, shape, initial))
            continue
        states = [
            CustomState(
                time_step=step,
                position=np.array([x, y]),
                orientation=theta,
                velocity=speed,
            )
            for step, (x, y, theta, speed) in enumerate(poses[1:], start=1)
        ]
        obstacles.append(
            DynamicObstacle(
                obstacle_id,
                ObstacleType.CAR,
                shape,
                initial,
                TrajectoryPrediction(Trajectory(1, states), shape),
            )
        )
    return obstacles


def _world_pose(frame: OfficialFrame, item: _Placed, t: float):
    from scripts.commonroad_drive import _valid_orientation

    station = item.s0 if item.static else item.s0 + item.speed * t
    if station >= frame.length:
        station = frame.length
        speed = 0.0
    else:
        speed = 0.0 if item.static else item.speed
    x, y, theta = pose_on_center(frame, station, item.lateral)
    return (
        x + frame.origin_x,
        y + frame.origin_y,
        _valid_orientation(theta),
        speed,
    )
