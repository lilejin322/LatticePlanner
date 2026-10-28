"""Closed-loop roads taken from the converted Apollo map.

The geometry is the Borregas Ave CommonRoad file written by the existing
converter. Lanelets are not created here. An actor's ``x`` is station along
the chosen centerline and ``y`` is the lateral offset, positive to the left.
"""

from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import shapely

import config
from scripts.closed_loop_sim import Actor
from scripts.commonroad_drive import (
    _longest_lanelet,
    _polyline_length,
    _resample,
    load_scenario,
    occupancy_clearance,
)

_MAP_NAME = "borregas_ave"
_CACHE: dict = {}


@dataclass
class OfficialFrame:
    scenario: object
    hdmap: object
    reference_line: object
    lanes: object
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
        from common.hd_map import HDMapUtil

        HDMapUtil.SetBaseMap(self.previous_map)

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

    ``role`` is ``through`` for the long lane, ``borrow-left``, or ``borrow-right``.
    """
    from common.hd_map import HDMapUtil

    frame = official_frame(role)
    previous = HDMapUtil.BaseMap()
    HDMapUtil.SetBaseMap(frame.hdmap)
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
    cached = _CACHE.get(role)
    if cached is not None:
        return cached
    scenario, _problems = _official_scenario()
    if role == "through":
        lanelet = _longest_lanelet(scenario.lanelet_network)
        neighbor = None
        side = None
    elif role == "borrow-left":
        lanelet, neighbor = _parallel_pair(scenario, "left")
        side = "left"
    elif role == "borrow-right":
        lanelet, neighbor = _parallel_pair(scenario, "right")
        side = "right"
    else:
        raise ValueError(f"unknown official-map role {role!r}")
    frame = _build_frame(scenario, lanelet, neighbor, side)
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


def _parallel_pair(scenario, side: str):
    network = scenario.lanelet_network
    best = None
    best_key = None
    for lanelet in network.lanelets:
        if side == "left":
            neighbor_id = lanelet.adj_left if lanelet.adj_left_same_direction else None
        else:
            neighbor_id = lanelet.adj_right if lanelet.adj_right_same_direction else None
        if neighbor_id is None:
            continue
        neighbor = network.find_lanelet_by_id(neighbor_id)
        if neighbor is None:
            continue
        distances = _center_distances(lanelet.center_vertices, neighbor.center_vertices)
        if len(distances) < 2:
            continue
        median = float(np.median(distances))
        spread = float(max(distances) - min(distances))
        if median < 2.8 or median > 4.5 or spread > 1.0:
            continue
        key = (_polyline_length(lanelet.center_vertices), -spread, -lanelet.lanelet_id)
        if best_key is None or key > best_key:
            best_key = key
            best = (lanelet, neighbor)
    if best is None:
        raise ValueError(f"converted {_MAP_NAME} has no same-direction {side} neighbor")
    return best


def _center_distances(center, other) -> List[float]:
    line = shapely.LineString(np.asarray(other, dtype=float))
    return [
        float(line.distance(shapely.Point(float(point[0]), float(point[1]))))
        for point in np.asarray(center, dtype=float)
    ]


def _build_frame(scenario, lanelet, neighbor, side: Optional[str]) -> OfficialFrame:
    from common.hd_map import HDMap, HDMapUtil
    from common.lane_types import LaneSegment
    from common.path import Path as MapPath
    from common.route_segments import RouteSegments
    from reference_line import ReferenceLine

    center_world, stations = _resample(np.asarray(lanelet.center_vertices, dtype=float), 1.0)
    origin = center_world[0].copy()
    center = center_world - origin
    left_width = _distance_samples(center_world, lanelet.left_vertices)
    right_width = _distance_samples(center_world, lanelet.right_vertices)
    road_left = list(left_width)
    road_right = list(right_width)
    neighbor_local = None
    lanelet_ids = [lanelet.lanelet_id]
    left_ids: List[str] = []
    right_ids: List[str] = []
    hdmap = HDMap()
    if neighbor is not None and side is not None:
        neighbor_world, neighbor_stations = _resample(np.asarray(neighbor.center_vertices, dtype=float), 1.0)
        neighbor_local = neighbor_world - origin
        neighbor_id = f"cr_{neighbor.lanelet_id}"
        lanelet_ids.append(neighbor.lanelet_id)
        neighbor_left = _distance_samples(neighbor_world, neighbor.left_vertices)
        neighbor_right = _distance_samples(neighbor_world, neighbor.right_vertices)
        if side == "left":
            left_ids = [neighbor_id]
            road_left = _distance_samples(center_world, neighbor.left_vertices)
        else:
            right_ids = [neighbor_id]
            road_right = _distance_samples(center_world, neighbor.right_vertices)
        hdmap.AddLane(
            _apollo_lane(
                neighbor_id,
                neighbor_local,
                neighbor_stations,
                neighbor_left,
                neighbor_right,
                neighbor_left,
                neighbor_right,
                [],
                [],
            )
        )
    ego_id = f"cr_{lanelet.lanelet_id}"
    ego_lane = _apollo_lane(
        ego_id,
        center,
        stations,
        left_width,
        right_width,
        road_left,
        road_right,
        left_ids,
        right_ids,
    )
    lane_info = hdmap.AddLane(ego_lane)
    HDMapUtil.SetBaseMap(hdmap)
    end_s = float(lane_info.total_length)
    end_s = end_s - 1.0 if end_s > 2.0 else end_s * 0.5
    route = RouteSegments()
    route.SetIsOnSegment(True)
    route.SetId(ego_id)
    route.append(LaneSegment(lane_info, 0.0, end_s))
    reference = ReferenceLine(MapPath(route))
    return OfficialFrame(
        scenario=scenario,
        hdmap=hdmap,
        reference_line=reference,
        lanes=route,
        center=center,
        stations=stations,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        lanelet_ids=lanelet_ids,
        neighbor_xy=neighbor_local,
        length=float(stations[-1]),
    )


def _apollo_lane(
    lane_id,
    local_xy,
    stations,
    left_width,
    right_width,
    road_left,
    road_right,
    left_ids,
    right_ids,
):
    from protoclass.lane import Curve, CurveSegment, Lane, LaneSampleAssociation, LineSegment
    from protoclass.point_enu import PointENU

    count = min(
        len(local_xy),
        len(stations),
        len(left_width),
        len(right_width),
        len(road_left),
        len(road_right),
    )
    points = [PointENU(x=float(local_xy[index][0]), y=float(local_xy[index][1])) for index in range(count)]
    left_sample = []
    right_sample = []
    left_road = []
    right_road = []
    for index in range(count):
        station = float(stations[index])
        left_sample.append(LaneSampleAssociation(s=station, width=max(float(left_width[index]), 1e-3)))
        right_sample.append(LaneSampleAssociation(s=station, width=max(float(right_width[index]), 1e-3)))
        left_road.append(LaneSampleAssociation(s=station, width=max(float(road_left[index]), 1e-3)))
        right_road.append(LaneSampleAssociation(s=station, width=max(float(road_right[index]), 1e-3)))
    return Lane(
        id=Lane.Id(lane_id),
        central_curve=Curve(segment=[CurveSegment(curve_type=LineSegment(point=points))]),
        length=float(stations[count - 1]) if count else 0.0,
        speed_limit=config.FLAGS_default_cruise_speed,
        left_sample=left_sample,
        right_sample=right_sample,
        left_road_sample=left_road,
        right_road_sample=right_road,
        left_neighbor_forward_lane_id=[Lane.Id(item) for item in left_ids],
        right_neighbor_forward_lane_id=[Lane.Id(item) for item in right_ids],
        type=Lane.LaneType.CITY_DRIVING,
    )


def _distance_samples(center_world: np.ndarray, boundary) -> List[float]:
    line = shapely.LineString(np.asarray(boundary, dtype=float))
    return [
        float(line.distance(shapely.Point(float(point[0]), float(point[1]))))
        for point in center_world
    ]


def _put(frame: OfficialFrame, actor: Actor, station: float, lateral: float, speed: float) -> None:
    x, y, theta = pose_on_center(frame, station, lateral)
    actor.x = x
    actor.y = y
    actor.theta = theta
    actor.kappa = 0.0
    actor.v = speed


def pose_on_center(frame: OfficialFrame, station: float, lateral: float) -> Tuple[float, float, float]:
    stations = frame.stations
    center = frame.center
    station = min(max(float(station), 0.0), float(stations[-1]))
    index = int(np.searchsorted(stations, station, side="right") - 1)
    index = min(max(index, 0), len(center) - 2)
    span = float(stations[index + 1] - stations[index])
    ratio = 0.0 if span < 1e-9 else (station - float(stations[index])) / span
    point = center[index] * (1.0 - ratio) + center[index + 1] * ratio
    delta = center[index + 1] - center[index]
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
