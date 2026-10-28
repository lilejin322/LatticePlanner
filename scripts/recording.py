"""Read one CommonRoad file into the inputs of a lattice cycle.

The file already contains the lanelets and the recorded vehicles. This module
copies a centerline into a local frame and samples obstacle poses with
``state_at_time``. It does not build roads or move the vehicles on its own.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import shapely

import config

MIN_REFERENCE_LENGTH_M = 100.0
_MAX_SUCCESSOR_TURN_RAD = math.radians(75.0)


@dataclass
class Vehicle:
    """Pose of one recorded vehicle in the planner's local frame."""

    actor_id: str
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    a: float = 0.0
    kappa: float = 0.0
    length: float = 4.0
    width: float = 2.0
    is_static: bool = False


@dataclass
class Recording:
    """One loaded scenario, its local reference line, and the vehicles on it."""

    scenario: object
    origin: np.ndarray
    lanelet: object
    path: object
    ego: Vehicle
    actors: List[Vehicle] = field(default_factory=list)
    linked: Dict[str, object] = field(default_factory=dict)


def load_scenario(path):
    """Read one existing CommonRoad file. Returns ``(scenario, planning_problems)``."""
    from commonroad.common.file_reader import CommonRoadFileReader

    return CommonRoadFileReader(str(path)).open()


def open_recording(scenario, problems, *, min_reference_length: float = MIN_REFERENCE_LENGTH_M) -> Recording:
    """Reference line and vehicles for ``scenario``'s planning problem."""
    from lanelet_path import merged_lanelet, path_from_lanelet, translated_lanelet

    chain = _lanelet_chain(
        scenario,
        planning_problems=problems,
        min_reference_length=min_reference_length,
    )
    merged = merged_lanelet(chain)
    origin = np.asarray(merged.center_vertices[0], dtype=float)
    lanelet = translated_lanelet(merged, origin)
    center = np.asarray(lanelet.center_vertices, dtype=float)
    path = path_from_lanelet(lanelet, config.FLAGS_default_cruise_speed)
    ego = _ego_start(origin, problems, center)
    actors, linked = _actors_from_scenario(scenario, origin)
    return Recording(scenario, origin, lanelet, path, ego, actors, linked)


def sync_vehicles(recording: Recording, t: float) -> None:
    """Copy each recorded obstacle's pose at time ``t`` onto its vehicle."""
    dt = float(recording.scenario.dt)
    for actor in recording.actors:
        _sync_vehicle(actor, recording.linked[actor.actor_id], t, recording.origin, dt)


def obstacles_at(recording: Recording, t: float):
    """Planner obstacles for the recorded vehicles at time ``t``."""
    dt = float(recording.scenario.dt)
    return [
        obstacle_for(actor, recording.linked[actor.actor_id], t, recording.origin, dt)
        for actor in recording.actors
    ]


def obstacle_for(actor: Vehicle, obstacle, t0: float, origin, scenario_dt: float):
    """One planner obstacle, sampled from ``obstacle`` across the planning horizon."""
    from lattice_motion import obstacle_from_actor, obstacle_from_samples

    if actor.is_static:
        return obstacle_from_actor(actor, obstacle.obstacle_id)
    sample_dt = config.FLAGS_trajectory_time_resolution
    horizon = config.FLAGS_trajectory_time_length
    samples = []
    relative = 0.0
    while relative <= horizon + 1e-9:
        state, held = _state_at(obstacle, _time_step(t0 + relative, scenario_dt))
        pose = _pose_from_state(state, origin)
        if pose is None:
            break
        x, y, theta, speed = pose
        samples.append((relative, x, y, theta, 0.0 if held else speed))
        relative += sample_dt
    if len(samples) < 2:
        return obstacle_from_actor(actor, obstacle.obstacle_id)
    return obstacle_from_samples(obstacle.obstacle_id, actor.length, actor.width, samples)


def _lanelet_chain(scenario, *, planning_problems, min_reference_length: float):
    network = scenario.lanelet_network
    start_state = _planning_state(planning_problems)
    lanelet_id = None if start_state is None else _lanelet_id_at(network, start_state.position)
    if lanelet_id is None:
        start = _longest_lanelet(network)
    else:
        start = network.find_lanelet_by_id(int(lanelet_id))
        if start is None:
            raise ValueError(f"lanelet {lanelet_id} is not in the loaded scenario")
    if len(np.asarray(start.center_vertices)) < 2:
        raise ValueError(f"lanelet {start.lanelet_id} has no centerline")

    chain = [start]
    seen = {start.lanelet_id}
    total = _polyline_length(start.center_vertices)
    while total < min_reference_length:
        current = chain[-1]
        heading = _end_heading(current.center_vertices)
        candidates = []
        for successor_id in current.successor or []:
            if successor_id in seen:
                continue
            successor = network.find_lanelet_by_id(successor_id)
            if successor is None or len(np.asarray(successor.center_vertices)) < 2:
                continue
            turn = _angle_between(heading, _start_heading(successor.center_vertices))
            if turn <= _MAX_SUCCESSOR_TURN_RAD:
                candidates.append((turn, successor))
        if not candidates:
            break
        _turn, successor = min(candidates, key=lambda item: (item[0], item[1].lanelet_id))
        chain.append(successor)
        seen.add(successor.lanelet_id)
        total += _polyline_length(successor.center_vertices)
    return chain


def _longest_lanelet(network):
    lanelets = [
        lanelet
        for lanelet in network.lanelets
        if _polyline_length(lanelet.center_vertices) > 0.0
    ]
    if not lanelets:
        raise ValueError("loaded scenario has no lanelets")
    return max(lanelets, key=lambda lanelet: (_polyline_length(lanelet.center_vertices), -lanelet.lanelet_id))


def _lanelet_id_at(network, position) -> Optional[int]:
    point = np.asarray(position, dtype=float)
    try:
        matches = network.find_lanelet_by_position([point])
    except (AssertionError, TypeError, ValueError):
        matches = []
    if matches and matches[0]:
        return int(matches[0][0])
    nearest = None
    nearest_distance = float("inf")
    here = shapely.Point(float(point[0]), float(point[1]))
    for lanelet in network.lanelets:
        vertices = np.asarray(lanelet.center_vertices, dtype=float)
        if len(vertices) < 2:
            continue
        distance = float(shapely.LineString(vertices).distance(here))
        if distance < nearest_distance:
            nearest_distance = distance
            nearest = lanelet.lanelet_id
    return nearest


def _planning_state(planning_problems):
    problems = getattr(planning_problems, "planning_problem_dict", None)
    if not problems:
        return None
    state = next(iter(problems.values())).initial_state
    if _position_of(state) is None:
        return None
    return state


def _ego_start(origin, planning_problems, local_center) -> Vehicle:
    state = _planning_state(planning_problems)
    if state is not None:
        position = _position_of(state)
        x = float(position[0] - origin[0])
        y = float(position[1] - origin[1])
        theta = _float_attr(state, "orientation", _start_heading(local_center))
        speed = _float_attr(state, "velocity", 5.0)
    else:
        x = float(local_center[0, 0])
        y = float(local_center[0, 1])
        theta = _start_heading(local_center)
        speed = 5.0
    return Vehicle("ego", x=x, y=y, theta=theta, v=max(0.0, speed), kappa=_start_kappa(local_center))


def _actors_from_scenario(scenario, origin):
    from commonroad.scenario.obstacle import StaticObstacle

    actors = []
    linked = {}
    for obstacle in list(scenario.static_obstacles) + list(scenario.dynamic_obstacles):
        if _position_of(obstacle.initial_state) is None:
            continue
        length, width = _box_size(obstacle)
        prediction = getattr(obstacle, "prediction", None)
        actor = Vehicle(
            actor_id=f"cr_{obstacle.obstacle_id}",
            x=0.0,
            length=length,
            width=width,
            is_static=isinstance(obstacle, StaticObstacle) or prediction is None,
        )
        _sync_vehicle(actor, obstacle, 0.0, origin, float(scenario.dt))
        actors.append(actor)
        linked[actor.actor_id] = obstacle
    return actors, linked


def _sync_vehicle(actor: Vehicle, obstacle, t: float, origin, scenario_dt: float) -> None:
    state, held = _state_at(obstacle, _time_step(t, scenario_dt))
    pose = _pose_from_state(state, origin)
    if pose is None:
        return
    actor.x, actor.y, actor.theta, speed = pose
    actor.v = 0.0 if held else speed
    actor.a = 0.0


def _box_size(obstacle) -> Tuple[float, float]:
    shape = obstacle.obstacle_shape
    length = getattr(shape, "length", None)
    width = getattr(shape, "width", None)
    if isinstance(length, (int, float)) and isinstance(width, (int, float)):
        return float(length), float(width)
    occupancy = obstacle.occupancy_at_time(obstacle.initial_state.time_step or 0)
    polygon = None if occupancy is None else getattr(occupancy, "shapely_object", None)
    if polygon is None or polygon.is_empty:
        return 4.0, 2.0
    rectangle = polygon.minimum_rotated_rectangle
    coords = list(rectangle.exterior.coords)
    edges = [
        math.hypot(coords[index + 1][0] - coords[index][0], coords[index + 1][1] - coords[index][1])
        for index in range(len(coords) - 1)
    ]
    edges = [edge for edge in edges if edge > 1e-6]
    if len(edges) < 2:
        return 4.0, 2.0
    return max(edges), min(edges)


def _state_at(obstacle, step: int):
    state = obstacle.state_at_time(step)
    if state is not None and _position_of(state) is not None:
        return state, False
    initial = obstacle.initial_state
    initial_step = initial.time_step or 0
    if step <= initial_step and _position_of(initial) is not None:
        return initial, step != initial_step
    prediction = getattr(obstacle, "prediction", None)
    final = getattr(prediction, "final_time_step", None) if prediction is not None else None
    if isinstance(final, int) and step > final:
        held = obstacle.state_at_time(final)
        if held is not None and _position_of(held) is not None:
            return held, True
    if _position_of(initial) is not None:
        return initial, True
    return None, True


def _pose_from_state(state, origin):
    position = None if state is None else _position_of(state)
    if position is None:
        return None
    origin_xy = np.asarray(origin, dtype=float)
    return (
        float(position[0] - origin_xy[0]),
        float(position[1] - origin_xy[1]),
        _float_attr(state, "orientation", 0.0),
        _float_attr(state, "velocity", 0.0),
    )


def _position_of(state):
    position = getattr(state, "position", None)
    if position is None:
        return None
    array = np.asarray(position, dtype=float)
    if array.shape != (2,) or not np.all(np.isfinite(array)):
        return None
    return array


def _float_attr(state, name: str, default: float) -> float:
    value = getattr(state, name, None)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return default


def _time_step(t: float, dt: float) -> int:
    if dt <= 0.0:
        return 0
    return max(0, int(math.floor(float(t) / float(dt) + 0.5)))


def _polyline_length(vertices) -> float:
    points = np.asarray(vertices, dtype=float)
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _start_heading(vertices) -> float:
    points = np.asarray(vertices, dtype=float)
    return math.atan2(points[1, 1] - points[0, 1], points[1, 0] - points[0, 0])


def _end_heading(vertices) -> float:
    points = np.asarray(vertices, dtype=float)
    return math.atan2(points[-1, 1] - points[-2, 1], points[-1, 0] - points[-2, 0])


def _angle_between(left: float, right: float) -> float:
    return abs((right - left + math.pi) % (2.0 * math.pi) - math.pi)


def _start_kappa(points: np.ndarray) -> float:
    if len(points) < 3:
        return 0.0
    a, b, c = points[0], points[1], points[2]
    ab = float(np.linalg.norm(b - a))
    bc = float(np.linalg.norm(c - b))
    ca = float(np.linalg.norm(a - c))
    area2 = float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    if ab * bc * ca < 1e-6:
        return 0.0
    return 2.0 * area2 / (ab * bc * ca)
