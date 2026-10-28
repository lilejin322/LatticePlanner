"""Drive the lattice closed loop on a CommonRoad scenario that already exists.

The scenario comes from ``CommonRoadFileReader`` or from the Apollo map
converter. This module does not build lanelets, obstacles, or a new scenario.
The planner sees the loaded centerline in a local frame. Obstacle poses are
copied from ``state_at_time``, and clearance uses each obstacle's occupancy
polygon. Playback draws those same lanelets and the driven ego; the ego is
not inserted into the scenario.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import shapely

import config
from scripts.closed_loop_sim import (
    Actor,
    SimResult,
    _obstacle_from_actor,
    _run_loop,
)

_SAMPLE_STEP_M = 1.0
_MIN_REFERENCE_LENGTH_M = 100.0
_MAX_SUCCESSOR_TURN_RAD = math.radians(75.0)


def load_scenario(path):
    """Read one existing CommonRoad file. Returns ``(scenario, planning_problems)``."""
    from commonroad.common.file_reader import CommonRoadFileReader

    return CommonRoadFileReader(str(path)).open()


def run_loaded_scenario(
    scenario,
    *,
    planning_problems=None,
    lanelet_id: Optional[int] = None,
    horizon_s: float = 3.0,
    init_v: Optional[float] = None,
    follow_successors: bool = True,
    min_reference_length: float = _MIN_REFERENCE_LENGTH_M,
) -> SimResult:
    """Replan along lanelets that are already in ``scenario``.

    ``lanelet_id`` selects one loaded lanelet. When it is omitted, a planning
    problem initial state selects the lanelet under that point, and otherwise
    the longest loaded lanelet is used. Successors already stored on those
    lanelets are appended while the centerline is shorter than
    ``min_reference_length``.
    """
    chain = _lanelet_chain(
        scenario,
        lanelet_id=lanelet_id,
        planning_problems=planning_problems,
        follow_successors=follow_successors,
        min_reference_length=min_reference_length,
    )
    from lanelet_path import merged_lanelet, path_from_lanelet, translated_lanelet

    merged = merged_lanelet(chain)
    origin = np.asarray(merged.center_vertices[0], dtype=float)
    local_lanelet = translated_lanelet(merged, origin)
    local_center = np.asarray(local_lanelet.center_vertices, dtype=float)
    path = path_from_lanelet(local_lanelet, config.FLAGS_default_cruise_speed)
    start = _ego_start(
        origin,
        planning_problems,
        init_v,
        local_center,
        use_planning_state=lanelet_id is None,
    )
    actors, linked = _actors_from_scenario(scenario, origin)
    scenario_obstacles = list(scenario.obstacles)
    scenario_dt = float(scenario.dt)
    clock = {"t": 0.0}

    def sync_fn(t: float) -> None:
        clock["t"] = t
        for actor in actors:
            sync_actor(actor, linked[actor.actor_id], t, origin, scenario_dt)

    from scripts.lattice_commonroad import plan_on_lanelet

    def plan_fn(ego_now: Actor, actors_now: List[Actor]):
        obstacles = [
            _planner_obstacle(actor, linked[actor.actor_id], clock["t"], origin, scenario_dt)
            for actor in actors_now
        ]
        ok, trajectory, type_name, _bound = plan_on_lanelet(
            local_lanelet,
            ego_now,
            actors_now,
            path=path,
            obstacles=obstacles,
        )
        return ok, trajectory, type_name, obstacles

    def collision_fn(_obstacles, executed, t: float):
        return occupancy_clearance(
            executed,
            scenario_obstacles,
            t0=t,
            scenario_dt=scenario_dt,
            origin=origin,
        )

    result = _run_loop(
        _run_name(scenario, chain),
        start,
        actors,
        plan_fn,
        horizon_s=horizon_s,
        sync_fn=sync_fn,
        collision_fn=collision_fn,
    )
    result.ref_x = [float(point[0]) for point in local_center]
    result.ref_y = [float(point[1]) for point in local_center]
    result.lanelet_ids = [lanelet.lanelet_id for lanelet in chain]
    result.origin_x = float(origin[0])
    result.origin_y = float(origin[1])
    result.commonroad_scenario = scenario
    return result


def sync_actor(actor: Actor, obstacle, t: float, origin, scenario_dt: float) -> None:
    """Copy one loaded obstacle pose at time ``t`` onto ``actor``."""
    state, held = _state_at(obstacle, _time_step(t, scenario_dt))
    pose = _pose_from_state(state, origin)
    if pose is None:
        return
    actor.x, actor.y, actor.theta, speed = pose
    actor.v = 0.0 if held else speed
    actor.a = 0.0


def occupancy_clearance(
    executed,
    obstacles,
    *,
    t0: float,
    scenario_dt: float,
    origin,
) -> Tuple[Optional[float], bool]:
    """Gap between the executed ego boxes and loaded occupancy polygons.

    ``executed`` is in the planner's local frame. ``origin`` maps it back onto
    the scenario. The scenario itself is not moved.
    """
    if executed is None or executed.NumOfPoints() == 0 or not obstacles:
        return None, False
    origin_xy = np.asarray(origin, dtype=float)
    best: Optional[float] = None
    collided = False
    for index in range(executed.NumOfPoints()):
        point = executed.TrajectoryPointAt(index)
        theta = _valid_orientation(point.path_point.theta or 0.0)
        relative = point.relative_time or 0.0
        step = _time_step(t0 + relative, scenario_dt)
        ego_polygon = _ego_polygon(point.path_point.x or 0.0, point.path_point.y or 0.0, theta, origin_xy)
        for obstacle in obstacles:
            occupancy = _occupancy_at(obstacle, step)
            polygon = None if occupancy is None else getattr(occupancy, "shapely_object", None)
            if polygon is None:
                continue
            distance = float(ego_polygon.distance(polygon))
            best = distance if best is None else min(best, distance)
            if distance <= 1e-6:
                collided = True
    return best, collided


def _lanelet_chain(scenario, *, lanelet_id, planning_problems, follow_successors, min_reference_length):
    network = scenario.lanelet_network
    start_state = _planning_state(planning_problems)
    if lanelet_id is None and start_state is not None:
        lanelet_id = _lanelet_id_at(network, start_state.position)
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
    while follow_successors and total < min_reference_length:
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


def _concat_boundaries(chain):
    return (
        _concat([lanelet.center_vertices for lanelet in chain]),
        _concat([lanelet.left_vertices for lanelet in chain]),
        _concat([lanelet.right_vertices for lanelet in chain]),
    )


def _concat(polylines: Sequence) -> np.ndarray:
    chunks = []
    for polyline in polylines:
        points = np.asarray(polyline, dtype=float)
        if len(points) == 0:
            continue
        if chunks and np.linalg.norm(chunks[-1][-1] - points[0]) < 1e-2:
            points = points[1:]
        if len(points):
            chunks.append(points)
    if not chunks:
        return np.zeros((0, 2))
    return np.vstack(chunks)


def _local_centerline(center: np.ndarray):
    samples, distances = _resample(center, _SAMPLE_STEP_M)
    origin = samples[0].copy()
    return samples - origin, origin, distances, float(distances[-1])


def _resample(points: np.ndarray, step: float) -> Tuple[np.ndarray, np.ndarray]:
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if float(segment.sum()) < 1e-6:
        raise ValueError("loaded centerline has no length")
    station = np.concatenate([[0.0], np.cumsum(segment)])
    total = float(station[-1])
    samples = np.arange(0.0, total, step)
    if samples.size == 0 or total - float(samples[-1]) > 1e-4:
        samples = np.append(samples, total)
    sampled = np.column_stack(
        (
            np.interp(samples, station, points[:, 0]),
            np.interp(samples, station, points[:, 1]),
        )
    )
    return sampled, samples


def _boundary_widths(center, left, right, stations) -> List[Tuple[float, float]]:
    if len(left) < 2 or len(right) < 2:
        raise ValueError("loaded lanelet is missing a boundary")
    left_line = shapely.LineString(left)
    right_line = shapely.LineString(right)
    widths = []
    for point in _points_at(center, stations):
        here = shapely.Point(float(point[0]), float(point[1]))
        widths.append((float(left_line.distance(here)), float(right_line.distance(here))))
    return widths


def _points_at(points: np.ndarray, stations: np.ndarray) -> np.ndarray:
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    station = np.concatenate([[0.0], np.cumsum(segment)])
    return np.column_stack(
        (
            np.interp(stations, station, points[:, 0]),
            np.interp(stations, station, points[:, 1]),
        )
    )


def _ego_start(origin, planning_problems, init_v, local_center, *, use_planning_state: bool) -> Actor:
    state = _planning_state(planning_problems) if use_planning_state else None
    if state is not None:
        position = _position_of(state)
        x = float(position[0] - origin[0])
        y = float(position[1] - origin[1])
        theta = _float_attr(state, "orientation", _start_heading(local_center))
        speed = init_v if init_v is not None else _float_attr(state, "velocity", 5.0)
    else:
        x = float(local_center[0, 0])
        y = float(local_center[0, 1])
        theta = _start_heading(local_center)
        speed = 5.0 if init_v is None else float(init_v)
    return Actor("ego", x=x, y=y, theta=theta, v=max(0.0, speed), kappa=_start_kappa(local_center))


def _actors_from_scenario(scenario, origin):
    from commonroad.scenario.obstacle import StaticObstacle

    actors = []
    linked = {}
    for obstacle in list(scenario.static_obstacles) + list(scenario.dynamic_obstacles):
        if _position_of(obstacle.initial_state) is None:
            continue
        length, width = _box_size(obstacle)
        prediction = getattr(obstacle, "prediction", None)
        actor = Actor(
            actor_id=f"cr_{obstacle.obstacle_id}",
            length=length,
            width=width,
            is_static=isinstance(obstacle, StaticObstacle) or prediction is None,
            replay=True,
        )
        sync_actor(actor, obstacle, 0.0, origin, float(scenario.dt))
        actors.append(actor)
        linked[actor.actor_id] = obstacle
    return actors, linked


def _planner_obstacle(actor: Actor, obstacle, t0: float, origin, scenario_dt: float):
    if actor.is_static:
        return _obstacle_from_actor(actor)
    from common.obstacle import Obstacle
    from protoclass.adc_trajectory import Point3D
    from protoclass.path_point import PathPoint
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType
    from protoclass.trajectory import Trajectory
    from protoclass.trajectory_point import TrajectoryPoint

    sample_dt = config.FLAGS_trajectory_time_resolution
    horizon = config.FLAGS_trajectory_time_length
    points = []
    relative = 0.0
    while relative <= horizon + 1e-9:
        state, held = _state_at(obstacle, _time_step(t0 + relative, scenario_dt))
        pose = _pose_from_state(state, origin)
        if pose is None:
            break
        x, y, theta, speed = pose
        points.append(
            TrajectoryPoint(
                path_point=PathPoint(
                    x=x,
                    y=y,
                    z=0.0,
                    theta=theta,
                    kappa=0.0,
                    s=0.0,
                    dkappa=0.0,
                    ddkappa=0.0,
                ),
                v=0.0 if held else speed,
                a=0.0,
                relative_time=relative,
            )
        )
        relative += sample_dt
    if len(points) < 2:
        return _obstacle_from_actor(actor)
    perception = PerceptionObstacle(
        id=int(obstacle.obstacle_id),
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=actor.x, y=actor.y, z=0.0),
        velocity=Point3D(
            x=actor.v * math.cos(actor.theta),
            y=actor.v * math.sin(actor.theta),
            z=0.0,
        ),
        length=actor.length,
        width=actor.width,
        height=1.5,
        theta=actor.theta,
    )
    return Obstacle(
        actor.actor_id,
        perception,
        is_static=False,
        trajectory=Trajectory(trajectory_point=points),
    )


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


def _occupancy_at(obstacle, step: int):
    if not hasattr(obstacle, "occupancy_at_time"):
        return None
    occupancy = obstacle.occupancy_at_time(step)
    if occupancy is not None:
        return occupancy
    initial_step = obstacle.initial_state.time_step or 0
    if step < initial_step:
        return obstacle.occupancy_at_time(initial_step)
    prediction = getattr(obstacle, "prediction", None)
    final = getattr(prediction, "final_time_step", None) if prediction is not None else None
    if isinstance(final, int) and step > final:
        return obstacle.occupancy_at_time(final)
    return None


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


def _valid_orientation(angle: float) -> float:
    from commonroad.common.util import make_valid_orientation

    return float(make_valid_orientation(float(angle)))


def _ego_polygon(x: float, y: float, theta: float, origin: np.ndarray):
    from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy

    shift = config.EGO_VEHICLE_LENGTH / 2.0 - config.EGO_BACK_EDGE_TO_CENTER
    center_x = float(x) + float(origin[0]) + shift * math.cos(theta)
    center_y = float(y) + float(origin[1]) + shift * math.sin(theta)
    return RectOccupancy(
        rect_center=shapely.Point(center_x, center_y),
        width=config.EGO_VEHICLE_WIDTH,
        length=config.EGO_VEHICLE_LENGTH,
        orientation=theta,
    ).shapely_object


def _run_name(scenario, chain) -> str:
    scenario_id = getattr(scenario, "scenario_id", "scenario")
    ids = "-".join(str(lanelet.lanelet_id) for lanelet in chain[:4])
    return f"{scenario_id}:lanelet_{ids}"


def ego_obstacle_from_drive(result: SimResult):
    """Ego as a CommonRoad dynamic obstacle in the loaded scenario's coordinates.

    The obstacle is not added to the scenario. Its position is the geometric
    center, shifted forward from the planner's rear-axle trace.
    """
    from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
    from commonroad.prediction.prediction import TrajectoryPrediction
    from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
    from commonroad.scenario.state import CustomState, InitialState
    from commonroad.scenario.trajectory import Trajectory

    scenario = result.commonroad_scenario
    if scenario is None:
        raise ValueError("this drive has no loaded CommonRoad scenario")
    if not result.ego_trace:
        raise ValueError("this drive has no ego trace")
    scenario_dt = float(scenario.dt)
    last_step = _time_step(result.ego_trace[-1].t, scenario_dt)
    poses = [_pose_at(result.ego_trace, step * scenario_dt) for step in range(last_step + 1)]
    centers = [_ego_center(pose, result.origin_x, result.origin_y) for pose in poses]
    shape = RectObstacleShape(width=config.EGO_VEHICLE_WIDTH, length=config.EGO_VEHICLE_LENGTH)
    x0, y0, theta0, speed0 = centers[0]
    initial = InitialState(
        time_step=0,
        position=np.array([x0, y0]),
        orientation=theta0,
        velocity=speed0,
    )
    prediction = None
    if len(centers) > 1:
        states = [
            CustomState(
                time_step=step,
                position=np.array([x, y]),
                orientation=theta,
                velocity=speed,
            )
            for step, (x, y, theta, speed) in enumerate(centers[1:], start=1)
        ]
        prediction = TrajectoryPrediction(Trajectory(1, states), shape)
    return DynamicObstacle(_fresh_obstacle_id(scenario), ObstacleType.CAR, shape, initial, prediction)


def render_loaded_frames(result: SimResult, steps: Optional[Sequence[int]] = None) -> List[np.ndarray]:
    """Draw the loaded lanelets and the driven ego at each scenario time step.

    ``steps`` selects scenario time steps. The default is every step of the
    driven trace. The scenario is not modified.
    """
    import matplotlib

    matplotlib.use("Agg", force=False)
    import matplotlib.pyplot as plt
    from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle
    from commonroad.visualization.draw_params import MPDrawParams
    from commonroad.visualization.mp_renderer import MPRenderer

    scenario = result.commonroad_scenario
    if scenario is None:
        raise ValueError("this drive has no loaded CommonRoad scenario")
    ego = ego_obstacle_from_drive(result)
    if steps is None:
        final = ego.initial_state.time_step
        if ego.prediction is not None:
            final = int(ego.prediction.final_time_step)
        steps = list(range(int(ego.initial_state.time_step), final + 1))
    limits = _playback_limits(ego, steps)
    params = MPDrawParams()
    params.dynamic_obstacle.trajectory.draw_trajectory = False
    params.dynamic_obstacle.history.draw_history = False
    params.dynamic_obstacle.draw_icon = False
    params.dynamic_obstacle.draw_signals = False
    params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#1f4e79"
    params.lanelet_network.draw_ids = list(result.lanelet_ids) or None

    figure, axes = plt.subplots(figsize=(8, 6), dpi=80)
    renderer = MPRenderer(draw_params=params, ax=axes, plot_limits=limits)
    frames: List[np.ndarray] = []
    try:
        for step in steps:
            params.time_begin = int(step)
            params.time_end = int(step)
            scenario.lanelet_network.draw(renderer, params.lanelet_network)
            for obstacle in scenario.obstacles:
                if isinstance(obstacle, DynamicObstacle):
                    obstacle.draw(renderer, params.dynamic_obstacle)
                elif isinstance(obstacle, StaticObstacle):
                    obstacle.draw(renderer, params.static_obstacle)
            ego.draw(renderer, params.dynamic_obstacle)
            renderer.render()
            figure.canvas.draw()
            frames.append(np.array(figure.canvas.buffer_rgba(), copy=True))
    finally:
        plt.close(figure)
    return frames


def write_loaded_gif(result: SimResult, path, *, steps: Optional[Sequence[int]] = None) -> int:
    """Write one GIF of the loaded scenario and the driven ego. Returns the frame count."""
    from pathlib import Path

    from PIL import Image

    frames = render_loaded_frames(result, steps)
    if not frames:
        raise ValueError("loaded drive produced no frames")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    images = [Image.fromarray(frame).convert("RGB") for frame in frames]
    scenario_dt = float(result.commonroad_scenario.dt)
    images[0].save(
        output,
        save_all=True,
        append_images=images[1:],
        duration=max(1, int(round(scenario_dt * 1000.0))),
        loop=0,
    )
    return len(images)


def _fresh_obstacle_id(scenario) -> int:
    used = {lanelet.lanelet_id for lanelet in scenario.lanelet_network.lanelets}
    used.update(obstacle.obstacle_id for obstacle in scenario.obstacles)
    return max(used, default=0) + 1


def _pose_at(trace, t: float):
    from scripts.closed_loop_sim import PoseSample

    if t <= trace[0].t:
        return trace[0]
    if t >= trace[-1].t:
        return trace[-1]
    for left, right in zip(trace, trace[1:]):
        if left.t <= t <= right.t:
            span = right.t - left.t
            if span <= 1e-9:
                return right
            ratio = (t - left.t) / span
            turn = math.atan2(math.sin(right.theta - left.theta), math.cos(right.theta - left.theta))
            return PoseSample(
                t=t,
                x=left.x + (right.x - left.x) * ratio,
                y=left.y + (right.y - left.y) * ratio,
                theta=left.theta + turn * ratio,
                v=left.v + (right.v - left.v) * ratio,
            )
    return trace[-1]


def _ego_center(pose, origin_x: float, origin_y: float):
    theta = _valid_orientation(pose.theta)
    shift = config.EGO_VEHICLE_LENGTH / 2.0 - config.EGO_BACK_EDGE_TO_CENTER
    return (
        pose.x + origin_x + shift * math.cos(theta),
        pose.y + origin_y + shift * math.sin(theta),
        theta,
        max(0.0, pose.v),
    )


def _playback_limits(ego, steps: Sequence[int]):
    xs = []
    ys = []
    for step in steps:
        occupancy = ego.occupancy_at_time(int(step))
        if occupancy is None:
            continue
        center = occupancy.center
        xs.append(float(center.x))
        ys.append(float(center.y))
    if not xs:
        raise ValueError("ego has no occupancy on the requested steps")
    margin = 25.0
    return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)
