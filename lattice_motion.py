"""CommonRoad obstacles for one lattice cycle.

Constant-speed actors and recorded scenario obstacles become a
``StaticObstacle`` or a ``DynamicObstacle`` with a ``TrajectoryPrediction``.
The sampler still reads rectangle corners at each time step. Ego uses
``RectObstacleShape.origin_x_shift`` so the rectangle is centered on the
geometric center while the state sits on the rear axle.
"""

from __future__ import annotations

from bisect import bisect_left
from typing import List, Optional, Sequence, Tuple

import numpy as np

from common.geometry_utils import NormalizeAngle
from commonroad.common.util import make_valid_orientation
from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
from commonroad.geometry.occupancy.rect_occupancy import RectOccupancy
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory

Sample = Tuple[float, float, float, float, float]
_NEXT_OBSTACLE_ID = 1_000_000


def obstacle_from_actor(actor, obstacle_id: Optional[int] = None):
    """Constant-speed rectangle along the actor's current heading."""
    import config

    speed = 0.0 if actor.is_static else float(actor.v)
    if actor.is_static or abs(speed) < 1e-3:
        return obstacle_from_samples(
            _obstacle_id(obstacle_id),
            actor.length,
            actor.width,
            [(0.0, float(actor.x), float(actor.y), float(actor.theta), 0.0)],
        )
    dt = config.FLAGS_trajectory_time_resolution
    horizon = config.FLAGS_trajectory_time_length
    samples = []
    t = 0.0
    while t <= horizon + 1e-9:
        samples.append(
            (
                t,
                float(actor.x) + speed * _cos(actor.theta) * t,
                float(actor.y) + speed * _sin(actor.theta) * t,
                float(actor.theta),
                speed,
            )
        )
        t += dt
    return obstacle_from_samples(_obstacle_id(obstacle_id), actor.length, actor.width, samples)


def obstacle_from_samples(obstacle_id: int, length: float, width: float, samples: Sequence[Sample]):
    """One sample is ``(t, x, y, theta, v)``. Gaps are filled at the planner time step."""
    if not samples:
        raise ValueError("an obstacle needs at least one sample")
    ordered = sorted(samples, key=lambda sample: sample[0])
    shape = RectObstacleShape(width=float(width), length=float(length))
    states = _states_on_planner_grid(ordered)
    initial = _initial_state(states[0])
    if len(states) < 2:
        return StaticObstacle(int(obstacle_id), ObstacleType.CAR, shape, initial)
    prediction = TrajectoryPrediction(Trajectory(1, states[1:]), shape)
    return DynamicObstacle(int(obstacle_id), ObstacleType.CAR, shape, initial, prediction)


def is_moving(obstacle) -> bool:
    prediction = getattr(obstacle, "prediction", None)
    return prediction is not None and int(prediction.final_time_step) > int(obstacle.initial_state.time_step)


def state_at(obstacle, t: float):
    """State at ``t`` seconds, held at the first or last recorded state outside the horizon."""
    if not is_moving(obstacle):
        return obstacle.initial_state
    step = min(max(_step(t), int(obstacle.initial_state.time_step)), int(obstacle.prediction.final_time_step))
    state = obstacle.state_at_time(step)
    return obstacle.initial_state if state is None else state


def state_in_horizon(obstacle, t: float):
    """Recorded state at ``t``, or ``None`` outside the prediction interval."""
    if not is_moving(obstacle):
        return None
    step = _step(t)
    initial = int(obstacle.initial_state.time_step)
    final = int(obstacle.prediction.final_time_step)
    if step < initial or step > final:
        return None
    return obstacle.state_at_time(step)


def corners_at(obstacle, t: float) -> List[Tuple[float, float]]:
    vertices = occupancy_at(obstacle, t).vertices
    return list(vertices[:-1])


def occupancy_at(obstacle, t: float, length_extra: float = 0.0, width_extra: float = 0.0) -> RectOccupancy:
    state = state_at(obstacle, t)
    shape = obstacle.obstacle_shape
    if length_extra or width_extra:
        shape = RectObstacleShape(
            width=float(shape.width) + float(width_extra),
            length=float(shape.length) + float(length_extra),
        )
    return shape.compute_occupancy_for_state(state)


def ego_occupancy(x: float, y: float, theta: float, length: float, width: float, back_edge_to_center: float) -> RectOccupancy:
    """Rectangle centered on the geometric center, not the rear axle."""
    shape = RectObstacleShape(
        width=float(width),
        length=float(length),
        origin_x_shift=float(back_edge_to_center) - float(length) / 2.0,
    )
    state = CustomState(
        position=np.array([float(x), float(y)]),
        orientation=make_valid_orientation(float(theta or 0.0)),
        time_step=0,
    )
    return shape.compute_occupancy_for_state(state)


def rectangles_overlap(left: RectOccupancy, right: RectOccupancy) -> bool:
    return left.shapely_object.intersects(right.shapely_object)


def _states_on_planner_grid(samples: Sequence[Sample]) -> list:
    import config

    dt = float(config.FLAGS_trajectory_time_resolution)
    start = float(samples[0][0])
    end = float(samples[-1][0])
    count = int(round((end - start) / dt)) if len(samples) > 1 else 0
    states = []
    for index in range(count + 1):
        _t, x, y, theta, speed = _sample_at(samples, start + index * dt)
        states.append(
            CustomState(
                time_step=index,
                position=np.array([x, y]),
                orientation=make_valid_orientation(theta),
                velocity=float(speed),
            )
        )
    return states


def _initial_state(state: CustomState) -> InitialState:
    return InitialState(
        time_step=0,
        position=np.array(state.position, dtype=float),
        orientation=float(state.orientation),
        velocity=float(state.velocity),
    )


def _sample_at(samples: Sequence[Sample], t: float) -> Sample:
    if t <= samples[0][0]:
        return samples[0]
    if t >= samples[-1][0]:
        return samples[-1]
    times = [sample[0] for sample in samples]
    index = bisect_left(times, t)
    left = samples[index - 1]
    right = samples[index]
    if abs(right[0] - left[0]) < 1e-9:
        return right
    return (
        t,
        _lerp(left[1], left[0], right[1], right[0], t),
        _lerp(left[2], left[0], right[2], right[0], t),
        _angle_lerp(left[3], left[0], right[3], right[0], t),
        _lerp(left[4], left[0], right[4], right[0], t),
    )


def _step(t: float) -> int:
    import config

    dt = float(config.FLAGS_trajectory_time_resolution)
    return int(t / dt + 0.5)


def _obstacle_id(obstacle_id: Optional[int]) -> int:
    global _NEXT_OBSTACLE_ID
    if obstacle_id is not None:
        return int(obstacle_id)
    _NEXT_OBSTACLE_ID += 1
    return _NEXT_OBSTACLE_ID


def _lerp(a0: float, t0: float, a1: float, t1: float, t: float) -> float:
    if abs(t1 - t0) <= 1e-10:
        return float(a0)
    return float(a0) + (float(a1) - float(a0)) * (t - t0) / (t1 - t0)


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
