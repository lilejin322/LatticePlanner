"""Classify one lattice plan on a fetched CommonRoad scenario.

The polynomials, costs, and constraint checks are not modified. This module
only runs one cycle and names what that cycle did.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

import config
from lanelet_path import merged_lanelet, path_from_lanelet, translated_lanelet
from lattice_planner import ComputeInitFrenetState
from path_matcher import PathMatcher
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from scripts.commonroad_drive import (
    _MIN_REFERENCE_LENGTH_M,
    _actors_from_scenario,
    _ego_start,
    _lanelet_chain,
    _planner_obstacle,
    load_scenario,
)
from scripts.commonroad_scenarios import scenario_file
from scripts.lattice_commonroad import plan_on_lanelet

BLOWUP_LIMIT = 1.0e5
FAR_MATCH_M = 20.0

NUMERIC_BLOWUP = "numeric_blowup"
NEGATIVE_SPEED_ASSERT = "negative_speed_assert"
FAR_MATCH = "far_match"
CONTAINED = "contained"
NO_TRAJECTORY = "no_trajectory"
EXCEPTION = "exception"
LEFT_ROAD = "left_road"
ACCEL_BOUND = "accel_bound"
ACCEL_LOW = -6.0
ACCEL_HIGH = 4.0


@dataclass
class PlanProbe:
    scenario_id: str
    path: str
    kind: str
    detail: str
    trajectory_type: Optional[str] = None
    ego_x: Optional[float] = None
    ego_y: Optional[float] = None
    ego_v: Optional[float] = None
    init_s: Optional[tuple] = None
    init_d: Optional[tuple] = None
    first_x: Optional[float] = None
    first_y: Optional[float] = None
    first_v: Optional[float] = None
    first_distance_m: Optional[float] = None


def _prepare_recording(scenario, problems):
    """Lanelet, ego, and recorded actors for an already loaded scenario."""
    chain = _lanelet_chain(
        scenario,
        lanelet_id=None,
        planning_problems=problems,
        follow_successors=True,
        min_reference_length=_MIN_REFERENCE_LENGTH_M,
    )
    merged = merged_lanelet(chain)
    origin = np.asarray(merged.center_vertices[0], dtype=float)
    local = translated_lanelet(merged, origin)
    center = np.asarray(local.center_vertices, dtype=float)
    path_line = path_from_lanelet(local, config.FLAGS_default_cruise_speed)
    ego = _ego_start(origin, problems, None, center, use_planning_state=True)
    actors, linked = _actors_from_scenario(scenario, origin)
    return origin, local, path_line, ego, actors, linked


def _open_recording(path: Path):
    """Lanelet, ego, and recorded actors for one CommonRoad file."""
    scenario, problems = load_scenario(path)
    return (scenario, *_prepare_recording(scenario, problems))


def _classify_plan_error(scenario_id: str, path: Path, exc: Exception) -> PlanProbe:
    kind = (
        NEGATIVE_SPEED_ASSERT
        if isinstance(exc, AssertionError) and "non-negative" in str(exc)
        else EXCEPTION
    )
    return PlanProbe(scenario_id, str(path), kind, f"{type(exc).__name__}: {exc}")


def probe_loaded(scenario, problems, scenario_id: str, source: Path | str = "") -> PlanProbe:
    """Plan once on an already loaded scenario and classify the result."""
    logging.disable(logging.CRITICAL)
    try:
        origin, local, path_line, ego, actors, linked = _prepare_recording(scenario, problems)
        obstacles = [
            _planner_obstacle(actor, linked[actor.actor_id], 0.0, origin, float(scenario.dt))
            for actor in actors
        ]
        start = TrajectoryPoint(
            path_point=PathPoint(
                x=ego.x,
                y=ego.y,
                theta=ego.theta,
                kappa=ego.kappa,
                s=0.0,
                dkappa=0.0,
            ),
            v=max(0.0, ego.v),
            a=ego.a,
            relative_time=0.0,
        )
        matched = PathMatcher.MatchToPath(path_line.points, ego.x, ego.y)
        init_s, init_d = ComputeInitFrenetState(matched, start)
        ok, trajectory, type_name, _unused = plan_on_lanelet(
            local, ego, actors, path=path_line, obstacles=obstacles
        )
    except Exception as exc:
        return _classify_plan_error(scenario_id, Path(source), exc)

    probe = PlanProbe(
        scenario_id,
        str(source),
        CONTAINED,
        type_name if ok else "plan returned no trajectory",
        trajectory_type=type_name,
        ego_x=ego.x,
        ego_y=ego.y,
        ego_v=ego.v,
        init_s=tuple(init_s),
        init_d=tuple(init_d),
    )
    if trajectory is None or len(trajectory) == 0:
        probe.kind = NO_TRAJECTORY
        return probe
    committed = trajectory.Evaluate(min(0.1, trajectory.GetTemporalLength()))
    probe.first_x = committed.path_point.x
    probe.first_y = committed.path_point.y
    probe.first_v = committed.v
    probe.first_distance_m = math.hypot(
        (probe.first_x or 0.0) - ego.x,
        (probe.first_y or 0.0) - ego.y,
    )
    samples = (trajectory[0], committed)
    magnitude = 0.0
    for point in samples:
        magnitude = max(
            magnitude,
            abs(point.path_point.x or 0.0),
            abs(point.path_point.y or 0.0),
            abs(point.v or 0.0),
        )
    if magnitude > BLOWUP_LIMIT:
        probe.kind = NUMERIC_BLOWUP
        probe.detail = (
            f"{type_name} committed point ({probe.first_x:.3g}, {probe.first_y:.3g}) "
            f"v={probe.first_v:.3g}"
        )
    elif probe.first_distance_m > FAR_MATCH_M:
        probe.kind = FAR_MATCH
        probe.detail = (
            f"{type_name} committed point is {probe.first_distance_m:.1f}m from the ego "
            f"at ({ego.x:.2f}, {ego.y:.2f})"
        )
    else:
        probe.kind = CONTAINED
        probe.detail = (
            f"{type_name} committed point {probe.first_distance_m:.2f}m from the ego, "
            f"v={probe.first_v:.2f}"
        )
    return probe


def probe_file(path: Path) -> PlanProbe:
    """Plan once from the scenario's planning problem and classify the result."""
    logging.disable(logging.CRITICAL)
    try:
        scenario, problems = load_scenario(path)
    except Exception as exc:
        return _classify_plan_error(path.stem, path, exc)
    return probe_loaded(scenario, problems, path.stem, path)


def probe_scenario(scenario_id: str) -> PlanProbe:
    return probe_file(scenario_file(scenario_id))


def commit_chain(scenario_id: str, horizon_s: float = 1.0) -> PlanProbe:
    """Replan from each committed point and classify those trajectories.

    The next planning init is the point one planning period along the trajectory
    just returned. Recorded vehicles are read from the file at that time. The
    kind is a property of a returned trajectory: a committed point that jumps
    away from its own init, or a commit-to-commit speed change outside the
    longitudinal acceleration bounds.
    """
    logging.disable(logging.CRITICAL)
    path = scenario_file(scenario_id)
    try:
        scenario, origin, local, path_line, ego, actors, linked = _open_recording(path)
    except Exception as exc:
        return _classify_plan_error(scenario_id, path, exc)

    from scripts.closed_loop_sim import _executed_state_is_usable
    from scripts.commonroad_drive import sync_actor

    dt = 1.0 / max(config.FLAGS_planning_loop_rate, 1e-3)
    scenario_dt = float(scenario.dt)
    t = 0.0
    low = None
    high = None
    worst = None
    cycles = 0
    while t < horizon_s - 1e-9:
        for actor in actors:
            sync_actor(actor, linked[actor.actor_id], t, origin, scenario_dt)
        obstacles = [
            _planner_obstacle(actor, linked[actor.actor_id], t, origin, scenario_dt)
            for actor in actors
        ]
        try:
            ok, trajectory, type_name, _unused = plan_on_lanelet(
                local, ego, actors, path=path_line, obstacles=obstacles
            )
        except Exception as exc:
            return _classify_plan_error(scenario_id, path, exc)
        cycles += 1
        if not ok or trajectory is None or len(trajectory) == 0:
            return PlanProbe(
                scenario_id,
                str(path),
                NO_TRAJECTORY,
                f"no trajectory at t={t:.2f}",
            )
        committed = trajectory.Evaluate(min(dt, trajectory.GetTemporalLength()))
        if not _executed_state_is_usable(ego, committed, dt):
            jump = math.hypot(
                (committed.path_point.x or 0.0) - ego.x,
                (committed.path_point.y or 0.0) - ego.y,
            )
            return PlanProbe(
                scenario_id,
                str(path),
                LEFT_ROAD,
                (
                    f"{type_name} at t={t:.2f} committed point is {jump:.3g} m "
                    f"from the init at ({ego.x:.2f}, {ego.y:.2f})"
                ),
                trajectory_type=type_name,
            )
        step = ((committed.v or 0.0) - ego.v) / dt
        sample_a = [point.a for point in trajectory if point.a is not None]
        if low is None or step < low:
            low = step
        if high is None or step > high:
            high = step
        excess = max(ACCEL_LOW - 0.05 - step, step - (ACCEL_HIGH + 0.05), 0.0)
        if worst is None or excess > worst[0]:
            worst = (excess, type_name, ego.v, committed.v or 0.0, sample_a, t)
        ego.x = committed.path_point.x or 0.0
        ego.y = committed.path_point.y or 0.0
        ego.theta = committed.path_point.theta or 0.0
        ego.kappa = committed.path_point.kappa or 0.0
        ego.v = committed.v or 0.0
        ego.a = committed.a or 0.0
        t += dt

    outside = low is not None and (low < ACCEL_LOW - 0.05 or high > ACCEL_HIGH + 0.05)
    if outside and worst is not None:
        _excess, type_name, v0, v1, sample_a, when = worst
        sample_span = (
            f"[{min(sample_a):.2f}, {max(sample_a):.2f}]"
            if sample_a
            else "n/a"
        )
        return PlanProbe(
            scenario_id,
            str(path),
            ACCEL_BOUND,
            (
                f"{type_name} at t={when:.2f} changes speed from {v0:.2f} to {v1:.2f} "
                f"over {dt:.1f}s ({(v1 - v0) / dt:.2f} m/s^2); sample a stays in {sample_span}"
            ),
            trajectory_type=type_name,
        )
    return PlanProbe(
        scenario_id,
        str(path),
        CONTAINED,
        f"{cycles} commits, speed change in [{low}, {high}] m/s^2",
    )


def executed_accel_span(result) -> tuple[float | None, float | None]:
    """Min and max acceleration of the committed closed-loop trace."""
    accels = []
    for left, right in zip(result.ego_trace, result.ego_trace[1:]):
        dt = right.t - left.t
        if dt > 1e-6:
            accels.append((right.v - left.v) / dt)
    if not accels:
        return None, None
    return min(accels), max(accels)


def drive_scenario(scenario_id: str, horizon_s: float = 1.0):
    """Run a short closed loop. Returns the ``SimResult``."""
    logging.disable(logging.CRITICAL)
    from scripts.commonroad_drive import load_scenario, run_loaded_scenario

    scenario, problems = load_scenario(scenario_file(scenario_id))
    return run_loaded_scenario(scenario, planning_problems=problems, horizon_s=horizon_s)
