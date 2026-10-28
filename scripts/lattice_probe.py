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


def probe_file(path: Path) -> PlanProbe:
    """Plan once from the scenario's planning problem and classify the result."""
    logging.disable(logging.CRITICAL)
    scenario_id = path.stem
    try:
        scenario, problems = load_scenario(path)
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
    except AssertionError as exc:
        kind = NEGATIVE_SPEED_ASSERT if "non-negative" in str(exc) else EXCEPTION
        return PlanProbe(scenario_id, str(path), kind, f"{type(exc).__name__}: {exc}")
    except Exception as exc:
        return PlanProbe(scenario_id, str(path), EXCEPTION, f"{type(exc).__name__}: {exc}")

    probe = PlanProbe(
        scenario_id,
        str(path),
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


def probe_scenario(scenario_id: str) -> PlanProbe:
    return probe_file(scenario_file(scenario_id))


def drive_scenario(scenario_id: str, horizon_s: float = 1.0):
    """Run a short closed loop. Returns the ``SimResult``."""
    logging.disable(logging.CRITICAL)
    from scripts.commonroad_drive import load_scenario, run_loaded_scenario

    scenario, problems = load_scenario(scenario_file(scenario_id))
    return run_loaded_scenario(scenario, planning_problems=problems, horizon_s=horizon_s)
