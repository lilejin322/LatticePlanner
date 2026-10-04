"""
Lattice / OnLane scenario definitions (shared by run_lattice_scenario_cases.py and the demo).

Driving scenarios run closed-loop: replan every cycle from the executed state.
Decider cases stay single-call checks of the path stack; they do not publish a
driven trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple, Union

from scripts.closed_loop_sim import (
    Actor,
    SimResult,
    run_curved_road,
    run_lane_change_road,
    run_on_lane_road,
    run_path_bounds_road,
    run_straight_road,
)
from scripts.planner_test_fixtures import (
    build_center_lane_reference_line,
    build_lattice_plan_frame,
    build_static_obstacle,
    curved_arc_pose,
)

# Component checks still return this tuple. Driving cases return SimResult.
BuilderResult = Union[SimResult, Tuple]


@dataclass
class Scenario:
    name: str
    description: str
    category: str  # lattice | decider | on_lane | stress | overtake | lane_change | sim
    builder: Callable[[], BuilderResult]
    expect_ok: bool = True
    backup: Optional[bool] = None
    informational: bool = False


def _static(actor_id: str, x: float, y: float = 0.0) -> Actor:
    return Actor(actor_id, x=x, y=y, is_static=True)


def _finish(
    result: SimResult,
    *,
    min_travel: Optional[float] = None,
    min_abs_y: Optional[float] = None,
    max_abs_y: Optional[float] = None,
    forbid_fallback: bool = False,
) -> SimResult:
    if not result.ok:
        return result
    if forbid_fallback and result.fallback_cycles:
        result.ok = False
        result.reason = "used backup trajectory"
        return result
    traveled = result.ego_x1 - result.ego_x0
    if min_travel is not None and traveled < min_travel:
        result.ok = False
        result.reason = f"only traveled {traveled:.1f}m"
        return result
    if min_abs_y is not None and result.max_abs_y < min_abs_y:
        result.ok = False
        result.reason = f"lateral offset {result.max_abs_y:.2f}m stayed below {min_abs_y:.2f}m"
        return result
    if max_abs_y is not None and result.max_abs_y > max_abs_y:
        result.ok = False
        result.reason = f"lateral offset {result.max_abs_y:.2f}m exceeded {max_abs_y:.2f}m"
        return result
    return result


def _straight(
    name: str,
    *,
    ego_v: float = 1.0,
    ego_x: float = 0.0,
    ego_y: float = 0.0,
    actors: Optional[Sequence[Actor]] = None,
    blocking: Optional[str] = None,
    backup: Optional[bool] = None,
    horizon_s: float = 3.0,
    road_length: float = 200.0,
    min_travel: Optional[float] = None,
    max_abs_y: Optional[float] = None,
    forbid_fallback: bool = False,
) -> SimResult:
    result = run_straight_road(
        name,
        Actor("ego", x=ego_x, y=ego_y, v=ego_v),
        list(actors or []),
        horizon_s=horizon_s,
        road_length=road_length,
        backup=backup,
        blocking_actor_id=blocking,
    )
    return _finish(
        result,
        min_travel=min_travel,
        max_abs_y=max_abs_y,
        forbid_fallback=forbid_fallback,
    )


# --- lattice core (closed loop) ---


def s_open_road() -> SimResult:
    return _straight("open_road", min_travel=6.0, forbid_fallback=True)


def s_far_obstacle_stop() -> SimResult:
    return _straight(
        "far_obstacle_stop",
        actors=[_static("far_car", 35.0)],
        blocking="far_car",
        min_travel=2.0,
    )


def s_close_obstacle_no_backup() -> SimResult:
    return _straight(
        "close_obstacle_no_backup",
        actors=[_static("close_car", 8.0)],
        blocking="close_car",
        backup=False,
    )


def s_close_obstacle_with_backup() -> SimResult:
    return _straight(
        "close_obstacle_with_backup",
        actors=[_static("close_car", 8.0)],
        blocking="close_car",
        backup=True,
    )


def s_higher_speed_cruise() -> SimResult:
    return _straight("higher_speed_cruise", ego_v=5.0, min_travel=12.0, forbid_fallback=True)


def s_stopped_start() -> SimResult:
    return _straight("stopped_start", ego_v=0.1, min_travel=2.0, forbid_fallback=True)


def s_mid_lane_start() -> SimResult:
    return _straight("mid_lane_start", ego_v=3.0, ego_x=20.0, min_travel=6.0, forbid_fallback=True)


def s_lateral_offset_start() -> SimResult:
    return _straight(
        "lateral_offset_start",
        ego_v=2.0,
        ego_y=0.6,
        min_travel=4.0,
        forbid_fallback=True,
    )


def s_two_obstacles_queue() -> SimResult:
    return _straight(
        "two_obstacles_queue",
        actors=[_static("q1", 22.0), _static("q2", 45.0)],
        blocking="q1",
    )


def s_obstacle_no_blocking_flag() -> SimResult:
    return _straight(
        "obstacle_no_blocking_flag",
        actors=[_static("silent_car", 30.0)],
        min_travel=2.0,
    )


def s_side_obstacle_adjacent_lane() -> SimResult:
    return _straight(
        "side_obstacle_adjacent_lane",
        actors=[_static("side_car", 28.0, y=2.8)],
        min_travel=6.0,
        max_abs_y=1.5,
        forbid_fallback=True,
    )


def s_medium_obstacle_18m() -> SimResult:
    return _straight(
        "medium_obstacle_18m",
        actors=[_static("med_car", 18.0)],
        blocking="med_car",
    )


def s_long_road_150m() -> SimResult:
    return _straight("long_road_150m", ego_v=2.0, road_length=150.0, min_travel=6.0, forbid_fallback=True)


def s_low_speed_creep() -> SimResult:
    return _straight("low_speed_creep", ego_v=0.3, min_travel=2.0, forbid_fallback=True)


def s_obstacle_60m_pass() -> SimResult:
    return _straight(
        "obstacle_60m_pass",
        actors=[_static("far_ahead", 60.0)],
        min_travel=6.0,
        forbid_fallback=True,
    )


def s_backup_off_open_road() -> SimResult:
    return _straight("backup_off_open_road", backup=False, min_travel=6.0, forbid_fallback=True)


def s_curved_open_road() -> SimResult:
    x, y, theta = curved_arc_pose(0.0, radius=60.0)
    result = run_curved_road(
        "curved_open_road",
        Actor("ego", x=x, y=y, theta=theta, v=2.0, kappa=1.0 / 60.0),
        [],
        horizon_s=3.0,
    )
    return _finish(result, min_travel=4.0, forbid_fallback=True)


def s_curved_obstacle_stop() -> SimResult:
    x, y, theta = curved_arc_pose(0.0, radius=60.0)
    ox, oy, otheta = curved_arc_pose(35.0, radius=60.0)
    result = run_curved_road(
        "curved_obstacle_stop",
        Actor("ego", x=x, y=y, theta=theta, v=2.0, kappa=1.0 / 60.0),
        [Actor("curve_car", x=ox, y=oy, theta=otheta, is_static=True)],
        blocking_actor_id="curve_car",
        horizon_s=3.0,
    )
    return result


# --- decider layer ---


def s_decider_bounds_assessment() -> BuilderResult:
    from common.path_assessment_decider import PathAssessmentDecider
    from common.path_bounds_decider import (
        BuildCandidatePathsFromBoundaries,
        PathBoundsDecider,
    )

    frame, rli, start = build_lattice_plan_frame()
    if not PathBoundsDecider().Process(None, rli, None).ok():
        return None, None, None, False, None, "decider produced no path"
    candidates = BuildCandidatePathsFromBoundaries(rli)
    rli.SetCandidatePathData(candidates)
    if not PathAssessmentDecider().Process(None, rli, None).ok() or rli.path_data is None:
        return None, None, None, False, None, "decider produced no path"
    extra = f"path_label={rli.path_data.path_label!r} n_candidates={len(candidates)}"
    return None, None, None, True, None, extra


def s_decider_lane_borrow_bounds() -> BuilderResult:
    from common.path_bounds_decider import PathBoundsDecider
    from common.planning_context import PlanningContext

    obs = build_static_obstacle("borrow_block", 35.0)
    _, rli = build_center_lane_reference_line()
    rli.Init([obs], 10.0)
    rli.SetBlockingObstacle("borrow_block")
    rli.set_is_path_lane_borrow(True)
    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ctx.planning_status.path_decider.decided_side_pass_direction = [1, 2]
    ok = PathBoundsDecider().Process(None, rli, ctx).ok()
    labels = [b.label for b in rli.GetCandidatePathBoundaries()]
    ok = ok and "regular/left/forward" in labels and "regular/right/forward" in labels
    extra = f"bounds_ok={ok} labels={labels}"
    return None, None, None, ok, None, extra


def s_decider_boundary_combine() -> BuilderResult:
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.path_assessment_decider import PathAssessmentDecider
    from common.path_bounds_decider import (
        BuildCandidatePathsFromBoundaries,
        PathBoundsDecider,
    )
    from common.planning_util import BuildCruiseSpeedData

    frame, rli, start = build_lattice_plan_frame()
    PathBoundsDecider().Process(None, rli, None)
    cands = BuildCandidatePathsFromBoundaries(rli)
    rli.SetCandidatePathData(cands)
    if not PathAssessmentDecider().Process(None, rli, None).ok():
        return None, rli, None, False, None, "decider_skip"
    rli.SetSpeedData(BuildCruiseSpeedData(rli))
    traj = DiscretizedTrajectory()
    combined = rli.CombinePathAndSpeedProfile(0.0, start.path_point.s, traj)
    extra = f"combine_ok={combined} traj_pts={len(traj)} label={rli.path_data.path_label!r}"
    return None, None, None, combined, None, extra


def s_decider_cruise_speed_data() -> BuilderResult:
    from common.planning_util import BuildCruiseSpeedData

    frame, rli, start = build_lattice_plan_frame()
    rli.SetLatticeCruiseSpeed(6.0)
    speed = BuildCruiseSpeedData(rli)
    ok = len(speed) >= 2 and all(pt.v == 6.0 for pt in speed)
    extra = f"speed_pts={len(speed)} v0={speed[0].v} s_end={speed[-1].s:.1f}"
    return None, None, None, ok, None, extra



# --- OnLane (closed loop) ---


def s_on_lane_open_road() -> SimResult:
    result = run_on_lane_road(
        "on_lane_open_road",
        Actor("ego", x=0.0, v=1.0),
        [],
        horizon_s=3.0,
    )
    return _finish(result, min_travel=2.0, forbid_fallback=True)


def s_on_lane_overtake_path_bounds() -> SimResult:
    # Same borrow stack OnLanePlanning runs before lattice, stepped in closed loop.
    # Full RunOnce each cycle also rebuilds the reference-line window and is too
    # heavy to repeat for the whole horizon; the open-road OnLane case covers RunOnce.
    result = run_path_bounds_road(
        "on_lane_overtake_path_bounds",
        Actor("ego", x=0.0, v=5.0),
        [_static("lead_1", 30.0)],
        horizon_s=5.0,
        path_label="regular/left/forward",
        blocking_actor_id="lead_1",
    )
    return _finish(result, min_abs_y=1.0)


# --- stress: short closed-loop probes, informational ---


def _sweep_summary(rows: List[str], collided: bool) -> SimResult:
    reason = "sweep [" + "; ".join(rows) + "]"
    return SimResult(
        name="stress",
        ok=not collided,
        reason=reason,
        horizon_s=1.0,
    )


def s_stress_obstacle_distance_sweep() -> SimResult:
    rows = []
    collided = False
    for x in (10, 15, 18, 22, 30, 40):
        result = run_straight_road(
            f"stress_x_{x}",
            Actor("ego", x=0.0, v=1.0),
            [_static(f"car_{x}", float(x))],
            horizon_s=1.0,
            blocking_actor_id=f"car_{x}",
            backup=True,
        )
        end_x = result.ego_x1 if result.cycles else None
        rows.append(f"x={x}:{'ok' if result.ok else 'fail'} end={end_x}")
        collided = collided or result.collided
    return _sweep_summary(rows, collided)


def s_stress_init_speed_sweep() -> SimResult:
    rows = []
    collided = False
    for v in (0.1, 1.0, 3.0, 6.0, 8.0):
        result = run_straight_road(
            f"stress_v_{v}",
            Actor("ego", x=0.0, v=v),
            [],
            horizon_s=1.0,
        )
        rows.append(f"v0={v}:{'ok' if result.ok else 'fail'} vend={result.ego_v1:.2f}")
        collided = collided or result.collided
    return _sweep_summary(rows, collided)


# --- Overtake: closed-loop PathBounds or lattice, no hand-drawn S-curve ---


def _borrow(
    name: str,
    leader_x: float,
    path_label: str,
    *,
    ego_v: float = 5.0,
    s_curve: bool = True,
    actors=None,
    horizon_s: float = 5.0,
) -> SimResult:
    if actors is None:
        actors = [_static("lead_1", leader_x)]
    result = run_path_bounds_road(
        name,
        Actor("ego", x=0.0, v=ego_v),
        list(actors),
        horizon_s=horizon_s,
        path_label=path_label,
        blocking_actor_id=actors[0].actor_id,
        s_curve=s_curve,
    )
    return result


def s_overtake_path_bounds_left() -> SimResult:
    return _finish(
        _borrow("overtake_path_bounds_left", 30.0, "regular/left/forward"),
        min_abs_y=1.0,
    )


def s_overtake_path_bounds_right() -> SimResult:
    return _finish(
        _borrow("overtake_path_bounds_right", 30.0, "regular/right/forward"),
        min_abs_y=1.0,
    )


def s_overtake_s_curve_pass() -> SimResult:
    """Same borrow stack as the left overtake; the trace is whatever the replanner executes."""
    return _finish(
        _borrow("overtake_s_curve_pass", 30.0, "regular/left/forward", ego_v=6.0),
        min_abs_y=1.0,
    )


def s_overtake_lattice_follow_no_pass() -> SimResult:
    return _finish(
        _straight(
            "overtake_lattice_follow_no_pass",
            ego_v=5.0,
            actors=[_static("slow_leader", 30.0)],
            blocking="slow_leader",
            horizon_s=4.0,
        ),
        max_abs_y=1.0,
    )


def s_overtake_borrow_offset_not_s_curve() -> SimResult:
    """Constant-offset borrow candidate, replanned every cycle (not an S-curve)."""
    return _borrow(
        "overtake_borrow_constant_offset",
        30.0,
        "regular/left/forward",
        s_curve=False,
    )


def s_overtake_two_leaders_s_curve() -> SimResult:
    return _finish(
        _borrow(
            "overtake_two_leaders",
            22.0,
            "regular/left/forward",
            ego_v=6.0,
            actors=[_static("lead_1", 22.0), _static("lead_2", 55.0)],
        ),
        min_abs_y=1.0,
    )


def s_overtake_far_leader_s_curve() -> SimResult:
    # The nudge starts about 10m before the leader, so the horizon has to
    # reach that station before a lateral offset shows up.
    return _finish(
        _borrow(
            "overtake_far_leader",
            45.0,
            "regular/left/forward",
            ego_v=6.0,
            horizon_s=8.0,
        ),
        min_abs_y=1.0,
    )


def s_overtake_stack_combine() -> SimResult:
    return _finish(
        _borrow("overtake_stack_combine", 30.0, "regular/left/forward", ego_v=5.0),
        min_abs_y=1.0,
    )


# --- lane change (closed loop on both reference lines) ---


def s_lane_change_overtake_slow_npc() -> SimResult:
    result = run_lane_change_road(
        "lane_change_overtake_slow_npc",
        Actor("ego", x=0.0, v=10.0),
        [Actor("npc_slow", x=20.0, v=3.0)],
        horizon_s=2.0,
    )
    return _finish(result, min_abs_y=1.0)


# --- explicit closed-loop regressions ---


def s_sim_open_road() -> SimResult:
    return _straight("sim_open_road_replan", ego_v=5.0, min_travel=12.0, forbid_fallback=True)


def s_sim_follow_leader() -> SimResult:
    return run_straight_road(
        "sim_follow_moving_leader",
        Actor("ego", x=0.0, v=8.0),
        [Actor("lead_1", x=28.0, v=4.0)],
        horizon_s=3.0,
    )


def s_sim_stopped_leader() -> SimResult:
    return run_straight_road(
        "sim_stopped_leader",
        Actor("ego", x=0.0, v=6.0),
        [_static("lead_1", 22.0)],
        horizon_s=3.0,
        blocking_actor_id="lead_1",
        backup=True,
    )


SCENARIOS: List[Scenario] = [
    Scenario("open_road", "Closed loop: open straight road", "lattice", s_open_road),
    Scenario("far_obstacle_stop", "Closed loop: 35m stationary car", "lattice", s_far_obstacle_stop),
    Scenario(
        "close_obstacle_no_backup",
        "Closed loop: 8m stationary car, backup off, planning should fail",
        "lattice",
        s_close_obstacle_no_backup,
        expect_ok=False,
    ),
    Scenario(
        "close_obstacle_with_backup",
        "Closed loop: 8m stationary car, backup on, no overlap",
        "lattice",
        s_close_obstacle_with_backup,
        backup=True,
    ),
    Scenario("higher_speed_cruise", "Closed loop: initial speed 5 m/s", "lattice", s_higher_speed_cruise),
    Scenario("stopped_start", "Closed loop: near-zero-speed start v=0.1", "lattice", s_stopped_start),
    Scenario("mid_lane_start", "Closed loop: continue from x=20m", "lattice", s_mid_lane_start),
    Scenario("lateral_offset_start", "Closed loop: lateral offset y=0.6m", "lattice", s_lateral_offset_start),
    Scenario("two_obstacles_queue", "Closed loop: 22m + 45m queue", "lattice", s_two_obstacles_queue),
    Scenario(
        "obstacle_no_blocking_flag",
        "Closed loop: 30m obstacle, not flagged blocking",
        "lattice",
        s_obstacle_no_blocking_flag,
    ),
    Scenario(
        "side_obstacle_adjacent_lane",
        "Closed loop: stationary car in the adjacent lane",
        "lattice",
        s_side_obstacle_adjacent_lane,
    ),
    Scenario("medium_obstacle_18m", "Closed loop: 18m blocking stopped car", "lattice", s_medium_obstacle_18m),
    Scenario("long_road_150m", "Closed loop: 150m reference line", "lattice", s_long_road_150m),
    Scenario("low_speed_creep", "Closed loop: creeping v=0.3", "lattice", s_low_speed_creep),
    Scenario("obstacle_60m_pass", "Closed loop: obstacle 60m ahead", "lattice", s_obstacle_60m_pass),
    Scenario(
        "backup_off_open_road",
        "Closed loop: backup off, open road still drives",
        "lattice",
        s_backup_off_open_road,
        backup=False,
    ),
    Scenario("curved_open_road", "Closed loop: curved lane", "lattice", s_curved_open_road),
    Scenario("curved_obstacle_stop", "Closed loop: curved lane, stopped leader", "lattice", s_curved_obstacle_stop),
    Scenario("decider_bounds_assessment", "PathBounds + PathAssessment", "decider", s_decider_bounds_assessment),
    Scenario(
        "decider_lane_borrow_bounds",
        "Lane-borrow PathBounds boundaries",
        "decider",
        s_decider_lane_borrow_bounds,
    ),
    Scenario("decider_boundary_combine", "Path + cruise speed Combine", "decider", s_decider_boundary_combine),
    Scenario("decider_cruise_speed_data", "BuildCruiseSpeedData", "decider", s_decider_cruise_speed_data),
    Scenario("on_lane_open_road", "Closed loop: OnLanePlanning every cycle", "on_lane", s_on_lane_open_road),
    Scenario(
        "on_lane_overtake_path_bounds",
        "Closed loop: OnLane PathBounds borrow",
        "on_lane",
        s_on_lane_overtake_path_bounds,
    ),
    Scenario(
        "stress_obstacle_distance_sweep",
        "Closed-loop obstacle distance sweep",
        "stress",
        s_stress_obstacle_distance_sweep,
        informational=True,
    ),
    Scenario(
        "stress_init_speed_sweep",
        "Closed-loop initial speed sweep",
        "stress",
        s_stress_init_speed_sweep,
        informational=True,
    ),
    Scenario(
        "overtake_path_bounds_left",
        "Closed loop: PathBounds left borrow",
        "overtake",
        s_overtake_path_bounds_left,
    ),
    Scenario(
        "overtake_path_bounds_right",
        "Closed loop: PathBounds right borrow",
        "overtake",
        s_overtake_path_bounds_right,
    ),
    Scenario(
        "overtake_s_curve_pass",
        "Closed loop: left borrow replanned every cycle",
        "overtake",
        s_overtake_s_curve_pass,
    ),
    Scenario(
        "overtake_lattice_follow_no_pass",
        "Closed loop: lattice follows, does not swerve around",
        "overtake",
        s_overtake_lattice_follow_no_pass,
    ),
    Scenario(
        "overtake_borrow_constant_offset",
        "Closed loop: constant-offset borrow candidate",
        "overtake",
        s_overtake_borrow_offset_not_s_curve,
        informational=True,
    ),
    Scenario(
        "overtake_two_leaders_s_curve",
        "Closed loop: left borrow around the first of two leaders",
        "overtake",
        s_overtake_two_leaders_s_curve,
    ),
    Scenario(
        "overtake_far_leader_s_curve",
        "Closed loop: left borrow around a leader at 45m",
        "overtake",
        s_overtake_far_leader_s_curve,
    ),
    Scenario(
        "overtake_stack_combine",
        "Closed loop: PathBounds + Combine, replanned every cycle",
        "overtake",
        s_overtake_stack_combine,
    ),
    Scenario(
        "lane_change_overtake_slow_npc",
        "Closed loop: lattice chooses between the current lane and the lane-change reference",
        "lane_change",
        s_lane_change_overtake_slow_npc,
    ),
    Scenario("sim_open_road_replan", "Closed loop: empty road, replan every cycle", "sim", s_sim_open_road),
    Scenario(
        "sim_follow_moving_leader",
        "Closed loop: faster ego follows a slower leader",
        "sim",
        s_sim_follow_leader,
    ),
    Scenario(
        "sim_stopped_leader",
        "Closed loop: stopped leader, executed path must not overlap",
        "sim",
        s_sim_stopped_leader,
    ),
]

SCENARIOS_BY_NAME = {s.name: s for s in SCENARIOS}
