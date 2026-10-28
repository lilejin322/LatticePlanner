#!/usr/bin/env python3
"""One-cycle contracts for sampling, cost order, fallback, and stitching.

Each check calls the lattice pieces the way ``LatticePlanner.PlanOnLanelet``
does. A pass is a property of the trajectories those pieces return.
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import config
from behavior.path_time_graph import PathTimeGraph
from behavior.prediction_querier import PredictionQuerier
from common.constraint_checker import ConstraintChecker
from common.constraint_checker1d import ConstraintChecker1d
from common.curve1d.piecewise_jerk_trajectory1d import PiecewiseJerkTrajectory1d
from common.curve1d.quintic_polynomial_curve1d import QuinticPolynomialCurve1d
from lanelet_path import path_from_lanelet, straight_lanelet
from lattice_motion import obstacle_from_samples
from lattice_planner import LatticePlanner
from protoclass.adc_trajectory import TrajectoryType
from protoclass.path_point import PathPoint
from protoclass.planning_target import PlanningTarget, StopPoint
from protoclass.trajectory_point import TrajectoryPoint
from trajectory_generation.end_condition_sampler import EndConditionSampler
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator
from trajectory_generation.trajectory_evaluator import TrajectoryEvaluator


def _road():
    path = path_from_lanelet(straight_lanelet(200.0, 2.0, 2.0), config.FLAGS_default_cruise_speed)
    return path


def _start(v: float = 5.0, x: float = 0.0, y: float = 0.0, a: float = 0.0) -> TrajectoryPoint:
    return TrajectoryPoint(
        path_point=PathPoint(x=x, y=y, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0),
        v=v,
        a=a,
        relative_time=0.0,
    )


def _graph(path, obstacles, init_s, init_d):
    return PathTimeGraph(
        obstacles,
        path.points,
        path.lane_width,
        init_s[0],
        init_s[0] + config.FLAGS_speed_lon_decision_horizon,
        0.0,
        config.FLAGS_trajectory_time_length,
        init_d,
    )


def _plan(path, start, obstacles, backup: bool):
    previous = config.FLAGS_enable_backup_trajectory
    config.FLAGS_enable_backup_trajectory = backup
    try:
        return LatticePlanner().PlanOnLanelet(
            start,
            obstacles,
            path.points,
            path.lane_width,
            path.speed_at(0.0),
        )
    finally:
        config.FLAGS_enable_backup_trajectory = previous


def check_open_road_returns_a_valid_normal_trajectory():
    path = _road()
    start = _start(5.0)
    ok, trajectory, traj_type = _plan(path, start, [], backup=True)
    assert ok and trajectory is not None and len(trajectory) > 1
    assert traj_type == TrajectoryType.NORMAL
    assert ConstraintChecker.ValidTrajectory(trajectory) == ConstraintChecker.Result.VALID
    first = trajectory[0]
    assert abs(first.path_point.x - start.path_point.x) < 1e-9
    assert abs(first.path_point.y - start.path_point.y) < 1e-9
    assert abs(first.v - start.v) < 1e-9
    assert abs(first.a - start.a) < 1e-9


def check_stopped_car_at_8m_is_fallback_or_failure():
    """A car 8 m ahead leaves no lattice pair. Backup is returned anyway."""
    path = _road()
    blocked = [obstacle_from_samples(1, 4.0, 2.0, [(0.0, 8.0, 0.0, 0.0, 0.0)])]
    ok, trajectory, traj_type = _plan(path, _start(5.0), blocked, backup=False)
    assert not ok and trajectory is None and traj_type == TrajectoryType.UNKNOWN

    ok, trajectory, traj_type = _plan(path, _start(5.0), blocked, backup=True)
    assert ok and trajectory is not None
    assert traj_type == TrajectoryType.PATH_FALLBACK
    assert ConstraintChecker.ValidTrajectory(trajectory) != ConstraintChecker.Result.VALID


def check_car_at_35m_still_has_a_normal_trajectory():
    path = _road()
    ahead = [obstacle_from_samples(2, 4.0, 2.0, [(0.0, 35.0, 0.0, 0.0, 0.0)])]
    ok, trajectory, traj_type = _plan(path, _start(1.0), ahead, backup=True)
    assert ok and trajectory is not None
    assert traj_type == TrajectoryType.NORMAL
    assert ConstraintChecker.ValidTrajectory(trajectory) == ConstraintChecker.Result.VALID


def check_cruise_bundle_is_quartics_and_stop_bundle_adds_nine_quintics():
    path = _road()
    init_s = [0.0, 5.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    graph = _graph(path, [], init_s, init_d)
    querier = PredictionQuerier([], path.points)
    sampler = EndConditionSampler(init_s, init_d, graph, querier)
    cruise_speed = config.FLAGS_default_cruise_speed
    cruise_conditions = sampler.SampleLonEndConditionsForCruising(cruise_speed)
    generator = Trajectory1dGenerator(init_s, init_d, graph, querier)
    lon, lat = generator.GenerateTrajectoryBundles(PlanningTarget(cruise_speed=cruise_speed))
    assert len(lon) == len(cruise_conditions)
    assert lat and isinstance(lat[0], PiecewiseJerkTrajectory1d)
    assert all(curve.has_target_velocity() and not curve.has_target_position() for curve in lon)

    stop_s = 80.0
    stopping = sampler.SampleLonEndConditionsForStopping(stop_s)
    assert len(stopping) == 9
    assert all(state == [stop_s, 0.0, 0.0] for state, _time in stopping)
    lon, lat = generator.GenerateTrajectoryBundles(
        PlanningTarget(cruise_speed=cruise_speed, stop_point=StopPoint(s=stop_s))
    )
    stop_curves = [curve for curve in lon if curve.has_target_position()]
    assert len(stop_curves) == 9
    for curve in stop_curves:
        duration = curve.ParamLength()
        assert abs(curve.Evaluate(0, duration) - stop_s) < 1e-6
        assert abs(curve.Evaluate(1, duration)) < 1e-6
        assert abs(curve.Evaluate(2, duration)) < 1e-6
    assert isinstance(lat[0], PiecewiseJerkTrajectory1d)


def check_polynomial_lateral_samples_stay_out_of_the_default_bundle():
    """Default search uses one OSQP lateral curve, not the 12 offset samples."""
    path = _road()
    init_s = [0.0, 5.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    graph = _graph(path, [], init_s, init_d)
    querier = PredictionQuerier([], path.points)
    previous = config.FLAGS_lateral_optimization
    config.FLAGS_lateral_optimization = False
    try:
        _lon, lat = Trajectory1dGenerator(init_s, init_d, graph, querier).GenerateTrajectoryBundles(
            PlanningTarget(cruise_speed=config.FLAGS_default_cruise_speed)
        )
    finally:
        config.FLAGS_lateral_optimization = previous
    assert len(lat) == 12
    assert {round(curve.target_position(), 3) for curve in lat} == {-0.5, 0.0, 0.5}
    assert {round(curve.ParamLength(), 3) for curve in lat} == {10.0, 20.0, 40.0, 80.0}
    assert all(isinstance(curve._trajectory1d, QuinticPolynomialCurve1d) for curve in lat)


def check_stop_filter_keeps_cost_order():
    path = _road()
    init_s = [0.0, 5.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    graph = _graph(path, [], init_s, init_d)
    querier = PredictionQuerier([], path.points)
    stop_s = 80.0
    target = PlanningTarget(cruise_speed=config.FLAGS_default_cruise_speed, stop_point=StopPoint(s=stop_s))
    lon, lat = Trajectory1dGenerator(init_s, init_d, graph, querier).GenerateTrajectoryBundles(target)
    evaluator = TrajectoryEvaluator(init_s, target, lon, lat, graph, path.points)
    costs = []
    end_time = config.FLAGS_trajectory_time_length
    while evaluator.has_more_trajectory_pairs():
        costs.append(evaluator.top_trajectory_pair_cost())
        lon_curve, _lat_curve = evaluator.next_top_trajectory_pair()
        end_s = lon_curve.Evaluate(0, end_time)
        assert end_s + config.FLAGS_lattice_stop_buffer <= stop_s
        assert ConstraintChecker1d.IsValidLongitudinalTrajectory(lon_curve)
    # One lateral curve, so this is the number of longitudinal curves that
    # pass both the stop station and the 0.1 s checker.
    assert len(costs) == 33
    assert all(costs[i] <= costs[i + 1] for i in range(len(costs) - 1))


def check_follow_samples_enter_the_bundle_as_quintics():
    path = _road()
    init_s = [0.0, 5.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    obstacle = obstacle_from_samples(
        9,
        4.0,
        2.0,
        [(0.0, 30.0, 0.0, 0.0, 4.0), (8.0, 62.0, 0.0, 0.0, 4.0)],
    )
    graph = _graph(path, [obstacle], init_s, init_d)
    querier = PredictionQuerier([obstacle], path.points)
    sampler = EndConditionSampler(init_s, init_d, graph, querier)
    followed = sampler.SampleLonEndConditionsForPathTimePoints()
    assert followed
    assert all(state[2] == 0.0 and time >= config.FLAGS_polynomial_minimal_param for state, time in followed)
    lon, _lat = Trajectory1dGenerator(init_s, init_d, graph, querier).GenerateTrajectoryBundles(
        PlanningTarget(cruise_speed=config.FLAGS_default_cruise_speed)
    )
    quintics = [curve for curve in lon if curve.has_target_position()]
    assert len(quintics) == len(followed)
    for curve in quintics:
        assert abs(curve.Evaluate(2, curve.ParamLength())) < 1e-9


def check_next_cycle_starts_at_the_committed_point():
    path = _road()
    ok, trajectory, traj_type = _plan(path, _start(5.0), [], backup=True)
    assert ok and traj_type == TrajectoryType.NORMAL
    dt = 1.0 / config.FLAGS_planning_loop_rate
    committed = trajectory.Evaluate(dt)
    second = _start(
        v=committed.v,
        x=committed.path_point.x,
        y=committed.path_point.y,
        a=committed.a,
    )
    second.path_point.theta = committed.path_point.theta
    second.path_point.kappa = committed.path_point.kappa
    ok, again, again_type = _plan(path, second, [], backup=True)
    assert ok and again_type == TrajectoryType.NORMAL
    first = again[0]
    assert abs(first.path_point.x - committed.path_point.x) < 1e-9
    assert abs(first.path_point.y - committed.path_point.y) < 1e-9
    assert abs(first.v - committed.v) < 1e-9
    assert abs(first.a - committed.a) < 1e-9
    assert abs((first.path_point.theta or 0.0) - (committed.path_point.theta or 0.0)) < 1e-9


def main() -> int:
    logging.disable(logging.CRITICAL)
    check_open_road_returns_a_valid_normal_trajectory()
    check_stopped_car_at_8m_is_fallback_or_failure()
    check_car_at_35m_still_has_a_normal_trajectory()
    check_cruise_bundle_is_quartics_and_stop_bundle_adds_nine_quintics()
    check_polynomial_lateral_samples_stay_out_of_the_default_bundle()
    check_stop_filter_keeps_cost_order()
    check_follow_samples_enter_the_bundle_as_quintics()
    check_next_cycle_starts_at_the_committed_point()
    print("lattice cycle checks succeeded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
