#!/usr/bin/env python3
"""Run focused smoke checks for Apollo lattice components."""

from pathlib import Path
import math
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from behavior.collision_checker import CollisionChecker
from behavior.path_time_graph import PathTimeGraph
from behavior.prediction_querier import PredictionQuerier
from common.discretized_trajectory import DiscretizedTrajectory
from common.constraint_checker import ConstraintChecker
from common.constraint_checker1d import ConstraintChecker1d
from lattice_planner import LatticePlanner
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d
from config import FLAGS_speed_lon_decision_horizon, FLAGS_trajectory_time_length
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from trajectory_generation.backup_trajectory_generator import BackupTrajectoryGenerator
from trajectory_generation.end_condition_sampler import EndConditionSampler
from trajectory_generation.lattice_trajectory1d import CreateLatticeTrajectory1d
from trajectory_generation.lateral_osqp_optimizer import LateralOSQPOptimizer
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator

def _build_reference_line(length: float = 100.0):
    import config as config_module
    from lanelet_path import path_from_lanelet, straight_lanelet

    path = path_from_lanelet(
        straight_lanelet(length, 2.0, 2.0), config_module.FLAGS_default_cruise_speed
    )
    return path.points, path.lane_width

def _build_path_time_graph(obstacles):
    discretized_ref_points, lane_width = _build_reference_line()
    init_s = [0.0, 1.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    return PathTimeGraph(
        obstacles,
        discretized_ref_points,
        lane_width,
        init_s[0],
        init_s[0] + FLAGS_speed_lon_decision_horizon,
        0.0,
        FLAGS_trajectory_time_length,
        init_d,
    )

def check_lattice_trajectory_extrapolation():
    base = QuarticPolynomialCurve1d([0.0, 1.0, 0.0], [1.0, 0.0], 1.0)
    lattice = CreateLatticeTrajectory1d(base)
    end_s = lattice.Evaluate(0, 1.0)
    extrapolated_s = lattice.Evaluate(0, 2.0)
    assert extrapolated_s >= end_s

def check_backup_generator():
    discretized_ref_points, lane_width = _build_reference_line()
    init_s = [0.0, 1.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    path_time_graph = _build_path_time_graph([])
    prediction_querier = PredictionQuerier([], discretized_ref_points)
    trajectory1d_generator = Trajectory1dGenerator(
        init_s, init_d, path_time_graph, prediction_querier
    )
    collision_checker = CollisionChecker(
        [],
        init_s[0],
        init_d[0],
        discretized_ref_points,
        lane_width,
        path_time_graph,
    )
    backup_generator = BackupTrajectoryGenerator(
        init_s, init_d, 0.0, collision_checker, trajectory1d_generator
    )
    initial_pair_count = len(backup_generator.trajectory_pair_pqueue)
    trajectory = backup_generator.GenerateTrajectory(discretized_ref_points)
    assert len(trajectory) > 0
    assert len(backup_generator.trajectory_pair_pqueue) < initial_pair_count

def check_discretized_trajectory_uses_proto_scalar_defaults():
    trajectory = DiscretizedTrajectory([
        TrajectoryPoint(path_point=PathPoint(), relative_time=0.0),
        TrajectoryPoint(path_point=PathPoint(), relative_time=1.0),
    ])

    point = trajectory.Evaluate(0.5)
    assert point.v == 0.0
    assert point.a == 0.0
    assert point.steer == 0.0
    assert point.path_point.x == 0.0
    assert point.path_point.y == 0.0
    assert point.path_point.theta == 0.0
    assert point.path_point.kappa == 0.0
    assert point.path_point.s == 0.0


def check_path_time_graph_lane_width_fallback():
    import config as config_module

    graph = PathTimeGraph(
        [],
        [],
        lambda _s: (False, 0.0, 0.0),
        0.0,
        50.0,
        0.0,
        FLAGS_trajectory_time_length,
        [0.0, 0.0, 0.0],
    )
    bounds = graph.GetLateralBounds(0.0, 2.0, 1.0)
    expected_lower = (
        -config_module.FLAGS_default_reference_line_width / 2.0
        + config_module.FLAGS_half_vehicle_width
    )
    expected_upper = (
        config_module.FLAGS_default_reference_line_width / 2.0
        - config_module.FLAGS_half_vehicle_width
    )
    assert all(abs(lower - expected_lower) < 1e-9 for lower, _ in bounds)
    assert all(abs(upper - expected_upper) < 1e-9 for _, upper in bounds)

    discretized_ref_points, lane_width = _build_reference_line()
    from lattice_motion import Mover, Pose

    obstacle = Mover("fallback_width_obstacle", 4.0, 0.2, [Pose(0.0, 20.0, 1.4, 0.0, 0.0)])
    graph = PathTimeGraph(
        [obstacle],
        discretized_ref_points,
        lane_width,
        0.0,
        50.0,
        0.0,
        FLAGS_trajectory_time_length,
        [0.0, 0.0, 0.0],
    )
    assert graph.IsObstacleInGraph("fallback_width_obstacle")

def check_collision_checker_lane_width_fallback():
    import config as config_module

    checker = CollisionChecker.__new__(CollisionChecker)
    checker.lane_width = lambda _s: (False, 0.0, 0.0)
    assert checker.IsEgoVehicleInLane(10.0, 0.0)
    assert checker.IsEgoVehicleInLane(10.0, 0.49 * config_module.FLAGS_default_reference_line_width)
    assert not checker.IsEgoVehicleInLane(10.0, 0.51 * config_module.FLAGS_default_reference_line_width)

def check_constraint_checker_dynamic_speed_bound():
    import config as config_module

    trajectory = DiscretizedTrajectory([
        TrajectoryPoint(
            path_point=PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0),
            v=2.0,
            a=0.0,
            relative_time=0.0,
        ),
        TrajectoryPoint(
            path_point=PathPoint(x=1.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=1.0),
            v=2.0,
            a=0.0,
            relative_time=0.1,
        ),
    ])
    old_upper = config_module.FLAGS_speed_upper_bound
    try:
        config_module.FLAGS_speed_upper_bound = 1.0
        assert ConstraintChecker.ValidTrajectory(trajectory) == ConstraintChecker.Result.LON_VELOCITY_OUT_OF_BOUND
    finally:
        config_module.FLAGS_speed_upper_bound = old_upper

def check_constraint_checker_matches_cpp_non_increasing_time():
    def make_point(relative_time):
        return TrajectoryPoint(
            path_point=PathPoint(
                x=0.0,
                y=0.0,
                theta=0.0,
                kappa=0.0,
                s=0.0,
                dkappa=0.0,
                ddkappa=0.0,
            ),
            v=1.0,
            a=0.0,
            relative_time=relative_time,
        )

    duplicate_time = DiscretizedTrajectory([make_point(0.0), make_point(0.0)])
    assert (
        ConstraintChecker.ValidTrajectory(duplicate_time)
        == ConstraintChecker.Result.LON_JERK_OUT_OF_BOUND
    )

    decreasing_time = DiscretizedTrajectory([make_point(0.0), make_point(-0.1)])
    assert ConstraintChecker.ValidTrajectory(decreasing_time) == ConstraintChecker.Result.VALID

def check_constraint_checker1d_dynamic_speed_bound():
    import config as config_module

    class ConstantSpeedCurve:
        def ParamLength(self):
            return 0.2

        def Evaluate(self, order, param):
            if order == 1:
                return 2.0
            return 0.0

    old_upper = config_module.FLAGS_speed_upper_bound
    try:
        config_module.FLAGS_speed_upper_bound = 1.0
        assert not ConstraintChecker1d.IsValidLongitudinalTrajectory(ConstantSpeedCurve())
    finally:
        config_module.FLAGS_speed_upper_bound = old_upper

def check_dynamic_obstacle_sampling():
    discretized_ref_points, _lane_width = _build_reference_line()
    obstacle = _build_dynamic_obstacle()
    path_time_graph = _build_path_time_graph([obstacle])
    prediction_querier = PredictionQuerier([obstacle], discretized_ref_points)
    sampler = EndConditionSampler(
        [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], path_time_graph, prediction_querier
    )
    assert len(path_time_graph.GetPathTimeObstacles()) == 1
    assert len(sampler.SampleLonEndConditionsForPathTimePoints()) > 0

def check_cruise_sampler_matches_cpp_negative_range_cast():
    sampler = EndConditionSampler(
        [0.0, 10.0, 0.0],
        [0.0, 0.0, 0.0],
        object(),
        object(),
    )
    conditions = sampler.SampleLonEndConditionsForCruising(1.0)
    one_second_conditions = [condition for condition in conditions if condition[1] == 1.0]
    assert len(one_second_conditions) == 6
    assert len(conditions) == 33


def check_lateral_osqp_optimizer():
    optimizer = LateralOSQPOptimizer()
    assert optimizer.Optimize([0.0, 0.0, 0.0], 1.0, [(-1.0, 1.0)] * 12)
    trajectory = optimizer.GetOptimalTrajectory()
    frenet_path = optimizer.GetFrenetFramePath()
    assert len(frenet_path) == 12
    assert abs(trajectory.Evaluate(0, 0.0)) < 1e-6
    assert abs(frenet_path[-1].dl) < 1e-6
    assert abs(frenet_path[-1].ddl) < 1e-6

def check_lateral_osqp_keeps_usable_non_solved_result():
    import numpy as np
    import trajectory_generation.lateral_osqp_optimizer as optimizer_module

    class Info:
        status = "maximum iterations reached"

    class Result:
        info = Info()
        x = np.zeros(36)

    class FakeOSQP:
        def setup(self, **_kwargs):
            return None

        def solve(self):
            return Result()

    original_osqp = optimizer_module.OSQP
    optimizer_module.OSQP = FakeOSQP
    try:
        optimizer = optimizer_module.LateralOSQPOptimizer()
        assert optimizer.Optimize([0.0, 0.0, 0.0], 1.0, [(-1.0, 1.0)] * 12)
        assert len(optimizer.GetFrenetFramePath()) == 12
    finally:
        optimizer_module.OSQP = original_osqp

def check_lateral_bundle_ignores_optimizer_return_value():
    import config as config_module
    import trajectory_generation.trajectory1d_generator as generator_module

    expected_trajectory = object()

    class FakeOptimizer:
        def Optimize(self, _d_state, _delta_s, _d_bounds):
            return False

        def GetOptimalTrajectory(self):
            return expected_trajectory

    class FakePathTimeGraph:
        def GetLateralBounds(self, _s_min, _s_max, _delta_s):
            return [(-1.0, 1.0)]

    generator = Trajectory1dGenerator.__new__(Trajectory1dGenerator)
    generator.init_lon_state = [0.0, 0.0, 0.0]
    generator.init_lat_state = [0.0, 0.0, 0.0]
    generator.path_time_graph = FakePathTimeGraph()

    original_flag = config_module.FLAGS_lateral_optimization
    original_factory = generator_module.CreateLateralOptimizer
    config_module.FLAGS_lateral_optimization = True
    generator_module.CreateLateralOptimizer = FakeOptimizer
    try:
        trajectories = []
        generator.GenerateLateralTrajectoryBundle(trajectories)
        assert trajectories == [expected_trajectory]
    finally:
        generator_module.CreateLateralOptimizer = original_factory
        config_module.FLAGS_lateral_optimization = original_flag


def check_lattice_main_path_without_backup():
    import config as config_module

    old_backup_flag = config_module.FLAGS_enable_backup_trajectory
    config_module.FLAGS_enable_backup_trajectory = False
    try:
        points, lane_width = _build_reference_line()
        start_point = TrajectoryPoint(
            path_point=PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
            v=1.0,
            a=0.0,
            relative_time=0.0,
        )
        ok, trajectory, _traj_type = LatticePlanner().PlanOnLanelet(
            start_point, [], points, lane_width, config_module.FLAGS_default_cruise_speed
        )
        assert ok
        assert trajectory is not None
        assert len(trajectory) > 0
    finally:
        config_module.FLAGS_enable_backup_trajectory = old_backup_flag



def check_obstacle_empty_trajectory_is_static_for_st_graph():
    """A single pose is a static obstacle in the ST graph."""
    from lattice_motion import Mover, Pose

    obs = Mover("empty_traj", 4.0, 2.0, [Pose(0.0, 20.0, 0.0, 0.0, 0.0)])
    assert not obs.moving()

    graph = _build_path_time_graph([obs])
    assert len(graph.static_obs_sl_boundaries) == 1
    assert len(graph.path_time_obstacles) == 1
    assert graph.path_time_obstacles[0].id() == "empty_traj"











def _build_dynamic_obstacle():
    from lattice_motion import Mover, Pose

    return Mover(
        "obs_1",
        4.0,
        2.0,
        [
            Pose(0.0, 10.0, 0.0, 0.0, 1.0),
            Pose(1.0, 11.0, 0.0, 0.0, 1.0),
            Pose(2.0, 12.0, 0.0, 0.0, 1.0),
        ],
    )




def check_collision_checker_obstacle_behind_ego_no_crash():
    """
    Regression test: CollisionChecker.__init__ used to call
    BuildPredictedEnvironment (which logs a warning via self.logger) before
    assigning self.logger, so any obstacle behind the ADC in-lane crashed
    the constructor with AttributeError. Verify construction now succeeds
    and that obstacle is correctly filtered out of the predicted environment.
    """
    discretized_ref_points, lane_width = _build_reference_line()
    from lattice_motion import Mover, Pose

    behind_obstacle = Mover("behind_obs", 4.0, 2.0, [Pose(0.0, -5.0, 0.0, 0.0, 0.0)])

    checker = CollisionChecker([behind_obstacle], 10.0, 0.0, discretized_ref_points, lane_width, None)
    assert checker.logger is not None
    assert checker.predicted_bounding_rectangles
    assert all(len(env) == 0 for env in checker.predicted_bounding_rectangles), (
        "obstacle behind the ADC in-lane should be excluded from the predicted environment"
    )




















def check_apollo_map_borregas_ave_loads():
    """The vendored ApolloAppleSilicon loader decodes a bundled binary map."""
    from shapely.geometry import Point

    from apollo_map import load_map

    service = load_map("borregas_ave")
    assert service.lane_table, "borregas_ave base_map.bin produced no lanes"
    lane_id = next(iter(service.lane_table))
    line = service.get_lane_central_curve_by_id(lane_id)
    nearest = service.get_nearest_lane(Point(line.coords[0]))
    assert nearest in service.lane_table

def check_apollo_map_exports_commonroad_lanelets():
    """Apollo base_map.bin becomes a CommonRoad file with one lanelet per lane."""
    import tempfile

    from commonroad.common.file_reader import CommonRoadFileReader

    from apollo_map import load_map
    from apollo_map.to_commonroad import convert_map

    service = load_map("borregas_ave", load_only=True)
    with tempfile.TemporaryDirectory() as output_dir:
        xml_path = convert_map("borregas_ave", output_dir)
        scenario, _planning = CommonRoadFileReader(str(xml_path)).open()
    assert len(scenario.lanelet_network.lanelets) == len(service.lane_table)
    assert scenario.lanelet_network.traffic_lights
    assert scenario.lanelet_network.traffic_signs

def check_loaded_commonroad_scenario_is_driven():
    """Closed-loop planning follows a CommonRoad file that already exists.

    The file is the converter output for a bundled Apollo map. The drive does
    not add lanelets or obstacles, and it leaves the loaded geometry where it was.
    """
    import tempfile

    import numpy as np
    from shapely.geometry import LineString, Point

    import config
    from apollo_map.to_commonroad import convert_map
    from common.discretized_trajectory import DiscretizedTrajectory
    from commonroad.geometry.obstacle_shapes.rect_obstacle_shape import RectObstacleShape
    from commonroad.prediction.prediction import TrajectoryPrediction
    from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle
    from commonroad.scenario.state import CustomState, InitialState
    from commonroad.scenario.trajectory import Trajectory
    from protoclass.path_point import PathPoint
    from protoclass.trajectory_point import TrajectoryPoint
    from scripts.closed_loop_sim import Actor, _advance_actor
    from scripts.commonroad_drive import (
        _planner_obstacle,
        ego_obstacle_from_drive,
        load_scenario,
        occupancy_clearance,
        render_loaded_frames,
        run_loaded_scenario,
        sync_actor,
    )

    shape = RectObstacleShape(width=2.0, length=4.0)
    dynamic = DynamicObstacle(
        7,
        ObstacleType.CAR,
        shape,
        InitialState(
            time_step=0,
            position=np.array([0.0, 0.0]),
            orientation=0.0,
            velocity=2.0,
        ),
        TrajectoryPrediction(
            Trajectory(
                1,
                [
                    CustomState(
                        time_step=1,
                        position=np.array([0.2, 0.0]),
                        orientation=0.0,
                        velocity=2.0,
                    )
                ],
            ),
            shape,
        ),
    )
    actor = Actor("cr_7", x=0.0, replay=True, v=2.0)
    sync_actor(actor, dynamic, 0.1, np.zeros(2), 0.1)
    assert abs(actor.x - 0.2) < 1e-6
    held_x = actor.x
    _advance_actor(actor, 0.1)
    assert actor.x == held_x
    planned = _planner_obstacle(actor, dynamic, 0.1, np.zeros(2), 0.1)
    predicted = planned.pose_at(0.0)
    assert abs(predicted.x - 0.2) < 1e-6

    shift = config.EGO_VEHICLE_LENGTH / 2.0 - config.EGO_BACK_EDGE_TO_CENTER
    box = RectObstacleShape(width=2.0, length=4.0)
    executed = DiscretizedTrajectory(
        [
            TrajectoryPoint(
                path_point=PathPoint(x=0.0, y=0.0, theta=0.0),
                v=0.0,
                a=0.0,
                relative_time=0.0,
            ),
            TrajectoryPoint(
                path_point=PathPoint(x=0.0, y=0.0, theta=0.0),
                v=0.0,
                a=0.0,
                relative_time=0.1,
            ),
        ]
    )
    overlap = StaticObstacle(
        1,
        ObstacleType.CAR,
        box,
        InitialState(time_step=0, position=np.array([shift, 0.0]), orientation=0.0, velocity=0.0),
    )
    clear = StaticObstacle(
        2,
        ObstacleType.CAR,
        box,
        InitialState(time_step=0, position=np.array([shift, 20.0]), orientation=0.0, velocity=0.0),
    )
    clearance, collided = occupancy_clearance(
        executed, [overlap], t0=0.0, scenario_dt=0.1, origin=np.zeros(2)
    )
    assert collided and clearance == 0.0
    clearance, collided = occupancy_clearance(
        executed, [clear], t0=0.0, scenario_dt=0.1, origin=np.zeros(2)
    )
    assert not collided and clearance > 10.0

    with tempfile.TemporaryDirectory() as output_dir:
        scenario, problems = load_scenario(convert_map("borregas_ave", output_dir))
    lanelets = list(scenario.lanelet_network.lanelets)
    lanelet_ids = {lanelet.lanelet_id for lanelet in lanelets}
    centers = {
        lanelet.lanelet_id: np.array(lanelet.center_vertices, copy=True) for lanelet in lanelets
    }
    obstacle_count = len(scenario.obstacles)
    result = run_loaded_scenario(
        scenario, planning_problems=problems, horizon_s=2.0, init_v=5.0
    )
    assert result.ok, result.summary()
    assert result.lanelet_ids and set(result.lanelet_ids) <= lanelet_ids
    assert not result.actor_traces
    assert result.min_clearance is None and not result.collided
    assert {lanelet.lanelet_id for lanelet in scenario.lanelet_network.lanelets} == lanelet_ids
    assert len(scenario.obstacles) == obstacle_count
    for lanelet_id, center in centers.items():
        loaded = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        assert np.allclose(loaded.center_vertices, center)
    traveled = 0.0
    for previous, sample in zip(result.ego_trace, result.ego_trace[1:]):
        traveled += math.hypot(sample.x - previous.x, sample.y - previous.y)
    assert traveled > 6.0, result.summary()
    road = LineString(list(zip(result.ref_x, result.ref_y)))
    lateral = max(road.distance(Point(sample.x, sample.y)) for sample in result.ego_trace)
    assert lateral < 1.5, f"{result.summary()} lateral={lateral:.2f}"
    if len(result.lanelet_ids) == 1:
        chosen = scenario.lanelet_network.find_lanelet_by_id(result.lanelet_ids[0])
        origin = chosen.center_vertices[0]
        end = chosen.center_vertices[-1]
        assert abs(result.ref_x[-1] - (end[0] - origin[0])) < 1e-3
        assert abs(result.ref_y[-1] - (end[1] - origin[1])) < 1e-3
        assert abs(result.origin_x - float(origin[0])) < 1e-6
        assert abs(result.origin_y - float(origin[1])) < 1e-6
    ego = ego_obstacle_from_drive(result)
    assert len(scenario.obstacles) == obstacle_count
    last_step = int(round(result.ego_trace[-1].t / float(scenario.dt)))
    start_center = ego.occupancy_at_time(0).center
    end_center = ego.occupancy_at_time(last_step).center
    assert math.hypot(end_center.x - start_center.x, end_center.y - start_center.y) > 6.0
    driven = [
        scenario.lanelet_network.find_lanelet_by_id(lanelet_id) for lanelet_id in result.lanelet_ids
    ]
    centerline = LineString(np.vstack([lanelet.center_vertices for lanelet in driven]))
    assert centerline.distance(Point(start_center.x, start_center.y)) < 2.0
    assert centerline.distance(Point(end_center.x, end_center.y)) < 2.0
    frames = render_loaded_frames(result, steps=[0, last_step])
    assert len(frames) == 2 and frames[0].shape == frames[1].shape and frames[0].size > 0
    assert not np.array_equal(frames[0], frames[1])
    assert len(scenario.obstacles) == obstacle_count
    assert {lanelet.lanelet_id for lanelet in scenario.lanelet_network.lanelets} == lanelet_ids

def main():
    check_lattice_trajectory_extrapolation()
    check_backup_generator()
    check_discretized_trajectory_uses_proto_scalar_defaults()
    check_collision_checker_lane_width_fallback()
    check_path_time_graph_lane_width_fallback()
    check_constraint_checker_dynamic_speed_bound()
    check_constraint_checker_matches_cpp_non_increasing_time()
    check_constraint_checker1d_dynamic_speed_bound()
    check_dynamic_obstacle_sampling()
    check_cruise_sampler_matches_cpp_negative_range_cast()
    check_lateral_osqp_optimizer()
    check_lateral_osqp_keeps_usable_non_solved_result()
    check_lateral_bundle_ignores_optimizer_return_value()
    check_obstacle_empty_trajectory_is_static_for_st_graph()
    check_lattice_main_path_without_backup()
    check_collision_checker_obstacle_behind_ego_no_crash()
    check_apollo_map_borregas_ave_loads()
    check_apollo_map_exports_commonroad_lanelets()
    check_loaded_commonroad_scenario_is_driven()
    print("lattice component checks succeeded")

if __name__ == "__main__":
    main()
