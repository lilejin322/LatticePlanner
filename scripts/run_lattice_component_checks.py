#!/usr/bin/env python3
"""Run focused smoke checks for Apollo lattice components."""

from pathlib import Path
import importlib
import math
import sys
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from lattice_planner import ToDiscretizedReferenceLine
from behavior.collision_checker import CollisionChecker
from behavior.path_time_graph import PathTimeGraph
from behavior.prediction_querier import PredictionQuerier
from common.discretized_trajectory import DiscretizedTrajectory
from common.frame import Frame
from common.map_path_point import MapPathPoint
from common.obstacle import Obstacle
from common.path import Path as MapPath, PathOverlap
from reference_line import ReferenceLine
from reference_line.reference_line_info import ReferenceLineInfo
from reference_line.reference_point import ReferencePoint
from common.route_segments import RouteSegments
from common.st_boundary import STBoundary
from common.st_graph_data import StGraphData
from common.st_point import STPoint
from common.vec2d import Vec2d
from common.box2d import Box2d
from common.aabox2d import AABox2d
from common.line_segment2d import LineSegment2d
from common.polygon2d import Polygon2d
from common.lane_segment_kd_tree import AABoxKDTree2d, AABoxKDTreeParams
from common.constraint_checker import ConstraintChecker
from common.constraint_checker1d import ConstraintChecker1d
from protoclass.adc_trajectory import ADCTrajectory
from lattice_planner import LatticePlanner
from common.curve1d.quartic_polynomial_curve1d import QuarticPolynomialCurve1d
from config import FLAGS_destination_obstacle_id, FLAGS_speed_lon_decision_horizon, FLAGS_trajectory_time_length
from protoclass.adc_trajectory import Point3D
from protoclass.header import Header
from protoclass.path_point import PathPoint
from protoclass.perception_obstacle import PerceptionObstacle
from protoclass.prediction_obstacles import PredictionObstacle, PredictionObstacles, Trajectory as PredictionTrajectory
from protoclass.sl_boundary import SLBoundary
from protoclass.trajectory import Trajectory
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState
from trajectory_generation.backup_trajectory_generator import BackupTrajectoryGenerator
from trajectory_generation.end_condition_sampler import EndConditionSampler
from trajectory_generation.lattice_trajectory1d import CreateLatticeTrajectory1d
from trajectory_generation.lateral_osqp_optimizer import LateralOSQPOptimizer
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator


def _build_reference_line(length: float = 100.0):
    from scripts.lattice_commonroad import _reference_line

    center = [(float(index), 0.0) for index in range(int(length) + 1)]
    reference_line = _reference_line(center, 2.0, 2.0)
    discretized_ref_points = ToDiscretizedReferenceLine(reference_line.reference_points)

    start_point = TrajectoryPoint(
        path_point=PathPoint(
            x=0.0,
            y=0.0,
            z=0.0,
            theta=0.0,
            kappa=0.0,
            s=0.0,
            dkappa=0.0,
            ddkappa=0.0,
        ),
        v=1.0,
        a=0.0,
        relative_time=0.0,
    )

    vehicle_state = VehicleState()
    vehicle_state.x = 0.0
    vehicle_state.y = 0.0
    vehicle_state.heading = 0.0
    route_segments = RouteSegments()
    reference_line_info = ReferenceLineInfo(
        vehicle_state, start_point, reference_line, route_segments
    )
    return reference_line, discretized_ref_points, reference_line_info


def _build_path_time_graph(obstacles):
    _, discretized_ref_points, reference_line_info = _build_reference_line()
    init_s = [0.0, 1.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    return PathTimeGraph(
        obstacles,
        discretized_ref_points,
        reference_line_info,
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
    _, discretized_ref_points, reference_line_info = _build_reference_line()
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
        reference_line_info,
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


def check_adjust_trajectory_resamples_cut_trajectory():
    def make_point(x, relative_time):
        return TrajectoryPoint(
            path_point=PathPoint(
                x=x,
                y=0.0,
                theta=0.0,
                kappa=0.0,
                s=x,
                dkappa=0.0,
                ddkappa=0.0,
            ),
            v=1.0,
            a=0.0,
            relative_time=relative_time,
        )

    planning_start_point = make_point(0.0, 0.1)
    ok, adjusted = ReferenceLineInfo.AdjustTrajectoryWhichStartsFromCurrentPos(
        planning_start_point,
        [make_point(1.0, 0.2), make_point(2.0, 0.4)],
    )

    assert ok
    assert adjusted is not None
    assert len(adjusted) > 2
    assert adjusted[0].relative_time == planning_start_point.relative_time
    assert all(
        adjusted[i - 1].relative_time < adjusted[i].relative_time
        for i in range(1, len(adjusted))
    )


def check_path_time_graph_lane_width_fallback():
    import config as config_module

    class ReferenceLineStub:
        def GetLaneWidth(self, s):
            return False, 0.0, 0.0

    reference_line_info = SimpleNamespace(reference_line=ReferenceLineStub())
    graph = PathTimeGraph(
        [],
        [],
        reference_line_info,
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

    _, discretized_ref_points, _ = _build_reference_line()
    perception = PerceptionObstacle(
        id=16,
        position=Point3D(x=20.0, y=1.4, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=0.2,
        height=1.5,
        theta=0.0,
    )
    obstacle = Obstacle("fallback_width_obstacle", perception, is_static=True)
    graph = PathTimeGraph(
        [obstacle],
        discretized_ref_points,
        reference_line_info,
        0.0,
        50.0,
        0.0,
        FLAGS_trajectory_time_length,
        [0.0, 0.0, 0.0],
    )
    assert graph.IsObstacleInGraph("fallback_width_obstacle")


def check_collision_checker_lane_width_fallback():
    class ReferenceLineStub:
        def GetLaneWidth(self, s):
            return False, 0.0, 0.0

    class ReferenceLineInfoStub:
        reference_line = ReferenceLineStub()

    import config as config_module

    checker = CollisionChecker.__new__(CollisionChecker)
    checker.reference_line_info = ReferenceLineInfoStub()
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
    _, discretized_ref_points, _ = _build_reference_line()
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


def check_reference_line_sl_boundary_lane_width_failure_matches_cpp():
    class MapPathStub:
        length = 10.0

        def GetLaneWidth(self, s):
            return False, 0.0, 0.0

    reference_line = ReferenceLine.__new__(ReferenceLine)
    reference_line._map_path = MapPathStub()

    crossing_center = SLBoundary(
        start_s=1.0,
        end_s=2.0,
        start_l=-0.1,
        end_l=0.1,
    )
    entirely_left = SLBoundary(
        start_s=1.0,
        end_s=2.0,
        start_l=0.1,
        end_l=0.2,
    )
    assert reference_line.IsOnLane(crossing_center)
    assert not reference_line.IsOnLane(entirely_left)


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


def check_prediction_time_alignment():
    prediction = PredictionObstacles(
        header=Header(timestamp_sec=10.0),
        prediction_obstacle=[
            PredictionObstacle(
                perception_obstacle=PerceptionObstacle(
                    id=2,
                    position=Point3D(x=0.0, y=0.0, z=0.0),
                    theta=0.0,
                    velocity=Point3D(x=0.0, y=0.0, z=0.0),
                    length=4.0,
                    width=2.0,
                    height=1.5,
                ),
                trajectory=[
                    PredictionTrajectory(
                        trajectory_point=[
                            TrajectoryPoint(path_point=PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0), relative_time=0.0),
                            TrajectoryPoint(path_point=PathPoint(x=1.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0), relative_time=1.0),
                            TrajectoryPoint(path_point=PathPoint(x=2.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0), relative_time=2.0),
                        ]
                    )
                ],
            )
        ],
    )
    Frame.AlignPredictionTime(11.0, prediction)
    aligned_points = prediction.prediction_obstacle[0].trajectory[0].trajectory_point
    assert [point.relative_time for point in aligned_points] == [0.0, 1.0]












def check_lattice_main_path_without_backup():
    import config as config_module

    old_backup_flag = config_module.FLAGS_enable_backup_trajectory
    config_module.FLAGS_enable_backup_trajectory = False
    try:
        reference_line, _, reference_line_info = _build_reference_line()
        assert reference_line_info.Init([], 10.0)
        frame = Frame(0)
        frame._reference_line_info = [reference_line_info]
        frame._obstacles = {}
        start_point = reference_line_info._adc_planning_point
        reference_line_info.Init = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("LatticePlanner must not initialize ReferenceLineInfo twice")
        )
        ok = LatticePlanner().Plan(start_point, frame, ADCTrajectory())
        assert ok
        assert reference_line_info.trajectory is not None
        assert len(reference_line_info.trajectory) > 0
    finally:
        config_module.FLAGS_enable_backup_trajectory = old_backup_flag
























def check_obstacle_decision_property_api():
    from common.obstacle import Obstacle
    from protoclass.decision_result import ObjectDecisionType, ObjectIgnore
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    perception = PerceptionObstacle(
        id=9,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=1.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("obs_prop", perception, is_static=True)
    ignore = ObjectDecisionType()
    ignore.object_tag = ObjectIgnore()
    obs.AddLongitudinalDecision("test", ignore)
    assert obs.HasNonIgnoreDecision() is False


def check_obstacle_copies_trajectory():
    """
    Regression test: Apollo's C++ Obstacle stores trajectory_/perception_obstacle_
    as value members, copy-constructed from the constructor arguments, then
    recomputes cumulative path_point.s on its OWN copy. The Python port must
    copy too - it must never rewrite path_point.s on the caller's original
    trajectory object.
    """
    from common.obstacle import Obstacle
    from protoclass.trajectory import Trajectory
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    trajectory = Trajectory(trajectory_point=[
        TrajectoryPoint(path_point=PathPoint(x=float(i), y=0.0, theta=0.0, s=100.0 + i), relative_time=float(i) * 0.1)
        for i in range(3)
    ])
    original_s_values = [tp.path_point.s for tp in trajectory.trajectory_point]

    perception = PerceptionObstacle(
        id=13,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=0.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("obs_traj_copy", perception, trajectory=trajectory)

    assert [tp.path_point.s for tp in trajectory.trajectory_point] == original_s_values, (
        "Obstacle(...) must not mutate the caller's original trajectory's path_point.s"
    )
    assert obs.Trajectory() is not trajectory
    assert [tp.path_point.s for tp in obs.Trajectory().trajectory_point] == [0.0, 1.0, 2.0]


def check_obstacle_empty_trajectory_is_static_for_st_graph():
    """
    Apollo Obstacle::HasTrajectory checks trajectory_point().empty(), not
    whether the trajectory protobuf object itself exists. An obstacle with an
    empty trajectory must still enter PathTimeGraph as a static obstacle.
    """
    perception = PerceptionObstacle(
        id=14,
        position=Point3D(x=20.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("empty_traj", perception, trajectory=Trajectory())
    assert not obs.HasTrajectory()

    graph = _build_path_time_graph([obs])
    assert len(graph.static_obs_sl_boundaries) == 1
    assert len(graph.path_time_obstacles) == 1
    assert graph.path_time_obstacles[0].id() == "empty_traj"


def check_path_time_graph_keeps_ignored_static_obstacle():
    """
    C++ PathTimeGraph skips virtual obstacles only. Ignore decisions attached
    by earlier tasks should not erase a real obstacle from ST graph setup.
    """
    from protoclass.decision_result import ObjectDecisionType, ObjectIgnore

    perception = PerceptionObstacle(
        id=15,
        position=Point3D(x=22.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("ignored_static", perception, is_static=True)
    ignore = ObjectDecisionType()
    ignore.object_tag = ObjectIgnore()
    obs.AddLongitudinalDecision("pre_path_decider", ignore)
    obs.AddLateralDecision("pre_path_decider", ignore)

    graph = _build_path_time_graph([obs])
    assert len(graph.static_obs_sl_boundaries) == 1




def check_polygon_box_distance():
    from common.box2d import Box2d
    from common.polygon2d import Polygon2d
    from common.vec2d import Vec2d

    box = Box2d(Vec2d(10.0, 0.0), 0.0, 4.0, 2.0)
    polygon = Polygon2d(box)
    ego = Box2d(Vec2d(0.0, 0.0), 0.0, 4.8, 2.0)
    dist = polygon.DistanceTo(ego)
    assert dist > 5.0
    assert dist < 8.0
    overlap_ego = Box2d(Vec2d(10.0, 0.0), 0.0, 4.8, 2.0)
    assert polygon.DistanceTo(overlap_ego) == 0.0


def check_polygon_overlap_contains_line_segment():
    from common.polygon2d import Polygon2d

    polygon = Polygon2d([
        Vec2d(0.0, 0.0),
        Vec2d(4.0, 0.0),
        Vec2d(4.0, 4.0),
        Vec2d(0.0, 4.0),
    ])
    inside_segment = LineSegment2d(Vec2d(1.0, 1.0), Vec2d(3.0, 3.0))
    assert polygon.HasOverlap(inside_segment)


def check_polygon_bounding_box_with_heading_uses_cross_projection():
    import math
    from common.polygon2d import Polygon2d

    polygon = Polygon2d([
        Vec2d(0.0, 0.0),
        Vec2d(4.0, 0.0),
        Vec2d(4.0, 2.0),
        Vec2d(0.0, 2.0),
    ])
    box = polygon.BoundingBoxWithHeading(math.pi / 2.0)
    assert abs(box.length - 2.0) < 1e-6
    assert abs(box.width - 4.0) < 1e-6


def check_polygon_extreme_points_accepts_triangle():
    from common.polygon2d import Polygon2d

    triangle = Polygon2d([
        Vec2d(0.0, 0.0),
        Vec2d(2.0, 0.0),
        Vec2d(1.0, 1.0),
    ])
    first, last = triangle.ExtremePoints(0.0)
    assert first.x == 0.0
    assert last.x == 2.0


def check_route_segments_uses_cpp_segmentation_epsilon():
    from common.route_segments import kSegmentationEpsilon

    assert kSegmentationEpsilon == 0.2


def check_box_distance_to_segment_canonical_state():
    from common.box2d import Box2d
    from common.line_segment2d import LineSegment2d
    from common.vec2d import Vec2d

    box = Box2d(Vec2d(0.0, 0.0), 0.0, 2.0, 2.0)
    segment = LineSegment2d(Vec2d(3.0, 1.5), Vec2d(-1.0, 1.5))
    assert abs(box.DistanceTo(segment) - 0.5) < 1e-6










def check_reference_point_remove_duplicates_uses_euclidean_distance():
    points = [
        ReferencePoint(MapPathPoint(Vec2d(0.0, 0.0), 0.0), 0.0, 0.0),
        ReferencePoint(MapPathPoint(Vec2d(0.8e-7, 0.8e-7), 0.0), 0.0, 0.0),
    ]
    ReferencePoint.RemoveDuplicates(points)
    assert len(points) == 2


def check_reference_line_get_sl_boundary_failure_is_false():
    from protoclass.sl_boundary import SLBoundary

    reference_line, _, _ = _build_reference_line()
    original_xy_to_sl = reference_line.XYToSL
    reference_line.XYToSL = lambda point: (False, None)
    try:
        sl_boundary = SLBoundary()
        result = reference_line.GetSLBoundary(
            [Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), Vec2d(1.0, 1.0)],
            0.0,
            sl_boundary,
        )
        assert result is False
    finally:
        reference_line.XYToSL = original_xy_to_sl


def check_reference_line_info_copies_reference_line():
    """
    Regression test: Apollo's C++ ReferenceLineInfo stores reference_line_/lanes_
    as value members, copy-constructed from the constructor arguments - so
    mutating a ReferenceLineInfo's reference line can never affect the caller's
    original ReferenceLine (e.g. one still cached in ReferenceLineProvider's
    history). The Python port must copy on construction too, not alias.
    """
    reference_line, _, reference_line_info = _build_reference_line()
    original_priority = reference_line.GetPriority()

    assert reference_line_info.reference_line is not reference_line

    reference_line_info.SetPriority(original_priority + 1)
    assert reference_line.GetPriority() == original_priority, (
        "ReferenceLineInfo.SetPriority must not mutate the caller's original ReferenceLine"
    )
    assert reference_line_info.GetPriority() == original_priority + 1


def _build_dynamic_obstacle():
    perception = PerceptionObstacle(
        id=1,
        position=Point3D(x=10.0, y=0.0, z=0.0),
        theta=0.0,
        velocity=Point3D(x=1.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
    )
    trajectory = Trajectory(
        trajectory_point=[
            TrajectoryPoint(
                path_point=PathPoint(
                    x=10.0,
                    y=0.0,
                    z=0.0,
                    theta=0.0,
                    kappa=0.0,
                    s=0.0,
                    dkappa=0.0,
                    ddkappa=0.0,
                ),
                v=1.0,
                a=0.0,
                relative_time=0.0,
            ),
            TrajectoryPoint(
                path_point=PathPoint(
                    x=11.0,
                    y=0.0,
                    z=0.0,
                    theta=0.0,
                    kappa=0.0,
                    s=0.0,
                    dkappa=0.0,
                    ddkappa=0.0,
                ),
                v=1.0,
                a=0.0,
                relative_time=1.0,
            ),
            TrajectoryPoint(
                path_point=PathPoint(
                    x=12.0,
                    y=0.0,
                    z=0.0,
                    theta=0.0,
                    kappa=0.0,
                    s=0.0,
                    dkappa=0.0,
                    ddkappa=0.0,
                ),
                v=1.0,
                a=0.0,
                relative_time=2.0,
            ),
        ]
    )
    return Obstacle("obs_1", perception, is_static=False, trajectory=trajectory)


def check_obstacle_non_increasing_prediction_time_does_not_abort():
    perception = PerceptionObstacle(
        id=101,
        position=Point3D(x=10.0, y=0.0, z=0.0),
        theta=0.0,
        velocity=Point3D(x=1.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
    )
    trajectory = Trajectory(trajectory_point=[
        TrajectoryPoint(path_point=PathPoint(x=10.0, y=0.0), relative_time=1.0),
        TrajectoryPoint(path_point=PathPoint(x=11.0, y=0.0), relative_time=1.0),
    ])
    obstacle = Obstacle("non_monotonic", perception, trajectory=trajectory)
    assert obstacle.Trajectory().trajectory_point[-1].path_point.s == 1.0


def check_obstacle_uses_full_adc_width_for_blocking():
    import config as config_module
    from protoclass.sl_boundary import SLBoundary

    perception = PerceptionObstacle(
        id=102,
        position=Point3D(x=10.0, y=0.0, z=0.0),
        theta=0.0,
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
    )
    obstacle = Obstacle("static_width", perception, is_static=True)
    obstacle.SetPerceptionSlBoundary(
        SLBoundary(start_s=8.0, end_s=12.0, start_l=-1.0, end_l=1.0)
    )

    class ReferenceLineStub:
        gap = None

        def IsBlockRoad(self, _box, gap):
            self.gap = gap
            return False

    reference_line = ReferenceLineStub()
    obstacle.BuildReferenceLineStBoundary(reference_line, 0.0)
    assert reference_line.gap == config_module.EGO_VEHICLE_WIDTH


def check_obstacle_first_st_search_window_matches_cpp():
    reference_line, _, _ = _build_reference_line()
    obstacle = _build_dynamic_obstacle()
    original = reference_line.GetApproximateSLBoundary
    starts = []

    def capture_start(box, start_s, end_s):
        starts.append(start_s)
        return original(box, start_s, end_s)

    reference_line.GetApproximateSLBoundary = capture_start
    obstacle.BuildTrajectoryStBoundary(reference_line, 50.0)
    assert starts
    assert starts[0] == 0.0


def check_collision_checker_obstacle_behind_ego_no_crash():
    """
    Regression test: CollisionChecker.__init__ used to call
    BuildPredictedEnvironment (which logs a warning via self.logger) before
    assigning self.logger, so any obstacle behind the ADC in-lane crashed
    the constructor with AttributeError. Verify construction now succeeds
    and that obstacle is correctly filtered out of the predicted environment.
    """
    reference_line, discretized_ref_points, reference_line_info = _build_reference_line()
    perception = PerceptionObstacle(
        id=901, position=Point3D(x=-5.0, y=0.0, z=0.0), theta=0.0,
        velocity=Point3D(x=0.0, y=0.0, z=0.0), length=4.0, width=2.0, height=1.5,
    )
    trajectory = Trajectory(trajectory_point=[
        TrajectoryPoint(
            path_point=PathPoint(x=-5.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
            v=0.0, a=0.0, relative_time=0.0,
        ),
    ])
    behind_obstacle = Obstacle("behind_obs", perception, is_static=True, trajectory=trajectory)

    checker = CollisionChecker([behind_obstacle], 10.0, 0.0, discretized_ref_points, reference_line_info, None)
    assert checker.logger is not None
    assert checker.predicted_bounding_rectangles
    assert all(len(env) == 0 for env in checker.predicted_bounding_rectangles), (
        "obstacle behind the ADC in-lane should be excluded from the predicted environment"
    )


def check_st_boundary_expand_by_t():
    """
    Regression test: STBoundary.ExpandByT used to call list.append(a, b)
    with two positional arguments instead of a tuple, raising TypeError
    on every call. Verify it now expands the boundary's time range.
    """
    pp = [
        (STPoint(0.0, 0.0), STPoint(5.0, 0.0)),
        (STPoint(3.0, 2.0), STPoint(8.0, 2.0)),
    ]
    boundary = STBoundary(pp)
    expanded = boundary.ExpandByT(1.0)
    assert expanded.min_t < boundary.min_t
    assert expanded.max_t > boundary.max_t


def check_st_boundary_out_of_range_index_does_not_raise():
    boundary = STBoundary()
    ok, left, right = boundary.GetIndexRange(
        [STPoint(0.0, 0.0), STPoint(1.0, 1.0)], 2.0
    )
    assert not ok
    assert left == 0
    assert right == 0


def check_st_graph_data_set_st_drivable_boundary():
    """
    Regression test: StGraphData._st_drivable_boundary was never
    initialized (stayed None) and the append() call sat outside the loop
    body, so SetSTDrivableBoundary either crashed with AttributeError or
    silently dropped all but the last point. Verify every point is kept.
    """
    sg = StGraphData()
    s_boundary = [(0.0, 1.0, 2.0), (1.0, 2.0, 3.0), (2.0, 3.0, 4.0)]
    v_obs_info = [(0.0, -1.0, 1.0), (1.0, -1.0, 1.0), (2.0, -1.0, 1.0)]
    assert sg.SetSTDrivableBoundary(s_boundary, v_obs_info)
    assert len(sg.st_drivable_boundary.st_boundary) == len(s_boundary)


def check_obstacle_build_trajectory_st_boundary():
    """
    Regression test: Obstacle.BuildTrajectoryStBoundary called
    ReferenceLine.GetApproximateSLBoundary with 4 positional arguments
    (mimicking the C++ output-parameter style) against a 3-argument,
    tuple-returning Python signature, raising TypeError for any moving
    obstacle with a predicted trajectory -- a normal lattice code path.
    """
    reference_line, _, _ = _build_reference_line()
    obstacle = _build_dynamic_obstacle()
    ok, boundary = obstacle.BuildTrajectoryStBoundary(reference_line, 0.0)
    assert ok is True
    assert isinstance(boundary, STBoundary)
    assert boundary.min_s <= boundary.max_s
    assert boundary.min_t <= boundary.max_t


def check_path_approximation_matches_exact_projection():
    """
    Regression test: PathApproximation.__init__ called self.Init(path)
    before initializing its own attributes, crashing any Path constructed
    with max_approximation_error > 0. Also covers two bugs that were
    masked behind it: PathApproximation.is_within_max_error() was called
    with one fewer positional argument than it requires, and
    PathApproximation.GetProjection() used Python's len() on a
    LineSegment2d (which has no __len__) instead of calling .length().
    Verify the approximated projection matches the exact one.
    """
    points = [MapPathPoint(Vec2d(0.0, 0.0)), MapPathPoint(Vec2d(10.0, 0.0)),
              MapPathPoint(Vec2d(20.0, 0.0)), MapPathPoint(Vec2d(30.0, 0.0)),
              MapPathPoint(Vec2d(40.0, 0.0))]
    approximated_path = MapPath(points, [], 0.5)
    exact_path = MapPath(points, [])

    query_point = Vec2d(20.5, 0.05)
    approx_result = approximated_path.GetProjection(query_point)
    exact_result = exact_path.GetProjection(query_point)
    assert approx_result[0] is True
    for approx_value, exact_value in zip(approx_result, exact_result):
        assert abs(approx_value - exact_value) < 1e-6


def check_path_approximation_projection_samples_advance():
    points = [MapPathPoint(Vec2d(float(i), 0.0)) for i in range(41)]
    path = MapPath(points, [], 0.5)
    samples = path._approximation._sampled_max_original_projections_to_left
    assert len(samples) > 1
    assert samples[-1] > samples[0]


def check_path_get_projection_with_warm_start_s():
    """
    Regression test: Path.GetProjectionWithWarmStartS called
    segment.start() on a LineSegment2d, where "start" is a Python
    @property (not a method), raising TypeError as soon as the binary
    search entered its loop (any path with >= 3 points).
    """
    points = [MapPathPoint(Vec2d(0.0, 0.0)), MapPathPoint(Vec2d(10.0, 0.0)),
              MapPathPoint(Vec2d(20.0, 0.0)), MapPathPoint(Vec2d(30.0, 0.0))]
    path = MapPath(points, [])
    ok, accumulate_s, lateral = path.GetProjectionWithWarmStartS(Vec2d(15.0, 1.0), 15.0)
    assert ok is True
    assert abs(accumulate_s - 15.0) < 1e-6
    assert abs(lateral - 1.0) < 1e-6


def check_vec2d_rmul():
    """
    Regression test: Vec2d only implemented __mul__ (Vec2d * scalar), not
    __rmul__ (scalar * Vec2d), raising TypeError anywhere C++'s free
    `operator*(double, const Vec2d&)` is used, e.g. Polygon2d.BoundingBoxWithHeading.
    """
    result = 0.5 * Vec2d(2.0, 4.0)
    assert abs(result.x - 1.0) < 1e-9
    assert abs(result.y - 2.0) < 1e-9


def check_aabox2d_distance_to():
    """
    Regression test: AABox2d.DistanceTo checked isinstance([0], Vec2d) /
    isinstance([0], AABox2d) -- the literal list [0], not args[0] -- so it
    always raised ValueError regardless of input.
    """
    box = AABox2d(Vec2d(0.0, 0.0), 2.0, 2.0)
    assert abs(box.DistanceTo(Vec2d(5.0, 0.0)) - 4.0) < 1e-9
    other = AABox2d(Vec2d(10.0, 0.0), 2.0, 2.0)
    assert abs(box.DistanceTo(other) - 8.0) < 1e-9


def check_box2d_has_overlap_line_segment():
    """
    Regression test: Box2d.HasOverlap(LineSegment2d) built its rotated
    start/end points via Vec2d(vec - vec) (double-wrapping an already
    constructed Vec2d, corrupting it) and its is_inside_rectangle() helper
    called point.y() on a Python @property, both raising exceptions for
    any non-trivially-rejected segment.
    """
    box = Box2d(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0)
    crossing_segment = LineSegment2d(Vec2d(-1.0, 0.0), Vec2d(1.0, 0.0))
    assert box.HasOverlap(crossing_segment) is True
    far_segment = LineSegment2d(Vec2d(100.0, 100.0), Vec2d(101.0, 101.0))
    assert box.HasOverlap(far_segment) is False


def check_st_point_from_vec2d():
    """
    Regression test: STPoint(Vec2d) read args[0].__dict (missing the
    trailing "__"), raising AttributeError on every call. Also covers the
    STPoint.x/.y properties, which used to always raise -- correct for a
    caller holding a *statically-typed* C++ Vec2d& (where the deleted
    STPoint::x()/y() never enters the call), but wrong in Python, where
    attribute access is always dynamically dispatched; library code like
    LineSegment2d's constructor needs .x/.y to keep working when handed
    STPoint instances (used as Vec2d(t, s)) directly, e.g. inside
    STBoundary.RemoveRedundantPoints.
    """
    point = STPoint(Vec2d(3.0, 7.0))
    assert point.t == 3.0
    assert point.s == 7.0
    assert point.x == 3.0
    assert point.y == 7.0


def check_polygon2d_get_overlap_no_overlap_does_not_crash():
    """
    Regression test: Polygon2d.GetOverlap only assigned its `first`/`last`
    locals inside conditional branches (mirroring C++'s output-pointer
    params, which are simply left untouched when unused). Since Python has
    no such "leave untouched" semantics for plain locals, a line segment
    whose bounding box overlaps the polygon but that neither starts/ends
    inside it nor crosses any edge -- a routine "no overlap" case -- hit
    UnboundLocalError on the final return.
    """
    triangle = Polygon2d([Vec2d(0.0, 0.0), Vec2d(4.0, 0.0), Vec2d(0.0, 4.0)])
    no_overlap_segment = LineSegment2d(Vec2d(3.5, 3.5), Vec2d(3.8, 3.2))
    has_overlap, first, last = triangle.GetOverlap(no_overlap_segment)
    assert has_overlap is False
    assert first is None and last is None

    crossing_segment = LineSegment2d(Vec2d(-1.0, 1.0), Vec2d(1.0, 1.0))
    has_overlap, first, last = triangle.GetOverlap(crossing_segment)
    assert has_overlap is True
    assert first is not None and last is not None


def check_path_overlap_with_approximation_enabled():
    """
    Regression test: Path.OverlapWith called
    self._approximation.OverlapWith(box, width), omitting the leading `path`
    argument PathApproximation.OverlapWith requires (mirroring
    approximation_.OverlapWith(*this, box, width) in path.cc), raising
    TypeError for any Path built with max_approximation_error > 0.
    """
    points = [MapPathPoint(Vec2d(0.0, 0.0)), MapPathPoint(Vec2d(10.0, 0.0)),
              MapPathPoint(Vec2d(20.0, 0.0))]
    approximated_path = MapPath(points, [], 0.5)
    overlapping_box = Box2d(Vec2d(10.0, 0.5), 0.0, 2.0, 1.0)
    assert approximated_path.OverlapWith(overlapping_box, 0.1) is True
    far_box = Box2d(Vec2d(100.0, 100.0), 0.0, 2.0, 1.0)
    assert approximated_path.OverlapWith(far_box, 0.1) is False


def check_kd_tree_nearest_object_matches_brute_force():
    """
    Regression test: AABoxKDTree2dNode.GetNearestObjectInternal threaded
    min_distance_sqr/nearest_object as plain return values instead of the
    C++ in-out pointer pair, so (a) a leaf node whose own objects didn't
    beat an already-found sibling's distance hit UnboundLocalError on
    `nearest_object`, and (b) even where it didn't crash, a subtree search
    that found nothing better than the incoming bound would unconditionally
    overwrite (and silently discard) a nearest_object already found by an
    earlier sibling search. Build a tree with enough objects to force
    multi-level node splitting (max_leaf_size=4) and verify the result
    matches a brute-force nearest-neighbor scan.
    """
    import math
    import random

    class _KdTestObject:
        def __init__(self, x: float, y: float):
            self._box = AABox2d(Vec2d(x, y), 0.1, 0.1)
            self.x = x
            self.y = y

        def aabox(self) -> AABox2d:
            return self._box

        def DistanceTo(self, point: Vec2d) -> float:
            return math.hypot(point.x - self.x, point.y - self.y)

        def DistanceSquareTo(self, point: Vec2d) -> float:
            distance = self.DistanceTo(point)
            return distance * distance

    rng = random.Random(42)
    objects = [_KdTestObject(rng.uniform(0.0, 50.0), rng.uniform(0.0, 50.0)) for _ in range(40)]
    tree = AABoxKDTree2d(objects, AABoxKDTreeParams(max_leaf_size=4))

    for _ in range(10):
        query = Vec2d(rng.uniform(0.0, 50.0), rng.uniform(0.0, 50.0))
        nearest = tree.GetNearestObject(query)
        brute_force_nearest = min(objects, key=lambda obj: obj.DistanceTo(query))
        assert nearest is not None
        assert abs(nearest.DistanceTo(query) - brute_force_nearest.DistanceTo(query)) < 1e-9


def check_obstacle_validators_treat_none_as_protobuf_zero_default():
    """
    Regression test: an unset protobuf scalar field reads back as its type's
    zero default in C++ (a `double` field that was never set() is 0.0, not
    null); this project's dataclasses instead default such fields to `None`.
    Obstacle.IsValidPerceptionObstacle/IsValidTrajectoryPoint/IsValidObstacle
    exist specifically to validate untrusted external (prediction-module)
    input, and used to call math.isnan()/do comparisons directly on fields
    that can legitimately be None, raising TypeError where C++ (reading the
    protobuf default) degrades gracefully to "invalid". Mirrors
    modules/planning/common/obstacle_test.cc's own default-message test.
    """
    # A completely unset PerceptionObstacle -- exactly the scenario the
    # vendored C++ unit test (obstacle_test.cc) exercises -- must be
    # rejected, not crash.
    assert Obstacle.IsValidPerceptionObstacle(PerceptionObstacle()) is False
    assert Obstacle.IsValidPerceptionObstacle(
        PerceptionObstacle(length=4.0, width=2.0, height=1.5)
    ) is True

    assert Obstacle.IsValidObstacle(PerceptionObstacle(length=4.0)) is False
    assert Obstacle.IsValidObstacle(PerceptionObstacle(length=4.0, width=2.0)) is True

    sparse_trajectory_point = TrajectoryPoint(
        path_point=PathPoint(x=0.0, y=0.0, z=0.0, kappa=0.0, s=0.0)
    )
    assert Obstacle.IsValidTrajectoryPoint(sparse_trajectory_point) is True


def check_create_obstacles_uses_perception_validator_like_cpp():
    """
    Regression test: Obstacle.CreateObstacles used to gate obstacles through
    IsValidObstacle (which only checks width/length), but
    modules/planning/common/obstacle.cc:200 gates through
    IsValidPerceptionObstacle (which additionally checks height and, when
    present, velocity/polygon-point NaNs). An obstacle with valid
    width/length but a NaN velocity must be rejected, matching C++, not
    silently accepted.
    """
    bad_velocity_obstacle = PerceptionObstacle(
        id=1, length=4.0, width=2.0, height=1.5,
        velocity=Point3D(x=float("nan"), y=0.0, z=0.0),
    )
    predictions = PredictionObstacles(
        prediction_obstacle=[
            PredictionObstacle(perception_obstacle=bad_velocity_obstacle, is_static=True)
        ]
    )
    assert Obstacle.CreateObstacles(predictions) == []


def check_path_evaluate_does_not_alias_stored_points():
    """Evaluate used to hand back the stored endpoint. Combine then did
    path_point.s += start_s and rewrote the path under itself."""
    from common.discretized_path import DiscretizedPath
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.path_data import PathData
    from common.speed_data import SpeedData

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    stored = DiscretizedPath([
        PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
        PathPoint(x=1.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=1.0, dkappa=0.0, ddkappa=0.0),
        PathPoint(x=2.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=2.0, dkappa=0.0, ddkappa=0.0),
    ])
    shifted = stored.Evaluate(0.0)
    shifted.s += 5.0
    assert stored[0].s == 0.0
    past_end = stored.Evaluate(10.0)
    past_end.s += 1.0
    assert stored[-1].s == 2.0

    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    assert path_data.SetDiscretizedPath(stored)
    speed = SpeedData()
    speed.AppendSpeedPoint(0.0, 0.0, 1.0, 0.0, 0.0)
    speed.AppendSpeedPoint(2.0, 2.0, 1.0, 0.0, 0.0)
    reference_line_info.SetPathData(path_data)
    reference_line_info.SetSpeedData(speed)
    trajectory = DiscretizedTrajectory()
    assert reference_line_info.CombinePathAndSpeedProfile(0.0, 4.0, trajectory)
    assert stored[0].s == 0.0
    assert stored[-1].s == 2.0
    assert trajectory[0].path_point.s == 4.0

    played = trajectory.Evaluate(trajectory[0].relative_time or 0.0)
    played.v = 99.0
    assert trajectory[0].v != 99.0


def check_discretized_path_evaluate_reverse_matches_cpp_reverse_case():
    """
    Regression test: DiscretizedPath.QueryUpperBound used bisect_right, which
    assumes an ascending `s` array. C++'s QueryUpperBound (discretized_path.cc)
    is std::upper_bound with a comparator that only makes sense for a
    *descending* `s` array -- it backs EvaluateReverse, used for reverse/
    parking paths. On a descending array bisect_right always returns len(self),
    so EvaluateReverse silently degenerated to "always return the last point,
    never interpolate". Reproduces
    modules/planning/common/path/discretized_path_test.cc's reverse_case
    fixture and expected values exactly.
    """
    from common.discretized_path import DiscretizedPath

    sqrt2 = math.sqrt(2.0)
    s1 = 0.0
    s2 = s1 - sqrt2
    s3 = s2 - sqrt2
    s4 = s3 - sqrt2
    discretized_path = DiscretizedPath([
        PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=s1),
        PathPoint(x=1.0, y=1.0, z=0.0, theta=0.0, kappa=0.0, s=s2),
        PathPoint(x=2.0, y=2.0, z=0.0, theta=0.0, kappa=0.0, s=s3),
        PathPoint(x=3.0, y=3.0, z=0.0, theta=0.0, kappa=0.0, s=s4),
    ])
    assert math.isclose(discretized_path.Length(), -sqrt2 * 3.0)

    eval_p1 = discretized_path.EvaluateReverse(0.0)
    assert math.isclose(eval_p1.s, 0.0, abs_tol=1e-9)
    assert math.isclose(eval_p1.x, 0.0, abs_tol=1e-9)
    assert math.isclose(eval_p1.y, 0.0, abs_tol=1e-9)

    eval_p2 = discretized_path.EvaluateReverse(-0.3 * sqrt2)
    assert math.isclose(eval_p2.x, 0.3, abs_tol=1e-9)
    assert math.isclose(eval_p2.y, 0.3, abs_tol=1e-9)

    eval_p3 = discretized_path.EvaluateReverse(-1.8)
    assert math.isclose(eval_p3.x, (1.0 + 0.8) / sqrt2, abs_tol=1e-9)
    assert math.isclose(eval_p3.y, (1.0 + 0.8) / sqrt2, abs_tol=1e-9)

    eval_p4 = discretized_path.EvaluateReverse(-2.5)
    assert math.isclose(eval_p4.x, (2.0 + 0.5) / sqrt2, abs_tol=1e-9)
    assert math.isclose(eval_p4.y, (2.0 + 0.5) / sqrt2, abs_tol=1e-9)


def check_path_data_sl_to_xy_matches_cpp_unconditional_dkappa_division():
    """
    Regression test: PathData.SLToXY guarded `dkappa = (kappa - last.kappa) /
    distance` with `if distance > 1e-6`, silently returning 0.0 whenever two
    consecutive frenet samples map to (near-)identical cartesian points. C++
    (path_data.cc SLToXY) divides unconditionally; when distance is exactly
    0.0 that is a real IEEE-754 +-inf/nan, not a translation artifact to be
    papered over.
    """
    from common.path_data import PathData
    from common.frenet_frame_path import FrenetFramePath
    from protoclass.frenet_frame_point import FrenetFramePoint
    from common.map_path_point import MapPathPoint

    class _CoincidentPointReferenceLine:
        def __init__(self, kappas):
            self._kappas = list(kappas)
            self._calls = 0

        def SLToXY(self, sl_point):
            return True, Vec2d(1.0, 1.0)

        def GetReferencePoint(self, s):
            kappa = self._kappas[self._calls]
            self._calls += 1
            return ReferencePoint(MapPathPoint(point=(1.0, 1.0), heading=0.0), kappa=kappa, dkappa=0.0)

    path_data = PathData()
    path_data.SetReferenceLine(_CoincidentPointReferenceLine(kappas=[0.0, 1.0]))
    frenet_path = FrenetFramePath([
        FrenetFramePoint(s=0.0, l=0.0, dl=0.0, ddl=0.0),
        FrenetFramePoint(s=1.0, l=0.0, dl=0.0, ddl=0.0),
    ])
    ok, discretized_path = path_data.SLToXY(frenet_path)
    assert ok
    assert discretized_path[0].s == 0.0
    assert discretized_path[1].s == 0.0, "coincident cartesian points must not advance s"
    assert discretized_path[1].dkappa == math.inf














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
    predicted = planned.GetPointAtTime(0.0)
    assert abs(predicted.path_point.x - 0.2) < 1e-6

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
    check_adjust_trajectory_resamples_cut_trajectory()
    check_collision_checker_lane_width_fallback()
    check_path_time_graph_lane_width_fallback()
    check_constraint_checker_dynamic_speed_bound()
    check_constraint_checker_matches_cpp_non_increasing_time()
    check_constraint_checker1d_dynamic_speed_bound()
    check_dynamic_obstacle_sampling()
    check_cruise_sampler_matches_cpp_negative_range_cast()
    check_reference_line_sl_boundary_lane_width_failure_matches_cpp()
    check_lateral_osqp_optimizer()
    check_lateral_osqp_keeps_usable_non_solved_result()
    check_lateral_bundle_ignores_optimizer_return_value()
    check_prediction_time_alignment()
    check_reference_point_remove_duplicates_uses_euclidean_distance()
    check_reference_line_get_sl_boundary_failure_is_false()
    check_reference_line_info_copies_reference_line()
    check_obstacle_decision_property_api()
    check_obstacle_copies_trajectory()
    check_obstacle_empty_trajectory_is_static_for_st_graph()
    check_path_time_graph_keeps_ignored_static_obstacle()
    check_polygon_box_distance()
    check_polygon_overlap_contains_line_segment()
    check_polygon_bounding_box_with_heading_uses_cross_projection()
    check_polygon_extreme_points_accepts_triangle()
    check_route_segments_uses_cpp_segmentation_epsilon()
    check_box_distance_to_segment_canonical_state()
    check_lattice_main_path_without_backup()
    check_collision_checker_obstacle_behind_ego_no_crash()
    check_st_boundary_expand_by_t()
    check_st_boundary_out_of_range_index_does_not_raise()
    check_st_graph_data_set_st_drivable_boundary()
    check_obstacle_build_trajectory_st_boundary()
    check_obstacle_non_increasing_prediction_time_does_not_abort()
    check_obstacle_uses_full_adc_width_for_blocking()
    check_obstacle_first_st_search_window_matches_cpp()
    check_path_approximation_matches_exact_projection()
    check_path_approximation_projection_samples_advance()
    check_path_get_projection_with_warm_start_s()
    check_vec2d_rmul()
    check_aabox2d_distance_to()
    check_box2d_has_overlap_line_segment()
    check_st_point_from_vec2d()
    check_polygon2d_get_overlap_no_overlap_does_not_crash()
    check_path_overlap_with_approximation_enabled()
    check_kd_tree_nearest_object_matches_brute_force()
    check_obstacle_validators_treat_none_as_protobuf_zero_default()
    check_create_obstacles_uses_perception_validator_like_cpp()
    check_path_evaluate_does_not_alias_stored_points()
    check_discretized_path_evaluate_reverse_matches_cpp_reverse_case()
    check_path_data_sl_to_xy_matches_cpp_unconditional_dkappa_division()
    check_apollo_map_borregas_ave_loads()
    check_apollo_map_exports_commonroad_lanelets()
    check_loaded_commonroad_scenario_is_driven()
    print("lattice component checks succeeded")


if __name__ == "__main__":
    main()
