#!/usr/bin/env python3
"""Run focused smoke checks for Apollo lattice components."""

from pathlib import Path
import importlib
import sys
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from lattice_planner import ToDiscretizedReferenceLine
from behavior.collision_checker import CollisionChecker
from behavior.path_time_graph import PathTimeGraph
from behavior.prediction_querier import PredictionQuerier
from common.discretized_trajectory import DiscretizedTrajectory
from common.frame import Frame, LocalView
from common.hd_map import HDMap, HDMapUtil, MakeMapId, BaseMapFile
from common.map_path_point import MapPathPoint
from common.obstacle import Obstacle
from common.path import Path as MapPath, PathOverlap
from common.pnc_map import PncMap
from reference_line import ReferenceLine
from reference_line.reference_line_info import ReferenceLineInfo
from reference_line.reference_line_provider import ReferenceLineProvider
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
from common.lane_types import LaneSegment
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.lane import Curve, CurveSegment, Lane, LaneBoundary, LaneBoundaryType, LaneSampleAssociation, LineSegment
from protoclass.point_enu import PointENU
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
from protoclass.decision_result import ChangeLaneType
from protoclass.lane_waypoint import LaneWaypoint as RoutingLaneWaypoint
from protoclass.routing import Passage, RoadSegment, RoutingRequest, RoutingResponse
from protoclass.planning_status import LaneSegment as RoutingLaneSegment
from protoclass.lattice_structure import StopPoint
from traffic_rules.traffic_decider import TrafficDecider, TrafficRuleConfig, TrafficRuleConfigs
from trajectory_generation.backup_trajectory_generator import BackupTrajectoryGenerator
from trajectory_generation.end_condition_sampler import EndConditionSampler
from trajectory_generation.lattice_trajectory1d import CreateLatticeTrajectory1d
from trajectory_generation.lateral_osqp_optimizer import LateralOSQPOptimizer
from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator


def _build_reference_line():
    hdmap = HDMap()
    lane_info = hdmap.AddLane(_build_straight_lane(length=100.0))
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("mock")
    route_segments.append(LaneSegment(lane_info, 0.0, 99.0))

    reference_line = ReferenceLine(MapPath(route_segments))
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


def _build_straight_lane(lane_id="lane_1", length: float = 50.0):
    return Lane(
        id=Lane.Id(lane_id),
        central_curve=Curve(
            segment=[
                CurveSegment(
                    curve_type=LineSegment(
                        point=[PointENU(x=0.0, y=0.0), PointENU(x=length, y=0.0)]
                    )
                )
            ]
        ),
        length=length,
        speed_limit=10.0,
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        left_road_sample=[LaneSampleAssociation(s=0.0, width=3.0)],
        right_road_sample=[LaneSampleAssociation(s=0.0, width=3.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )


def check_hdmap_basic_queries():
    hdmap = HDMap()
    lane_info = hdmap.AddLane(_build_straight_lane())
    HDMapUtil.SetBaseMap(hdmap)

    assert HDMapUtil.BaseMap().GetLaneById(MakeMapId("lane_1")) is lane_info
    ok, nearest_lane, nearest_s, nearest_l = hdmap.GetNearestLane(PointENU(x=5.0, y=1.0))
    assert ok
    assert nearest_lane is lane_info
    assert abs(nearest_s - 5.0) < 1e-6
    assert abs(nearest_l - 1.0) < 1e-6
    heading_lanes = hdmap.GetLanesWithHeading(PointENU(x=5.0, y=0.2), 5.0, 0.0, 0.2)
    assert heading_lanes == [lane_info]

    provider = ReferenceLineProvider()
    vehicle_state = VehicleState(x=5.0, y=0.0, heading=0.0)
    ok, route_segments = provider.CreateRouteSegments(vehicle_state)
    assert not ok
    assert route_segments == []


def check_hdmap_get_nearest_lane_uses_clamped_distance():
    """
    Regression test: HDMap.GetNearestLane/GetNearestLaneWithHeading used to
    rank candidate lanes by abs(lane_l) from GetProjection, which is
    deliberately UNCLAMPED beyond a lane's first/last segment (it returns
    perpendicular distance to the infinite extension of that segment, not
    distance to the actual finite lane). C++'s HDMapImpl::GetNearestLane
    ranks by the real clamped distance (LaneInfo::DistanceTo, backed by a
    KD-tree over actual segments). Build a short lane and a longer lane
    further away: query a point far past the short lane's end but roughly
    aligned with its heading, so the unclamped metric would have picked the
    short lane (small perpendicular distance to its infinite extension)
    while the true nearest lane is the other one.
    """
    hdmap = HDMap()
    short_lane = Lane(
        id=Lane.Id("short"),
        central_curve=Curve(segment=[CurveSegment(curve_type=LineSegment(
            point=[PointENU(x=0.0, y=0.0), PointENU(x=5.0, y=0.0)]))]),
        length=5.0,
        speed_limit=10.0,
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )
    far_lane = Lane(
        id=Lane.Id("far"),
        central_curve=Curve(segment=[CurveSegment(curve_type=LineSegment(
            point=[PointENU(x=0.0, y=10.0), PointENU(x=100.0, y=10.0)]))]),
        length=100.0,
        speed_limit=10.0,
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )
    hdmap.AddLane(short_lane)
    far_info = hdmap.AddLane(far_lane)
    HDMapUtil.SetBaseMap(hdmap)

    query = PointENU(x=50.0, y=0.3)
    ok, nearest_lane, s, l = hdmap.GetNearestLane(query)
    assert ok
    assert nearest_lane is far_info, "must pick the truly-nearest lane, not the one with the smallest unclamped projection"
    assert abs(s - 50.0) < 1e-6
    assert abs(l - (-9.7)) < 1e-6


def check_hdmap_get_roads_uses_nearby_lane_membership():
    near_lane = _build_straight_lane("near_lane", length=20.0)
    far_lane = _build_straight_lane("far_lane", length=20.0)
    for point in far_lane.central_curve.segment[0].curve_type.point:
        point.y = 100.0

    road_near = SimpleNamespace(
        id=Lane.Id("road_near"),
        type=1,
        section=[
            SimpleNamespace(
                id=Lane.Id("section_near"), lane_id=[near_lane.id]
            )
        ],
    )
    road_far = SimpleNamespace(
        id=Lane.Id("road_far"),
        type=1,
        section=[
            SimpleNamespace(
                id=Lane.Id("section_far"), lane_id=[far_lane.id]
            )
        ],
    )
    map_proto = SimpleNamespace(
        lane=[near_lane, far_lane], overlap=[], road=[road_near, road_far]
    )

    hdmap = HDMap()
    assert hdmap.LoadMapFromProto(map_proto) == 0
    assert hdmap.GetLaneById("near_lane").road_id == "road_near"
    assert hdmap.GetLaneById("near_lane").section_id == "section_near"
    roads = hdmap.GetRoads(PointENU(x=5.0, y=0.0), 2.0)
    assert [road.id.id for road in roads] == ["road_near"]


def check_map_path_route_segment_regressions():
    hdmap = HDMapUtil.BaseMap()
    lane_info = hdmap.GetLaneById(MakeMapId("lane_1"))
    if lane_info is None:
        lane_info = hdmap.AddLane(_build_straight_lane())

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("lane_1")
    route_segments.append(LaneSegment(lane_info, 0.0, 20.0))

    map_path = MapPath(route_segments)
    assert map_path.length > 0.0
    ok, s, l, _ = map_path.GetProjection(Vec2d(5.0, 1.0))
    assert ok
    assert abs(s - 5.0) < 1e-6
    assert abs(l - 1.0) < 1e-6
    ok, left_width, right_width = map_path.GetLaneWidth(5.0)
    assert ok
    assert left_width > 0.0 and right_width > 0.0
    assert map_path.dead_end_overlaps == []

    reference_line = ReferenceLine(map_path)
    ok, sl = reference_line.XYToSL(Vec2d(5.0, 1.0))
    assert ok
    assert abs(sl.s - 5.0) < 1e-6
    assert reference_line.GetSpeedLimitFromS(5.0) == 10.0

    other = RouteSegments()
    other.append(LaneSegment(lane_info, 10.0, 30.0))
    assert route_segments.Stitch(other)
    assert route_segments.Shrink(8.0, 3.0, 10.0)


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


def _build_parallel_lanes():
    left_lane = _build_straight_lane("lane_left")
    left_lane.left_neighbor_forward_lane_id = []
    left_lane.right_neighbor_forward_lane_id = [Lane.Id("lane_right")]
    left_lane.successor_id = []
    left_lane.predecessor_id = []

    right_lane = Lane(
        id=Lane.Id("lane_right"),
        central_curve=Curve(
            segment=[
                CurveSegment(
                    curve_type=LineSegment(
                        point=[PointENU(x=0.0, y=-3.5), PointENU(x=50.0, y=-3.5)]
                    )
                )
            ]
        ),
        length=50.0,
        speed_limit=10.0,
        left_neighbor_forward_lane_id=[Lane.Id("lane_left")],
        right_neighbor_forward_lane_id=[],
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )
    return left_lane, right_lane


def _build_change_lane_routing():
    return RoutingResponse(
        routing_request=RoutingRequest(
            waypoint=[
                RoutingLaneWaypoint(id="lane_left", s=0.0),
                RoutingLaneWaypoint(id="lane_right", s=45.0),
            ]
        ),
        road=[
            RoadSegment(
                id="road_1",
                passage=[
                    Passage(
                        segment=[RoutingLaneSegment(id="lane_left", start_s=0.0, end_s=50.0)],
                        can_exit=False,
                        change_lane_type=ChangeLaneType.RIGHT,
                    ),
                    Passage(
                        segment=[RoutingLaneSegment(id="lane_right", start_s=0.0, end_s=50.0)],
                        can_exit=True,
                        change_lane_type=ChangeLaneType.FORWARD,
                    ),
                ],
            )
        ],
    )


def check_pnc_map_multi_reference_lines():
    hdmap = HDMap()
    left_lane, right_lane = _build_parallel_lanes()
    hdmap.AddLane(left_lane)
    hdmap.AddLane(right_lane)
    HDMapUtil.SetBaseMap(hdmap)

    pnc_map = PncMap(hdmap)
    routing = _build_change_lane_routing()
    assert pnc_map.UpdateRoutingResponse(routing)

    vehicle_state = VehicleState(x=45.0, y=0.0, heading=0.0, linear_velocity=5.0)
    segments = pnc_map.GetRouteSegments(vehicle_state)
    assert len(segments) == 2
    on_segment = [seg for seg in segments if seg.IsOnSegment()]
    neighbor = [seg for seg in segments if not seg.IsOnSegment()]
    assert len(on_segment) == 1
    assert len(neighbor) == 1
    assert on_segment[0].NextAction() == ChangeLaneType.RIGHT
    assert neighbor[0].PreviousAction() == ChangeLaneType.RIGHT


def check_reference_line_provider_with_routing():
    hdmap = HDMapUtil.BaseMap()
    if hdmap.GetLaneById(MakeMapId("lane_left")) is None:
        left_lane, right_lane = _build_parallel_lanes()
        hdmap.AddLane(left_lane)
        hdmap.AddLane(right_lane)

    provider = ReferenceLineProvider()
    provider.UpdateRoutingResponse(_build_change_lane_routing())
    provider.UpdateVehicleState(VehicleState(x=45.0, y=0.0, heading=0.0, linear_velocity=5.0))
    ok, reference_lines, route_segments = provider.CreateReferenceLine()
    assert ok
    assert len(reference_lines) >= 2
    assert len(reference_lines) == len(route_segments)


def check_reference_line_provider_reuses_unchanged_reference_line():
    hdmap = HDMap()
    lane_info = hdmap.AddLane(_build_straight_lane("stable_lane", length=30.0))
    HDMapUtil.SetBaseMap(hdmap)
    segments = RouteSegments()
    segments.SetIsOnSegment(True)
    segments.SetId("stable")
    segments.append(LaneSegment(lane_info, 0.0, 29.0))
    reference_line = ReferenceLine(MapPath(segments))

    provider = ReferenceLineProvider()
    provider.UpdateReferenceLine([reference_line], [segments])
    cached_reference_line = provider._reference_lines[0]
    cached_route_segment = provider._route_segments[0]
    history_size = len(provider._reference_line_history)

    provider.UpdateReferenceLine([ReferenceLine(MapPath(segments))], [segments])
    assert provider._reference_lines[0] is cached_reference_line
    assert provider._route_segments[0] is cached_route_segment
    assert len(provider._reference_line_history) == history_size + 1

    provider.UpdateReferenceLine([reference_line], [])
    assert provider._reference_lines[0] is cached_reference_line
    assert provider._route_segments[0] is cached_route_segment
    assert len(provider._reference_line_history) == history_size + 1


def check_traffic_decider_stop_point():
    hdmap = HDMap()
    lane_info = hdmap.AddLane(_build_straight_lane("short_lane", length=30.0))
    HDMapUtil.SetBaseMap(hdmap)
    segments = RouteSegments()
    segments.SetIsOnSegment(True)
    segments.SetId("short")
    segments.append(LaneSegment(lane_info, 0.0, 29.0))
    reference_line = ReferenceLine(MapPath(segments))
    vehicle_state = VehicleState(x=0.0, y=0.0, heading=0.0)
    start_point = TrajectoryPoint(
        path_point=PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
        v=1.0,
        a=0.0,
        relative_time=0.0,
    )
    reference_line_info = ReferenceLineInfo(vehicle_state, start_point, reference_line, segments)
    reference_line_info.Init([], 10.0)

    frame = Frame(0)
    frame._reference_line_info = [reference_line_info]
    frame._local_view = LocalView()
    frame._hdmap = HDMapUtil.BaseMap()

    decider = TrafficDecider()
    decider.Init()
    decider.Execute(frame, reference_line_info)
    stop_point = reference_line_info.planning_target.stop_point
    assert stop_point is not None
    assert stop_point.s is not None
    assert stop_point.s < reference_line.Length()
    assert stop_point.type == StopPoint.Type.HARD


def _build_lane_reference_line_info():
    hdmap = HDMap()
    lane_info = hdmap.AddLane(_build_straight_lane())
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("lane_1")
    route_segments.append(LaneSegment(lane_info, 0.0, 50.0))

    reference_line = ReferenceLine(MapPath(route_segments))
    vehicle_state = VehicleState(x=0.0, y=0.0, heading=0.0, linear_velocity=1.0)
    start_point = TrajectoryPoint(
        path_point=PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
        v=1.0,
        a=0.0,
        relative_time=0.0,
    )
    reference_line_info = ReferenceLineInfo(vehicle_state, start_point, reference_line, route_segments)
    assert reference_line_info.Init([], 10.0)

    frame = Frame(0)
    frame._reference_line_info = [reference_line_info]
    frame._local_view = LocalView()
    frame._hdmap = hdmap
    return frame, reference_line_info


def check_yield_sign_rule_stop_point():
    frame, reference_line_info = _build_lane_reference_line_info()
    reference_line_info.reference_line.map_path._yield_sign_overlaps = [
        PathOverlap("yield_1", 20.0, 21.0)
    ]
    decider = TrafficDecider()
    decider.Init(TrafficRuleConfigs([TrafficRuleConfig("YIELD_SIGN")]))
    status = decider.Execute(frame, reference_line_info)
    assert status.ok()
    stop_point = reference_line_info.planning_target.stop_point
    assert stop_point is not None
    assert stop_point.type == StopPoint.Type.HARD
    assert "YS_yield_1" in reference_line_info.path_decision.obstacles


def check_keep_clear_rule_obstacle():
    frame, reference_line_info = _build_lane_reference_line_info()
    reference_line_info.reference_line.map_path._clear_area_overlaps = [
        PathOverlap("clear_1", 15.0, 20.0)
    ]
    decider = TrafficDecider()
    decider.Init(TrafficRuleConfigs([TrafficRuleConfig("KEEP_CLEAR")]))
    status = decider.Execute(frame, reference_line_info)
    assert status.ok()
    obstacle = reference_line_info.path_decision.obstacles.get("KC_clear_1")
    assert obstacle is not None
    assert obstacle.reference_line_st_boundary().boundary_type == STBoundary.BoundaryType.KEEP_CLEAR


def check_destination_rule_respects_passed_destination():
    """
    Regression test: Apollo's C++ Destination::MakeDecisions skips the
    destination stop wall only if the ADC is past it AND the destination
    hasn't already been marked reached (`!dest.has_passed_destination()`).
    The Python port must honor that override too, not just the position
    check, otherwise the stop wall can vanish on a later planning cycle
    after the destination was already marked reached.
    """
    frame, reference_line_info = _build_lane_reference_line_info()
    frame._is_near_destination = True
    frame._reference_line_provider = None

    start_wp = RoutingLaneWaypoint(id="start_lane", s=0.0, pose=PointENU(x=0.0, y=0.0, z=0.0))
    dest_wp = RoutingLaneWaypoint(id="lane_1", s=5.0, pose=PointENU(x=-10.0, y=0.0, z=0.0))
    frame._local_view.routing = RoutingResponse(
        routing_request=RoutingRequest(waypoint=[start_wp, dest_wp])
    )

    decider = TrafficDecider()
    decider.Init(TrafficRuleConfigs([TrafficRuleConfig("DESTINATION")]), frame.planning_context)

    status = decider.Execute(frame, reference_line_info)
    assert status.ok()
    assert FLAGS_destination_obstacle_id not in reference_line_info.path_decision.obstacles, (
        "destination is behind the ADC and not yet marked passed: should not stop here"
    )

    frame.planning_context.planning_status.destination.has_passed_destination = True
    status = decider.Execute(frame, reference_line_info)
    assert status.ok()
    assert FLAGS_destination_obstacle_id in reference_line_info.path_decision.obstacles, (
        "once marked passed, the destination stop wall must still be (re)built"
    )


def check_path_decider_static_nudge():
    from common.path_decider import PathDecider
    from common.obstacle import Obstacle
    from common.planning_util import SetupNominalPathData, GetADCStopDeceleration
    from protoclass.adc_trajectory import Point3D
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    _, _, reference_line_info = _build_reference_line()
    reference_line_info._vehicle_state.linear_velocity = 5.0
    reference_line_info.Init([], 10.0)
    init_s = [0.0, 0.0, 0.0]
    init_d = [0.0, 0.0, 0.0]
    SetupNominalPathData(reference_line_info, init_s[0], init_d, 80.0)
    perception = PerceptionObstacle(
        id=2,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=20.0, y=0.3, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("static_1", perception, is_static=True)
    reference_line_info.AddObstacle(obs)
    status = PathDecider(reference_line_info).Execute(reference_line_info)
    assert status.ok()
    assert obs.HasLateralDecision() or obs.HasLongitudinalDecision()
    assert GetADCStopDeceleration(reference_line_info._vehicle_state, 0.0, 50.0) > 0.0


def check_hdmap_load_from_file_if_available():
    map_file = BaseMapFile()
    if map_file is None:
        return
    hdmap = HDMap()
    try:
        assert hdmap.LoadMapFromFile(map_file) == 0
    except ImportError:
        return
    assert len(hdmap._lanes) > 0


def check_build_frenet_path_from_lat_trajectory():
    from common.curve1d.piecewise_jerk_trajectory1d import PiecewiseJerkTrajectory1d
    from common.planning_util import BuildFrenetPathFromLatTrajectory

    _, _, reference_line_info = _build_reference_line()
    lat = PiecewiseJerkTrajectory1d(0.5, 0.0, 0.0)
    lat.AppendSegment(0.0, 40.0)
    path_data = BuildFrenetPathFromLatTrajectory(reference_line_info, 0.0, lat, 30.0, step=1.0)
    frenet_path = path_data.frenet_frame_path
    assert len(frenet_path) >= 30
    assert abs(frenet_path[0].l - 0.5) < 1e-6
    assert abs(frenet_path[-1].l - 0.5) < 1e-6


def check_path_decider_after_lateral_trajectory():
    from common.path_decider import PathDecider
    from common.obstacle import Obstacle
    from common.curve1d.piecewise_jerk_trajectory1d import PiecewiseJerkTrajectory1d
    from common.planning_util import BuildFrenetPathFromLatTrajectory
    from protoclass.perception_obstacle import PerceptionObstacleType

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    lat = PiecewiseJerkTrajectory1d(0.0, 0.0, 0.0)
    lat.AppendSegment(0.0, 80.0)
    BuildFrenetPathFromLatTrajectory(reference_line_info, 0.0, lat, 80.0)
    perception = PerceptionObstacle(
        id=3,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=20.0, y=0.3, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("static_2", perception, is_static=True)
    reference_line_info.AddObstacle(obs)
    status = PathDecider(reference_line_info).Execute(reference_line_info)
    assert status.ok()
    assert obs.HasLateralDecision() or obs.HasLongitudinalDecision()


def check_qp_spline_reference_line_smoothing():
    old_qp = None
    old_smooth = None
    try:
        import config as config_module

        old_qp = config_module.FLAGS_enable_qp_spline_reference_line
        old_smooth = config_module.FLAGS_enable_smooth_reference_line
        config_module.FLAGS_enable_qp_spline_reference_line = True
        config_module.FLAGS_enable_smooth_reference_line = True
        hdmap = HDMap()
        hdmap.AddLane(_build_straight_lane())
        HDMapUtil.SetBaseMap(hdmap)
        provider = ReferenceLineProvider()
        provider.UpdateRoutingResponse(
            RoutingResponse(
                routing_request=RoutingRequest(
                    waypoint=[
                        RoutingLaneWaypoint(id="lane_1", s=0.0),
                        RoutingLaneWaypoint(id="lane_1", s=90.0),
                    ]
                ),
                road=[
                    RoadSegment(
                        id="road_1",
                        passage=[
                            Passage(
                                segment=[
                                    RoutingLaneSegment(
                                        id="lane_1", start_s=0.0, end_s=99.0
                                    )
                                ],
                                can_exit=True,
                                change_lane_type=ChangeLaneType.FORWARD,
                            )
                        ],
                    )
                ],
            )
        )
        provider.UpdateVehicleState(VehicleState(x=10.0, y=0.0, heading=0.0))
        ok, reference_lines, _ = provider.CreateReferenceLine()
        assert ok
        assert reference_lines[0].Length() > 0.0
    finally:
        if old_qp is not None:
            import config as config_module
            config_module.FLAGS_enable_qp_spline_reference_line = old_qp
            config_module.FLAGS_enable_smooth_reference_line = old_smooth


def check_qp_spline_solver_basic():
    from planning_math.smoothing_spline.osqp_spline_2d_solver import OsqpSpline2dSolver
    from common.vec2d import Vec2d

    t_knots = [0.0, 1.0, 2.0]
    solver = OsqpSpline2dSolver(t_knots, 5)
    solver.reset(t_knots, 5)
    evaluated_t = [0.0, 0.5, 1.0, 1.5, 2.0]
    headings = [0.0] * 5
    xy_points = [Vec2d(t, 0.0) for t in evaluated_t]
    bounds_lon = [1.0] * 5
    bounds_lat = [0.5] * 5
    constraint = solver.mutable_constraint
    assert constraint.add_2d_boundary(evaluated_t, headings, xy_points, bounds_lon, bounds_lat)
    assert constraint.add_second_derivative_smooth_constraint()
    kernel = solver.mutable_kernel
    kernel.add_second_order_derivative_matrix(200.0)
    kernel.add_third_order_derivative_matrix(1000.0)
    kernel.add_regularization(1e-5)
    assert solver.solve()
    assert abs(solver.spline.y(1.0)) < 0.5


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


def check_reference_line_provider_history_fallback():
    provider = ReferenceLineProvider()
    hdmap = HDMap()
    lane_info = hdmap.AddLane(_build_straight_lane("history_lane", length=10.0))
    HDMapUtil.SetBaseMap(hdmap)
    segments = RouteSegments()
    segments.SetIsOnSegment(True)
    segments.SetId("history")
    segments.append(LaneSegment(lane_info, 0.0, 9.0))
    reference_line = ReferenceLine(MapPath(segments))
    provider.UpdateReferenceLine([reference_line], [segments])
    provider._reference_lines = []
    refs, segs = provider.GetReferenceLines()
    assert len(refs) == 1
    assert len(segs) == 1


def check_trajectory_stitcher_reinit():
    from common.trajectory_stitcher import TrajectoryStitcher

    vehicle_state = VehicleState(x=0.0, y=0.0, heading=0.0, linear_velocity=0.0, linear_acceleration=0.0)
    stitching = TrajectoryStitcher.compute_stitching_trajectory(
        vehicle_state, 1.0, 0.1, 20, True, None, []
    )
    assert len(stitching) == 1
    assert stitching[0].path_point.s == 0.0


def check_trajectory_stitcher_preserves_previous_trajectory():
    """
    Regression test: with a real (non-None) previous trajectory, stitching must
    return independent copies of the points it re-bases, not aliases of the
    points still owned by prev_trajectory. It must also not crash when slicing
    a PublishableTrajectory.
    """
    from common.publishable_trajectory import PublishableTrajectory
    from common.trajectory_stitcher import TrajectoryStitcher

    points = [
        TrajectoryPoint(
            path_point=PathPoint(x=float(i), y=0.0, theta=0.0, s=float(i)),
            v=1.0,
            a=0.0,
            relative_time=float(i) * 0.1,
        )
        for i in range(5)
    ]
    prev_trajectory = PublishableTrajectory(header_time=100.0, discretized_trajectory=points)
    snapshot = [(tp.path_point.s, tp.relative_time) for tp in prev_trajectory]

    vehicle_state = VehicleState(x=0.0, y=0.0, heading=0.0, linear_velocity=1.0, linear_acceleration=0.0)
    replan_reason = []
    stitching = TrajectoryStitcher.compute_stitching_trajectory(
        vehicle_state, 100.05, 0.1, 5, False, prev_trajectory, replan_reason
    )

    assert replan_reason == [], f"expected the real stitching path, got a replan: {replan_reason}"
    assert len(stitching) > 1
    assert [(tp.path_point.s, tp.relative_time) for tp in prev_trajectory] == snapshot, (
        "compute_stitching_trajectory must not mutate points still owned by prev_trajectory"
    )
    for i in range(min(len(stitching), len(prev_trajectory))):
        assert stitching[i] is not prev_trajectory[i], (
            "stitched points must be independent copies, not aliases of prev_trajectory's points"
        )


def check_lattice_path_assessment_blocking():
    from common.path_assessment_decider import ApplyLatticePathAssessment, FindBlockingObstacleId
    from common.obstacle import Obstacle
    from common.planning_util import SetupNominalPathData
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    SetupNominalPathData(reference_line_info, 0.0, [0.0, 0.0, 0.0], 80.0)
    perception = PerceptionObstacle(
        id=10,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=20.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    obs = Obstacle("block_1", perception, is_static=True)
    reference_line_info.AddObstacle(obs)
    ApplyLatticePathAssessment(reference_line_info)
    assert FindBlockingObstacleId(reference_line_info) == "block_1"
    assert reference_line_info.GetBlockingObstacle() is not None


def check_on_lane_planning_output():
    import config as config_module
    from on_lane_planning import OnLanePlanning
    from common.frame import LocalView
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.pose import Pose
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.prediction_obstacles import PredictionObstacles
    from common.planning_context import PlanningContext

    reference_line, _, reference_line_info = _build_reference_line()
    provider = ReferenceLineProvider()
    provider._reference_lines = [reference_line]
    provider._route_segments = [reference_line_info.Lanes()]
    local_view = LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(
                position=PointENU(x=0.0, y=0.0, z=0.0),
                heading=0.0,
            ),
            measurement_time=0.0,
        ),
        chassis=Chassis(speed_mps=0.1, header=Header(timestamp_sec=0.0)),
        prediction_obstacles=PredictionObstacles(),
    )
    old_thread = config_module.FLAGS_enable_reference_line_provider_thread
    try:
        config_module.FLAGS_enable_reference_line_provider_thread = True
        planner = OnLanePlanning(provider)
        adc_trajectory = ADCTrajectory()
        planning_context = PlanningContext()
        status = planner.RunOnce(local_view, adc_trajectory, planning_context)
        assert status.ok()
        assert len(adc_trajectory.trajectory_point) > 0
        assert adc_trajectory.decision is not None
        assert planner._vehicle_state_provider.vehicle_state is not None
        assert planner._vehicle_state_provider.vehicle_state.linear_velocity == 0.1
        assert planner._last_frame.planning_context is planning_context
    finally:
        config_module.FLAGS_enable_reference_line_provider_thread = old_thread


def check_path_assessment_compare_paths():
    from common.path_assessment_decider import ComparePathData, PathAssessmentDecider
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    short_path = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 20.0, path_label="regular/self"
    )
    long_path = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 60.0, path_label="regular/self"
    )
    assert ComparePathData(long_path, short_path, None)
    reference_line_info.SetCandidatePathData([short_path, long_path])
    status = PathAssessmentDecider().Process(None, reference_line_info, None)
    assert status.ok()
    assert reference_line_info.path_data.frenet_frame_path[-1].s >= 59.0


def check_combine_path_and_speed_profile():
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.planning_util import BuildLatticeCandidatePath, BuildSpeedDataFromLonTrajectory
    from common.curve1d.piecewise_jerk_trajectory1d import PiecewiseJerkTrajectory1d

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    path_data = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 80.0, path_label="regular/self"
    )
    reference_line_info.SetPathData(path_data)
    lon = PiecewiseJerkTrajectory1d(0.0, 5.0, 0.0)
    lon.AppendSegment(0.0, 8.0)
    reference_line_info.SetSpeedData(BuildSpeedDataFromLonTrajectory(lon, 0.0))
    trajectory = DiscretizedTrajectory()
    assert reference_line_info.CombinePathAndSpeedProfile(0.0, 0.0, trajectory)
    assert len(trajectory) > 0


def check_path_assessment_keeps_cpp_default_obstacle_distance():
    from common.path_assessment_decider import SetPathInfo
    from common.obstacle import Obstacle
    from common.planning_util import BuildLatticeCandidatePath
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    perception = PerceptionObstacle(
        id=11,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=15.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    reference_line_info.AddObstacle(Obstacle("dist_obs", perception, is_static=True))
    path_data = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 40.0, path_label="regular/self"
    )
    SetPathInfo(reference_line_info, path_data)
    assert path_data.path_point_decision_guide
    assert all(
        distance == float("inf")
        for _, _, distance in path_data.path_point_decision_guide
    )


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


def check_path_bounds_decider_multi_candidates():
    from common.obstacle import Obstacle
    from common.path_bounds_decider import PathBoundsDecider, BuildCandidatePathsFromBoundaries
    from common.planning_util import SetupNominalPathData
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    SetupNominalPathData(reference_line_info, 0.0, [0.0, 0.0, 0.0], 80.0)
    perception = PerceptionObstacle(
        id=12,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=25.0, y=0.0, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    reference_line_info.AddObstacle(Obstacle("bounds_block", perception, is_static=True))
    reference_line_info.SetBlockingObstacle("bounds_block")

    status = PathBoundsDecider().Process(None, reference_line_info, None)
    assert status.ok()
    boundaries = reference_line_info.GetCandidatePathBoundaries()
    assert len(boundaries) >= 2
    labels = {b.label for b in boundaries}
    assert "fallback" in labels
    assert any("left" in label or "right" in label or "self" in label for label in labels)

    candidates = BuildCandidatePathsFromBoundaries(reference_line_info)
    assert len(candidates) >= 1
    candidate_labels = {c.path_label for c in candidates}
    assert "fallback" not in candidate_labels
    assert any("left" in label or "right" in label or "self" in label for label in candidate_labels)


def check_path_bounds_decider_respects_committed_borrow_direction():
    from common.path_bounds_decider import PathBoundsDecider
    from common.planning_context import PlanningContext

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.set_is_path_lane_borrow(True)

    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ctx.planning_status.path_decider.decided_side_pass_direction = [1]

    status = PathBoundsDecider().Process(None, reference_line_info, ctx)
    assert status.ok()
    labels = {b.label for b in reference_line_info.GetCandidatePathBoundaries()}
    assert any("left" in label for label in labels)
    assert not any("right" in label for label in labels)


def check_path_bounds_static_obstacle_tightens_boundary():
    from common.path_bounds_decider import PathBoundsDecider
    from protoclass.sl_boundary import SLBoundary

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    perception = PerceptionObstacle(
        id=121,
        position=Point3D(x=10.0, y=-1.25, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=4.0,
        width=0.5,
        height=1.5,
        theta=0.0,
    )
    obstacle = Obstacle("right_side", perception, is_static=True)
    obstacle.SetPerceptionSlBoundary(
        SLBoundary(start_s=9.0, end_s=11.0, start_l=-1.5, end_l=-1.0)
    )
    reference_line_info.path_decision.AddObstacle(obstacle)
    path_bound = [
        (5.0, -2.0, 2.0),
        (10.0, -2.0, 2.0),
        (12.0, -2.0, 2.0),
    ]
    blocking_id = [""]
    assert PathBoundsDecider()._get_boundary_from_static_obstacles(
        reference_line_info, path_bound, blocking_id
    )
    assert path_bound[0][1] == -2.0
    assert path_bound[1][1] > -2.0


def check_record_debug_info():
    from common.path_assessment_decider import PathAssessmentDecider, RecordDebugInfo
    from common.path_bounds_decider import PathBoundsDecider
    from common.planning_debug import RecordPathBoundaryDebugInfo
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)
    short_path = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 20.0, path_label="regular/self"
    )
    long_path = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 60.0, path_label="regular/left/forward"
    )
    reference_line_info.SetCandidatePathData([short_path, long_path])
    status = PathAssessmentDecider().Process(None, reference_line_info, None)
    assert status.ok()

    debug_paths = reference_line_info.debug.planning_data.path
    assert debug_paths
    names = {p.name for p in debug_paths}
    assert "Planning PathData" in names
    assert any(name.startswith("Candidate/") for name in names)

    PathBoundsDecider().Process(None, reference_line_info, None)
    boundary = reference_line_info.GetCandidatePathBoundaries()[0]
    RecordPathBoundaryDebugInfo(boundary, "test_boundary", reference_line_info)
    names = {p.name for p in reference_line_info.debug.planning_data.path}
    assert any("test_boundary/left" in name for name in names)
    assert any("test_boundary/right" in name for name in names)

    RecordDebugInfo(short_path, "ManualPath", reference_line_info)
    assert "ManualPath" in {p.name for p in reference_line_info.debug.planning_data.path}


def check_box_polygon_overlap():
    from common.box2d import Box2d
    from common.polygon2d import Polygon2d
    from common.vec2d import Vec2d

    box = Box2d(Vec2d(10.0, 0.0), 0.0, 4.0, 2.0)
    polygon = Polygon2d(box)
    separated = Box2d(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0)
    assert not separated.HasOverlap(polygon)
    assert not polygon.HasOverlap(separated)
    overlapping = Box2d(Vec2d(10.0, 0.0), 0.0, 4.8, 2.0)
    assert box.HasOverlap(polygon)
    assert overlapping.HasOverlap(polygon)


def check_box_distance_to_segment_canonical_state():
    from common.box2d import Box2d
    from common.line_segment2d import LineSegment2d
    from common.vec2d import Vec2d

    box = Box2d(Vec2d(0.0, 0.0), 0.0, 2.0, 2.0)
    segment = LineSegment2d(Vec2d(3.0, 1.5), Vec2d(-1.0, 1.5))
    assert abs(box.DistanceTo(segment) - 0.5) < 1e-6


def check_lattice_migration_pipeline():
    """PathBounds + PathAssessment decider stack (OnLane layer, not LatticePlanner core)."""
    from common.path_assessment_decider import PathAssessmentDecider, RecordDebugInfo
    from common.path_bounds_decider import BuildCandidatePathsFromBoundaries, PathBoundsDecider

    _, _, reference_line_info = _build_reference_line()
    reference_line_info.Init([], 10.0)

    PathBoundsDecider().Process(None, reference_line_info, None)
    candidates = BuildCandidatePathsFromBoundaries(reference_line_info)
    assert candidates
    reference_line_info.SetCandidatePathData(candidates)

    status = PathAssessmentDecider().Process(None, reference_line_info, None)
    assert status.ok()
    assert reference_line_info.path_data is not None
    assert reference_line_info.path_data.path_label

    RecordDebugInfo(reference_line_info.path_data, "Planning PathData", reference_line_info)
    debug_paths = reference_line_info.debug.planning_data.path
    assert debug_paths
    debug_names = {p.name for p in debug_paths}
    assert "Planning PathData" in debug_names

    boundaries = reference_line_info.GetCandidatePathBoundaries()
    assert len(boundaries) >= 2


def check_infer_lattice_path_label():
    from common.planning_util import InferLatticePathLabel
    from common.curve1d.piecewise_jerk_trajectory1d import PiecewiseJerkTrajectory1d

    lat = PiecewiseJerkTrajectory1d(1.2, 0.0, 0.0)
    lat.AppendSegment(0.0, 10.0)
    assert InferLatticePathLabel(0.0, 50.0, [0.0, 0.0, 0.0], lat) == "regular/left/forward"
    lat2 = PiecewiseJerkTrajectory1d(-1.2, 0.0, 0.0)
    lat2.AppendSegment(0.0, 10.0)
    assert InferLatticePathLabel(0.0, 50.0, [0.0, 0.0, 0.0], lat2) == "regular/right/forward"


def check_relative_map_reference_lines():
    from protoclass.planning_internal import MapMsg, NavigationPath
    from protoclass.path_point import Path, PathPoint as NavPathPoint

    hdmap = HDMap()
    lane = _build_straight_lane()
    lane_info = hdmap.AddLane(lane)
    HDMapUtil.SetBaseMap(hdmap)
    lane_id = lane_info.id.id
    nav_points = [
        NavPathPoint(x=float(i), y=0.0, z=0.0, theta=0.0, kappa=0.0, s=float(i), dkappa=0.0)
        for i in range(20)
    ]
    relative_map = MapMsg(
        navigation_path={
            lane_id: NavigationPath(path=Path(path_point=nav_points), path_priority=0)
        }
    )
    provider = ReferenceLineProvider(relative_map=relative_map)
    provider.UpdateVehicleState(VehicleState(x=5.0, y=0.0, heading=0.0))
    import config as config_module

    old_flag = config_module.FLAGS_use_navigation_mode
    try:
        config_module.FLAGS_use_navigation_mode = True
        reference_lines, route_segments = provider.GetReferenceLinesFromRelativeMap()
        assert len(reference_lines) >= 1
        assert len(route_segments) == len(reference_lines)
        assert reference_lines[0].Length() > 0.0
    finally:
        config_module.FLAGS_use_navigation_mode = old_flag


def check_reference_line_smoothing():
    import config as config_module

    old_flag = config_module.FLAGS_enable_smooth_reference_line
    try:
        config_module.FLAGS_enable_smooth_reference_line = True
        hdmap = HDMap()
        hdmap.AddLane(_build_straight_lane())
        HDMapUtil.SetBaseMap(hdmap)
        provider = ReferenceLineProvider()
        provider.UpdateRoutingResponse(
            RoutingResponse(
                routing_request=RoutingRequest(
                    waypoint=[
                        RoutingLaneWaypoint(id="lane_1", s=0.0),
                        RoutingLaneWaypoint(id="lane_1", s=90.0),
                    ]
                ),
                road=[
                    RoadSegment(
                        id="road_1",
                        passage=[
                            Passage(
                                segment=[
                                    RoutingLaneSegment(
                                        id="lane_1", start_s=0.0, end_s=99.0
                                    )
                                ],
                                can_exit=True,
                                change_lane_type=ChangeLaneType.FORWARD,
                            )
                        ],
                    )
                ],
            )
        )
        provider.UpdateVehicleState(VehicleState(x=10.0, y=0.0, heading=0.0))
        ok, reference_lines, _ = provider.CreateReferenceLine()
        assert ok
        assert reference_lines[0].Length() > 0.0
    finally:
        config_module.FLAGS_enable_smooth_reference_line = old_flag


def check_reference_line_smoothing_shrinks_box_bounds():
    import math
    from reference_line.discrete_points_reference_line_smoother import (
        AnchorPoint,
        DiscretePointsReferenceLineSmoother,
    )

    reference_line, _, _ = _build_reference_line()
    smoother = DiscretePointsReferenceLineSmoother()
    smoother.SetAnchorPoints([
        AnchorPoint(PathPoint(x=0.0, y=0.0), lateral_bound=2.0),
        AnchorPoint(PathPoint(x=10.0, y=0.0), lateral_bound=2.0),
        AnchorPoint(PathPoint(x=20.0, y=0.0), lateral_bound=2.0),
    ])
    captured_bounds = []

    class CapturingSolver:
        def Solve(self, points, bounds):
            captured_bounds.extend(bounds)
            return [point[0] for point in points], [point[1] for point in points]

    smoother._solver = CapturingSolver()
    assert smoother.Smooth(reference_line) is not None
    assert captured_bounds[0] == 0.0
    assert abs(captured_bounds[1] - math.sqrt(2.0)) < 1e-9
    assert captured_bounds[-1] == 0.0


def check_reference_line_anchor_curb_shift():
    hdmap = HDMap()
    lane = _build_straight_lane()
    lane.right_boundary = LaneBoundary(
        boundary_type=[
            LaneBoundaryType(s=0.0, types=[LaneBoundaryType.LaneBoundaryTypeEnum.CURB])
        ]
    )
    lane_info = hdmap.AddLane(lane)
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("curb_lane")
    route_segments.append(LaneSegment(lane_info, 0.0, 50.0))
    reference_line = ReferenceLine(MapPath(route_segments))
    anchor = ReferenceLineProvider().GetAnchorPoint(reference_line, 10.0)
    assert anchor.path_point.s == 10.0
    assert anchor.path_point.y > 0.0
    assert anchor.lateral_bound <= 0.5


def check_reference_point_remove_duplicates_uses_euclidean_distance():
    points = [
        ReferencePoint(MapPathPoint(PointENU(x=0.0, y=0.0, z=0.0), 0.0), 0.0, 0.0),
        ReferencePoint(MapPathPoint(PointENU(x=0.8e-7, y=0.8e-7, z=0.0), 0.0), 0.0, 0.0),
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


def check_path_bounds_sweep_line_updates_center_line_per_edge():
    """
    Regression test: PathBoundsDecider._get_boundary_from_static_obstacles
    used to batch ALL obstacle-edge events that land within the same 0.5m
    path-bound step and only recompute center_line once, after the whole
    batch -- so a second edge in the same step used a stale center_line left
    over from the PREVIOUS point, instead of the fresh one produced by the
    first edge in its own batch (path_bounds_decider.cc calls
    UpdatePathBoundaryAndCenterLineWithBuffer once per edge, not once per
    point). Construct two obstacles whose buffered edges land in the same
    0.5m step: obstacle A (clearly to the right) shifts the centerline
    left when processed first; obstacle B straddles the boundary between
    the stale centerline (0.0) and A's fresh centerline, so which side B
    is assigned to differs between the two centerlines. With the stale
    centerline, B is wrongly classified into a "pass on the right" group
    that conflicts with A's "pass on the left" group and the corridor
    becomes infeasible (l_min > l_max) purely from ordering, not geometry;
    with the fix, both obstacles are classified consistently and produce a
    valid, non-empty corridor.
    """
    import common.path_bounds_decider as path_bounds_decider_module
    from common.path_bounds_decider import PathBoundsDecider

    class _FakeSlBoundary:
        def __init__(self, start_s, end_s, start_l, end_l):
            self.start_s = start_s
            self.end_s = end_s
            self.start_l = start_l
            self.end_l = end_l

    class _FakeObstacle:
        def __init__(self, obstacle_id, sl_boundary):
            self._id = obstacle_id
            self._sl_boundary = sl_boundary

        def Id(self):
            return self._id

        def PerceptionSLBoundary(self):
            return self._sl_boundary

    class _FakePathDecision:
        def __init__(self, obstacles):
            self.obstacles = obstacles

    class _FakeReferenceLineInfo:
        def __init__(self, obstacles):
            self.path_decision = _FakePathDecision(obstacles)

    obstacle_a = _FakeObstacle("A", _FakeSlBoundary(8.2, 50.0, -3.0, -1.0))
    obstacle_b = _FakeObstacle("B", _FakeSlBoundary(8.3, 50.0, 0.5, 1.5))
    reference_line_info = _FakeReferenceLineInfo({"A": obstacle_a, "B": obstacle_b})

    decider = PathBoundsDecider()
    decider.adc_frenet_s = 0.0
    decider.adc_frenet_l = 0.0
    path_bound = [(i * 0.5, -3.5, 3.5) for i in range(21)]

    original_scope_filter = path_bounds_decider_module.IsWithinPathDeciderScopeObstacle
    path_bounds_decider_module.IsWithinPathDeciderScopeObstacle = lambda obstacle: True
    try:
        blocking_id_holder = [""]
        ok = decider._get_boundary_from_static_obstacles(
            reference_line_info, path_bound, blocking_id_holder
        )
    finally:
        path_bounds_decider_module.IsWithinPathDeciderScopeObstacle = original_scope_filter

    assert ok is True
    assert blocking_id_holder[0] == "", "path must not be reported as blocked"
    assert len(path_bound) == 21, "path bound must not be truncated"
    bound_at_5_5 = next(entry for entry in path_bound if abs(entry[0] - 5.5) < 1e-9)
    assert bound_at_5_5[1] <= bound_at_5_5[2], "corridor at s=5.5 must remain feasible (l_min <= l_max)"


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
    check_hdmap_basic_queries()
    check_hdmap_get_nearest_lane_uses_clamped_distance()
    check_hdmap_get_roads_uses_nearby_lane_membership()
    check_map_path_route_segment_regressions()
    check_pnc_map_multi_reference_lines()
    check_reference_line_provider_with_routing()
    check_reference_line_provider_reuses_unchanged_reference_line()
    check_traffic_decider_stop_point()
    check_yield_sign_rule_stop_point()
    check_keep_clear_rule_obstacle()
    check_destination_rule_respects_passed_destination()
    check_path_decider_static_nudge()
    check_build_frenet_path_from_lat_trajectory()
    check_path_decider_after_lateral_trajectory()
    check_hdmap_load_from_file_if_available()
    check_reference_line_smoothing()
    check_reference_line_smoothing_shrinks_box_bounds()
    check_reference_line_anchor_curb_shift()
    check_reference_point_remove_duplicates_uses_euclidean_distance()
    check_reference_line_get_sl_boundary_failure_is_false()
    check_reference_line_info_copies_reference_line()
    check_qp_spline_reference_line_smoothing()
    check_qp_spline_solver_basic()
    check_obstacle_decision_property_api()
    check_obstacle_copies_trajectory()
    check_obstacle_empty_trajectory_is_static_for_st_graph()
    check_path_time_graph_keeps_ignored_static_obstacle()
    check_reference_line_provider_history_fallback()
    check_trajectory_stitcher_reinit()
    check_trajectory_stitcher_preserves_previous_trajectory()
    check_lattice_path_assessment_blocking()
    check_path_assessment_compare_paths()
    check_combine_path_and_speed_profile()
    check_path_assessment_keeps_cpp_default_obstacle_distance()
    check_polygon_box_distance()
    check_polygon_overlap_contains_line_segment()
    check_polygon_bounding_box_with_heading_uses_cross_projection()
    check_polygon_extreme_points_accepts_triangle()
    check_route_segments_uses_cpp_segmentation_epsilon()
    check_box_polygon_overlap()
    check_box_distance_to_segment_canonical_state()
    check_path_bounds_decider_multi_candidates()
    check_path_bounds_decider_respects_committed_borrow_direction()
    check_path_bounds_static_obstacle_tightens_boundary()
    check_record_debug_info()
    check_lattice_migration_pipeline()
    check_infer_lattice_path_label()
    check_on_lane_planning_output()
    check_relative_map_reference_lines()
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
    check_path_bounds_sweep_line_updates_center_line_per_edge()
    print("lattice component checks succeeded")


if __name__ == "__main__":
    main()
