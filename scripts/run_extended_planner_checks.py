#!/usr/bin/env python3
"""
Extended smoke / regression checks for the Python Lattice Planner port.

Complements scripts/run_lattice_component_checks.py with coverage for:
  - Frenet derivative helpers and PathData XY fallback
  - Path assessment validity / collision / compare heuristics
  - Path bounds + planning context lane borrow
  - Obstacle scope filtering for path decider
  - Lattice plan under blocking obstacle (backup on/off)
  - Legacy flag mode (migration toggles off)

Run:
  .venv/bin/python scripts/run_extended_planner_checks.py
"""

from __future__ import annotations

import importlib
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from cartesian_frenet_converter import CartesianFrenetConverter
from lattice_planner import LatticePlanner
from common.frame import Frame
from common.path_boundary import PathBoundPoint, PathBoundary
from common.planning_context import PlanningContext
from common.path_decider_obstacle_utils import IsWithinPathDeciderScopeObstacle
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.frenet_frame_point import FrenetFramePoint
from protoclass.path_point import PathPoint
from scripts.planner_test_fixtures import (
    build_lattice_plan_frame,
    build_left_lane_reference_line,
    build_reference_line,
    build_static_obstacle,
)


def check_cartesian_frenet_lateral_derivative_roundtrip():
    rtheta, rkappa = 0.0, 0.0
    l, dl = 1.0, 0.2
    theta = CartesianFrenetConverter.CalculateTheta(rtheta, rkappa, l, dl)
    dl_back = CartesianFrenetConverter.CalculateLateralDerivative(rtheta, theta, l, rkappa)
    assert abs(dl - dl_back) < 1e-6

    ddl = 0.05
    kappa = CartesianFrenetConverter.CalculateKappa(rkappa, 0.0, l, dl, ddl)
    ddl_back = CartesianFrenetConverter.CalculateSecondOrderLateralDerivative(
        rtheta, theta, rkappa, kappa, 0.0, l
    )
    assert abs(ddl - ddl_back) < 0.05


def check_reference_line_get_frenet_point():
    reference_line, _, _ = build_reference_line()
    path_point = PathPoint(x=10.0, y=0.5, z=0.0, theta=0.05, kappa=0.0, s=10.0)
    frenet = reference_line.GetFrenetPoint(path_point)
    assert frenet is not None
    assert abs(frenet.s - 10.0) < 0.5
    assert abs(frenet.l - 0.5) < 0.2
    assert frenet.dl is not None


def check_path_decider_obstacle_scope_filter():
    from common.obstacle import Obstacle
    from protoclass.adc_trajectory import Point3D
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    static = build_static_obstacle("static_scope", 25.0)
    assert IsWithinPathDeciderScopeObstacle(static)

    moving_perception = PerceptionObstacle(
        id=2,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=25.0, y=0.0, z=0.0),
        velocity=Point3D(x=2.0, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    moving = Obstacle("dyn_scope", moving_perception, is_static=False)
    assert not IsWithinPathDeciderScopeObstacle(moving)

    virtual = Obstacle(
        "virtual_scope",
        PerceptionObstacle(
            id=-1,
            position=Point3D(x=5.0, y=0.0, z=0.0),
            velocity=Point3D(x=0.0, y=0.0, z=0.0),
            length=0.1,
            width=0.1,
            height=0.1,
            theta=0.0,
        ),
        is_static=True,
    )
    assert virtual.IsVirtual()
    assert not IsWithinPathDeciderScopeObstacle(virtual)


def check_assign_frenet_path_large_lateral_has_xy():
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    path_data = BuildLatticeCandidatePath(
        reference_line_info,
        0.0,
        [2.5, 0.0, 0.0],
        60.0,
        path_label="regular/left/forward",
    )
    assert len(path_data.frenet_frame_path) > 0
    assert len(path_data.discretized_path) > 0
    assert path_data.discretized_path[0].y > 0.5


def check_path_assessment_off_reference_invalid():
    from common.path_assessment_decider import IsValidRegularPath
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    path_data = BuildLatticeCandidatePath(
        reference_line_info,
        0.0,
        [25.0, 0.0, 0.0],
        40.0,
        path_label="regular/self",
    )
    assert not IsValidRegularPath(reference_line_info, path_data)


def check_path_assessment_collision_detection():
    from common.path_assessment_decider import IsCollidingWithStaticObstacles
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.AddObstacle(
        build_static_obstacle("col_1", 15.0, y=0.0, width=3.0)
    )
    on_obstacle = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 40.0, path_label="regular/self"
    )
    assert IsCollidingWithStaticObstacles(reference_line_info, on_obstacle)


def check_path_assessment_projection_failure_is_unsafe():
    from common.path_assessment_decider import IsCollidingWithStaticObstacles
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.AddObstacle(build_static_obstacle("projection_guard", 15.0))
    path_data = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 40.0, path_label="regular/self"
    )
    reference_line_info.reference_line.XYToSL = lambda *args: (False, None)
    assert IsCollidingWithStaticObstacles(reference_line_info, path_data)


def check_compare_path_data_prefers_longer_self_lane():
    from common.path_assessment_decider import ComparePathData
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    short_path = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 25.0, path_label="regular/self"
    )
    long_path = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 70.0, path_label="regular/self"
    )
    assert ComparePathData(long_path, short_path, None)
    assert not ComparePathData(short_path, long_path, None)


def check_path_boundary_boundary_api():
    boundary = PathBoundary([], 0.5)
    boundary.append(PathBoundPoint(-1.5, 1.5, 0.0))
    boundary.append(PathBoundPoint(-1.0, 2.0, 0.5))
    boundary.set_label("regular/self/forward")
    tuples = boundary.boundary()
    assert len(tuples) == 2
    assert tuples[0] == (-1.5, 1.5)
    assert tuples[1] == (-1.0, 2.0)


def check_path_bounds_lane_borrow_from_context():
    from common.path_bounds_decider import PathBoundsDecider, LaneBorrowInfo

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.AddObstacle(build_static_obstacle("borrow_block", 35.0))
    reference_line_info.SetBlockingObstacle("borrow_block")

    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ctx.planning_status.path_decider.decided_side_pass_direction = [1, 2]
    reference_line_info.set_is_path_lane_borrow(True)

    status = PathBoundsDecider().Process(None, reference_line_info, ctx)
    assert status.ok()
    labels = {b.label for b in reference_line_info.GetCandidatePathBoundaries()}
    assert "fallback" in labels
    assert any("left" in label for label in labels)
    assert any("right" in label for label in labels)

    decider = PathBoundsDecider()
    infos = decider._lane_borrow_infos(reference_line_info, ctx)
    assert LaneBorrowInfo.LEFT_BORROW in infos
    assert LaneBorrowInfo.RIGHT_BORROW in infos


def check_path_bounds_requires_decided_borrow_direction():
    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.set_is_path_lane_borrow(True)
    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True

    from common.path_bounds_decider import PathBoundsDecider

    status = PathBoundsDecider().Process(None, reference_line_info, ctx)
    assert status.ok()
    labels = {b.label for b in reference_line_info.GetCandidatePathBoundaries()}
    assert not any("left" in label or "right" in label for label in labels)


def check_path_bounds_uses_real_neighbor_lane_width():
    _, reference_line_info = build_left_lane_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.set_is_path_lane_borrow(True)
    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ctx.planning_status.path_decider.decided_side_pass_direction = [2]

    from common.path_bounds_decider import PathBoundsDecider

    status = PathBoundsDecider().Process(None, reference_line_info, ctx)
    assert status.ok()
    boundaries = {
        boundary.label: boundary for boundary in reference_line_info.GetCandidatePathBoundaries()
    }
    self_boundary = next(value for key, value in boundaries.items() if "self" in key)
    right_boundary = next(value for key, value in boundaries.items() if "right" in key)
    assert right_boundary[0].l_lower.l < self_boundary[0].l_lower.l - 2.0


def check_path_lane_borrow_state_machine():
    from common.path_lane_borrow_decider import PathLaneBorrowDecider

    obstacle = build_static_obstacle("long_term_block", 25.0)
    frame, reference_line_info, start = build_lattice_plan_frame([obstacle], init_v=2.0)
    frame._planning_start_point = start
    reference_line_info.SetBlockingObstacle(obstacle.Id())
    context = PlanningContext()
    status = context.planning_status.path_decider
    status.front_static_obstacle_id = obstacle.Id()
    status.front_static_obstacle_cycle_counter = 3

    result = PathLaneBorrowDecider().Process(frame, reference_line_info, context)
    assert result.ok()
    assert reference_line_info.is_path_lane_borrow()
    assert status.is_in_path_lane_borrow_scenario
    assert status.decided_side_pass_direction == [1, 2]

    status.able_to_use_self_lane_counter = 6
    PathLaneBorrowDecider().Process(frame, reference_line_info, context)
    assert not reference_line_info.is_path_lane_borrow()
    assert not status.is_in_path_lane_borrow_scenario
    assert status.decided_side_pass_direction == []


def check_static_obstacle_sweep_keeps_pass_direction():
    from common.path_bounds_decider import PathBoundsDecider

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.AddObstacle(
        build_static_obstacle("sweep_right", 30.0, y=-1.2, width=0.5)
    )
    status = PathBoundsDecider().Process(None, reference_line_info, PlanningContext())
    assert status.ok()
    self_boundary = next(
        boundary
        for boundary in reference_line_info.GetCandidatePathBoundaries()
        if "regular/self" in boundary.label
    )
    affected = [point for point in self_boundary if 26.0 <= point.s <= 33.0]
    assert affected
    assert all(point.l_lower.l > 0.4 for point in affected)


def check_path_assessment_cpp_tail_and_counter_semantics():
    from common.path_assessment_decider import (
        K_NUM_EXTRA_TAIL_BOUND_POINT,
        _update_path_decider_status,
    )
    from common.planning_util import BuildLatticeCandidatePath

    assert K_NUM_EXTRA_TAIL_BOUND_POINT == 20

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.SetPathData(
        BuildLatticeCandidatePath(
            reference_line_info,
            0.0,
            [0.0, 0.0, 0.0],
            30.0,
            path_label="regular/self",
        )
    )
    context = PlanningContext()
    status = context.planning_status.path_decider

    status.front_static_obstacle_cycle_counter = 5
    _update_path_decider_status(reference_line_info, context)
    assert status.front_static_obstacle_cycle_counter == 4

    obstacle = build_static_obstacle("counter_block", 20.0)
    reference_line_info.AddObstacle(obstacle)
    reference_line_info.SetBlockingObstacle(obstacle.Id())
    status.front_static_obstacle_cycle_counter = -5
    _update_path_decider_status(reference_line_info, context)
    assert status.front_static_obstacle_cycle_counter == -4


def check_lattice_plan_blocking_with_backup():
    import config as config_module

    obs = build_static_obstacle("block_backup", 35.0)
    _, _, reference_line_info = build_reference_line()
    frame = Frame(0)
    frame._obstacles = {obs.Id(): obs}
    reference_line_info.Init([obs], 10.0)
    reference_line_info.SetBlockingObstacle("block_backup")
    frame._reference_line_info = [reference_line_info]

    old_backup = config_module.FLAGS_enable_backup_trajectory
    try:
        config_module.FLAGS_enable_backup_trajectory = True
        ok = LatticePlanner().Plan(
            reference_line_info._adc_planning_point, frame, ADCTrajectory()
        )
        assert ok
        assert reference_line_info.trajectory is not None
        assert len(reference_line_info.trajectory) > 0
    finally:
        config_module.FLAGS_enable_backup_trajectory = old_backup


def check_lattice_default_plan():
    """Default config matches pure lattice_planner.cc (no decider migration flags)."""
    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    frame = Frame(0)
    frame._reference_line_info = [reference_line_info]
    frame._obstacles = {}

    ok = LatticePlanner().Plan(
        reference_line_info._adc_planning_point, frame, ADCTrajectory()
    )
    assert ok
    assert reference_line_info.trajectory is not None
    assert len(reference_line_info.trajectory) > 0


def check_path_decider_with_path_assessment_pipeline():
    import config as config_module
    from common.path_decider import PathDecider
    from common.path_assessment_decider import ApplyLatticePathAssessment
    from common.planning_util import SetupNominalPathData

    _, _, reference_line_info = build_reference_line()
    reference_line_info._vehicle_state.linear_velocity = 3.0
    reference_line_info.Init([], 10.0)
    reference_line_info.AddObstacle(build_static_obstacle("pd_1", 22.0, y=0.4))

    old_pd = config_module.FLAGS_enable_path_decider
    try:
        config_module.FLAGS_enable_path_decider = True
        SetupNominalPathData(reference_line_info, 0.0, [0.0, 0.0, 0.0], 80.0)
        ApplyLatticePathAssessment(reference_line_info)
        status = PathDecider(reference_line_info).Execute(reference_line_info)
        assert status.ok()
        obs = reference_line_info.path_decision.Find("pd_1")
        assert obs is not None
        assert obs.HasLateralDecision() or obs.HasLongitudinalDecision()
    finally:
        config_module.FLAGS_enable_path_decider = old_pd


def check_set_path_info_skips_empty_discretized():
    from common.path_assessment_decider import SetPathInfo
    from common.path_data import PathData
    from common.frenet_frame_path import FrenetFramePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    empty_path = PathData()
    empty_path.SetReferenceLine(reference_line_info.reference_line)
    empty_path.set_path_label("regular/self")
    empty_path._frenet_path = FrenetFramePath(
        [FrenetFramePoint(s=0.0, l=0.0, dl=0.0, ddl=0.0)]
    )
    SetPathInfo(reference_line_info, empty_path)
    assert not empty_path.path_point_decision_guide


def check_line_segment_distance_scalar_for_polygon():
    from common.line_segment2d import LineSegment2d
    from common.vec2d import Vec2d

    seg = LineSegment2d(Vec2d(0.0, 0.0), Vec2d(10.0, 0.0))
    dist, nearest = seg.DistanceTo(Vec2d(5.0, 3.0))
    assert isinstance(dist, float)
    assert dist == 3.0
    assert abs(nearest.x - 5.0) < 1e-6


def check_frame_has_planning_context():
    frame = Frame(1)
    assert frame.planning_context is not None
    assert frame.planning_context.planning_status.path_decider is not None


def check_stopped_ego_collision_uses_vehicle_box():
    from common.frame import EgoInfo
    from protoclass.vehicle_state import VehicleState

    frame = Frame(1)
    obstacle = build_static_obstacle("ego_overlap", 0.0)
    frame._obstacles = {obstacle.Id(): obstacle}
    ego_info = EgoInfo.FromVehicleState(
        VehicleState(x=0.0, y=0.0, heading=0.0)
    )
    assert ego_info.ego_box is not None
    assert frame.FindCollisionObstacle(ego_info) is obstacle


def check_vehicle_state_timestamp_falls_back_to_chassis():
    from common.vehicle_state_provider import VehicleStateProvider
    from on_lane_planning import OnLanePlanning
    from protoclass.adc_trajectory import Point3D
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.point_enu import PointENU
    from protoclass.pose import Pose

    localization = LocalizationEstimate(
        pose=Pose(
            position=PointENU(x=1.0, y=2.0, z=0.0),
            heading=0.0,
            angular_velocity=Point3D(x=0.0, y=0.0, z=0.0),
            linear_acceleration=Point3D(x=0.0, y=0.0, z=0.0),
            euler_angles=Point3D(x=0.0, y=0.0, z=0.0),
        )
    )
    localization = OnLanePlanning._normalize_localization(localization)
    provider = VehicleStateProvider()
    status = provider.Update(
        localization,
        Chassis(speed_mps=0.0, header=Header(timestamp_sec=123.0)),
    )
    assert status.ok()
    assert provider.vehicle_state.timestamp == 123.0


def check_traffic_light_without_header_is_safe():
    from common.frame import LocalView
    from protoclass.traffic_light_detection import TrafficLightDetection
    from protoclass.trajectory_point import TrajectoryPoint
    from protoclass.vehicle_state import VehicleState

    frame = Frame(
        1,
        LocalView(traffic_light=TrafficLightDetection()),
        TrajectoryPoint(),
        VehicleState(),
    )
    frame.ReadTrafficLights()
    assert frame._traffic_lights == {}


def check_vehicle_state_nan_is_invalid():
    from common.frame import util
    from protoclass.vehicle_state import VehicleState

    assert util.IsVehicleStateValid(VehicleState())
    assert not util.IsVehicleStateValid(VehicleState(heading=float("nan")))


def check_resolve_vehicle_state_rejects_nan_chassis_speed():
    """
    Regression test: OnLanePlanning._resolve_vehicle_state used to mask a
    NaN/None chassis.speed_mps to 0.0 before calling
    VehicleStateProvider.Update, silently treating corrupted sensor data as
    "vehicle stationary" and letting planning proceed. C++'s RunOnce has no
    such sanitization: a NaN speed flows straight into vehicle_state_ and is
    caught by !util::IsVehicleStateValid(vehicle_state) right after Update(),
    which aborts to a stop trajectory instead of planning on unreliable data.
    The Python port must reject it the same way, before any routing-change
    side effects (which clear PlanningContext / rebuild the planner) can fire.
    """
    from common.frame import LocalView
    from on_lane_planning import OnLanePlanning
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.point_enu import PointENU
    from protoclass.pose import Pose

    planner = OnLanePlanning()
    local_view = LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(position=PointENU(x=0.0, y=0.0, z=0.0), heading=0.0),
            measurement_time=0.0,
        ),
        chassis=Chassis(speed_mps=float("nan"), header=Header(timestamp_sec=0.0)),
    )
    vehicle_state, status = planner._resolve_vehicle_state(local_view)
    assert vehicle_state is None
    assert not status.ok()


def check_routing_sequence_change_semantics():
    from on_lane_planning import OnLanePlanning
    from protoclass.header import Header
    from protoclass.routing import RoutingResponse

    first = RoutingResponse(header=Header(sequence_num=7))
    same = RoutingResponse(header=Header(sequence_num=7))
    changed = RoutingResponse(header=Header(sequence_num=8))
    assert not OnLanePlanning._is_different_routing(first, same)
    assert OnLanePlanning._is_different_routing(first, changed)
    assert OnLanePlanning._is_different_routing(None, first)


def check_routing_change_clears_planning_context():
    from on_lane_planning import OnLanePlanning
    from protoclass.header import Header
    from protoclass.routing import RoutingResponse

    planner = OnLanePlanning()
    context = PlanningContext()
    first = RoutingResponse(header=Header(sequence_num=7))
    changed = RoutingResponse(header=Header(sequence_num=8))

    context.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    assert planner._update_routing(first, context)
    assert not context.planning_status.path_decider.is_in_path_lane_borrow_scenario

    context.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    assert not planner._update_routing(first, context)
    assert context.planning_status.path_decider.is_in_path_lane_borrow_scenario

    assert planner._update_routing(changed, context)
    assert not context.planning_status.path_decider.is_in_path_lane_borrow_scenario


def check_failed_reference_line_update_stops_planning():
    from common.frame import LocalView
    from on_lane_planning import OnLanePlanning
    from protoclass.adc_trajectory import ADCTrajectory
    from protoclass.chassis import Chassis
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.point_enu import PointENU
    from protoclass.pose import Pose
    from reference_line.reference_line_provider import ReferenceLineProvider

    provider = ReferenceLineProvider()
    provider._is_reference_line_updated = False
    planner = OnLanePlanning(provider)
    local_view = LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(position=PointENU(x=0.0, y=0.0, z=0.0), heading=0.0),
            measurement_time=0.0,
        ),
        chassis=Chassis(speed_mps=0.0),
    )
    output = ADCTrajectory()
    status = planner.RunOnce(local_view, output)
    assert not status.ok()
    assert "reference line" in status.error_message
    assert output.trajectory_point


def check_build_cruise_speed_data():
    from common.planning_util import BuildCruiseSpeedData

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    reference_line_info.SetLatticeCruiseSpeed(5.0)
    speed = BuildCruiseSpeedData(reference_line_info)
    assert len(speed) >= 2
    assert all(pt.v == 5.0 for pt in speed)
    assert speed[-1].s > speed[0].s


def check_boundary_only_path_assessment_combine():
    import config as config_module
    from common.path_bounds_decider import PathBoundsDecider, BuildCandidatePathsFromBoundaries
    from common.path_assessment_decider import PathAssessmentDecider

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)

    PathBoundsDecider().Process(None, reference_line_info, None)
    candidates = BuildCandidatePathsFromBoundaries(reference_line_info)
    assert candidates
    reference_line_info.SetCandidatePathData(candidates)
    status = PathAssessmentDecider().Process(None, reference_line_info, None)
    assert status.ok()
    assert reference_line_info.path_data is not None
    assert reference_line_info.path_data.lattice_candidate_id() <= -1000

    old_backup = config_module.FLAGS_enable_backup_trajectory
    try:
        config_module.FLAGS_enable_backup_trajectory = False

        from common.planning_util import BuildCruiseSpeedData
        from common.discretized_trajectory import DiscretizedTrajectory

        reference_line_info.SetSpeedData(BuildCruiseSpeedData(reference_line_info))
        trajectory = DiscretizedTrajectory()
        assert reference_line_info.CombinePathAndSpeedProfile(
            0.0, reference_line_info._adc_planning_point.path_point.s, trajectory
        )
        assert len(trajectory) > 0
    finally:
        config_module.FLAGS_enable_backup_trajectory = old_backup


def check_path_decider_after_lateral_pipeline():
    import config as config_module
    from common.planning_util import BuildLatticeCandidatePath

    _, _, reference_line_info = build_reference_line()
    reference_line_info._vehicle_state.linear_velocity = 2.0
    reference_line_info.Init([], 10.0)
    reference_line_info.AddObstacle(build_static_obstacle("lat_pd", 50.0, y=2.5))

    old_pd = config_module.FLAGS_enable_path_decider
    old_after = config_module.FLAGS_enable_path_decider_after_lateral
    try:
        config_module.FLAGS_enable_path_decider = True
        config_module.FLAGS_enable_path_decider_after_lateral = True

        path_data = BuildLatticeCandidatePath(
            reference_line_info, 0.0, [0.0, 0.0, 0.0], 70.0, path_label="regular/self"
        )
        path_data.set_lattice_candidate_id(0)
        reference_line_info.SetCandidatePathData([path_data])
        from common.path_assessment_decider import PathAssessmentDecider

        assert PathAssessmentDecider().Process(None, reference_line_info, None).ok()
        assert reference_line_info.path_data is not None

        from common.path_decider import PathDecider

        status = PathDecider(reference_line_info).Execute(reference_line_info)
        assert status.ok()
        obs = reference_line_info.path_decision.Find("lat_pd")
        assert obs is not None
    finally:
        config_module.FLAGS_enable_path_decider = old_pd
        config_module.FLAGS_enable_path_decider_after_lateral = old_after


def check_path_bounds_uses_planning_start_frenet():
    from common.frame import Frame
    from common.path_bounds_decider import PathBoundsDecider
    from protoclass.path_point import PathPoint
    from protoclass.trajectory_point import TrajectoryPoint

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    start = TrajectoryPoint(
        path_point=PathPoint(x=5.0, y=0.3, z=0.0, theta=0.0, kappa=0.0, s=5.0),
        v=1.0,
        a=0.0,
        relative_time=0.0,
    )
    frame = Frame(0, None, start, reference_line_info._vehicle_state)
    decider = PathBoundsDecider()
    decider._init_state(frame, reference_line_info)
    assert abs(decider.adc_frenet_s - 5.0) < 0.5
    assert abs(decider.adc_frenet_l - 0.3) < 0.3


def check_lattice_fails_when_blocked_without_backup():
    """Pure lattice_planner.cc: blocked path with no backup -> plan fails."""
    import config as config_module

    obs = build_static_obstacle("block_no_bk", 8.0)
    _, _, reference_line_info = build_reference_line()
    frame = Frame(0)
    frame._obstacles = {obs.Id(): obs}
    reference_line_info.Init([obs], 10.0)
    reference_line_info.SetBlockingObstacle("block_no_bk")
    frame._reference_line_info = [reference_line_info]

    old_backup = config_module.FLAGS_enable_backup_trajectory
    try:
        config_module.FLAGS_enable_backup_trajectory = False
        ok = LatticePlanner().Plan(
            reference_line_info._adc_planning_point, frame, ADCTrajectory()
        )
        assert not ok
    finally:
        config_module.FLAGS_enable_backup_trajectory = old_backup


def check_lattice_accepts_empty_backup_trajectory_like_cpp():
    import config as config_module
    from common.discretized_trajectory import DiscretizedTrajectory
    from unittest.mock import patch

    obs = build_static_obstacle("block_empty_backup", 8.0)
    frame, reference_line_info, start_point = build_lattice_plan_frame(
        [obs], blocking_obstacle_id="block_empty_backup"
    )
    old_backup = config_module.FLAGS_enable_backup_trajectory
    try:
        config_module.FLAGS_enable_backup_trajectory = True
        with patch(
            "lattice_planner.BackupTrajectoryGenerator.GenerateTrajectory",
            return_value=DiscretizedTrajectory(),
        ):
            ok = LatticePlanner().Plan(start_point, frame, ADCTrajectory())
        assert ok
        assert reference_line_info.IsDrivable()
        assert len(reference_line_info.trajectory) == 0
    finally:
        config_module.FLAGS_enable_backup_trajectory = old_backup


def check_on_lane_aggregate_path_speed():
    import config as config_module
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.planning_util import (
        AggregateReferenceLineTrajectory,
        BuildCruiseSpeedData,
        BuildLatticeCandidatePath,
    )

    _, _, reference_line_info = build_reference_line()
    reference_line_info.Init([], 10.0)
    path_data = BuildLatticeCandidatePath(
        reference_line_info, 0.0, [0.0, 0.0, 0.0], 60.0, path_label="regular/self"
    )
    reference_line_info.SetPathData(path_data)
    reference_line_info.SetLatticeCruiseSpeed(4.0)
    reference_line_info.SetSpeedData(BuildCruiseSpeedData(reference_line_info))
    reference_line_info.SetTrajectory(DiscretizedTrajectory())

    old_flag = config_module.FLAGS_enable_on_lane_combine_path_and_speed
    try:
        config_module.FLAGS_enable_on_lane_combine_path_and_speed = True
        start = reference_line_info._adc_planning_point
        assert AggregateReferenceLineTrajectory(reference_line_info, start)
        assert len(reference_line_info.trajectory) > 0
    finally:
        config_module.FLAGS_enable_on_lane_combine_path_and_speed = old_flag


def check_module_import_surface():
    modules = [
        "on_lane_planning",
        "common.vehicle_state_provider",
        "common.trajectory_stitcher",
        "common.constraint_checker",
        "common.discretized_trajectory",
        "trajectory_generation.trajectory_combiner",
        "trajectory_generation.trajectory_evaluator",
    ]
    for name in modules:
        importlib.import_module(name)


# Areas intentionally not covered here (need bag/sim or heavy deps):
# - Full multi-reference-line OnLanePlanning with routing refresh
# - OSQP failure / infeasible lateral QP edge cases
# - HD map binaries without local map files
# - Pullover / open-space scenario stack

CHECKS = [
    ("cartesian_frenet_lateral_derivative_roundtrip", check_cartesian_frenet_lateral_derivative_roundtrip),
    ("reference_line_get_frenet_point", check_reference_line_get_frenet_point),
    ("path_decider_obstacle_scope_filter", check_path_decider_obstacle_scope_filter),
    ("assign_frenet_path_large_lateral_has_xy", check_assign_frenet_path_large_lateral_has_xy),
    ("path_assessment_off_reference_invalid", check_path_assessment_off_reference_invalid),
    ("path_assessment_collision_detection", check_path_assessment_collision_detection),
    ("path_assessment_projection_failure_is_unsafe", check_path_assessment_projection_failure_is_unsafe),
    ("compare_path_data_prefers_longer_self_lane", check_compare_path_data_prefers_longer_self_lane),
    ("path_boundary_boundary_api", check_path_boundary_boundary_api),
    ("path_bounds_lane_borrow_from_context", check_path_bounds_lane_borrow_from_context),
    ("path_bounds_requires_decided_borrow_direction", check_path_bounds_requires_decided_borrow_direction),
    ("path_bounds_uses_real_neighbor_lane_width", check_path_bounds_uses_real_neighbor_lane_width),
    ("path_lane_borrow_state_machine", check_path_lane_borrow_state_machine),
    ("static_obstacle_sweep_keeps_pass_direction", check_static_obstacle_sweep_keeps_pass_direction),
    ("path_assessment_cpp_tail_and_counter_semantics", check_path_assessment_cpp_tail_and_counter_semantics),
    ("lattice_plan_blocking_with_backup", check_lattice_plan_blocking_with_backup),
    ("lattice_default_plan", check_lattice_default_plan),
    ("path_decider_with_path_assessment_pipeline", check_path_decider_with_path_assessment_pipeline),
    ("set_path_info_skips_empty_discretized", check_set_path_info_skips_empty_discretized),
    ("line_segment_distance_scalar_for_polygon", check_line_segment_distance_scalar_for_polygon),
    ("frame_has_planning_context", check_frame_has_planning_context),
    ("stopped_ego_collision_uses_vehicle_box", check_stopped_ego_collision_uses_vehicle_box),
    ("vehicle_state_timestamp_falls_back_to_chassis", check_vehicle_state_timestamp_falls_back_to_chassis),
    ("traffic_light_without_header_is_safe", check_traffic_light_without_header_is_safe),
    ("vehicle_state_nan_is_invalid", check_vehicle_state_nan_is_invalid),
    ("resolve_vehicle_state_rejects_nan_chassis_speed", check_resolve_vehicle_state_rejects_nan_chassis_speed),
    ("routing_sequence_change_semantics", check_routing_sequence_change_semantics),
    ("routing_change_clears_planning_context", check_routing_change_clears_planning_context),
    ("failed_reference_line_update_stops_planning", check_failed_reference_line_update_stops_planning),
    ("build_cruise_speed_data", check_build_cruise_speed_data),
    ("boundary_only_path_assessment_combine", check_boundary_only_path_assessment_combine),
    ("path_decider_after_lateral_pipeline", check_path_decider_after_lateral_pipeline),
    ("path_bounds_uses_planning_start_frenet", check_path_bounds_uses_planning_start_frenet),
    ("lattice_fails_when_blocked_without_backup", check_lattice_fails_when_blocked_without_backup),
    ("lattice_accepts_empty_backup_trajectory_like_cpp", check_lattice_accepts_empty_backup_trajectory_like_cpp),
    ("on_lane_aggregate_path_speed", check_on_lane_aggregate_path_speed),
    ("module_import_surface", check_module_import_surface),
]


def main() -> int:
    failed = []
    for name, fn in CHECKS:
        try:
            fn()
            print(f"  OK  {name}")
        except Exception as exc:
            print(f"  FAIL {name}: {exc}", file=sys.stderr)
            failed.append(name)

    if failed:
        print(f"\n{len(failed)} extended check(s) failed: {', '.join(failed)}", file=sys.stderr)
        return 1

    print(f"\nextended planner checks succeeded ({len(CHECKS)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
