"""
Lattice / OnLane 场景定义（供 run_lattice_scenario_cases.py 与 demo 复用）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import config
from lattice_planner import LatticePlanner
from protoclass.adc_trajectory import ADCTrajectory
from scripts.planner_test_fixtures import (
    apply_borrow_path_trajectory,
    build_curved_lattice_plan_frame,
    build_curved_static_obstacle,
    build_lattice_plan_frame,
    build_overtake_lattice_frame,
    build_path_bounds_overtake_frame,
    build_synthetic_overtake_frame,
    build_lattice_overtake_combined_frame,
    build_slow_leader_obstacle,
    build_static_obstacle,
)

# builder 返回: frame, rli, start, expect_ok, backup_flag, extra
BuilderResult = Tuple


@dataclass
class Scenario:
    name: str
    description: str
    category: str  # lattice | decider | on_lane | stress | overtake
    builder: Callable[[], BuilderResult]
    expect_ok: bool = True
    backup: Optional[bool] = None
    informational: bool = False
    skip_lattice: bool = False  # 已预置 trajectory（借道超车等）


def _lattice(
    obstacles=None,
    *,
    expect_ok: bool = True,
    backup: Optional[bool] = None,
    length: float = 100.0,
    init_v: float = 1.0,
    start_x: float = 0.0,
    start_y: float = 0.0,
    blocking_obstacle_id: str | None = None,
    extra: str | None = None,
) -> BuilderResult:
    frame, rli, start = build_lattice_plan_frame(
        obstacles,
        length=length,
        init_v=init_v,
        start_x=start_x,
        start_y=start_y,
        blocking_obstacle_id=blocking_obstacle_id,
    )
    return frame, rli, start, expect_ok, backup, extra


# --- lattice 核心 ---


def s_open_road() -> BuilderResult:
    return _lattice()


def s_far_obstacle_stop() -> BuilderResult:
    obs = build_static_obstacle("far_car", 35.0)
    return _lattice([obs], blocking_obstacle_id="far_car")


def s_close_obstacle_no_backup() -> BuilderResult:
    obs = build_static_obstacle("close_car", 8.0)
    return _lattice([obs], expect_ok=False, backup=False, blocking_obstacle_id="close_car")


def s_close_obstacle_with_backup() -> BuilderResult:
    obs = build_static_obstacle("close_car", 8.0)
    return _lattice([obs], backup=True, blocking_obstacle_id="close_car")


def s_higher_speed_cruise() -> BuilderResult:
    return _lattice(init_v=5.0)


def s_stopped_start() -> BuilderResult:
    return _lattice(init_v=0.1)


def s_mid_lane_start() -> BuilderResult:
    return _lattice(init_v=3.0, start_x=20.0)


def s_lateral_offset_start() -> BuilderResult:
    return _lattice(init_v=2.0, start_y=0.6)


def s_two_obstacles_queue() -> BuilderResult:
    obs1 = build_static_obstacle("q1", 22.0)
    obs2 = build_static_obstacle("q2", 45.0)
    return _lattice([obs1, obs2], blocking_obstacle_id="q1")


def s_obstacle_no_blocking_flag() -> BuilderResult:
    obs = build_static_obstacle("silent_car", 30.0)
    return _lattice([obs])


def s_side_obstacle_adjacent_lane() -> BuilderResult:
    obs = build_static_obstacle("side_car", 28.0, y=2.8)
    return _lattice([obs])


def s_medium_obstacle_18m() -> BuilderResult:
    obs = build_static_obstacle("med_car", 18.0)
    return _lattice([obs], blocking_obstacle_id="med_car")


def s_long_road_150m() -> BuilderResult:
    return _lattice(length=150.0, init_v=2.0)


def s_low_speed_creep() -> BuilderResult:
    return _lattice(init_v=0.3)


def s_obstacle_60m_pass() -> BuilderResult:
    obs = build_static_obstacle("far_ahead", 60.0)
    return _lattice([obs])


def s_backup_off_open_road() -> BuilderResult:
    return _lattice(backup=False)


def s_curved_open_road() -> BuilderResult:
    frame, rli, start = build_curved_lattice_plan_frame(init_v=2.0)
    return frame, rli, start, True, None, "left_arc radius=60m"


def s_curved_obstacle_stop() -> BuilderResult:
    obs = build_curved_static_obstacle("curve_car", 35.0)
    frame, rli, start = build_curved_lattice_plan_frame(
        [obs],
        init_v=2.0,
        blocking_obstacle_id="curve_car",
    )
    return frame, rli, start, True, None, "left_arc obstacle_s=35m"


# --- decider 层 ---


def s_decider_bounds_assessment() -> BuilderResult:
    from common.path_assessment_decider import PathAssessmentDecider
    from common.path_bounds_decider import (
        BuildCandidatePathsFromBoundaries,
        PathBoundsDecider,
    )

    frame, rli, start = build_lattice_plan_frame()
    if not PathBoundsDecider().Process(None, rli, None).ok():
        return frame, rli, start, True, None, "decider_skip"
    candidates = BuildCandidatePathsFromBoundaries(rli)
    rli.SetCandidatePathData(candidates)
    if not PathAssessmentDecider().Process(None, rli, None).ok() or rli.path_data is None:
        return frame, rli, start, True, None, "decider_skip"
    extra = f"path_label={rli.path_data.path_label!r} n_candidates={len(candidates)}"
    return frame, rli, start, True, None, extra


def s_decider_lane_borrow_bounds() -> BuilderResult:
    from common.path_bounds_decider import PathBoundsDecider
    from common.planning_context import PlanningContext

    obs = build_static_obstacle("borrow_block", 35.0)
    frame, rli, start = build_lattice_plan_frame(
        [obs], blocking_obstacle_id="borrow_block"
    )
    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ok = PathBoundsDecider().Process(None, rli, ctx).ok()
    labels = [b.label for b in rli.GetCandidatePathBoundaries()]
    extra = f"bounds_ok={ok} labels={labels}"
    return None, rli, None, ok, None, extra


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
    return None, rli, None, combined, None, extra


def s_decider_cruise_speed_data() -> BuilderResult:
    from common.planning_util import BuildCruiseSpeedData

    frame, rli, start = build_lattice_plan_frame()
    rli.SetLatticeCruiseSpeed(6.0)
    speed = BuildCruiseSpeedData(rli)
    ok = len(speed) >= 2 and all(pt.v == 6.0 for pt in speed)
    extra = f"speed_pts={len(speed)} v0={speed[0].v} s_end={speed[-1].s:.1f}"
    return None, rli, None, ok, None, extra


# --- OnLane ---


def s_on_lane_open_road() -> BuilderResult:
    import config as config_module
    from on_lane_planning import OnLanePlanning
    from common.frame import LocalView
    from reference_line.reference_line_provider import ReferenceLineProvider
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.pose import Pose
    from protoclass.prediction_obstacles import PredictionObstacles
    from protoclass.point_enu import PointENU
    from scripts.planner_test_fixtures import build_reference_line

    reference_line, _, reference_line_info = build_reference_line()
    provider = ReferenceLineProvider()
    provider._reference_lines = [reference_line]
    provider._route_segments = [reference_line_info.Lanes()]

    local_view = LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(position=PointENU(x=0.0, y=0.0, z=0.0), heading=0.0),
            measurement_time=0.0,
        ),
        chassis=Chassis(speed_mps=1.0, header=Header(timestamp_sec=0.0)),
        prediction_obstacles=PredictionObstacles(),
    )
    old_thread = config_module.FLAGS_enable_reference_line_provider_thread
    try:
        config_module.FLAGS_enable_reference_line_provider_thread = True
        planner = OnLanePlanning(provider)
        adc = ADCTrajectory()
        status = planner.RunOnce(local_view, adc)
        extra = f"status={status.code.name} out_pts={len(adc.trajectory_point)}"
        return None, None, None, status.ok(), None, extra, adc
    finally:
        config_module.FLAGS_enable_reference_line_provider_thread = old_thread


def s_on_lane_overtake_path_bounds() -> BuilderResult:
    import config as config_module
    from on_lane_planning import OnLanePlanning
    from common.frame import LocalView
    from common.planning_context import PlanningContext
    from reference_line.reference_line_provider import ReferenceLineProvider
    from protoclass.adc_trajectory import Point3D
    from protoclass.chassis import Chassis
    from protoclass.header import Header
    from protoclass.localization_estimate import LocalizationEstimate
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType
    from protoclass.pose import Pose
    from protoclass.prediction_obstacles import PredictionObstacle, PredictionObstacles
    from protoclass.point_enu import PointENU
    from scripts.planner_test_fixtures import build_left_lane_reference_line

    # Needs a reference line with a real neighbor lane (unlike the generic
    # single-lane build_reference_line fixture): PathBoundsDecider now looks
    # up the actual neighbor lane width via ReferenceLineInfo.GetNeighborLaneInfo
    # instead of assuming a flat default lane width, so a borrow boundary can
    # only be non-trivial where a real adjacent lane exists.
    reference_line, reference_line_info = build_left_lane_reference_line(length=100.0, init_v=5.0)
    provider = ReferenceLineProvider()
    provider._reference_lines = [reference_line]
    provider._route_segments = [reference_line_info.Lanes()]

    local_view = LocalView(
        localization_estimate=LocalizationEstimate(
            pose=Pose(position=PointENU(x=0.0, y=0.0, z=0.0), heading=0.0),
            measurement_time=0.0,
        ),
        chassis=Chassis(speed_mps=5.0, header=Header(timestamp_sec=0.0)),
        prediction_obstacles=PredictionObstacles(
            prediction_obstacle=[
                PredictionObstacle(
                    perception_obstacle=PerceptionObstacle(
                        id=1,
                        type=PerceptionObstacleType.VEHICLE,
                        position=Point3D(x=30.0, y=0.0, z=0.0),
                        velocity=Point3D(x=0.0, y=0.0, z=0.0),
                        length=4.0,
                        width=2.0,
                        height=1.5,
                        theta=0.0,
                    ),
                    is_static=True,
                )
            ]
        ),
    )
    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    # Only lane_right (this fixture's right neighbor) actually exists as a
    # real neighbor lane, so only RIGHT_BORROW can produce a non-trivial
    # boundary; a left-borrow attempt would legitimately fall back to
    # neighbor_width=0.0 with no real left neighbor to borrow.
    ctx.planning_status.path_decider.decided_side_pass_direction = [2]

    old_values = (
        config_module.FLAGS_enable_reference_line_provider_thread,
        config_module.FLAGS_enable_path_bounds_decider,
        config_module.FLAGS_enable_on_lane_combine_path_and_speed,
        config_module.FLAGS_enable_path_decider_after_lateral,
    )
    try:
        config_module.FLAGS_enable_reference_line_provider_thread = True
        config_module.FLAGS_enable_path_bounds_decider = True
        config_module.FLAGS_enable_on_lane_combine_path_and_speed = True
        config_module.FLAGS_enable_path_decider_after_lateral = True
        planner = OnLanePlanning(provider)
        adc = ADCTrajectory()
        status = planner.RunOnce(local_view, adc, planning_context=ctx)
    finally:
        (
            config_module.FLAGS_enable_reference_line_provider_thread,
            config_module.FLAGS_enable_path_bounds_decider,
            config_module.FLAGS_enable_on_lane_combine_path_and_speed,
            config_module.FLAGS_enable_path_decider_after_lateral,
        ) = old_values

    traj = adc.trajectory_point or []
    max_y = max((abs(p.path_point.y) for p in traj), default=0.0)
    near = [
        abs(p.path_point.y)
        for p in traj
        if p.path_point is not None and abs(p.path_point.x - 30.0) < 8.0
    ]
    near_peak = max(near) if near else 0.0
    label = ""
    if planner._last_publishable_trajectory is not None:
        label = "published"
    frame = getattr(planner, "_last_frame", None)
    viz_rli = frame.FindDriveReferenceLineInfo() if frame is not None else None
    obstacle_count = (
        len(viz_rli.path_decision.obstacles)
        if viz_rli is not None and viz_rli.path_decision is not None
        else 0
    )
    extra = (
        f"status={status.code.name} pts={len(traj)} max|y|={max_y:.2f} "
        f"near_obstacle|y|={near_peak:.2f} obstacles={obstacle_count} {label}"
    )
    ok = status.ok() and len(traj) > 0 and max_y > 1.0 and near_peak > 1.0
    return None, viz_rli, None, ok, None, extra, adc


# --- stress / 探测 ---


def s_stress_obstacle_distance_sweep() -> BuilderResult:
    """在 10–40m 多档障碍距离上探测 lattice 是否成功（信息用例）。"""
    results = []
    for x in (10, 15, 18, 22, 30, 40):
        obs = build_static_obstacle(f"car_{x}", float(x))
        frame, rli, start = build_lattice_plan_frame(
            [obs], blocking_obstacle_id=f"car_{x}"
        )
        old = config.FLAGS_enable_backup_trajectory
        config.FLAGS_enable_backup_trajectory = True
        try:
            ok = LatticePlanner().Plan(start, frame, ADCTrajectory())
        finally:
            config.FLAGS_enable_backup_trajectory = old
        end_x = (
            rli.trajectory[-1].path_point.x
            if ok and rli.trajectory
            else None
        )
        results.append(f"x={x}:{'ok' if ok else 'fail'} end={end_x}")
    extra = "sweep [" + "; ".join(results) + "]"
    return None, None, None, True, None, extra


def s_stress_init_speed_sweep() -> BuilderResult:
    results = []
    for v in (0.1, 1.0, 3.0, 6.0, 8.0):
        frame, rli, start = build_lattice_plan_frame(init_v=v)
        ok = LatticePlanner().Plan(start, frame, ADCTrajectory())
        end_v = rli.trajectory[-1].v if ok and rli.trajectory else None
        results.append(f"v0={v}:{'ok' if ok else 'fail'} vend={end_v}")
    extra = "sweep [" + "; ".join(results) + "]"
    return None, None, None, True, None, extra


# --- 超车：PathBounds S 形（推荐）/ 合成参考 / Lattice 跟停对照 ---


def s_overtake_path_bounds_left() -> BuilderResult:
    """推荐：PathBounds 左借道 + S 形 l(s) + Combine（对齐 C++ 借道超车栈）。"""
    frame, rli, start, ok, backup, detail = build_path_bounds_overtake_frame(
        30.0, path_label="regular/left/forward", init_v=5.0
    )
    return frame, rli, start, ok, backup, detail


def s_overtake_path_bounds_right() -> BuilderResult:
    """PathBounds 右借道 + S 形剖面。"""
    frame, rli, start, ok, backup, detail = build_path_bounds_overtake_frame(
        30.0, path_label="regular/right/forward", init_v=5.0
    )
    return frame, rli, start, ok, backup, detail


def s_overtake_s_curve_pass() -> BuilderResult:
    """参考动画：Cartesian S 形（与 PathBounds 剖面形态一致，便于对照）。"""
    frame, rli, start, ok, backup, detail = build_synthetic_overtake_frame(
        30.0, init_v=5.0, peak_y=3.2
    )
    return frame, rli, start, ok, backup, detail


def s_overtake_lattice_follow_no_pass() -> BuilderResult:
    """纯 Lattice：单车道只能跟停，不能横向超车（对照组）。"""
    frame, rli, start = build_overtake_lattice_frame(30.0, init_v=5.0)
    return frame, rli, start, True, None, "lattice_follow_not_pass"


def s_overtake_borrow_offset_not_s_curve() -> BuilderResult:
    """
    PathBounds 左借道：全程固定横向偏移（旧版错误「超车」）。
    用于对比说明：这不是绕障回归，而是借道定线行驶。
    """
    from common.path_bounds_decider import PathBoundsDecider, BuildCandidatePathsFromBoundaries
    from common.planning_context import PlanningContext
    from common.planning_util import BuildCruiseSpeedData
    from common.discretized_trajectory import DiscretizedTrajectory

    frame, rli, start = build_overtake_lattice_frame(30.0, init_v=5.0)
    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ctx.planning_status.path_decider.decided_side_pass_direction = [1]
    rli.set_is_path_lane_borrow(True)
    PathBoundsDecider().Process(None, rli, ctx)
    cands = BuildCandidatePathsFromBoundaries(rli)
    left = next(c for c in cands if c.path_label == "regular/left/forward")
    rli.SetPathData(left)
    rli.SetSpeedData(BuildCruiseSpeedData(rli))
    traj = DiscretizedTrajectory()
    ok = rli.CombinePathAndSpeedProfile(0.0, start.path_point.s, traj)
    if ok:
        rli.SetTrajectory(traj)
    ys = [p.path_point.y for p in traj] if traj else [0]
    detail = f"constant_offset y≈{ys[len(ys)//2]:.2f} (NOT S-curve)"
    return frame, rli, start, ok, None, detail


def s_overtake_two_leaders_s_curve() -> BuilderResult:
    """双前车：S 形绕行通过第一辆（第二辆仍较远）。"""
    from scripts.planner_test_fixtures import (
        apply_cartesian_overtake_trajectory,
        build_cartesian_overtake_path_points,
        build_lattice_plan_frame,
    )

    frame, rli, start = build_lattice_plan_frame(
        [
            build_slow_leader_obstacle("lead1", 22.0),
            build_slow_leader_obstacle("lead2", 55.0),
        ],
        init_v=5.0,
        blocking_obstacle_id="lead1",
    )
    pts = build_cartesian_overtake_path_points(
        obstacle_x=22.0,
        peak_y=3.0,
        pass_length=8.0,
        length_x=85.0,
    )
    ok, detail = apply_cartesian_overtake_trajectory(rli, start, pts, cruise_v=6.0)
    return frame, rli, start, ok, None, detail


def s_overtake_far_leader_s_curve() -> BuilderResult:
    """前车较远 45m：更容易完成绕行。"""
    frame, rli, start, ok, backup, detail = build_synthetic_overtake_frame(
        45.0, init_v=6.0, peak_y=2.8
    )
    return frame, rli, start, ok, backup, detail


def s_overtake_stack_combine() -> BuilderResult:
    """经 PathData + Combine 栈生成 S 形超车轨迹。"""
    frame, rli, start, ok, backup, detail = build_lattice_overtake_combined_frame(
        30.0, init_v=5.0, peak_y=3.2
    )
    return frame, rli, start, ok, backup, detail


SCENARIOS: List[Scenario] = [
    # lattice
    Scenario("open_road", "空旷直道", "lattice", s_open_road),
    Scenario("far_obstacle_stop", "35m 静止车 + blocking", "lattice", s_far_obstacle_stop),
    Scenario(
        "close_obstacle_no_backup",
        "8m 静止车，关 backup，应失败",
        "lattice",
        s_close_obstacle_no_backup,
        expect_ok=False,
    ),
    Scenario(
        "close_obstacle_with_backup",
        "8m 静止车，开 backup，fallback 成功",
        "lattice",
        s_close_obstacle_with_backup,
        backup=True,
    ),
    Scenario("higher_speed_cruise", "初速 5 m/s", "lattice", s_higher_speed_cruise),
    Scenario("stopped_start", "近零速起步 v=0.1", "lattice", s_stopped_start),
    Scenario("mid_lane_start", "从 x=20m 处接续规划", "lattice", s_mid_lane_start),
    Scenario("lateral_offset_start", "横向偏置 y=0.6m 起步", "lattice", s_lateral_offset_start),
    Scenario("two_obstacles_queue", "22m + 45m 双车队列", "lattice", s_two_obstacles_queue),
    Scenario(
        "obstacle_no_blocking_flag",
        "30m 障碍但不标 blocking",
        "lattice",
        s_obstacle_no_blocking_flag,
    ),
    Scenario(
        "side_obstacle_adjacent_lane",
        "邻车道静止车 y=2.8m",
        "lattice",
        s_side_obstacle_adjacent_lane,
    ),
    Scenario("medium_obstacle_18m", "18m blocking 停车", "lattice", s_medium_obstacle_18m),
    Scenario("long_road_150m", "150m 长参考线", "lattice", s_long_road_150m),
    Scenario("low_speed_creep", "蠕行 v=0.3", "lattice", s_low_speed_creep),
    Scenario("obstacle_60m_pass", "60m 远处障碍", "lattice", s_obstacle_60m_pass),
    Scenario(
        "backup_off_open_road",
        "关 backup 空旷道仍应成功",
        "lattice",
        s_backup_off_open_road,
        backup=False,
    ),
    Scenario("curved_open_road", "曲线车道空旷巡航", "lattice", s_curved_open_road),
    Scenario("curved_obstacle_stop", "曲线车道前车停车", "lattice", s_curved_obstacle_stop),
    # decider
    Scenario(
        "decider_bounds_assessment",
        "PathBounds + PathAssessment",
        "decider",
        s_decider_bounds_assessment,
    ),
    Scenario(
        "decider_lane_borrow_bounds",
        "借道场景 PathBounds 多边界",
        "decider",
        s_decider_lane_borrow_bounds,
    ),
    Scenario(
        "decider_boundary_combine",
        "Path + 巡航速度 Combine",
        "decider",
        s_decider_boundary_combine,
    ),
    Scenario(
        "decider_cruise_speed_data",
        "BuildCruiseSpeedData",
        "decider",
        s_decider_cruise_speed_data,
    ),
    # on_lane
    Scenario(
        "on_lane_open_road",
        "OnLanePlanning.RunOnce 空旷",
        "on_lane",
        s_on_lane_open_road,
        informational=True,
    ),
    Scenario(
        "on_lane_overtake_path_bounds",
        "OnLanePlanning PathBounds 借道超车",
        "on_lane",
        s_on_lane_overtake_path_bounds,
    ),
    # stress
    Scenario(
        "stress_obstacle_distance_sweep",
        "障碍距离 10–40m 扫描",
        "stress",
        s_stress_obstacle_distance_sweep,
        informational=True,
    ),
    Scenario(
        "stress_init_speed_sweep",
        "初速 0.1–8 m/s 扫描",
        "stress",
        s_stress_init_speed_sweep,
        informational=True,
    ),
    # overtake
    Scenario(
        "overtake_path_bounds_left",
        "【推荐】PathBounds 左借道 S 形超车",
        "overtake",
        s_overtake_path_bounds_left,
        skip_lattice=True,
    ),
    Scenario(
        "overtake_path_bounds_right",
        "PathBounds 右借道 S 形超车",
        "overtake",
        s_overtake_path_bounds_right,
        skip_lattice=True,
    ),
    Scenario(
        "overtake_s_curve_pass",
        "Cartesian S 形参考轨迹（动画对照）",
        "overtake",
        s_overtake_s_curve_pass,
        skip_lattice=True,
    ),
    Scenario(
        "overtake_lattice_follow_no_pass",
        "纯 Lattice：单车道跟停，无法横向超车（对照）",
        "overtake",
        s_overtake_lattice_follow_no_pass,
    ),
    Scenario(
        "overtake_borrow_constant_offset",
        "PathBounds 左借道：全程固定横向偏移（非 S 形，旧误解）",
        "overtake",
        s_overtake_borrow_offset_not_s_curve,
        skip_lattice=True,
        informational=True,
    ),
    Scenario(
        "overtake_two_leaders_s_curve",
        "双前车：S 形绕开第一辆",
        "overtake",
        s_overtake_two_leaders_s_curve,
        skip_lattice=True,
    ),
    Scenario(
        "overtake_far_leader_s_curve",
        "前车 45m：S 形绕行",
        "overtake",
        s_overtake_far_leader_s_curve,
        skip_lattice=True,
    ),
    Scenario(
        "overtake_stack_combine",
        "PathData + Combine 栈生成 S 形超车",
        "overtake",
        s_overtake_stack_combine,
        skip_lattice=True,
    ),
]

SCENARIOS_BY_NAME = {s.name: s for s in SCENARIOS}
