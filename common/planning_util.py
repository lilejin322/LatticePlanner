"""
Planning utility functions aligned with modules/planning/common/util/common.cc.
"""

import math
from typing import Any, List, Optional

from common.discretized_path import DiscretizedPath
from common.discretized_trajectory import DiscretizedTrajectory
from common.frenet_frame_path import FrenetFramePath
from common.frame import Frame
from common.path_boundary import PathBoundary
from common.path_data import PathData
from common.polygon2d import Polygon2d
from reference_line.reference_line_info import ReferenceLineInfo
from common.speed_data import SpeedData
from common.vec2d import Vec2d
from protoclass.decision_result import ObjectDecisionType, ObjectStop, StopReasonCode
from protoclass.frenet_frame_point import FrenetFramePoint
from protoclass.point_enu import PointENU
from protoclass.vehicle_state import VehicleState
import config as config_module


def _make_stop_decision(
    stop_reason_code: StopReasonCode,
    stop_distance: float,
    stop_heading: float,
    stop_point,
    wait_for_obstacles: List[str],
) -> ObjectDecisionType:
    stop_obj = ObjectStop(
        reason_code=stop_reason_code,
        distance_s=-stop_distance,
        stop_heading=stop_heading,
        stop_point=PointENU(x=stop_point.x, y=stop_point.y, z=0.0),
        wait_for_obstacle=list(wait_for_obstacles or []),
    )
    decision = ObjectDecisionType()
    decision.object_tag = stop_obj
    return decision


def WithinBound(lower: float, upper: float, value: float) -> bool:
    return lower <= value <= upper


def GetADCStopDeceleration(
    vehicle_state: Optional[VehicleState],
    adc_front_edge_s: float,
    stop_line_s: float,
) -> float:
    """Required deceleration to stop at stop_line_s from current speed."""
    if vehicle_state is None:
        return float("inf")
    adc_speed = vehicle_state.linear_velocity or 0.0
    if adc_speed < config_module.FLAGS_max_abs_speed_when_stopped:
        return 0.0
    stop_distance = max(0.0, stop_line_s - adc_front_edge_s)
    if stop_distance < 1e-5:
        return float("inf")
    return (adc_speed * adc_speed) / (2.0 * stop_distance)


def CrosswalkPolygon(crosswalk: Any) -> Optional[Polygon2d]:
    polygon = getattr(crosswalk, "polygon", None)
    if polygon is None:
        return None
    points = getattr(polygon, "point", None) or []
    if len(points) < 3:
        return None
    return Polygon2d([Vec2d(p.x, p.y) for p in points])


def BuildStopDecision(
    stop_wall_id: str,
    stop_line_s: float,
    stop_distance: float,
    stop_reason_code: StopReasonCode,
    wait_for_obstacles: List[str],
    decision_tag: str,
    frame: Frame,
    reference_line_info: ReferenceLineInfo,
) -> int:
    reference_line = reference_line_info.reference_line
    if not WithinBound(0.0, reference_line.Length(), stop_line_s):
        return 0

    obstacle = frame.CreateStopObstacle(reference_line_info, stop_wall_id, stop_line_s)
    if obstacle is None:
        return -1
    stop_wall = reference_line_info.AddObstacle(obstacle)
    if stop_wall is None:
        return -1

    stop_s = stop_line_s - stop_distance
    stop_point = reference_line.GetReferencePoint(stop_s)
    stop = _make_stop_decision(
        stop_reason_code, stop_distance, stop_point.heading, stop_point, wait_for_obstacles
    )
    reference_line_info.path_decision.AddLongitudinalDecision(decision_tag, stop_wall.Id(), stop)
    return 0


def BuildStopDecisionOnLane(
    stop_wall_id: str,
    lane_id: str,
    lane_s: float,
    stop_distance: float,
    stop_reason_code: StopReasonCode,
    wait_for_obstacles: List[str],
    decision_tag: str,
    frame: Frame,
    reference_line_info: ReferenceLineInfo,
) -> int:
    reference_line = reference_line_info.reference_line
    obstacle = frame.CreateStopObstacle(stop_wall_id, lane_id, lane_s)
    if obstacle is None:
        return -1

    stop_wall = reference_line_info.AddObstacle(obstacle)
    if stop_wall is None:
        return -1

    stop_wall_box = stop_wall.PerceptionBoundingBox()
    if not reference_line.IsOnLane(stop_wall_box.center):
        return 0

    stop_point = reference_line.GetReferencePoint(
        stop_wall.PerceptionSLBoundary().start_s - stop_distance
    )
    # NOTE: the C++ lane_id/lane_s overload (modules/planning/common/util/common.cc)
    # never calls add_wait_for_obstacle() for this variant -- wait_for_obstacles is
    # accepted for signature parity with BuildStopDecision but is otherwise dead
    # here, so it must not be forwarded into the resulting decision.
    stop = _make_stop_decision(
        stop_reason_code, stop_distance, stop_point.heading, stop_point, []
    )
    reference_line_info.path_decision.AddLongitudinalDecision(decision_tag, stop_wall.Id(), stop)
    return 0


def _assign_frenet_path(path_data: PathData, frenet_points: List[FrenetFramePoint]) -> None:
    from common.discretized_path import DiscretizedPath
    from protoclass.path_point import PathPoint
    from protoclass.sl_boundary import SLPoint
    from cartesian_frenet_converter import CartesianFrenetConverter

    frenet_path = FrenetFramePath(frenet_points)
    if path_data.SetFrenetPath(frenet_path):
        return

    reference_line = path_data._reference_line
    if reference_line is None:
        path_data._frenet_path = frenet_path
        return

    max_s = reference_line.Length()
    kept_frenet: List[FrenetFramePoint] = []
    path_points: List[PathPoint] = []
    for frenet_point in frenet_points:
        s = max(0.0, min(frenet_point.s, max_s))
        ok, cartesian_point = reference_line.SLToXY(SLPoint(s, frenet_point.l))
        if not ok:
            continue
        ref_point = reference_line.GetReferencePoint(s)
        theta = CartesianFrenetConverter.CalculateTheta(
            ref_point.heading, ref_point.kappa, frenet_point.l, frenet_point.dl
        )
        kappa = CartesianFrenetConverter.CalculateKappa(
            ref_point.kappa,
            ref_point.dkappa,
            frenet_point.l,
            frenet_point.dl,
            frenet_point.ddl,
        )
        path_s = 0.0
        dkappa = 0.0
        if path_points:
            last = path_points[-1]
            dx = cartesian_point.x - last.x
            dy = cartesian_point.y - last.y
            path_s = last.s + math.hypot(dx, dy)
            if path_s > last.s + 1e-6:
                dkappa = (kappa - last.kappa) / (path_s - last.s)
        path_points.append(
            PathPoint(
                cartesian_point.x,
                cartesian_point.y,
                0.0,
                theta,
                kappa,
                path_s,
                dkappa,
            )
        )
        kept_frenet.append(
            FrenetFramePoint(
                s=s,
                l=frenet_point.l,
                dl=frenet_point.dl,
                ddl=frenet_point.ddl,
            )
        )

    if kept_frenet and path_points:
        path_data._frenet_path = FrenetFramePath(kept_frenet)
        path_data._discretized_path = DiscretizedPath(path_points)
    else:
        path_data._frenet_path = frenet_path


def SetupNominalPathData(
    reference_line_info: ReferenceLineInfo,
    start_s: float,
    init_d: List[float],
    end_s: float,
    step: float = 0.5,
    *,
    set_on_reference_line: bool = True,
    path_label: str = "regular/self",
) -> PathData:
    """Build a constant-lateral nominal frenet path for PathDecider."""
    path_data = BuildLatticeCandidatePath(
        reference_line_info,
        start_s,
        init_d,
        end_s,
        lat_trajectory=None,
        step=step,
        path_label=path_label,
    )
    if set_on_reference_line:
        reference_line_info.SetPathData(path_data)
    return path_data


def BuildLatticeCandidatePath(
    reference_line_info: ReferenceLineInfo,
    start_s: float,
    init_d: List[float],
    end_s: float,
    lat_trajectory=None,
    step: float = 0.5,
    path_label: str = "regular/self",
) -> PathData:
    """Build candidate path data without mutating reference_line_info.path_data."""
    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    path_data.set_path_label(path_label)

    if lat_trajectory is None:
        d = init_d[0] if init_d else 0.0
        dd = init_d[1] if len(init_d) > 1 else 0.0
        ddd = init_d[2] if len(init_d) > 2 else 0.0
        frenet_points = []
        s = start_s
        while s <= end_s + 1e-6:
            frenet_points.append(FrenetFramePoint(s=s, l=d, dl=dd, ddl=ddd))
            s += step
        if not frenet_points:
            frenet_points.append(FrenetFramePoint(s=start_s, l=d, dl=dd, ddl=ddd))
    else:
        relative_horizon = min(max(0.0, end_s - start_s), lat_trajectory.ParamLength())
        frenet_points = []
        s = start_s
        while s <= start_s + relative_horizon + 1e-6:
            rel_s = min(s - start_s, lat_trajectory.ParamLength())
            frenet_points.append(
                FrenetFramePoint(
                    s=s,
                    l=lat_trajectory.Evaluate(0, rel_s),
                    dl=lat_trajectory.Evaluate(1, rel_s),
                    ddl=lat_trajectory.Evaluate(2, rel_s),
                )
            )
            s += step
        if not frenet_points:
            frenet_points.append(
                FrenetFramePoint(
                    s=start_s,
                    l=lat_trajectory.Evaluate(0, 0.0),
                    dl=lat_trajectory.Evaluate(1, 0.0),
                    ddl=lat_trajectory.Evaluate(2, 0.0),
                )
            )

    _assign_frenet_path(path_data, frenet_points)
    return path_data


def BuildFrenetPathFromLatTrajectory(
    reference_line_info: ReferenceLineInfo,
    start_s: float,
    lat_trajectory,
    end_s: float,
    step: float = 0.5,
    *,
    set_on_reference_line: bool = True,
    path_label: str = "regular/self",
) -> PathData:
    """Build frenet path from the selected lateral lattice trajectory."""
    path_data = BuildLatticeCandidatePath(
        reference_line_info,
        start_s,
        [0.0, 0.0, 0.0],
        end_s,
        lat_trajectory=lat_trajectory,
        step=step,
        path_label=path_label,
    )
    if set_on_reference_line:
        reference_line_info.SetPathData(path_data)
    return path_data


def AggregateReferenceLineTrajectory(
    reference_line_info: ReferenceLineInfo,
    planning_start_point,
) -> bool:
    """
    Build discretized trajectory from path_data + speed_data (lane_follow_stage.cc).
    No-op when path or speed is missing; keeps existing trajectory if combine fails.
    """
    if not config_module.FLAGS_enable_on_lane_combine_path_and_speed:
        return reference_line_info.trajectory is not None and len(
            reference_line_info.trajectory
        ) > 0

    path_data = reference_line_info.path_data
    speed_data = reference_line_info.speed_data
    if path_data is None or path_data.Empty() or not speed_data:
        return reference_line_info.trajectory is not None and len(
            reference_line_info.trajectory
        ) > 0

    trajectory = DiscretizedTrajectory()
    start_s = (planning_start_point.path_point.s or 0.0) if planning_start_point.path_point else 0.0
    rel_t = planning_start_point.relative_time or 0.0
    if reference_line_info.CombinePathAndSpeedProfile(rel_t, start_s, trajectory):
        reference_line_info.SetTrajectory(trajectory)
        return len(trajectory) > 0
    return reference_line_info.trajectory is not None and len(
        reference_line_info.trajectory
    ) > 0


def BuildCruiseSpeedData(
    reference_line_info: ReferenceLineInfo,
    start_path_s: float = 0.0,
    cruise_speed: Optional[float] = None,
    horizon_time: Optional[float] = None,
) -> SpeedData:
    """Constant-speed profile along path s (aligned with lane_follow combine fallback)."""
    speed_data = SpeedData()
    v = cruise_speed
    if v is None:
        target = reference_line_info.planning_target
        if target is not None and (target.cruise_speed or 0.0) > config_module.FLAGS_numerical_epsilon:
            v = target.cruise_speed
        else:
            v = reference_line_info.GetCruiseSpeed()
    if v is None or v < config_module.FLAGS_numerical_epsilon:
        v = config_module.FLAGS_default_cruise_speed
    v = max(v, config_module.FLAGS_numerical_epsilon)

    horizon = horizon_time if horizon_time is not None else config_module.FLAGS_trajectory_time_length
    t = 0.0
    while t <= horizon + 1e-6:
        path_s = start_path_s + v * t
        speed_data.AppendSpeedPoint(path_s, t, v, 0.0, 0.0)
        t += config_module.FLAGS_trajectory_time_resolution
    if len(speed_data) < 2:
        speed_data.AppendSpeedPoint(
            start_path_s + v * config_module.FLAGS_trajectory_time_resolution,
            config_module.FLAGS_trajectory_time_resolution,
            v,
            0.0,
            0.0,
        )
    return speed_data


def BuildLatticeLongitudinalSpeedData(
    reference_line_info: ReferenceLineInfo,
    planning_point,
    obstacles: Optional[List] = None,
) -> SpeedData:
    """Lattice longitudinal bundle (cruise, follow, stop), not a constant-speed tape.

    Falls back to cruise only when every generated curve is invalid.
    """
    from behavior.path_time_graph import PathTimeGraph
    from behavior.prediction_querier import PredictionQuerier
    from common.constraint_checker1d import ConstraintChecker1d
    from lattice_planner import ComputeInitFrenetState, ToDiscretizedReferenceLine
    from path_matcher import PathMatcher
    from trajectory_generation.trajectory1d_generator import Trajectory1dGenerator

    ref_points = ToDiscretizedReferenceLine(reference_line_info.reference_line.reference_points)
    matched = PathMatcher.MatchToPath(
        ref_points,
        planning_point.path_point.x,
        planning_point.path_point.y,
    )
    init_s, init_d = ComputeInitFrenetState(matched, planning_point)
    if obstacles is None and reference_line_info.path_decision is not None:
        obstacles = list(reference_line_info.path_decision.obstacles.values())
    obstacles = list(obstacles or [])
    querier = PredictionQuerier(obstacles, ref_points)
    path_time_graph = PathTimeGraph(
        querier.GetObstacles(),
        ref_points,
        reference_line_info,
        init_s[0],
        init_s[0] + config_module.FLAGS_speed_lon_decision_horizon,
        0.0,
        config_module.FLAGS_trajectory_time_length,
        init_d,
    )
    cruise = planning_point.v or 0.0
    if cruise < config_module.FLAGS_numerical_epsilon:
        cruise = reference_line_info.GetCruiseSpeed() or config_module.FLAGS_default_cruise_speed
    reference_line_info.SetLatticeCruiseSpeed(cruise)
    generator = Trajectory1dGenerator(init_s, init_d, path_time_graph, querier)
    lon_bundle = []
    generator.GenerateLongitudinalTrajectoryBundle(reference_line_info.planning_target, lon_bundle)
    valid = [
        curve
        for curve in lon_bundle
        if curve is not None and ConstraintChecker1d.IsValidLongitudinalTrajectory(curve)
    ]
    if not valid:
        return BuildCruiseSpeedData(reference_line_info, cruise_speed=cruise)

    block_s = _nearest_blocking_s(
        obstacles,
        init_s[0],
        reference_line_info.path_data,
    )

    def _score(curve) -> float:
        # Closed loop only executes the next fraction of a second, so rank
        # curves by that prefix instead of where they end several seconds later.
        horizon = min(1.0, curve.ParamLength())
        if horizon <= 1e-6:
            return 1.0e6
        end_s = curve.Evaluate(0, horizon)
        samples = [i * 0.1 for i in range(int(horizon / 0.1) + 1)]
        speed_error = sum(abs(curve.Evaluate(1, t) - cruise) for t in samples) / len(samples)
        if block_s is None:
            return speed_error
        if end_s > block_s:
            return 1000.0 + (end_s - block_s)
        return (block_s - end_s) * 0.1 + speed_error

    best = min(valid, key=_score)
    return BuildSpeedDataFromLonTrajectory(best, init_s[0])


def _path_clears_obstacle(path_data, sl) -> bool:
    if path_data is None or path_data.Empty() or not path_data.frenet_frame_path:
        return False
    frenet_path = path_data.frenet_frame_path
    if len(frenet_path) < 2 or sl.start_s is None:
        return False
    # EvaluateByS past the last knot returns that knot. A short lookahead
    # must not be treated as having already passed an obstacle it never reaches.
    if frenet_path[-1].s < sl.start_s - 1.0:
        return False
    point = frenet_path.EvaluateByS(sl.start_s)
    margin = config_module.EGO_VEHICLE_WIDTH * 0.5
    return point.l > (sl.end_l or 0.0) + margin or point.l < (sl.start_l or 0.0) - margin


def _nearest_blocking_s(obstacles, ego_s: float, path_data=None) -> Optional[float]:
    half_width = config_module.EGO_VEHICLE_WIDTH * 0.5
    nearest = None
    for obstacle in obstacles:
        if obstacle.IsVirtual():
            continue
        sl = obstacle.PerceptionSLBoundary()
        if sl is None or sl.start_s is None or sl.end_l is None or sl.start_l is None:
            continue
        if sl.end_s < ego_s:
            continue
        if _path_clears_obstacle(path_data, sl):
            continue
        if sl.end_l < -half_width or sl.start_l > half_width:
            continue
        stop_s = sl.start_s - config_module.FLAGS_min_stop_distance_obstacle
        nearest = stop_s if nearest is None else min(nearest, stop_s)
    return nearest


def BuildSpeedDataFromLonTrajectory(lon_trajectory, start_ref_s: float) -> SpeedData:
    """Build SpeedData from a lattice longitudinal 1D curve."""
    speed_data = SpeedData()
    s0 = lon_trajectory.Evaluate(0, 0.0)
    t = 0.0
    while t <= config_module.FLAGS_trajectory_time_length + 1e-6:
        ref_s = lon_trajectory.Evaluate(0, t)
        path_s = max(0.0, ref_s - s0)
        v = lon_trajectory.Evaluate(1, t)
        a = lon_trajectory.Evaluate(2, t)
        speed_data.AppendSpeedPoint(path_s, t, v, a, 0.0)
        t += config_module.FLAGS_trajectory_time_resolution
    if len(speed_data) < 2:
        speed_data.AppendSpeedPoint(max(0.0, start_ref_s - s0), config_module.FLAGS_trajectory_time_resolution, 0.0, 0.0, 0.0)
    return speed_data


def InferLatticePathLabel(
    start_s: float,
    end_s: float,
    init_d: List[float],
    lat_trajectory=None,
) -> str:
    """Infer Apollo-style path label from lateral offset."""
    if lat_trajectory is None:
        lateral = init_d[0] if init_d else 0.0
    else:
        rel = min(max(0.0, end_s - start_s), lat_trajectory.ParamLength())
        lateral = lat_trajectory.Evaluate(0, rel)
    if lateral > 0.5:
        return "regular/left/forward"
    if lateral < -0.5:
        return "regular/right/forward"
    return "regular/self"


def BuildPathDataFromDiscretizedTrajectory(
    reference_line_info: ReferenceLineInfo,
    trajectory: DiscretizedTrajectory,
    path_label: str = "fallback/self",
) -> PathData:
    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    path_data.set_path_label(path_label)
    discretized = DiscretizedPath()
    for point in trajectory:
        discretized.append(point.path_point)
    path_data.SetDiscretizedPath(discretized)
    return path_data


def BuildSpeedDataFromDiscretizedTrajectory(trajectory: DiscretizedTrajectory) -> SpeedData:
    speed_data = SpeedData()
    if not trajectory:
        return speed_data
    t0 = trajectory[0].relative_time or 0.0
    for point in trajectory:
        rel_t = (point.relative_time or 0.0) - t0
        speed_data.AppendSpeedPoint(
            point.path_point.s or 0.0,
            rel_t,
            point.v or 0.0,
            point.a or 0.0,
            0.0,
        )
    if len(speed_data) < 2 and trajectory:
        speed_data.AppendSpeedPoint(
            trajectory[-1].path_point.s or 0.0,
            (trajectory[-1].relative_time or 0.0) - t0 + config_module.FLAGS_trajectory_time_resolution,
            0.0,
            0.0,
            0.0,
        )
    return speed_data


K_PATH_BOUNDS_DECIDER_RESOLUTION = 0.5


def _pass_l_from_boundary_point(pt, label: str) -> float:
    """Recommended lateral passing position within the borrow-lane corridor
    (biased toward the upper bound when borrowing left, lower bound when
    borrowing right)."""
    if "left" in label:
        return pt.l_upper.l * 0.85 + pt.l_lower.l * 0.15
    if "right" in label:
        return pt.l_lower.l * 0.85 + pt.l_upper.l * 0.15
    return 0.5 * (pt.l_lower.l + pt.l_upper.l)


def _target_l_from_path_boundary(boundary: PathBoundary) -> float:
    if not boundary:
        return 0.0
    label = boundary.label or ""
    l_values = [_pass_l_from_boundary_point(pt, label) for pt in boundary]
    return sum(l_values) / len(l_values) if l_values else 0.0


def _lateral_solution_inside_bounds(opt_d, d_bounds, slack: float = 0.05) -> bool:
    if len(opt_d) != len(d_bounds):
        return False
    for value, (lo, hi) in zip(opt_d, d_bounds):
        if value != value or value < lo - slack or value > hi + slack:
            return False
    return True


def BuildOvertakePathDataFromPathBoundary(
    reference_line_info: ReferenceLineInfo,
    boundary: PathBoundary,
) -> Optional[PathData]:
    """Solve a lateral QP inside the borrow corridor.

    Hard bounds are the PathBounds tube. The objective tracks the pass side
    of that tube over a short lookahead, and the last knot is pinned there
    so a 0.1s execution step cannot stay on a flat start. Replanning from
    the executed heading continues the maneuver. This replaces the previous
    hand-drawn smoothstep S-curve.
    """
    points = list(boundary)
    if len(points) < 2:
        return None

    delta_s = points[1].s - points[0].s
    if delta_s <= 1e-3:
        return None
    label = boundary.label or "regular/self"
    d_bounds = []
    ref_d = []
    for pt in points:
        lo = float(pt.l_lower.l)
        hi = float(pt.l_upper.l)
        if lo > hi:
            lo, hi = hi, lo
        if hi - lo < 0.05:
            mid = 0.5 * (lo + hi)
            lo, hi = mid - 0.025, mid + 0.025
        d_bounds.append((lo, hi))
        preferred = _pass_l_from_boundary_point(pt, label)
        ref_d.append(min(max(preferred, lo), hi))

    # A full-corridor QP postpones the borrow, so a 0.1s execution step never
    # leaves the flat start. Keep a short lookahead and pin the last knot to
    # the corridor's pass side so the immediate path has to move.
    lookahead = max(delta_s * 2.0, 6.0)
    keep = 1
    for index, pt in enumerate(points):
        if pt.s - points[0].s <= lookahead + 1e-6:
            keep = index + 1
    d_bounds = d_bounds[:keep]
    ref_d = ref_d[:keep]
    points = points[:keep]
    end_l = ref_d[-1]
    pinned_bounds = list(d_bounds)
    pinned_bounds[-1] = (end_l, end_l)
    loose_bounds = list(d_bounds)

    adc = reference_line_info.AdcSlBoundary()
    if adc is not None and adc.start_l is not None and adc.end_l is not None:
        d0 = 0.5 * (adc.start_l + adc.end_l)
    else:
        d0 = 0.0
    d0 = min(max(d0, d_bounds[0][0]), d_bounds[0][1])
    planning_point = reference_line_info._adc_planning_point
    heading = 0.0
    if planning_point is not None and planning_point.path_point is not None:
        heading = planning_point.path_point.theta or 0.0
    ref_heading = reference_line_info.reference_line.GetReferencePoint(points[0].s).heading or 0.0
    dl0 = math.tan(max(-1.0, min(1.0, heading - ref_heading)))
    dl0 = max(-2.0, min(2.0, dl0))

    from trajectory_generation.lateral_osqp_optimizer import LateralOSQPOptimizer

    optimizer = LateralOSQPOptimizer()
    old_weight = config_module.FLAGS_weight_lateral_obstacle_distance
    old_jerk = config_module.FLAGS_lateral_third_order_derivative_max
    # Offset weight 1 loses the race against a 0.1s execution step: each replan
    # spreads the borrow over the whole corridor, so the vehicle only climbs a
    # few centimeters per cycle and hits the obstacle-side wall. A stronger
    # tracking weight makes the QP commit to the pass side early.
    config_module.FLAGS_weight_lateral_obstacle_distance = max(old_weight, 100.0)
    solved = False
    try:
        # The default jerk limit can make a sudden borrow corridor infeasible.
        # OSQP then returns a finite but wildly infeasible primal; reject that
        # and retry with a looser jerk limit until the path stays in the tube.
        for bounds in (pinned_bounds, loose_bounds):
            if solved:
                break
            for jerk_limit in (old_jerk, 0.5, 1.0, 2.0, 10.0):
                config_module.FLAGS_lateral_third_order_derivative_max = jerk_limit
                if not optimizer.Optimize([d0, dl0, 0.0], delta_s, bounds, ref_d):
                    continue
                if _lateral_solution_inside_bounds(optimizer._opt_d, bounds):
                    solved = True
                    break
        if not solved and optimizer._opt_d and len(optimizer._opt_d) == len(loose_bounds):
            optimizer._opt_d = [
                min(max(value, lo), hi)
                for value, (lo, hi) in zip(optimizer._opt_d, loose_bounds)
            ]
            optimizer._opt_d_prime = [0.0] * len(optimizer._opt_d)
            optimizer._opt_d_pprime = [0.0] * len(optimizer._opt_d)
            solved = _lateral_solution_inside_bounds(optimizer._opt_d, loose_bounds)
    finally:
        config_module.FLAGS_weight_lateral_obstacle_distance = old_weight
        config_module.FLAGS_lateral_third_order_derivative_max = old_jerk
    if not solved or not optimizer._opt_d:
        return None

    frenet_points = optimizer.GetFrenetFramePath()
    start_s = points[0].s
    for index, frenet_point in enumerate(frenet_points):
        frenet_point.s = start_s + index * delta_s

    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    path_data.set_path_label(boundary.label or "regular/self")
    blocking_id = boundary.blocking_obstacle_id
    if blocking_id:
        path_data.set_blocking_obstacle_id(blocking_id)
    _assign_frenet_path(path_data, frenet_points)
    return path_data


def BuildPathDataFromPathBoundary(
    reference_line_info: ReferenceLineInfo,
    boundary: PathBoundary,
) -> Optional[PathData]:
    if not boundary:
        return None
    target_l = _target_l_from_path_boundary(boundary)
    start_s = boundary.start_s()
    end_s = boundary[-1].s if boundary else start_s
    return BuildLatticeCandidatePath(
        reference_line_info,
        start_s,
        [target_l, 0.0, 0.0],
        end_s,
        lat_trajectory=None,
        step=K_PATH_BOUNDS_DECIDER_RESOLUTION,
        path_label=boundary.label or "regular/self",
    )
