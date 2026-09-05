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
    stop = _make_stop_decision(
        stop_reason_code, stop_distance, stop_point.heading, stop_point, wait_for_obstacles
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


def _smoothstep01(r: float) -> float:
    r = max(0.0, min(1.0, r))
    return r * r * (3.0 - 2.0 * r)


def _pass_l_from_boundary_point(pt, label: str) -> float:
    """借道走廊内推荐通行横向位置（左借道偏上界，右借道偏下界）。"""
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


def _blocking_s_range_for_overtake(
    reference_line_info: ReferenceLineInfo,
    boundary: PathBoundary,
    *,
    nudge_before: float = 10.0,
    return_after: float = 8.0,
) -> tuple[float, float]:
    """从前车 SL 或走廊收窄段推断绕行 s 区间。"""
    obs = reference_line_info.GetBlockingObstacle()
    if obs is not None:
        sl = obs.PerceptionSLBoundary()
        if sl is not None and sl.start_s is not None and sl.end_s is not None:
            return sl.start_s - nudge_before, sl.end_s + return_after

    adc_end_s = reference_line_info.AdcSlBoundary().end_s
    nearest_sl = None
    for obstacle in reference_line_info.path_decision.obstacles.values():
        if obstacle.IsVirtual() or not obstacle.IsStatic():
            continue
        sl = obstacle.PerceptionSLBoundary()
        if sl is None or sl.end_s < adc_end_s:
            continue
        if sl.end_l < -config_module.FLAGS_half_vehicle_width or sl.start_l > config_module.FLAGS_half_vehicle_width:
            continue
        if nearest_sl is None or sl.start_s < nearest_sl.start_s:
            nearest_sl = sl
    if nearest_sl is not None:
        return nearest_sl.start_s - nudge_before, nearest_sl.end_s + return_after

    label = boundary.label or ""
    adc_s = boundary.start_s() if boundary else 0.0
    best_s = adc_s + 25.0
    min_width = float("inf")
    for pt in boundary:
        if pt.s < adc_s + 5.0:
            continue
        width = pt.l_upper.l - pt.l_lower.l
        if width < min_width:
            min_width = width
            best_s = pt.s
    return best_s - nudge_before, best_s + return_after


def BuildOvertakePathDataFromPathBoundary(
    reference_line_info: ReferenceLineInfo,
    boundary: PathBoundary,
    *,
    nudge_ramp: float = 5.0,
    return_ramp: float = 5.0,
) -> Optional[PathData]:
    """
    借道 PathBounds 走廊生成 S 形 Frenet 路径：本车道 → 绕开 blocking → 回归。
    对应 C++ 栈里 PathBounds + path optimizer 应产出的超车形态（Python 用剖面近似）。
    """
    if not boundary:
        return None

    label = boundary.label or "regular/self"
    s_out, s_return_done = _blocking_s_range_for_overtake(reference_line_info, boundary)
    s_nudge_done = s_out + nudge_ramp
    s_return_begin = s_return_done - return_ramp

    frenet_points: List[FrenetFramePoint] = []
    for pt in boundary:
        s = pt.s
        pass_l = _pass_l_from_boundary_point(pt, label)
        if s < s_out:
            l = 0.0
        elif s < s_nudge_done:
            l = pass_l * _smoothstep01((s - s_out) / max(nudge_ramp, 1e-3))
        elif s < s_return_begin:
            l = pass_l
        elif s < s_return_done:
            l = pass_l * (1.0 - _smoothstep01((s - s_return_begin) / max(return_ramp, 1e-3)))
        else:
            l = 0.0
        frenet_points.append(FrenetFramePoint(s=s, l=l, dl=0.0, ddl=0.0))

    if not frenet_points:
        return None

    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    path_data.set_path_label(label)
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
