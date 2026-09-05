"""Shared fixtures for lattice / planning smoke tests."""

from __future__ import annotations

import math

from common.hd_map import HDMap, HDMapUtil
from common.path import Path as MapPath
from reference_line import ReferenceLine
from reference_line.reference_line_info import ReferenceLineInfo
from common.route_segments import RouteSegments
from common.lane_types import LaneSegment
from lattice_planner import ToDiscretizedReferenceLine
from protoclass.lane import Curve, CurveSegment, Lane, LaneSampleAssociation, LineSegment
from protoclass.point_enu import PointENU
from protoclass.path_point import PathPoint
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState
from protoclass.decision_result import ChangeLaneType


def build_straight_lane(lane_id: str = "lane_0", length: float = 100.0) -> Lane:
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


def curved_arc_pose(s: float, *, radius: float = 60.0) -> tuple[float, float, float]:
    theta = s / radius
    return radius * math.sin(theta), radius * (1.0 - math.cos(theta)), theta


def build_curved_lane(
    lane_id: str = "curve_lane",
    *,
    radius: float = 60.0,
    arc_length: float = 80.0,
    step: float = 1.0,
) -> Lane:
    points = []
    s = 0.0
    while s <= arc_length + 1e-6:
        x, y, _ = curved_arc_pose(s, radius=radius)
        points.append(PointENU(x=x, y=y))
        s += step
    if points[-1].x != points[0].x or len(points) < 2:
        x, y, _ = curved_arc_pose(arc_length, radius=radius)
        if abs(points[-1].x - x) > 1e-6 or abs(points[-1].y - y) > 1e-6:
            points.append(PointENU(x=x, y=y))

    return Lane(
        id=Lane.Id(lane_id),
        central_curve=Curve(
            segment=[
                CurveSegment(curve_type=LineSegment(point=points))
            ]
        ),
        length=arc_length,
        speed_limit=8.0,
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        left_road_sample=[LaneSampleAssociation(s=0.0, width=3.0)],
        right_road_sample=[LaneSampleAssociation(s=0.0, width=3.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )


def build_reference_line(
    length: float = 100.0,
    *,
    start_x: float = 0.0,
    start_y: float = 0.0,
    start_heading: float = 0.0,
    init_v: float = 1.0,
):
    hdmap = HDMap()
    lane_info = hdmap.AddLane(build_straight_lane(length=length))
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("mock")
    route_segments.append(LaneSegment(lane_info, 0.0, length - 1.0))

    reference_line = ReferenceLine(MapPath(route_segments))
    discretized_ref_points = ToDiscretizedReferenceLine(reference_line.reference_points)

    start_point = TrajectoryPoint(
        path_point=PathPoint(
            x=start_x,
            y=start_y,
            z=0.0,
            theta=start_heading,
            kappa=0.0,
            s=0.0,
            dkappa=0.0,
            ddkappa=0.0,
        ),
        v=init_v,
        a=0.0,
        relative_time=0.0,
    )

    vehicle_state = VehicleState()
    vehicle_state.x = start_x
    vehicle_state.y = start_y
    vehicle_state.heading = start_heading
    reference_line_info = ReferenceLineInfo(
        vehicle_state, start_point, reference_line, route_segments
    )
    return reference_line, discretized_ref_points, reference_line_info


def build_curved_reference_line(
    *,
    radius: float = 60.0,
    arc_length: float = 80.0,
    init_v: float = 2.0,
):
    hdmap = HDMap()
    lane_info = hdmap.AddLane(build_curved_lane(radius=radius, arc_length=arc_length))
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("curve_lane")
    route_segments.append(LaneSegment(lane_info, 0.0, arc_length - 1.0))

    reference_line = ReferenceLine(MapPath(route_segments))
    start_x, start_y, start_heading = curved_arc_pose(0.0, radius=radius)
    start_point = TrajectoryPoint(
        path_point=PathPoint(
            x=start_x,
            y=start_y,
            z=0.0,
            theta=start_heading,
            kappa=1.0 / radius,
            s=0.0,
            dkappa=0.0,
            ddkappa=0.0,
        ),
        v=init_v,
        a=0.0,
        relative_time=0.0,
    )

    vehicle_state = VehicleState()
    vehicle_state.x = start_x
    vehicle_state.y = start_y
    vehicle_state.heading = start_heading
    reference_line_info = ReferenceLineInfo(
        vehicle_state, start_point, reference_line, route_segments
    )
    return reference_line, ToDiscretizedReferenceLine(reference_line.reference_points), reference_line_info


def build_static_obstacle(
    obs_id: str,
    x: float,
    y: float = 0.0,
    *,
    length: float = 4.0,
    width: float = 2.0,
):
    from common.obstacle import Obstacle
    from protoclass.adc_trajectory import Point3D
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    perception = PerceptionObstacle(
        id=int(obs_id.split("_")[-1]) if obs_id.split("_")[-1].isdigit() else 1,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=x, y=y, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=length,
        width=width,
        height=1.5,
        theta=0.0,
    )
    return Obstacle(obs_id, perception, is_static=True)


def build_curved_static_obstacle(
    obs_id: str,
    s: float,
    *,
    radius: float = 60.0,
    length: float = 4.0,
    width: float = 2.0,
):
    from common.obstacle import Obstacle
    from protoclass.adc_trajectory import Point3D
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType

    x, y, theta = curved_arc_pose(s, radius=radius)
    perception = PerceptionObstacle(
        id=int(obs_id.split("_")[-1]) if obs_id.split("_")[-1].isdigit() else 1,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=x, y=y, z=0.0),
        velocity=Point3D(x=0.0, y=0.0, z=0.0),
        length=length,
        width=width,
        height=1.5,
        theta=theta,
    )
    return Obstacle(obs_id, perception, is_static=True)


def build_dynamic_obstacle(
    obs_id: str = "dyn_1",
    start_x: float = 15.0,
    start_y: float = 0.0,
    vx: float = 2.0,
    *,
    steps: int = 6,
):
    from common.obstacle import Obstacle
    from protoclass.adc_trajectory import Point3D
    from protoclass.perception_obstacle import PerceptionObstacle, PerceptionObstacleType
    from protoclass.trajectory import Trajectory

    perception = PerceptionObstacle(
        id=1,
        type=PerceptionObstacleType.VEHICLE,
        position=Point3D(x=start_x, y=start_y, z=0.0),
        velocity=Point3D(x=vx, y=0.0, z=0.0),
        length=4.0,
        width=2.0,
        height=1.5,
        theta=0.0,
    )
    trajectory = Trajectory(
        trajectory_point=[
            TrajectoryPoint(
                path_point=PathPoint(
                    x=start_x + vx * t,
                    y=start_y,
                    z=0.0,
                    theta=0.0,
                    kappa=0.0,
                    s=0.0,
                    dkappa=0.0,
                    ddkappa=0.0,
                ),
                v=vx,
                a=0.0,
                relative_time=float(t),
            )
            for t in range(steps)
        ]
    )
    return Obstacle(obs_id, perception, is_static=False, trajectory=trajectory)


def build_lattice_plan_frame(
    obstacles=None,
    *,
    length: float = 100.0,
    init_v: float = 1.0,
    start_x: float = 0.0,
    start_y: float = 0.0,
    start_heading: float = 0.0,
    blocking_obstacle_id: str | None = None,
):
    """
    Build Frame + ReferenceLineInfo ready for LatticePlanner.Plan.

    obstacles: list of Obstacle, or None for empty scene.
    """
    from common.frame import Frame

    _, _, reference_line_info = build_reference_line(
        length=length,
        start_x=start_x,
        start_y=start_y,
        start_heading=start_heading,
        init_v=init_v,
    )

    obstacle_list = list(obstacles or [])
    frame = Frame(0)
    frame._obstacles = {obs.Id(): obs for obs in obstacle_list}
    reference_line_info.Init(obstacle_list, 10.0)
    if blocking_obstacle_id is not None:
        reference_line_info.SetBlockingObstacle(blocking_obstacle_id)
    frame._reference_line_info = [reference_line_info]
    return frame, reference_line_info, reference_line_info._adc_planning_point


def build_curved_lattice_plan_frame(
    obstacles=None,
    *,
    radius: float = 60.0,
    arc_length: float = 80.0,
    init_v: float = 2.0,
    blocking_obstacle_id: str | None = None,
):
    from common.frame import Frame

    _, _, reference_line_info = build_curved_reference_line(
        radius=radius,
        arc_length=arc_length,
        init_v=init_v,
    )
    obstacle_list = list(obstacles or [])
    frame = Frame(0)
    frame._obstacles = {obs.Id(): obs for obs in obstacle_list}
    reference_line_info.Init(obstacle_list, 10.0)
    if blocking_obstacle_id is not None:
        reference_line_info.SetBlockingObstacle(blocking_obstacle_id)
    frame._reference_line_info = [reference_line_info]
    return frame, reference_line_info, reference_line_info._adc_planning_point


def build_parallel_lanes_hdmap(length: float = 100.0):
    """双车道直道：lane_left (y=0) 与 lane_right (y=-3.5)。"""
    left = build_straight_lane("lane_left", length=length)
    left.left_neighbor_forward_lane_id = []
    left.right_neighbor_forward_lane_id = [Lane.Id("lane_right")]

    right = Lane(
        id=Lane.Id("lane_right"),
        central_curve=Curve(
            segment=[
                CurveSegment(
                    curve_type=LineSegment(
                        point=[
                            PointENU(x=0.0, y=-3.5),
                            PointENU(x=length, y=-3.5),
                        ]
                    )
                )
            ]
        ),
        length=length,
        speed_limit=10.0,
        left_neighbor_forward_lane_id=[Lane.Id("lane_left")],
        right_neighbor_forward_lane_id=[],
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )

    hdmap = HDMap()
    left_info = hdmap.AddLane(left)
    hdmap.AddLane(right)
    HDMapUtil.SetBaseMap(hdmap)
    return hdmap, left_info


def build_left_lane_reference_line(length: float = 100.0, init_v: float = 1.0):
    """在左车道（lane_left）上建参考线。"""
    _, left_info = build_parallel_lanes_hdmap(length=length)
    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("lane_left")
    route_segments.append(LaneSegment(left_info, 0.0, length - 1.0))
    reference_line = ReferenceLine(MapPath(route_segments))
    start_point = TrajectoryPoint(
        path_point=PathPoint(x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0),
        v=init_v,
        a=0.0,
        relative_time=0.0,
    )
    vehicle_state = VehicleState()
    vehicle_state.x = 0.0
    vehicle_state.y = 0.0
    vehicle_state.heading = 0.0
    rli = ReferenceLineInfo(vehicle_state, start_point, reference_line, route_segments)
    return reference_line, rli


def build_center_lane_reference_line(length: float = 100.0, init_v: float = 1.0):
    """Build a center-lane reference line with real forward neighbors on both sides."""

    def offset_lane(lane_id: str, y: float) -> Lane:
        lane = build_straight_lane(lane_id, length=length)
        lane.central_curve = Curve(
            segment=[
                CurveSegment(
                    curve_type=LineSegment(
                        point=[PointENU(x=0.0, y=y), PointENU(x=length, y=y)]
                    )
                )
            ]
        )
        return lane

    center = offset_lane("lane_center", 0.0)
    left = offset_lane("lane_left_neighbor", 3.5)
    right = offset_lane("lane_right_neighbor", -3.5)
    center.left_neighbor_forward_lane_id = [Lane.Id("lane_left_neighbor")]
    center.right_neighbor_forward_lane_id = [Lane.Id("lane_right_neighbor")]
    left.right_neighbor_forward_lane_id = [Lane.Id("lane_center")]
    right.left_neighbor_forward_lane_id = [Lane.Id("lane_center")]
    center.left_road_sample = [LaneSampleAssociation(s=0.0, width=5.5)]
    center.right_road_sample = [LaneSampleAssociation(s=0.0, width=5.5)]

    hdmap = HDMap()
    center_info = hdmap.AddLane(center)
    hdmap.AddLane(left)
    hdmap.AddLane(right)
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("lane_center")
    route_segments.append(LaneSegment(center_info, 0.0, length - 1.0))
    reference_line = ReferenceLine(MapPath(route_segments))
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
        v=init_v,
        a=0.0,
        relative_time=0.0,
    )
    vehicle_state = VehicleState(x=0.0, y=0.0, heading=0.0)
    reference_line_info = ReferenceLineInfo(
        vehicle_state, start_point, reference_line, route_segments
    )
    return reference_line, reference_line_info


def build_slow_leader_obstacle(
    obs_id: str,
    x: float,
    *,
    y: float = 0.0,
    vx: float = 0.0,
):
    """前车：默认静止；vx>0 表示慢速行驶（仅用于标注速度，仍按静态参与规划）。"""
    from protoclass.adc_trajectory import Point3D

    obs = build_static_obstacle(obs_id, x, y=y)
    if vx > 0:
        obs._perception_obstacle.velocity = Point3D(x=vx, y=0.0, z=0.0)
    return obs


def _smoothstep01(r: float) -> float:
    r = max(0.0, min(1.0, r))
    return r * r * (3.0 - 2.0 * r)


def build_cartesian_overtake_path_points(
    *,
    length_x: float = 95.0,
    step: float = 0.5,
    obstacle_x: float = 30.0,
    peak_y: float = 3.2,
    nudge_start_before: float = 10.0,
    nudge_ramp: float = 5.0,
    pass_length: float = 10.0,
    return_ramp: float = 5.0,
):
    """在 Cartesian 下直接构造 S 形绕行路径（用于可视化与 Combine）。"""
    from protoclass.path_point import PathPoint

    x_out = obstacle_x - nudge_start_before
    x_nudge_done = x_out + nudge_ramp
    x_return_begin = obstacle_x + pass_length * 0.5
    x_return_done = x_return_begin + return_ramp

    points = []
    x = 0.0
    s_acc = 0.0
    prev_x, prev_y = 0.0, 0.0
    while x <= length_x + 1e-6:
        if x < x_out:
            y = 0.0
        elif x < x_nudge_done:
            y = peak_y * _smoothstep01((x - x_out) / max(nudge_ramp, 1e-3))
        elif x < x_return_begin:
            y = peak_y
        elif x < x_return_done:
            y = peak_y * (1.0 - _smoothstep01((x - x_return_begin) / max(return_ramp, 1e-3)))
        else:
            y = 0.0
        if points:
            dx = x - prev_x
            dy = y - prev_y
            s_acc += (dx * dx + dy * dy) ** 0.5
        theta = 0.0 if x >= length_x - step else __import__("math").atan2(y - prev_y, max(x - prev_x, 1e-3))
        points.append(
            PathPoint(
                x=x,
                y=y,
                z=0.0,
                theta=theta,
                kappa=0.0,
                s=s_acc,
                dkappa=0.0,
                ddkappa=0.0,
            )
        )
        prev_x, prev_y = x, y
        x += step
    return points


def apply_cartesian_overtake_trajectory(
    reference_line_info,
    start_point,
    path_points,
    *,
    cruise_v: float = 6.0,
) -> tuple[bool, str]:
    from common.discretized_path import DiscretizedPath
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.path_data import PathData
    from common.planning_util import BuildCruiseSpeedData
    from protoclass.adc_trajectory import ADCTrajectory

    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    path_data.set_path_label("synthetic/overtake_s_curve")
    dp = DiscretizedPath()
    for p in path_points:
        dp.append(p)
    path_data.SetDiscretizedPath(dp)

    reference_line_info.SetPathData(path_data)
    reference_line_info.SetLatticeCruiseSpeed(cruise_v)
    reference_line_info.SetSpeedData(
        BuildCruiseSpeedData(reference_line_info, cruise_speed=cruise_v)
    )
    trajectory = DiscretizedTrajectory()
    ok = reference_line_info.CombinePathAndSpeedProfile(
        0.0, start_point.path_point.s, trajectory
    )
    if not ok or len(trajectory) == 0:
        return False, "Combine failed"

    reference_line_info.SetTrajectory(trajectory)
    reference_line_info.set_trajectory_type(ADCTrajectory.TrajectoryType.NORMAL)

    ys = [p.path_point.y for p in trajectory]
    xs = [p.path_point.x for p in trajectory]
    near = [abs(p.path_point.y) for p in trajectory if abs(p.path_point.x - 30.0) < 8.0]
    tail = [abs(p.path_point.y) for p in trajectory if p.path_point.x > 45.0]
    return True, (
        f"S-curve end_x={xs[-1]:.1f} max|y|={max(abs(y) for y in ys):.2f} "
        f"peak_near_obs={max(near) if near else 0:.2f} tail|y|={max(tail) if tail else 0:.2f}"
    )


def build_frenet_overtake_path_points(
    start_s: float,
    end_s: float,
    *,
    peak_l: float = 3.2,
    obstacle_s: float = 30.0,
    nudge_start_before: float = 10.0,
    nudge_ramp: float = 5.0,
    pass_length: float = 10.0,
    return_ramp: float = 5.0,
    step: float = 0.5,
):
    """
  构造 Frenet 超车轮廓：本车道 → 横向绕开障碍（S 形）→ 回到本车道。
  obstacle_s 为前车中心在参考线 s 上的大致位置。
    """
    from protoclass.frenet_frame_point import FrenetFramePoint

    s_out = obstacle_s - nudge_start_before
    s_nudge_done = s_out + nudge_ramp
    s_return_begin = obstacle_s + pass_length * 0.5
    s_return_done = s_return_begin + return_ramp

    points = []
    s = start_s
    while s <= end_s + 1e-6:
        if s < s_out:
            l = 0.0
        elif s < s_nudge_done:
            l = peak_l * _smoothstep01((s - s_out) / max(nudge_ramp, 1e-3))
        elif s < s_return_begin:
            l = peak_l
        elif s < s_return_done:
            l = peak_l * (1.0 - _smoothstep01((s - s_return_begin) / max(return_ramp, 1e-3)))
        else:
            l = 0.0
        points.append(FrenetFramePoint(s=s, l=l, dl=0.0, ddl=0.0))
        s += step
    return points


def apply_frenet_overtake_trajectory(
    reference_line_info,
    start_point,
    frenet_points,
    *,
    cruise_v: float = 6.0,
) -> tuple[bool, str]:
    """优先 Frenet→XY；失败则回退为 Cartesian 点列。"""
    from common.frenet_frame_path import FrenetFramePath
    from common.path_data import PathData

    path_data = PathData()
    path_data.SetReferenceLine(reference_line_info.reference_line)
    path_data.set_path_label("synthetic/overtake_s_curve")
    if path_data.SetFrenetPath(FrenetFramePath(frenet_points)):
        reference_line_info.SetPathData(path_data)
        reference_line_info.SetLatticeCruiseSpeed(cruise_v)
        from common.discretized_trajectory import DiscretizedTrajectory
        from common.planning_util import BuildCruiseSpeedData

        reference_line_info.SetSpeedData(
            BuildCruiseSpeedData(reference_line_info, cruise_speed=cruise_v)
        )
        trajectory = DiscretizedTrajectory()
        ok = reference_line_info.CombinePathAndSpeedProfile(
            0.0, start_point.path_point.s, trajectory
        )
        if ok and len(trajectory) > 0:
            reference_line_info.SetTrajectory(trajectory)
            ys = [abs(p.path_point.y) for p in trajectory]
            if max(ys) > 0.5:
                return True, f"frenet_xy max|y|={max(ys):.2f}"

    # Frenet SLToXY 在部分参考线上不可用 → Cartesian S 曲线
    cart_pts = build_cartesian_overtake_path_points(
        obstacle_x=30.0,
        peak_y=3.2,
    )
    return apply_cartesian_overtake_trajectory(
        reference_line_info, start_point, cart_pts, cruise_v=cruise_v
    )


def build_synthetic_overtake_frame(
    leader_x: float = 30.0,
    *,
    init_v: float = 5.0,
    peak_y: float = 3.2,
    length: float = 100.0,
):
    """标准超车动画：前车 + Cartesian S 形绕行（绕开→回归，非全程偏置）。"""
    leader = build_slow_leader_obstacle("slow_leader", leader_x)
    frame, rli, start = build_lattice_plan_frame(
        [leader],
        length=length,
        init_v=init_v,
        blocking_obstacle_id="slow_leader",
    )
    cart_pts = build_cartesian_overtake_path_points(
        length_x=length - 5.0,
        obstacle_x=leader_x,
        peak_y=peak_y,
    )
    ok, detail = apply_cartesian_overtake_trajectory(
        rli, start, cart_pts, cruise_v=max(init_v, 6.0)
    )
    return frame, rli, start, ok, None, detail


def build_lattice_overtake_combined_frame(
    leader_x: float = 30.0,
    *,
    init_v: float = 5.0,
    peak_y: float = 3.2,
):
    """Lattice 纵向往前 + Cartesian S 形路径经 Combine（合成栈一致）。"""
    leader = build_slow_leader_obstacle("slow_leader", leader_x)
    frame, rli, start = build_lattice_plan_frame(
        [leader], init_v=init_v, blocking_obstacle_id="slow_leader"
    )
    ok, detail = apply_cartesian_overtake_trajectory(
        rli,
        start,
        build_cartesian_overtake_path_points(obstacle_x=leader_x, peak_y=peak_y),
        cruise_v=max(init_v, 6.0),
    )
    return frame, rli, start, ok, None, f"lattice_stack+{detail}"


def apply_path_bounds_overtake_trajectory(
    reference_line_info,
    start_point,
    path_label: str = "regular/left/forward",
    *,
    cruise_v: float = 6.0,
    obstacle_x: float = 30.0,
) -> tuple[bool, str]:
    """
    C++ 对齐超车：PathBounds 借道走廊 + S 形 l(s) 剖面 + Combine（非纯 Lattice）。
    """
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.path_bounds_decider import PathBoundsDecider
    from common.planning_context import PlanningContext
    from common.planning_util import (
        BuildCruiseSpeedData,
        BuildOvertakePathDataFromPathBoundary,
    )
    from protoclass.adc_trajectory import ADCTrajectory

    ctx = PlanningContext()
    ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
    ctx.planning_status.path_decider.decided_side_pass_direction = [
        1 if "/left/" in path_label else 2
    ]
    reference_line_info.set_is_path_lane_borrow(True)

    if not PathBoundsDecider().Process(None, reference_line_info, ctx).ok():
        return False, "PathBounds failed"

    boundary = next(
        (
            b
            for b in reference_line_info.GetCandidatePathBoundaries()
            if b.label == path_label
        ),
        None,
    )
    if boundary is None:
        labels = [b.label for b in reference_line_info.GetCandidatePathBoundaries()]
        return False, f"no boundary {path_label!r} in {labels}"

    path_data = BuildOvertakePathDataFromPathBoundary(reference_line_info, boundary)
    if path_data is None or path_data.Empty():
        return False, "BuildOvertakePathData failed"

    reference_line_info.SetPathData(path_data)
    reference_line_info.SetLatticeCruiseSpeed(cruise_v)
    reference_line_info.SetSpeedData(
        BuildCruiseSpeedData(reference_line_info, cruise_speed=cruise_v)
    )
    trajectory = DiscretizedTrajectory()
    ok = reference_line_info.CombinePathAndSpeedProfile(
        0.0, start_point.path_point.s, trajectory
    )
    if not ok or len(trajectory) == 0:
        return False, "Combine failed"

    reference_line_info.SetTrajectory(trajectory)
    reference_line_info.set_trajectory_type(ADCTrajectory.TrajectoryType.NORMAL)

    ys = [abs(p.path_point.y) for p in trajectory]
    near = [
        abs(p.path_point.y)
        for p in trajectory
        if abs(p.path_point.x - obstacle_x) < 8.0
    ]
    tail = [abs(p.path_point.y) for p in trajectory if p.path_point.x > 45.0]
    fp = path_data.frenet_frame_path
    max_l = max(abs(p.l) for p in fp) if fp else 0.0
    return True, (
        f"path_bounds_s_curve label={path_label} max|y|={max(ys):.2f} "
        f"max|l|={max_l:.2f} peak_near={max(near) if near else 0:.2f} "
        f"tail|y|={max(tail) if tail else 0:.2f}"
    )


def build_path_bounds_overtake_frame(
    leader_x: float = 30.0,
    *,
    path_label: str = "regular/left/forward",
    init_v: float = 5.0,
    length: float = 100.0,
    cruise_v: float = 6.0,
):
    """超车场景（推荐）：PathBounds 左借道 + S 形剖面 + Combine。"""
    leader = build_slow_leader_obstacle("slow_leader", leader_x)
    from common.frame import Frame

    _, rli = build_center_lane_reference_line(length=length, init_v=init_v)
    frame = Frame(0)
    frame._obstacles = {leader.Id(): leader}
    rli.Init([leader], 10.0)
    rli.SetBlockingObstacle("slow_leader")
    frame._reference_line_info = [rli]
    start = rli._adc_planning_point
    ok, detail = apply_path_bounds_overtake_trajectory(
        rli, start, path_label, cruise_v=max(cruise_v, init_v), obstacle_x=leader_x
    )
    return frame, rli, start, ok, None, detail


def apply_borrow_path_trajectory(
    reference_line_info,
    start_point,
    path_label: str,
    *,
    lane_borrow: bool = True,
) -> tuple[bool, str]:
    """
    PathBounds 借道候选 + 指定 path_label + Combine 写入 reference_line_info.trajectory。
    用于「超车绕行」类场景动画（不经过 Lattice 1D 搜索）。
    """
    from common.discretized_trajectory import DiscretizedTrajectory
    from common.path_bounds_decider import PathBoundsDecider, BuildCandidatePathsFromBoundaries
    from common.planning_context import PlanningContext
    from common.planning_util import BuildCruiseSpeedData

    ctx = PlanningContext()
    if lane_borrow:
        ctx.planning_status.path_decider.is_in_path_lane_borrow_scenario = True
        ctx.planning_status.path_decider.decided_side_pass_direction = [
            1 if "/left/" in path_label else 2
        ]
        reference_line_info.set_is_path_lane_borrow(True)

    if not PathBoundsDecider().Process(None, reference_line_info, ctx).ok():
        return False, "PathBounds failed"

    candidates = BuildCandidatePathsFromBoundaries(reference_line_info)
    path_data = next((c for c in candidates if c.path_label == path_label), None)
    if path_data is None:
        labels = [c.path_label for c in candidates]
        return False, f"no path {path_label!r} in {labels}"

    reference_line_info.SetPathData(path_data)
    reference_line_info.SetLatticeCruiseSpeed(8.0)
    reference_line_info.SetSpeedData(BuildCruiseSpeedData(reference_line_info))
    trajectory = DiscretizedTrajectory()
    ok = reference_line_info.CombinePathAndSpeedProfile(
        0.0, start_point.path_point.s, trajectory
    )
    if not ok or len(trajectory) == 0:
        return False, "CombinePathAndSpeedProfile failed"

    reference_line_info.SetTrajectory(trajectory)
    max_y = max(abs(p.path_point.y) for p in trajectory)
    end_x = trajectory[-1].path_point.x
    fp = path_data.frenet_frame_path
    max_l = max(abs(p.l) for p in fp) if fp else 0.0
    return True, (
        f"path={path_label} end_x={end_x:.1f} max|y|={max_y:.2f} max|l|={max_l:.2f}"
    )


def build_overtake_lattice_frame(
    slow_x: float,
    *,
    init_v: float = 5.0,
    slow_y: float = 0.0,
    length: float = 100.0,
):
    """Lattice 超车尝试：本车道前车（多为跟停，非借道超车）。"""
    leader = build_slow_leader_obstacle("slow_leader", slow_x, y=slow_y)
    return build_lattice_plan_frame(
        [leader],
        length=length,
        init_v=init_v,
        blocking_obstacle_id="slow_leader",
    )


def build_overtake_borrow_frame(
    slow_x: float,
    path_label: str,
    *,
    init_v: float = 5.0,
    length: float = 100.0,
):
    """借道绕行轨迹（left/right），已写入 trajectory，供动画直接播放。"""
    from common.frame import Frame

    leader = build_slow_leader_obstacle("slow_leader", slow_x)
    frame, rli, start = build_lattice_plan_frame(
        [leader],
        length=length,
        init_v=init_v,
        blocking_obstacle_id="slow_leader",
    )
    ok, detail = apply_borrow_path_trajectory(rli, start, path_label)
    return frame, rli, start, ok, None, detail


def build_lane_change_curve(
    length: float,
    y_final: float,
    transition_length: float,
    *,
    step: float = 1.0,
) -> list:
    """从 y=0 平滑过渡到 y=y_final 的换道曲线点序列（升余弦过渡，两端切线水平，
    避免与直线车道拼接处出现曲率突变）。"""
    points = []
    x = 0.0
    while x <= length + 1e-6:
        if x <= transition_length:
            y = y_final * 0.5 * (1.0 - math.cos(math.pi * x / transition_length))
        else:
            y = y_final
        points.append(PointENU(x=x, y=y))
        x += step
    return points


def build_lane_change_frame(
    *,
    length: float = 100.0,
    ego_v: float = 10.0,
    npc_start_x: float = 20.0,
    npc_v: float = 3.0,
    npc_steps: int = 16,
    transition_length: float = 30.0,
):
    """
    双车道换道超车场景：本车道（lane_left, y=0）前方有一辆匀速慢速 NPC，
    目标车道（lane_right）用一条从 y=0 平滑过渡到 y=-3.5 的"换道曲线"表示
    （模拟真实换道参考线，而非简单的车道宽度侧移——直接侧移会让 init_d 过大，
    超出 lattice 1D 采样器的横向端点范围 [-0.5, 0, 0.5]，导致规划失败）。

    返回 (frame, target_reference_line_info, start_point)。
    frame.mutable_reference_line_info == [current_rli, target_rli]，
    交给 LatticePlanner.Plan() 后可用 frame.FindDriveReferenceLineInfo()
    取得成本更低的那条（预期是 target_rli）。
    """
    from common.frame import Frame

    y_target = -3.5

    left = build_straight_lane("lane_left", length=length)
    left.right_neighbor_forward_lane_id = [Lane.Id("lane_right")]

    right = Lane(
        id=Lane.Id("lane_right"),
        central_curve=Curve(
            segment=[
                CurveSegment(
                    curve_type=LineSegment(
                        point=build_lane_change_curve(length, y_target, transition_length)
                    )
                )
            ]
        ),
        length=length,
        speed_limit=10.0,
        left_neighbor_forward_lane_id=[Lane.Id("lane_left")],
        right_neighbor_forward_lane_id=[],
        left_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        right_sample=[LaneSampleAssociation(s=0.0, width=2.0)],
        type=Lane.LaneType.CITY_DRIVING,
    )

    hdmap = HDMap()
    left_info = hdmap.AddLane(left)
    right_info = hdmap.AddLane(right)
    HDMapUtil.SetBaseMap(hdmap)

    def _build_rli(lane_info, lane_id: str, on_segment: bool, previous_action=None):
        route_segments = RouteSegments()
        route_segments.SetIsOnSegment(on_segment)
        route_segments.SetId(lane_id)
        if previous_action is not None:
            route_segments.SetPreviousAction(previous_action)
        route_segments.append(LaneSegment(lane_info, 0.0, length - 1.0))
        reference_line = ReferenceLine(MapPath(route_segments))
        start_point = TrajectoryPoint(
            path_point=PathPoint(
                x=0.0, y=0.0, z=0.0, theta=0.0, kappa=0.0, s=0.0, dkappa=0.0, ddkappa=0.0
            ),
            v=ego_v,
            a=0.0,
            relative_time=0.0,
        )
        vehicle_state = VehicleState()
        vehicle_state.x = 0.0
        vehicle_state.y = 0.0
        vehicle_state.heading = 0.0
        rli = ReferenceLineInfo(vehicle_state, start_point, reference_line, route_segments)
        return rli, start_point

    current_rli, start_point = _build_rli(left_info, "lane_left", True)
    target_rli, _ = _build_rli(right_info, "lane_right", False, ChangeLaneType.RIGHT)

    npc = build_dynamic_obstacle(
        "npc_slow", start_x=npc_start_x, start_y=0.0, vx=npc_v, steps=npc_steps
    )
    obstacle_list = [npc]

    current_rli.Init(obstacle_list, 10.0)
    target_rli.Init(obstacle_list, 10.0)

    frame = Frame(0)
    frame._obstacles = {npc.Id(): npc}
    frame._reference_line_info = [current_rli, target_rli]

    return frame, target_rli, start_point
