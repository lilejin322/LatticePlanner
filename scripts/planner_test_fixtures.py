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


def build_straight_lane(
    lane_id: str = "lane_0",
    length: float = 100.0,
    *,
    sample_step: float | None = None,
) -> Lane:
    if sample_step is None or sample_step <= 0.0:
        points = [PointENU(x=0.0, y=0.0), PointENU(x=length, y=0.0)]
    else:
        points = []
        x = 0.0
        while x <= length + 1e-6:
            points.append(PointENU(x=x, y=0.0))
            x += sample_step
        if not points or points[-1].x < length - 1e-6:
            points.append(PointENU(x=length, y=0.0))
    return Lane(
        id=Lane.Id(lane_id),
        central_curve=Curve(
            segment=[
                CurveSegment(
                    curve_type=LineSegment(point=points)
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
    sample_step: float | None = None,
):
    hdmap = HDMap()
    lane_info = hdmap.AddLane(build_straight_lane(length=length, sample_step=sample_step))
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
    start_x: float | None = None,
    start_y: float | None = None,
    start_heading: float | None = None,
):
    hdmap = HDMap()
    lane_info = hdmap.AddLane(build_curved_lane(radius=radius, arc_length=arc_length))
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("curve_lane")
    route_segments.append(LaneSegment(lane_info, 0.0, arc_length - 1.0))

    reference_line = ReferenceLine(MapPath(route_segments))
    origin_x, origin_y, origin_heading = curved_arc_pose(0.0, radius=radius)
    if start_x is None:
        start_x = origin_x
    if start_y is None:
        start_y = origin_y
    if start_heading is None:
        start_heading = origin_heading
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
    dt: float = 1.0,
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
    points = []
    for i in range(steps):
        t = i * dt
        points.append(
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
        )
    trajectory = Trajectory(trajectory_point=points)
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
    start_x: float | None = None,
    start_y: float | None = None,
    start_heading: float | None = None,
):
    from common.frame import Frame

    _, _, reference_line_info = build_curved_reference_line(
        radius=radius,
        arc_length=arc_length,
        init_v=init_v,
        start_x=start_x,
        start_y=start_y,
        start_heading=start_heading,
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
    """Two-lane straight road: lane_left (y=0) and lane_right (y=-3.5)."""
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
    """Build a reference line on the left lane (lane_left)."""
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


def build_center_lane_reference_line(
    length: float = 100.0,
    init_v: float = 1.0,
    *,
    sample_step: float | None = None,
):
    """Build a center-lane reference line with real forward neighbors on both sides."""

    def offset_lane(lane_id: str, y: float) -> Lane:
        lane = build_straight_lane(lane_id, length=length, sample_step=sample_step)
        for point in lane.central_curve.segment[0].curve_type.point:
            point.y = y
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



def apply_path_bounds_overtake_trajectory(
    reference_line_info,
    start_point,
    path_label: str = "regular/left/forward",
    *,
    cruise_v: float = 6.0,
    obstacle_x: float = 30.0,
) -> tuple[bool, str]:
    """
    C++-aligned overtaking: PathBounds lane-borrow corridor + S-curve l(s)
    profile + Combine (not pure Lattice).
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



def apply_borrow_path_trajectory(
    reference_line_info,
    start_point,
    path_label: str,
    *,
    lane_borrow: bool = True,
) -> tuple[bool, str]:
    """
    PathBounds lane-borrow candidate + the given path_label + Combine, written
    into reference_line_info.trajectory.
    Used for "overtaking bypass" style scenario animations (bypasses the
    Lattice 1D search).
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



def build_lane_change_curve(
    length: float,
    y_final: float,
    transition_length: float,
    *,
    step: float = 1.0,
) -> list:
    """Lane-change curve point sequence transitioning smoothly from y=0 to
    y=y_final (raised-cosine transition, horizontal tangent at both ends,
    avoiding a curvature discontinuity at the splice with the straight lane)."""
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
    ego_x: float = 0.0,
    ego_y: float = 0.0,
    ego_heading: float = 0.0,
    npc_start_x: float = 20.0,
    npc_x: float | None = None,
    npc_y: float = 0.0,
    npc_v: float = 3.0,
    npc_steps: int = 81,
    npc_dt: float = 0.1,
    transition_length: float = 30.0,
):
    """
    Two-lane lane-change overtaking scenario: the ego lane (lane_left, y=0)
    has a constant-speed, slow-moving NPC ahead; the target lane (lane_right)
    is represented by a "lane-change curve" transitioning smoothly from y=0
    to y=-3.5 (simulating a realistic lane-change reference line, rather than
    a simple lane-width lateral shift -- a direct shift would make init_d too
    large, exceeding the lattice 1D sampler's lateral endpoint range
    [-0.5, 0, 0.5] and causing planning to fail).

    Returns (frame, target_reference_line_info, start_point).
    frame.mutable_reference_line_info == [current_rli, target_rli]; after
    handing this to LatticePlanner.Plan(), frame.FindDriveReferenceLineInfo()
    can be used to obtain whichever has the lower cost (expected to be
    target_rli).
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
                x=ego_x,
                y=ego_y,
                z=0.0,
                theta=ego_heading,
                kappa=0.0,
                s=0.0,
                dkappa=0.0,
                ddkappa=0.0,
            ),
            v=ego_v,
            a=0.0,
            relative_time=0.0,
        )
        vehicle_state = VehicleState()
        vehicle_state.x = ego_x
        vehicle_state.y = ego_y
        vehicle_state.heading = ego_heading
        rli = ReferenceLineInfo(vehicle_state, start_point, reference_line, route_segments)
        return rli, start_point

    current_rli, start_point = _build_rli(left_info, "lane_left", True)
    target_rli, _ = _build_rli(right_info, "lane_right", False, ChangeLaneType.RIGHT)

    npc = build_dynamic_obstacle(
        "npc_slow",
        start_x=npc_start_x if npc_x is None else npc_x,
        start_y=npc_y,
        vx=npc_v,
        steps=npc_steps,
        dt=npc_dt,
    )
    obstacle_list = [npc]

    current_rli.Init(obstacle_list, 10.0)
    target_rli.Init(obstacle_list, 10.0)

    frame = Frame(0)
    frame._obstacles = {npc.Id(): npc}
    frame._reference_line_info = [current_rli, target_rli]

    return frame, target_rli, start_point
