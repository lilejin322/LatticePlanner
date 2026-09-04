#!/usr/bin/env python3
"""Run a minimal Apollo-style lattice planning smoke test."""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from lattice_planner import LatticePlanner
from common.frame import Frame
from common.hd_map import HDMap, HDMapUtil
from common.path import Path as MapPath
from reference_line import ReferenceLine
from reference_line.reference_line_info import ReferenceLineInfo
from common.route_segments import RouteSegments
from common.lane_types import LaneSegment
from protoclass.adc_trajectory import ADCTrajectory
from protoclass.lane import Curve, CurveSegment, Lane, LaneSampleAssociation, LineSegment
from protoclass.path_point import PathPoint
from protoclass.point_enu import PointENU
from protoclass.trajectory_point import TrajectoryPoint
from protoclass.vehicle_state import VehicleState
import config as config_module


def build_reference_line(length: float = 100.0) -> tuple[ReferenceLine, RouteSegments]:
    lane = Lane(
        id=Lane.Id("minimal_lane"),
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
    hdmap = HDMap()
    lane_info = hdmap.AddLane(lane)
    HDMapUtil.SetBaseMap(hdmap)

    route_segments = RouteSegments()
    route_segments.SetIsOnSegment(True)
    route_segments.SetId("minimal_mock")
    route_segments.append(LaneSegment(lane_info, 0.0, length - 1.0))

    return ReferenceLine(MapPath(route_segments)), route_segments


def main() -> int:
    reference_line, route_segments = build_reference_line()

    planning_start_point = TrajectoryPoint(
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
        v=0.1,
        a=0.0,
        relative_time=0.0,
    )

    vehicle_state = VehicleState()
    vehicle_state.x = 0.0
    vehicle_state.y = 0.0
    vehicle_state.heading = 0.0

    reference_line_info = ReferenceLineInfo(
        vehicle_state, planning_start_point, reference_line, route_segments
    )

    frame = Frame(0)
    frame._reference_line_info = [reference_line_info]
    frame._obstacles = {}
    if not reference_line_info.Init([], config_module.FLAGS_default_cruise_speed):
        print("failed to initialize reference line info", file=sys.stderr)
        return 1

    ok = LatticePlanner().Plan(planning_start_point, frame, ADCTrajectory())
    if not ok:
        print("minimal lattice plan failed", file=sys.stderr)
        return 1

    if config_module.FLAGS_enable_lattice_path_assessment:
        assert reference_line_info.path_data is not None
        debug_paths = reference_line_info.debug.planning_data.path
        if debug_paths:
            assert any(p.name == "Planning PathData" for p in debug_paths)

    print("minimal lattice plan succeeded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
