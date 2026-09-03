"""Planning debug helpers aligned with RecordDebugInfo in Apollo deciders."""

from __future__ import annotations

from copy import deepcopy

from common.path_boundary import PathBoundary
from common.path_data import PathData
from reference_line.reference_line_info import ReferenceLineInfo
from protoclass.debug import Debug, PlanningData
from protoclass.path_point import Path, PathPoint


def ensure_planning_data(reference_line_info: ReferenceLineInfo) -> PlanningData:
    if reference_line_info.debug.planning_data is None:
        reference_line_info.debug.planning_data = PlanningData()
    return reference_line_info.debug.planning_data


def RecordPathDataDebugInfo(
    path_data: PathData,
    debug_name: str,
    reference_line_info: ReferenceLineInfo,
) -> None:
    planning_data = ensure_planning_data(reference_line_info)
    points = [
        PathPoint(
            x=p.x,
            y=p.y,
            z=p.z,
            theta=p.theta,
            kappa=p.kappa,
            dkappa=p.dkappa,
            ddkappa=p.ddkappa,
            s=p.s,
        )
        for p in path_data.discretized_path
    ]
    planning_data.path.append(Path(name=debug_name, path_point=points))


def RecordPathBoundaryDebugInfo(
    path_boundary: PathBoundary,
    debug_name: str,
    reference_line_info: ReferenceLineInfo,
) -> None:
    if not path_boundary:
        return

    left_path = PathData()
    left_path.SetReferenceLine(reference_line_info.reference_line)
    right_path = PathData()
    right_path.SetReferenceLine(reference_line_info.reference_line)

    from common.frenet_frame_path import FrenetFramePath
    from protoclass.frenet_frame_point import FrenetFramePoint

    left_frenet = FrenetFramePath(
        [
            FrenetFramePoint(s=pt.s, l=pt.l_upper.l, dl=0.0, ddl=0.0)
            for pt in path_boundary
        ]
    )
    right_frenet = FrenetFramePath(
        [
            FrenetFramePoint(s=pt.s, l=pt.l_lower.l, dl=0.0, ddl=0.0)
            for pt in path_boundary
        ]
    )
    left_path.SetFrenetPath(left_frenet)
    right_path.SetFrenetPath(right_frenet)

    RecordPathDataDebugInfo(left_path, f"{debug_name}/left", reference_line_info)
    RecordPathDataDebugInfo(right_path, f"{debug_name}/right", reference_line_info)
