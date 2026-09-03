"""Path bounds decider aligned with path_bounds_decider.cc (lattice port)."""

from __future__ import annotations

from enum import Enum
from typing import List, Optional, Tuple

from common.frame import Frame
from common.path_boundary import PathBoundPoint, PathBoundary
from common.path_data import PathData
from common.planning_context import PlanningContext
from reference_line.reference_line_info import ReferenceLineInfo
from common.status import Status
from common.path_decider_obstacle_utils import IsWithinPathDeciderScopeObstacle
from common.planning_debug import RecordPathBoundaryDebugInfo
import config as config_module
from protoclass.header import ErrorCode

K_PATH_BOUNDS_DECIDER_HORIZON = 100.0
K_PATH_BOUNDS_DECIDER_RESOLUTION = 0.5
K_NUM_EXTRA_TAIL_BOUND_POINT = 2


class LaneBorrowInfo(Enum):
    NO_BORROW = 0
    LEFT_BORROW = 1
    RIGHT_BORROW = 2


PathBound = List[Tuple[float, float, float]]


class PathBoundsDecider:
    def __init__(self):
        self.adc_frenet_s = 0.0
        self.adc_frenet_l = 0.0
        self.adc_lane_width = config_module.FLAGS_default_lane_width

    def Process(
        self,
        frame: Frame,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext] = None,
    ) -> Status:
        if config_module.FLAGS_enable_skip_path_tasks and reference_line_info.path_reusable:
            return Status.OK()

        self._init_state(frame, reference_line_info)
        candidate_boundaries: List[PathBoundary] = []

        fallback_bound = self._generate_fallback_path_bound(reference_line_info)
        if not fallback_bound:
            return Status(ErrorCode.PLANNING_ERROR, "Failed to get fallback path boundary")
        candidate_boundaries.append(
            self._to_path_boundary(fallback_bound, "fallback")
        )
        RecordPathBoundaryDebugInfo(
            candidate_boundaries[-1], "path_boundary/fallback", reference_line_info
        )

        borrow_infos = self._lane_borrow_infos(reference_line_info, planning_context)
        for borrow_info in borrow_infos:
            regular_bound, blocking_id, borrow_lane_type = self._generate_regular_path_bound(
                reference_line_info, borrow_info
            )
            if not regular_bound:
                continue
            label = self._path_label(borrow_info, borrow_lane_type)
            boundary = self._to_path_boundary(regular_bound, label)
            boundary.set_blocking_obstacle_id(blocking_id)
            candidate_boundaries.append(boundary)
            RecordPathBoundaryDebugInfo(
                boundary, f"path_boundary/{label.replace('/', '_')}", reference_line_info
            )

        reference_line_info.SetCandidatePathBoundaries(candidate_boundaries)
        return Status.OK()

    def _init_state(
        self,
        frame: Optional[Frame],
        reference_line_info: ReferenceLineInfo,
    ) -> None:
        planning_start = None
        if frame is not None:
            planning_start = getattr(frame, "_planning_start_point", None)
        if planning_start is None:
            planning_start = getattr(reference_line_info, "_adc_planning_point", None)
        if planning_start is not None:
            s_condition, l_condition = reference_line_info.reference_line.ToFrenetFrame(
                planning_start
            )
            self.adc_frenet_s = s_condition[0]
            self.adc_frenet_l = l_condition[0]
        else:
            adc_sl = reference_line_info.AdcSlBoundary()
            self.adc_frenet_s = adc_sl.start_s
            self.adc_frenet_l = 0.5 * (adc_sl.start_l + adc_sl.end_l)

        ok, left, right = reference_line_info.reference_line.GetLaneWidth(self.adc_frenet_s)
        self.adc_lane_width = (left + right) if ok else config_module.FLAGS_default_lane_width

    def _lane_borrow_infos(
        self,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext],
    ) -> List[LaneBorrowInfo]:
        infos = [LaneBorrowInfo.NO_BORROW]
        lane_borrow = False
        if planning_context is not None:
            status = planning_context.planning_status.path_decider
            lane_borrow = bool(status.is_in_path_lane_borrow_scenario)
        if reference_line_info.GetBlockingObstacle() is not None:
            lane_borrow = True
        if lane_borrow or reference_line_info.is_path_lane_borrow():
            infos.extend([LaneBorrowInfo.LEFT_BORROW, LaneBorrowInfo.RIGHT_BORROW])
        return infos

    @staticmethod
    def _path_label(borrow_info: LaneBorrowInfo, borrow_lane_type: str) -> str:
        if borrow_info == LaneBorrowInfo.LEFT_BORROW:
            side = "left"
        elif borrow_info == LaneBorrowInfo.RIGHT_BORROW:
            side = "right"
        else:
            side = "self"
        lane_type = borrow_lane_type or "forward"
        return f"regular/{side}/{lane_type}"

    def _init_path_boundary(self, reference_line_info: ReferenceLineInfo) -> PathBound:
        reference_line = reference_line_info.reference_line
        horizon = max(
            K_PATH_BOUNDS_DECIDER_HORIZON,
            reference_line_info.GetCruiseSpeed() * config_module.FLAGS_trajectory_time_length,
        )
        end_s = min(self.adc_frenet_s + horizon, reference_line.Length())
        path_bound: PathBound = []
        s = self.adc_frenet_s
        while s <= end_s + 1e-6:
            path_bound.append((s, float("-inf"), float("inf")))
            s += K_PATH_BOUNDS_DECIDER_RESOLUTION
        return path_bound

    def _generate_fallback_path_bound(
        self, reference_line_info: ReferenceLineInfo
    ) -> PathBound:
        path_bound = self._init_path_boundary(reference_line_info)
        if not path_bound:
            return []
        borrow_lane_type = "forward"
        if not self._get_boundary_from_lanes_and_adc(
            reference_line_info, LaneBorrowInfo.NO_BORROW, 0.5, path_bound, True
        ):
            return []
        return path_bound

    def _generate_regular_path_bound(
        self,
        reference_line_info: ReferenceLineInfo,
        borrow_info: LaneBorrowInfo,
    ) -> Tuple[PathBound, str, str]:
        path_bound = self._init_path_boundary(reference_line_info)
        if not path_bound:
            return [], "", ""
        if not self._get_boundary_from_lanes_and_adc(
            reference_line_info, borrow_info, 0.1, path_bound, False
        ):
            return [], "", ""
        blocking_id_holder = [""]
        temp_bound = list(path_bound)
        if not self._get_boundary_from_static_obstacles(
            reference_line_info, path_bound, blocking_id_holder
        ):
            return [], "", ""
        blocking_id = blocking_id_holder[0]
        counter = 0
        while blocking_id and len(path_bound) < len(temp_bound) and counter < K_NUM_EXTRA_TAIL_BOUND_POINT:
            path_bound.append(temp_bound[len(path_bound)])
            counter += 1
        borrow_lane_type = "forward"
        if borrow_info == LaneBorrowInfo.LEFT_BORROW:
            borrow_lane_type = "forward"
        elif borrow_info == LaneBorrowInfo.RIGHT_BORROW:
            borrow_lane_type = "forward"
        return path_bound, blocking_id, borrow_lane_type

    def _get_boundary_from_lanes_and_adc(
        self,
        reference_line_info: ReferenceLineInfo,
        borrow_info: LaneBorrowInfo,
        adc_buffer: float,
        path_bound: PathBound,
        is_fallback: bool,
    ) -> bool:
        del is_fallback
        reference_line = reference_line_info.reference_line
        half_width = config_module.FLAGS_half_vehicle_width + adc_buffer

        for i, (s, l_min, l_max) in enumerate(path_bound):
            ok, lane_left, lane_right = reference_line.GetLaneWidth(s)
            if not ok:
                lane_left = self.adc_lane_width / 2.0
                lane_right = self.adc_lane_width / 2.0

            curr_left = lane_left
            curr_right = -lane_right
            if borrow_info == LaneBorrowInfo.LEFT_BORROW:
                curr_left += config_module.FLAGS_default_lane_width
            elif borrow_info == LaneBorrowInfo.RIGHT_BORROW:
                curr_right -= config_module.FLAGS_default_lane_width

            curr_left = max(curr_left, self.adc_frenet_l + half_width)
            curr_right = min(curr_right, self.adc_frenet_l - half_width)
            if curr_left <= curr_right:
                return False
            path_bound[i] = (s, curr_right, curr_left)
        return True

    def _get_boundary_from_static_obstacles(
        self,
        reference_line_info: ReferenceLineInfo,
        path_bound: PathBound,
        blocking_id_holder: List[str],
    ) -> bool:
        buffer = config_module.FLAGS_path_decider_static_obstacle_buffer
        half_width = config_module.FLAGS_half_vehicle_width
        best_block_s = float("inf")

        for i, (s, l_min, l_max) in enumerate(path_bound):
            for obstacle in reference_line_info.path_decision.obstacles.values():
                if not IsWithinPathDeciderScopeObstacle(obstacle):
                    continue
                sl = obstacle.PerceptionSLBoundary()
                if sl.end_s < s or sl.start_s > s + K_PATH_BOUNDS_DECIDER_RESOLUTION:
                    continue
                obs_l_min = sl.start_l - buffer
                obs_l_max = sl.end_l + buffer
                path_center = 0.5 * (l_min + l_max)
                if obs_l_max < path_center:
                    new_right = max(l_min, obs_l_max + half_width)
                    l_min = min(l_min, new_right)
                elif obs_l_min > path_center:
                    new_left = min(l_max, obs_l_min - half_width)
                    l_max = max(l_max, new_left)
                else:
                    if sl.start_s < best_block_s:
                        best_block_s = sl.start_s
                        blocking_id_holder[0] = obstacle.Id()
                    l_max = min(l_max, obs_l_min - half_width)
                    l_min = max(l_min, obs_l_max + half_width)
                if l_min >= l_max:
                    path_bound[:] = path_bound[: i + 1]
                    return True
                path_bound[i] = (s, l_min, l_max)
        return True

    @staticmethod
    def _to_path_boundary(path_bound: PathBound, label: str) -> PathBoundary:
        boundary = PathBoundary([], K_PATH_BOUNDS_DECIDER_RESOLUTION)
        for s, l_min, l_max in path_bound:
            boundary.append(PathBoundPoint(l_min, l_max, s))
        boundary.set_label(label)
        return boundary


def BuildCandidatePathsFromBoundaries(
    reference_line_info: ReferenceLineInfo,
) -> List[PathData]:
    from common.planning_util import BuildPathDataFromPathBoundary

    candidates: List[PathData] = []
    for idx, boundary in enumerate(reference_line_info.GetCandidatePathBoundaries()):
        if boundary.label == "fallback":
            continue
        path_data = BuildPathDataFromPathBoundary(reference_line_info, boundary)
        if path_data is None or path_data.Empty():
            continue
        path_data.set_lattice_candidate_id(-1000 - idx)
        blocking_id = boundary.blocking_obstacle_id
        if blocking_id:
            path_data.set_blocking_obstacle_id(blocking_id)
        candidates.append(path_data)
    return candidates
