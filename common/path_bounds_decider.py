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
        self.adc_frenet_ld = 0.0
        self.adc_l_to_lane_center = 0.0
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
            self.adc_frenet_ld = l_condition[1] * s_condition[1]
        else:
            adc_sl = reference_line_info.AdcSlBoundary()
            self.adc_frenet_s = adc_sl.start_s
            self.adc_frenet_l = 0.5 * (adc_sl.start_l + adc_sl.end_l)
            self.adc_frenet_ld = 0.0

        _, offset_to_map = reference_line_info.reference_line.GetOffsetToMap(
            self.adc_frenet_s
        )
        self.adc_l_to_lane_center = self.adc_frenet_l + offset_to_map

        ok, left, right = reference_line_info.reference_line.GetLaneWidth(self.adc_frenet_s)
        self.adc_lane_width = (left + right) if ok else config_module.FLAGS_default_lane_width

    def _lane_borrow_infos(
        self,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext],
    ) -> List[LaneBorrowInfo]:
        infos = [LaneBorrowInfo.NO_BORROW]
        if not reference_line_info.is_path_lane_borrow() or planning_context is None:
            return infos

        decided_directions = list(
            planning_context.planning_status.path_decider.decided_side_pass_direction
            or []
        )
        for direction in decided_directions:
            if direction == 1 and LaneBorrowInfo.LEFT_BORROW not in infos:
                infos.append(LaneBorrowInfo.LEFT_BORROW)
            elif direction == 2 and LaneBorrowInfo.RIGHT_BORROW not in infos:
                infos.append(LaneBorrowInfo.RIGHT_BORROW)
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
        while s < end_s:
            path_bound.append((s, float("-inf"), float("inf")))
            s += K_PATH_BOUNDS_DECIDER_RESOLUTION
        return path_bound

    def _generate_fallback_path_bound(
        self, reference_line_info: ReferenceLineInfo
    ) -> PathBound:
        path_bound = self._init_path_boundary(reference_line_info)
        if not path_bound:
            return []
        ok, _ = self._get_boundary_from_lanes_and_adc(
            reference_line_info, LaneBorrowInfo.NO_BORROW, 0.5, path_bound, True
        )
        if not ok:
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
        ok, borrow_lane_type = self._get_boundary_from_lanes_and_adc(
            reference_line_info, borrow_info, 0.1, path_bound, False
        )
        if not ok:
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
        return path_bound, blocking_id, borrow_lane_type

    def _get_boundary_from_lanes_and_adc(
        self,
        reference_line_info: ReferenceLineInfo,
        borrow_info: LaneBorrowInfo,
        adc_buffer: float,
        path_bound: PathBound,
        is_fallback: bool,
    ) -> Tuple[bool, str]:
        reference_line = reference_line_info.reference_line
        past_lane_left = self.adc_lane_width / 2.0
        past_lane_right = self.adc_lane_width / 2.0
        borrowing_reverse_lane = False

        for i, (s, l_min, l_max) in enumerate(path_bound):
            ok, lane_left, lane_right = reference_line.GetLaneWidth(s)
            if not ok:
                lane_left = past_lane_left
                lane_right = past_lane_right
            else:
                _, lane_center_offset = reference_line.GetOffsetToMap(s)
                lane_left += lane_center_offset
                lane_right -= lane_center_offset
                past_lane_left = lane_left
                past_lane_right = lane_right

            neighbor_width = 0.0
            if self._check_lane_boundary_type(
                reference_line_info, s, borrow_info
            ):
                if borrow_info == LaneBorrowInfo.LEFT_BORROW:
                    found, _, neighbor_width = reference_line_info.GetNeighborLaneInfo(
                        ReferenceLineInfo.LaneType.LeftForward, s
                    )
                    if not found:
                        found, _, neighbor_width = reference_line_info.GetNeighborLaneInfo(
                            ReferenceLineInfo.LaneType.LeftReverse, s
                        )
                        borrowing_reverse_lane = borrowing_reverse_lane or found
                    if not found:
                        neighbor_width = 0.0
                elif borrow_info == LaneBorrowInfo.RIGHT_BORROW:
                    found, _, neighbor_width = reference_line_info.GetNeighborLaneInfo(
                        ReferenceLineInfo.LaneType.RightForward, s
                    )
                    if not found:
                        found, _, neighbor_width = reference_line_info.GetNeighborLaneInfo(
                            ReferenceLineInfo.LaneType.RightReverse, s
                        )
                        borrowing_reverse_lane = borrowing_reverse_lane or found
                    if not found:
                        neighbor_width = 0.0

            curr_left_lane = lane_left + (
                neighbor_width if borrow_info == LaneBorrowInfo.LEFT_BORROW else 0.0
            )
            curr_right_lane = -lane_right - (
                neighbor_width if borrow_info == LaneBorrowInfo.RIGHT_BORROW else 0.0
            )
            _, offset_to_map = reference_line.GetOffsetToMap(s)

            if (
                config_module.FLAGS_path_bounds_decider_extend_lane_bounds_to_include_adc
                or is_fallback
            ):
                speed_buffer = (
                    (1.0 if self.adc_frenet_ld > 0.0 else -1.0)
                    * self.adc_frenet_ld
                    * self.adc_frenet_ld
                    / 3.0
                )
                adc_left = (
                    max(
                        self.adc_l_to_lane_center,
                        self.adc_l_to_lane_center + speed_buffer,
                    )
                    + config_module.FLAGS_half_vehicle_width
                    + adc_buffer
                )
                adc_right = (
                    min(
                        self.adc_l_to_lane_center,
                        self.adc_l_to_lane_center + speed_buffer,
                    )
                    - config_module.FLAGS_half_vehicle_width
                    - adc_buffer
                )
                curr_left = max(curr_left_lane, adc_left) - offset_to_map
                curr_right = min(curr_right_lane, adc_right) - offset_to_map
            else:
                curr_left = curr_left_lane - offset_to_map
                curr_right = curr_right_lane - offset_to_map

            coeff = config_module.FLAGS_path_bounds_decider_adc_buffer_coeff
            new_l_min = max(
                l_min, curr_right + coeff * config_module.FLAGS_half_vehicle_width
            )
            new_l_max = min(
                l_max, curr_left - coeff * config_module.FLAGS_half_vehicle_width
            )
            if new_l_min > new_l_max:
                del path_bound[i:]
                break
            path_bound[i] = (s, new_l_min, new_l_max)
        borrow_lane_type = "reverse" if borrowing_reverse_lane else "forward"
        return True, borrow_lane_type

    @staticmethod
    def _check_lane_boundary_type(
        reference_line_info: ReferenceLineInfo,
        check_s: float,
        borrow_info: LaneBorrowInfo,
    ) -> bool:
        if borrow_info == LaneBorrowInfo.NO_BORROW:
            return False
        left_type, right_type = reference_line_info.reference_line.GetLaneBoundaryType(
            check_s
        )
        boundary_type = (
            left_type
            if borrow_info == LaneBorrowInfo.LEFT_BORROW
            else right_type
        )
        from protoclass.lane import LaneBoundaryType

        return boundary_type not in {
            LaneBoundaryType.LaneBoundaryTypeEnum.SOLID_YELLOW,
            LaneBoundaryType.LaneBoundaryTypeEnum.SOLID_WHITE,
        }

    def _get_boundary_from_static_obstacles(
        self,
        reference_line_info: ReferenceLineInfo,
        path_bound: PathBound,
        blocking_id_holder: List[str],
    ) -> bool:
        edges = []
        for obstacle in reference_line_info.path_decision.obstacles.values():
            if not IsWithinPathDeciderScopeObstacle(obstacle):
                continue
            sl = obstacle.PerceptionSLBoundary()
            if sl.end_s < self.adc_frenet_s:
                continue
            l_min = sl.start_l - config_module.FLAGS_obstacle_lat_buffer
            l_max = sl.end_l + config_module.FLAGS_obstacle_lat_buffer
            edges.append(
                (
                    1,
                    sl.start_s - config_module.FLAGS_obstacle_lon_start_buffer,
                    l_min,
                    l_max,
                    obstacle.Id(),
                )
            )
            edges.append(
                (
                    0,
                    sl.end_s + config_module.FLAGS_obstacle_lon_end_buffer,
                    l_min,
                    l_max,
                    obstacle.Id(),
                )
            )
        edges.sort(key=lambda edge: (edge[1], -edge[0]))

        center_line = self.adc_frenet_l
        edge_index = 0
        active = {}
        for i in range(1, len(path_bound)):
            s, l_min, l_max = path_bound[i]
            entering_id = ""
            while edge_index < len(edges) and edges[edge_index][1] < s:
                is_start, _, obs_l_min, obs_l_max, obstacle_id = edges[edge_index]
                if is_start:
                    entering_id = obstacle_id
                    pass_left = obs_l_min + obs_l_max < center_line * 2.0
                    active[obstacle_id] = (pass_left, obs_l_min, obs_l_max)
                else:
                    active.pop(obstacle_id, None)
                edge_index += 1

            right_bound = max(
                (details[2] for details in active.values() if details[0]),
                default=float("-inf"),
            )
            left_bound = min(
                (details[1] for details in active.values() if not details[0]),
                default=float("inf"),
            )
            new_l_min = max(
                l_min, right_bound + config_module.FLAGS_half_vehicle_width
            )
            new_l_max = min(
                l_max, left_bound - config_module.FLAGS_half_vehicle_width
            )
            if new_l_min > new_l_max:
                if entering_id:
                    blocking_id_holder[0] = entering_id
                elif active:
                    blocking_id_holder[0] = next(iter(active))
                del path_bound[i:]
                break
            path_bound[i] = (s, new_l_min, new_l_max)
            center_line = 0.5 * (new_l_min + new_l_max)
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
