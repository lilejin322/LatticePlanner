"""Path assessment decider aligned with path_assessment_decider.cc."""

from __future__ import annotations

import math
from copy import deepcopy
from functools import cmp_to_key
from typing import List, Optional, Tuple

from common.box2d import Box2d
from common.frame import Frame
from common.path_data import PathData, PathPointType
from common.planning_context import PlanningContext
from reference_line.reference_line_info import ReferenceLineInfo
from common.status import Status
from common.vec2d import Vec2d
from common.planning_debug import RecordPathDataDebugInfo
from common.path_decider_obstacle_utils import IsWithinPathDeciderScopeObstacle
import config as config_module
from protoclass.header import ErrorCode
from protoclass.sl_boundary import SLBoundary

PathPointDecision = Tuple[float, PathPointType, float]

K_MIN_OBSTACLE_AREA = 1e-4
K_OFF_REFERENCE_LINE_THRESHOLD = 20.0
K_OFF_ROAD_THRESHOLD = 10.0
K_SELF_PATH_LENGTH_COMPARISON_TOLERANCE = 15.0
K_NEIGHBOR_PATH_LENGTH_COMPARISON_TOLERANCE = 25.0
K_BACK_TO_SELF_LANE_COMPARISON_TOLERANCE = 20.0
K_PATH_BOUNDS_DECIDER_RESOLUTION = 0.5
K_NUM_EXTRA_TAIL_BOUND_POINT = 2


def _path_l_at_s(path_data: PathData, s: float) -> Optional[float]:
    frenet_path = path_data.frenet_frame_path
    if not frenet_path:
        return None
    if s <= frenet_path[0].s:
        return frenet_path[0].l
    for point in frenet_path:
        if point.s >= s:
            return point.l
    return frenet_path[-1].l


def FindBlockingObstacleId(
    reference_line_info: ReferenceLineInfo,
    path_data: Optional[PathData] = None,
) -> str:
    path_data = path_data or reference_line_info.path_data
    if path_data is None or not path_data.frenet_frame_path:
        return ""

    adc_end_s = reference_line_info.AdcSlBoundary().end_s
    half_width = config_module.FLAGS_half_vehicle_width
    static_buffer = config_module.FLAGS_path_decider_static_obstacle_buffer
    best_id = ""
    best_s = float("inf")

    for obstacle in reference_line_info.path_decision.obstacles.values():
        if not obstacle.IsStatic() or obstacle.IsVirtual():
            continue
        if (
            obstacle.HasLongitudinalDecision()
            and obstacle.LongitudinalDecision().stop is not None
        ):
            continue
        sl = obstacle.PerceptionSLBoundary()
        if sl.end_s < adc_end_s:
            continue
        path_l = _path_l_at_s(path_data, sl.start_s)
        if path_l is None:
            continue
        if sl.start_l > path_l + half_width + static_buffer:
            continue
        if sl.end_l < path_l - half_width - static_buffer:
            continue
        if sl.start_s < best_s:
            best_s = sl.start_s
            best_id = obstacle.Id()
    return best_id


def _is_within_path_decider_scope(obstacle) -> bool:
    return IsWithinPathDeciderScopeObstacle(obstacle)


def _ego_center_box_at_path_point(path_point) -> Box2d:
    ego_length = config_module.EGO_VEHICLE_LENGTH
    ego_width = config_module.EGO_VEHICLE_WIDTH
    ego_back_to_center = config_module.EGO_BACK_EDGE_TO_CENTER
    ego_center_shift = ego_length / 2.0 - ego_back_to_center
    ego_theta = path_point.theta
    ego_box = Box2d(
        Vec2d(path_point.x, path_point.y),
        ego_theta,
        ego_length,
        ego_width,
    )
    shift_vec = Vec2d(
        ego_center_shift * math.cos(ego_theta),
        ego_center_shift * math.sin(ego_theta),
    )
    ego_box.Shift(shift_vec)
    return ego_box


def ContainsOutOnReverseLane(path_point_decision: List[PathPointDecision]) -> int:
    return sum(
        1
        for _, point_type, _ in path_point_decision
        if point_type == PathPointType.OUT_ON_REVERSE_LANE
    )


def GetBackToInLaneIndex(path_point_decision: List[PathPointDecision]) -> int:
    for i in range(len(path_point_decision) - 1, -1, -1):
        if path_point_decision[i][1] != PathPointType.IN_LANE:
            return i
    return 0


def ComparePathData(
    lhs: PathData,
    rhs: PathData,
    blocking_obstacle=None,
) -> bool:
    """Return True if lhs should be ranked before rhs (lhs is better)."""
    if lhs.Empty():
        return False
    if rhs.Empty():
        return True

    lhs_is_regular = "regular" in lhs.path_label
    rhs_is_regular = "regular" in rhs.path_label
    if lhs_is_regular != rhs_is_regular:
        return lhs_is_regular

    lhs_on_selflane = "self" in lhs.path_label
    rhs_on_selflane = "self" in rhs.path_label
    lhs_path_length = lhs.frenet_frame_path[-1].s
    rhs_path_length = rhs.frenet_frame_path[-1].s
    if lhs_on_selflane or rhs_on_selflane:
        if abs(lhs_path_length - rhs_path_length) > K_SELF_PATH_LENGTH_COMPARISON_TOLERANCE:
            return lhs_path_length > rhs_path_length
        return lhs_on_selflane

    if abs(lhs_path_length - rhs_path_length) > K_NEIGHBOR_PATH_LENGTH_COMPARISON_TOLERANCE:
        return lhs_path_length > rhs_path_length

    lhs_on_reverse = ContainsOutOnReverseLane(lhs.path_point_decision_guide)
    rhs_on_reverse = ContainsOutOnReverseLane(rhs.path_point_decision_guide)
    if abs(lhs_on_reverse - rhs_on_reverse) > 6:
        return lhs_on_reverse < rhs_on_reverse

    if (
        ("left" in lhs.path_label and "right" in rhs.path_label)
        or ("right" in lhs.path_label and "left" in rhs.path_label)
    ):
        if blocking_obstacle is not None:
            sl = blocking_obstacle.PerceptionSLBoundary()
            obstacle_l = (sl.start_l + sl.end_l) / 2.0
            if obstacle_l > 0.0:
                return "right" in lhs.path_label
            return "left" in lhs.path_label
        adc_l = lhs.frenet_frame_path[0].l
        if adc_l < -1.0:
            return "right" in lhs.path_label
        if adc_l > 1.0:
            return "left" in lhs.path_label

    lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide)
    rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide)
    lhs_back_s = lhs.frenet_frame_path[lhs_back_idx].s
    rhs_back_s = rhs.frenet_frame_path[rhs_back_idx].s
    if abs(lhs_back_s - rhs_back_s) > K_BACK_TO_SELF_LANE_COMPARISON_TOLERANCE:
        return lhs_back_idx < rhs_back_idx

    lhs_on_leftlane = "left" in lhs.path_label
    rhs_on_leftlane = "left" in rhs.path_label
    if lhs_on_leftlane != rhs_on_leftlane:
        return lhs_on_leftlane
    return False


def IsGreatlyOffReferenceLine(path_data: PathData) -> bool:
    for point in path_data.frenet_frame_path:
        if abs(point.l) > K_OFF_REFERENCE_LINE_THRESHOLD:
            return True
    return False


def IsGreatlyOffRoad(reference_line_info: ReferenceLineInfo, path_data: PathData) -> bool:
    reference_line = reference_line_info.reference_line
    for point in path_data.frenet_frame_path:
        ok, road_left_width, road_right_width = reference_line.GetRoadWidth(point.s)
        if not ok:
            continue
        if point.l > road_left_width + K_OFF_ROAD_THRESHOLD:
            return True
        if point.l < -road_right_width - K_OFF_ROAD_THRESHOLD:
            return True
    return False


def IsCollidingWithStaticObstacles(
    reference_line_info: ReferenceLineInfo,
    path_data: PathData,
) -> bool:
    if not path_data.discretized_path or not path_data.frenet_frame_path:
        return False

    obstacle_boxes = []
    for obstacle in reference_line_info.path_decision.obstacles.values():
        if not _is_within_path_decider_scope(obstacle):
            continue
        obstacle_box = obstacle.PerceptionBoundingBox()
        if obstacle_box is None or obstacle_box.area < K_MIN_OBSTACLE_AREA:
            continue
        obstacle_boxes.append(obstacle_box)

    tail_s = path_data.frenet_frame_path[-1].s
    half_width = config_module.FLAGS_half_vehicle_width
    for i, path_point in enumerate(path_data.discretized_path):
        if tail_s - path_data.frenet_frame_path[i].s < (
            (K_NUM_EXTRA_TAIL_BOUND_POINT + 1) * K_PATH_BOUNDS_DECIDER_RESOLUTION
        ):
            break
        ego_box = _ego_center_box_at_path_point(path_point)
        for obstacle_box in obstacle_boxes:
            if ego_box.HasOverlap(obstacle_box):
                return True

        frenet_point = path_data.frenet_frame_path[i]
        path_l = frenet_point.l
        for obstacle in reference_line_info.path_decision.obstacles.values():
            if not _is_within_path_decider_scope(obstacle):
                continue
            sl = obstacle.PerceptionSLBoundary()
            if sl.start_l > path_l + half_width:
                continue
            if sl.end_l < path_l - half_width:
                continue
            if sl.start_s <= frenet_point.s <= sl.end_s:
                return True
    return False


def IsStopOnReverseNeighborLane(
    reference_line_info: ReferenceLineInfo,
    path_data: PathData,
) -> bool:
    if "left" not in path_data.path_label and "right" not in path_data.path_label:
        return False

    all_stop_point_sl = reference_line_info.GetAllStopDecisionSLPoint()
    if not all_stop_point_sl:
        return False

    check_s = 0.0
    adc_end_s = reference_line_info.AdcSlBoundary().end_s
    for stop_point_sl in all_stop_point_sl:
        if stop_point_sl.s - adc_end_s < 5.0:
            continue
        check_s = stop_point_sl.s
        break
    if check_s <= 0.0:
        return False

    ok, lane_left_width, lane_right_width = reference_line_info.reference_line.GetLaneWidth(check_s)
    if not ok:
        return False

    path_point_l = None
    for point in path_data.frenet_frame_path:
        if abs(point.s - check_s) < 0.3:
            path_point_l = point.l
    if path_point_l is None:
        return False

    if "left" in path_data.path_label and path_point_l > lane_left_width:
        ok_neighbor, _, _ = reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo.LaneType.LeftReverse, check_s
        )
        return ok_neighbor
    if "right" in path_data.path_label and path_point_l < -lane_right_width:
        ok_neighbor, _, _ = reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo.LaneType.RightReverse, check_s
        )
        return ok_neighbor
    return False


def IsValidRegularPath(reference_line_info: ReferenceLineInfo, path_data: PathData) -> bool:
    if path_data.Empty():
        return False
    if IsGreatlyOffReferenceLine(path_data):
        return False
    if IsGreatlyOffRoad(reference_line_info, path_data):
        return False
    if IsCollidingWithStaticObstacles(reference_line_info, path_data):
        return False
    if IsStopOnReverseNeighborLane(reference_line_info, path_data):
        return False
    return True


def IsValidFallbackPath(reference_line_info: ReferenceLineInfo, path_data: PathData) -> bool:
    if path_data.Empty():
        return False
    if IsGreatlyOffReferenceLine(path_data):
        return False
    if IsGreatlyOffRoad(reference_line_info, path_data):
        return False
    return True


def InitPathPointDecision(path_data: PathData) -> List[PathPointDecision]:
    return [
        (point.s, PathPointType.UNKNOWN, float("inf"))
        for point in path_data.frenet_frame_path
    ]


def SetPathPointType(
    reference_line_info: ReferenceLineInfo,
    path_data: PathData,
    is_lane_change_path: bool,
    path_point_decision: List[PathPointDecision],
) -> None:
    ego_length = config_module.EGO_VEHICLE_LENGTH
    ego_width = config_module.EGO_VEHICLE_WIDTH
    ego_back_to_center = config_module.EGO_BACK_EDGE_TO_CENTER
    ego_center_shift = ego_length / 2.0 - ego_back_to_center
    is_prev_point_out_lane = False

    for i, rear_center_path_point in enumerate(path_data.discretized_path):
        ego_theta = rear_center_path_point.theta
        ego_box = Box2d(
            Vec2d(rear_center_path_point.x, rear_center_path_point.y),
            ego_theta,
            ego_length,
            ego_width,
        )
        shift_vec = Vec2d(
            ego_center_shift * math.cos(ego_theta),
            ego_center_shift * math.sin(ego_theta),
        )
        ego_box.Shift(shift_vec)
        ego_sl_boundary = SLBoundary()
        if not reference_line_info.reference_line.GetSLBoundary(ego_box, ego_sl_boundary):
            continue

        middle_s = (ego_sl_boundary.start_s + ego_sl_boundary.end_s) / 2.0
        ok, lane_left_width, lane_right_width = reference_line_info.reference_line.GetLaneWidth(
            middle_s
        )
        if not ok:
            continue

        back_to_inlane_extra_buffer = 0.2
        in_and_out_lane_hysteresis_buffer = 0.2 if is_prev_point_out_lane else 0.0

        if is_lane_change_path:
            if (
                ego_sl_boundary.start_l > lane_left_width
                or ego_sl_boundary.end_l < -lane_right_width
            ):
                path_point_decision[i] = (
                    path_point_decision[i][0],
                    PathPointType.IN_LANE,
                    path_point_decision[i][2],
                )
            elif (
                ego_sl_boundary.start_l > -lane_right_width + back_to_inlane_extra_buffer
                and ego_sl_boundary.end_l < lane_left_width - back_to_inlane_extra_buffer
            ):
                path_point_decision[i] = (
                    path_point_decision[i][0],
                    PathPointType.IN_LANE,
                    path_point_decision[i][2],
                )
            else:
                path_point_decision[i] = (
                    path_point_decision[i][0],
                    PathPointType.OUT_ON_FORWARD_LANE,
                    path_point_decision[i][2],
                )
        else:
            if (
                ego_sl_boundary.end_l
                > lane_left_width + in_and_out_lane_hysteresis_buffer
                or ego_sl_boundary.start_l
                < -lane_right_width - in_and_out_lane_hysteresis_buffer
            ):
                if "reverse" in path_data.path_label:
                    point_type = PathPointType.OUT_ON_REVERSE_LANE
                elif "forward" in path_data.path_label:
                    point_type = PathPointType.OUT_ON_FORWARD_LANE
                else:
                    point_type = PathPointType.UNKNOWN
                path_point_decision[i] = (
                    path_point_decision[i][0],
                    point_type,
                    path_point_decision[i][2],
                )
                if not is_prev_point_out_lane and (
                    ego_sl_boundary.end_l > lane_left_width + back_to_inlane_extra_buffer
                    or ego_sl_boundary.start_l
                    < -lane_right_width - back_to_inlane_extra_buffer
                ):
                    is_prev_point_out_lane = True
            else:
                path_point_decision[i] = (
                    path_point_decision[i][0],
                    PathPointType.IN_LANE,
                    path_point_decision[i][2],
                )
                if is_prev_point_out_lane:
                    is_prev_point_out_lane = False


def SetObstacleDistance(
    reference_line_info: ReferenceLineInfo,
    path_data: PathData,
    path_point_decision: List[PathPointDecision],
) -> None:
    obstacle_polygons = []
    for obstacle in reference_line_info.path_decision.obstacles.values():
        if not _is_within_path_decider_scope(obstacle):
            continue
        polygon = obstacle.PerceptionPolygon()
        if polygon is None or polygon.area < K_MIN_OBSTACLE_AREA:
            continue
        obstacle_polygons.append(polygon)

    for i, path_point in enumerate(path_data.discretized_path):
        vehicle_box = _ego_center_box_at_path_point(path_point)
        min_distance = float("inf")
        for polygon in obstacle_polygons:
            min_distance = min(min_distance, polygon.DistanceTo(vehicle_box))
        path_point_decision[i] = (
            path_point_decision[i][0],
            path_point_decision[i][1],
            min_distance,
        )


def SetPathInfo(reference_line_info: ReferenceLineInfo, path_data: PathData) -> None:
    if not path_data.frenet_frame_path or not path_data.discretized_path:
        return
    path_decision = InitPathPointDecision(path_data)
    if reference_line_info.IsChangeLanePath():
        SetPathPointType(reference_line_info, path_data, True, path_decision)
    elif "fallback" not in path_data.path_label and "self" not in path_data.path_label:
        SetPathPointType(reference_line_info, path_data, False, path_decision)
    path_data.SetPathPointDecisionGuide(path_decision)


def TrimTailingOutLanePoints(path_data: PathData) -> None:
    if "fallback" in path_data.path_label or "self" in path_data.path_label:
        return

    frenet_path = list(path_data.frenet_frame_path)
    path_point_decision = list(path_data.path_point_decision_guide)
    if len(frenet_path) != len(path_point_decision):
        return

    while path_point_decision and path_point_decision[-1][1] != PathPointType.IN_LANE:
        frenet_path.pop()
        path_point_decision.pop()

    from common.frenet_frame_path import FrenetFramePath

    path_data.SetFrenetPath(FrenetFramePath(frenet_path))
    path_data.SetPathPointDecisionGuide(path_point_decision)


def _update_path_decider_status(
    reference_line_info: ReferenceLineInfo,
    planning_context: Optional[PlanningContext],
) -> None:
    if planning_context is None:
        return
    status = planning_context.planning_status.path_decider
    blocking = reference_line_info.GetBlockingObstacle()
    if blocking is not None:
        counter = max(status.front_static_obstacle_cycle_counter or 0, 0)
        status.front_static_obstacle_cycle_counter = min(counter + 1, 10)
        status.front_static_obstacle_id = blocking.Id()
    else:
        counter = min(status.front_static_obstacle_cycle_counter or 0, 0)
        status.front_static_obstacle_cycle_counter = max(counter - 1, -10)

    if "self" in reference_line_info.path_data.path_label:
        able = status.able_to_use_self_lane_counter or 0
        if able < 0:
            able = 0
        status.able_to_use_self_lane_counter = min(able + 1, 10)
    else:
        status.able_to_use_self_lane_counter = 0

    if status.is_in_path_lane_borrow_scenario:
        left_borrow = False
        right_borrow = False
        path_label = reference_line_info.path_data.path_label
        for direction in status.decided_side_pass_direction or []:
            if direction == 1 and "left" in path_label:
                left_borrow = True
            if direction == 2 and "right" in path_label:
                right_borrow = True
        status.decided_side_pass_direction = []
        if right_borrow:
            status.decided_side_pass_direction.append(2)
        if left_borrow:
            status.decided_side_pass_direction.append(1)


class PathAssessmentDecider:
    """Select the best candidate path after path optimization."""

    def Process(
        self,
        frame: Frame,
        reference_line_info: ReferenceLineInfo,
        planning_context: Optional[PlanningContext] = None,
    ) -> Status:
        del frame
        if config_module.FLAGS_enable_skip_path_tasks and reference_line_info.path_reusable:
            return Status.OK()

        candidate_path_data = list(reference_line_info.GetCandidatePathData())
        if not candidate_path_data and reference_line_info.path_data is not None:
            candidate_path_data = [reference_line_info.path_data]

        valid_path_data: List[PathData] = []
        for curr_path_data in candidate_path_data:
            if "fallback" in curr_path_data.path_label:
                if IsValidFallbackPath(reference_line_info, curr_path_data):
                    valid_path_data.append(curr_path_data)
            elif IsValidRegularPath(reference_line_info, curr_path_data):
                valid_path_data.append(curr_path_data)

        blocking_obstacle_on_selflane = None
        compacted: List[PathData] = []
        for curr_path_data in valid_path_data:
            if "fallback" in curr_path_data.path_label:
                if not curr_path_data.Empty():
                    compacted.append(curr_path_data)
                continue

            SetPathInfo(reference_line_info, curr_path_data)
            if "pullover" not in curr_path_data.path_label:
                TrimTailingOutLanePoints(curr_path_data)

            if "self" in curr_path_data.path_label:
                blocking_id = curr_path_data.blocking_obstacle_id()
                if not blocking_id:
                    blocking_id = FindBlockingObstacleId(
                        reference_line_info, curr_path_data
                    )
                    curr_path_data.set_blocking_obstacle_id(blocking_id)
                if blocking_id:
                    blocking_obstacle_on_selflane = reference_line_info.path_decision.Find(
                        blocking_id
                    )

            if not curr_path_data.Empty():
                compacted.append(curr_path_data)

        valid_path_data = compacted
        if not valid_path_data:
            return Status(
                ErrorCode.PLANNING_ERROR,
                "Neither regular nor fallback path is valid.",
            )

        def _compare(lhs: PathData, rhs: PathData) -> int:
            if ComparePathData(lhs, rhs, blocking_obstacle_on_selflane):
                return -1
            if ComparePathData(rhs, lhs, blocking_obstacle_on_selflane):
                return 1
            return 0

        valid_path_data.sort(key=cmp_to_key(_compare))

        best = valid_path_data[0]
        if "fallback" in best.path_label:
            config_module.FLAGS_static_obstacle_nudge_l_buffer = 0.8

        reference_line_info.SetPathData(deepcopy(best))
        if best.blocking_obstacle_id():
            reference_line_info.SetBlockingObstacle(best.blocking_obstacle_id())
        else:
            reference_line_info._blocking_obstacle = None
        reference_line_info.SetCandidatePathData(valid_path_data)
        _update_path_decider_status(reference_line_info, planning_context)
        RecordPathDataDebugInfo(
            reference_line_info.path_data, "Planning PathData", reference_line_info
        )
        for candidate in valid_path_data:
            RecordPathDataDebugInfo(
                candidate,
                f"Candidate/{candidate.path_label}",
                reference_line_info,
            )
        return Status.OK()


def RecordDebugInfo(
    path_data: PathData,
    debug_name: str,
    reference_line_info: ReferenceLineInfo,
) -> None:
    RecordPathDataDebugInfo(path_data, debug_name, reference_line_info)


def ApplyLatticePathAssessment(reference_line_info: ReferenceLineInfo) -> None:
    """Single-path blocking assessment used before lattice sampling."""
    path_data = reference_line_info.path_data
    if path_data is None:
        return
    if not path_data.path_label:
        path_data.set_path_label("regular/self")
    blocking_id = FindBlockingObstacleId(reference_line_info, path_data)
    path_data.set_blocking_obstacle_id(blocking_id)
    if blocking_id:
        reference_line_info.SetBlockingObstacle(blocking_id)
    if path_data.discretized_path:
        SetPathInfo(reference_line_info, path_data)
