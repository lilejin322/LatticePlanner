"""
Discrete-points reference line smoother aligned with Apollo.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from common.fem_pos_deviation_smoother import FemPosDeviationSmoother
from reference_line import ReferenceLine
from reference_line.reference_point import ReferencePoint
from common.map_path_point import MapPathPoint
from common.vec2d import Vec2d
from protoclass.path_point import PathPoint
from protoclass.sl_boundary import SLPoint
import config as config_module


@dataclass
class AnchorPoint:
    path_point: PathPoint
    lateral_bound: float = 0.0
    longitudinal_bound: float = 0.0
    enforced: bool = False


class DiscretePointsReferenceLineSmoother:
    def __init__(self):
        self._anchor_points: List[AnchorPoint] = []
        self._solver = FemPosDeviationSmoother(
            weight_fem_pos_deviation=config_module.FLAGS_fem_pos_weight_deviation,
            weight_ref_deviation=config_module.FLAGS_fem_pos_weight_ref_deviation,
            weight_path_length=config_module.FLAGS_fem_pos_weight_path_length,
        )
        self._ref_x = 0.0
        self._ref_y = 0.0

    def SetAnchorPoints(self, anchor_points: List[AnchorPoint]) -> None:
        self._anchor_points = list(anchor_points)

    @staticmethod
    def _normalize_points(points: List[Tuple[float, float]]) -> Tuple[List[Tuple[float, float]], float, float]:
        if not points:
            return points, 0.0, 0.0
        ref_x, ref_y = points[0]
        normalized = [(x - ref_x, y - ref_y) for x, y in points]
        return normalized, ref_x, ref_y

    @staticmethod
    def _compute_heading_and_kappa(
        xs: List[float], ys: List[float]
    ) -> Tuple[List[float], List[float], List[float]]:
        headings, kappas, dkappas = [], [], []
        n = len(xs)
        for i in range(n):
            if i == 0:
                dx = xs[1] - xs[0]
                dy = ys[1] - ys[0]
            elif i == n - 1:
                dx = xs[i] - xs[i - 1]
                dy = ys[i] - ys[i - 1]
            else:
                dx = xs[i + 1] - xs[i - 1]
                dy = ys[i + 1] - ys[i - 1]
            headings.append(math.atan2(dy, dx))

        for i in range(n):
            if i == 0 or i == n - 1:
                kappas.append(0.0)
                dkappas.append(0.0)
                continue
            x1, y1 = xs[i - 1], ys[i - 1]
            x2, y2 = xs[i], ys[i]
            x3, y3 = xs[i + 1], ys[i + 1]
            dx1, dy1 = x2 - x1, y2 - y1
            dx2, dy2 = x3 - x2, y3 - y2
            cross = dx1 * dy2 - dy1 * dx2
            dot = dx1 * dx2 + dy1 * dy2
            ds = math.hypot(dx1, dy1) + math.hypot(dx2, dy2)
            kappas.append(2.0 * cross / max(ds * ds, 1e-6))
            dkappas.append(0.0)
        return headings, kappas, dkappas

    def Smooth(self, raw_reference_line: ReferenceLine) -> Optional[ReferenceLine]:
        if not self._anchor_points:
            return ReferenceLine(raw_reference_line)

        raw_point2d = [(ap.path_point.x, ap.path_point.y) for ap in self._anchor_points]
        box_ratio = 1.0 / math.sqrt(2.0)
        bounds = [ap.lateral_bound * box_ratio for ap in self._anchor_points]
        bounds[0] = 0.0
        bounds[-1] = 0.0

        normalized, self._ref_x, self._ref_y = self._normalize_points(raw_point2d)
        solved, opt_x, opt_y = self._solver.Solve(normalized, bounds)
        if not solved:
            return None
        opt_x = [x + self._ref_x for x in opt_x]
        opt_y = [y + self._ref_y for y in opt_y]

        headings, kappas, dkappas = self._compute_heading_and_kappa(opt_x, opt_y)
        ref_points: List[ReferencePoint] = []
        for i, (x, y) in enumerate(zip(opt_x, opt_y)):
            ok, sl = raw_reference_line.XYToSL(Vec2d(x, y))
            if not ok or sl is None:
                continue
            if sl.s < -1e-6 or sl.s > raw_reference_line.Length():
                continue
            sl.s = max(0.0, sl.s)
            raw_point = raw_reference_line.GetReferencePoint(sl.s)
            lane_waypoints = raw_point.lane_waypoints
            for lane_waypoint in lane_waypoints:
                lane_waypoint.l = sl.l
            ref_points.append(
                ReferencePoint(
                    MapPathPoint(Vec2d(x, y), headings[i], lane_waypoints),
                    kappas[i],
                    dkappas[i],
                )
            )

        if len(ref_points) < 2:
            return None
        ReferencePoint.RemoveDuplicates(ref_points)
        if len(ref_points) < 2:
            return None
        return ReferenceLine(ref_points)
