"""QP spline reference line smoother aligned with qp_spline_reference_line_smoother.cc."""

from __future__ import annotations

import math
from typing import List, Optional

from reference_line.discrete_points_reference_line_smoother import AnchorPoint, DiscretePointsReferenceLineSmoother
from common.map_path_point import MapPathPoint
from reference_line import ReferenceLine
from reference_line.reference_point import ReferencePoint
from common.vec2d import Vec2d
import config as config_module
from planning_math.curve_math import ComputeCurvature, ComputeCurvatureDerivative
from planning_math.smoothing_spline.osqp_spline_2d_solver import OsqpSpline2dSolver


class QpSplineReferenceLineSmoother:
    """Reference line smoother using 2D QP splines."""

    def __init__(self):
        self._anchor_points: List[AnchorPoint] = []
        self._t_knots: List[float] = []
        self._ref_x = 0.0
        self._ref_y = 0.0
        self._fallback = DiscretePointsReferenceLineSmoother()
        self._solver = OsqpSpline2dSolver([], config_module.FLAGS_qp_spline_order)

    def SetAnchorPoints(self, anchor_points: List[AnchorPoint]) -> None:
        if len(anchor_points) < 2:
            raise ValueError("anchor_points must contain at least two points")
        self._anchor_points = list(anchor_points)

    def Smooth(self, raw_reference_line: ReferenceLine) -> Optional[ReferenceLine]:
        if not self._anchor_points:
            return ReferenceLine(raw_reference_line)

        self._t_knots.clear()
        if not self._sampling():
            return None
        self._solver.reset(self._t_knots, config_module.FLAGS_qp_spline_order)
        if not self._add_constraint():
            return None
        if not self._add_kernel():
            return None
        if not self._solver.solve():
            self._fallback.SetAnchorPoints(self._anchor_points)
            return self._fallback.Smooth(raw_reference_line)

        start_t = self._t_knots[0]
        end_t = self._t_knots[-1]
        resolution = (end_t - start_t) / max(1, config_module.FLAGS_num_of_total_reference_points - 1)
        spline = self._solver.spline
        ref_points: List[ReferencePoint] = []
        t = start_t
        for _ in range(config_module.FLAGS_num_of_total_reference_points):
            if t >= end_t:
                break
            heading = math.atan2(spline.derivative_y(t), spline.derivative_x(t))
            kappa = ComputeCurvature(
                spline.derivative_x(t),
                spline.second_derivative_x(t),
                spline.derivative_y(t),
                spline.second_derivative_y(t),
            )
            dkappa = ComputeCurvatureDerivative(
                spline.derivative_x(t),
                spline.second_derivative_x(t),
                spline.third_derivative_x(t),
                spline.derivative_y(t),
                spline.second_derivative_y(t),
                spline.third_derivative_y(t),
            )
            x, y = spline(t)
            x += self._ref_x
            y += self._ref_y
            ok, sl = raw_reference_line.XYToSL(Vec2d(x, y))
            if not ok or sl is None:
                t += resolution
                continue
            if sl.s < -1e-6 or sl.s > raw_reference_line.Length():
                t += resolution
                continue
            sl.s = max(0.0, sl.s)
            raw_point = raw_reference_line.GetReferencePoint(sl.s)
            lane_waypoints = raw_point.lane_waypoints
            for lane_waypoint in lane_waypoints:
                lane_waypoint.l = sl.l
            ref_points.append(
                ReferencePoint(
                    MapPathPoint(Vec2d(x, y), heading, lane_waypoints),
                    kappa,
                    dkappa,
                )
            )
            t += resolution

        if len(ref_points) < 2:
            return None
        ReferencePoint.RemoveDuplicates(ref_points)
        if len(ref_points) < 2:
            return None
        return ReferenceLine(ref_points)

    def _sampling(self) -> bool:
        length = self._anchor_points[-1].path_point.s - self._anchor_points[0].path_point.s
        num_spline = max(1, int(length / config_module.FLAGS_qp_spline_max_spline_length + 0.5))
        self._t_knots = [float(i) for i in range(num_spline + 1)]
        self._ref_x = self._anchor_points[0].path_point.x
        self._ref_y = self._anchor_points[0].path_point.y
        return True

    def _add_constraint(self) -> bool:
        headings: List[float] = []
        longitudinal_bound: List[float] = []
        lateral_bound: List[float] = []
        xy_points: List[Vec2d] = []
        for point in self._anchor_points:
            path_point = point.path_point
            headings.append(path_point.theta)
            longitudinal_bound.append(point.longitudinal_bound)
            lateral_bound.append(point.lateral_bound)
            xy_points.append(Vec2d(path_point.x - self._ref_x, path_point.y - self._ref_y))

        scale = (
            (self._anchor_points[-1].path_point.s - self._anchor_points[0].path_point.s)
            / max(1e-6, self._t_knots[-1] - self._t_knots[0])
        )
        evaluated_t = [point.path_point.s / scale for point in self._anchor_points]
        constraint = self._solver.mutable_constraint
        if not constraint.add_2d_boundary(
            evaluated_t, headings, xy_points, longitudinal_bound, lateral_bound
        ):
            return False
        if config_module.FLAGS_enable_reference_line_stitching and not constraint.add_point_angle_constraint(
            evaluated_t[0], headings[0]
        ):
            return False
        return constraint.add_second_derivative_smooth_constraint()

    def _add_kernel(self) -> bool:
        kernel = self._solver.mutable_kernel
        if config_module.FLAGS_qp_spline_second_derivative_weight > 0.0:
            kernel.add_second_order_derivative_matrix(config_module.FLAGS_qp_spline_second_derivative_weight)
        if config_module.FLAGS_qp_spline_third_derivative_weight > 0.0:
            kernel.add_third_order_derivative_matrix(config_module.FLAGS_qp_spline_third_derivative_weight)
        kernel.add_regularization(config_module.FLAGS_qp_spline_regularization_weight)
        return True
