from common.PathPoint import PathPoint
import math
from typing import List, Tuple

class PathMatcher:
    """
    PathMatcher class
    Note that this class should not be instantiated. All methods should be called in a static context.
    """

    @staticmethod
    def NormalizeAngle(angle: float) -> float:
        """
        Normalize_Angle

        :param float angle: angle
        :returns: normalized angle
        :rtype: float
        """

        a: float = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += 2.0 * math.pi
        return a - math.pi

    @staticmethod
    def slerp(a0: float, t0: float, a1:float, t1: float, t: float, epsilon:float=1e-10) -> float:
        """
        Slerp function
        
        :param float a0: a0
        :param float t0: t0
        :param float a1: a1
        :param float t1: t1
        :param float t: t
        :param float epsilon: epsilon, default 1e-10
        :returns: slerp result
        :rtype: float
        """

        if abs(t1 - t0) <= epsilon:
            print("The time difference is too small")
            return PathMatcher.NormalizeAngle(a0)
        a0_n: float = PathMatcher.NormalizeAngle(a0)
        a1_n: float = PathMatcher.NormalizeAngle(a1)
        d: float = a1_n - a0_n
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        r: float = (t - t0) / (t1 - t0)
        a: float = a0_n + d * r
        return PathMatcher.NormalizeAngle(a)

    @staticmethod
    def InterpoliateUsingLinearApproximation(p0: PathPoint, p1: PathPoint, s: float) -> PathPoint:
        """
        Interpoliate_Using_Linear_Approximation

        :param PathPoint p0: path point 0
        :param PathPoint p1: path point 1
        :param float s: s
        :returns: interpolated path point
        :rtype: PathPoint
        """

        s0: float = p0.s
        s1: float = p1.s

        weight: float = (s - s0) / (s1 - s0)
        x: float = (1 - weight) * p0.x + weight * p1.x
        y: float = (1 - weight) * p0.y + weight * p1.y
        theta: float = PathMatcher.slerp(p0.theta, p0.s, p1.theta, p1.s, s)
        kappa: float = (1 - weight) * p0.kappa + weight * p1.kappa
        dkappa: float = (1 - weight) * p0.dkappa + weight * p1.dkappa
        ddkappa: float = (1 - weight) * p0.ddkappa + weight * p1.ddkappa

        return PathPoint(x, y, theta, kappa, dkappa, ddkappa, s)

    @staticmethod
    def FindProjectionPoint(p0: PathPoint, p1: PathPoint, x: float, y: float) -> PathPoint:
        """
        Find_Projection_Point

        :param PathPoint p0: path point 0
        :param PathPoint p1: path point 1
        :param float x: x coordinate
        :param float y: y coordinate
        """

        v0x: float = x - p0.x
        v0y: float = y - p0.y

        v1x: float = p1.x - p0.x
        v1y: float = p1.y - p0.y

        v1_norm: float = math.sqrt(v1x * v1x + v1y * v1y)
        dot: float = v0x * v1x + v0y * v1y
        delta_s: float = dot / v1_norm

        return PathMatcher.InterpoliateUsingLinearApproximation(p0, p1, p0.s + delta_s)

    @staticmethod
    def MatchToPath(reference_line: List[PathPoint], x: float, y: float) -> PathPoint:
        """
        Match_To_Path

        :param List[PathPoint] reference_line: reference line
        :param float x: x coordinate
        :param float y: y coordinate
        :returns: matched point
        :rtype: PathPoint
        """

        if len(reference_line) == 0:
            raise ValueError("The reference line is empty")
        
        def func_distance_square(point, x, y) -> float:
            return (point.x - x) ** 2 + (point.y - y) ** 2

        distance_min: float = func_distance_square(reference_line.front(), x, y)
        index_min: int = 0

        for i in range(1, len(reference_line)):
            distance_temp = func_distance_square(reference_line[i], x, y)
            if distance_temp < distance_min:
                distance_min = distance_temp
                index_min = i

        index_start = index_min if index_min == 0 else index_min - 1
        index_end = index_min if index_min + 1 == len(reference_line) else index_min + 1
        if index_start == index_end:
            return reference_line[index_start]

        return PathMatcher.FindProjectionPoint(reference_line[index_start], reference_line[index_end], x, y)

    @staticmethod
    def GetPathFrenetCoordinate(reference_line: List[PathPoint], x: float, y: float) -> Tuple[float, float]:
        """
        Get_Path_Frenet_Coordinate

        :param List[PathPoint] reference_line: reference line
        :param float x: x coordinate
        :param float y: y coordinate
        :returns: frenet coordinate
        :rtype: Tuple[float, float]
        """

        matched_path_point: PathPoint = PathMatcher.MatchToPath(reference_line, x, y)
        rtheta: float = matched_path_point.theta
        rx: float = matched_path_point.x
        ry: float = matched_path_point.y
        delta_x: float = x - rx
        delta_y: float = y - ry
        side: float = math.cos(rtheta) * delta_y - math.sin(rtheta) * delta_x
        relative_coordinate: Tuple[float, float] = (matched_path_point.s, 
                                                math.copysign(math.hypot(delta_x, delta_y), side))
        return relative_coordinate
