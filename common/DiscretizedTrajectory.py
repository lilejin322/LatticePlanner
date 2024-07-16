from common.ADCTrajectory import ADCTrajectory
from protoclass.TrajectoryPoint import TrajectoryPoint
from bisect import bisect_left
from logging import Logger
import math
from protoclass.PathPoint import PathPoint
from common.Vec2d import Vec2d

class DiscretizedTrajectory(list):
    """
    A class to represent a series of discretized trajectory points
    List[TrajectoryPoint]
    """
    
    def __init__(self, *args):
        """
        Constructor
        """

        super().__init__()
        self.is_reversed = False
        self.logger = Logger("DiscretizedTrajectory")
        assert len(args) == 1, "Invalid input"
        if isinstance(args[0], ADCTrajectory):
            # Create a DiscretizedTrajectory based on protobuf message
            self.extend(args[0].trajectory_point)
        elif isinstance(args[0], list):
            assert args[0], "Input list is empty"
            assert all(isinstance(p, TrajectoryPoint) for p in args[0]), "Input list contains non-TrajectoryPoint object"
            self.extend(args[0])

    def StartPoint(self) -> TrajectoryPoint:
        """
        Get the start point of the trajectory

        :returns: The start point of the trajectory
        :rtype: TrajectoryPoint
        """

        assert self, "Trajectory is empty"
        return self[0]
    
    def GetTemporalLength(self) -> float:
        """
        Get the temporal length of the trajectory

        :returns: The temporal length of the trajectory
        :rtype: float
        """

        if not self:
            return 0.0
        return self[-1].relative_time - self[0].relative_time
    
    def GetSpatialLength(self) -> float:
        """
        Get the spatial length of the trajectory
        
        :returns: The spatial length of the trajectory
        :rtype: float
        """

        if not self:
            return 0.0
        return self[-1].path_point.s - self[0].path_point.s
    
    def Evaluate(self, relative_time: float) -> TrajectoryPoint:
        """
        Evaluate the trajectory at a given relative time

        :param float relative_time: The relative time
        :returns: The trajectory point at the given relative time
        :rtype: TrajectoryPoint
        """
        
        times = [p.relative_time for p in self]
        idx = bisect_left(times, relative_time)

        if idx == 0:
            return self[0]
        elif idx == len(self):
            self.logger.warning(f"Warning: When evaluating trajectory, relative_time({relative_time}) is too large")
            return self[-1]
        else:
            return self.InterpolateUsingLinearApproximation(self[idx - 1], self[idx], relative_time)

    def lerp(self, x0: float, t0: float, x1: float, t1: float, t: float) -> float:
        """
        Linear interpolation.

        :param float x0: the x0 value.
        :param float t0: the t0 value.
        :param float x1: the x1 value.
        :param float t1: the t1 value.
        :param float t: the t value.
        :returns: the interpolated value.
        :rtype: float
        """

        if abs(t1 - t0) <= 1.0e-6:
            self.logger.error("Input time difference is too small")
            return x0
        r = (t - t0) / (t1 - t0)
        x = x0 + r * (x1 - x0)
        return x

    @staticmethod
    def NormalizeAngle(angle: float) -> float:
        """
        Normalize the angle to [-pi, pi]

        :param float angle: the angle to normalize
        :returns: the normalized angle
        :rtype: float
        """

        a: float = math.fmod(angle + math.pi, 2 * math.pi)
        if a < 0.0:
            a += 2 * math.pi
        return a - math.pi

    def slerp(self, a0: float, t0: float, a1:float, t1: float, t: float, epsilon:float=1e-10) -> float:
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
            self.logger.warning("The time difference is too small")
            return self.NormalizeAngle(a0)
        a0_n: float = self.NormalizeAngle(a0)
        a1_n: float = self.NormalizeAngle(a1)
        d: float = a1_n - a0_n
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        r: float = (t - t0) / (t1 - t0)
        a: float = a0_n + d * r
        return self.NormalizeAngle(a)

    def InterpolateUsingLinearApproximation(self, tp0: TrajectoryPoint, tp1: TrajectoryPoint, t: float) -> TrajectoryPoint:
        """
        Interpolate using linear approximation

        :param TrajectoryPoint tp0: The first trajectory point
        :param TrajectoryPoint tp1: The second trajectory point
        :param float t: The relative time
        :returns: The interpolated trajectory point
        :rtype: TrajectoryPoint
        """

        if not tp0.has_path_point or not tp1.has_path_point:
            # This part has not been implemented
            p: TrajectoryPoint = TrajectoryPoint()
            p.mutable_path_point = PathPoint()
            return p
        pp0: PathPoint = tp0.path_point
        pp1: PathPoint = tp1.path_point
        t0: float = tp0.relative_time
        t1: float = tp1.relative_time

        tp: TrajectoryPoint = TrajectoryPoint()
        tp.v = self.lerp(tp0.v, t0, tp1.v, t1, t)
        tp.a = self.lerp(tp0.a, t0, tp1.a, t1, t)
        tp.relative_time = t
        tp.steer = self.slerp(tp0.steer, t0, tp1.steer, t1, t)

        path_point: PathPoint = tp.path_point
        path_point.x = self.lerp(pp0.x, t0, pp1.x, t1, t)
        path_point.y = self.lerp(pp0.y, t0, pp1.y, t1, t)
        path_point.theta = self.slerp(pp0.theta, t0, pp1.theta, t1, t)
        path_point.kappa = self.lerp(pp0.kappa, t0, pp1.kappa, t1, t)
        path_point.dkappa = self.lerp(pp0.dkappa, t0, pp1.dkappa, t1, t)
        path_point.ddkappa = self.lerp(pp0.ddkappa, t0, pp1.ddkappa, t1, t)
        path_point.s = self.lerp(pp0.s, t0, pp1.s, t1, t)

        return tp

    def QueryLowerBoundPoint(self, relative_time: float, epsilon: float) -> int:
        """
        Query the lower bound point

        :param float relative_time: The relative time
        :param float epsilon: The epsilon
        :returns: The index of the lower bound point
        :rtype: int
        """

        assert self, "Trajectory points list is empty"

        if relative_time >= self[-1].relative_time:
            return len(self) - 1
        times = [tp.relative_time for tp in self]
        idx = bisect_left(times, relative_time - epsilon)
        return idx

    def QueryNearestPoint(self, position: Vec2d) -> int:
        """
        Query the nearest point

        :param Vec2d position: The position
        :returns: The index of the nearest point
        :rtype: int
        """

        dist_sqr_min: float = float('inf')
        index_min: int = 0
        for i, tp in enumerate(self):
            curr_point: Vec2d = Vec2d(tp.path_point.x, tp.path_point.y)
            dist_sqr: float = curr_point.DistanceSquareTo(position)
            if dist_sqr < dist_sqr_min:
                dist_sqr_min = dist_sqr
                index_min = i
        return index_min

    def QueryNearestPointWithBuffer(self, position: Vec2d, buffer: float) -> int:
        """
        Query the nearest point with buffer

        :param Vec2d position: The position
        :param float buffer: The buffer
        :returns: The index of the nearest point with buffer
        :rtype: int
        """

        dist_sqr_min: float = float('inf')
        index_min: int = 0
        for i, tp in enumerate(self):
            curr_point: Vec2d = Vec2d(tp.path_point.x, tp.path_point.y)
            dist_sqr: float = curr_point.DistanceSquareTo(position)
            if dist_sqr < dist_sqr_min + buffer:
                dist_sqr_min = dist_sqr
                index_min = i
        return index_min

    def AppendTrajectoryPoint(self, trajectory_point: TrajectoryPoint) -> None:
        """
        Append a trajectory point to the list

        :param TrajectoryPoint trajectory_point: The trajectory point to append
        """
        if not self:
            assert trajectory_point.relative_time > self[-1].relative_time, "The relative time of the new trajectory point is not larger than the last one"
        self.append(trajectory_point)

    def TrajectoryPointAt(self, index: int) -> TrajectoryPoint:
        """
        Get the trajectory point at the given index

        :param int index: The index
        :returns: The trajectory point at the given index
        :rtype: TrajectoryPoint
        """

        assert 0 <= index < self.NumOfPoints(), "Index out of range"
        return self[index]
    
    def NumOfPoints(self) -> int:
        """
        Get the number of trajectory points

        :returns: The number of trajectory points
        :rtype: int
        """

        return len(self)
