import heapq
from typing import List, Tuple

# 定义类型别名
Trajectory1dPair = Tuple['Curve1d', 'Curve1d']
State = List[float]
FLAGS_trajectory_time_length = 1.0  # 假设时间长度为1.0

class Curve1d:
    def __init__(self, coefficients: List[float]):
        self.coefficients = coefficients

    def Evaluate(self, derivative_order: int, t: float) -> float:
        if derivative_order == 1:
            return sum(c * t**i for i, c in enumerate(self.coefficients[1:], 1))
        return sum(c * t**i for i, c in enumerate(self.coefficients))

class ConstantDecelerationTrajectory1d(Curve1d):
    def __init__(self, initial_s: float, initial_v: float, deceleration: float):
        super().__init__([initial_s, initial_v, 0.5 * deceleration])

class Trajectory1dGenerator:
    def GenerateLateralTrajectoryBundle(self, lat_trajectories: List[Curve1d]):
        lat_trajectories.append(Curve1d([0, 0, 0.1]))
        lat_trajectories.append(Curve1d([0, 0, -0.1]))



class BackupTrajectoryGenerator:
    def __init__(self, trajectory1d_generator: Trajectory1dGenerator):
        self.ptr_trajectory1d_generator = trajectory1d_generator
        self.trajectory_pair_pqueue: List[Tuple[CostComparator, Trajectory1dPair]] = []

    def GenerateTrajectory1dPairs(self, init_s: State, init_d: State):
        lon_trajectories: List[Curve1d] = []
        dds_condidates = [-0.1, -1.0, -2.0, -3.0, -4.0]
        
        for dds in dds_condidates:
            lon_trajectories.append(ConstantDecelerationTrajectory1d(init_s[0], init_s[1], dds))

        lat_trajectories: List[Curve1d] = []
        self.ptr_trajectory1d_generator.GenerateLateralTrajectoryBundle(lat_trajectories)

        for lon in lon_trajectories:
            for lat in lat_trajectories:
                trajectory_pair = (lon, lat)
                comparator = CostComparator(trajectory_pair)
                heapq.heappush(self.trajectory_pair_pqueue, (comparator, trajectory_pair))

    def get_top_trajectory_pair(self) -> Trajectory1dPair:
        return heapq.heappop(self.trajectory_pair_pqueue)[1]

# 示例使用
init_s = [0.0, 0.0, 0.0]
init_d = [0.0, 0.0, 0.0]
trajectory1d_generator = Trajectory1dGenerator()

backup_trajectory_generator = BackupTrajectoryGenerator(trajectory1d_generator)
backup_trajectory_generator.GenerateTrajectory1dPairs(init_s, init_d)

# 输出生成的轨迹对
top_trajectory_pair = backup_trajectory_generator.get_top_trajectory_pair()
print(f"Top Lon Trajectory Coefficients: {top_trajectory_pair[0].coefficients}")
print(f"Top Lat Trajectory Coefficients: {top_trajectory_pair[1].coefficients}")
