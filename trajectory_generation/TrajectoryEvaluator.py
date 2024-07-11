
class TrajectoryEvaluator:
    def __init__(self, init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle, path_time_graph, path_points):
        pass

    def has_more_trajectory_pairs(self):
        return False

    def top_trajectory_pair_cost(self):
        return 0.0

    def next_top_trajectory_pair(self):
        return None, None