
"""Flags the lattice planner and the closed-loop drive still read."""

# Vehicle geometry fallback values for the simplified Python environment.
# Apollo normally reads these from VehicleConfigHelper, not planning_gflags.
EGO_VEHICLE_LENGTH = 4.933
EGO_VEHICLE_WIDTH = 2.11
EGO_BACK_EDGE_TO_CENTER = 1.043
FRONT_EDGE_TO_CENTER = 3.89
FLAGS_half_vehicle_width = EGO_VEHICLE_WIDTH / 2.0

# Reference line / trajectory publishing.
FLAGS_default_reference_line_width = 4.0
FLAGS_trajectory_time_length = 8.0

# Dynamic constraints.
FLAGS_speed_lower_bound = -0.1
FLAGS_speed_upper_bound = 40.0
FLAGS_longitudinal_acceleration_lower_bound = -6.0
FLAGS_longitudinal_acceleration_upper_bound = 4.0
FLAGS_longitudinal_jerk_lower_bound = -4.0
FLAGS_longitudinal_jerk_upper_bound = 2.0
FLAGS_lateral_acceleration_bound = 4.0
FLAGS_lateral_jerk_bound = 4.0
FLAGS_kappa_bound = 0.1979

# Lattice planner core.
FLAGS_numerical_epsilon = 1e-6
FLAGS_default_cruise_speed = 11.18  # planning.conf overrides 5.0.
FLAGS_trajectory_time_resolution = 0.1
FLAGS_trajectory_space_resolution = 1.0
FLAGS_speed_lon_decision_horizon = 200.0
FLAGS_num_velocity_sample = 6
FLAGS_enable_backup_trajectory = True
FLAGS_min_velocity_sample_gap = 1.0
FLAGS_lon_collision_buffer = 2.0
FLAGS_lat_collision_buffer = 0.1
FLAGS_num_sample_follow_per_timestamp = 3

# Lattice trajectory evaluator.
FLAGS_weight_lon_objective = 10.0
FLAGS_weight_lon_jerk = 1.0
FLAGS_weight_lon_collision = 5.0
FLAGS_weight_lat_offset = 2.0
FLAGS_weight_lat_comfort = 10.0
FLAGS_weight_centripetal_acceleration = 1.5
FLAGS_weight_same_side_offset = 1.0
FLAGS_weight_opposite_side_offset = 10.0
FLAGS_weight_dist_travelled = 10.0
FLAGS_weight_target_speed = 1.0
FLAGS_lat_offset_bound = 3.0
FLAGS_lon_collision_yield_buffer = 1.0
FLAGS_lon_collision_overtake_buffer = 5.0
FLAGS_lon_collision_cost_std = 0.5
FLAGS_default_lon_buffer = 5.0
FLAGS_time_min_density = 1.0
FLAGS_comfort_acceleration_factor = 0.5
FLAGS_polynomial_minimal_param = 0.01
FLAGS_lattice_stop_buffer = 0.02

# Lateral OSQP optimization.
FLAGS_lateral_optimization = True
FLAGS_weight_lateral_offset = 1.0
FLAGS_weight_lateral_derivative = 500.0
FLAGS_weight_lateral_second_order_derivative = 1000.0
FLAGS_weight_lateral_obstacle_distance = 0.0
FLAGS_lateral_third_order_derivative_max = 0.1
FLAGS_max_s_lateral_optimization = 60.0
FLAGS_default_delta_s_lateral_optimization = 1.0
FLAGS_bound_buffer = 0.1
FLAGS_nudge_buffer = 0.3
FLAGS_enable_osqp_debug = False

# Closed-loop cycle.
FLAGS_planning_loop_rate = 10.0
FLAGS_enable_lateral_jerk_check = False
