
"""Apollo planning flags used by this Python lattice-planner port.

Values mirror modules/planning/common/planning_gflags.cc. Entries overridden by
modules/planning/conf/planning.conf keep the runtime override value.
"""

# Vehicle geometry fallback values for the simplified Python environment.
# Apollo normally reads these from VehicleConfigHelper, not planning_gflags.
EGO_VEHICLE_LENGTH = 4.933
EGO_VEHICLE_WIDTH = 2.11
EGO_BACK_EDGE_TO_CENTER = 1.043
FRONT_EDGE_TO_CENTER = 3.89
BACK_EDGE_TO_CENTER = 1.043
LEFT_EDGE_TO_CENTER = 1.055
RIGHT_EDGE_TO_CENTER = 1.055
FLAGS_half_vehicle_width = EGO_VEHICLE_WIDTH / 2.0
MinSafeTurnRadius = 1.0

# Reference line / trajectory publishing.
FLAGS_default_reference_line_width = 4.0
FLAGS_default_lane_width = 3.048
FLAGS_planning_upper_speed_limit = 20.0  # planning.conf overrides 31.3.
FLAGS_trajectory_time_length = 8.0
FLAGS_trajectory_time_min_interval = 0.02
FLAGS_trajectory_time_max_interval = 0.1
FLAGS_trajectory_time_high_density_period = 1.0
FLAGS_trajectory_point_num_for_debug = 10

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

# ST boundary and obstacle decision parameters.
FLAGS_st_max_s = 100.0
FLAGS_st_max_t = 8.0
FLAGS_static_obstacle_nudge_l_buffer = 0.3
FLAGS_nonstatic_obstacle_nudge_l_buffer = 0.4
FLAGS_max_stop_distance_obstacle = 10.0
FLAGS_min_stop_distance_obstacle = 6.0
FLAGS_destination_obstacle_id = "DEST"
FLAGS_destination_check_distance = 5.0
FLAGS_virtual_stop_wall_length = 0.1
FLAGS_virtual_stop_wall_height = 2.0
FLAGS_signal_expire_time_sec = 5.0
FLAGS_obstacle_lon_ignore_buffer = 3.0

# Lattice planner core.
FLAGS_numerical_epsilon = 1e-6
FLAGS_default_cruise_speed = 11.18  # planning.conf overrides 5.0.
FLAGS_trajectory_time_resolution = 0.1
FLAGS_trajectory_space_resolution = 1.0
FLAGS_speed_lon_decision_horizon = 200.0
FLAGS_num_velocity_sample = 6
FLAGS_enable_backup_trajectory = True
FLAGS_backup_trajectory_cost = 1000.0
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
FLAGS_weight_lateral_third_order_derivative = 1000.0
FLAGS_weight_lateral_obstacle_distance = 0.0
FLAGS_lateral_third_order_derivative_max = 0.1
FLAGS_lateral_derivative_bound_default = 2.0
FLAGS_max_s_lateral_optimization = 60.0
FLAGS_default_delta_s_lateral_optimization = 1.0
FLAGS_bound_buffer = 0.1
FLAGS_nudge_buffer = 0.3
FLAGS_enable_osqp_debug = False

# Speed limits.
FLAGS_speed_bump_speed_limit = 4.4704
FLAGS_default_city_road_speed_limit = 15.67
FLAGS_default_highway_speed_limit = 29.06

# Miscellaneous planning flags currently referenced by the Python port.
FLAGS_enable_smooth_trajectory = True
FLAGS_use_navigation_mode = False
FLAGS_use_multi_thread_to_add_obstacles = False
FLAGS_turn_signal_distance = 100.0
FLAGS_passed_destination_threshold = 0.01
FLAGS_reverse_heading_vehicle_state = False
FLAGS_enable_map_reference_unify = False
FLAGS_state_transform_to_com_reverse = False
FLAGS_state_transform_to_com_drive = False
FLAGS_align_prediction_time = False
FLAGS_message_latency_threshold = 0.02

# Reference line / PncMap (planning_gflags + pnc_map.cc).
FLAGS_look_backward_distance = 50.0
FLAGS_look_forward_short_distance = 180.0
FLAGS_look_forward_long_distance = 250.0
FLAGS_look_forward_time_sec = 8.0
FLAGS_replan_lateral_distance_threshold = 0.5
FLAGS_replan_longitudinal_distance_threshold = 2.5
FLAGS_prioritize_change_lane = True
FLAGS_enable_smooth_reference_line = True
FLAGS_smoothed_reference_line_max_diff = 5.0
FLAGS_enable_reference_line_stitching = True
FLAGS_enable_reference_line_provider_thread = False
FLAGS_map_dir = "modules/map/data/demo"
FLAGS_base_map_filename = "base_map.bin|base_map.xml|base_map.txt"

# Reference line smoother defaults (discrete_points_smoother_config.pb.txt).
FLAGS_max_constraint_interval = 0.25
FLAGS_longitudinal_boundary_bound = 2.0
FLAGS_max_lateral_boundary_bound = 0.5
FLAGS_min_lateral_boundary_bound = 0.1
FLAGS_curb_shift = 0.2
FLAGS_lateral_buffer = 0.2
FLAGS_fem_pos_weight_deviation = 1e10
FLAGS_fem_pos_weight_ref_deviation = 1.0
FLAGS_fem_pos_weight_path_length = 1.0
FLAGS_num_of_total_reference_points = 500

# QP spline reference line smoother (qp_spline_smoother_config.pb.txt).
FLAGS_enable_qp_spline_reference_line = False
FLAGS_qp_spline_order = 5
FLAGS_qp_spline_max_spline_length = 25.0
FLAGS_qp_spline_regularization_weight = 1e-5
FLAGS_qp_spline_second_derivative_weight = 200.0
FLAGS_qp_spline_third_derivative_weight = 1000.0

# Traffic rules.
REF_LINE_END_VO_ID_PREFIX = "REF_END_"
FLAGS_reference_line_end_stop_distance = 0.5
FLAGS_reference_line_end_min_remain_length = 50.0
FLAGS_destination_stop_distance = 0.5

# Path decider (planning_gflags + path_decider_config.pb.txt).
FLAGS_lateral_ignore_buffer = 3.0
FLAGS_path_decider_static_obstacle_buffer = 0.3
FLAGS_static_obstacle_speed_threshold = 0.5
FLAGS_enable_skip_path_tasks = False
FLAGS_obstacle_lat_buffer = 0.4
FLAGS_obstacle_lon_start_buffer = 3.0
FLAGS_obstacle_lon_end_buffer = 2.0
FLAGS_lane_borrow_max_speed = 5.0
FLAGS_long_term_blocking_obstacle_cycle_threshold = 3
FLAGS_allow_lane_borrowing = True
FLAGS_path_bounds_decider_adc_buffer_coeff = 1.0
FLAGS_path_bounds_decider_extend_lane_bounds_to_include_adc = False
FLAGS_max_abs_speed_when_stopped = 0.2

# Reference line stitching (planning_gflags.cc).
FLAGS_look_forward_extend_distance = 50.0
FLAGS_reference_line_stitch_overlap_distance = 20.0

# Traffic rule virtual obstacle id prefixes.
STOP_SIGN_VO_ID_PREFIX = "SS_"
TRAFFIC_LIGHT_VO_ID_PREFIX = "TL_"
CROSSWALK_VO_ID_PREFIX = "CW_"
YIELD_SIGN_VO_ID_PREFIX = "YS_"
KEEP_CLEAR_VO_ID_PREFIX = "KC_"
KEEP_CLEAR_JUNCTION_VO_ID_PREFIX = "KC_JC_"

# Backside vehicle rule.
FLAGS_backside_vehicle_enabled = False
FLAGS_backside_vehicle_lane_width = 4.0

# Traffic light rule (traffic_rule_config.pb.txt).
FLAGS_traffic_light_enabled = True
FLAGS_traffic_light_stop_distance = 1.0
FLAGS_traffic_light_max_stop_deceleration = 4.0

# Stop sign rule.
FLAGS_stop_sign_enabled = True
FLAGS_stop_sign_stop_distance = 1.0

# Yield sign rule.
FLAGS_yield_sign_enabled = True
FLAGS_yield_sign_stop_distance = 1.0

# Crosswalk rule.
FLAGS_crosswalk_enabled = True
FLAGS_crosswalk_stop_distance = 1.0
FLAGS_crosswalk_max_stop_deceleration = 6.0
FLAGS_crosswalk_min_pass_s_distance = 1.0
FLAGS_crosswalk_expand_s_distance = 2.0
FLAGS_crosswalk_stop_strict_l_distance = 6.0
FLAGS_crosswalk_stop_loose_l_distance = 8.0
FLAGS_crosswalk_stop_timeout = 4.0

# Keep clear rule.
FLAGS_keep_clear_zone_enabled = True
FLAGS_keep_clear_junction_enabled = True
FLAGS_keep_clear_min_pass_s_distance = 2.0
FLAGS_keep_clear_align_with_traffic_sign_tolerance = 4.5

# Rerouting rule.
FLAGS_rerouting_prepare_time = 2.0
FLAGS_rerouting_cooldown_time = 3.0

# LatticePlanner core (lattice_planner.cc) — keep defaults off for decider stack.
FLAGS_enable_traffic_rules = True
FLAGS_enable_path_decider = False
FLAGS_enable_path_decider_after_lateral = False
FLAGS_enable_lattice_path_assessment = False
FLAGS_enable_path_bounds_decider = False
FLAGS_lattice_path_assessment_max_candidates = 8
FLAGS_enable_combine_path_and_speed_profile = False
# OnLanePlanning / lane_follow optional layer (not in lattice_planner.cc).
FLAGS_enable_on_lane_combine_path_and_speed = False
FLAGS_enable_boundary_only_fallback = False
FLAGS_cost_non_priority_reference_line = 5.0

# Trajectory stitcher / on-lane planning (planning_gflags.cc).
FLAGS_enable_trajectory_stitcher = True
FLAGS_trajectory_stitching_preserved_length = 20
FLAGS_planning_loop_rate = 10.0
FLAGS_fallback_total_time = 3.0
FLAGS_fallback_time_unit = 0.1
FLAGS_enable_lateral_jerk_check = False
