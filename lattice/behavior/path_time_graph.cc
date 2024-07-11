namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::lerp;
using apollo::common::math::PathMatcher;
using apollo::common::math::Polygon2d;



void PathTimeGraph::SetDynamicObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  double relative_time = time_range_.first;
  while (relative_time < time_range_.second) {
    TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
    Box2d box = obstacle->GetBoundingBox(point);
    SLBoundary sl_boundary =
        ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);

    double left_width = FLAGS_default_reference_line_width * 0.5;
    double right_width = FLAGS_default_reference_line_width * 0.5;
    ptr_reference_line_info_->reference_line().GetLaneWidth(
        sl_boundary.start_s(), &left_width, &right_width);

    // The obstacle is not shown on the region to be considered.
    if (sl_boundary.start_s() > path_range_.second ||
        sl_boundary.end_s() < path_range_.first ||
        sl_boundary.start_l() > left_width ||
        sl_boundary.end_l() < -right_width) {
      if (path_time_obstacle_map_.find(obstacle->Id()) !=
          path_time_obstacle_map_.end()) {
        break;
      }
      relative_time += FLAGS_trajectory_time_resolution;
      continue;
    }

    if (path_time_obstacle_map_.find(obstacle->Id()) ==
        path_time_obstacle_map_.end()) {
      path_time_obstacle_map_[obstacle->Id()].set_id(obstacle->Id());

      path_time_obstacle_map_[obstacle->Id()].set_bottom_left_point(
          SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(),
                           relative_time));
      path_time_obstacle_map_[obstacle->Id()].set_upper_left_point(
          SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));
    }

    path_time_obstacle_map_[obstacle->Id()].set_bottom_right_point(
        SetPathTimePoint(obstacle->Id(), sl_boundary.start_s(), relative_time));
    path_time_obstacle_map_[obstacle->Id()].set_upper_right_point(
        SetPathTimePoint(obstacle->Id(), sl_boundary.end_s(), relative_time));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

STPoint PathTimeGraph::SetPathTimePoint(const std::string& obstacle_id,
                                        const double s, const double t) const {
  STPoint path_time_point(s, t);
  return path_time_point;
}

const std::vector<STBoundary>& PathTimeGraph::GetPathTimeObstacles() const {
  return path_time_obstacles_;
}

bool PathTimeGraph::GetPathTimeObstacle(const std::string& obstacle_id,
                                        STBoundary* path_time_obstacle) {
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return false;
  }
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;
}

std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
    const double t) const {
  ACHECK(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto& pt_obstacle : path_time_obstacles_) {
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }
    double s_upper = lerp(pt_obstacle.upper_left_point().s(),
                          pt_obstacle.upper_left_point().t(),
                          pt_obstacle.upper_right_point().s(),
                          pt_obstacle.upper_right_point().t(), t);

    double s_lower = lerp(pt_obstacle.bottom_left_point().s(),
                          pt_obstacle.bottom_left_point().t(),
                          pt_obstacle.bottom_right_point().s(),
                          pt_obstacle.bottom_right_point().t(), t);

    intervals.emplace_back(s_lower, s_upper);
  }
  return intervals;
}

std::vector<std::vector<std::pair<double, double>>>
PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                        const double t_end,
                                        const double t_resolution) {
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = t_start; t <= t_end; t += t_resolution) {
    intervals.push_back(GetPathBlockingIntervals(t));
  }
  return intervals;
}

std::pair<double, double> PathTimeGraph::get_path_range() const {
  return path_range_;
}

std::pair<double, double> PathTimeGraph::get_time_range() const {
  return time_range_;
}

std::vector<STPoint> PathTimeGraph::GetObstacleSurroundingPoints(
    const std::string& obstacle_id, const double s_dist,
    const double t_min_density) const {
  ACHECK(t_min_density > 0.0);
  std::vector<STPoint> pt_pairs;
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return pt_pairs;
  }

  const auto& pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

  double s0 = 0.0;
  double s1 = 0.0;

  double t0 = 0.0;
  double t1 = 0.0;
  if (s_dist > 0.0) {
    s0 = pt_obstacle.upper_left_point().s();
    s1 = pt_obstacle.upper_right_point().s();

    t0 = pt_obstacle.upper_left_point().t();
    t1 = pt_obstacle.upper_right_point().t();
  } else {
    s0 = pt_obstacle.bottom_left_point().s();
    s1 = pt_obstacle.bottom_right_point().s();

    t0 = pt_obstacle.bottom_left_point().t();
    t1 = pt_obstacle.bottom_right_point().t();
  }

  double time_gap = t1 - t0;
  ACHECK(time_gap > -FLAGS_numerical_epsilon);
  time_gap = std::fabs(time_gap);

  size_t num_sections = static_cast<size_t>(time_gap / t_min_density + 1);
  double t_interval = time_gap / static_cast<double>(num_sections);

  for (size_t i = 0; i <= num_sections; ++i) {
    double t = t_interval * static_cast<double>(i) + t0;
    double s = lerp(s0, t0, s1, t1, t) + s_dist;

    STPoint ptt;
    ptt.set_t(t);
    ptt.set_s(s);
    pt_pairs.push_back(std::move(ptt));
  }

  return pt_pairs;
}

bool PathTimeGraph::IsObstacleInGraph(const std::string& obstacle_id) {
  return path_time_obstacle_map_.find(obstacle_id) !=
         path_time_obstacle_map_.end();
}

std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(
    const double s_start, const double s_end, const double s_resolution) {
  CHECK_LT(s_start, s_end);
  CHECK_GT(s_resolution, FLAGS_numerical_epsilon);
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;
  double s_range = s_end - s_start;
  double s_curr = s_start;
  size_t num_bound = static_cast<size_t>(s_range / s_resolution);

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_width = vehicle_config.vehicle_param().width();

  // Initialize bounds by reference line width
  for (size_t i = 0; i < num_bound; ++i) {
    double left_width = FLAGS_default_reference_line_width / 2.0;
    double right_width = FLAGS_default_reference_line_width / 2.0;
    ptr_reference_line_info_->reference_line().GetLaneWidth(s_curr, &left_width,
                                                            &right_width);
    double ego_d_lower = init_d_[0] - ego_width / 2.0;
    double ego_d_upper = init_d_[0] + ego_width / 2.0;
    bounds.emplace_back(
        std::min(-right_width, ego_d_lower - FLAGS_bound_buffer),
        std::max(left_width, ego_d_upper + FLAGS_bound_buffer));
    discretized_path.push_back(s_curr);
    s_curr += s_resolution;
  }

  for (const SLBoundary& static_sl_boundary : static_obs_sl_boundaries_) {
    UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start,
                                  s_end, &bounds);
  }

  for (size_t i = 0; i < bounds.size(); ++i) {
    bounds[i].first += ego_width / 2.0;
    bounds[i].second -= ego_width / 2.0;
    if (bounds[i].first >= bounds[i].second) {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }
  return bounds;
}

void PathTimeGraph::UpdateLateralBoundsByObstacle(
    const SLBoundary& sl_boundary, const std::vector<double>& discretized_path,
    const double s_start, const double s_end,
    std::vector<std::pair<double, double>>* const bounds) {
  if (sl_boundary.start_s() > s_end || sl_boundary.end_s() < s_start) {
    return;
  }
  auto start_iter = std::lower_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  auto end_iter = std::upper_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  size_t start_index = start_iter - discretized_path.begin();
  size_t end_index = end_iter - discretized_path.begin();
  if (sl_boundary.end_l() > -FLAGS_numerical_epsilon &&
      sl_boundary.start_l() < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < end_index; ++i) {
      bounds->operator[](i).first = -FLAGS_numerical_epsilon;
      bounds->operator[](i).second = FLAGS_numerical_epsilon;
    }
    return;
  }
  if (sl_boundary.end_l() < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->operator[](i).first =
          std::max(bounds->operator[](i).first,
                   sl_boundary.end_l() + FLAGS_nudge_buffer);
    }
    return;
  }
  if (sl_boundary.start_l() > -FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->operator[](i).second =
          std::min(bounds->operator[](i).second,
                   sl_boundary.start_l() - FLAGS_nudge_buffer);
    }
    return;
  }
}

}  // namespace planning
}  // namespace apollo
