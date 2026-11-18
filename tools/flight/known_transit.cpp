// Compiled into the native manager. Only point-to-view proposals starting at rest
// are eligible. Moving C2 requests retain their original boundary derivatives.
bool FastPlannerManager::planKnownTransit(const Eigen::Vector3d& start, const Eigen::Vector3d& vel,
    const Eigen::Vector3d& acc, const Eigen::Vector3d& yaw, const Eigen::Vector3d& goal,
    double view_yaw) {
  static const bool enabled = [] {
    const char* mode = std::getenv("ASPIRE_KNOWN_TRANSIT");
    return mode && std::string(mode) == "1";
  }();
  if (!enabled || vel.norm() > 1e-8 || acc.norm() > 1e-8 || yaw.tail<2>().norm() > 1e-8)
    return false;
  const Eigen::Vector3d delta = goal - start;
  const double distance = delta.norm(), resolution = sdf_map_->getResolution();
  if (!(distance >= 1e-4 && distance <= 7. && resolution > 0.) ||
      !(pp_.max_vel_ > 0. && pp_.max_acc_ > 0. && pp_.max_jerk_ > 0. && pp_.max_yawdot_ > 0.))
    return false;

  // Each short piece is covered by its entire voxel bounding box, including
  // both sides of boundary ties. Any unknown or inflated cell rejects the shortcut.
  const int pieces = static_cast<int>(std::ceil(distance / (.5 * resolution)));
  for (int i = 0; i < pieces; ++i) {
    Eigen::Vector3d a = start + delta * (double(i) / pieces);
    Eigen::Vector3d b = start + delta * (double(i + 1) / pieces);
    Eigen::Vector3i lo, hi;
    sdf_map_->posToIndex(a.cwiseMin(b).array() - 1e-8, lo);
    sdf_map_->posToIndex(a.cwiseMax(b).array() + 1e-8, hi);
    for (int x = lo.x(); x <= hi.x(); ++x)
      for (int y = lo.y(); y <= hi.y(); ++y)
        for (int z = lo.z(); z <= hi.z(); ++z) {
          Eigen::Vector3i cell(x, y, z);
          if (!sdf_map_->isInBox(cell) || sdf_map_->getOccupancy(cell) != SDFMap::FREE ||
              sdf_map_->getInflateOccupancy(cell) != 0) return false;
        }
  }

  // Collinear cubic controls give zero speed and acceleration at both ends,
  // with one continuous traverse through all internal voxel/waypoint boundaries.
  const int spans = 7, yaw_spans = 24;
  const double turn = std::remainder(view_yaw - yaw[0], 2. * M_PI);
  const double stride = distance / (spans - 2);
  const double pos_dt = std::max({stride / pp_.max_vel_, std::sqrt(stride / pp_.max_acc_),
                                  std::cbrt(2. * stride / pp_.max_jerk_)});
  const double duration = std::max({.5, spans * pos_dt,
      yaw_spans * std::abs(turn) / ((yaw_spans - 2) * pp_.max_yawdot_)});
  const double yaw_dt = duration / yaw_spans;
  const int turn_spans = std::min(yaw_spans, 2 + static_cast<int>(
      std::ceil(std::abs(turn) / (yaw_dt * pp_.max_yawdot_))));
  const int hold_spans = yaw_spans - turn_spans;
  Eigen::MatrixXd positions(spans + 3, 3), headings(yaw_spans + 3, 1);
  for (int i = 0; i < positions.rows(); ++i) {
    const double fraction = std::min(1., std::max(0., (i - 2.) / (spans - 2)));
    positions.row(i) = (start + fraction * delta).transpose();
  }
  for (int i = 0; i < headings.rows(); ++i) {
    const double fraction = std::min(1., std::max(0., (i - hold_spans - 2.) /
                                                       std::max(1, turn_spans - 2)));
    headings(i, 0) = yaw[0] + fraction * turn;
  }
  local_data_.position_traj_.setUniformBspline(positions, 3, duration / spans);
  local_data_.yaw_traj_.setUniformBspline(headings, 3, yaw_dt);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = yaw_dt;
  updateTrajInfo();
  ROS_INFO("KNOWN_TRANSIT length=%.4f duration=%.4f hold_yaw_s=%.4f turn_rad=%.5f",
           distance, duration, hold_spans * yaw_dt, turn);
  return true;
}
