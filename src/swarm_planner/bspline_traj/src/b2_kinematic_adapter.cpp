#include <bspline_race/b2_kinematic_adapter.h>

#include <algorithm>
#include <cmath>

namespace FLAG_Race {

B2KinematicAdapter::B2KinematicAdapter() = default;

B2KinematicAdapter::B2KinematicAdapter(const B2KinematicConfig& config)
    : config_(config) {}

void B2KinematicAdapter::setConfig(const B2KinematicConfig& config) {
  config_ = config;
  reset();
}

const B2KinematicConfig& B2KinematicAdapter::config() const { return config_; }

void B2KinematicAdapter::reset() {
  last_cmd_ = B2VelocityCommand{};
  has_last_cmd_ = false;
}

double B2KinematicAdapter::wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double B2KinematicAdapter::clamp(double value, double lower, double upper) {
  if (lower > upper) std::swap(lower, upper);
  return std::max(lower, std::min(value, upper));
}

double B2KinematicAdapter::limitRate(double desired,
                                     double previous,
                                     double rate_limit,
                                     double dt) {
  if (dt <= 1e-6 || rate_limit <= 1e-9) return desired;
  const double delta = desired - previous;
  const double max_delta = std::max(0.0, rate_limit) * dt;
  return previous + clamp(delta, -max_delta, max_delta);
}

B2VelocityCommand B2KinematicAdapter::compute(const Eigen::Vector3d& guidance_world,
                                              double yaw,
                                              double dt) {
  const Eigen::Vector2d guidance_xy = guidance_world.head<2>();
  if (guidance_xy.norm() <= std::max(0.0, config_.guidance_deadband)) {
    reset();
    return B2VelocityCommand{};
  }

  const double heading_des = std::atan2(guidance_xy.y(), guidance_xy.x());
  const double heading_err = wrapAngle(heading_des - yaw);

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double body_x = c * guidance_xy.x() + s * guidance_xy.y();
  const double body_y = -s * guidance_xy.x() + c * guidance_xy.y();

  B2VelocityCommand desired;
  desired.vx = clamp(body_x, -std::abs(config_.vx_max), std::abs(config_.vx_max));
  desired.vy = clamp(body_y, -std::abs(config_.vy_max), std::abs(config_.vy_max));
  desired.yaw_rate = clamp(config_.heading_kp * heading_err,
                           -std::abs(config_.yaw_rate_max),
                           std::abs(config_.yaw_rate_max));

  if (!has_last_cmd_) {
    last_cmd_ = B2VelocityCommand{};
    has_last_cmd_ = true;
  }

  B2VelocityCommand limited;
  limited.vx = limitRate(desired.vx, last_cmd_.vx, std::abs(config_.acc_x_max), dt);
  limited.vy = limitRate(desired.vy, last_cmd_.vy, std::abs(config_.acc_y_max), dt);
  limited.yaw_rate = limitRate(desired.yaw_rate, last_cmd_.yaw_rate,
                               std::abs(config_.yaw_acc_max), dt);

  last_cmd_ = limited;
  return limited;
}

}  // namespace FLAG_Race
