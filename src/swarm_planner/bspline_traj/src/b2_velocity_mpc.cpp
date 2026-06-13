#include <bspline_race/b2_velocity_mpc.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace FLAG_Race {

B2VelocityMpcController::B2VelocityMpcController() = default;

B2VelocityMpcController::B2VelocityMpcController(const B2MpcConfig& config)
    : config_(config) {}

void B2VelocityMpcController::setConfig(const B2MpcConfig& config) {
  config_ = config;
  reset();
}

const B2MpcConfig& B2VelocityMpcController::config() const { return config_; }

const B2MpcDebug& B2VelocityMpcController::lastDebug() const { return last_debug_; }

void B2VelocityMpcController::reset() {
  last_solution_.clear();
  last_cmd_ = B2MpcCommand{};
  has_last_cmd_ = false;
  last_debug_ = B2MpcDebug{};
}

double B2VelocityMpcController::wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double B2VelocityMpcController::clamp(double value, double lower, double upper) {
  if (lower > upper) std::swap(lower, upper);
  return std::max(lower, std::min(value, upper));
}

Eigen::Vector2d B2VelocityMpcController::worldToBody(const Eigen::Vector2d& world,
                                                     double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * world.x() + s * world.y(),
                         -s * world.x() + c * world.y());
}

Eigen::Vector2d B2VelocityMpcController::bodyToWorld(const Eigen::Vector2d& body,
                                                     double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * body.x() - s * body.y(),
                         s * body.x() + c * body.y());
}

B2MpcCommand B2VelocityMpcController::clampVelocity(const B2MpcCommand& cmd) const {
  B2MpcCommand out;
  out.vx = clamp(cmd.vx, -std::abs(config_.vx_max), std::abs(config_.vx_max));
  out.vy = clamp(cmd.vy, -std::abs(config_.vy_max), std::abs(config_.vy_max));
  out.yaw_rate = clamp(cmd.yaw_rate,
                       -std::abs(config_.yaw_rate_max),
                       std::abs(config_.yaw_rate_max));
  return out;
}

B2MpcCommand B2VelocityMpcController::limitRate(const B2MpcCommand& desired,
                                                const B2MpcCommand& previous,
                                                double dt) const {
  if (dt <= 1e-6) return clampVelocity(desired);

  B2MpcCommand out;
  out.vx = previous.vx + clamp(desired.vx - previous.vx,
                              -std::abs(config_.acc_x_max) * dt,
                              std::abs(config_.acc_x_max) * dt);
  out.vy = previous.vy + clamp(desired.vy - previous.vy,
                              -std::abs(config_.acc_y_max) * dt,
                              std::abs(config_.acc_y_max) * dt);
  out.yaw_rate = previous.yaw_rate + clamp(desired.yaw_rate - previous.yaw_rate,
                                          -std::abs(config_.yaw_acc_max) * dt,
                                          std::abs(config_.yaw_acc_max) * dt);
  return clampVelocity(out);
}

void B2VelocityMpcController::projectSequence(Sequence& sequence, double first_dt) const {
  B2MpcCommand previous = has_last_cmd_ ? last_cmd_ : B2MpcCommand{};
  const double dt0 = std::max(1e-3, first_dt);
  const double horizon_dt = std::max(1e-3, config_.dt);

  for (size_t i = 0; i < sequence.size(); ++i) {
    const double dt = (i == 0) ? dt0 : horizon_dt;
    sequence[i] = limitRate(clampVelocity(sequence[i]), previous, dt);
    previous = sequence[i];
  }
}

B2VelocityMpcController::Sequence B2VelocityMpcController::initialSequence(
    const Eigen::Vector2d& reference_world,
    const B2MpcState& state,
    double first_dt) const {
  const int horizon = std::max(1, config_.horizon_steps);
  Sequence sequence(horizon);

  const Eigen::Vector2d reference_body = worldToBody(reference_world, state.yaw);
  const double ref_norm = reference_world.norm();
  B2MpcCommand desired;
  desired.vx = reference_body.x();
  desired.vy = reference_body.y();
  if (config_.heading_weight > 1e-9 &&
      ref_norm > std::max(0.0, config_.guidance_deadband)) {
    desired.yaw_rate = clamp(wrapAngle(std::atan2(reference_world.y(), reference_world.x()) - state.yaw) /
                                 std::max(0.2, static_cast<double>(horizon) * std::max(1e-3, config_.dt)),
                             -std::abs(config_.yaw_rate_max),
                             std::abs(config_.yaw_rate_max));
  }
  desired = clampVelocity(desired);

  if (!last_solution_.empty()) {
    for (int i = 0; i < horizon; ++i) {
      const int shifted = std::min<int>(i + 1, static_cast<int>(last_solution_.size()) - 1);
      sequence[i] = last_solution_[shifted];
    }
    sequence.back() = desired;
  } else {
    std::fill(sequence.begin(), sequence.end(), desired);
  }

  projectSequence(sequence, first_dt);
  return sequence;
}

double B2VelocityMpcController::evaluateCost(const Sequence& sequence,
                                             const Eigen::Vector2d& reference_world,
                                             const B2MpcState& state) const {
  const double horizon_dt = std::max(1e-3, config_.dt);
  const double beta_x = clamp(horizon_dt / std::max(1e-3, config_.tau_x), 0.0, 1.0);
  const double beta_y = clamp(horizon_dt / std::max(1e-3, config_.tau_y), 0.0, 1.0);
  const double beta_w = clamp(horizon_dt / std::max(1e-3, config_.tau_yaw), 0.0, 1.0);

  Eigen::Vector2d body_velocity = worldToBody(state.velocity_world, state.yaw);
  double yaw = state.yaw;
  double yaw_rate = state.yaw_rate;
  B2MpcCommand previous = has_last_cmd_ ? last_cmd_ : B2MpcCommand{};
  double cost = 0.0;
  const double ref_norm = reference_world.norm();
  const double heading_des = ref_norm > std::max(0.0, config_.guidance_deadband)
                                 ? std::atan2(reference_world.y(), reference_world.x())
                                 : yaw;

  for (const auto& cmd_raw : sequence) {
    const B2MpcCommand cmd = clampVelocity(cmd_raw);
    body_velocity.x() += beta_x * (cmd.vx - body_velocity.x());
    body_velocity.y() += beta_y * (cmd.vy - body_velocity.y());
    yaw_rate += beta_w * (cmd.yaw_rate - yaw_rate);
    yaw = wrapAngle(yaw + yaw_rate * horizon_dt);

    const Eigen::Vector2d velocity_world = bodyToWorld(body_velocity, yaw);
    const Eigen::Vector2d velocity_error = velocity_world - reference_world;
    const double heading_error = wrapAngle(heading_des - yaw);

    cost += config_.velocity_weight * velocity_error.squaredNorm();
    cost += config_.heading_weight * heading_error * heading_error;
    cost += config_.control_weight * (cmd.vx * cmd.vx + cmd.vy * cmd.vy + cmd.yaw_rate * cmd.yaw_rate);
    cost += config_.control_rate_weight *
            ((cmd.vx - previous.vx) * (cmd.vx - previous.vx) +
             (cmd.vy - previous.vy) * (cmd.vy - previous.vy) +
             (cmd.yaw_rate - previous.yaw_rate) * (cmd.yaw_rate - previous.yaw_rate));
    previous = cmd;
  }

  return cost;
}

B2MpcCommand& B2VelocityMpcController::commandAt(Sequence& sequence, int flat_index) {
  return sequence[flat_index / 3];
}

double B2VelocityMpcController::commandValue(const Sequence& sequence, int flat_index) const {
  const auto& cmd = sequence[flat_index / 3];
  switch (flat_index % 3) {
    case 0: return cmd.vx;
    case 1: return cmd.vy;
    default: return cmd.yaw_rate;
  }
}

void B2VelocityMpcController::setCommandValue(Sequence& sequence, int flat_index, double value) {
  auto& cmd = commandAt(sequence, flat_index);
  switch (flat_index % 3) {
    case 0: cmd.vx = value; break;
    case 1: cmd.vy = value; break;
    default: cmd.yaw_rate = value; break;
  }
}

B2MpcCommand B2VelocityMpcController::compute(const Eigen::Vector3d& guidance_world,
                                              const B2MpcState& state,
                                              double dt) {
  const Eigen::Vector2d reference_world = guidance_world.head<2>();
  if (reference_world.norm() <= std::max(0.0, config_.guidance_deadband)) {
    reset();
    return B2MpcCommand{};
  }

  if (!has_last_cmd_) {
    last_cmd_ = B2MpcCommand{};
    has_last_cmd_ = true;
  }

  Sequence sequence = initialSequence(reference_world, state, dt);
  const int horizon = static_cast<int>(sequence.size());
  const int variable_count = 3 * horizon;
  const int iterations = std::max(0, config_.iterations);
  const double eps = std::max(1e-5, config_.gradient_eps);
  const double alpha = std::max(0.0, config_.gradient_step);

  for (int iter = 0; iter < iterations; ++iter) {
    Sequence next = sequence;
    for (int j = 0; j < variable_count; ++j) {
      Sequence plus = sequence;
      Sequence minus = sequence;
      setCommandValue(plus, j, commandValue(plus, j) + eps);
      setCommandValue(minus, j, commandValue(minus, j) - eps);
      projectSequence(plus, dt);
      projectSequence(minus, dt);

      const double grad = (evaluateCost(plus, reference_world, state) -
                           evaluateCost(minus, reference_world, state)) /
                          (2.0 * eps);
      setCommandValue(next, j, commandValue(next, j) - alpha * grad);
    }
    projectSequence(next, dt);
    if (evaluateCost(next, reference_world, state) <= evaluateCost(sequence, reference_world, state)) {
      sequence = next;
    } else {
      break;
    }
  }

  projectSequence(sequence, dt);
  last_solution_ = sequence;
  last_cmd_ = sequence.front();
  return last_cmd_;
}

}  // namespace FLAG_Race
