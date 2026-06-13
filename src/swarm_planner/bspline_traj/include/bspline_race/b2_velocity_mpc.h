#ifndef BSPLINE_RACE_B2_VELOCITY_MPC_H
#define BSPLINE_RACE_B2_VELOCITY_MPC_H

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace FLAG_Race {

struct B2MpcConfig {
  double vx_max = 0.6;
  double vy_max = 0.4;
  double yaw_rate_max = 0.8;
  double acc_x_max = 0.5;
  double acc_y_max = 0.3;
  double yaw_acc_max = 1.0;
  double guidance_deadband = 1e-4;

  double dt = 0.1;
  int horizon_steps = 10;
  int iterations = 6;
  double gradient_step = 0.04;
  double gradient_eps = 1e-3;

  double tau_x = 0.25;
  double tau_y = 0.30;
  double tau_yaw = 0.20;

  double velocity_weight = 20.0;
  double heading_weight = 1.0;
  double control_weight = 0.02;
  double control_rate_weight = 0.20;
  double lateral_error_weight = 2.0;
  double reference_invalid_penalty = 1e4;
};

struct B2MpcState {
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  Eigen::Vector2d velocity_world = Eigen::Vector2d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct B2MpcCommand {
  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
};

struct B2MpcDebug {
  double progress_w_start = 0.0;
  double progress_w_end = 0.0;
  Eigen::Vector2d first_reference_world = Eigen::Vector2d::Zero();
  int valid_reference_count = 0;
  int invalid_reference_count = 0;
};

class B2VelocityMpcController {
 public:
  B2VelocityMpcController();
  explicit B2VelocityMpcController(const B2MpcConfig& config);

  void setConfig(const B2MpcConfig& config);
  const B2MpcConfig& config() const;

  B2MpcCommand compute(const Eigen::Vector3d& guidance_world,
                       const B2MpcState& state,
                       double dt);
  template <typename GvfLikePtr>
  B2MpcCommand computeWaware(const GvfLikePtr& gvf_like,
                             double progress_w0,
                             const B2MpcState& state,
                             double dt);
  const B2MpcDebug& lastDebug() const;
  void reset();

  static double wrapAngle(double angle);

 private:
  using Sequence = std::vector<B2MpcCommand>;

  static double clamp(double value, double lower, double upper);
  static Eigen::Vector2d worldToBody(const Eigen::Vector2d& world, double yaw);
  static Eigen::Vector2d bodyToWorld(const Eigen::Vector2d& body, double yaw);

  B2MpcCommand clampVelocity(const B2MpcCommand& cmd) const;
  B2MpcCommand limitRate(const B2MpcCommand& desired,
                         const B2MpcCommand& previous,
                         double dt) const;
  void projectSequence(Sequence& sequence, double first_dt) const;
  Sequence initialSequence(const Eigen::Vector2d& reference_world,
                           const B2MpcState& state,
                           double first_dt) const;
  double evaluateCost(const Sequence& sequence,
                      const Eigen::Vector2d& reference_world,
                      const B2MpcState& state) const;
  template <typename GvfLikePtr>
  double evaluateWawareCost(const Sequence& sequence,
                            const GvfLikePtr& gvf_like,
                            double progress_w0,
                            const B2MpcState& state) const;
  template <typename GvfLikePtr>
  void updateWawareDebug(const Sequence& sequence,
                         const GvfLikePtr& gvf_like,
                         double progress_w0,
                         const B2MpcState& state);
  B2MpcCommand& commandAt(Sequence& sequence, int flat_index);
  double commandValue(const Sequence& sequence, int flat_index) const;
  void setCommandValue(Sequence& sequence, int flat_index, double value);

  B2MpcConfig config_;
  Sequence last_solution_;
  B2MpcCommand last_cmd_;
  bool has_last_cmd_ = false;
  B2MpcDebug last_debug_;
};

template <typename GvfLikePtr>
double B2VelocityMpcController::evaluateWawareCost(const Sequence& sequence,
                                                   const GvfLikePtr& gvf_like,
                                                   double progress_w0,
                                                   const B2MpcState& state) const {
  const double horizon_dt = std::max(1e-3, config_.dt);
  const double beta_x = clamp(horizon_dt / std::max(1e-3, config_.tau_x), 0.0, 1.0);
  const double beta_y = clamp(horizon_dt / std::max(1e-3, config_.tau_y), 0.0, 1.0);
  const double beta_w = clamp(horizon_dt / std::max(1e-3, config_.tau_yaw), 0.0, 1.0);

  Eigen::Vector3d position = state.position_world;
  Eigen::Vector2d body_velocity = worldToBody(state.velocity_world, state.yaw);
  double yaw = state.yaw;
  double yaw_rate = state.yaw_rate;
  double progress_w = progress_w0;
  B2MpcCommand previous = has_last_cmd_ ? last_cmd_ : B2MpcCommand{};
  double cost = 0.0;

  for (const auto& cmd_raw : sequence) {
    const B2MpcCommand cmd = clampVelocity(cmd_raw);
    body_velocity.x() += beta_x * (cmd.vx - body_velocity.x());
    body_velocity.y() += beta_y * (cmd.vy - body_velocity.y());
    yaw_rate += beta_w * (cmd.yaw_rate - yaw_rate);
    yaw = wrapAngle(yaw + yaw_rate * horizon_dt);

    const Eigen::Vector2d velocity_world = bodyToWorld(body_velocity, yaw);
    position.x() += velocity_world.x() * horizon_dt;
    position.y() += velocity_world.y() * horizon_dt;

    const auto guidance = gvf_like->calcLiftedGuidance3D(position, progress_w);
    if (!guidance.valid) {
      cost += std::max(0.0, config_.reference_invalid_penalty);
      previous = cmd;
      continue;
    }

    const Eigen::Vector2d reference_world = guidance.v_cmd.template head<2>();
    const Eigen::Vector2d velocity_error = velocity_world - reference_world;
    const double ref_norm = reference_world.norm();
    const double heading_des = ref_norm > std::max(0.0, config_.guidance_deadband)
                                   ? std::atan2(reference_world.y(), reference_world.x())
                                   : yaw;
    const double heading_error = wrapAngle(heading_des - yaw);

    cost += config_.velocity_weight * velocity_error.squaredNorm();
    cost += config_.heading_weight * heading_error * heading_error;
    cost += std::max(0.0, config_.lateral_error_weight) *
            guidance.e_perp.template head<2>().squaredNorm();
    cost += config_.control_weight * (cmd.vx * cmd.vx + cmd.vy * cmd.vy + cmd.yaw_rate * cmd.yaw_rate);
    cost += config_.control_rate_weight *
            ((cmd.vx - previous.vx) * (cmd.vx - previous.vx) +
             (cmd.vy - previous.vy) * (cmd.vy - previous.vy) +
             (cmd.yaw_rate - previous.yaw_rate) * (cmd.yaw_rate - previous.yaw_rate));

    progress_w = guidance.w_proj + guidance.w_dot * horizon_dt;
    previous = cmd;
  }

  return cost;
}

template <typename GvfLikePtr>
void B2VelocityMpcController::updateWawareDebug(const Sequence& sequence,
                                                const GvfLikePtr& gvf_like,
                                                double progress_w0,
                                                const B2MpcState& state) {
  last_debug_ = B2MpcDebug{};
  last_debug_.progress_w_start = progress_w0;
  last_debug_.progress_w_end = progress_w0;

  const double horizon_dt = std::max(1e-3, config_.dt);
  const double beta_x = clamp(horizon_dt / std::max(1e-3, config_.tau_x), 0.0, 1.0);
  const double beta_y = clamp(horizon_dt / std::max(1e-3, config_.tau_y), 0.0, 1.0);
  const double beta_w = clamp(horizon_dt / std::max(1e-3, config_.tau_yaw), 0.0, 1.0);

  Eigen::Vector3d position = state.position_world;
  Eigen::Vector2d body_velocity = worldToBody(state.velocity_world, state.yaw);
  double yaw = state.yaw;
  double yaw_rate = state.yaw_rate;
  double progress_w = progress_w0;

  for (const auto& cmd_raw : sequence) {
    const B2MpcCommand cmd = clampVelocity(cmd_raw);
    body_velocity.x() += beta_x * (cmd.vx - body_velocity.x());
    body_velocity.y() += beta_y * (cmd.vy - body_velocity.y());
    yaw_rate += beta_w * (cmd.yaw_rate - yaw_rate);
    yaw = wrapAngle(yaw + yaw_rate * horizon_dt);

    const Eigen::Vector2d velocity_world = bodyToWorld(body_velocity, yaw);
    position.x() += velocity_world.x() * horizon_dt;
    position.y() += velocity_world.y() * horizon_dt;

    const auto guidance = gvf_like->calcLiftedGuidance3D(position, progress_w);
    if (!guidance.valid) {
      ++last_debug_.invalid_reference_count;
      continue;
    }
    if (last_debug_.valid_reference_count == 0) {
      last_debug_.first_reference_world = guidance.v_cmd.template head<2>();
    }
    ++last_debug_.valid_reference_count;
    progress_w = guidance.w_proj + guidance.w_dot * horizon_dt;
    last_debug_.progress_w_end = progress_w;
  }
}

template <typename GvfLikePtr>
B2MpcCommand B2VelocityMpcController::computeWaware(const GvfLikePtr& gvf_like,
                                                    double progress_w0,
                                                    const B2MpcState& state,
                                                    double dt) {
  last_debug_ = B2MpcDebug{};
  last_debug_.progress_w_start = progress_w0;
  last_debug_.progress_w_end = progress_w0;
  if (!gvf_like) {
    reset();
    return B2MpcCommand{};
  }

  const auto first_guidance = gvf_like->calcLiftedGuidance3D(state.position_world, progress_w0);
  if (!first_guidance.valid ||
      first_guidance.v_cmd.template head<2>().norm() <= std::max(0.0, config_.guidance_deadband)) {
    reset();
    return B2MpcCommand{};
  }
  const Eigen::Vector2d first_reference_world = first_guidance.v_cmd.template head<2>();
  last_debug_.first_reference_world = first_reference_world;

  if (!has_last_cmd_) {
    last_cmd_ = B2MpcCommand{};
    has_last_cmd_ = true;
  }

  Sequence sequence = initialSequence(first_reference_world, state, dt);
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

      const double grad = (evaluateWawareCost(plus, gvf_like, progress_w0, state) -
                           evaluateWawareCost(minus, gvf_like, progress_w0, state)) /
                          (2.0 * eps);
      setCommandValue(next, j, commandValue(next, j) - alpha * grad);
    }
    projectSequence(next, dt);
    if (evaluateWawareCost(next, gvf_like, progress_w0, state) <=
        evaluateWawareCost(sequence, gvf_like, progress_w0, state)) {
      sequence = next;
    } else {
      break;
    }
  }

  projectSequence(sequence, dt);
  updateWawareDebug(sequence, gvf_like, progress_w0, state);
  last_solution_ = sequence;
  last_cmd_ = sequence.front();
  return last_cmd_;
}

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_B2_VELOCITY_MPC_H
