#ifndef BSPLINE_RACE_B2_VELOCITY_MPC_H
#define BSPLINE_RACE_B2_VELOCITY_MPC_H

#include <Eigen/Dense>
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
};

struct B2MpcState {
  Eigen::Vector2d velocity_world = Eigen::Vector2d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct B2MpcCommand {
  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
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
  B2MpcCommand& commandAt(Sequence& sequence, int flat_index);
  double commandValue(const Sequence& sequence, int flat_index) const;
  void setCommandValue(Sequence& sequence, int flat_index, double value);

  B2MpcConfig config_;
  Sequence last_solution_;
  B2MpcCommand last_cmd_;
  bool has_last_cmd_ = false;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_B2_VELOCITY_MPC_H
