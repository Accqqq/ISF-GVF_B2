#ifndef BSPLINE_RACE_B2_KINEMATIC_ADAPTER_H
#define BSPLINE_RACE_B2_KINEMATIC_ADAPTER_H

#include <Eigen/Dense>

namespace FLAG_Race {

struct B2KinematicConfig {
  double vx_max = 0.6;
  double vy_max = 0.15;
  double yaw_rate_max = 0.8;
  double acc_x_max = 0.5;
  double acc_y_max = 0.3;
  double yaw_acc_max = 1.0;
  double heading_kp = 1.5;
  double guidance_deadband = 1e-4;
};

struct B2VelocityCommand {
  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
};

class B2KinematicAdapter {
 public:
  B2KinematicAdapter();
  explicit B2KinematicAdapter(const B2KinematicConfig& config);

  void setConfig(const B2KinematicConfig& config);
  const B2KinematicConfig& config() const;

  B2VelocityCommand compute(const Eigen::Vector3d& guidance_world,
                            double yaw,
                            double dt);
  void reset();

  static double wrapAngle(double angle);

 private:
  static double clamp(double value, double lower, double upper);
  static double limitRate(double desired, double previous, double rate_limit, double dt);

  B2KinematicConfig config_;
  B2VelocityCommand last_cmd_;
  bool has_last_cmd_ = false;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_B2_KINEMATIC_ADAPTER_H
