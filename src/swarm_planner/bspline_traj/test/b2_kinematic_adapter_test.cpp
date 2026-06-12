#include <gtest/gtest.h>
#include <bspline_race/b2_kinematic_adapter.h>

#include <cmath>

using FLAG_Race::B2KinematicAdapter;
using FLAG_Race::B2KinematicConfig;

TEST(B2KinematicAdapterTest, ConvertsWorldVelocityToBodyFrame) {
  B2KinematicConfig cfg;
  cfg.vx_max = 2.0;
  cfg.vy_max = 2.0;
  cfg.yaw_rate_max = 2.0;
  cfg.acc_x_max = 100.0;
  cfg.acc_y_max = 100.0;
  cfg.yaw_acc_max = 100.0;
  cfg.heading_kp = 1.0;

  B2KinematicAdapter adapter(cfg);
  const auto cmd = adapter.compute(Eigen::Vector3d(1.0, 0.0, 0.0), M_PI / 2.0, 0.02);

  EXPECT_NEAR(cmd.vx, 0.0, 1e-6);
  EXPECT_NEAR(cmd.vy, -1.0, 1e-6);
}

TEST(B2KinematicAdapterTest, WrapsHeadingErrorAcrossPiBoundary) {
  B2KinematicConfig cfg;
  cfg.vx_max = 2.0;
  cfg.vy_max = 2.0;
  cfg.yaw_rate_max = 10.0;
  cfg.acc_x_max = 100.0;
  cfg.acc_y_max = 100.0;
  cfg.yaw_acc_max = 100.0;
  cfg.heading_kp = 1.0;

  B2KinematicAdapter adapter(cfg);
  const double yaw = M_PI - 0.05;
  const Eigen::Vector3d vg(std::cos(-M_PI + 0.05), std::sin(-M_PI + 0.05), 0.0);
  const auto cmd = adapter.compute(vg, yaw, 0.02);

  EXPECT_NEAR(cmd.yaw_rate, 0.1, 1e-6);
}

TEST(B2KinematicAdapterTest, AppliesVelocityAndAccelerationLimits) {
  B2KinematicConfig cfg;
  cfg.vx_max = 0.6;
  cfg.vy_max = 0.15;
  cfg.yaw_rate_max = 0.8;
  cfg.acc_x_max = 0.5;
  cfg.acc_y_max = 0.3;
  cfg.yaw_acc_max = 1.0;
  cfg.heading_kp = 10.0;

  B2KinematicAdapter adapter(cfg);
  const auto first = adapter.compute(Eigen::Vector3d(10.0, 10.0, 0.0), 0.0, 0.1);

  EXPECT_NEAR(first.vx, 0.05, 1e-6);
  EXPECT_NEAR(first.vy, 0.03, 1e-6);
  EXPECT_NEAR(first.yaw_rate, 0.1, 1e-6);

  const auto second = adapter.compute(Eigen::Vector3d(10.0, 10.0, 0.0), 0.0, 10.0);
  EXPECT_NEAR(second.vx, 0.6, 1e-6);
  EXPECT_NEAR(second.vy, 0.15, 1e-6);
  EXPECT_NEAR(second.yaw_rate, 0.8, 1e-6);
}

TEST(B2KinematicAdapterTest, ZeroGuidanceStopsCommandAndResetsRateState) {
  B2KinematicConfig cfg;
  cfg.acc_x_max = 0.5;
  cfg.acc_y_max = 0.3;
  cfg.yaw_acc_max = 1.0;
  B2KinematicAdapter adapter(cfg);

  adapter.compute(Eigen::Vector3d(10.0, 0.0, 0.0), 0.0, 1.0);
  const auto stopped = adapter.compute(Eigen::Vector3d::Zero(), 0.0, 0.02);
  EXPECT_NEAR(stopped.vx, 0.0, 1e-6);
  EXPECT_NEAR(stopped.vy, 0.0, 1e-6);
  EXPECT_NEAR(stopped.yaw_rate, 0.0, 1e-6);

  const auto restarted = adapter.compute(Eigen::Vector3d(10.0, 0.0, 0.0), 0.0, 0.1);
  EXPECT_NEAR(restarted.vx, 0.05, 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
