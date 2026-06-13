#include <gtest/gtest.h>
#include <bspline_race/b2_velocity_mpc.h>

#include <cmath>
#include <memory>

using FLAG_Race::B2MpcConfig;
using FLAG_Race::B2MpcState;
using FLAG_Race::B2VelocityMpcController;

namespace {

B2MpcConfig TestConfig() {
  B2MpcConfig cfg;
  cfg.vx_max = 0.6;
  cfg.vy_max = 0.4;
  cfg.yaw_rate_max = 0.8;
  cfg.acc_x_max = 10.0;
  cfg.acc_y_max = 10.0;
  cfg.yaw_acc_max = 10.0;
  cfg.horizon_steps = 8;
  cfg.dt = 0.1;
  cfg.iterations = 8;
  cfg.velocity_weight = 20.0;
  cfg.heading_weight = 0.0;
  cfg.control_weight = 0.0;
  cfg.control_rate_weight = 0.0;
  return cfg;
}

}  // namespace

TEST(B2VelocityMpcTest, TracksLateralGuidanceWithoutExtraVySuppression) {
  B2VelocityMpcController controller(TestConfig());
  B2MpcState state;
  state.yaw = 0.0;
  state.velocity_world = Eigen::Vector2d::Zero();

  const auto cmd = controller.compute(Eigen::Vector3d(0.0, 1.0, 0.0), state, 0.1);

  EXPECT_NEAR(cmd.vx, 0.0, 1e-3);
  EXPECT_NEAR(cmd.vy, 0.4, 1e-3);
  EXPECT_NEAR(cmd.yaw_rate, 0.0, 1e-3);
}

TEST(B2VelocityMpcTest, AppliesSdkVelocityAndAccelerationLimits) {
  B2MpcConfig cfg = TestConfig();
  cfg.acc_x_max = 0.5;
  cfg.acc_y_max = 0.3;
  cfg.yaw_acc_max = 1.0;
  cfg.heading_weight = 5.0;
  B2VelocityMpcController controller(cfg);
  B2MpcState state;
  state.yaw = 0.0;

  const auto first = controller.compute(Eigen::Vector3d(10.0, 10.0, 0.0), state, 0.1);
  EXPECT_LE(std::abs(first.vx), 0.05 + 1e-6);
  EXPECT_LE(std::abs(first.vy), 0.03 + 1e-6);
  EXPECT_LE(std::abs(first.yaw_rate), 0.1 + 1e-6);

  const auto second = controller.compute(Eigen::Vector3d(10.0, 10.0, 0.0), state, 10.0);
  EXPECT_LE(std::abs(second.vx), 0.6 + 1e-6);
  EXPECT_LE(std::abs(second.vy), 0.4 + 1e-6);
  EXPECT_LE(std::abs(second.yaw_rate), 0.8 + 1e-6);
}

TEST(B2VelocityMpcTest, ConvertsWorldReferenceUsingCurrentYaw) {
  B2VelocityMpcController controller(TestConfig());
  B2MpcState state;
  state.yaw = M_PI / 2.0;

  const auto cmd = controller.compute(Eigen::Vector3d(1.0, 0.0, 0.0), state, 0.1);

  EXPECT_NEAR(cmd.vx, 0.0, 1e-3);
  EXPECT_NEAR(cmd.vy, -0.4, 1e-3);
}

TEST(B2VelocityMpcTest, ZeroGuidancePublishesStopAndResetsRateState) {
  B2MpcConfig cfg = TestConfig();
  cfg.acc_x_max = 0.5;
  B2VelocityMpcController controller(cfg);
  B2MpcState state;

  controller.compute(Eigen::Vector3d(1.0, 0.0, 0.0), state, 1.0);
  const auto stopped = controller.compute(Eigen::Vector3d::Zero(), state, 0.1);
  EXPECT_NEAR(stopped.vx, 0.0, 1e-9);
  EXPECT_NEAR(stopped.vy, 0.0, 1e-9);
  EXPECT_NEAR(stopped.yaw_rate, 0.0, 1e-9);

  const auto restarted = controller.compute(Eigen::Vector3d(1.0, 0.0, 0.0), state, 0.1);
  EXPECT_NEAR(restarted.vx, 0.05, 1e-6);
}


namespace {

struct FakeGuidanceResult {
  Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
  double w_proj = 0.0;
  double w_dot = 0.0;
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  bool valid = false;
};

struct TurnFakeGvf {
  FakeGuidanceResult calcLiftedGuidance3D(const Eigen::Vector3d& pos, double w_prev) const {
    FakeGuidanceResult out;
    out.valid = true;
    out.w_proj = w_prev;
    out.w_dot = 1.0;
    out.v_cmd = (w_prev < 0.25) ? Eigen::Vector3d(1.0, 0.0, 0.0)
                                : Eigen::Vector3d(0.0, 1.0, 0.0);
    out.e_perp = Eigen::Vector3d(0.0, pos.y(), 0.0);
    return out;
  }
};

}  // namespace

TEST(B2VelocityMpcTest, WAwareModeAdvancesDynamicProgressThroughHorizon) {
  B2MpcConfig cfg = TestConfig();
  cfg.horizon_steps = 6;
  cfg.dt = 0.1;
  cfg.heading_weight = 0.0;
  cfg.lateral_error_weight = 0.0;
  B2VelocityMpcController controller(cfg);
  B2MpcState state;
  state.position_world = Eigen::Vector3d::Zero();

  const auto gvf = std::make_shared<TurnFakeGvf>();
  controller.computeWaware(gvf, 0.0, state, 0.1);
  const auto debug = controller.lastDebug();

  EXPECT_NEAR(debug.progress_w_start, 0.0, 1e-9);
  EXPECT_GT(debug.progress_w_end, 0.25);
  EXPECT_NEAR(debug.first_reference_world.x(), 1.0, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
