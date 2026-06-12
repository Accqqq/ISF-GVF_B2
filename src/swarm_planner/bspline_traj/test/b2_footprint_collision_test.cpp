#include <gtest/gtest.h>
#include <bspline_race/b2_footprint_collision.h>

#include <cmath>

using FLAG_Race::B2FootprintCollisionChecker;
using FLAG_Race::B2FootprintConfig;
using FLAG_Race::FootprintQueryResult;

namespace {

B2FootprintConfig TestConfig() {
  B2FootprintConfig cfg;
  cfg.length = 1.0;
  cfg.width = 0.4;
  cfg.margin = 0.0;
  cfg.sample_resolution = 0.5;
  return cfg;
}

bool HasPoint(const std::vector<Eigen::Vector3d>& points,
              double x,
              double y,
              double eps = 1e-9) {
  for (const auto& p : points) {
    if (std::abs(p.x() - x) <= eps && std::abs(p.y() - y) <= eps) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(B2FootprintCollisionTest, SamplesCenterAndRectangleEdges) {
  B2FootprintCollisionChecker checker(TestConfig());
  const auto samples = checker.sampleFootprint(Eigen::Vector3d(1.0, 2.0, 0.5), 0.0);

  EXPECT_TRUE(HasPoint(samples, 1.0, 2.0));
  EXPECT_TRUE(HasPoint(samples, 0.5, 1.8));
  EXPECT_TRUE(HasPoint(samples, 1.5, 2.2));
}

TEST(B2FootprintCollisionTest, RotatesFootprintByYaw) {
  B2FootprintCollisionChecker checker(TestConfig());
  const auto samples = checker.sampleFootprint(Eigen::Vector3d::Zero(), M_PI / 2.0);

  EXPECT_TRUE(HasPoint(samples, 0.0, 0.5, 1e-6));
  EXPECT_TRUE(HasPoint(samples, -0.2, 0.0, 1e-6));
}

TEST(B2FootprintCollisionTest, DetectsSideObstacleThatCenterPointWouldMiss) {
  B2FootprintCollisionChecker checker(TestConfig());
  const auto result = checker.check(
      Eigen::Vector3d::Zero(), 0.0, 0.15,
      [](const Eigen::Vector3d& point) {
        FootprintQueryResult query;
        query.distance = std::abs(point.y() - 0.2);
        query.occupied = point.y() > 0.15;
        return query;
      });

  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.occupied_samples, 0);
}

TEST(B2FootprintCollisionTest, UsesMinimumDistanceAcrossFootprint) {
  B2FootprintCollisionChecker checker(TestConfig());
  const auto result = checker.check(
      Eigen::Vector3d::Zero(), 0.0, 0.1,
      [](const Eigen::Vector3d& point) {
        FootprintQueryResult query;
        query.distance = 0.3 + point.x();
        return query;
      });

  EXPECT_NEAR(result.min_distance, -0.2, 1e-9);
  EXPECT_TRUE(result.collision);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
