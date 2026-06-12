#ifndef BSPLINE_RACE_B2_FOOTPRINT_COLLISION_H
#define BSPLINE_RACE_B2_FOOTPRINT_COLLISION_H

#include <Eigen/Dense>

#include <functional>
#include <vector>

namespace FLAG_Race {

struct B2FootprintConfig {
  bool enable = true;
  double length = 1.10;
  double width = 0.45;
  double margin = 0.05;
  double sample_resolution = 0.15;
};

struct FootprintQueryResult {
  bool in_map = true;
  bool occupied = false;
  bool unknown = false;
  double distance = 1e9;
};

struct FootprintCheckResult {
  bool collision = false;
  bool reliable = true;
  double min_distance = 1e9;
  int total_samples = 0;
  int occupied_samples = 0;
  int unknown_samples = 0;
  int out_of_map_samples = 0;
};

class B2FootprintCollisionChecker {
 public:
  using QueryFn = std::function<FootprintQueryResult(const Eigen::Vector3d&)>;

  B2FootprintCollisionChecker();
  explicit B2FootprintCollisionChecker(const B2FootprintConfig& config);

  void setConfig(const B2FootprintConfig& config);
  const B2FootprintConfig& config() const;

  std::vector<Eigen::Vector3d> sampleFootprint(const Eigen::Vector3d& center,
                                               double yaw) const;
  FootprintCheckResult check(const Eigen::Vector3d& center,
                             double yaw,
                             double collision_threshold,
                             const QueryFn& query) const;

 private:
  static std::vector<double> axisSamples(double half_extent, double resolution);

  B2FootprintConfig config_;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_B2_FOOTPRINT_COLLISION_H
