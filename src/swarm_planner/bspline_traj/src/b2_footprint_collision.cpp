#include <bspline_race/b2_footprint_collision.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace FLAG_Race {

B2FootprintCollisionChecker::B2FootprintCollisionChecker() = default;

B2FootprintCollisionChecker::B2FootprintCollisionChecker(const B2FootprintConfig& config)
    : config_(config) {}

void B2FootprintCollisionChecker::setConfig(const B2FootprintConfig& config) {
  config_ = config;
}

const B2FootprintConfig& B2FootprintCollisionChecker::config() const { return config_; }

std::vector<double> B2FootprintCollisionChecker::axisSamples(double half_extent,
                                                            double resolution) {
  half_extent = std::max(0.0, half_extent);
  resolution = std::max(1e-3, resolution);

  std::vector<double> values;
  values.push_back(-half_extent);

  for (double v = -half_extent + resolution; v < half_extent - 1e-9; v += resolution) {
    values.push_back(v);
  }

  if (half_extent > 1e-9) {
    values.push_back(0.0);
    values.push_back(half_extent);
  }

  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end(), [](double a, double b) {
                 return std::abs(a - b) < 1e-9;
               }),
               values.end());
  return values;
}

std::vector<Eigen::Vector3d> B2FootprintCollisionChecker::sampleFootprint(
    const Eigen::Vector3d& center,
    double yaw) const {
  const double half_length = 0.5 * std::max(0.0, config_.length) + std::max(0.0, config_.margin);
  const double half_width = 0.5 * std::max(0.0, config_.width) + std::max(0.0, config_.margin);
  const auto xs = axisSamples(half_length, config_.sample_resolution);
  const auto ys = axisSamples(half_width, config_.sample_resolution);

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<Eigen::Vector3d> samples;
  samples.reserve(xs.size() * ys.size());

  for (const double x : xs) {
    for (const double y : ys) {
      Eigen::Vector3d p = center;
      p.x() += c * x - s * y;
      p.y() += s * x + c * y;
      samples.push_back(p);
    }
  }
  return samples;
}

FootprintCheckResult B2FootprintCollisionChecker::check(const Eigen::Vector3d& center,
                                                        double yaw,
                                                        double collision_threshold,
                                                        const QueryFn& query) const {
  FootprintCheckResult result;
  if (!config_.enable) {
    const auto q = query(center);
    result.total_samples = 1;
    result.reliable = q.in_map && !q.unknown;
    result.min_distance = q.distance;
    result.occupied_samples = q.occupied ? 1 : 0;
    result.unknown_samples = q.unknown ? 1 : 0;
    result.out_of_map_samples = q.in_map ? 0 : 1;
    result.collision = q.occupied || q.distance < collision_threshold;
    return result;
  }

  const auto samples = sampleFootprint(center, yaw);
  result.total_samples = static_cast<int>(samples.size());
  result.min_distance = std::numeric_limits<double>::infinity();

  for (const auto& p : samples) {
    const auto q = query(p);
    if (!q.in_map) {
      result.reliable = false;
      result.out_of_map_samples++;
      continue;
    }
    if (q.unknown) {
      result.reliable = false;
      result.unknown_samples++;
    }

    result.min_distance = std::min(result.min_distance, q.distance);
    if (q.occupied) {
      result.occupied_samples++;
      result.collision = true;
    }
    if (q.distance < collision_threshold) {
      result.collision = true;
    }
  }

  if (!std::isfinite(result.min_distance)) {
    result.min_distance = std::numeric_limits<double>::infinity();
  }
  return result;
}

}  // namespace FLAG_Race
