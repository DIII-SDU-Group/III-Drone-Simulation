#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace iii_drone::simulation::mmwave_fov
{

struct Vec3
{
  double x{};
  double y{};
  double z{};
};

inline Vec3 operator+(const Vec3 & left, const Vec3 & right)
{
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vec3 operator-(const Vec3 & left, const Vec3 & right)
{
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vec3 operator*(const Vec3 & value, const double scalar)
{
  return {value.x * scalar, value.y * scalar, value.z * scalar};
}

inline double Dot(const Vec3 & left, const Vec3 & right)
{
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline double SquaredNorm(const Vec3 & value)
{
  return Dot(value, value);
}

inline double Norm(const Vec3 & value)
{
  return std::sqrt(SquaredNorm(value));
}

enum ActiveBoundary : std::uint16_t
{
  NONE = 0u,
  AZIMUTH_NEGATIVE = 1u << 0u,
  AZIMUTH_POSITIVE = 1u << 1u,
  ELEVATION_NEGATIVE = 1u << 2u,
  ELEVATION_POSITIVE = 1u << 3u,
  MINIMUM_RANGE = 1u << 4u,
  MAXIMUM_RANGE = 1u << 5u,
};

enum class GeneratingGeometryClass : std::uint8_t
{
  ORTHOGONAL_NEAREST = 1u,
  FOV_LIMITED = 2u,
};

struct SensorFov
{
  double minimum_range_m{0.0};
  double maximum_range_m{18.0};
  double azimuth_half_angle_rad{0.6107259643892086};  // atan(0.7)
  double elevation_half_angle_rad{0.6107259643892086};
};

struct SegmentProjection
{
  Vec3 point_sensor;
  double parameter{};
  double range{};
  std::uint16_t active_boundaries{NONE};
  GeneratingGeometryClass geometry_class{GeneratingGeometryClass::ORTHOGONAL_NEAREST};
  double unconstrained_nearest_range{};
  double line_of_sight_tangent_angle_rad{};
};

inline bool IsFiniteFov(const SensorFov & fov)
{
  return std::isfinite(fov.minimum_range_m) && std::isfinite(fov.maximum_range_m) &&
      std::isfinite(fov.azimuth_half_angle_rad) && std::isfinite(fov.elevation_half_angle_rad) &&
      fov.minimum_range_m >= 0.0 && fov.maximum_range_m > fov.minimum_range_m &&
      fov.azimuth_half_angle_rad > 0.0 && fov.azimuth_half_angle_rad < 1.5707963267948966 &&
      fov.elevation_half_angle_rad > 0.0 && fov.elevation_half_angle_rad < 1.5707963267948966;
}

inline bool IsVisible(const Vec3 & point, const SensorFov & fov)
{
  constexpr double epsilon = 1e-10;
  const double range = Norm(point);
  if (!IsFiniteFov(fov) || point.x <= epsilon || range < fov.minimum_range_m - epsilon ||
      range > fov.maximum_range_m + epsilon)
  {
    return false;
  }
  const double azimuth = std::atan2(point.y, point.x);
  const double elevation = std::atan2(point.z, std::hypot(point.x, point.y));
  return std::abs(azimuth) <= fov.azimuth_half_angle_rad + epsilon &&
      std::abs(elevation) <= fov.elevation_half_angle_rad + epsilon;
}

inline std::uint16_t ActiveBoundaries(const Vec3 & point, const SensorFov & fov)
{
  constexpr double angular_tolerance = 1e-7;
  constexpr double range_tolerance = 1e-7;
  std::uint16_t result = NONE;
  const double range = Norm(point);
  const double azimuth = std::atan2(point.y, point.x);
  const double elevation = std::atan2(point.z, std::hypot(point.x, point.y));
  if (std::abs(azimuth + fov.azimuth_half_angle_rad) <= angular_tolerance)
  {
    result |= AZIMUTH_NEGATIVE;
  }
  if (std::abs(azimuth - fov.azimuth_half_angle_rad) <= angular_tolerance)
  {
    result |= AZIMUTH_POSITIVE;
  }
  if (std::abs(elevation + fov.elevation_half_angle_rad) <= angular_tolerance)
  {
    result |= ELEVATION_NEGATIVE;
  }
  if (std::abs(elevation - fov.elevation_half_angle_rad) <= angular_tolerance)
  {
    result |= ELEVATION_POSITIVE;
  }
  if (std::abs(range - fov.minimum_range_m) <= range_tolerance)
  {
    result |= MINIMUM_RANGE;
  }
  if (std::abs(range - fov.maximum_range_m) <= range_tolerance)
  {
    result |= MAXIMUM_RANGE;
  }
  return result;
}

inline void AddRoot(const double coefficient_a, const double coefficient_b,
    const double coefficient_c, std::vector<double> * parameters)
{
  constexpr double epsilon = 1e-12;
  if (std::abs(coefficient_a) <= epsilon)
  {
    if (std::abs(coefficient_b) > epsilon)
    {
      parameters->push_back(-coefficient_c / coefficient_b);
    }
    return;
  }
  const double discriminant = coefficient_b * coefficient_b -
      4.0 * coefficient_a * coefficient_c;
  if (discriminant < -epsilon)
  {
    return;
  }
  const double square_root = std::sqrt(std::max(0.0, discriminant));
  parameters->push_back((-coefficient_b - square_root) / (2.0 * coefficient_a));
  if (square_root > epsilon)
  {
    parameters->push_back((-coefficient_b + square_root) / (2.0 * coefficient_a));
  }
}

inline std::optional<SegmentProjection> ConstrainedClosestPointOnSegment(
    const Vec3 & start, const Vec3 & end, const SensorFov & fov)
{
  constexpr double epsilon = 1e-10;
  if (!IsFiniteFov(fov))
  {
    return std::nullopt;
  }

  const Vec3 direction = end - start;
  const double direction_squared = SquaredNorm(direction);
  if (direction_squared <= epsilon)
  {
    return std::nullopt;
  }

  const double unconstrained_parameter = std::clamp(
      -Dot(start, direction) / direction_squared, 0.0, 1.0);
  const Vec3 unconstrained_point = start + direction * unconstrained_parameter;
  const double unconstrained_range = Norm(unconstrained_point);

  std::vector<double> boundaries{0.0, 1.0, unconstrained_parameter};
  // x = 0, azimuth = +/- half-angle.
  AddRoot(0.0, direction.x, start.x, &boundaries);
  const double azimuth_slope = std::tan(fov.azimuth_half_angle_rad);
  AddRoot(0.0, direction.y - azimuth_slope * direction.x,
      start.y - azimuth_slope * start.x, &boundaries);
  AddRoot(0.0, direction.y + azimuth_slope * direction.x,
      start.y + azimuth_slope * start.x, &boundaries);

  // elevation boundary: z^2 = tan(elevation)^2 * (x^2 + y^2).
  const double elevation_slope = std::tan(fov.elevation_half_angle_rad);
  const double elevation_slope_squared = elevation_slope * elevation_slope;
  AddRoot(
      direction.z * direction.z - elevation_slope_squared *
      (direction.x * direction.x + direction.y * direction.y),
      2.0 * (start.z * direction.z - elevation_slope_squared *
      (start.x * direction.x + start.y * direction.y)),
      start.z * start.z - elevation_slope_squared *
      (start.x * start.x + start.y * start.y), &boundaries);

  // Sphere intersections for both range boundaries.
  for (const double radius : {fov.minimum_range_m, fov.maximum_range_m})
  {
    AddRoot(direction_squared, 2.0 * Dot(start, direction),
        SquaredNorm(start) - radius * radius, &boundaries);
  }

  for (double & value : boundaries)
  {
    value = std::clamp(value, 0.0, 1.0);
  }
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
      [](const double left, const double right) { return std::abs(left - right) <= epsilon; }),
      boundaries.end());

  std::optional<SegmentProjection> best;
  const auto consider = [&](const double parameter) {
    const Vec3 point = start + direction * parameter;
    if (!IsVisible(point, fov))
    {
      return;
    }
    const double range = Norm(point);
    if (best.has_value() && range >= best->range - epsilon)
    {
      return;
    }
    const double tangent_norm = std::sqrt(direction_squared);
    const double cosine = std::clamp(Dot(point, direction) / (range * tangent_norm), -1.0, 1.0);
    SegmentProjection candidate;
    candidate.point_sensor = point;
    candidate.parameter = parameter;
    candidate.range = range;
    candidate.active_boundaries = ActiveBoundaries(point, fov);
    candidate.geometry_class = IsVisible(unconstrained_point, fov) &&
        std::abs(parameter - unconstrained_parameter) <= 1e-8 ?
        GeneratingGeometryClass::ORTHOGONAL_NEAREST :
        GeneratingGeometryClass::FOV_LIMITED;
    candidate.unconstrained_nearest_range = unconstrained_range;
    candidate.line_of_sight_tangent_angle_rad = std::acos(std::abs(cosine));
    best = candidate;
  };

  for (std::size_t index = 0; index < boundaries.size(); ++index)
  {
    consider(boundaries[index]);
    if (index + 1 >= boundaries.size())
    {
      continue;
    }
    const double lower = boundaries[index];
    const double upper = boundaries[index + 1];
    if (upper - lower <= epsilon)
    {
      continue;
    }
    const double midpoint = 0.5 * (lower + upper);
    if (!IsVisible(start + direction * midpoint, fov))
    {
      continue;
    }
    consider(std::clamp(unconstrained_parameter, lower, upper));
  }
  return best;
}

}  // namespace iii_drone::simulation::mmwave_fov
