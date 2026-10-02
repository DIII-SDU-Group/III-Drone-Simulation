#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <string>

#include "iii_drone_simulation/mmwave_fov_geometry.hpp"

namespace geometry = iii_drone::simulation::mmwave_fov;

namespace
{

using geometry::ConstrainedClosestPointOnSegment;
using geometry::GeneratingGeometryClass;
using geometry::IsVisible;
using geometry::SensorFov;
using geometry::Vec3;

void Require(const bool condition, const std::string & message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

std::optional<double> DenseOracle(const Vec3 & start, const Vec3 & end, const SensorFov & fov)
{
  constexpr int samples = 200001;
  const Vec3 direction{end.x - start.x, end.y - start.y, end.z - start.z};
  std::optional<double> best;
  double best_range = INFINITY;
  for (int index = 0; index < samples; ++index)
  {
    const double t = static_cast<double>(index) / static_cast<double>(samples - 1);
    const Vec3 point{start.x + direction.x * t, start.y + direction.y * t,
        start.z + direction.z * t};
    if (!IsVisible(point, fov))
    {
      continue;
    }
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range < best_range)
    {
      best_range = range;
      best = t;
    }
  }
  return best;
}

}  // namespace

int main()
{
  const SensorFov fov{1.0, 10.0, 0.4, 0.3};
  const auto inside = ConstrainedClosestPointOnSegment({5.0, -2.0, 0.0}, {5.0, 2.0, 0.0}, fov);
  Require(inside.has_value(), "inside-FOV segment must return a point");
  Require(inside->geometry_class == GeneratingGeometryClass::ORTHOGONAL_NEAREST,
      "interior nearest point must be orthogonal-nearest");
  Require(std::abs(inside->parameter - 0.5) < 1e-9, "interior nearest parameter");

  const double azimuth_edge_y = 5.0 * std::tan(fov.azimuth_half_angle_rad);
  Require(ConstrainedClosestPointOnSegment({5.0, azimuth_edge_y - 1e-8, 0.0},
      {5.0, azimuth_edge_y - 1e-8, 0.5}, fov).has_value(),
      "point just inside azimuth boundary");
  Require(!ConstrainedClosestPointOnSegment({5.0, azimuth_edge_y + 1e-5, 0.0},
      {5.0, azimuth_edge_y + 1e-5, 0.5}, fov).has_value(),
      "point just outside azimuth boundary");
  const double elevation_edge_z = 5.0 * std::tan(fov.elevation_half_angle_rad);
  Require(ConstrainedClosestPointOnSegment({5.0, 0.0, elevation_edge_z - 1e-8},
      {5.5, 0.0, (elevation_edge_z - 1e-8) * 1.1}, fov).has_value(),
      "point just inside elevation boundary");
  Require(!ConstrainedClosestPointOnSegment({5.0, 0.0, elevation_edge_z + 1e-5},
      {5.5, 0.0, (elevation_edge_z + 1e-5) * 1.1}, fov).has_value(),
      "point just outside elevation boundary");

  const auto az_limited = ConstrainedClosestPointOnSegment({5.0, 5.0, 0.0}, {10.0, 0.0, 0.0}, fov);
  Require(az_limited.has_value(), "visible azimuth-limited segment must not disappear");
  Require(az_limited->geometry_class == GeneratingGeometryClass::FOV_LIMITED,
      "azimuth-limited return classification");
  Require((az_limited->active_boundaries & geometry::AZIMUTH_POSITIVE) != 0u,
      "azimuth-limited active boundary");

  const auto elevation_limited = ConstrainedClosestPointOnSegment({5.0, 0.0, 5.0}, {10.0, 0.0, 0.0}, fov);
  Require(elevation_limited.has_value(), "visible elevation-limited segment must not disappear");
  Require((elevation_limited->active_boundaries & geometry::ELEVATION_POSITIVE) != 0u,
      "elevation-limited active boundary");

  Require(!ConstrainedClosestPointOnSegment({-5.0, -1.0, 0.0}, {-5.0, 1.0, 0.0}, fov).has_value(),
      "segment behind radar must not fabricate a return");
  const auto min_limited = ConstrainedClosestPointOnSegment({0.5, 0.0, 0.0}, {5.0, 0.0, 0.0}, fov);
  Require(min_limited.has_value() && (min_limited->active_boundaries & geometry::MINIMUM_RANGE) != 0u,
      "minimum-range clipping");
  Require(!ConstrainedClosestPointOnSegment({15.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, fov).has_value(),
      "maximum-range clipping must not fabricate a return");

  const auto boundary_tangent = ConstrainedClosestPointOnSegment(
      {5.0, azimuth_edge_y, -2.0}, {5.0, azimuth_edge_y, 2.0}, fov);
  Require(boundary_tangent.has_value() &&
      (boundary_tangent->active_boundaries & geometry::AZIMUTH_POSITIVE) != 0u,
      "conductor tangent to FOV boundary");
  Require(ConstrainedClosestPointOnSegment({3.0, -3.0, -1.0}, {11.0, 2.0, 2.0}, fov).has_value(),
      "oblique conductor geometry");

  // A nominal forward-aligned line remains visible; rotating it beyond the
  // configured yaw envelope must either select a boundary point or fail closed.
  Require(ConstrainedClosestPointOnSegment({2.0, 0.0, 0.0}, {9.0, 0.0, 0.0}, fov).has_value(),
      "nominal forward-aligned flight geometry");
  const auto yaw_misaligned = ConstrainedClosestPointOnSegment(
      {2.0, 4.0, 0.0}, {9.0, 9.0, 0.0}, fov);
  Require(!yaw_misaligned.has_value() ||
      yaw_misaligned->geometry_class == GeneratingGeometryClass::FOV_LIMITED,
      "yaw-misaligned geometry must not masquerade as an orthogonal nearest return");

  const auto first_conductor = ConstrainedClosestPointOnSegment(
      {5.0, -1.0, 0.0}, {10.0, -1.0, 0.0}, fov);
  const auto second_conductor = ConstrainedClosestPointOnSegment(
      {5.0, 5.0, 0.0}, {10.0, 0.0, 0.0}, fov);
  Require(first_conductor.has_value() && second_conductor.has_value(),
      "multiple conductors must enter FOV independently");

  double previous_range = -1.0;
  for (int degree = -15; degree <= 15; ++degree)
  {
    const double angle = static_cast<double>(degree) * 3.14159265358979323846 / 180.0;
    const Vec3 start{5.0, 4.0 * std::sin(angle), 4.0 * std::cos(angle)};
    const Vec3 end{10.0, 0.0, 0.0};
    const auto rotation_case = ConstrainedClosestPointOnSegment(start, end, fov);
    Require(rotation_case.has_value(), "small sensor rotation continuity visibility");
    if (previous_range >= 0.0)
    {
      Require(std::abs(rotation_case->range - previous_range) < 0.2,
          "small sensor rotations must not cause a non-topological range jump");
    }
    previous_range = rotation_case->range;
  }

  std::mt19937 generator(20260918u);
  std::uniform_real_distribution<double> distribution(-14.0, 14.0);
  for (int trial = 0; trial < 180; ++trial)
  {
    const Vec3 start{distribution(generator), distribution(generator), distribution(generator)};
    const Vec3 end{distribution(generator), distribution(generator), distribution(generator)};
    const auto result = ConstrainedClosestPointOnSegment(start, end, fov);
    const auto oracle = DenseOracle(start, end, fov);
    Require(result.has_value() == oracle.has_value(), "constrained-oracle visibility agreement");
    if (result.has_value())
    {
      Require(std::abs(result->parameter - oracle.value()) < 2e-4,
          "constrained-oracle nearest parameter agreement");
    }
  }

  std::cout << "mmwave_fov_geometry: PASS (analytic boundaries plus 180 independent dense-oracle sweeps)" << std::endl;
  return EXIT_SUCCESS;
}
