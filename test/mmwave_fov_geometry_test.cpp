#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <random>

#include "iii_drone_simulation/mmwave_fov_geometry.hpp"

namespace geometry = iii_drone::simulation::mmwave_fov;

namespace
{

using geometry::ConstrainedClosestPointOnSegment;
using geometry::GeneratingGeometryClass;
using geometry::IsVisible;
using geometry::SensorFov;
using geometry::Vec3;

const SensorFov kFov{1.0, 10.0, 0.4, 0.3};

// Brute-force reference: the visible segment parameter with the smallest range.
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

TEST(MmwaveFovGeometry, InteriorNearestPointIsOrthogonal)
{
  const auto inside = ConstrainedClosestPointOnSegment({5.0, -2.0, 0.0}, {5.0, 2.0, 0.0}, kFov);
  ASSERT_TRUE(inside.has_value());
  EXPECT_EQ(inside->geometry_class, GeneratingGeometryClass::ORTHOGONAL_NEAREST);
  EXPECT_NEAR(inside->parameter, 0.5, 1e-9);
}

TEST(MmwaveFovGeometry, AngularBoundariesAreExact)
{
  const double azimuth_edge_y = 5.0 * std::tan(kFov.azimuth_half_angle_rad);
  EXPECT_TRUE(ConstrainedClosestPointOnSegment({5.0, azimuth_edge_y - 1e-8, 0.0},
      {5.0, azimuth_edge_y - 1e-8, 0.5}, kFov).has_value());
  EXPECT_FALSE(ConstrainedClosestPointOnSegment({5.0, azimuth_edge_y + 1e-5, 0.0},
      {5.0, azimuth_edge_y + 1e-5, 0.5}, kFov).has_value());

  const double elevation_edge_z = 5.0 * std::tan(kFov.elevation_half_angle_rad);
  EXPECT_TRUE(ConstrainedClosestPointOnSegment({5.0, 0.0, elevation_edge_z - 1e-8},
      {5.5, 0.0, (elevation_edge_z - 1e-8) * 1.1}, kFov).has_value());
  EXPECT_FALSE(ConstrainedClosestPointOnSegment({5.0, 0.0, elevation_edge_z + 1e-5},
      {5.5, 0.0, (elevation_edge_z + 1e-5) * 1.1}, kFov).has_value());

  const auto boundary_tangent = ConstrainedClosestPointOnSegment(
      {5.0, azimuth_edge_y, -2.0}, {5.0, azimuth_edge_y, 2.0}, kFov);
  ASSERT_TRUE(boundary_tangent.has_value());
  EXPECT_NE(boundary_tangent->active_boundaries & geometry::AZIMUTH_POSITIVE, 0u);
}

TEST(MmwaveFovGeometry, FovLimitedSegmentsSelectABoundaryPoint)
{
  const auto az_limited = ConstrainedClosestPointOnSegment({5.0, 5.0, 0.0}, {10.0, 0.0, 0.0}, kFov);
  ASSERT_TRUE(az_limited.has_value());
  EXPECT_EQ(az_limited->geometry_class, GeneratingGeometryClass::FOV_LIMITED);
  EXPECT_NE(az_limited->active_boundaries & geometry::AZIMUTH_POSITIVE, 0u);

  const auto elevation_limited =
    ConstrainedClosestPointOnSegment({5.0, 0.0, 5.0}, {10.0, 0.0, 0.0}, kFov);
  ASSERT_TRUE(elevation_limited.has_value());
  EXPECT_NE(elevation_limited->active_boundaries & geometry::ELEVATION_POSITIVE, 0u);

  // A yaw-misaligned line either selects a boundary point or fails closed; it
  // never masquerades as an orthogonal nearest return.
  EXPECT_TRUE(ConstrainedClosestPointOnSegment({2.0, 0.0, 0.0}, {9.0, 0.0, 0.0}, kFov).has_value());
  const auto yaw_misaligned = ConstrainedClosestPointOnSegment({2.0, 4.0, 0.0}, {9.0, 9.0, 0.0}, kFov);
  EXPECT_TRUE(!yaw_misaligned.has_value() ||
    yaw_misaligned->geometry_class == GeneratingGeometryClass::FOV_LIMITED);

  EXPECT_TRUE(ConstrainedClosestPointOnSegment({3.0, -3.0, -1.0}, {11.0, 2.0, 2.0}, kFov).has_value());
  EXPECT_TRUE(ConstrainedClosestPointOnSegment({5.0, -1.0, 0.0}, {10.0, -1.0, 0.0}, kFov).has_value());
  EXPECT_TRUE(ConstrainedClosestPointOnSegment({5.0, 5.0, 0.0}, {10.0, 0.0, 0.0}, kFov).has_value());
}

TEST(MmwaveFovGeometry, RangeLimitsAndRearGeometryNeverFabricateReturns)
{
  EXPECT_FALSE(ConstrainedClosestPointOnSegment({-5.0, -1.0, 0.0}, {-5.0, 1.0, 0.0}, kFov).has_value());

  const auto min_limited = ConstrainedClosestPointOnSegment({0.5, 0.0, 0.0}, {5.0, 0.0, 0.0}, kFov);
  ASSERT_TRUE(min_limited.has_value());
  EXPECT_NE(min_limited->active_boundaries & geometry::MINIMUM_RANGE, 0u);

  EXPECT_FALSE(ConstrainedClosestPointOnSegment({15.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, kFov).has_value());
}

TEST(MmwaveFovGeometry, SmallSensorRotationsKeepRangeContinuous)
{
  double previous_range = -1.0;
  for (int degree = -15; degree <= 15; ++degree)
  {
    const double angle = static_cast<double>(degree) * M_PI / 180.0;
    const Vec3 start{5.0, 4.0 * std::sin(angle), 4.0 * std::cos(angle)};
    const auto rotation_case = ConstrainedClosestPointOnSegment(start, {10.0, 0.0, 0.0}, kFov);
    ASSERT_TRUE(rotation_case.has_value()) << "degree " << degree;
    if (previous_range >= 0.0)
    {
      EXPECT_LT(std::abs(rotation_case->range - previous_range), 0.2) << "degree " << degree;
    }
    previous_range = rotation_case->range;
  }
}

TEST(MmwaveFovGeometry, AgreesWithDenseOracle)
{
  std::mt19937 generator(20260918u);
  std::uniform_real_distribution<double> distribution(-14.0, 14.0);
  for (int trial = 0; trial < 180; ++trial)
  {
    const Vec3 start{distribution(generator), distribution(generator), distribution(generator)};
    const Vec3 end{distribution(generator), distribution(generator), distribution(generator)};
    const auto result = ConstrainedClosestPointOnSegment(start, end, kFov);
    const auto oracle = DenseOracle(start, end, kFov);
    ASSERT_EQ(result.has_value(), oracle.has_value()) << "trial " << trial;
    if (result.has_value())
    {
      EXPECT_NEAR(result->parameter, oracle.value(), 2e-4) << "trial " << trial;
    }
  }
}
