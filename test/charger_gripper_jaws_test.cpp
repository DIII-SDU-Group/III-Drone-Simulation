#include <gtest/gtest.h>

#include <iii_drone_simulation/charger_gripper_jaws.hpp>

using gz::math::Vector3d;
using iii_drone::simulation::ChargerGripperJaws;

// A vehicle pushing up presses the slot floor against the cable; the jaws
// carry nothing, so opening them cannot change the vehicle's motion. Within
// the cavity nothing acts.
TEST(ChargerGripperJaws, OnlyTheFloorCarriesAVehiclePushingUp) {
  const ChargerGripperJaws jaws;
  EXPECT_DOUBLE_EQ(jaws.Force(Vector3d(0.0, 0.0, 0.004), Vector3d::Zero, 1.0).Z(), 0.0);
  // Floor: the cable below the seat pushes the vehicle down.
  EXPECT_NEAR(jaws.Force(Vector3d(0.0, 0.0, -0.001), Vector3d::Zero, 1.0).Z(), -4.0, 1e-9);
}

// A vehicle hanging from the cable drops until the cable meets the closed
// jaws, which then hold it up.
TEST(ChargerGripperJaws, HoldAHangingVehicleUp) {
  const ChargerGripperJaws jaws;
  const auto force = jaws.Force(Vector3d(0.0, 0.0, 0.011), Vector3d::Zero, 1.0);
  EXPECT_NEAR(force.Z(), 4000.0 * 0.006, 1e-9);
  // Damped while moving further out of the slot, never pulling down.
  EXPECT_GT(jaws.Force(Vector3d(0.0, 0.0, 0.011), Vector3d(0.0, 0.0, -0.1), 1.0).Z(), force.Z());
  EXPECT_DOUBLE_EQ(jaws.Force(Vector3d(0.0, 0.0, 0.0051), Vector3d(0.0, 0.0, 1.0), 1.0).Z(), 0.0);
}

TEST(ChargerGripperJaws, WallsKeepTheCableInTheSlotAndGripAlongIt) {
  const ChargerGripperJaws jaws;
  EXPECT_DOUBLE_EQ(jaws.Force(Vector3d(0.0, 0.006, 0.002), Vector3d::Zero, 1.0).Y(), 0.0);
  EXPECT_NEAR(jaws.Force(Vector3d(0.0, 0.008, 0.002), Vector3d::Zero, 1.0).Y(), 4.0, 1e-9);
  EXPECT_NEAR(jaws.Force(Vector3d(0.0, -0.008, 0.002), Vector3d::Zero, 1.0).Y(), -4.0, 1e-9);
  EXPECT_NEAR(jaws.Force(Vector3d(0.01, 0.0, 0.002), Vector3d::Zero, 1.0).X(), 6.0, 1e-9);
}

// Friction of the closed jaws damps the vehicle's swing about the cable, and
// nothing else.
TEST(ChargerGripperJaws, DampRotationAboutTheCable) {
  const ChargerGripperJaws jaws;
  const auto torque = jaws.Torque(Vector3d(2.0, 3.0, 4.0), 1.0);
  EXPECT_DOUBLE_EQ(torque.X(), -2.0);
  EXPECT_DOUBLE_EQ(torque.Y(), 0.0);
  EXPECT_DOUBLE_EQ(torque.Z(), 0.0);
}

TEST(ChargerGripperJaws, RampInAndStayBelowTheForceLimit) {
  const ChargerGripperJaws jaws;
  EXPECT_DOUBLE_EQ(jaws.Force(Vector3d(0.0, 0.0, 0.02), Vector3d::Zero, 0.0).Length(), 0.0);
  EXPECT_NEAR(jaws.Force(Vector3d(0.0, 0.0, 0.5), Vector3d::Zero, 1.0).Length(), 250.0, 1e-9);
}
