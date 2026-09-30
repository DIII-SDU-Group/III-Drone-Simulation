#pragma once

#include <algorithm>
#include <cmath>

#include <gz/math/Vector3.hh>

namespace iii_drone::simulation
{

/**
 * The closed charger gripper, in the gripper frame (x along the cable, z out
 * of the slot opening).
 *
 * The real gripper does not latch rigidly: the slot floor and walls are rigid
 * parts of the airframe and the closed jaws only stop the cable from leaving
 * the slot through its opening. A vehicle hanging from the cable rests on the
 * jaws; a vehicle pushing up presses the slot floor against the cable and the
 * jaws carry nothing, so opening them changes nothing.
 *
 * While closed, the cavity (floor, walls, jaws) acts as smooth one-sided
 * constraints on the cable centre, the jaws grip along the cable, and their
 * friction damps the vehicle's rotation about it. Once open, only the
 * airframe's physical collisions (floor, walls) remain.
 */
struct ChargerGripperJaws
{
  /** Cable travel from the seat (resting on the slot floor) to the jaws. */
  double clearance{0.005};
  /** Lateral cable travel from the slot centre to either wall. */
  double wall_clearance{0.007};
  double stiffness{4000.0};
  double damping{150.0};
  double along_stiffness{600.0};
  double along_damping{80.0};
  /** Friction torque per rad/s of rotation about the cable. */
  double rotational_damping{1.0};
  double force_limit{250.0};

  /**
   * Force on the vehicle at the seat point, in the gripper frame.
   *
   * @param cable_from_seat Cable centre relative to the seat point.
   * @param seat_velocity Velocity of the vehicle's seat point (the cable is
   * static).
   * @param engagement 0..1 ramp after the jaws close.
   */
  gz::math::Vector3d Force(
    const gz::math::Vector3d & cable_from_seat,
    const gz::math::Vector3d & seat_velocity,
    double engagement) const
  {
    gz::math::Vector3d force{
      this->along_stiffness * cable_from_seat.X() - this->along_damping * seat_velocity.X(),
      this->Contact(cable_from_seat.Y(), this->wall_clearance, seat_velocity.Y()),
      // Floor below the seat, jaws above it.
      this->Contact(cable_from_seat.Z() - 0.5 * this->clearance, 0.5 * this->clearance, seat_velocity.Z())};
    force *= std::clamp(engagement, 0.0, 1.0);
    const double length = force.Length();
    if (length > this->force_limit) {
      force *= this->force_limit / length;
    }
    return force;
  }

  /**
   * Friction torque on the vehicle, in the gripper frame, opposing its
   * rotation about the cable (x).
   */
  gz::math::Vector3d Torque(const gz::math::Vector3d & angular_velocity, double engagement) const
  {
    return {-this->rotational_damping * angular_velocity.X() * std::clamp(engagement, 0.0, 1.0), 0.0, 0.0};
  }

private:
  /**
   * One-sided contact keeping `offset` within +-`free` of the centre: it
   * pushes the vehicle towards the cable and never pulls.
   */
  double Contact(double offset, double free, double velocity) const
  {
    const double penetration = std::abs(offset) - free;
    if (penetration <= 0.0) {
      return 0.0;
    }
    const double direction = offset > 0.0 ? 1.0 : -1.0;
    return direction * std::max(0.0, this->stiffness * penetration - direction * this->damping * velocity);
  }
};

}  // namespace iii_drone::simulation
