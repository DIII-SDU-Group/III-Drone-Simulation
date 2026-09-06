#!/usr/bin/env python3
"""Publish the simulation world-to-drone TF from Gazebo ground truth."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def gazebo_enu_pose_to_iii_world(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float, float, float, float, float]:
    """Convert Gazebo ENU/FLU pose into III's north/west/up world frame.

    The production odometry adapter publishes ``world -> drone`` with x=north,
    y=west, and z=up. Gazebo ground truth is x=east, y=north, and z=up, so a
    HIL ground-truth broadcaster must rotate both translation and orientation
    by -90 degrees about world Z. Without this conversion, two physically
    identical cable points differ by a quarter turn during live association.
    """
    half_sqrt_two = math.sqrt(0.5)
    rz = -half_sqrt_two
    rw = half_sqrt_two

    # q_world = q_z(-pi/2) * q_gazebo
    out_qx = rw * qx - rz * qy
    out_qy = rw * qy + rz * qx
    out_qz = rw * qz + rz * qw
    out_qw = rw * qw - rz * qz
    return y, -x, z, out_qx, out_qy, out_qz, out_qw


class GroundTruthFrameBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("drone_frame_broadcaster", namespace="drone_frame_broadcaster")
        self.declare_parameter("odometry_topic", "/simulation/ground_truth/drone/odometry")
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("drone_frame_id", "drone")
        self._world_frame = str(self.get_parameter("world_frame_id").value)
        self._drone_frame = str(self.get_parameter("drone_frame_id").value)
        self._broadcaster = TransformBroadcaster(self)
        self._alive = self.create_publisher(Header, "is_alive", 1)
        self.create_timer(1.0, self._publish_alive)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            qos_profile_sensor_data,
        )

    def _publish_alive(self) -> None:
        message = Header()
        message.stamp = self.get_clock().now().to_msg()
        self._alive.publish(message)

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        converted = gazebo_enu_pose_to_iii_world(
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._world_frame
        transform.child_frame_id = self._drone_frame
        transform.transform.translation.x = converted[0]
        transform.transform.translation.y = converted[1]
        transform.transform.translation.z = converted[2]
        transform.transform.rotation.x = converted[3]
        transform.transform.rotation.y = converted[4]
        transform.transform.rotation.z = converted[5]
        transform.transform.rotation.w = converted[6]
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = GroundTruthFrameBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
