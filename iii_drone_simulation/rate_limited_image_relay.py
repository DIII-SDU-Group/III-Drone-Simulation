#!/usr/bin/env python3
"""Keep the newest camera frame while bounding cross-host HIL traffic."""

from __future__ import annotations

import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class RateLimitedImageRelay(Node):
    def __init__(self) -> None:
        super().__init__("rate_limited_image_relay")
        self.declare_parameter("input_topic", "/simulation/local/cable_camera/image_raw")
        self.declare_parameter("output_topic", "/sensor/cable_camera/image_raw")
        self.declare_parameter("max_hz", 2.5)
        self.declare_parameter("input_stale_timeout_s", 3.0)
        self._input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        max_hz = float(self.get_parameter("max_hz").value)
        self._input_stale_timeout = float(
            self.get_parameter("input_stale_timeout_s").value
        )
        if max_hz <= 0.0:
            raise ValueError("max_hz must be positive")
        if self._input_stale_timeout <= 0.0:
            raise ValueError("input_stale_timeout_s must be positive")

        self._minimum_period = 1.0 / max_hz
        self._last_published_at = float("-inf")
        self._last_received_at = time.monotonic()
        # A camera relay only needs the newest sample.  Depth one prevents a
        # large image backlog from consuming DDS writer/reader resources when
        # the Pi graph is restarted or temporarily unavailable.
        self._image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Image, output_topic, self._image_qos)
        self._subscription = self._create_subscription()
        # A CycloneDDS reader can remain discovered yet stop delivering after
        # split-host graph churn.  Recreating the reader inside the same process
        # is insufficient in that failure mode because the participant itself
        # is stale.  Fail the relay process instead; the launch description
        # respawns it with a fresh participant while Gazebo and its bridge keep
        # running.  System time is deliberate: recovery must also work while
        # simulation time is paused.
        self._watchdog = self.create_timer(
            min(1.0, self._input_stale_timeout / 2.0),
            self._fail_on_stale_input,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

    def _create_subscription(self):
        return self.create_subscription(
            Image, self._input_topic, self._on_image, self._image_qos
        )

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        self._last_received_at = now
        if now - self._last_published_at < self._minimum_period:
            return
        self._last_published_at = now
        self._publisher.publish(message)

    def _fail_on_stale_input(self) -> None:
        now = time.monotonic()
        stale_for = now - self._last_received_at
        if stale_for < self._input_stale_timeout:
            return
        message = (
            f"No camera input for {stale_for:.1f}s; restarting the relay DDS participant"
        )
        self.get_logger().error(message)
        raise RuntimeError(message)


def main() -> None:
    rclpy.init()
    node = RateLimitedImageRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
