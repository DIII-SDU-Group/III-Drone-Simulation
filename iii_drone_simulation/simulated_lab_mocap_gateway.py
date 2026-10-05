#!/usr/bin/env python3
"""Publish Gazebo ground truth the way the OptiTrack lab gateway publishes a rigid body.

In the SDU OptiTrack lab a gateway publishes each Motive rigid body as
geometry_msgs/PoseStamped on /body_splitter/body_<id>/pose (best effort,
volatile, keep last 1, ~120 Hz, stamped by the gateway) in a lab-side ROS
domain, and the drone's opti_track_pose_relay feeds that pose to PX4. This
node publishes the same stream from Gazebo ground truth, including the lab
stream's outages, so SIM and HIL can exercise the identical relay path
against a vision-only PX4 SITL.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from iii_drone_simulation.ground_truth_frame_broadcaster import gazebo_enu_pose_to_iii_world


GROUND_TRUTH_ODOMETRY_TOPIC = "/simulation/ground_truth/drone/odometry"
LAB_FRAME_ID = "world"
DEFAULT_RIGID_BODY_ID = 1
DEFAULT_RATE_HZ = 120.0
# One lab gateway watchdog restart: about 8 s without data before the watchdog
# fires plus about 25 s until the restarted gateway publishes again.
DEFAULT_DROPOUT_DURATION_S = 33.0

# The lab gateway's publisher QoS: a late or reconnecting reader receives the
# newest pose only, never a backlog.
LAB_POSE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

NANOSECONDS_PER_SECOND = 1_000_000_000


def lab_pose_topic(rigid_body_id: int) -> str:
    """Topic on which the lab gateway publishes one Motive rigid body."""
    if rigid_body_id < 0:
        raise ValueError(
            f"rigid_body_id must be a non-negative Motive streaming ID, got {rigid_body_id}"
        )
    return f"/body_splitter/body_{rigid_body_id}/pose"


def gazebo_ground_truth_to_lab_pose(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float, float, float, float, float]:
    """Convert a Gazebo ENU/FLU ground-truth pose into the lab pose convention.

    The lab world is Z-up and the rigid body is created with the nose along
    Motive +x, so lab poses carry a forward-left-up body in a Z-up world. The
    simulated lab world is III's world frame (x north, y west, z up). III's
    world is PX4 NED with y and z negated, which is exactly the 180 degree
    rotation about x that opti_track_pose_relay applies, so the relay hands
    PX4 the NED/FRD pose that PX4 SITL simulates for the same Gazebo state.
    """
    return gazebo_enu_pose_to_iii_world(x, y, z, qx, qy, qz, qw)


def seconds_to_ns(seconds: float) -> int:
    return int(round(seconds * NANOSECONDS_PER_SECOND))


def min_publish_interval_ns(rate_hz: float) -> int:
    """Minimum spacing of forwarded ground-truth samples under a rate cap."""
    if not rate_hz > 0.0:
        raise ValueError(f"rate_hz must be positive, got {rate_hz}")
    return seconds_to_ns(1.0 / rate_hz)


def decimation_admits(
    sample_ns: int, last_forwarded_ns: int | None, min_interval_ns: int
) -> bool:
    """Whether a ground-truth sample is forwarded under the rate cap.

    Samples are forwarded on arrival, so every published pose is a distinct
    simulator sample, as every lab pose is a distinct Motive frame. A sample
    is dropped only when it follows the last forwarded one by less than the
    cap's minimum interval, which thins a faster source to a regular cadence.
    A sample older than the last forwarded one means the simulation clock went
    back (world reset) and restarts the cadence.
    """
    if last_forwarded_ns is None or sample_ns < last_forwarded_ns:
        return True
    return sample_ns - last_forwarded_ns >= min_interval_ns


def periodic_dropout_active(elapsed_ns: int, period_ns: int, duration_ns: int) -> bool:
    """Whether a periodic outage pauses publishing elapsed_ns into the schedule.

    Each period is (period - duration) of stream followed by a duration-long
    outage. The stream therefore comes up healthy, so PX4 can align its
    estimator, and the first outage begins period - duration after the
    schedule starts.
    """
    if period_ns <= 0 or duration_ns <= 0 or elapsed_ns < 0:
        return False
    return elapsed_ns % period_ns >= period_ns - duration_ns


class DropoutInjector:
    """Outage schedule of the simulated gateway: periodic plus scripted one-shot."""

    def __init__(
        self,
        period_s: float = 0.0,
        duration_s: float = DEFAULT_DROPOUT_DURATION_S,
    ) -> None:
        if not period_s >= 0.0:
            raise ValueError(f"dropout_period_s must be >= 0 (0 disables), got {period_s}")
        if period_s > 0.0 and not 0.0 < duration_s < period_s:
            raise ValueError(
                f"dropout_duration_s must be positive and shorter than dropout_period_s "
                f"({period_s} s), got {duration_s}"
            )
        self._period_ns = seconds_to_ns(period_s)
        self._duration_ns = seconds_to_ns(duration_s) if period_s > 0.0 else 0
        self._anchor_ns: int | None = None
        self._pending_one_shot_ns: int | None = None
        self._one_shot_window: tuple[int, int] | None = None

    def request_one_shot(self, duration_s: float) -> None:
        """Pause publishing for duration_s from the next sample on; 0 cancels."""
        if not duration_s >= 0.0:
            raise ValueError(f"one_shot_dropout_s must be >= 0, got {duration_s}")
        self._pending_one_shot_ns = seconds_to_ns(duration_s)

    def outage(self, now_ns: int) -> str | None:
        """Name the outage that pauses publishing at now_ns, or None while streaming."""
        if self._anchor_ns is None or now_ns < self._anchor_ns:
            # The schedule starts with the first sample; a backwards clock jump
            # (world reset) restarts it.
            self._anchor_ns = now_ns
        if self._pending_one_shot_ns is not None:
            duration_ns = self._pending_one_shot_ns
            self._pending_one_shot_ns = None
            self._one_shot_window = (now_ns, now_ns + duration_ns) if duration_ns > 0 else None
        if self._one_shot_window is not None:
            start_ns, end_ns = self._one_shot_window
            if start_ns <= now_ns < end_ns:
                return "one-shot"
        if periodic_dropout_active(now_ns - self._anchor_ns, self._period_ns, self._duration_ns):
            return "periodic"
        return None


class SimulatedLabMocapGateway(Node):
    """Publish Gazebo ground truth as the lab gateway publishes a Motive rigid body."""

    def __init__(self, **node_options) -> None:
        super().__init__("simulated_lab_mocap_gateway", **node_options)
        self._lab_context: Context | None = None
        self._lab_node: Node = self

        rigid_body_id = self._declare_read_only(
            "rigid_body_id", DEFAULT_RIGID_BODY_ID, "Motive streaming ID of the rigid body"
        )
        rate_hz = self._declare_read_only(
            "rate_hz", DEFAULT_RATE_HZ, "Cap on the forwarded ground-truth rate"
        )
        dropout_period_s = self._declare_read_only(
            "dropout_period_s", 0.0, "Period of injected stream outages; 0 disables them"
        )
        dropout_duration_s = self._declare_read_only(
            "dropout_duration_s",
            DEFAULT_DROPOUT_DURATION_S,
            "Outage at the end of every dropout period",
        )
        odometry_topic = self._declare_read_only(
            "odometry_topic", GROUND_TRUTH_ODOMETRY_TOPIC, "Gazebo ground-truth odometry topic"
        )
        self._frame_id = self._declare_read_only(
            "frame_id", LAB_FRAME_ID, "header.frame_id of the published poses"
        )
        lab_domain_id = self._declare_read_only(
            "lab_domain_id",
            -1,
            "ROS domain of the published poses; -1 publishes in this process's domain",
        )
        self.declare_parameter(
            "one_shot_dropout_s",
            0.0,
            ParameterDescriptor(
                description="Setting D > 0 pauses publishing for D s from the next sample; "
                "0 cancels"
            ),
        )

        self._topic = lab_pose_topic(rigid_body_id)
        self._min_interval_ns = min_publish_interval_ns(rate_hz)
        self._dropouts = DropoutInjector(dropout_period_s, dropout_duration_s)
        one_shot_dropout_s = float(self.get_parameter("one_shot_dropout_s").value)
        if one_shot_dropout_s != 0.0:
            self._dropouts.request_one_shot(one_shot_dropout_s)
        self._last_forwarded_ns: int | None = None
        self._outage: str | None = None
        self._outage_since_ns = 0

        if lab_domain_id >= 0 and lab_domain_id != self.context.get_domain_id():
            # A participant of its own puts only the gateway topic into the lab
            # domain, as in the lab; ground truth and /clock stay in this domain.
            # Nothing spins the lab node, so it offers no services.
            self._lab_context = Context()
            rclpy.init(args=[], context=self._lab_context, domain_id=lab_domain_id)
            self._lab_node = Node(
                self.get_name(),
                context=self._lab_context,
                enable_rosout=False,
                start_parameter_services=False,
                parameter_overrides=[Parameter("start_type_description_service", value=False)],
            )
        self._publisher = self._lab_node.create_publisher(PoseStamped, self._topic, LAB_POSE_QOS)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)
        self.create_subscription(
            Odometry, odometry_topic, self._on_ground_truth, qos_profile_sensor_data
        )

        periodic = "off"
        if dropout_period_s > 0.0:
            periodic = f"{dropout_duration_s:g} s every {dropout_period_s:g} s"
        self.get_logger().info(
            f"Simulating the lab gateway: {odometry_topic} -> {self._topic} "
            f"(frame '{self._frame_id}', ROS domain {self._lab_node.context.get_domain_id()}, "
            f"rate cap {rate_hz:g} Hz, periodic dropout {periodic})"
        )

    def _declare_read_only(self, name: str, default, description: str):
        return self.declare_parameter(
            name, default, ParameterDescriptor(description=description, read_only=True)
        ).value

    def _validate_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "one_shot_dropout_s" and not float(parameter.value) >= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="one_shot_dropout_s must be >= 0 (0 cancels the one-shot outage)",
                )
        return SetParametersResult(successful=True)

    def _apply_parameters(self, parameters: list[Parameter]) -> None:
        for parameter in parameters:
            if parameter.name == "one_shot_dropout_s":
                self._dropouts.request_one_shot(float(parameter.value))

    def _on_ground_truth(self, message: Odometry) -> None:
        now = self.get_clock().now()
        if now.nanoseconds <= 0:
            # Simulation time has not arrived yet; the lab never stamps zero.
            return
        outage = self._dropouts.outage(now.nanoseconds)
        self._report_outage(outage, now.nanoseconds)
        if outage is not None:
            return
        stamp = message.header.stamp
        sample_ns = stamp.sec * NANOSECONDS_PER_SECOND + stamp.nanosec
        if not decimation_admits(sample_ns, self._last_forwarded_ns, self._min_interval_ns):
            return
        self._last_forwarded_ns = sample_ns

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        lab_pose = gazebo_ground_truth_to_lab_pose(
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = lab_pose[0]
        pose.pose.position.y = lab_pose[1]
        pose.pose.position.z = lab_pose[2]
        pose.pose.orientation.x = lab_pose[3]
        pose.pose.orientation.y = lab_pose[4]
        pose.pose.orientation.z = lab_pose[5]
        pose.pose.orientation.w = lab_pose[6]
        self._publisher.publish(pose)

    def _report_outage(self, outage: str | None, now_ns: int) -> None:
        if outage == self._outage:
            return
        if outage is None:
            lasted_s = (now_ns - self._outage_since_ns) / NANOSECONDS_PER_SECOND
            self.get_logger().info(
                f"Simulated {self._outage} lab outage over after {lasted_s:.1f} s; "
                f"publishing {self._topic} again"
            )
        else:
            self.get_logger().warning(f"Simulated {outage} lab outage: withholding {self._topic}")
        self._outage = outage
        self._outage_since_ns = now_ns

    def destroy_node(self):
        if self._lab_context is not None:
            self._lab_node.destroy_node()
            self._lab_context.try_shutdown()
            self._lab_context = None
            self._lab_node = self
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SimulatedLabMocapGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
