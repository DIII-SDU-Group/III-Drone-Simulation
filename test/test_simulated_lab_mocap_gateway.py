import contextlib
import itertools
import math
import os
import time

import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from iii_drone_simulation.simulated_lab_mocap_gateway import (
    GROUND_TRUTH_ODOMETRY_TOPIC,
    LAB_POSE_QOS,
    DropoutInjector,
    decimation_admits,
    gazebo_ground_truth_to_lab_pose,
    lab_pose_topic,
    min_publish_interval_ns,
    periodic_dropout_active,
)


S = 1_000_000_000
UP = (0.0, 0.0, 1.0)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def _axis_angle(axis, angle):
    half_sin = math.sin(angle / 2.0) / math.sqrt(sum(c * c for c in axis))
    return (axis[0] * half_sin, axis[1] * half_sin, axis[2] * half_sin, math.cos(angle / 2.0))


def _rotate(q, vector):
    x, y, z, _ = _multiply(_multiply(q, (*vector, 0.0)), _conjugate(q))
    return (x, y, z)


def _assert_same_rotation(actual, expected):
    # q and -q are the same rotation.
    assert abs(sum(a * e for a, e in zip(actual, expected))) == pytest.approx(1.0, abs=1e-9)


def _relay_lab_to_ned(pose):
    """opti_track_pose_relay's contract: rotate the lab pose 180 degrees about x."""
    x, y, z, *q = pose
    about_x = (1.0, 0.0, 0.0, 0.0)
    return (x, -y, -z, *_multiply(_multiply(about_x, q), _conjugate(about_x)))


def _px4_sitl_ned(gazebo_pose):
    """PX4 GZBridge's ENU/FLU -> NED/FRD conversion of the same Gazebo pose."""
    x, y, z, *q = gazebo_pose
    enu_to_ned = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
    flu_to_frd = (1.0, 0.0, 0.0, 0.0)
    return (y, x, -z, *_multiply(_multiply(enu_to_ned, q), _conjugate(flu_to_frd)))


def test_lab_pose_topic_names_the_motive_rigid_body():
    assert lab_pose_topic(1) == "/body_splitter/body_1/pose"
    assert lab_pose_topic(17) == "/body_splitter/body_17/pose"
    with pytest.raises(ValueError):
        lab_pose_topic(-1)


def test_lab_pose_qos_matches_the_lab_gateway():
    assert LAB_POSE_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert LAB_POSE_QOS.durability == DurabilityPolicy.VOLATILE
    assert LAB_POSE_QOS.history == HistoryPolicy.KEEP_LAST
    assert LAB_POSE_QOS.depth == 1


def test_level_nose_north_pose_is_the_lab_identity():
    # Gazebo ENU: east -1.4 m, north 2.2 m, up 3 m; a +90 deg ENU yaw is nose north.
    lab = gazebo_ground_truth_to_lab_pose(-1.4, 2.2, 3.0, *_axis_angle(UP, math.pi / 2.0))

    assert lab[:3] == pytest.approx((2.2, 1.4, 3.0))
    _assert_same_rotation(lab[3:], (0.0, 0.0, 0.0, 1.0))
    ned = _relay_lab_to_ned(lab)
    assert ned[:3] == pytest.approx((2.2, -1.4, -3.0))
    _assert_same_rotation(ned[3:], (0.0, 0.0, 0.0, 1.0))


@pytest.mark.parametrize(
    "gazebo_yaw, lab_yaw, lab_forward",
    [
        # Nose west: a quarter turn left, forward along lab +y.
        (math.pi, math.pi / 2.0, (0.0, 1.0, 0.0)),
        # Nose east: a quarter turn right, forward along lab -y.
        (0.0, -math.pi / 2.0, (0.0, -1.0, 0.0)),
    ],
)
def test_quarter_turn_yaw_maps_to_the_lab_forward_axis(gazebo_yaw, lab_yaw, lab_forward):
    lab = gazebo_ground_truth_to_lab_pose(0.0, 0.0, 1.0, *_axis_angle(UP, gazebo_yaw))

    _assert_same_rotation(lab[3:], _axis_angle(UP, lab_yaw))
    assert _rotate(lab[3:], (1.0, 0.0, 0.0)) == pytest.approx(lab_forward, abs=1e-12)
    assert _rotate(lab[3:], UP) == pytest.approx(UP, abs=1e-12)


def test_pitched_pose_keeps_its_nose_up_in_the_lab_and_in_px4():
    pitch = math.radians(20.0)
    # Nose north and raised by 20 deg: a negative turn about the FLU left axis.
    gazebo_q = _multiply(_axis_angle(UP, math.pi / 2.0), _axis_angle((0.0, 1.0, 0.0), -pitch))

    lab = gazebo_ground_truth_to_lab_pose(0.0, 0.0, 1.0, *gazebo_q)

    q = lab[3:]
    assert _rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((math.cos(pitch), 0.0, math.sin(pitch)))
    assert _rotate(q, (0.0, 1.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    assert _rotate(q, UP) == pytest.approx((-math.sin(pitch), 0.0, math.cos(pitch)))
    # PX4 FRD/NED pitch is positive nose up.
    x, y, z, w = _relay_lab_to_ned(lab)[3:]
    assert math.asin(2.0 * (w * y - z * x)) == pytest.approx(pitch)


@pytest.mark.parametrize(
    "position, axis, angle",
    [
        ((0.0, 0.0, 0.0), UP, 0.0),
        ((3.0, -2.0, 1.5), (0.0, 0.0, 1.0), 2.5),
        ((-7.1, 0.4, 12.0), (0.6, -0.8, 0.0), 0.3),
        ((1.0, 2.0, 3.0), (1.0, 2.0, 3.0), -1.9),
    ],
)
def test_relay_conversion_of_the_lab_pose_is_px4_sitl_ground_truth(position, axis, angle):
    gazebo_pose = (*position, *_axis_angle(axis, angle))

    ned = _relay_lab_to_ned(gazebo_ground_truth_to_lab_pose(*gazebo_pose))

    expected = _px4_sitl_ned(gazebo_pose)
    assert ned[:3] == pytest.approx(expected[:3])
    _assert_same_rotation(ned[3:], expected[3:])


def _forwarded(sample_times_ns, rate_hz):
    interval_ns = min_publish_interval_ns(rate_hz)
    last_ns = None
    forwarded = []
    for sample_ns in sample_times_ns:
        if decimation_admits(sample_ns, last_ns, interval_ns):
            last_ns = sample_ns
            forwarded.append(sample_ns)
    return forwarded


def test_rate_cap_forwards_every_gazebo_sample_below_it():
    # Gazebo odometry is throttled to 100 Hz on a 4 ms physics step.
    for spacing_ns in (10_000_000, 12_000_000):
        samples = [k * spacing_ns for k in range(200)]
        assert _forwarded(samples, 120.0) == samples


def test_rate_cap_thins_a_faster_source_to_a_regular_cadence():
    samples = [k * 10_000_000 for k in range(100)]

    forwarded = _forwarded(samples, 30.0)

    assert forwarded == samples[::4]


def test_rate_cap_drops_duplicates_and_restarts_after_a_world_reset():
    interval_ns = min_publish_interval_ns(120.0)

    assert decimation_admits(5 * S, None, interval_ns)
    assert not decimation_admits(5 * S, 5 * S, interval_ns)
    assert decimation_admits(S, 5 * S, interval_ns)
    for rate_hz in (0.0, -120.0, math.nan):
        with pytest.raises(ValueError):
            min_publish_interval_ns(rate_hz)


def test_periodic_dropout_pauses_the_end_of_every_period():
    period_ns, duration_ns = 60 * S, 20 * S

    assert not periodic_dropout_active(0, period_ns, duration_ns)
    assert not periodic_dropout_active(40 * S - 1, period_ns, duration_ns)
    assert periodic_dropout_active(40 * S, period_ns, duration_ns)
    assert periodic_dropout_active(60 * S - 1, period_ns, duration_ns)
    assert not periodic_dropout_active(60 * S, period_ns, duration_ns)
    assert periodic_dropout_active(100 * S, period_ns, duration_ns)
    assert not periodic_dropout_active(-S, period_ns, duration_ns)
    assert not periodic_dropout_active(50 * S, 0, duration_ns)


def test_dropout_schedule_starts_with_the_first_sample_and_restarts_after_a_reset():
    # The default outage is one gateway watchdog restart (8 s + 25 s).
    injector = DropoutInjector(period_s=120.0)
    start_ns = 500 * S

    assert injector.outage(start_ns) is None
    assert injector.outage(start_ns + 87 * S - 1) is None
    assert injector.outage(start_ns + 87 * S) == "periodic"
    assert injector.outage(start_ns + 120 * S - 1) == "periodic"
    assert injector.outage(start_ns + 120 * S) is None
    # A world reset sends simulation time back; the schedule restarts there.
    assert injector.outage(10 * S) is None
    assert injector.outage(96 * S) is None
    assert injector.outage(97 * S) == "periodic"


def test_dropouts_are_off_by_default():
    injector = DropoutInjector()

    assert all(injector.outage(t * S) is None for t in range(0, 3600, 7))


@pytest.mark.parametrize(
    "period_s, duration_s",
    [(-1.0, 10.0), (30.0, 33.0), (30.0, 30.0), (30.0, 0.0), (30.0, -1.0), (math.nan, 10.0)],
)
def test_dropout_schedule_rejects_impossible_outages(period_s, duration_s):
    with pytest.raises(ValueError):
        DropoutInjector(period_s, duration_s)


def test_one_shot_dropout_starts_at_the_next_sample_and_can_be_cancelled():
    injector = DropoutInjector()
    assert injector.outage(100 * S) is None

    injector.request_one_shot(25.0)
    assert injector.outage(101 * S) == "one-shot"
    assert injector.outage(126 * S - 1) == "one-shot"
    assert injector.outage(126 * S) is None

    injector.request_one_shot(90.0)
    assert injector.outage(200 * S) == "one-shot"
    injector.request_one_shot(0.0)
    assert injector.outage(201 * S) is None
    with pytest.raises(ValueError):
        injector.request_one_shot(-1.0)


def test_one_shot_dropout_requested_before_the_stream_covers_its_start():
    injector = DropoutInjector(period_s=120.0, duration_s=33.0)
    injector.request_one_shot(10.0)

    assert injector.outage(50 * S) == "one-shot"
    assert injector.outage(60 * S) is None
    assert injector.outage(137 * S) == "periodic"


@pytest.fixture
def isolated_domains(monkeypatch):
    """Two coordinated ROS domains on this host, never a live SIM/HIL domain."""
    import domain_coordinator

    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    # Devcontainer SIM, HIL and dataset defaults plus any inherited choice.
    reserved = {0, 42, 74}
    for name in ("ROS_DOMAIN_ID", "III_HIL_ROS_DOMAIN_ID", "III_DATASET_ROS_DOMAIN_ID"):
        value = os.environ.get(name, "").strip()
        if value.isdigit():
            reserved.add(int(value))
    candidates = itertools.cycle([d for d in range(1, 101) if d not in reserved])
    with contextlib.ExitStack() as stack:
        yield [
            stack.enter_context(domain_coordinator.domain_id(lambda: next(candidates)))
            for _ in range(2)
        ]


def test_gateway_publishes_ground_truth_into_the_lab_domain(isolated_domains):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock

    from iii_drone_simulation.simulated_lab_mocap_gateway import SimulatedLabMocapGateway

    simulation_domain, lab_domain = isolated_domains
    topic = lab_pose_topic(7)
    simulation_context, lab_context = Context(), Context()
    rclpy.init(args=[], context=simulation_context, domain_id=simulation_domain)
    rclpy.init(args=[], context=lab_context, domain_id=lab_domain)
    nodes, executors = [], []
    try:
        gateway = SimulatedLabMocapGateway(
            context=simulation_context,
            parameter_overrides=[
                Parameter("use_sim_time", value=True),
                Parameter("rigid_body_id", value=7),
                Parameter("lab_domain_id", value=lab_domain),
            ],
        )
        nodes.append(gateway)
        simulator = Node("simulator", context=simulation_context)
        nodes.append(simulator)
        lab_probe = Node("lab_probe", context=lab_context)
        nodes.append(lab_probe)
        clock_publisher = simulator.create_publisher(Clock, "/clock", 10)
        odometry_publisher = simulator.create_publisher(Odometry, GROUND_TRUTH_ODOMETRY_TOPIC, 10)
        received, leaked = [], []
        lab_probe.create_subscription(PoseStamped, topic, received.append, LAB_POSE_QOS)
        simulator.create_subscription(PoseStamped, topic, leaked.append, LAB_POSE_QOS)
        executors.append(SingleThreadedExecutor(context=simulation_context))
        executors.append(SingleThreadedExecutor(context=lab_context))
        executors[0].add_node(gateway)
        executors[0].add_node(simulator)
        executors[1].add_node(lab_probe)

        def spin(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                for executor in executors:
                    executor.spin_once(timeout_sec=0.002)

        # Nose north (ENU yaw +90 deg) at east -1.4 m, north 2.2 m, up 3 m.
        orientation = _axis_angle(UP, math.pi / 2.0)
        sample_ns = itertools.count(start=12_000_000, step=12_000_000)

        def stream(clock_s, count=None, until=None, timeout_s=15.0):
            """Publish simulation time and ground truth until count or until() holds."""
            deadline = time.monotonic() + timeout_s
            for published in itertools.count():
                if published == count or (until is not None and until()):
                    return True
                if time.monotonic() > deadline:
                    return False
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(round(clock_s * S), S)
                clock_publisher.publish(clock)
                odometry = Odometry()
                stamp = odometry.header.stamp
                stamp.sec, stamp.nanosec = divmod(next(sample_ns), S)
                odometry.pose.pose.position.x = -1.4
                odometry.pose.pose.position.y = 2.2
                odometry.pose.pose.position.z = 3.0
                (
                    odometry.pose.pose.orientation.x,
                    odometry.pose.pose.orientation.y,
                    odometry.pose.pose.orientation.z,
                    odometry.pose.pose.orientation.w,
                ) = orientation
                odometry_publisher.publish(odometry)
                spin(0.02)

        assert stream(1234.5, until=lambda: len(received) >= 3), "no pose reached the lab domain"

        pose = received[0]
        assert pose.header.frame_id == "world"
        assert (pose.header.stamp.sec, pose.header.stamp.nanosec) == (1234, 500_000_000)
        assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == pytest.approx(
            (2.2, 1.4, 3.0)
        )
        q = pose.pose.orientation
        _assert_same_rotation((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0))
        assert leaked == []

        def set_parameter(name, value):
            return gateway.set_parameters([Parameter(name, value=value)])[0].successful

        assert not set_parameter("rate_hz", 50.0)
        assert not set_parameter("one_shot_dropout_s", -1.0)
        # The outage starts with the next sample (simulation time 1234.5 s or
        # 1236 s) and lasts 5 s of simulation time.
        assert set_parameter("one_shot_dropout_s", 5.0)
        spin(0.3)
        received.clear()
        assert stream(1236.0, count=30)
        spin(0.3)
        assert received == []

        assert stream(1242.0, until=lambda: len(received) >= 1), "the stream did not resume"
        assert (received[0].header.stamp.sec, received[0].header.stamp.nanosec) == (1242, 0)
    finally:
        for executor in executors:
            executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        simulation_context.try_shutdown()
        lab_context.try_shutdown()
