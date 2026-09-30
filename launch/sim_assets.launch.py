from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration


HIL_RATE_LIMITED_CAMERA_TOPIC = "/simulation/local/cable_camera/image_rate_limited"

def _resolve_ros_params_file() -> str:
    seed_runtime_configuration("sim")
    return str(resolve_active_parameter_file("sim"))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": True}]

def generate_launch_description():
    include_diagnostics = LaunchConfiguration("include_diagnostics")
    use_camera_rate_limiter = LaunchConfiguration("use_camera_rate_limiter")
    camera_output_topic = LaunchConfiguration("camera_output_topic")
    camera_rate_hz = LaunchConfiguration("camera_rate_hz")
    include_diagnostics_arg = DeclareLaunchArgument(
        "include_diagnostics",
        default_value="true",
        description="Bridge high-bandwidth diagnostic streams in addition to mission sensor inputs",
    )
    use_camera_rate_limiter_arg = DeclareLaunchArgument(
        "use_camera_rate_limiter",
        default_value="false",
        description="Bound camera traffic before it crosses a split-host HIL link",
    )
    camera_output_topic_arg = DeclareLaunchArgument(
        "camera_output_topic",
        default_value="/sensor/cable_camera/image_raw",
        description="ROS output topic used by the Gazebo camera bridge",
    )
    camera_rate_hz_arg = DeclareLaunchArgument(
        "camera_rate_hz",
        default_value="2.5",
        description="Maximum camera frames per second when the HIL relay is enabled",
    )
    clock_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_gz_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=_parameter_sources(),
    )

    camera_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_gz_bridge',
        arguments=["/sensor/cable_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
        parameters=_parameter_sources(),
        remappings=[("/sensor/cable_camera/image_raw", camera_output_topic)],
    )

    camera_rate_limiter = Node(
        package="iii_drone_simulation",
        executable="rate_limited_image_relay",
        name="camera_rate_limiter",
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                "input_topic": camera_output_topic,
                # Stays on the workstation: only the compressed stream below
                # crosses the split-host link.
                "output_topic": HIL_RATE_LIMITED_CAMERA_TOPIC,
                "max_hz": camera_rate_hz,
            }
        ],
        condition=IfCondition(use_camera_rate_limiter),
    )
    
    # Consumers read /sensor/cable_camera/image_raw/compressed: lossless PNG,
    # ~7 KB per simulated frame instead of ~920 KB raw. Raw frames sent
    # best-effort across the HIL link lost ~25% of frames (a frame is lost with
    # any of its ~15 UDP fragments), which starved cable detection. SIM
    # publishes the same stream so SIM and HIL run identical consumers.
    def camera_compressor(input_topic, condition):
        return Node(
            package="image_transport",
            executable="republish",
            name="camera_compressor",
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                {
                    "in_transport": "raw",
                    "out_transport": "compressed",
                    "out.compressed.format": "png",
                    "out.compressed.png_level": 3,
                    "qos_overrides." + input_topic + ".subscription.reliability": "best_effort",
                }
            ],
            remappings=[
                ("in", input_topic),
                ("out/compressed", "/sensor/cable_camera/image_raw/compressed"),
            ],
            condition=condition,
        )

    hil_camera_compressor = camera_compressor(
        HIL_RATE_LIMITED_CAMERA_TOPIC, IfCondition(use_camera_rate_limiter))
    sim_camera_compressor = camera_compressor(
        "/sensor/cable_camera/image_raw", UnlessCondition(use_camera_rate_limiter))

    depth_cam_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_cam_gz_bridge',
        arguments=["/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
        condition=IfCondition(include_diagnostics),
    )

    mmwave_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='mmwave_gz_bridge',
        arguments=["/sensor/mmwave/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
    )

    mmwave_full_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='mmwave_full_gz_bridge',
        arguments=["/sensor/mmwave/points_full@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
        condition=IfCondition(include_diagnostics),
    )

    ground_truth_odometry_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_odometry_gz_bridge',
        arguments=["/simulation/ground_truth/drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        parameters=_parameter_sources(),
    )

    mmwave_labels_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='mmwave_labels_gz_bridge',
        arguments=["/simulation/ground_truth/mmwave/conductor_labels@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
        condition=IfCondition(include_diagnostics),
    )

    conductor_id_map_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='conductor_id_map_gz_bridge',
        arguments=["/simulation/ground_truth/conductor_id_map@std_msgs/msg/String[gz.msgs.StringMsg"],
        parameters=_parameter_sources(),
        condition=IfCondition(include_diagnostics),
    )

    return LaunchDescription([
        include_diagnostics_arg,
        use_camera_rate_limiter_arg,
        camera_output_topic_arg,
        camera_rate_hz_arg,
        clock_gz_bridge,
        camera_gz_bridge,
        camera_rate_limiter,
        hil_camera_compressor,
        sim_camera_compressor,
        depth_cam_gz_bridge,
        mmwave_gz_bridge,
        mmwave_full_gz_bridge,
        ground_truth_odometry_gz_bridge,
        mmwave_labels_gz_bridge,
        conductor_id_map_gz_bridge,
    ])
