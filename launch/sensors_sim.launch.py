from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration

def _resolve_ros_params_file() -> str:
    seed_runtime_configuration("sim")
    return str(resolve_active_parameter_file("sim"))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": True}]

def generate_launch_description():
    ros_params = _resolve_ros_params_file()

    mmwave_log_level = LaunchConfiguration("mmwave_log_level")

    mmwave_log_level_arg = DeclareLaunchArgument(
        "mmwave_log_level",
        default_value=["info"],
        description="The logging level for the mmwave node, default is INFO",
    )
    
    camera_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_gz_bridge',
        arguments=["/sensor/cable_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
        parameters=_parameter_sources(),
    )
    
    depth_cam_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_cam_gz_bridge',
        arguments=["/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
    )
    
    mmwave = Node(
        package='iii_drone_simulation',
        executable='depth_cam_to_mmwave',
        name='depth_cam_to_mmwave',
        arguments=["--ros-args", "--log-level", mmwave_log_level],
        parameters=_parameter_sources(),
    )

    return LaunchDescription([
        mmwave_log_level_arg,
        mmwave,
        camera_gz_bridge,
        depth_cam_gz_bridge
    ])
