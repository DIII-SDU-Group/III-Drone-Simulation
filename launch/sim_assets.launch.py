from launch import LaunchDescription
from launch_ros.actions import Node

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration

def _resolve_ros_params_file() -> str:
    seed_runtime_configuration("sim")
    return str(resolve_active_parameter_file("sim"))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": True}]

def generate_launch_description():
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
    )
    
    depth_cam_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_cam_gz_bridge',
        arguments=["/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
    )

    mmwave_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='mmwave_gz_bridge',
        arguments=["/sensor/mmwave/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=_parameter_sources(),
    )

    return LaunchDescription([
        clock_gz_bridge,
        camera_gz_bridge,
        depth_cam_gz_bridge,
        mmwave_gz_bridge,
    ])
