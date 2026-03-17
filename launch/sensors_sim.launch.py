from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

def generate_launch_description():
    iii_config_dir = os.path.join(os.path.expanduser(os.getenv("CONFIG_BASE_DIR", default="~/.config")), "iii_drone")
    ros_params = os.path.join(iii_config_dir, "ros_params_sim.yaml")

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
        parameters=[ros_params],
    )
    
    depth_cam_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_cam_gz_bridge',
        arguments=["/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
        parameters=[ros_params],
    )
    
    mmwave = Node(
        package='iii_drone_simulation',
        executable='depth_cam_to_mmwave',
        name='depth_cam_to_mmwave',
        arguments=["--ros-args", "--log-level", mmwave_log_level],
        parameters=[ros_params],
    )

    return LaunchDescription([
        mmwave_log_level_arg,
        mmwave,
        camera_gz_bridge,
        depth_cam_gz_bridge
    ])
