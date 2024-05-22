from struct import pack
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ros_params = "/home/" + os.getenv("USER") + "/.config/iii_drone/ros_params.yaml"

    mmwave_log_level = LaunchConfiguration("mmwave_log_level")

    mmwave_log_level_arg = DeclareLaunchArgument(
        "mmwave_log_level",
        default_value=["info"],
        description="The logging level for the mmwave node, default is INFO",
    )
    
    camera_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/sensor/cable_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
        parameters=[ros_params],
    )
    
    depth_cam_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
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
