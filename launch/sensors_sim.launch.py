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
    mmwave_log_level = LaunchConfiguration("mmwave_log_level")

    mmwave_log_level_arg = DeclareLaunchArgument(
        "mmwave_log_level",
        default_value=["info"],
        description="The logging level for the mmwave node, default is INFO",
    )
    
    mmwave = Node(
        package='iii_drone_simulation',
        executable='depth_cam_to_mmwave',
        name='depth_cam_to_mmwave',
        arguments=["--ros-args", "--log-level", mmwave_log_level]
    )

    return LaunchDescription([
        mmwave,
        mmwave_log_level_arg
    ])
