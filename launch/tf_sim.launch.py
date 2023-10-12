from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
import yaml

def generate_launch_description():
    config = "/home/" + os.getenv("USER") + "/.config/iii_drone/params.yaml"

    config_dict = yaml.safe_load(open(config,"r").read())

    drone_frame_id = config_dict["/**"]["ros__parameters"]["drone_frame_id"]
    cable_gripper_frame_id = config_dict["/**"]["ros__parameters"]["cable_gripper_frame_id"]
    mmwave_frame_id = config_dict["/**"]["ros__parameters"]["mmwave_frame_id"]
    depth_cam_frame_id = config_dict["/**"]["ros__parameters"]["depth_cam_frame_id"]

    args = [str(val) for val in config_dict["tf"]["sim"]["ros__parameters"]["drone_to_cable_gripper"]] + [drone_frame_id, cable_gripper_frame_id]
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args
    )

    args = [str(val) for val in config_dict["tf"]["sim"]["ros__parameters"]["drone_to_mmwave"]] + [drone_frame_id, mmwave_frame_id]
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args
    )

    args = [str(val) for val in config_dict["tf"]["sim"]["ros__parameters"]["drone_to_depth_cam"]] + [drone_frame_id, depth_cam_frame_id]
    tf_drone_to_depth_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args
    )

    world_to_drone = Node(
        package="iii_drone_core",
        executable="drone_frame_broadcaster",
        parameters=[config]
    )


    return LaunchDescription([
        tf_drone_to_cable_gripper,
        tf_drone_to_iwr,
        tf_drone_to_depth_cam,
        world_to_drone
    ])
