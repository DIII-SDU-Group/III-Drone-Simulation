from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    drone_frame_broadcaster_log_level = LaunchConfiguration("drone_frame_broadcaster_log_level")

    drone_frame_broadcaster_log_level_arg = DeclareLaunchArgument(
        "drone_frame_broadcaster_log_level",
        default_value=["info"],
        description="The logging level for the drone frame broadcaster node, default is INFO",
    )
    
    iii_config_dir = os.path.join(os.path.expanduser(os.getenv("CONFIG_BASE_DIR", default="~/.config")), "iii_drone")
    ros_params = os.path.join(iii_config_dir, "ros_params_sim.yaml")
    with open(ros_params, "r") as file:
        ros_params_dict = yaml.safe_load(file) or {}
    params = ros_params_dict["/**"]["ros__parameters"]

    drone_frame_id = params["/tf/drone_frame_id"]
    cable_gripper_frame_id = params["/tf/cable_gripper_frame_id"]
    mmwave_frame_id = params["/tf/mmwave_frame_id"]
    depth_cam_frame_id = params["/tf/sim/depth_cam_frame_id"]

    args = [str(val) for val in params["/tf/sim/drone_to_cable_gripper"]] + [drone_frame_id, cable_gripper_frame_id]
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=[ros_params],
    )

    args = [str(val) for val in params["/tf/sim/drone_to_mmwave"]] + [drone_frame_id, mmwave_frame_id]
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=[ros_params],
    )

    args = [str(val) for val in params["/tf/sim/drone_to_depth_cam"]] + [drone_frame_id, depth_cam_frame_id]
    tf_drone_to_depth_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=[ros_params],
    )

    world_to_drone = Node(
        package="iii_drone_core",
        executable="drone_frame_broadcaster",
        arguments=["--ros-args", "--log-level", drone_frame_broadcaster_log_level],
        parameters=[ros_params],
    )

    return LaunchDescription([
        drone_frame_broadcaster_log_level_arg,
        tf_drone_to_cable_gripper,
        tf_drone_to_iwr,
        tf_drone_to_depth_cam,
        world_to_drone,
    ])
