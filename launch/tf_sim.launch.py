from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
import yaml

def generate_launch_description():
    drone_frame_broadcaster_log_level = LaunchConfiguration("drone_frame_broadcaster_log_level")

    drone_frame_broadcaster_log_level_arg = DeclareLaunchArgument(
        "drone_frame_broadcaster_log_level",
        default_value=["info"],
        description="The logging level for the drone frame broadcaster node, default is INFO",
    )
    
    iii_config_dir = os.path.join(os.getenv("CONFIG_BASE_DIR", default="~/.config"), "iii_drone")
    ros_params = os.path.join(iii_config_dir, "ros_params.yaml")
    ros_params_dict = yaml.safe_load(open(ros_params,"r").read())
    parameters_dir = os.path.join(iii_config_dir, ros_params_dict["/**"]["ros__parameters"]["parameters_path_postfix"])
    default_parameter_file = ros_params_dict["/**"]["ros__parameters"]["default_parameter_file"]
    parameters_file = os.path.join(parameters_dir, default_parameter_file)

    # Replace "~" with "/home/<user>" in the path
    if parameters_file[0] == "~":
        parameters_file = "/home/" + os.getenv("USER") + parameters_file[1:]
    
    params_dict = yaml.safe_load(open(parameters_file,"r").read())

    drone_frame_id = params_dict["tf"]["drone_frame_id"]["value"]
    cable_gripper_frame_id = params_dict["tf"]["cable_gripper_frame_id"]["value"]
    mmwave_frame_id = params_dict["tf"]["mmwave_frame_id"]["value"]
    depth_cam_frame_id = params_dict["tf"]["sim"]["depth_cam_frame_id"]["value"]

    args = [str(val) for val in params_dict["tf"]["sim"]["drone_to_cable_gripper"]["value"]] + [drone_frame_id, cable_gripper_frame_id]
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=[ros_params],
    )

    args = [str(val) for val in params_dict["tf"]["sim"]["drone_to_mmwave"]["value"]] + [drone_frame_id, mmwave_frame_id]
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=[ros_params],
    )

    args = [str(val) for val in params_dict["tf"]["sim"]["drone_to_depth_cam"]["value"]] + [drone_frame_id, depth_cam_frame_id]
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
