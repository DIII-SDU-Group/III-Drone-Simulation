from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration


def _resolve_ros_params_file() -> str:
    seed_runtime_configuration("sim")
    return str(resolve_active_parameter_file("sim"))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": True}]

def generate_launch_description():
    drone_frame_broadcaster_log_level = LaunchConfiguration("drone_frame_broadcaster_log_level")

    drone_frame_broadcaster_log_level_arg = DeclareLaunchArgument(
        "drone_frame_broadcaster_log_level",
        default_value=["info"],
        description="The logging level for the drone frame broadcaster node, default is INFO",
    )
    
    ros_params = _resolve_ros_params_file()
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
        parameters=_parameter_sources(),
    )

    args = [str(val) for val in params["/tf/sim/drone_to_mmwave"]] + [drone_frame_id, mmwave_frame_id]
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=_parameter_sources(),
    )

    args = [str(val) for val in params["/tf/sim/drone_to_depth_cam"]] + [drone_frame_id, depth_cam_frame_id]
    tf_drone_to_depth_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=_parameter_sources(),
    )

    world_to_drone = Node(
        package="iii_drone_core",
        executable="drone_frame_broadcaster",
        arguments=["--ros-args", "--log-level", drone_frame_broadcaster_log_level],
        parameters=_parameter_sources(),
    )

    return LaunchDescription([
        drone_frame_broadcaster_log_level_arg,
        tf_drone_to_cable_gripper,
        tf_drone_to_iwr,
        tf_drone_to_depth_cam,
        world_to_drone,
    ])
