from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import yaml

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration



def _static_transform_arguments(values, frame_id, child_frame_id):
    """Named static_transform_publisher arguments for [x, y, z, yaw, pitch, roll].

    The positional form is deprecated in Jazzy and logs a warning per start.
    """
    if len(values) != 6:
        raise ValueError(
            f"static transform {frame_id}->{child_frame_id} needs [x, y, z, yaw, pitch, roll], got {values!r}"
        )
    names = ("--x", "--y", "--z", "--yaw", "--pitch", "--roll")
    arguments = []
    for name, value in zip(names, values):
        arguments += [name, str(value)]
    return arguments + ["--frame-id", frame_id, "--child-frame-id", child_frame_id]

def _resolve_ros_params_file() -> str:
    seed_runtime_configuration("sim")
    return str(resolve_active_parameter_file("sim"))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": True}]

def generate_launch_description():
    drone_frame_broadcaster_log_level = LaunchConfiguration("drone_frame_broadcaster_log_level")
    use_ground_truth_odometry = LaunchConfiguration("use_ground_truth_odometry")
    publish_world_to_drone = LaunchConfiguration("publish_world_to_drone")

    drone_frame_broadcaster_log_level_arg = DeclareLaunchArgument(
        "drone_frame_broadcaster_log_level",
        default_value=["info"],
        description="The logging level for the drone frame broadcaster node, default is INFO",
    )
    use_ground_truth_odometry_arg = DeclareLaunchArgument(
        "use_ground_truth_odometry",
        default_value="false",
        description="Publish world-to-drone TF from Gazebo ground truth instead of PX4 uXRCE odometry",
    )
    publish_world_to_drone_arg = DeclareLaunchArgument(
        "publish_world_to_drone",
        default_value="true",
        description="Publish a dynamic world-to-drone TF source; disable when another host owns it",
    )
    
    ros_params = _resolve_ros_params_file()
    with open(ros_params, "r") as file:
        ros_params_dict = yaml.safe_load(file) or {}
    params = ros_params_dict["/**"]["ros__parameters"]

    drone_frame_id = params["/tf/drone_frame_id"]
    cable_gripper_frame_id = params["/tf/cable_gripper_frame_id"]
    mmwave_frame_id = params["/tf/mmwave_frame_id"]
    depth_cam_frame_id = params["/tf/sim/depth_cam_frame_id"]

    args = _static_transform_arguments(params["/tf/sim/drone_to_cable_gripper"], drone_frame_id, cable_gripper_frame_id)
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=_parameter_sources(),
    )

    args = _static_transform_arguments(params["/tf/sim/drone_to_mmwave"], drone_frame_id, mmwave_frame_id)
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=_parameter_sources(),
    )

    args = _static_transform_arguments(params["/tf/sim/drone_to_depth_cam"], drone_frame_id, depth_cam_frame_id)
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
        condition=IfCondition(PythonExpression([
            "'", publish_world_to_drone, "' == 'true' and '", use_ground_truth_odometry, "' != 'true'"
        ])),
    )

    ground_truth_world_to_drone = Node(
        package="iii_drone_simulation",
        executable="ground_truth_frame_broadcaster",
        parameters=[
            {
                "use_sim_time": True,
                "world_frame_id": params["/tf/world_frame_id"],
                "drone_frame_id": drone_frame_id,
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", publish_world_to_drone, "' == 'true' and '", use_ground_truth_odometry, "' == 'true'"
        ])),
    )

    return LaunchDescription([
        drone_frame_broadcaster_log_level_arg,
        use_ground_truth_odometry_arg,
        publish_world_to_drone_arg,
        tf_drone_to_cable_gripper,
        tf_drone_to_iwr,
        tf_drone_to_depth_cam,
        world_to_drone,
        ground_truth_world_to_drone,
    ])
