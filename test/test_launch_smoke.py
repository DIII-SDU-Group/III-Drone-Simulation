import importlib.util
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_module(relative_path: str):
    module_path = PACKAGE_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(module_path.stem.replace(".", "_"), module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_config_tree(base_dir: Path):
    config_root = base_dir / "iii_drone"
    parameters_dir = config_root / "parameters"
    parameters_dir.mkdir(parents=True, exist_ok=True)

    (config_root / "ros_params.yaml").write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    parameters_path_postfix: parameters\n"
        "    sim_parameter_file: sim.yaml\n"
        "    default_parameter_file: sim.yaml\n"
    )

    (parameters_dir / "sim.yaml").write_text(
        "tf:\n"
        "  drone_frame_id:\n"
        "    value: drone\n"
        "  cable_gripper_frame_id:\n"
        "    value: cable_gripper\n"
        "  mmwave_frame_id:\n"
        "    value: mmwave\n"
        "  sim:\n"
        "    depth_cam_frame_id:\n"
        "      value: depth_camera\n"
        "    drone_to_cable_gripper:\n"
        "      value: [0, 0, 0, 0, 0, 0]\n"
        "    drone_to_mmwave:\n"
        "      value: [0, 0, 0, 0, 0, 0]\n"
        "    drone_to_depth_cam:\n"
        "      value: [0, 0, 0, 0, 0, 0]\n"
    )


def test_simulation_launch_files_generate_descriptions(tmp_path, monkeypatch):
    _write_config_tree(tmp_path)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))

    sensors_module = _load_module("launch/sensors_sim.launch.py")
    tf_module = _load_module("launch/tf_sim.launch.py")

    sensors_description = sensors_module.generate_launch_description()
    tf_description = tf_module.generate_launch_description()

    assert isinstance(sensors_description, LaunchDescription)
    assert isinstance(tf_description, LaunchDescription)
    assert len(sensors_description.entities) == 4
    assert len(tf_description.entities) == 5


def test_sensors_launch_contains_expected_nodes_and_argument(tmp_path, monkeypatch):
    _write_config_tree(tmp_path)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))

    sensors_module = _load_module("launch/sensors_sim.launch.py")
    description = sensors_module.generate_launch_description()

    assert isinstance(description.entities[0], DeclareLaunchArgument)
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert [node._Node__node_name for node in nodes] == [
        "depth_cam_to_mmwave",
        "camera_gz_bridge",
        "depth_cam_gz_bridge",
    ]
    assert [node._Node__package for node in nodes] == [
        "iii_drone_simulation",
        "ros_gz_bridge",
        "ros_gz_bridge",
    ]


def test_tf_launch_uses_frame_ids_from_configuration(tmp_path, monkeypatch):
    _write_config_tree(tmp_path)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path))

    tf_module = _load_module("launch/tf_sim.launch.py")
    description = tf_module.generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert description.entities[0].name == "drone_frame_broadcaster_log_level"
    assert nodes[0]._Node__arguments[-2:] == ["drone", "cable_gripper"]
    assert nodes[1]._Node__arguments[-2:] == ["drone", "mmwave"]
    assert nodes[2]._Node__arguments[-2:] == ["drone", "depth_camera"]
    assert nodes[3]._Node__package == "iii_drone_core"
    assert nodes[3]._Node__node_executable == "drone_frame_broadcaster"
