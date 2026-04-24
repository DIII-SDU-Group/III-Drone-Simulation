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
    profiles_dir = config_root / "profiles"
    parameter_set_dir = config_root / "parameter_sets" / "sim" / "tracked"
    profiles_dir.mkdir(parents=True, exist_ok=True)
    parameter_set_dir.mkdir(parents=True, exist_ok=True)

    (profiles_dir / "sim.yaml").write_text(
        "version: 1\n"
        "active_parameter_set: tracked/default.yaml\n"
    )

    (parameter_set_dir / "default.yaml").write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    /tf/drone_frame_id: drone\n"
        "    /tf/cable_gripper_frame_id: cable_gripper\n"
        "    /tf/mmwave_frame_id: mmwave\n"
        "    /tf/sim/depth_cam_frame_id: depth_camera\n"
        "    /tf/sim/drone_to_cable_gripper: [0, 0, 0, 0, 0, 0]\n"
        "    /tf/sim/drone_to_mmwave: [0, 0, 0, 0, 0, 0]\n"
        "    /tf/sim/drone_to_depth_cam: [0, 0, 0, 0, 0, 0]\n"
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


def test_tf_launch_static_transform_argument_counts_use_production_config(monkeypatch):
    production_params = (
        PACKAGE_ROOT.parent
        / "III-Drone-Configuration"
        / "config"
        / "parameter_sets"
        / "sim"
        / "tracked"
        / "default.yaml"
    )
    monkeypatch.setenv("III_SYSTEM_PARAMETER_FILE", str(production_params))

    tf_module = _load_module("launch/tf_sim.launch.py")
    description = tf_module.generate_launch_description()
    static_transform_nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node) and entity._Node__node_executable == "static_transform_publisher"
    ]

    assert len(static_transform_nodes) == 3
    for node in static_transform_nodes:
        assert len(node._Node__arguments) in (8, 9)
