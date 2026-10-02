import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def _load_module(relative_path: str):
    module_path = PACKAGE_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(
        module_path.stem.replace(".", "_"), module_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_config_tree(base_dir: Path, sensor_layout: str = "d4s_dc_drone"):
    config_root = base_dir / "iii_drone"
    profiles_dir = config_root / "profiles"
    parameter_set_dir = config_root / "parameter_sets" / "sim" / "tracked"
    profiles_dir.mkdir(parents=True, exist_ok=True)
    parameter_set_dir.mkdir(parents=True, exist_ok=True)

    (profiles_dir / "sim.yaml").write_text(
        "version: 1\n" "active_parameter_set: tracked/default.yaml\n"
    )
    (parameter_set_dir / "default.yaml").write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    /tf/drone_frame_id: drone\n"
        "    /tf/world_frame_id: world\n"
        "    /tf/cable_gripper_frame_id: cable_gripper\n"
        "    /tf/mmwave_frame_id: mmwave\n"
        "    /tf/sim/depth_cam_frame_id: depth_camera\n"
        "    /tf/sim/drone_to_cable_gripper: [0, 0, 0, 0, 0, 0]\n"
        "    /tf/sim/drone_to_mmwave: [0, 0, 0, 0, 0, 0]\n"
        "    /tf/sim/drone_to_depth_cam: [0, 0, 0, 0, 0, 0]\n"
        "    /tf/cable_camera_frame_id: cable_camera\n"
        "    /tf/mmwave_forward_frame_id: mmwave_forward\n"
        f"    /tf/sim/sensor_layout: {sensor_layout}\n"
        "    /tf/sim/drone_to_cable_camera: [0, 0, 1, 0, 0, 0]\n"
        "    /tf/sim/powerline_eval/drone_to_cable_camera: [0, 0, 2, 0, 0, 0]\n"
        "    /tf/sim/powerline_eval/drone_to_mmwave_forward: [0, 0, 3, 0, 0, 0]\n"
    )


def _isolate_runtime_state(base_dir: Path, monkeypatch, sensor_layout: str = "d4s_dc_drone"):
    _write_config_tree(base_dir, sensor_layout)
    monkeypatch.setenv("CONFIG_BASE_DIR", str(base_dir))
    monkeypatch.setenv("III_OPERATIONS_ROOT", str(base_dir / "operations"))


def test_simulation_launch_files_generate_descriptions(tmp_path, monkeypatch):
    _isolate_runtime_state(tmp_path, monkeypatch)

    sim_assets_module = _load_module("launch/sim_assets.launch.py")
    tf_module = _load_module("launch/tf_sim.launch.py")

    sim_assets_description = sim_assets_module.generate_launch_description()
    tf_description = tf_module.generate_launch_description()

    assert isinstance(sim_assets_description, LaunchDescription)
    assert isinstance(tf_description, LaunchDescription)
    assert len(sim_assets_description.entities) == 15
    assert len(tf_description.entities) == 9


def test_sim_assets_launch_contains_expected_bridge_nodes(tmp_path, monkeypatch):
    _isolate_runtime_state(tmp_path, monkeypatch)

    sim_assets_module = _load_module("launch/sim_assets.launch.py")
    description = sim_assets_module.generate_launch_description()

    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert description.entities[0].name == "include_diagnostics"
    assert description.entities[1].name == "use_camera_rate_limiter"
    assert [node._Node__node_name for node in nodes] == [
        "clock_gz_bridge",
        "camera_gz_bridge",
        "camera_rate_limiter",
        "camera_compressor",
        "camera_compressor",
        "depth_cam_gz_bridge",
        "mmwave_gz_bridge",
        "mmwave_full_gz_bridge",
        "ground_truth_odometry_gz_bridge",
        "mmwave_labels_gz_bridge",
        "conductor_id_map_gz_bridge",
    ]
    assert [node._Node__package for node in nodes] == [
        "ros_gz_bridge",
        "ros_gz_bridge",
        "iii_drone_simulation",
        "image_transport",
        "image_transport",
        "ros_gz_bridge",
        "ros_gz_bridge",
        "ros_gz_bridge",
        "ros_gz_bridge",
        "ros_gz_bridge",
        "ros_gz_bridge",
    ]
    assert nodes[2]._ExecuteLocal__respawn is True
    assert nodes[2]._ExecuteLocal__respawn_delay == 1.0
    assert nodes[0]._Node__arguments == ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    assert nodes[6]._Node__arguments == [
        "/sensor/mmwave/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
    ]
    assert nodes[7]._Node__arguments == [
        "/sensor/mmwave/points_full@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
    ]
    assert nodes[8]._Node__arguments == [
        "/simulation/ground_truth/drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
    ]
    assert nodes[9]._Node__arguments == [
        "/simulation/ground_truth/mmwave/conductor_labels@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
    ]
    assert nodes[10]._Node__arguments == [
        "/simulation/ground_truth/conductor_id_map@std_msgs/msg/String[gz.msgs.StringMsg"
    ]
    # Only lossless PNG frames cross the HIL link; raw frames stay local.
    launch_source = (Path(__file__).resolve().parents[1] / "launch" / "sim_assets.launch.py").read_text()
    assert '"output_topic": HIL_RATE_LIMITED_CAMERA_TOPIC' in launch_source
    hil_compressor, sim_compressor = nodes[3], nodes[4]

    def remaps(node):
        text = lambda subs: "".join(sub.text for sub in subs)
        return {text(src): text(dst) for src, dst in node._Node__remappings}

    for compressor in (hil_compressor, sim_compressor):
        assert remaps(compressor)["out/compressed"] == "/sensor/cable_camera/image_raw/compressed"
    assert remaps(hil_compressor)["in"] == sim_assets_module.HIL_RATE_LIMITED_CAMERA_TOPIC
    assert remaps(sim_compressor)["in"] == "/sensor/cable_camera/image_raw"
    assert sim_assets_module.HIL_RATE_LIMITED_CAMERA_TOPIC != "/sensor/cable_camera/image_raw"


def test_drone_model_publishes_authoritative_3d_ground_truth_odometry():
    model_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "models"
        / "d4s_dc_drone"
        / "model.sdf"
    )
    root = ET.parse(model_path).getroot()
    plugins = root.findall(".//plugin[@name='gz::sim::systems::OdometryPublisher']")

    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.get("filename") == "gz-sim-odometry-publisher-system"
    assert plugin.findtext("odom_frame") == "world"
    assert plugin.findtext("robot_base_frame") == "drone"
    assert plugin.findtext("dimensions") == "3"
    assert plugin.findtext("odom_publish_frequency") == "100"
    assert plugin.findtext("odom_topic") == "/simulation/ground_truth/drone/odometry"


def test_drone_model_configures_measurement_provenance_topics():
    model_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "models"
        / "d4s_dc_drone"
        / "model.sdf"
    )
    root = ET.parse(model_path).getroot()
    plugins = root.findall(
        ".//plugin[@name='iii_drone::simulation::MmwaveConductorSensorPlugin']"
    )

    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.findtext("full_topic") == "/sensor/mmwave/points_full"
    assert (
        plugin.findtext("label_topic")
        == "/simulation/ground_truth/mmwave/conductor_labels"
    )
    assert plugin.findtext("camera_image_topic") == "/sensor/cable_camera/image_raw"
    assert plugin.findtext("camera_mask_topic") == (
        "/simulation/ground_truth/cable_camera/conductor_instance_mask"
    )
    assert plugin.findtext("conductor_id_map_topic") == (
        "/simulation/ground_truth/conductor_id_map"
    )


def test_ground_truth_plugin_reads_simulator_state_and_publishes_typed_truth():
    source = (PACKAGE_ROOT / "src" / "mmwave_conductor_sensor_plugin.cpp").read_text()

    assert "WorldLinearVelocity(ecm)" in source
    assert "WorldAngularVelocity(ecm)" in source
    assert 'state.header.frame_id = "world"' in source
    assert '"/simulation/ground_truth/drone/state"' in source
    assert '"/simulation/ground_truth/conductors/geometry"' in source
    assert '"/simulation/ground_truth/mmwave/scan"' in source
    assert '"/simulation/ground_truth/cable_camera/frame"' in source
    for forbidden in ("vehicle_odometry", "vehicle_local_position", "estimator_status"):
        assert forbidden not in source


def test_ground_truth_tf_converts_gazebo_enu_to_iii_world():
    from iii_drone_simulation.ground_truth_frame_broadcaster import (
        gazebo_enu_pose_to_iii_world,
    )

    # Exact convention boundary observed in HIL: Gazebo east/north becomes
    # III north/west, and an ENU heading of pi/2 becomes III yaw zero.
    converted = gazebo_enu_pose_to_iii_world(
        -1.4,
        2.2,
        3.0,
        0.0,
        0.0,
        2**-0.5,
        2**-0.5,
    )

    assert converted[:3] == (2.2, 1.4, 3.0)
    assert converted[3] == pytest.approx(0.0, abs=1e-12)
    assert converted[4] == pytest.approx(0.0, abs=1e-12)
    assert converted[5] == pytest.approx(0.0, abs=1e-12)
    assert converted[6] == pytest.approx(1.0, abs=1e-12)


def test_sim_gripper_gazebo_callback_is_thread_safe_and_wall_time_throttled():
    source = (PACKAGE_ROOT / "src" / "sim_charger_gripper_node.cpp").read_text()

    assert "gz_state_mutex_" in source
    assert "last_gz_log_steady_ns_" in source
    assert "std::chrono::steady_clock" in source
    assert "RCLCPP_INFO_THROTTLE" not in source


def test_tf_launch_uses_frame_ids_from_configuration(tmp_path, monkeypatch):
    _isolate_runtime_state(tmp_path, monkeypatch)

    tf_module = _load_module("launch/tf_sim.launch.py")
    description = tf_module.generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert description.entities[0].name == "drone_frame_broadcaster_log_level"
    assert description.entities[1].name == "use_ground_truth_odometry"
    assert description.entities[2].name == "publish_world_to_drone"
    assert nodes[0]._Node__arguments[-4:] == ["--frame-id", "drone", "--child-frame-id", "cable_gripper"]
    assert nodes[1]._Node__arguments[-4:] == ["--frame-id", "drone", "--child-frame-id", "mmwave"]
    assert nodes[2]._Node__arguments[-4:] == ["--frame-id", "drone", "--child-frame-id", "depth_camera"]
    assert nodes[3]._Node__arguments[-4:] == ["--frame-id", "drone", "--child-frame-id", "cable_camera"]
    assert nodes[3]._Node__arguments[1:12:2] == ["0", "0", "1", "0", "0", "0"]
    assert nodes[4]._Node__package == "iii_drone_core"
    assert nodes[4]._Node__node_executable == "drone_frame_broadcaster"
    assert nodes[5]._Node__package == "iii_drone_simulation"
    assert nodes[5]._Node__node_executable == "ground_truth_frame_broadcaster"
    child_frames = [node._Node__arguments[-1] for node in nodes[:4]]
    assert "mmwave_forward" not in child_frames


def test_powerline_eval_layout_publishes_its_camera_mount_and_forward_radar_frame(tmp_path, monkeypatch):
    _isolate_runtime_state(tmp_path, monkeypatch, "d4s_dc_drone_powerline_eval")

    tf_module = _load_module("launch/tf_sim.launch.py")
    description = tf_module.generate_launch_description()
    static_nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node) and entity._Node__node_executable == "static_transform_publisher"
    ]

    by_child = {node._Node__arguments[-1]: node._Node__arguments for node in static_nodes}
    assert set(by_child) == {"cable_gripper", "mmwave", "depth_camera", "cable_camera", "mmwave_forward"}
    assert by_child["cable_camera"][1:12:2] == ["0", "0", "2", "0", "0", "0"]
    assert by_child["mmwave_forward"][1:12:2] == ["0", "0", "3", "0", "0", "0"]
    assert by_child["mmwave_forward"][-4:] == ["--frame-id", "drone", "--child-frame-id", "mmwave_forward"]


def test_powerline_eval_layout_bridges_the_forward_radar(tmp_path, monkeypatch):
    _isolate_runtime_state(tmp_path, monkeypatch, "d4s_dc_drone_powerline_eval")

    sim_assets_module = _load_module("launch/sim_assets.launch.py")
    description = sim_assets_module.generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    bridges = {node._Node__node_name: node for node in nodes}

    assert len(description.entities) == 18
    assert bridges["mmwave_forward_gz_bridge"]._Node__arguments == [
        "/sensor/mmwave_forward/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
    ]
    assert bridges["mmwave_forward_full_gz_bridge"]._Node__arguments == [
        "/sensor/mmwave_forward/points_full@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
    ]
    assert bridges["mmwave_forward_labels_gz_bridge"]._Node__arguments == [
        "/simulation/ground_truth/mmwave_forward/conductor_labels@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
    ]
    # Like the upward radar, the full cloud and labels are diagnostics streams.
    assert bridges["mmwave_forward_gz_bridge"].condition is None
    assert bridges["mmwave_forward_full_gz_bridge"].condition is not None
    assert bridges["mmwave_forward_labels_gz_bridge"].condition is not None


def test_hil_launcher_leaves_world_to_drone_to_the_pi():
    launcher = (WORKSPACE_ROOT / "tools" / "simulation" / "launch_hil_workstation.sh").read_text()

    # The Pi publishes the sole dynamic world -> drone transform from the same
    # PX4 odometry its controller uses (docs/simulation-and-px4-integration.md);
    # the workstation adapters must not publish a competing one.
    assert "tf_sim.launch.py use_ground_truth_odometry:=true publish_world_to_drone:=false" in launcher
    assert "sim_assets.launch.py include_diagnostics:=false" in launcher
    assert "use_camera_rate_limiter:=true" in launcher


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
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity._Node__node_executable == "static_transform_publisher"
    ]

    assert len(static_transform_nodes) == 4
    # Named (non-deprecated) arguments: six pose values plus both frame ids.
    for node in static_transform_nodes:
        assert len(node._Node__arguments) == 16
        assert node._Node__arguments[0::2][:6] == ["--x", "--y", "--z", "--yaw", "--pitch", "--roll"]
    assert static_transform_nodes[0]._Node__arguments[1:12:2] == [
        "0.0",
        "0.0",
        "0.4",
        "1.57079632679",
        "0.0",
        "0.0",
    ]
    assert static_transform_nodes[1]._Node__arguments[1:12:2] == [
        "0.025",
        "-0.24",
        "0.295",
        "3.1415",
        "-1.57079632679",
        "0.0",
    ]
    assert static_transform_nodes[3]._Node__arguments[1:12:2] == [
        "0.0",
        "-0.215",
        "0.3",
        "0.0",
        "-1.571",
        "0.0",
    ]
    assert static_transform_nodes[3]._Node__arguments[-1] == "cable_camera"


def _rotation_rpy(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll): SDF poses and static_transform_publisher alike."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _assert_mount_matches_sdf(sdf_pose: str, mount: list):
    """A [x, y, z, yaw, pitch, roll] TF mount equals an SDF x y z roll pitch yaw pose."""
    x, y, z, roll, pitch, yaw = (float(value) for value in sdf_pose.split())
    assert mount[:3] == pytest.approx([x, y, z], abs=1e-9)
    sdf_rotation = _rotation_rpy(roll, pitch, yaw)
    tf_rotation = _rotation_rpy(mount[5], mount[4], mount[3])
    for sdf_row, tf_row in zip(sdf_rotation, tf_rotation):
        # The configured 3.1415 approximates pi in both sources.
        assert tf_row == pytest.approx(sdf_row, abs=1e-3)


def _tracked_sim_parameters() -> dict:
    path = (
        PACKAGE_ROOT.parent / "III-Drone-Configuration" / "config" / "parameter_sets"
        / "sim" / "tracked" / "default.yaml"
    )
    return yaml.safe_load(path.read_text())["/**"]["ros__parameters"]


def _model_root(model: str):
    return ET.parse(PACKAGE_ROOT / "Gazebo-simulation-assets" / "models" / model / "model.sdf").getroot()


def _radar_plugins(root):
    return root.findall(".//plugin[@name='iii_drone::simulation::MmwaveConductorSensorPlugin']")


def test_configured_sim_mounts_match_the_drone_model_sensor_poses():
    params = _tracked_sim_parameters()
    production = _model_root("d4s_dc_drone")
    camera = production.find(".//sensor[@name='cable_camera']")
    _assert_mount_matches_sdf(camera.findtext("pose"), params["/tf/sim/drone_to_cable_camera"])
    (radar,) = _radar_plugins(production)
    _assert_mount_matches_sdf(radar.findtext("sensor_pose"), params["/tf/sim/drone_to_mmwave"])

    variant = _model_root("d4s_dc_drone_powerline_eval")
    camera = variant.find(".//sensor[@name='cable_camera']")
    _assert_mount_matches_sdf(
        camera.findtext("pose"), params["/tf/sim/powerline_eval/drone_to_cable_camera"])
    radars = {plugin.findtext("radar_instance"): plugin for plugin in _radar_plugins(variant)}
    _assert_mount_matches_sdf(radars["mmwave"].findtext("sensor_pose"), params["/tf/sim/drone_to_mmwave"])
    _assert_mount_matches_sdf(
        radars["mmwave_forward"].findtext("sensor_pose"),
        params["/tf/sim/powerline_eval/drone_to_mmwave_forward"],
    )
    assert params["/tf/sim/sensor_layout"] == "d4s_dc_drone"


def test_powerline_eval_variant_has_two_simulator_v2_radars_and_the_tilted_camera():
    variant = _model_root("d4s_dc_drone_powerline_eval")
    radars = {plugin.findtext("radar_instance"): plugin for plugin in _radar_plugins(variant)}
    assert set(radars) == {"mmwave", "mmwave_forward"}
    for instance, profile in (("mmwave", "RADAR_U.yaml"), ("mmwave_forward", "RADAR_F.yaml")):
        plugin = radars[instance]
        assert plugin.findtext("radar_model") == "AOP_FAST_POINT"
        assert plugin.findtext("aop_config") == f"model://d4s_dc_drone_powerline_eval/radar/{profile}"
        assert plugin.findtext("topic") == f"/sensor/{instance}/points"
        assert plugin.findtext("full_topic") == f"/sensor/{instance}/points_full"
        assert plugin.findtext("frame_id") == instance
        assert plugin.findtext("camera_pose") == variant.find(".//sensor[@name='cable_camera']").findtext("pose")
    # Only Radar-U publishes the camera, drone-state and geometry truth.
    assert radars["mmwave"].findtext("publish_camera") == "true"
    assert radars["mmwave_forward"].findtext("publish_camera") == "false"
    assert radars["mmwave"].findtext("publish_camera_info") == "true"
    camera = variant.find(".//sensor[@name='cable_camera']")
    assert camera.findtext("gz_frame_id") == "cable_camera"
    assert variant.find(".//sensor[@name='pylon_semantic_camera']").findtext("pose") == camera.findtext("pose")
    for profile in ("RADAR_U.yaml", "RADAR_F.yaml"):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "Gazebo-simulation-assets/models/d4s_dc_drone_powerline_eval/radar" / profile).read_text())
        assert config["mode"] == "AOP_FAST_POINT"
        assert config["scatterers_path"] == "model://hcaa_pylon_setup/radar/scene_scatterers_r22_v1.json"
    assert (
        PACKAGE_ROOT / "Gazebo-simulation-assets/world_models/hcaa_pylon_setup/radar/scene_scatterers_r22_v1.json"
    ).is_file()


def test_generated_powerline_eval_variant_is_current():
    script = PACKAGE_ROOT / "Gazebo-simulation-assets" / "scripts" / "create_powerline_eval_drone_variant.py"
    result = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_static_transform_arguments_reject_non_euler_values():
    tf_module = _load_module("launch/tf_sim.launch.py")
    with pytest.raises(ValueError):
        tf_module._static_transform_arguments([0, 0, 0, 0, 0, 0, 1], "drone", "mmwave")


def test_conductor_asset_is_present_and_contains_four_conductors():
    asset = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "world_models"
        / "hcaa_pylon_setup"
        / "conductors.yaml"
    )

    content = asset.read_text()

    assert "frame_id: world" in content
    assert content.count("- id: conductor_") == 4
    assert content.count("samples:") == 4


def test_px4_gazebo_model_has_continuous_magnetometer_source():
    model_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "models"
        / "d4s_dc_drone"
        / "model.sdf"
    )
    airframe_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "init.d-posix_airframes"
        / "99999_gz_d4s_dc_drone"
    )
    world_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "worlds"
        / "hca_full_pylon_setup.sdf"
    )

    model = ET.parse(model_path).getroot()
    world = ET.parse(world_path).getroot()
    magnetometers = model.findall(".//sensor[@type='magnetometer']")
    magnetometer_systems = world.findall(
        ".//plugin[@name='gz::sim::systems::Magnetometer']"
    )

    assert len(magnetometers) == 1
    assert magnetometers[0].get("name") == "magnetometer_sensor"
    assert magnetometers[0].findtext("always_on") == "1"
    assert magnetometers[0].findtext("update_rate") == "100"
    assert len(magnetometer_systems) == 1
    assert magnetometer_systems[0].get("filename") == "gz-sim-magnetometer-system"
    assert "SENS_EN_MAGSIM" not in airframe_path.read_text()


def test_powerline_eval_variant_uses_px4_simulated_magnetometer():
    # PX4 reads Gazebo's magnetometer with a declination that disagrees with
    # its world magnetic model, which biases its heading against truth; the
    # evaluation variant uses PX4's simulated magnetometer instead.
    variant = _model_root("d4s_dc_drone_powerline_eval")
    airframe = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "init.d-posix_airframes"
        / "99997_gz_d4s_dc_drone_powerline_eval"
    ).read_text()

    assert variant.findall(".//sensor[@type='magnetometer']") == []
    assert "param set-default SENS_EN_MAGSIM 1\n" in airframe


def test_sim_airframe_tolerates_short_mavlink_joystick_gaps():
    airframe_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "init.d-posix_airframes"
        / "99999_gz_d4s_dc_drone"
    )

    assert "param set-default COM_RC_LOSS_T 5.0" in airframe_path.read_text()


def test_sim_airframe_endurance_covers_full_inspection_acceptance_cycle():
    airframe_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "init.d-posix_airframes"
        / "99999_gz_d4s_dc_drone"
    )

    assert "param set-default SIM_BAT_DRAIN 300" in airframe_path.read_text()


def test_asset_airframe_matches_px4_romfs_copy():
    asset_airframe = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "init.d-posix_airframes"
        / "99999_gz_d4s_dc_drone"
    )
    px4_airframe = (
        WORKSPACE_ROOT
        / "PX4-Autopilot"
        / "ROMFS"
        / "px4fmu_common"
        / "init.d-posix"
        / "airframes"
        / "99999_gz_d4s_dc_drone"
    )

    assert px4_airframe.read_bytes() == asset_airframe.read_bytes()


def test_simulated_gripper_matches_the_controller_cable_gripper_frame():
    model_path = (
        PACKAGE_ROOT
        / "Gazebo-simulation-assets"
        / "models"
        / "d4s_dc_drone"
        / "model.sdf"
    )
    model = ET.parse(model_path).getroot()
    plugin = model.find(
        ".//plugin[@name='iii_drone::simulation::SimChargerGripperPlugin']"
    )

    assert plugin is not None
    gripper_pose = [float(value) for value in plugin.findtext("gripper_pose").split()]
    # The published /tf/sim/drone_to_cable_gripper is [x, y, z, yaw, pitch,
    # roll]; SDF poses are [x, y, z, roll, pitch, yaw]. The plugin must use
    # the same frame (x along the cable), otherwise the controller can center
    # the cable in its frame without ever latching.
    parameters = yaml.safe_load((
        WORKSPACE_ROOT / "src" / "III-Drone-Configuration" / "config" / "parameter_sets"
        / "sim" / "tracked" / "default.yaml").read_text())
    x, y, z, yaw, pitch, roll = parameters["/**"]["ros__parameters"]["/tf/sim/drone_to_cable_gripper"]
    assert gripper_pose == [x, y, z, roll, pitch, yaw]
    assert float(plugin.findtext("latch_radius")) <= 0.04
    assert float(plugin.findtext("support_ramp_duration_s")) > 0.0


def test_canonical_simulation_launcher_requires_exact_tmux_session_name():
    launcher = WORKSPACE_ROOT / "tools" / "simulation" / "launch_simulation_tools.sh"
    source = launcher.read_text()

    assert 'tmux_command has-session -t "=${SESSION_NAME}"' in source
    process_group_function = source.split("px4_simulation_process_groups()", 1)[1].split(
        "cleanup_stale_px4_simulation()", 1
    )[0]
    assert "return 0" in process_group_function


def test_canonical_simulation_status_probes_gazebo_as_session_user():
    launcher = WORKSPACE_ROOT / "tools" / "simulation" / "launch_simulation_tools.sh"
    source = launcher.read_text()

    assert "session_user_command timeout" in source
    assert "source '${WORKSPACE_ROOT}/setup/setup_dev.bash' && gz service" in source


def test_canonical_simulation_recreate_clears_selected_instance_parameters():
    launcher = WORKSPACE_ROOT / "tools" / "simulation" / "launch_simulation_tools.sh"
    source = launcher.read_text()

    assert '"${rootfs}/${PX4_INSTANCE}/parameters.bson"' in source
    assert '"${rootfs}/${PX4_INSTANCE}/parameters_backup.bson"' in source


def test_simulation_launcher_never_owns_qgroundcontrol_and_declares_host_udp_transport():
    launcher = WORKSPACE_ROOT / "tools" / "simulation" / "launch_simulation_tools.sh"
    source = launcher.read_text()
    dockerfile = (WORKSPACE_ROOT / "Dockerfile.dev").read_text()
    devcontainer = (WORKSPACE_ROOT / ".devcontainer/devcontainer.json").read_text()

    assert "QGC_COMMAND" not in source
    assert "QGroundControl.AppImage" not in source
    assert 'HOST_QGC_UDP_PORT="${III_SIM_TOOLS_HOST_QGC_UDP_PORT:-14550}"' in source
    assert "host_qgc_udp_listener" in source
    assert "lifecycle owned by iii qgc" in source
    assert "QGroundControl.AppImage" not in dockerfile
    assert "tools/QGroundControl.org" not in dockerfile
    assert '"--network","host"' in devcontainer
    assert not (
        WORKSPACE_ROOT / "tools/simulation/managed_qgroundcontrol_config.yaml"
    ).exists()
