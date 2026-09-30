import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
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


def _write_config_tree(base_dir: Path):
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
    )


def _isolate_runtime_state(base_dir: Path, monkeypatch):
    _write_config_tree(base_dir)
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
    assert len(tf_description.entities) == 8


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
    assert nodes[3]._Node__package == "iii_drone_core"
    assert nodes[3]._Node__node_executable == "drone_frame_broadcaster"
    assert nodes[4]._Node__package == "iii_drone_simulation"
    assert nodes[4]._Node__node_executable == "ground_truth_frame_broadcaster"


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

    assert len(static_transform_nodes) == 3
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
    # This is the published /tf/sim/drone_to_cable_gripper contract.  The
    # Gazebo plugin must use the same origin and roll convention, otherwise
    # the controller can center the cable in its frame without ever latching.
    assert gripper_pose == [0.0, 0.0, 0.4, 1.57079632679, 0.0, 0.0]
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
