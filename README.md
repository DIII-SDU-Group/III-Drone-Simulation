# III-Drone-Simulation

`iii_drone_simulation` contains the simulation-side ROS wiring and supporting assets used to run the III stack in a PX4/Gazebo environment.

## Package Role

This package provides:

- ROS launch files for simulated TF and sensor bridges
- a depth-camera to mmWave conversion node used by the simulated perception pipeline
- Gazebo/PX4 support assets and helper scripts for installing them into a simulation setup

## Module Map

### Launch Files

- `launch/sim_assets.launch.py`: starts the simulated Gazebo asset bridge nodes, including `/clock`
- `launch/tf_sim.launch.py`: starts static transform publishers and the drone frame broadcaster for simulation

These launch files are the main integration points consumed by the larger III launch stack when simulation mode is enabled.

### Runtime Source

- `src/depth_cam_to_mmwave.cpp`: converts simulated depth camera data into the mmWave-style output used elsewhere in the stack
- `src/mmwave_conductor_sensor_plugin.cpp`: publishes conductor detections on
  `/sensor/mmwave/points` and matching `x`, `y`, `z`, `velocity`, `snr`, and
  `noise` fields on `/sensor/mmwave/points_full`. Optional SDF settings, all
  off by default:
  - `fov_model`: `VIEW_CONE` (default; `view_cone_slope`, `max_point_dist`) or
    `FINITE_RECTANGULAR` (`min_point_dist`, `max_point_dist`,
    `azimuth_half_angle_rad`, `elevation_half_angle_rad`);
  - `pylon_asset_uri` and `pylon_returns_enabled`: evaluator pylon map, pylon
    radar returns, and pylon camera truth from a segmentation camera on
    `pylon_semantic_topic`;
  - `radar_model` `AOP_FAST_POINT` with `aop_config`, `radar_instance` and
    `radar_seed`: the simulator-v2 IWR6843AOP model
    (`include/iii_drone_simulation/aop_radar_*.hpp`);
  - `publish_camera_info`, and `publish_ros_sensor_streams` for direct ROS
    copies of the camera image and full cloud where no Gazebo bridge carries
    them;
  - `-DIII_DRONE_SIMULATION_CUDA=ON` builds an optional CUDA backend for the
    simulator-v2 model; `III_DRONE_SIMULATION_OPTIMIZE_RADAR_KERNELS` (default
    `ON`) compiles the plugin with `-O2` in Debug builds.

  The `d4s_dc_drone_powerline_eval` model variant enables the pylon and finite
  FOV settings; see `docs/perception-dataset-ground-truth.md` in the workspace.

### Support Assets

- `Gazebo-simulation-assets/`: PX4/Gazebo models, airframes, and world assets
- `patches/`: patch files for supported simulator/firmware integration points
- `scripts/install_gazebo_simulation_assets.sh`: installs package-owned assets into a PX4 checkout

## Configuration Expectations

The simulation launch files expect a configuration tree rooted at:

- `$CONFIG_BASE_DIR/iii_drone/profiles/sim.yaml`
- `$CONFIG_BASE_DIR/iii_drone/parameter_sets/sim/<active-parameter-set>`

The `sim.yaml` selector chooses the active parameter set for simulation. That parameter-set file provides frame IDs and transform values for the simulated TF publishers.

## Tests

The current tests validate:

- both simulation launch files generate valid launch descriptions
- expected launch entities are present
- frame IDs and executable/package wiring are read from configuration as intended
- the finite-FOV radar geometry (`test/mmwave_fov_geometry_test.cpp`) against a dense oracle

Typical package-only commands:

```bash
colcon build --packages-select iii_drone_simulation
colcon test --packages-select iii_drone_simulation --ctest-args --output-on-failure
colcon test-result --verbose
```

## Extension Guidelines

- keep launch-file configuration reads explicit and deterministic
- document any new required simulation parameter keys in this README
- add launch smoke coverage when adding new simulated nodes or bridges
