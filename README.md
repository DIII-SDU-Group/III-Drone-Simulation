# III-Drone-Simulation

`iii_drone_simulation` contains the simulation-side ROS wiring and supporting assets used to run the III stack in a PX4/Gazebo environment.

## Package Role

This package provides:

- ROS launch files for simulated TF and sensor bridges
- a depth-camera to mmWave conversion node used by the simulated perception pipeline
- Gazebo/PX4 support assets and helper scripts for installing them into a simulation setup

## Module Map

### Launch Files

- `launch/sensors_sim.launch.py`: starts the simulated sensor bridge nodes and the `depth_cam_to_mmwave` node
- `launch/tf_sim.launch.py`: starts static transform publishers and the drone frame broadcaster for simulation

These launch files are the main integration points consumed by the larger III launch stack when simulation mode is enabled.

### Runtime Source

- `src/depth_cam_to_mmwave.cpp`: converts simulated depth camera data into the mmWave-style output used elsewhere in the stack

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
