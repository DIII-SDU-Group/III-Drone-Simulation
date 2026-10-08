# III-Drone-Simulation

`iii_drone_simulation` contains the simulation-side ROS wiring and supporting assets used to run the III stack in a PX4/Gazebo environment.

## Package Role

This package provides:

- ROS launch files for simulated TF and sensor bridges
- a depth-camera to mmWave conversion node used by the simulated perception pipeline
- a simulated OptiTrack lab gateway that publishes Gazebo ground truth as a Motive rigid-body pose stream
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
  `noise` fields on `/sensor/mmwave/points_full`
- `iii_drone_simulation/simulated_lab_mocap_gateway.py` (executable
  `simulated_lab_mocap_gateway`): publishes Gazebo ground truth the way the
  OptiTrack lab gateway publishes a rigid body; see
  [Simulated Lab Mocap Gateway](#simulated-lab-mocap-gateway)

### Support Assets

- `Gazebo-simulation-assets/`: PX4/Gazebo models, airframes, and world assets
- `patches/`: patch files for supported simulator/firmware integration points
- `scripts/install_gazebo_simulation_assets.sh`: installs package-owned assets into a PX4 checkout

## Configuration Expectations

The simulation launch files expect a configuration tree rooted at:

- `$CONFIG_BASE_DIR/iii_drone/profiles/sim.yaml`
- `$CONFIG_BASE_DIR/iii_drone/parameter_sets/sim/<active-parameter-set>`

The `sim.yaml` selector chooses the active parameter set for simulation. That parameter-set file provides frame IDs and transform values for the simulated TF publishers.

## Simulated Lab Mocap Gateway

In the SDU OptiTrack lab a gateway publishes each Motive rigid body as
`geometry_msgs/PoseStamped` on `/body_splitter/body_<id>/pose` in a lab-side
ROS domain, and the drone's `opti_track_pose_relay` (Core) converts it for
PX4. `simulated_lab_mocap_gateway` publishes that stream from Gazebo ground
truth, including the lab stream's outages, so SIM and HIL can drive the same
relay path into a vision-only PX4 SITL. No launch file or runtime profile
starts it yet.

| Property | Lab gateway | Simulated gateway |
| --- | --- | --- |
| Topic and type | `/body_splitter/body_<id>/pose`, `PoseStamped` | same; `<id>` is `rigid_body_id` |
| QoS | best effort, volatile, keep last 1 | same |
| Rate | about 120 Hz | every ground-truth sample, capped at `rate_hz` |
| `header.stamp` | gateway clock at publication | node clock at publication: simulation time with `use_sim_time:=true`, wall time otherwise |
| `header.frame_id` | not yet recorded | `world` (`frame_id`) |
| World frame | Z-up lab frame | III world: x north, y west, z up |
| Body frame | forward-left-up (rigid body created with the nose along Motive +x) | the drone's FLU body |

**Frames.** The source is `/simulation/ground_truth/drone/odometry`
(`nav_msgs/Odometry` from Gazebo's OdometryPublisher through
`ground_truth_odometry_gz_bridge`): Gazebo ENU world, FLU body. The gateway
converts it with the same `gazebo_enu_pose_to_iii_world` as
`ground_truth_frame_broadcaster`, so the simulated lab world is III's world
frame with Gazebo's world origin. III's world is PX4 NED with y and z negated,
which is exactly the relay's 180 degree rotation about x: the relay hands PX4
the NED/FRD pose that PX4 SITL simulates for the same Gazebo state. The tests
check this against PX4's own ENU-to-NED conversion.

**Rate.** Gazebo publishes ground truth at 100 Hz of simulation time,
quantised to the 4 ms physics step, which is below the lab's 120 Hz. A
120 Hz timer would republish stale poses under fresh stamps, which Motive
never does, so the gateway forwards each sample on arrival. `rate_hz` is a
cap: a sample stamped less than `1 / rate_hz` after the last forwarded one is
dropped, which thins a faster source to a regular cadence (for example a
100 Hz source capped at 30 Hz gives 25 Hz). With the default cap every sample
is forwarded.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `rigid_body_id` | `1` | Motive streaming ID; selects the topic |
| `rate_hz` | `120.0` | cap on the forwarded rate |
| `dropout_period_s` | `0.0` | period of injected outages; `0` disables them |
| `dropout_duration_s` | `33.0` | outage at the end of every period; must be positive and shorter than the period |
| `one_shot_dropout_s` | `0.0` | setting `D > 0` pauses publishing for `D` s from the next sample (each set restarts it, `0` cancels it, a startup value covers the first `D` s) |
| `odometry_topic` | `/simulation/ground_truth/drone/odometry` | ground-truth source |
| `frame_id` | `world` | `header.frame_id`; set it to the gateway's value once it is recorded |
| `lab_domain_id` | `-1` | ROS domain of the published poses; `-1` publishes in the process's own domain |

Only `one_shot_dropout_s` can change at runtime; the other parameters are
read-only. Pass doubles with a decimal point (`30.0`, not `30`).

**Outages.** Outages follow the node clock, so they run on simulation time
under `use_sim_time:=true`. The periodic schedule starts with the first
sample: each period is `dropout_period_s - dropout_duration_s` of stream
followed by `dropout_duration_s` of silence, so the stream always comes up
healthy. The default 33 s is one lab gateway watchdog restart (about 8 s
without data before the watchdog fires plus about 25 s of restart); lab
outages have lasted up to about 90 s. The gateway logs a warning when an
outage starts and an info line when it ends. A scripted test triggers a
one-shot outage from the gateway process's ROS domain (domain 42 in HIL; the
lab-domain participant offers no parameter services):

```bash
ros2 param set /simulated_lab_mocap_gateway one_shot_dropout_s 25.0
```

A paused stream keeps its DDS writer, whereas the lab gateway's watchdog
restarts the gateway process with a new DDS participant. Restart the gateway
process to emulate that.

**ROS domains.** Mirror the lab by publishing in the lab-side ROS domain
(domain 0). SIM already runs the whole stack in domain 0, so run the gateway
with `lab_domain_id` unset. HIL workstation adapters run in domain 42
(`setup/setup_hil.bash`); run the gateway in that environment with
`lab_domain_id:=0`. The poses then leave through a separate DDS participant in
domain 0, so domain 0 carries only the gateway topic, as in the lab, while
ground truth and `/clock` stay in domain 42. Running the whole process with
`ROS_DOMAIN_ID=0` instead would require bridging ground truth and `/clock`
into the lab domain, which the real lab domain never carries.

**SIM rehearsal.** The workspace runs the `opti_track` profile against PX4
SITL with this gateway: `tools/simulation/opti_track_rehearsal.sh` starts PX4
with a vision-only estimator (external-vision position, height and yaw; no
GNSS, no magnetometer), the Gazebo clock and ground-truth bridges, the gateway
with `use_sim_time:=true`, and the daemon and Runtime API in profile
`opti_track`. `scripts/workspace/run_opti_track_rehearsal.py` then flies the
OptiTrack missions and judges the run (see the workspace `docs/testing.md`).
The rehearsal injects one pose outage with `one_shot_dropout_s` while the
aircraft is on the ground. Longer outages (one 33 s gateway restart, a 90 s
outage) and in-flight outages are not part of it.

## Tests

The current tests validate:

- both simulation launch files generate valid launch descriptions
- expected launch entities are present
- frame IDs and executable/package wiring are read from configuration as intended
- the simulated lab mocap gateway's frame mapping (including the cross-check
  of relay output against PX4 SITL ground truth), topic naming, rate cap and
  outage schedule; one node test runs it in two coordinated ROS domains
  (never 0, 42 or 74) with discovery limited to this host

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
