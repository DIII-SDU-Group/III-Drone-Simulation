# III-Drone-Simulation
Simulation assets for the III-Drone system

## Compatibility
This version is compatible with
- `ROS2 Humble`
- [`PX4-Autopilot` DIII fork tag `v1.14.0`](https://github.com/DIII-SDU-Group/PX4-Autopilot/tree/v1.14.0)
- [`px4_msgs` DIII fork tag `v1.14`](https://github.com/DIII-SDU-Group/px4_msgs/tree/v1.14)
- `Gazebo Garden` (installed from PX4-Autopilot install script)
- [`micro-ROS-agent` DIII fork tag `III-Drone-v2.2`](https://github.com/DIII-SDU-Group/micro-ROS-Agent/tree/III-Drone-v2.2)
- [`micro_ros_msgs` DIII fork tag `III-Drone-v2.2`](https://github.com/DIII-SDU-Group/micro_ros_msgs/tree/III-Drone-v2.2)
- [`III-Drone-Core` v2.2](https://github.com/DIII-SDU-Group/III-Drone-Core/tree/v2.2-staging)
- [`III-Drone-Interfaces` v2.2](https://github.com/DIII-SDU-Group/III-Drone-Interfaces/tree/v2.2-staging)

See [`III-Drone-Core`](https://github.com/DIII-SDU-Group/III-Drone-Core/tree/v2.2-staging) for more information.

### Installing required ROS packages
Please perform the installation steps in the `III-Drone-Core` repository.

## Installing simulation environment
Clone the `III-Drone-Simulation` repository:
```
cd <ros2-ws>/src
git clone -b v2.2-staging git@github.com:DIII-SDU-Group/III-Drone-Simulation.git --recursive
```
Clone the `PX4-Autopilot` DIII fork tag `v1.14.0`:
```
cd <desired-PX4-Autopilot-parent-directory>
git clone git clone git@github.com:DIII-SDU-Group/PX4-Autopilot.git -b v1.14.0 --recursive
```
Run the `PX4-Autopilot` setup script:
```
cd PX4-Autopilot
./Tools/setup/ubuntu.sh
```
Log out and log in again. Then run the install script:
```
cd <ros2-ws>
./src/III-Drone-Simulation/scripts/install_gazebo_simulation_assets.sh <PX4-Autopilot-directory>
```
Go to the ROS2 workspace and build:
```
cd <ros2-ws>
colcon build
```

## Running a simulation
Navigate to the `PX4-Autopilot` directory and start Gazebo:
```
PX4_NO_FOLLOW_MODE=1 make px4_sitl gazebo-classic_d4s_dc_drone__hca_full_pylon_setup
```

## Launching the simulation ROS2 system
After installing `III-Drone-Core`, set `simulation` to true in the system config:
```
vim ~/.config/iii_drone/parameters/parameters.yaml

global:
  simulation:
    type: bool
    value: true
    constant: true
...
```
Then, launch the system:
```
cd <ros2-ws>
source install/setup.sh
ros2 launch iii_drone_core iii_drone.launch.py
```
In a new terminal, open RVIZ:
```
cd <ros2-ws>
source install/setup.sh
rviz2 -d src/III-Drone-Core/rviz/rviz_config.rviz
```
