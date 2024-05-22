# III-Drone-Simulation
Simulation assets for the III-Drone system

## Compatibility
This version is compatible with
- `ROS2 Humble`
- [`PX4-Autopilot` DIII fork tag `DIII-v2.2`](https://github.com/DIII-SDU-Group/PX4-Autopilot/tree/DIII-v2.2)
- [`px4_msgs` DIII fork tag `DIII-v2.2`](https://github.com/DIII-SDU-Group/px4_msgs/tree/DIII-v2.2)
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
Clone the `PX4-Autopilot` DIII fork tag `DIII-v2.2`:
```
cd <desired-PX4-Autopilot-parent-directory>
git clone git@github.com:DIII-SDU-Group/PX4-Autopilot.git -b DIII-v2.2 --recursive
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
Install the ROS2 Gazebo Garden tools (as per [these instructions](https://gazebosim.org/docs/garden/ros_installation#gazebo-garden)):
```
sudo apt remove ros-humble-ros-gz*
sudo apt install ros-humble-ros-gzgarden
```

Go to the ROS2 workspace and build:
```
cd <ros2-ws>
colcon build
```

## Running a simulation
Navigate to the `PX4-Autopilot` directory and start Gazebo:
```
make px4_sitl gz_d4s_dc_drone
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
Note that for the sensor ouput to be available as `ROS2` topics, the `ros_gz_bridge` bridge needs to run. This is done in the `sensors_sim.launch.py` launch file, which is automatically started when launching the `III-Drone-Core` launch file `iii_drone.launch.py` when the `simulation` parameter is set to true.

## References
* The [ros_gz packages](https://github.com/gazebosim/ros_gz?tab=readme-ov-file#readme)
* [ROS2 Humble gz bridge issue](https://github.com/gazebosim/ros_gz/issues/358)
* [Gazebo sim + ROS2 version matching](https://gazebosim.org/docs/garden/ros_installation#gazebo-garden)
* [PX4 on using the ros2_gz_bridge](https://docs.px4.io/main/en/ros/ros2_comm.html)
* [ROS2 on using the ros2_gz_bridge](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
* [Gazebo ROS2 integration overview](https://classic.gazebosim.org/tutorials?tut=ros2_overview)
* [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
* [PX4 Gazebo vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html)
* [Issue on Gazebo depth camera](https://github.com/PX4/PX4-user_guide/pull/2264)
* [Another issue on Gazebo depth camera](https://github.com/PX4/PX4-user_guide/issues/2283)
* [Gazebo plugins](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
* [Nicolaj's original tutorial](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo)
