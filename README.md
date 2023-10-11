# III-Drone-Simulation
Simulation assets for the III-Drone system

## Compatibility
This version is compatible with
- `ROS2 Humble`
- `PX4-Autopilot` DIII fork tag `v1.14.0-rc2`
- `Gazebo Garden` (installed from PX4-Autopilot install script)
- `micro-ROS-agent` DIII fork branch `humble`
- `micro_ros_msgs` DIII fork branch `humble`

## Installing simulation environment
Clone the `PX4-Autopilot` DIII fork tag `v1.14.0-rc2`:
```
	cd <desired-PX4-Autopilot-parent-directory>
	git clone git clone git@github.com:DIII-SDU-Group/PX4-Autopilot.git -b v1.14.0-rc2 --recursive
```
Run the `PX4-Autopilot` setup script:
```
	cd PX4-Autopilot
	./Tools/setup/ubuntu.sh
```
Log out and log in again. Then, navigate to the `III-Drone-Simulation` directory and run the install script:
```
	cd <III-Drone-Simulation-directory>
	./scripts/install_gazebo_simulation_assets.sh <desired-PX4-Autopilot-parent-directory>/PX4-Autopilot
```

### Installing required ROS packages
Please refer to the `III-Drone-Core` repository for installation of additional required ROS packages

## Running a simulation
Navigate to the `PX4-Autopilot` directory and build the simulator:
```
	cd <desired-PX4-Autopilot-parent-directory>/PX4-Autopilot
	make px4_sitl
```
Now, a simulation with the `d4s_dc_drone` model in the `hca_full_pylon_setup` world autostart id 99999 can be started with:
```
	PX4_SYS_AUTOSTART=99999 ./build/px4_sitl_default/bin/px4
```
If Gazebo doesn't launch, kill it and try again:
```
	pkill -f "gz sim"
```
	

