#!/bin/bash

set -e
# set -x

PX4FIRMDIR=$1
CWD=$(dirname "$(readlink -f "$0")")

if [[ "$1" == "" ]]; then
  echo "USAGE: ./install_simulation_assets.sh /path/to/PX4-Autopilot_root"
  exit
fi

echo "Installing models.."
cp -f -r $CWD/../Gazebo-simulation-assets/models/* $PX4FIRMDIR/Tools/simulation/gz/models/ -v

echo

echo "Installing airframes.."
cp -f -r $CWD/../Gazebo-simulation-assets/init.d-posix_airframes/* $PX4FIRMDIR/ROMFS/px4fmu_common/init.d-posix/airframes/ -v
echo

echo "Installing worlds.."
cp -f -r $CWD/../Gazebo-simulation-assets/worlds/* $PX4FIRMDIR/Tools/simulation/gz/worlds/ -v

echo "Installing world models.."
cp -f -r $CWD/../Gazebo-simulation-assets/world_models/* $PX4FIRMDIR/Tools/simulation/gz/models/ -v

echo "Updating CMakeLists.txt files.."
$CWD/update_airframes_cmakelists.py $PX4FIRMDIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt $CWD/../Gazebo-simulation-assets/init.d-posix_airframes/
# $CWD/update_sitl_targets_cmakelists.py $PX4FIRMDIR/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake $CWD/../Gazebo-simulation-assets/models/ $CWD/../Gazebo-simulation-assets/world_models/ $CWD/../Gazebo-simulation-assets/worlds/

# echo "Applying PX4-Autopilot patch.."
# PWD_temp=${PWD}
# cd $PX4FIRMDIR
# git apply $CWD/../patches/PX4-Autopilot.patch -v 2>/dev/null
# cd $PWD_temp

echo

echo

echo "Done"
