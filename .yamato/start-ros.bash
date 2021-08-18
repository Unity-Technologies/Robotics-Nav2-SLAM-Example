source /opt/ros/galactic/setup.bash

set -e

# Assuming this script is invoked from the root of the repository...
DIR_ORIGINAL=$PWD
cd ros2_docker/colcon_ws
colcon install
source install/local_setup.bash
cd "$DIR_ORIGINAL"
ros2 launch unity_slam_example unity_slam_example.py &
