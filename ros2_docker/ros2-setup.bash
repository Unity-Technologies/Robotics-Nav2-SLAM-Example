#!/usr/bin/env bash
set -e

echo "export PATH=\$PATH:~/.local/bin" >> ~/.bashrc
python3 -m pip install --no-warn-script-location \
  ipython

source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/$WS_NAME
rosdep update
rosdep install -yrq --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
