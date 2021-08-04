#!/usr/bin/env bash
set -e

# This install script is based on https://github.com/Tiryoh/ros2_setup_scripts_ubuntu, released under the Apache-2.0.
# REF: https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/
# by Open Robotics, licensed under CC-BY-4.0
# source: https://github.com/ros2/ros2_documentation

# curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
#echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
#  |  tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "export PATH=\$PATH:~/.local/bin" >> ~/.bashrc
python3 -m pip install --no-warn-script-location \
  ipython

source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/$WS_NAME
rosdep update
rosdep install -yrq --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
