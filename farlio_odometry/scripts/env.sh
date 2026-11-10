#!/usr/bin/env bash
# Common environment for the FAR-LIO ROS 1 port on this machine.
#
# The FAR-LIO workspace (farlio_ws) is an overlay on the main lio_ws workspace.
# catkin setup scripts rebuild CMAKE_PREFIX_PATH, so both prefixes are re-added
# explicitly here; otherwise farlio binaries are found as packages but not as
# executables (and fast_livo disappears from rosrun).
#
# Usage:  source scripts/env.sh

source /opt/ros/noetic/setup.bash

if [[ -f /home/komasa/lio_ws/farlio_ws/devel/setup.bash ]]; then
  source /home/komasa/lio_ws/farlio_ws/devel/setup.bash
fi
if [[ -f /home/komasa/lio_ws/devel/setup.bash ]]; then
  source /home/komasa/lio_ws/devel/setup.bash
fi

export CMAKE_PREFIX_PATH=/home/komasa/lio_ws/farlio_ws/devel:/home/komasa/lio_ws/devel:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/komasa/lio_ws/farlio_ws/devel/lib:/home/komasa/lio_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
export ROS_PACKAGE_PATH=/home/komasa/lio_ws/src:${ROS_PACKAGE_PATH}

# MVS SDK ships an old libusb that shadows the system one and breaks PCL.
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
