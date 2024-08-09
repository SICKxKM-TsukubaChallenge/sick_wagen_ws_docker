#!/bin/bash
set -e

# NVIDIA settings (kept as-is from original script)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Source ROS and custom workspace setup
mkdir -p /root/sick_wagen_ws_docker
echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
echo 'source /root/sick_wagen_ws_docker/devel/setup.bash' >> /root/.bashrc
echo 'export ROS_MAP=/root/sick_wagen_ws_docker/sick_wagen_ws/maps/map_1119' >> /root/.bashrc
echo 'export ROS_MASTER_PC_IP=192.168.0.152' >> /root/.bashrc
echo 'export SICK_WAGEN_XACRO=sick_wagen.xacro' >> /root/.bashrc

# Source ROS setup for the current shell
source /opt/ros/noetic/setup.bash
source ~/.bashrc

# Move to the mounted directory
cd /root/sick_wagen_ws_docker

# Execute the provided command, or fallback to a login shell
if which "$1" > /dev/null 2>&1 ; then
    exec "$@"
else
    exec $SHELL -li
fi
