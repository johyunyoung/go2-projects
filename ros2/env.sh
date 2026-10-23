#!/usr/bin/env bash
# ROS 2 environment for the go2:humble container, matching the controller's DDS settings
# (domain 0, interface enP8p1s0, CycloneDDS). Source it:   source ~/go2_ws/go2-projects/ros2/env.sh
# Override the interface with NET_IF=... when the robot network is on another interface.
NET_IF=${NET_IF:-enP8p1s0}
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "ros2/env.sh: /opt/ros/humble not found; see ros2/README.md (install)" >&2
fi
# unitree_ros2 workspace (rmw_cyclonedds built from source, unitree_go / unitree_api message types)
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NET_IF\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
