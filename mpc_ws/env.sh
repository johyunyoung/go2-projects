# Source inside the go2-mpc container:   source ~/Quadruped/go2/mpc_ws/env.sh
#
# The container mounts the host home, so the host ~/.bashrc runs here too. It puts
# the host's source-built ROS 2 Humble (Ubuntu 20.04) and ~/mujoco_install on
# LD_LIBRARY_PATH / PATH, which would shadow the container's /opt/ros/humble.
# This strips those entries for the current shell only, then sources ROS 2.

if [ ! -f /.dockerenv ]; then
  echo "env.sh: run this inside the go2-mpc container" >&2
  return 1 2>/dev/null || exit 1
fi

_go2_strip() {  # drop entries under $HOME (host builds) from a colon-separated list
  printf '%s' "$1" | tr ':' '\n' | grep -v -e "^$HOME/" -e '^$' | paste -sd: -
}
export LD_LIBRARY_PATH="$(_go2_strip "$LD_LIBRARY_PATH")"
export PATH="$(_go2_strip "$PATH")"
unset -f _go2_strip
unset PYTHONPATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ROOT ROS_MASTER_URI ROS_ETC_DIR

source /opt/ros/humble/setup.bash

# Other ROS 2 containers on this PC share the host network; a separate domain keeps
# their topics out of ours. (unitree_sdk2 DDS is independent of this setting.)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

GO2_MPC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GO2_MPC_WS
if [ -f "$GO2_MPC_WS/install/setup.bash" ]; then
  source "$GO2_MPC_WS/install/setup.bash"
fi
