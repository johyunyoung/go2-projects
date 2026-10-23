#!/usr/bin/env bash
# Runs INSIDE the go2:humble container (started by go2-ros2-bridge.service on the Jetson host).
#   run_state_bridge.sh          start go2_state_bridge.py, console log to ~/go2_ws/logs/bridge_<time>.log
#   run_state_bridge.sh --check  only check that ROS 2, the unitree_go messages and the script are found
set +u   # ROS setup.bash uses unset variables
WS=/home/unitree/go2_ws
ROS2_DIR=$WS/go2-projects/ros2
NET_IF=${NET_IF:-enP8p1s0}
RATE=${BRIDGE_RATE:-100.0}

# shellcheck disable=SC1091
source "$ROS2_DIR/env.sh"

if [ "${1:-}" = "--check" ]; then
    echo "script : $(ls -l "$ROS2_DIR/go2_state_bridge.py" 2>&1)"
    echo "ros2   : $(command -v ros2 || echo MISSING)"
    python3 -c "import rclpy, unitree_go.msg, sensor_msgs.msg" && echo "python : OK" || { echo "python : MISSING modules"; exit 1; }
    echo "rmw    : $RMW_IMPLEMENTATION, interface $NET_IF"
    exit 0
fi

mkdir -p "$WS/logs"
LOG="$WS/logs/bridge_$(date +%F_%H%M%S).log"
# Wait for the DDS interface like run_go2_ctrl.sh does
for i in $(seq 1 90); do
    if [ "$(cat /sys/class/net/$NET_IF/operstate 2>/dev/null)" = "up" ] && { ! command -v ip >/dev/null || ip -4 addr show dev "$NET_IF" 2>/dev/null | grep -q inet; }; then
        break
    fi
    [ "$i" = 1 ] && echo "$(date '+%F %T') waiting for $NET_IF ..." >> "$LOG"
    sleep 1
done
echo "$(date '+%F %T') $NET_IF $(cat /sys/class/net/$NET_IF/operstate 2>/dev/null), rate $RATE Hz" >> "$LOG"
exec python3 "$ROS2_DIR/go2_state_bridge.py" --ros-args -p rate:="$RATE" >> "$LOG" 2>&1
