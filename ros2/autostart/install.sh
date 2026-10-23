#!/usr/bin/env bash
# Run on the Jetson HOST (unitree@ubuntu), not in the container.
#   ./install.sh <container name>     install + enable + start go2-ros2-bridge.service (also at every boot)
#   ./install.sh --uninstall          disable and remove it
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
UNIT=/etc/systemd/system/go2-ros2-bridge.service

if [ "${1:-}" = "--uninstall" ]; then
    sudo systemctl disable --now go2-ros2-bridge.service || true
    sudo rm -f "$UNIT" && sudo systemctl daemon-reload
    echo "removed go2-ros2-bridge.service"; exit 0
fi

CONTAINER=${1:-}
if [ -z "$CONTAINER" ] && [ -f "$HERE/../../jetson/local.env" ]; then source "$HERE/../../jetson/local.env"; fi
[ -n "${CONTAINER:-}" ] || { echo "usage: $0 <container name>   (docker ps -a)"; exit 1; }
docker inspect "$CONTAINER" >/dev/null || { echo "no container $CONTAINER"; exit 1; }

echo "== check inside the container"
docker start "$CONTAINER" >/dev/null
docker exec "$CONTAINER" chmod +x /home/unitree/go2_ws/go2-projects/ros2/autostart/run_state_bridge.sh /home/unitree/go2_ws/go2-projects/ros2/go2_state_bridge.py
docker exec "$CONTAINER" /home/unitree/go2_ws/go2-projects/ros2/autostart/run_state_bridge.sh --check

sed "s/__CONTAINER__/$CONTAINER/g" "$HERE/go2-ros2-bridge.service" | sudo tee "$UNIT" >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now go2-ros2-bridge.service
echo "installed and started. status: systemctl status go2-ros2-bridge   log: ls -t ~/go2_ws/logs/bridge_*.log | head -1"
