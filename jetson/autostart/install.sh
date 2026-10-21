#!/usr/bin/env bash
# Run on the Jetson HOST (unitree@ubuntu), not in the container.
#   ./install.sh <container name>     install + enable go2-rl.service (starts at next boot)
#   ./install.sh --uninstall          disable and remove it
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
UNIT=/etc/systemd/system/go2-rl.service

if [ "${1:-}" = "--uninstall" ]; then
    sudo systemctl disable --now go2-rl.service || true
    sudo rm -f "$UNIT" && sudo systemctl daemon-reload
    echo "removed go2-rl.service"; exit 0
fi

CONTAINER=${1:-}
if [ -z "$CONTAINER" ] && [ -f "$HERE/../local.env" ]; then source "$HERE/../local.env"; fi
[ -n "${CONTAINER:-}" ] || { echo "usage: $0 <container name>   (docker ps -a)"; exit 1; }
docker inspect "$CONTAINER" >/dev/null || { echo "no container $CONTAINER"; exit 1; }

echo "== check inside the container"
docker start "$CONTAINER" >/dev/null
docker exec "$CONTAINER" /home/unitree/go2_ws/go2-projects/jetson/autostart/run_go2_ctrl.sh --check

sed "s/__CONTAINER__/$CONTAINER/g" "$HERE/go2-rl.service" | sudo tee "$UNIT" >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable go2-rl.service
echo "installed. start now: sudo systemctl start go2-rl   status: systemctl status go2-rl"
