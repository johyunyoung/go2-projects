#!/usr/bin/env bash
# Runs INSIDE the go2:humble container (started by go2-rl.service on the Jetson host).
#   run_go2_ctrl.sh          start the controller, console log to ~/go2_ws/logs/autostart_<time>.log
#   run_go2_ctrl.sh --check  only check that the binary, config and shared libraries are found
#
# docker exec does not read ~/.bashrc, so the library path of a working interactive shell is saved once:
#   (inside the container)  echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH" > ~/go2_ws/go2_ctrl.env
set -u
WS=/home/unitree/go2_ws
ENV_FILE=$WS/go2_ctrl.env
CTRL_DIR=$WS/unitree_cpp_deploy/deploy/robots/go2
NET_IF=${NET_IF:-enP8p1s0}

[ -f "$ENV_FILE" ] && source "$ENV_FILE"
cd "$CTRL_DIR" || { echo "missing $CTRL_DIR"; exit 2; }

if [ "${1:-}" = "--check" ]; then
    echo "binary : $(ls -l build/go2_ctrl 2>&1)"
    echo "config : $(grep -c 'handover:' config/config.yaml) handover block(s), policy_dir:"
    grep -n "policy_dir" config/config.yaml
    missing=$(ldd build/go2_ctrl | grep "not found")
    if [ -n "$missing" ]; then echo "libs   : MISSING"; echo "$missing"; exit 1; fi
    echo "libs   : OK"
    exit 0
fi

mkdir -p "$WS/logs"
exec ./build/go2_ctrl -n "$NET_IF" >> "$WS/logs/autostart_$(date +%F_%H%M%S).log" 2>&1
