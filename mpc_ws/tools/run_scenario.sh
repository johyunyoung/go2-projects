#!/usr/bin/env bash
# Runs one scripted scenario while recording the debug topics, for the report tools.
# Needs the simulator and `ros2 launch go2_mpc_viz viz.launch.py` running.
#
#   tools/run_scenario.sh <tag> <script.yaml> [go2_mpc args...]
#   CONFIG=/path/to/other.yaml tools/run_scenario.sh ...   # another config
#
# Bag: /tmp/scenario_<tag>, controller log: /tmp/scenario_<tag>.log. Script paths are
# relative to src/go2_mpc. Example:
#   tools/run_scenario.sh kf scripts/m6_velocity.yaml --estimator kalman --sim-noise 1
#   python3 tools/velocity_report.py /tmp/scenario_kf src/go2_mpc/scripts/m6_velocity.yaml
source "$(dirname "$0")/../env.sh" 2>/dev/null
tag=$1; script=$2; shift 2
bag=/tmp/scenario_$tag
rm -rf "$bag"
ros2 bag record -o "$bag" /go2_mpc/mpc /go2_mpc/status > "$bag.bag.log" 2>&1 &
rec=$!
sleep 2
cd "$GO2_MPC_WS/src/go2_mpc"
dur=$(python3 -c "import yaml; print(int(max(e['t'] for e in yaml.safe_load(open('$script')))) + 3)")
timeout "$dur" "$GO2_MPC_WS/install/go2_mpc/lib/go2_mpc/go2_mpc" "${CONFIG:-config/go2_mpc_sim.yaml}" \
  --script "$script" "$@" > "$bag.log" 2>&1
kill -INT "$rec"; wait "$rec" 2>/dev/null
grep -E "main\] (state|simulated)|tilt|watchdog|stale|mpc\] stop" "$bag.log"
