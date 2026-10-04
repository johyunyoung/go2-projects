#!/usr/bin/env bash
# Selects which of the two processes this container runs. They talk over DDS on the
# loopback interface at domain 0, so running them as separate containers requires
# both to share the host network namespace (network_mode: host).
set -e

MJ=/opt/unitree_mujoco/simulate/build
CTRL=/opt/unitree_rl_lab/deploy/robots/go2/build

case "${1:-help}" in
  sim)
    exec "$MJ/unitree_mujoco"
    ;;
  ctrl)
    shift
    if [ ! -f /policy/params/deploy.yaml ] || [ ! -f /policy/exported/policy.onnx ]; then
      echo "No policy on /policy. Mount a trained run directory containing:" >&2
      echo "  params/deploy.yaml      (joint mapping, gains, observation layout)" >&2
      echo "  exported/policy.onnx    (the network)" >&2
      echo "e.g. docker run -v \$HOME/runs/go2:/policy:ro ..." >&2
      exit 1
    fi
    # --network selects the DDS interface: lo against the simulator, or the robot's
    # interface (e.g. eth0) for the real Go2.
    exec "$CTRL/go2_ctrl" --network "${DDS_INTERFACE:-lo}" "$@"
    ;;
  jstest)
    exec "$MJ/jstest"
    ;;
  shell)
    exec /bin/bash
    ;;
  help|*)
    cat <<'EOH'
go2-deploy — MuJoCo simulator + the Go2 deployment controller (no policy baked in)

  sim     run the MuJoCo simulator (needs X11 for its window)
  ctrl    run go2_ctrl against a policy mounted on /policy
  jstest  probe a gamepad on /dev/input/js0, if one is passed in
  shell   interactive shell

Keyboard controls in the MuJoCo window (there is no gamepad in a container):
  1 stand   2 run policy   0 passive/stop
  W/S  forward/back +-0.1 m/s     A/D  strafe +-0.1
  Q/E  turn +-0.1 rad/s           X    zero all commands
  Each press steps the setpoint; it is not a hold-to-move control.

Set DDS_INTERFACE to pick the DDS interface for `ctrl` (default: lo).
EOH
    ;;
esac
