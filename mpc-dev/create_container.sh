#!/usr/bin/env bash
# Creates the go2-mpc container: same options as ../create_container.sh, but on the
# go2-mpc-dev image (go2-deploy-user + ROS 2 Humble).
#
# Do not run the simulator or a controller here while go2-deploy runs its own:
# both containers share the host network, so they would meet on the same DDS domain.
xhost +local:

docker run -dit \
  --name="go2-mpc" \
  --network="host" \
  --gpus="all" \
  --env="NVIDIA_DISABLE_REQUIRE=1" \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME:$HOME:rw" \
  go2-mpc-dev shell
