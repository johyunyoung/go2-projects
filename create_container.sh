#!/usr/bin/env bash
xhost +local:

docker run -dit \
  --name="go2-deploy" \
  --network="host" \
  --gpus="all" \
  --env="NVIDIA_DISABLE_REQUIRE=1" \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME:$HOME:rw" \
  go2-deploy-user shell

