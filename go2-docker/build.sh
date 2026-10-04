#!/usr/bin/env bash
# Builds the images. The deploy image is self-contained; the training image needs an
# NGC login first (docker login nvcr.io) because its base layer is Isaac Sim.
set -euo pipefail
cd "$(dirname "$0")"
set -a; . ./.env; set +a

case "${1:-deploy}" in
  deploy)
    docker build -t "${DEPLOY_IMAGE}:${DEPLOY_TAG}" -f deploy/Dockerfile deploy/
    ;;
  isaaclab)
    docker build -t "${ISAACLAB_IMAGE}:${ISAACLAB_TAG}" \
      --build-arg "ISAACSIM_BASE_IMAGE=${ISAACSIM_BASE_IMAGE}" \
      --build-arg "ISAACSIM_VERSION=${ISAACSIM_VERSION}" \
      -f isaaclab/Dockerfile isaaclab/
    ;;
  all)
    "$0" deploy && "$0" isaaclab
    ;;
  *)
    echo "usage: $0 [deploy|isaaclab|all]" >&2; exit 1
    ;;
esac
