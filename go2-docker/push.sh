#!/usr/bin/env bash
# Pushes only the deploy image. The training image is deliberately excluded: it embeds
# NVIDIA's Isaac Sim, which is EULA-gated and distributed through NGC.
set -euo pipefail
cd "$(dirname "$0")"
set -a; . ./.env; set +a

echo "pushing ${DEPLOY_IMAGE}:${DEPLOY_TAG}"
docker push "${DEPLOY_IMAGE}:${DEPLOY_TAG}"
