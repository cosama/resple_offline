#!/usr/bin/env bash
# Build the RESPLE offline image from a minimal context (bridge core plus the
# pinned upstream estimator sources) so the repository's datasets never enter
# the context.
#
#   frameworks/resple/docker/build.sh [image-tag]
#
# Default tag: ghcr.io/cosama/resple_offline:latest (the `rot resple` default;
# override at run time with ROS_OFFLINE_RESPLE_IMAGE).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
tag="${1:-ghcr.io/cosama/resple_offline:latest}"
if [ "$#" -gt 0 ]; then shift; fi
engine="${CONTAINER_ENGINE:-docker}"

tar -C "${repo_root}" -c \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='.git' \
    --exclude='frameworks/resple/core/build' \
    --exclude='frameworks/resple/core/.venv' \
    --transform='s,^frameworks/resple/docker/Dockerfile$,Dockerfile,' \
    --transform='s,^frameworks/resple/,,' \
    frameworks/resple/docker/Dockerfile \
    frameworks/resple/core \
    frameworks/resple/upstream \
  | "${engine}" build -t "${tag}" "$@" -
