#!/usr/bin/env bash
# rig's BUILD-phase entrypoint (rigging.yaml `build.command`): build the self-contained runtime
# image and push it to the fleet registry. rig invokes it as
#   tools/build_image.sh <registry> [tag]        (cwd = repo root, tag defaults to latest)
# and the deploy compose pulls the result as ${RIG_IMAGE_REGISTRY}/glim:${RIG_IMAGE_TAG}.
# Works standalone too:  tools/build_image.sh 192.168.1.10:5000 mytag
#                        PUSH=0 tools/build_image.sh local dev      # -> local/glim:dev, not pushed
#
# rig build-env contract (rig exports these; scripts opt in; all rig-owned vars are set-or-popped,
# so the ${VAR:+...}/${VAR:-...} expansions below are safe under set -u):
#   RIG_BUILD_NO_CACHE  set => --no-cache --pull, and the compiler cache mount is bypassed too
#                       (Docker --no-cache leaves RUN cache mounts intact).
#   ROS_DISTRO          fleet distro (vehicle.yaml ros.distro); default lyrical.
#   RIG_BASE_IMAGE      the deployment's one runtime base image (fleet-ros), forwarded to every
#                       dependent build; empty/absent falls back to the standalone ros-core parent.
#   RIG_BUILD_PLATFORMS comma-separated Docker platforms; buildx publishes one image index.
#   RIG_BUILD_RESULT    optional JSONL path for the published image/index digest.
#
# Service knobs: GLIM_BUILD_PLATFORM=linux/<arch> (standalone cross-arch; RIG_BUILD_PLATFORMS wins
# under rig) and GLIM_BUILD_JOBS=N (parallel compile jobs; GTSAM needs ~2 GB RAM per job, so cap it
# on small builders). The GTSAM/GLIM compile is slow under qemu — build arm64 on an arm64 host.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=tools/rig-build.sh
. "$REPO/tools/rig-build.sh"

REGISTRY="${1:?usage: build_image.sh <registry> [tag]}"
TAG="${2:-latest}"
IMAGE="${REGISTRY}/glim:${TAG}"

# Wrapper identity for the OCI labels, captured before .dockerignore strips .git from the build
# context: exact tag, else commit SHA; a dirty tree (tracked or untracked changes) is marked as such
# and cannot claim a revision. Require this root's own .git so a tarball unpacked inside another
# checkout cannot borrow that checkout's identity.
SRC_REF=unknown
SRC_REV=unknown
if [ -e "$REPO/.git" ] && REV="$(git -C "$REPO" rev-parse --verify HEAD 2>/dev/null)"; then
  SRC_REV="$REV"
  SRC_REF="$(git -C "$REPO" describe --tags --exact-match "$REV" 2>/dev/null || printf '%s\n' "$REV")"
  if ! STATUS="$(git -C "$REPO" status --porcelain --untracked-files=normal 2>/dev/null)" \
      || [ -n "$STATUS" ]; then
    SRC_REF="${SRC_REF}-dirty"
    SRC_REV=unknown
  fi
fi

set --
if [ -n "${RIG_BUILD_NO_CACHE:-}" ]; then
  set -- --build-arg GLIM_DISABLE_CCACHE=1 "$@"
fi
if [ -n "${GLIM_BUILD_JOBS:-}" ]; then
  set -- --build-arg "BUILD_JOBS=${GLIM_BUILD_JOBS}" "$@"
fi
if [ -z "${RIG_BUILD_PLATFORMS:-}" ] && [ -n "${GLIM_BUILD_PLATFORM:-}" ]; then
  set -- --platform "$GLIM_BUILD_PLATFORM" "$@"
fi
rig_build "$IMAGE" "$@" \
  --build-arg "ROS_DISTRO=${ROS_DISTRO:-lyrical}" \
  --build-arg "BASE_IMAGE=${RIG_BASE_IMAGE:-ros:${ROS_DISTRO:-lyrical}-ros-core}" \
  --build-arg "GLIM_SERVICE_REF=${SRC_REF}" \
  --build-arg "GLIM_SERVICE_REV=${SRC_REV}" \
  -f "$REPO/docker/Dockerfile.runtime" "$REPO"
