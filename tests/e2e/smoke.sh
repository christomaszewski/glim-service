#!/usr/bin/env bash
# End-to-end smoke test: the real image, the real GLIM, driven through the real launcher.
#
#   GLIM_IMAGE=glim:ci tests/e2e/smoke.sh
#
# Synthetic Ouster-like clouds + an SBG-like IMU (tests/e2e/fake_sensors.py) feed a standalone
# instance configured from autonomy/glim.example.yaml. It asserts GLIM maps (odometry at the lidar
# rate, health active), that standby saves a loadable map dump with its session record, that a new
# activation opens a new session, and that `down` saves that one and exits 0. Needs Linux host
# networking (CI runners, a Jetson); Docker Desktop on macOS also works. RMW_IMPLEMENTATION picks
# the transport (default rmw_fastrtps_cpp; rmw_zenoh_cpp also starts a router).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${GLIM_IMAGE:?set GLIM_IMAGE to the image under test}"
WORK="$(mktemp -d)"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}"
export GLIM_IMAGE="$IMAGE" GLIM_DATA_DIR="$WORK/data" COMPOSE_PROJECT_NAME=glim_e2e_smoke
SENSORS=glim_e2e_sensors
ROUTER=glim_e2e_router
DOCKER_ROS=(--network host --ipc host --no-healthcheck -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION)

cleanup() {
  "$REPO/glim-up" "$WORK/slam.yaml" down >/dev/null 2>&1 || true
  docker rm -f "$SENSORS" "$ROUTER" >/dev/null 2>&1 || true
  docker volume rm glim_slam_maps >/dev/null 2>&1 || true
  rm -rf "$WORK" "$REPO/var/run/${COMPOSE_PROJECT_NAME}.target" "$REPO"/var/run/slam.*.instance.json
}
trap cleanup EXIT

fail() { echo "smoke: FAIL: $*" >&2; docker compose -p "$COMPOSE_PROJECT_NAME" logs --tail 60 >&2 || true; exit 1; }
step() { echo "smoke: $*" >&2; }
session_field() {  # <session dir> <field>
  python3 -c 'import json, sys; print(json.load(open(sys.argv[1] + "/session.json"))[sys.argv[2]])' "$1" "$2"
}

mkdir -p "$WORK/data"
cp "$REPO/autonomy/glim.example.yaml" "$WORK/slam.yaml"

if [ "$RMW_IMPLEMENTATION" = rmw_zenoh_cpp ]; then   # the fleet's RMW: sessions need a router
  step "starting a zenoh router"
  docker rm -f "$ROUTER" >/dev/null 2>&1 || true
  docker run -d --name "$ROUTER" "${DOCKER_ROS[@]}" "$IMAGE" ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null
fi

step "starting synthetic sensors (domain $ROS_DOMAIN_ID, $RMW_IMPLEMENTATION)"
docker rm -f "$SENSORS" >/dev/null 2>&1 || true
docker run -d --name "$SENSORS" "${DOCKER_ROS[@]}" -v "$REPO/tests/e2e:/e2e:ro" "$IMAGE" \
  python3 /e2e/fake_sensors.py >/dev/null

step "up -d"
"$REPO/glim-up" "$WORK/slam.yaml" up -d || fail "up did not converge"
state="$("$REPO/glim-up" "$WORK/slam.yaml" state 2>/dev/null)"
case "$state" in *'"state": "active"'*) ;; *) fail "not active after up: $state" ;; esac

step "waiting for odometry and health"
docker run --rm "${DOCKER_ROS[@]}" -v "$REPO/tests/e2e:/e2e:ro" "$IMAGE" \
  python3 /e2e/probe.py slam 8 90 || fail "GLIM is not mapping"

step "standby saves the map"
"$REPO/glim-up" "$WORK/slam.yaml" standby || fail "standby failed"
first="$(find "$WORK/data/maps/slam" -mindepth 1 -maxdepth 1 -type d | sort | head -1)"
[ "$(session_field "$first" outcome)" = saved ] || fail "first session outcome: $(session_field "$first" outcome)"
[ "$(session_field "$first" map_saved)" = True ] || fail "first session map not saved"
[ -s "$first/dump/graph.bin" ] || fail "no graph.bin in $first/dump"
[ -s "$first/dump/traj_lidar.txt" ] || fail "no trajectory in $first/dump"
[ -f "$first/config/config.json" ] || fail "session config missing"
[ "$(stat -c %u "$first/session.json" 2>/dev/null || stat -f %u "$first/session.json")" = "$(id -u)" ] \
  || fail "session files are not owned by the data dir's owner"

step "activate opens a new session; down saves it"
"$REPO/glim-up" "$WORK/slam.yaml" activate || fail "activate failed"
docker run --rm "${DOCKER_ROS[@]}" -v "$REPO/tests/e2e:/e2e:ro" "$IMAGE" \
  python3 /e2e/probe.py slam 8 90 || fail "GLIM is not mapping after activate"
"$REPO/glim-up" "$WORK/slam.yaml" down || fail "down reported a failed shutdown"
second="$(find "$WORK/data/maps/slam" -mindepth 1 -maxdepth 1 -type d | sort | tail -1)"
[ "$second" != "$first" ] || fail "activate did not open a new session"
[ "$(session_field "$second" outcome)" = saved ] || fail "second session outcome: $(session_field "$second" outcome)"
[ -s "$second/dump/graph.bin" ] || fail "no graph.bin after down"

step "PASS ($(wc -l < "$second/dump/traj_lidar.txt") poses in the last session)"
