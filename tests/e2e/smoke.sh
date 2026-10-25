#!/usr/bin/env bash
# End-to-end smoke test: the real image, the real GLIM, driven through the real launcher.
#
#   GLIM_IMAGE=glim:ci tests/e2e/smoke.sh
#
# Synthetic Ouster-like clouds, an SBG-like IMU and its fused odometry (tests/e2e/fake_sensors.py)
# feed a standalone instance configured from autonomy/glim.example.yaml plus an evaluation block.
# Phases:
#   live      GLIM maps (odometry at the lidar rate, health active, map -> odom on /tf); standby
#             saves a loadable dump, the PLY export and the session's accuracy evaluation
#   remap     a recorded bag of the same streams is mapped offline (`glim-up remap`) into its own
#             session, evaluated against the reference read from the bag
#   restart   activate opens a new session; `down` saves it and exits 0
#   tf off    frames.publish_tf: false — a config change recreates the container (saving first)
#             and GLIM maps without broadcasting TF
# Needs Linux host networking (CI runners, a Jetson); Docker Desktop on macOS also works.
# RMW_IMPLEMENTATION picks the transport (default rmw_fastrtps_cpp; rmw_zenoh_cpp also starts a
# router). Recording the bag needs `ros2 bag` in the image or apt access to install it.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${GLIM_IMAGE:?set GLIM_IMAGE to the image under test}"
WORK="$(mktemp -d)"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}"
export GLIM_IMAGE="$IMAGE" GLIM_DATA_DIR="$WORK/data" COMPOSE_PROJECT_NAME=glim_e2e_smoke
SENSORS=glim_e2e_sensors
ROUTER=glim_e2e_router
RECORDER=glim_e2e_recorder
DOCKER_ROS=(--network host --ipc host --no-healthcheck -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION)

cleanup() {
  "$REPO/glim-up" "$WORK/slam.yaml" down >/dev/null 2>&1 || true
  docker rm -f "$SENSORS" "$ROUTER" "$RECORDER" >/dev/null 2>&1 || true
  docker volume rm glim_slam_maps >/dev/null 2>&1 || true
  # the recorder ran as root: hand its files back before removing the work dir
  docker run --rm --no-healthcheck -v "$WORK:/w" "$IMAGE" chown -R "$(id -u):$(id -g)" /w >/dev/null 2>&1 || true
  rm -rf "$WORK" "$REPO/var/run/${COMPOSE_PROJECT_NAME}.target" "$REPO"/var/run/slam.*.instance.json
}
trap cleanup EXIT

fail() { echo "smoke: FAIL: $*" >&2; docker compose -p "$COMPOSE_PROJECT_NAME" logs --tail 60 >&2 || true; exit 1; }
step() { echo "smoke: $*" >&2; }
json_field() {  # <json file> <field> [subfield]
  python3 -c 'import json, sys
d = json.load(open(sys.argv[1]))
for k in sys.argv[2:]:
    d = d[k]
print(d)' "$@"
}
probe() { docker run --rm "${DOCKER_ROS[@]}" -v "$REPO/tests/e2e:/e2e:ro" "$IMAGE" python3 /e2e/probe.py slam 8 90 "$@"; }
ply_vertices() { python3 -c 'import sys
for line in open(sys.argv[1], "rb"):
    if line.startswith(b"element vertex"):
        print(int(line.split()[2])); break' "$1"; }
sessions() { find "$WORK/data/maps/slam" -mindepth 1 -maxdepth 1 -type d | sort; }

mkdir -p "$WORK/data"
cp "$REPO/autonomy/glim.example.yaml" "$WORK/slam.yaml"
printf '\nevaluation:\n  reference_topic: /ins_main/odom\n' >> "$WORK/slam.yaml"

if [ "$RMW_IMPLEMENTATION" = rmw_zenoh_cpp ]; then   # the fleet's RMW: sessions need a router
  step "starting a zenoh router"
  docker rm -f "$ROUTER" >/dev/null 2>&1 || true
  docker run -d --name "$ROUTER" "${DOCKER_ROS[@]}" "$IMAGE" ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null
fi

step "starting synthetic sensors (domain $ROS_DOMAIN_ID, $RMW_IMPLEMENTATION)"
docker rm -f "$SENSORS" >/dev/null 2>&1 || true
docker run -d --name "$SENSORS" "${DOCKER_ROS[@]}" -v "$REPO/tests/e2e:/e2e:ro" "$IMAGE" \
  python3 /e2e/fake_sensors.py >/dev/null

# --- live ------------------------------------------------------------------------------------
step "live: up -d"
"$REPO/glim-up" "$WORK/slam.yaml" up -d || fail "up did not converge"
state="$("$REPO/glim-up" "$WORK/slam.yaml" state 2>/dev/null)"
case "$state" in *'"state": "active"'*) ;; *) fail "not active after up: $state" ;; esac
step "live: waiting for odometry, health and TF"
probe expect || fail "GLIM is not mapping"
step "live: standby saves the map, its PLY export and its evaluation"
"$REPO/glim-up" "$WORK/slam.yaml" standby || fail "standby failed"
first="$(sessions | head -1)"
[ "$(json_field "$first/session.json" outcome)" = saved ] || fail "first session outcome"
[ "$(json_field "$first/session.json" map_saved)" = True ] || fail "first session map not saved"
[ -s "$first/dump/graph.bin" ] || fail "no graph.bin in $first/dump"
[ -s "$first/dump/traj_lidar.txt" ] || fail "no trajectory in $first/dump"
[ -f "$first/config/config.json" ] || fail "session config missing"
[ "$(ply_vertices "$first/map.ply")" -gt 1000 ] || fail "map.ply missing or nearly empty"
ate="$(json_field "$first/evaluation.json" ate_rmse_m)" || fail "no evaluation.json"
python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < 0.5 else 1)" "$ate" \
  || fail "static scene ATE $ate m"
[ "$(stat -c %u "$first/session.json" 2>/dev/null || stat -f %u "$first/session.json")" = "$(id -u)" ] \
  || fail "session files are not owned by the data dir's owner"
step "live: $(ply_vertices "$first/map.ply") map points exported; ATE ${ate} m"

# --- remap -----------------------------------------------------------------------------------
step "remap: recording 20 s of the sensors"
docker rm -f "$RECORDER" >/dev/null 2>&1 || true
docker run -d --name "$RECORDER" "${DOCKER_ROS[@]}" -v "$WORK:/w" "$IMAGE" bash -c '
  ros2 bag --help >/dev/null 2>&1 || { apt-get update -qq >/dev/null &&
    apt-get install -y -qq "ros-${ROS_DISTRO}-ros2bag" "ros-${ROS_DISTRO}-rosbag2-storage-mcap" >/dev/null; }
  exec ros2 bag record -s mcap -o /w/bag --topics /top/points /ins_main/imu/data /ins_main/odom' >/dev/null
for _ in $(seq 1 120); do [ -d "$WORK/bag" ] && break; sleep 1; done
[ -d "$WORK/bag" ] || { docker logs "$RECORDER" >&2; fail "the recorder never started"; }
sleep 20
docker kill -s INT "$RECORDER" >/dev/null
docker wait "$RECORDER" >/dev/null
docker run --rm --no-healthcheck -v "$WORK:/w" "$IMAGE" chown -R "$(id -u):$(id -g)" /w/bag >/dev/null
step "remap: mapping the bag offline"
summary="$("$REPO/glim-up" "$WORK/slam.yaml" remap "$WORK/bag")" || fail "remap failed: $summary"
remapped="$WORK/data/maps/slam/$(python3 -c 'import json, sys; print(json.loads(sys.argv[1])["session"])' "$summary")"
[ "$(json_field "$remapped/session.json" mode)" = remap ] || fail "remap session record"
[ "$(json_field "$remapped/session.json" outcome)" = saved ] || fail "remap outcome"
[ -s "$remapped/dump/graph.bin" ] || fail "remap saved no map"
[ "$(ply_vertices "$remapped/map.ply")" -gt 1000 ] || fail "remap exported no PLY"
json_field "$remapped/evaluation.json" ate_rmse_m >/dev/null || fail "remap was not evaluated"
step "remap: $(wc -l < "$remapped/dump/traj_lidar.txt") poses, ATE $(json_field "$remapped/evaluation.json" ate_rmse_m) m"

# --- restart ---------------------------------------------------------------------------------
step "restart: activate opens a new session; down saves it"
"$REPO/glim-up" "$WORK/slam.yaml" activate || fail "activate failed"
probe expect || fail "GLIM is not mapping after activate"
"$REPO/glim-up" "$WORK/slam.yaml" down || fail "down reported a failed shutdown"
latest="$(sessions | grep -v -- -remap | tail -1)"
[ "$latest" != "$first" ] || fail "activate did not open a new session"
[ "$(json_field "$latest/session.json" outcome)" = saved ] || fail "restarted session outcome"

# --- tf off ----------------------------------------------------------------------------------
step "tf off: frames.publish_tf: false"
python3 - "$WORK/slam.yaml" <<'PY'
import sys
path = sys.argv[1]
lines = open(path).read().splitlines(keepends=True)
at = next((i for i, line in enumerate(lines) if line.startswith("  publish_tf:")), None)
if at is None:   # not in the example: add it under frames
    at = next(i for i, line in enumerate(lines) if line.startswith("  publish_imu_to_lidar:")) + 1
    lines.insert(at, "")
lines[at] = "  publish_tf: false\n"
open(path, "w").write("".join(lines))
PY
grep -q '^  publish_tf: false' "$WORK/slam.yaml" || fail "could not set publish_tf: false"
"$REPO/glim-up" "$WORK/slam.yaml" up -d || fail "up with publish_tf: false"
probe forbid || fail "GLIM broadcast TF with publish_tf: false (or stopped mapping)"
"$REPO/glim-up" "$WORK/slam.yaml" down || fail "final down"

step "PASS ($(sessions | wc -l | tr -d ' ') sessions)"
