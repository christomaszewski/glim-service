# glim-service — rig autonomy service for GLIM lidar-inertial SLAM

Runs [GLIM](https://github.com/koide3/glim) (Koide et al., range-inertial mapping on factor graphs)
as a [rig](https://github.com/christomaszewski/rig) **autonomy-tier** service: one config = one SLAM
instance that consumes a lidar point cloud and an IMU from the sensor tier (the `ouster` and `sbg`
services), publishes odometry, poses, TF and a map, and saves every mapping session — map, config,
logs, provenance — into the vehicle's run directory.

The repo is a wrapper: it holds no GLIM source. `glim.repos` pins GTSAM, gtsam_points, GLIM and
glim_ros2 to exact commits; the image build fetches and compiles them. Like every rig service it
depends on nothing from rig — rig depends on it through `rigging.yaml` and the `glim-up` launcher.

## How it works

```
 sensor tier                                   autonomy tier: one container per instance
 ouster `top`     /top/points ───────┐         ┌──────────────────────────────────────────────┐
 sbg `ins_main`   /ins_main/imu/data ┴────────►│ supervisor (python3 -m glim_supervisor)       │
                                               │   ├─ glim_rosnode   (one process per session) │──► /slam/glim_ros/{odom,pose,map,…}
 rig / glim-up ── standby|activate|state ─────►│   ├─ control socket                           │──► /tf  map → odom → base
 mission software ── ROS lifecycle ───────────►│   ├─ lifecycle node /slam/glim_supervisor     │──► /diagnostics (fleet health)
                                               │   └─ health monitor                           │
                                               └──────────────┬───────────────────────────────┘
                                                              ▼
                                   <data_dir>/current/maps/slam/<session>/{session.json,config,logs,dump}
```

- **`glim-up`** (the launcher) owns the per-instance mechanics: validate the config
  (`tools/render_config.py`), render the instance file, select + parameterize the static compose,
  honor rig's `COMPOSE_PROJECT_NAME`, and converge `up` to the requested operational state.
- **The supervisor** owns GLIM. Each *session* (one activation) gets a fresh directory and a GLIM
  config composed from the pinned upstream defaults for the selected pipeline plus the instance's
  overrides. Standby or shutdown sends GLIM SIGINT, which makes it finish optimizing and serialize
  its map; an unexpected exit while active is restarted with capped exponential backoff.
- **rig** owns tiers, ordering (autonomy comes up after every sensor and goes down first), the fleet
  env (ROS domain, RMW, vehicle id, registry/tag, data dir, sim time), and run directories.

## Layout

```
rigging.yaml                     rig descriptor: tier autonomy, verbs (incl. the state trio), build, replay
glim-up                          launcher (the rig contract surface)
glim.repos                       upstream pins (GTSAM 4.3a1, gtsam_points, GLIM, glim_ros2) — full SHAs
autonomy/glim.example.yaml       annotated example config, wired to ouster `top` + sbg `ins_main`
tools/render_config.py           config validation -> identity env + instance file (launch surface)
tools/runtime_images.py          rig runtime-image planning / pinned up (shared helper, launch surface)
tools/build_image.sh             `rig build` entrypoint; tools/rig-build.sh is rig's shared build helper
tools/fetch_upstream.py          shallow-fetch the pinned SHAs (image build and local dev)
docker/Dockerfile.runtime        build GTSAM -> gtsam_points -> GLIM -> glim_ros2; slim runtime + gates
docker/compose/compose.deploy.yaml   the per-instance compose; compose.datadir.yaml maps -> data dir
docker/entrypoints/              ros-entrypoint.sh (ROS env, zenoh overrides), glim-statectl (state client)
supervisor/glim_supervisor/      the in-container supervisor (controller, config, health, ROS, socket)
tests/, tools/test_build_contract.py   regression suites (host, and inside the image in CI)
```

## Quick start (standalone, no rig)

```bash
# Build the runtime image (the C++ compile took ~2.5 min with 8 jobs on an arm64 workstation, cold
# cache; expect several times that on a Jetson). PUSH=0 keeps it local as local/glim:dev; under rig,
# `rig build` runs the same script and pushes to the registry.
PUSH=0 tools/build_image.sh local dev

# One config per instance: copy the example, then set the topics and the lidar–IMU extrinsic.
cp autonomy/glim.example.yaml autonomy/slam.yaml

GLIM_IMAGE=local/glim:dev ./glim-up autonomy/slam.yaml up -d   # waits until "active"
./glim-up autonomy/slam.yaml state                             # {"state": "active", ...}
./glim-up autonomy/slam.yaml logs -f
./glim-up autonomy/slam.yaml down                              # saves the map, then removes
```

Standalone maps go to the `glim_<name>_maps` volume, or to a host directory with
`GLIM_DATA_DIR=/path`. The launcher needs Docker Compose v2 and Python 3 with PyYAML
(`GLIM_PYTHON` selects the interpreter).

## Wiring it into a rig deployment

```yaml
# services.yaml
services:
  glim: { path: ../glim-service }

# vehicle.yaml — GLIM consumes the sensor tier, so it lives under autonomy:
sensors:
  - { name: ins_main, service: sbg,    config: config/sensors/ins_main.yaml, enabled: true, order: 20 }
  - { name: top,      service: ouster, config: config/sensors/top.yaml,      enabled: true, order: 30 }
autonomy:
  - { name: slam,     service: glim,   config: config/autonomy/slam.yaml,    enabled: true, order: 10 }
```

`rig add ../glim-service` does the routing and copies the example to `config/autonomy/` (autonomy
rows arrive as a commented menu entry). Then `rig doctor`, `rig up`, `rig status` (OP column) and
`rig standby` / `rig activate` work as for any service. Set `data_dir` on the vehicle: maps then
land in the open run (`<data_dir>/current/maps/slam/…`), travel with it through `rig run export`
and `rig fleet sync`, and are never touched by `rig down --purge`.

Before the first real run, check the sensor configs too:

| Where | Setting | Why |
|---|---|---|
| ouster `driver_params.point_type` | `original` (the ouster example's) | carries the per-point `t` GLIM deskews scans with; `xyz`/`xyzi` do not |
| ouster `driver_params.timestamp_mode` + sbg `time.source` | one shared clock | see [Time](#time-synchronization) |
| sbg `driver_params.convention.use_enu` | `true` | REP-103 FLU IMU frame (otherwise express FRD in the extrinsic) |
| sbg `driver_params.tf.broadcast_odom_to_base` | `false` if GLIM owns `odom → base_link` | a frame has one parent |
| both `zenoh.shared_memory` (rmw_zenoh) | `true` here and on the ouster instance | point clouds otherwise cross TCP loopback |

## Configuration

`autonomy/glim.example.yaml` is the annotated reference; `tools/render_config.py` is the schema.
Unknown keys anywhere in the wrapper blocks are errors — a typo must not pass as configured.

| Block | Purpose |
|---|---|
| `service`, `name`, `ros.namespace` | identity: compose project `glim_<name>` standalone, ROS namespace `/<namespace>` (default `name`) |
| `initial_state` | `active` (default) or `standby`; `RIG_TARGET_STATE` (`rig up --standby`) wins |
| `ros.use_sim_time` | omit to follow rig replay (`RIG_SIM_TIME`); explicit `true`/`false` wins both ways |
| `inputs.points`, `inputs.imu` | the `PointCloud2` / `Imu` topics and optional QoS (profile name or mapping; omitted keys keep GLIM's defaults, e.g. the IMU's 1000-deep queue) |
| `extrinsics.T_lidar_imu` | **required** — IMU pose in the lidar frame, `[x y z qx qy qz qw]`, unit quaternion enforced |
| `frames` | `map`, `odom`, `base` (`""` = IMU frame), `imu`/`lidar` (`""` = from headers), `publish_imu_to_lidar` |
| `pipeline` | GLIM module preset: `lightweight` (default), `cpu`, `odometry` — see below |
| `glim` | opaque per-module overrides (`ros`, `sensors`, `preprocess`, `odometry`, `sub_mapping`, `global_mapping`) |
| `map.save_timeout_s` | budget for GLIM's final optimization + save on standby/stop; sizes the stop grace period |
| `health` | `/diagnostics` interval, stall timeout, expected odometry rate, producer `limits` |
| `zenoh` | rmw_zenoh session overrides — the fleet-uniform block the ouster wrapper and bag logger take |

Pipelines (all CPU; the image is built without CUDA):

| `pipeline` | Odometry | Sub-mapping | Global mapping | Use |
|---|---|---|---|---|
| `lightweight` | `odometry_cpu` (iVox GICP) | `passthrough` | `pose_graph` (loop closure) | real-time on an embedded CPU |
| `cpu` | `odometry_cpu` | `sub_mapping_cpu` (VGICP) | `global_mapping_cpu` (VGICP) | better maps, much heavier |
| `odometry` | `odometry_cpu` | — | — | odometry only; nothing to save |

The `glim` block layers onto upstream's defaults for the chosen pipeline (each key replaces one
upstream key). Keys the wrapper derives — topics, QoS, frames, `T_lidar_imu`, `so_name`, the mapping
switches — are rejected there, so each setting has exactly one spelling. Keys absent from
upstream's defaults are applied but reported as a WARN on `/diagnostics` and in `session.json`
(GLIM silently ignores unknown keys). Defaults are in the image under
`/opt/glim/share/glim/config/`; tuning guidance: <https://koide3.github.io/glim/parameters/>.

### Time synchronization

GLIM fuses each scan with the IMU samples spanning it, so both streams must be stamped on **one
clock**. Ouster's default `TIME_FROM_INTERNAL_OSC` counts from sensor power-on and will not line up
with the SBG. Either discipline both to one time source (Ouster `TIME_FROM_PTP_1588` against the
SBG's PTP output, with sbg `time.source: device_utc`), or stamp both on arrival (Ouster
`TIME_FROM_ROS_TIME`, sbg `time.source: receive_time`, the SBG default). GLIM logs
`large time difference between points and imu` when they disagree.

### TF ownership

GLIM always broadcasts `map → odom → <base>` and, with `publish_imu_to_lidar`, `<imu> → <lidar>`.
A frame can have only one parent, so decide ownership before enabling it on a vehicle:

- **Default (`base: ""`)** — GLIM's tracked frame is the IMU frame (`odom → imu_link`). Works with
  no URDF; nothing else may parent the IMU frame.
- **With a URDF / static TF tree** — set `base: base_link`; GLIM looks up `<imu> → base_link`
  and publishes `odom → base_link`. Turn off the SBG's `tf.broadcast_odom_to_base`.
- Leave `publish_imu_to_lidar: false` when the URDF or the Ouster's static TF connects the lidar frame.

## Operational states

The service implements rig's operational-state trio (`rigging.yaml` declares all three):

```bash
./glim-up autonomy/slam.yaml standby    # finish + save the current map, stop GLIM (container stays up)
./glim-up autonomy/slam.yaml activate   # start a NEW mapping session
./glim-up autonomy/slam.yaml state      # {"state": "active|standby|transitioning|down", "target": …, "detail": …}
rig standby slam / rig activate slam / rig up --standby   # the same, fleet-wide
```

- **active** = a session is running and GLIM's node is on the ROS graph; **standby** = no session,
  last map saved. Each activation starts a new session (a fresh map); there is no pause.
- The target persists across container restarts (`restart: unless-stopped`); `up` re-asserts the
  requested target over the control socket and waits for it (budget `GLIM_SETTLE_TIMEOUT`, default
  stop grace + 60 s). The startup target reaches a new container through a mounted file, never the
  Compose environment, so `rig up` after `rig activate` leaves the running session alone; a changed
  *config* does recreate the container, which saves the running map first.
- **Mission software** can drive the same states with standard ROS lifecycle calls — the
  supervisor's lifecycle node mirrors the target (`inactive` ⇔ standby, `active` ⇔ active), and
  both paths go through one controller, so `rig status` always agrees with ROS:

  ```bash
  ros2 lifecycle set /slam/glim_supervisor activate     # start mapping
  ros2 lifecycle set /slam/glim_supervisor deactivate   # save + stop
  ```

  A lifecycle `shutdown` parks GLIM and retires the lifecycle interface until the container restarts.
- `down` (and `docker stop`) saves the running session's map first. A save that misses its budget
  (`map.save_timeout_s`) or fails makes `standby` exit non-zero, the container exit non-zero on
  stop (so `down` reports it), and `/diagnostics` WARN until the next clean save.
- Health is readiness, not data flow: a parked standby is healthy; the container turns unhealthy
  only when GLIM keeps failing (three consecutive failed sessions).

## Map sessions

```
<data_dir>/current/maps/<name>/<UTC start>/     # flat <data_dir>/maps/… without an open run;
  session.json     instance, pipeline, inputs,  #   the glim_<name>_maps volume without a data dir
                   start/end, outcome (saved | save_timeout | failed | exited | failed_to_start),
                   map_saved, config warnings, upstream SHAs, wrapper build identity
  config/          the exact GLIM config the session ran with
  logs/            GLIM's per-module logs
  dump/            GLIM's map: factor graph, submaps, odom_*/traj_* trajectories (TUM format)
```

A session pins the run that was open when it **started** (rig's writer rule), and files are
chowned to the data dir's owner. Open `dump/` with GLIM's `offline_viewer` on a desktop (it is not
built into this headless image) to inspect, add manual loop closures, or export a PLY point cloud.

## ROS interface

All under `/<ns>/glim_ros/` (GLIM's node) unless absolute. Most topics publish only while subscribed.

| Topic | Type | Content |
|---|---|---|
| `odom`, `odom_corrected` | `nav_msgs/Odometry` | IMU-frame odometry in `odom`; `_corrected` after global optimization |
| `pose`, `pose_corrected`, `pose_corrected_with_cov` | `geometry_msgs/PoseStamped` (+ covariance) | IMU pose in `map` |
| `lidar_odom*`, `lidar_pose*` | as above | the same, for the lidar frame |
| `*_scanend*` | as above | at the end of each scan instead of its start |
| `points`, `aligned_points`, `*_corrected` | `sensor_msgs/PointCloud2` | each preprocessed scan, in its sensor frame / in `map` |
| `map` | `sensor_msgs/PointCloud2` | merged global map (transient local, ≤ every 10 s) |
| `imu_bias`, `imu_bias_with_cov` | `geometry_msgs/Twist*` | estimated IMU biases |
| `/tf` | `tf2_msgs/TFMessage` | `map → odom → base` (+ `imu → lidar`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | fleet health (below) |
| `/<ns>/glim_supervisor/*` | lifecycle services | `change_state`, `get_state`, … |

## Health

The supervisor publishes `/diagnostics` every `health.interval_s`, following the fleet contract
(`docs/SERVICE_HEALTH.md` in the ouster/dashboard repos): identity metadata (`health.vehicle_id`,
`health.instance`, `health.service`, sequence, availability, …) on every status, and components:

| Component | Values | Levels |
|---|---|---|
| `<name>: service` | `op.state`, `op.target`, `glim.session`, `glim.restarts`, `uptime_s` | ERROR while GLIM keeps failing; WARN on config warnings |
| `<name>: odometry` | `fps.delivered`, `frame_age_s`, `fps.expected` | WARN when stalled while active |
| `<name>: imu` | `fps.delivered`, `frame_age_s` | WARN when the IMU input stalls while active |
| `<name>: points` | `publishers` | WARN with no publisher (graph only: no second subscriber to the cloud) |
| `<name>: storage` | `disk.free_gb`, `disk.free_pct` | producer `limits` only |

Stream checks are demanded only while active (standby reports them `paused`), with a short grace
after each session start. Thresholds are producer configuration (`health.limits`), never built in.

## Replay

`rigging.yaml` declares `replay: {sim_time: true}`: under `rig replay <run> slam` the launcher
sets `use_sim_time` from `RIG_SIM_TIME`, and the bag player feeds the recorded point clouds and
IMU (the bag logger must have recorded the lidar topic). The new session's map lands in the replay
run beside the source run's, which makes A/B comparison of GLIM tuning against one recording a
matter of editing `glim:` and replaying again. Keep the playback rate at or below what the
pipeline sustains in real time.

## Building and verification

- `tools/build_image.sh <registry> [tag]` is `rigging.yaml`'s `build.command`. It honors rig's build
  env (`RIG_BASE_IMAGE`, `RIG_BUILD_PLATFORMS`, `RIG_BUILD_NO_CACHE`, `RIG_BUILD_RESULT`) and
  `GLIM_BUILD_JOBS` (GTSAM needs about 2 GB of RAM per compile job). Build arm64 natively.
- GTSAM is built from source on purpose: GLIM saves maps through GTSAM's Boost serialization, which
  the ROS-packaged `gtsam` disables (and builds with TBB, against GLIM's recommendation).
- The image's build gates: every shipped library's dependencies are derived from the binaries
  (`ldd` → `dpkg -S`), every library must load fully bound in the runtime stage, and the ros-*
  versions must match what GLIM compiled against.
- CI (`.github/workflows/ci.yml`): image build + in-image tests on amd64 and arm64, `rig certify
  --require-build-check`, host regressions, build-contract tests, shellcheck.

```bash
python3 -m unittest discover -s tests -v                                 # host (config tests skip)
docker run --rm -v "$PWD:/workspace:ro" -w /workspace glim:dev python3 -m unittest discover -s tests
../rig/rig certify --repo . --config autonomy/glim.example.yaml
```

## Updating upstream

1. Pick the new GLIM / glim_ros2 commits; they move together (glim_ros2 follows glim's API).
2. Check what they need below them: GLIM's `find_package(gtsam_points …)` version and any new
   gtsam_points API; gtsam_points' supported GTSAM. Note that this GLIM still uses
   `gtsam::make_shared`, which GTSAM 4.3.0 removed — move to 4.3.0 only once GLIM has.
3. Update the full SHAs (and the comments) in `glim.repos`, build the image, run the tests in it,
   and diff the upstream default configs for renamed keys (`git diff <old>..<new> -- config/`).

## Limitations

- CPU-only. GLIM's GPU modules need a CUDA build variant per runtime (rig `build.platforms`, e.g.
  the fleet's `jp6`/`jp7`) and a CUDA userspace compatible with the Lyrical base; not built yet.
- No camera or `glim_ext` extension modules (no OpenCV/cv_bridge in the image).

## License

The wrapper is Apache-2.0 (see `LICENSE`). The image contains GLIM and glim_ros2 (MIT), gtsam_points
(MIT) and GTSAM (BSD-3-Clause), each under its own license.
