#!/usr/bin/env python3
# Copyright 2026 Chris Tomaszewski
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Validate a *rig* GLIM instance config and render what `glim-up` needs from it.

The instance YAML is the single source of truth. Wrapper blocks (`inputs`, `extrinsics`, `frames`,
`pipeline`, `map`, `health`, `zenoh`) are validated here and DERIVED onto GLIM's own config keys;
the `glim:` block is OPAQUE per-module overrides, layered over the pinned upstream defaults inside
the container (where those defaults live). Keys the wrapper derives are rejected inside `glim:` —
each has exactly one spelling. Used by `glim-up`:

  render_config.py --env0 <config.yaml>                     # identity env, NUL-delimited pairs
  render_config.py --instance [--sim-time V] <config.yaml>  # the instance JSON the supervisor reads

Output is deterministic (sorted keys, no timestamps): `rig certify` and `rig bake` compare renders.
Needs PyYAML on the host (apt: python3-yaml).
"""
import json
import math
import re
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.stderr.write("render_config: PyYAML required on the host (apt install python3-yaml)\n")
    sys.exit(2)

INSTANCE_SCHEMA_VERSION = 1

# Instance identity tokens end up in compose project names, ROS namespaces, volume names and
# file paths — keep them to a safe, unambiguous alphabet.
_IDENT_RE = re.compile(r"^[a-z][a-z0-9_]*$")
# A ROS topic name: absolute (/top/points) or relative to GLIM's namespace (points); no `~`.
_TOPIC_RE = re.compile(r"^/?[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$")
# A tf2 frame id: no leading slash (tf2 rejects it), no whitespace.
_FRAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_./-]*$")

TOP_KEYS = {"service", "name", "initial_state", "ros", "inputs", "extrinsics", "frames",
            "pipeline", "glim", "map", "health", "zenoh"}

# Module presets, all CPU (this image is built without CUDA). Values are upstream config file
# names inside the image plus the two backend switches GLIM's ROS node reads.
PIPELINES = ("lightweight", "cpu", "odometry")

# `glim:` override modules -> the upstream JSON section they patch.
GLIM_MODULES = {"ros": "glim_ros", "sensors": "sensors", "preprocess": "preprocess",
                "odometry": "odometry_estimation", "sub_mapping": "sub_mapping",
                "global_mapping": "global_mapping"}
# Keys the wrapper derives, by override module -> the wrapper block that owns them.
RESERVED = {
    "ros": {"imu_topic": "inputs.imu.topic", "points_topic": "inputs.points.topic",
            "image_topic": "(camera extensions are not built into this image)",
            "imu_qos": "inputs.imu.qos", "points_qos": "inputs.points.qos",
            "image_qos": "(camera extensions are not built into this image)",
            "imu_frame_id": "frames.imu", "lidar_frame_id": "frames.lidar",
            "base_frame_id": "frames.base", "odom_frame_id": "frames.odom",
            "map_frame_id": "frames.map", "publish_imu2lidar": "frames.publish_imu_to_lidar",
            "enable_local_mapping": "pipeline", "enable_global_mapping": "pipeline"},
    "sensors": {"T_lidar_imu": "extrinsics.T_lidar_imu"},
}
SO_NAME_OWNER = "pipeline"  # every module's so_name is selected by the pipeline preset

QOS_PROFILES = ("default", "system_default", "sensor_data", "services_default", "parameters",
                "parameter_events")
QOS_POLICIES = {
    "reliability": ("system_default", "reliable", "best_effort", "best_available"),
    "durability": ("system_default", "transient_local", "volatile", "best_available"),
    "history": ("system_default", "keep_last", "keep_all"),
}
# GLIM's own subscription defaults (glim_ros.cpp): IMU sensor_data with a deep queue — IMU arrives
# at hundreds of Hz and a dropped sample is a hole in the preintegration.
DEFAULT_QOS = {"imu": {"profile": "sensor_data", "depth": 1000}, "points": {"profile": "sensor_data"}}

# /diagnostics components (`<instance>: <component>`) and the metrics producer limits may bound.
HEALTH_COMPONENTS = {
    "service": (),
    "odometry": ("fps.delivered", "frame_age_s"),
    "imu": ("fps.delivered", "frame_age_s"),
    "points": (),
    "storage": ("disk.free_gb", "disk.free_pct"),
}
LIMIT_BOUNDS = ("warn_above", "warn_below", "error_above", "error_below")

SAVE_TIMEOUT_DEFAULT_S = 240
STOP_MARGIN_S = 30  # supervisor + container teardown on top of the map save budget


class ConfigError(Exception):
    pass


def _mapping(value, where):
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ConfigError(f"{where} must be a mapping")
    return value


def _unknown(block, allowed, where):
    extra = sorted(map(str, set(block) - set(allowed)))   # YAML keys can be ints, dates, ...
    if extra:
        raise ConfigError(f"unknown {where} key(s): {', '.join(extra)} "
                          f"(supported: {', '.join(sorted(allowed))})")


def _ident(what, value):
    if not isinstance(value, str) or not _IDENT_RE.match(value):
        raise ConfigError(f"{what} {value!r} must match [a-z][a-z0-9_]* "
                          "(it becomes a compose project / ROS namespace / volume name)")
    return value


def _number(value, where, *, positive=False, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f"{where} must be a finite number")
    if positive and value <= 0:
        raise ConfigError(f"{where} must be positive")
    if minimum is not None and value < minimum:
        raise ConfigError(f"{where} must be >= {minimum}")
    return value


def _bool(value, where):
    if not isinstance(value, bool):
        raise ConfigError(f"{where} must be true or false")
    return value


def derive_qos(value, default, where):
    """None -> GLIM's own default; a profile name; or {profile, depth, reliability, durability,
    history}, layered over that default. Layering matters: GLIM treats ANY configured key as a
    complete spec, so `{reliability: reliable}` alone would shrink the IMU queue from 1000 to the
    profile's 5. Validated here because GLIM only WARNs on unknown values and falls back."""
    if value is None:
        return dict(default)
    if isinstance(value, str):
        value = {"profile": value}
    block = _mapping(value, where)
    _unknown(block, ("profile", "depth", *QOS_POLICIES), where)
    out = dict(default)
    if "profile" in block:
        if block["profile"] not in QOS_PROFILES:
            raise ConfigError(f"{where}.profile {block['profile']!r} must be one of "
                              f"{', '.join(QOS_PROFILES)}")
        out["profile"] = block["profile"]
    if "depth" in block:
        depth = block["depth"]
        if isinstance(depth, bool) or not isinstance(depth, int) or depth < 1:
            raise ConfigError(f"{where}.depth must be a positive integer")
        out["depth"] = depth
    for key, allowed in QOS_POLICIES.items():
        if key in block:
            if block[key] not in allowed:
                raise ConfigError(f"{where}.{key} {block[key]!r} must be one of {', '.join(allowed)}")
            out[key] = block[key]
    return out


def derive_inputs(inputs):
    inputs = _mapping(inputs, "inputs")
    _unknown(inputs, ("points", "imu"), "inputs")
    out = {}
    for stream in ("points", "imu"):
        block = _mapping(inputs.get(stream), f"inputs.{stream}")
        _unknown(block, ("topic", "qos"), f"inputs.{stream}")
        topic = block.get("topic")
        if not topic:
            raise ConfigError(f"inputs.{stream}.topic is required — the "
                              f"{'sensor_msgs/PointCloud2' if stream == 'points' else 'sensor_msgs/Imu'}"
                              " topic GLIM subscribes to (e.g. "
                              f"{'/top/points' if stream == 'points' else '/ins_main/imu/data'})")
        if not isinstance(topic, str) or not _TOPIC_RE.match(topic):
            raise ConfigError(f"inputs.{stream}.topic {topic!r} is not a ROS topic name")
        out[stream] = {"topic": topic,
                       "qos": derive_qos(block.get("qos"), DEFAULT_QOS[stream], f"inputs.{stream}.qos")}
    return out


def derive_extrinsics(extrinsics):
    """T_lidar_imu is REQUIRED: upstream's default is an Ouster-internal-IMU offset, silently wrong
    for an external IMU — and a wrong extrinsic degrades the map without failing anything."""
    extrinsics = _mapping(extrinsics, "extrinsics")
    _unknown(extrinsics, ("T_lidar_imu",), "extrinsics")
    value = extrinsics.get("T_lidar_imu")
    if value is None:
        raise ConfigError("extrinsics.T_lidar_imu is required: the IMU pose in the lidar frame, "
                          "[x, y, z, qx, qy, qz, qw] (p_lidar = T_lidar_imu * p_imu)")
    if not isinstance(value, list) or len(value) != 7:
        raise ConfigError("extrinsics.T_lidar_imu must be [x, y, z, qx, qy, qz, qw]")
    values = [float(_number(v, f"extrinsics.T_lidar_imu[{i}]")) for i, v in enumerate(value)]
    norm = math.sqrt(sum(q * q for q in values[3:]))
    if abs(norm - 1.0) > 1e-3:
        raise ConfigError(f"extrinsics.T_lidar_imu quaternion has norm {norm:.6f}; it must be a unit "
                          "quaternion in [qx, qy, qz, qw] order (TUM), not Euler angles")
    return values[:3] + [q / norm for q in values[3:]]


def derive_frames(frames):
    frames = _mapping(frames, "frames")
    _unknown(frames, ("map", "odom", "base", "imu", "lidar", "publish_imu_to_lidar"), "frames")
    out = {}
    for key, default, required in (("map", "map", True), ("odom", "odom", True),
                                   ("base", "", False), ("imu", "", False), ("lidar", "", False)):
        value = frames.get(key, default)
        if value is None:
            value = ""
        if not isinstance(value, str) or (value and not _FRAME_RE.match(value)):
            raise ConfigError(f"frames.{key} {value!r} is not a tf2 frame id (no leading '/')")
        if required and not value:
            raise ConfigError(f"frames.{key} must not be empty")
        out[key] = value
    if out["map"] == out["odom"]:
        raise ConfigError("frames.map and frames.odom must differ (GLIM publishes map -> odom)")
    for key in ("base", "imu", "lidar"):
        if out[key] in (out["map"], out["odom"]):
            raise ConfigError(f"frames.{key} {out[key]!r} would make GLIM publish a transform from "
                              f"a frame to itself (it publishes map -> odom -> base)")
    out["publish_imu_to_lidar"] = _bool(frames.get("publish_imu_to_lidar", False),
                                        "frames.publish_imu_to_lidar")
    return out


def derive_glim(glim):
    glim = _mapping(glim, "glim")
    _unknown(glim, GLIM_MODULES, "glim")
    out = {}
    for module, block in glim.items():
        block = _mapping(block, f"glim.{module}")
        for key in block:
            if not isinstance(key, str):
                raise ConfigError(f"glim.{module} key {key!r} must be a string")
            owner = RESERVED.get(module, {}).get(key)
            if owner:
                raise ConfigError(f"glim.{module}.{key} is derived by this wrapper; set {owner} instead")
            if key == "so_name":
                raise ConfigError(f"glim.{module}.so_name is selected by `{SO_NAME_OWNER}`")
        if block:
            out[module] = block
    exts = (out.get("ros") or {}).get("extension_modules")
    if exts is not None:
        if not isinstance(exts, list) or not all(isinstance(e, str) and e for e in exts):
            raise ConfigError("glim.ros.extension_modules must be a list of library names")
        if "librviz_viewer.so" not in exts:
            sys.stderr.write("render_config: glim.ros.extension_modules omits librviz_viewer.so — "
                             "GLIM will publish NO ROS outputs (odometry, poses, TF, map)\n")
    return out


def derive_map(block):
    block = _mapping(block, "map")
    _unknown(block, ("save_timeout_s",), "map")
    timeout = _number(block.get("save_timeout_s", SAVE_TIMEOUT_DEFAULT_S), "map.save_timeout_s",
                      positive=True)
    return {"save_timeout_s": float(timeout)}


def derive_health(health):
    """Fleet /diagnostics settings (docs/SERVICE_HEALTH.md). Producer-side thresholds only — no
    built-in limits beyond the stall timeouts, which only WARN."""
    health = _mapping(health, "health")
    _unknown(health, ("interval_s", "stale_after_s", "stall_timeout_s", "expected_odometry_hz",
                      "limits"), "health")
    out = {
        "interval_s": float(_number(health.get("interval_s", 1.0), "health.interval_s", positive=True)),
        "stall_timeout_s": float(_number(health.get("stall_timeout_s", 5.0),
                                         "health.stall_timeout_s", positive=True)),
    }
    stale = health.get("stale_after_s", max(5.0, 3 * out["interval_s"]))
    out["stale_after_s"] = float(_number(stale, "health.stale_after_s", positive=True))
    if out["stale_after_s"] < 2 * out["interval_s"]:
        raise ConfigError("health.stale_after_s must cover at least two health.interval_s periods")
    if health.get("expected_odometry_hz") is not None:
        out["expected_odometry_hz"] = float(_number(health["expected_odometry_hz"],
                                                    "health.expected_odometry_hz", positive=True))
    limits = _mapping(health.get("limits"), "health.limits")
    _unknown(limits, [c for c, metrics in HEALTH_COMPONENTS.items() if metrics], "health.limits")
    out_limits = {}
    for component, metrics in limits.items():
        metrics = _mapping(metrics, f"health.limits.{component}")
        _unknown(metrics, HEALTH_COMPONENTS[component], f"health.limits.{component}")
        for metric, bounds in metrics.items():
            where = f"health.limits.{component}.{metric}"
            bounds = _mapping(bounds, where)
            _unknown(bounds, LIMIT_BOUNDS, where)
            out_limits.setdefault(component, {})[metric] = {
                key: float(_number(value, f"{where}.{key}")) for key, value in bounds.items()}
    out["limits"] = out_limits
    return out


def derive_zenoh(zenoh):
    """Validate the OPTIONAL `zenoh:` block -> (override_pairs, warnings).

    Fleet-uniform knob — identical in shape and flattening to the ouster wrapper and the rig-infra
    ros2-bag-logger. Renders one ZENOH_CONFIG_OVERRIDE "path=json;..." pair string ("" when the
    block sets nothing); the entrypoint applies it ONLY under rmw_zenoh_cpp. SHM matters here: GLIM
    subscribes to full-rate point clouds, which otherwise cross TCP loopback.
    """
    zenoh = _mapping(zenoh, "zenoh")
    _unknown(zenoh, ("shared_memory", "shm_pool_mb", "overrides"), "zenoh")
    enable = _bool(zenoh.get("shared_memory", False), "zenoh.shared_memory")
    pool_mb = zenoh.get("shm_pool_mb")
    if pool_mb is not None and (isinstance(pool_mb, bool) or not isinstance(pool_mb, int)
                                or pool_mb <= 0):
        raise ConfigError("zenoh.shm_pool_mb must be a positive integer (MB)")
    warns = []
    if pool_mb is not None and not enable:
        warns.append("zenoh.shm_pool_mb is ignored without zenoh.shared_memory: true")
        pool_mb = None
    pairs = []
    if enable:
        pairs.append(("transport/shared_memory/enabled", "true"))
        if pool_mb:
            pairs.append(("transport/shared_memory/transport_optimization/pool_size",
                          str(pool_mb * 1024 * 1024)))
    overrides = zenoh.get("overrides")
    if overrides is not None and not isinstance(overrides, dict):
        raise ConfigError("zenoh.overrides must be a mapping (nested zenoh session config)")
    if overrides:
        def leaves(prefix, val):
            if isinstance(val, dict) and val:
                for key in sorted(val):
                    leaves(f"{prefix}/{key}" if prefix else str(key), val[key])
            else:
                pairs.append((prefix, json.dumps(val, sort_keys=True, separators=(",", ":"),
                                                 allow_nan=False)))
        leaves("", overrides)
    for path, value in pairs:
        if ";" in path or ";" in value:
            raise ConfigError(f"zenoh override at {path!r} contains ';' — rmw_zenoh's "
                              "ZENOH_CONFIG_OVERRIDE parser splits on ';'")
    return ";".join(f"{p}={v}" for p, v in pairs), warns


def parse_sim_time(raw):
    """RIG_SIM_TIME: rig's one clock token, set by `rig replay` ('1') and popped otherwise."""
    if raw in (None, ""):
        return False
    if raw in ("1", "true", "True"):
        return True
    if raw in ("0", "false", "False"):
        return False
    raise ConfigError(f"RIG_SIM_TIME={raw!r} must be 1 or 0")


def load(path):
    with open(path) as handle:
        cfg = yaml.safe_load(handle) or {}
    if not isinstance(cfg, dict):
        raise ConfigError("the config must be a YAML mapping")
    return cfg


def _default(cfg, key, default):
    """`is None`, not `or`: YAML reads `name: no` as False, which must fail, not become a default."""
    value = cfg.get(key)
    return default if value is None else value


def identity(cfg, strict=True):
    if strict:
        _unknown(cfg, TOP_KEYS, "top-level")
    service = _ident("service", _default(cfg, "service", "glim"))
    if service != "glim":
        raise ConfigError(f"service {service!r}: this launcher only runs `glim` configs")
    name = _ident("name", _default(cfg, "name", "glim"))
    ros = _mapping(cfg.get("ros"), "ros")
    if strict:
        _unknown(ros, ("namespace", "use_sim_time"), "ros")
    namespace = _ident("ros.namespace", _default(ros, "namespace", name))
    initial_state = _default(cfg, "initial_state", "active")
    if initial_state not in ("active", "standby"):
        raise ConfigError(f"initial_state {initial_state!r} must be 'active' or 'standby'")
    return service, name, namespace, initial_state


def render_env(cfg, warn=True, lenient=False):
    """Identity env. `lenient` is for verbs that inspect or tear down a running instance: a later
    edit that broke an unrelated block must never stop `down`, `ps`, `logs` or `state` from
    reaching it — only identity (which keys the compose project) is still enforced."""
    if lenient:
        service, name, namespace, initial_state = identity(cfg, strict=False)
        try:
            zenoh_pairs, zenoh_warns = derive_zenoh(cfg.get("zenoh"))
        except ConfigError:
            zenoh_pairs, zenoh_warns = "", []
        try:
            timeout = derive_map(cfg.get("map"))["save_timeout_s"]
        except ConfigError:
            timeout = float(SAVE_TIMEOUT_DEFAULT_S)
    else:
        service, name, namespace, initial_state = identity(cfg)
        zenoh_pairs, zenoh_warns = derive_zenoh(cfg.get("zenoh"))
        timeout = derive_map(cfg.get("map"))["save_timeout_s"]
    # warn in ONE mode only: a `glim-up` run renders both, and each warning should surface once
    for message in zenoh_warns if warn else ():
        sys.stderr.write("render_config: " + message + "\n")
    return [("SERVICE", service), ("NAME", name), ("NAMESPACE", "/" + namespace),
            ("INITIAL_STATE", initial_state),
            ("STOP_GRACE_S", str(int(math.ceil(timeout)) + STOP_MARGIN_S)),
            ("ZENOH_OVERRIDE", zenoh_pairs)]


def render_instance(cfg, sim_time_env=None):
    service, name, namespace, _ = identity(cfg)
    ros = _mapping(cfg.get("ros"), "ros")
    use_sim_time = ros.get("use_sim_time")
    if use_sim_time is None:
        use_sim_time = parse_sim_time(sim_time_env)  # explicit config wins both ways
    else:
        _bool(use_sim_time, "ros.use_sim_time")
    pipeline = _default(cfg, "pipeline", "lightweight")
    if pipeline not in PIPELINES:
        raise ConfigError(f"pipeline {pipeline!r} must be one of {', '.join(PIPELINES)}"
                          " (this image is CPU-only)")
    inputs = derive_inputs(cfg.get("inputs"))
    frames = derive_frames(cfg.get("frames"))
    overrides = derive_glim(cfg.get("glim"))

    # Wrapper-derived GLIM keys, layered OVER the user's opaque overrides (which cannot set them).
    ros_keys = dict(overrides.get("ros") or {})
    ros_keys.update({
        "points_topic": inputs["points"]["topic"], "points_qos": inputs["points"]["qos"],
        "imu_topic": inputs["imu"]["topic"], "imu_qos": inputs["imu"]["qos"],
        "map_frame_id": frames["map"], "odom_frame_id": frames["odom"],
        "base_frame_id": frames["base"], "imu_frame_id": frames["imu"],
        "lidar_frame_id": frames["lidar"], "publish_imu2lidar": frames["publish_imu_to_lidar"],
    })
    ros_keys.setdefault("extension_modules", ["librviz_viewer.so"])
    sensors = dict(overrides.get("sensors") or {})
    sensors["T_lidar_imu"] = derive_extrinsics(cfg.get("extrinsics"))
    glim = dict(overrides, ros=ros_keys, sensors=sensors)

    return {
        "version": INSTANCE_SCHEMA_VERSION,
        "service": service,
        "instance": name,
        "namespace": "/" + namespace,
        "use_sim_time": bool(use_sim_time),
        "pipeline": pipeline,
        "inputs": {stream: block["topic"] for stream, block in inputs.items()},
        "glim": {GLIM_MODULES[module]: block for module, block in sorted(glim.items())},
        "map": derive_map(cfg.get("map")),
        "health": derive_health(cfg.get("health")),
    }


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args and args[0] in ("--env0", "--instance") else None
    lenient = mode == "--env0" and args[:1] == ["--lenient"]
    if lenient:
        args = args[1:]
    sim_time = None
    if mode == "--instance" and args[:1] == ["--sim-time"]:
        if len(args) < 2:
            sys.stderr.write("render_config: --sim-time needs a value\n")
            return 2
        sim_time = args[1]
        args = args[2:]
    if mode is None or len(args) != 1:
        sys.stderr.write("usage: render_config.py --env0 [--lenient] <config.yaml>\n"
                         "       render_config.py --instance [--sim-time V] <config.yaml>\n")
        return 2
    try:
        cfg = load(args[0])
        if mode == "--env0":
            out = sys.stdout.buffer
            for key, value in render_env(cfg, lenient=lenient):
                out.write(key.encode() + b"\0" + value.encode() + b"\0")
            out.flush()
        else:
            # render_env's checks too, so `config`/`up` fail on everything the launcher reads
            render_env(cfg, warn=False)
            # allow_nan=False: GLIM's JSON parser rejects NaN/Infinity, so fail here, not there
            text = json.dumps(render_instance(cfg, sim_time), sort_keys=True, indent=2,
                              allow_nan=False)
            sys.stdout.write(text + "\n")
    except ConfigError as error:
        sys.stderr.write(f"render_config: {args[0]}: {error}\n")
        return 2
    except (TypeError, ValueError) as error:   # YAML values JSON cannot carry (dates, NaN, ...)
        sys.stderr.write(f"render_config: {args[0]}: unsupported value: {error}\n")
        return 2
    except (OSError, yaml.YAMLError) as error:
        sys.stderr.write(f"render_config: {error}\n")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
