"""Compose a session's GLIM config directory: pinned upstream defaults + instance overrides.

GLIM reads <config_path>/config.json, whose `global` block names one JSON file per module. The
defaults are upstream's own files shipped in the image (commented JSON, which GLIM's nlohmann
parser accepts and Python's does not). Each override REPLACES one top-level key of its module's
section — GLIM's parameters are flat, apart from the *_qos mappings the wrapper derives whole.
Keys absent from upstream's defaults are still applied, but reported: GLIM ignores unknown keys
silently, so a typo would otherwise pass as configured.
"""
import json
from pathlib import Path

# Upstream file per GLIM `global` key, by pipeline preset (all CPU; this image has no CUDA).
_COMMON = {
    "config_ros": "config_ros.json",
    "config_logging": "config_logging.json",
    "config_viewer": "config_viewer.json",
    "config_sensors": "config_sensors.json",
    "config_preprocess": "config_preprocess.json",
}
PIPELINES = {
    "lightweight": {"config_odometry": "config_odometry_cpu.json",
                    "config_sub_mapping": "config_sub_mapping_passthrough.json",
                    "config_global_mapping": "config_global_mapping_pose_graph.json",
                    "local_mapping": True, "global_mapping": True},
    "cpu": {"config_odometry": "config_odometry_cpu.json",
            "config_sub_mapping": "config_sub_mapping_cpu.json",
            "config_global_mapping": "config_global_mapping_cpu.json",
            "local_mapping": True, "global_mapping": True},
    "odometry": {"config_odometry": "config_odometry_cpu.json",
                 "config_sub_mapping": "config_sub_mapping_passthrough.json",
                 "config_global_mapping": "config_global_mapping_pose_graph.json",
                 "local_mapping": False, "global_mapping": False},
}
# GLIM `global` key -> the JSON section inside that file.
SECTIONS = {
    "config_ros": "glim_ros", "config_logging": "logging", "config_viewer": None,
    "config_sensors": "sensors", "config_preprocess": "preprocess",
    "config_odometry": "odometry_estimation", "config_sub_mapping": "sub_mapping",
    "config_global_mapping": "global_mapping",
}
# Keys this wrapper adds that upstream's defaults do not carry (its own patches/extensions); never
# reported as unknown.
WRAPPER_KEYS = {"glim_ros": {"publish_tf"}}
MAP_EXPORT_MODULE = "libmap_export_monitor.so"   # ext/glim_map_export

# Section -> the instance YAML's `glim:` module name, for operator-facing messages.
MODULE_NAMES = {"glim_ros": "ros", "sensors": "sensors", "preprocess": "preprocess",
                "odometry_estimation": "odometry", "sub_mapping": "sub_mapping",
                "global_mapping": "global_mapping"}


def strip_comments(text):
    """Remove // and /* */ comments outside JSON strings (nlohmann's ignore_comments dialect)."""
    out, i, n, in_string = [], 0, len(text), False
    while i < n:
        char = text[i]
        if in_string:
            out.append(char)
            if char == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            if char == '"':
                in_string = False
            i += 1
        elif char == '"':
            in_string = True
            out.append(char)
            i += 1
        elif text.startswith("//", i):
            end = text.find("\n", i)
            i = n if end < 0 else end
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            if end < 0:
                raise ValueError("unterminated /* comment")
            out.append(" ")
            i = end + 2
        else:
            out.append(char)
            i += 1
    return "".join(out)


def load_jsonc(path):
    return json.loads(strip_comments(Path(path).read_text()))


def compose(defaults_dir, instance, session_dir):
    """Write <session_dir>/config/ for `instance` (the rendered instance file's dict).

    Returns (config_dir, warnings). Raises ValueError for an unusable pipeline or defaults dir.
    """
    defaults_dir, session_dir = Path(defaults_dir), Path(session_dir)
    pipeline = PIPELINES.get(instance.get("pipeline"))
    if pipeline is None:
        raise ValueError(f"unknown pipeline {instance.get('pipeline')!r}")
    overrides = instance.get("glim") or {}
    unknown_sections = sorted(set(overrides) - set(MODULE_NAMES))
    if unknown_sections:
        raise ValueError(f"instance overrides name unknown GLIM sections: {unknown_sections}")

    config_dir = session_dir / "config"
    config_dir.mkdir(parents=True, exist_ok=True)
    files = {**_COMMON, **{k: v for k, v in pipeline.items() if k.startswith("config_")}}
    global_block = {"config_path": ""}
    warnings = []
    for key, filename in files.items():
        document = load_jsonc(defaults_dir / filename)
        section = SECTIONS[key]
        if section is not None:
            if section not in document or not isinstance(document[section], dict):
                raise ValueError(f"{defaults_dir / filename} has no `{section}` section")
            merged = dict(document[section])
            for name, value in (overrides.get(section) or {}).items():
                if name not in merged and name not in WRAPPER_KEYS.get(section, ()):
                    warnings.append(f"glim.{MODULE_NAMES[section]}.{name} is not in upstream's "
                                    f"{filename} — applied, but GLIM may ignore it")
                merged[name] = value
            if section == "glim_ros":
                merged["enable_local_mapping"] = pipeline["local_mapping"]
                merged["enable_global_mapping"] = pipeline["global_mapping"]
                export = (instance.get("map") or {}).get("export_ply", False)
                modules = list(merged.get("extension_modules") or [])
                if export and pipeline["global_mapping"] and MAP_EXPORT_MODULE not in modules:
                    modules.append(MAP_EXPORT_MODULE)
                merged["extension_modules"] = modules
                # the exporter's own section (ext/glim_map_export reads it from this file)
                document["glim_service"] = {
                    "export_ply": bool(export),
                    "export_voxel_m": float((instance.get("map") or {}).get("export_voxel_m", 0.1))}
            elif section == "logging":
                # GLIM's per-module log files travel with the session they describe.
                merged.update(log_dir=str(session_dir / "logs"), save_logs=True)
            document[section] = merged
        (config_dir / filename).write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
        global_block[key] = filename
    (config_dir / "config.json").write_text(
        json.dumps({"global": global_block}, indent=2, sort_keys=True) + "\n")
    return config_dir, warnings
