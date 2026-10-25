#!/usr/bin/env python3
# Copyright 2026 Chris Tomaszewski
# Licensed under the Apache License, Version 2.0.
"""`glim-up <config> export` — slim this instance's map sessions for `rig run export`.

rig's export contract (rigging.yaml `export: {data: maps/{name}}`): rig hands over
  RIG_EXPORT_SOURCE   the run directory
  RIG_EXPORT_DEST     where the export tree is (the run itself with RIG_EXPORT_INPLACE=1)
  RIG_EXPORT_OPTIONS  a YAML file: this instance's block of the deployment's export profile
  RIG_EXPORT_NAME     the instance name the data was recorded under (default: the config's)
  RIG_EXPORT_LOSSY    set when the operator allows dropping data in place
and lays out everything else itself. Options:
  dump: trajectories  GLIM's dump: `full` (everything offline_viewer needs), `trajectories`
                      (the default: odom_*/traj_* and graph.txt — small), or `none`
  logs: true          GLIM's per-module logs
session.json, config/, map.ply, reference.tum and evaluation.json always travel. Files are
hardlinked when the filesystem allows (no disk cost on the vehicle), else copied.
"""
import os
from pathlib import Path
import shutil
import sys

import yaml

ALWAYS = ("session.json", "evaluation.json", "reference.tum", "map.ply")
DUMP_TEXT = ("odom_imu.txt", "odom_lidar.txt", "traj_imu.txt", "traj_lidar.txt", "graph.txt")


def link_or_copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def kept_files(session, options):
    for name in ALWAYS:
        if (session / name).is_file():
            yield session / name
    for tree in ("config",) + (("logs",) if options["logs"] else ()):
        if (session / tree).is_dir():
            yield from (p for p in sorted((session / tree).rglob("*")) if p.is_file())
    dump = session / "dump"
    if options["dump"] == "full" and dump.is_dir():
        yield from (p for p in sorted(dump.rglob("*")) if p.is_file())
    elif options["dump"] == "trajectories":
        yield from (dump / name for name in DUMP_TEXT if (dump / name).is_file())


def load_options(path):
    raw = yaml.safe_load(Path(path).read_text()) if path and Path(path).is_file() else None
    raw = raw or {}
    if not isinstance(raw, dict) or set(raw) - {"dump", "logs"}:
        raise ValueError(f"export options support dump, logs; got {raw!r}")
    options = {"dump": raw.get("dump", "trajectories"), "logs": raw.get("logs", True)}
    if options["dump"] not in ("full", "trajectories", "none") or not isinstance(options["logs"], bool):
        raise ValueError("export options: dump is full|trajectories|none, logs is true|false")
    return options


def export(source, dest, name, options, in_place=False, lossy=False):
    """-> (sessions, files) exported or rewritten."""
    root = Path(source) / "maps" / name
    if not root.is_dir():
        return 0, 0
    sessions = sorted(d for d in root.iterdir() if d.is_dir())
    if in_place:
        if options["dump"] == "full" and options["logs"]:
            return len(sessions), 0          # lossless and nothing to slim: no change
        if not lossy:
            raise PermissionError("slimming maps in place drops data (the map dump"
                                  + ("" if options["logs"] else ", logs") + "): pass --lossy")
        dropped = 0
        for session in sessions:
            keep = set(kept_files(session, options))
            for path in sorted((session / "dump").rglob("*"), reverse=True) if (session / "dump").is_dir() else []:
                if path.is_file() and path not in keep:
                    path.unlink()
                    dropped += 1
                elif path.is_dir() and not any(path.iterdir()):
                    path.rmdir()
            if not options["logs"] and (session / "logs").is_dir():
                dropped += sum(1 for p in (session / "logs").rglob("*") if p.is_file())
                shutil.rmtree(session / "logs")
        return len(sessions), dropped
    count = 0
    for session in sessions:
        target = Path(dest) / "maps" / name / session.name
        for path in kept_files(session, options):
            link_or_copy(path, target / path.relative_to(session))
            count += 1
    return len(sessions), count


def main():
    source, dest = os.environ.get("RIG_EXPORT_SOURCE"), os.environ.get("RIG_EXPORT_DEST")
    name = os.environ.get("RIG_EXPORT_NAME") or os.environ.get("GLIM_NAME")
    if not source or not dest or not name:
        sys.stderr.write("export_maps: run by `rig run export` (RIG_EXPORT_SOURCE/DEST and an "
                         "instance name are required)\n")
        return 2
    try:
        options = load_options(os.environ.get("RIG_EXPORT_OPTIONS"))
        in_place = os.environ.get("RIG_EXPORT_INPLACE") == "1"
        sessions, files = export(source, dest, name, options, in_place=in_place,
                                 lossy=os.environ.get("RIG_EXPORT_LOSSY") == "1")
    except PermissionError as error:
        sys.stderr.write(f"export_maps: {error}\n")
        return 3
    except (OSError, ValueError, yaml.YAMLError) as error:
        sys.stderr.write(f"export_maps: {error}\n")
        return 2
    verb = "slimmed in place" if in_place else "exported"
    sys.stderr.write(f"export_maps: {name}: {sessions} session(s) {verb} "
                     f"(dump: {options['dump']}, logs: {options['logs']}; {files} file(s))\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
