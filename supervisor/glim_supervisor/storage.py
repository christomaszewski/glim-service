"""Where map sessions live, and keeping them owned by whoever owns the data dir.

Layout under the map root (GLIM_MAP_ROOT: the vehicle data dir, or the fallback volume):
    <root>/current/maps/<instance>/<session>/   when rig has a run open (`current` symlink)
    <root>/maps/<instance>/<session>/           otherwise (flat; standalone or no run registry)
A session pins the run when it STARTS (rig's writer rule): a later `current` flip never splits one
session across two runs.
"""
import json
import logging
import os
from pathlib import Path
import shutil

LOG = logging.getLogger("glim-supervisor")


def instance_root(map_root, instance):
    map_root = Path(map_root)
    current = map_root / "current"
    base = map_root
    if current.is_symlink() or current.exists():
        try:
            resolved = current.resolve(strict=True)
            # rig writes a RELATIVE runs/<id> target; anything resolving outside the mount is a
            # host path this container cannot reach — fall back rather than write somewhere odd.
            if resolved.is_dir() and resolved.is_relative_to(map_root.resolve()):
                base = resolved
            else:
                LOG.warning("ignoring %s -> %s (not a run inside the data dir); using the flat layout",
                            current, resolved)
        except OSError as error:
            LOG.warning("ignoring unresolvable %s (%s); using the flat layout", current, error)
    return base / "maps" / instance


def new_session_dir(root, stamp):
    """A fresh, unique session directory named by its UTC start stamp."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for suffix in [""] + [f"_{i}" for i in range(2, 100)]:
        candidate = root / f"{stamp}{suffix}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError(f"cannot allocate a session directory under {root}")


def adopt_ownership(path, reference, recursive=True):
    """chown `path` (recursively by default) to the owner of `reference` (best effort).

    GLIM runs as root in the container; files it writes into a user's data dir should belong to
    that user, so sync/cleanup tooling run without sudo can manage them.
    """
    try:
        info = os.stat(reference)
    except OSError:
        return
    if info.st_uid == os.geteuid() and info.st_gid == os.getegid():
        return
    if not recursive:
        try:
            os.lchown(path, info.st_uid, info.st_gid)
        except OSError as error:
            LOG.warning("cannot chown %s: %s", path, error)
        return
    for directory, _, files in os.walk(path):
        for entry in [directory, *(os.path.join(directory, f) for f in files)]:
            try:
                os.lchown(entry, info.st_uid, info.st_gid)
            except OSError as error:
                LOG.warning("cannot chown %s: %s", entry, error)
                return


def write_json(path, document):
    """Atomic JSON write (a reader never sees a half-written session record)."""
    path = Path(path)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def disk_free(path):
    """(free GB, free percent) of the filesystem holding `path`."""
    usage = shutil.disk_usage(path)
    return usage.free / 1e9, 100.0 * usage.free / usage.total if usage.total else 0.0
