"""The container's own memory budget, from cgroup v2 (what the kernel's OOM killer enforces).

GLIM's memory grows with the map. When the container has a memory limit, the supervisor ends
the session cleanly (saving the map) before the kernel would SIGKILL GLIM and lose it. Only
anonymous memory counts: page cache is reclaimable, and shared-memory transport pools (zenoh SHM
in /dev/shm) belong to the transport, not to the map.
"""
from pathlib import Path

CGROUP = Path("/sys/fs/cgroup")


def memory(root=CGROUP):
    """-> {"used_bytes", "limit_bytes"} (limit None = unlimited), or None if unreadable."""
    try:
        limit_text = (root / "memory.max").read_text().strip()
        stat = (root / "memory.stat").read_text()
    except OSError:
        return None
    used = None
    for line in stat.splitlines():
        key, _, value = line.partition(" ")
        if key == "anon":
            used = int(value)
            break
    if used is None:
        return None
    limit = None if limit_text == "max" else int(limit_text)
    return {"used_bytes": used, "limit_bytes": limit}
