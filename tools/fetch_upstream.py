#!/usr/bin/env python3
"""Fetch the pinned GLIM stack (glim.repos) into a fresh source directory.

Used by the image build and for local development (`python3 tools/fetch_upstream.py src`). Every
pin must be a full commit SHA; each is shallow-fetched by SHA with plain git (no vcstool needed),
verified, and recorded with its URL in <dest>/upstream-sources.json (shipped in the image).
`vcs import src < glim.repos` remains an equivalent full-clone alternative for development.
"""

import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile
import time

import yaml

EXPECTED = ("gtsam", "gtsam_points", "glim", "glim_ros2")
SHA_RE = re.compile(r"[0-9a-f]{40}")
FETCH_ATTEMPTS = 3   # a cold image build makes four network fetches; ride out a transient 5xx


def run(*args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)


def load_manifest(path):
    document = yaml.safe_load(Path(path).read_text()) or {}
    repositories = document.get("repositories") if isinstance(document, dict) else None
    if not isinstance(repositories, dict) or set(repositories) != set(EXPECTED):
        raise ValueError(f"{path} must list exactly {', '.join(EXPECTED)} under `repositories`")
    for key, spec in repositories.items():
        if not isinstance(spec, dict):
            raise ValueError(f"{path}: {key} must be a mapping with type, url, version")
        if spec.get("type") != "git" or not spec.get("url"):
            raise ValueError(f"{path}: {key} must be a git repository with a url")
        if not SHA_RE.fullmatch(str(spec.get("version"))):
            raise ValueError(f"{path}: {key} must be pinned to a full commit SHA, not "
                             f"{spec.get('version')!r}")
    return repositories


def fetch_one(checkout, url, rev):
    """Shallow-fetch exactly one commit (GitHub serves any reachable SHA)."""
    checkout.mkdir(parents=True)
    run("git", "-C", str(checkout), "init", "--quiet")
    run("git", "-C", str(checkout), "remote", "add", "origin", url)
    for attempt in range(1, FETCH_ATTEMPTS + 1):
        try:
            run("git", "-C", str(checkout), "fetch", "--quiet", "--depth", "1", "origin", rev)
            break
        except subprocess.CalledProcessError:
            if attempt == FETCH_ATTEMPTS:
                raise
            time.sleep(2 ** attempt)
    run("git", "-C", str(checkout), "checkout", "--quiet", "--detach", "FETCH_HEAD")
    run("git", "-C", str(checkout), "submodule", "update", "--quiet", "--init", "--recursive",
        "--depth", "1")
    revision = run("git", "-C", str(checkout), "rev-parse", "HEAD", capture_output=True).stdout.strip()
    if revision != rev:
        raise ValueError(f"{checkout.name}: fetched {revision}, expected {rev}")
    return revision


def fetch(source_root, manifest):
    source_root = Path(source_root).resolve()
    repositories = load_manifest(manifest)
    for key in (*EXPECTED, "upstream-sources.json"):
        if (source_root / key).exists() or (source_root / key).is_symlink():
            raise ValueError(f"refusing to replace {source_root / key}; use a fresh source directory")
    source_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".glim-fetch-", dir=source_root) as tmp:
        staging = Path(tmp)
        sources = {}
        for key in EXPECTED:
            spec = repositories[key]
            sources[key] = {"repo": spec["url"],
                            "rev": fetch_one(staging / key, spec["url"], spec["version"])}
        (staging / "upstream-sources.json").write_text(
            json.dumps({"version": 1, "sources": sources}, indent=2, sort_keys=True) + "\n")
        # Publish only complete fetches: a failed pin leaves no partial tree behind.
        for key in (*EXPECTED, "upstream-sources.json"):
            (staging / key).rename(source_root / key)
    return source_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_root", nargs="?", default="src")
    parser.add_argument("--manifest", type=Path,
                        default=Path(__file__).resolve().parents[1] / "glim.repos")
    args = parser.parse_args()
    try:
        print(fetch(args.source_root, args.manifest))
    except (ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"fetch_upstream: {error}\n")


if __name__ == "__main__":
    main()
