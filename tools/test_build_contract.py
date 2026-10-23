#!/usr/bin/env python3
"""Check the real build entrypoint's Docker arguments and digest reporting without a daemon."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
IMAGE = "glim"
PLATFORM_ENV = "GLIM_BUILD_PLATFORM"
DIGEST = "sha256:" + "a" * 64


class BuildContractTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.repo = self.root / "source with spaces"
        (self.repo / "tools").mkdir(parents=True)
        for name in ("build_image.sh", "rig-build.sh"):
            shutil.copy2(ROOT / "tools" / name, self.repo / "tools" / name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        docker = self.bin / "docker"
        docker.write_text(f"#!{sys.executable}\n" + """
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["DOCKER_CAPTURE"], "a") as output:
    output.write(json.dumps(args) + "\\n")
if os.environ.get("DOCKER_FAIL"):
    raise SystemExit(19)
if "--metadata-file" in args:
    data = {} if os.environ.get("DOCKER_NO_DIGEST") else {"containerimage.digest": os.environ["TEST_DIGEST"]}
    Path(args[args.index("--metadata-file") + 1]).write_text(json.dumps(data))
""")
        docker.chmod(0o755)
        self.capture = self.root / "docker.jsonl"
        self.report = self.root / "result.jsonl"
        self.ref = f"reg.example:5000/{IMAGE}:test"
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith(("RIG_", "GLIM_", "GIT_", "DOCKER_"))
                    and k not in ("PUSH", "ROS_DISTRO")}
        self.env.update(PATH=f"{self.bin}:{os.environ['PATH']}",
                        DOCKER_CAPTURE=str(self.capture), TEST_DIGEST=DIGEST)

    def run_build(self, **env):
        self.capture.unlink(missing_ok=True)
        result = subprocess.run(
            ["bash", str(self.repo / "tools/build_image.sh"), "reg.example:5000", "test"],
            cwd=self.root, env=dict(self.env, **env), capture_output=True, text=True)
        calls = ([json.loads(line) for line in self.capture.read_text().splitlines()]
                 if self.capture.exists() else [])
        return result, calls

    def successful_build(self, **env):
        result, calls = self.run_build(**env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(calls[0][-1], str(self.repo))
        self.assertEqual(calls[0][calls[0].index("-f") + 1],
                         str(self.repo / "docker/Dockerfile.runtime"))
        return calls

    def test_standalone_defaults_build_and_push_once(self):
        build, push = self.successful_build()
        self.assertEqual(build[:3], ["build", "-t", self.ref])
        self.assertEqual(push, ["push", self.ref])
        self.assertIn("ROS_DISTRO=lyrical", build)
        self.assertIn("BASE_IMAGE=ros:lyrical-ros-core", build)
        for flag in ("--no-cache", "--pull", "--platform"):
            self.assertNotIn(flag, build)
        self.assertNotIn("GLIM_DISABLE_CCACHE=1", build)

    def test_standalone_platform_override_is_preserved(self):
        build, _ = self.successful_build(**{PLATFORM_ENV: "linux/amd64"})
        self.assertEqual(build[build.index("--platform") + 1], "linux/amd64")

    def test_clean_standalone_platform_build_disables_compiler_cache(self):
        build, _ = self.successful_build(
            RIG_BUILD_NO_CACHE="1", **{PLATFORM_ENV: "linux/amd64"})
        self.assertEqual(build[build.index("--platform") + 1], "linux/amd64")
        self.assertIn("--no-cache", build)
        self.assertIn("GLIM_DISABLE_CCACHE=1", build)

    def test_no_cache_and_fleet_parent_reach_both_build_paths(self):
        for platforms in ("", "linux/arm64,linux/amd64"):
            with self.subTest(platforms=platforms):
                build = self.successful_build(
                    RIG_BUILD_PLATFORMS=platforms, RIG_BUILD_NO_CACHE="1",
                    ROS_DISTRO="fleet-distro", RIG_BASE_IMAGE="registry.test/fleet@sha256:123")[0]
                self.assertIn("--no-cache", build)
                self.assertIn("--pull", build)
                self.assertIn("GLIM_DISABLE_CCACHE=1", build)
                self.assertIn("ROS_DISTRO=fleet-distro", build)
                self.assertIn("BASE_IMAGE=registry.test/fleet@sha256:123", build)

    def test_multiarch_publishes_one_index_and_appends_digest(self):
        previous = {"ref": "registry.test/other:v1", "digest": DIGEST}
        self.report.write_text(json.dumps(previous) + "\n")
        build, = self.successful_build(
            RIG_BUILD_PLATFORMS="linux/arm64,linux/amd64",
            RIG_BUILD_RESULT=str(self.report), **{PLATFORM_ENV: "linux/amd64"})
        self.assertEqual(build[:2], ["buildx", "build"])
        self.assertEqual(build.count("--platform"), 1)
        self.assertEqual(build[build.index("--platform") + 1], "linux/arm64,linux/amd64")
        self.assertEqual(build[build.index("-t") + 1], self.ref)
        self.assertIn("--push", build)
        self.assertIn("--provenance=mode=min", build)
        self.assertNotIn("--no-cache", build)
        self.assertEqual([json.loads(line) for line in self.report.read_text().splitlines()],
                         [previous, {"ref": self.ref, "digest": DIGEST}])

    def test_single_arch_can_load_locally(self):
        build, = self.successful_build(RIG_BUILD_PLATFORMS="linux/arm64", PUSH="0")
        self.assertIn("--load", build)
        self.assertNotIn("--push", build)

    def test_multiarch_cannot_silently_load_only_one_architecture(self):
        result, calls = self.run_build(RIG_BUILD_PLATFORMS="linux/arm64,linux/amd64", PUSH="0")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("multiarch builds require PUSH=1", result.stderr)
        self.assertEqual(calls, [])

    def test_failure_does_not_report_success_or_push_a_stale_image(self):
        for platforms in ("", "linux/arm64,linux/amd64"):
            with self.subTest(platforms=platforms):
                result, calls = self.run_build(
                    RIG_BUILD_PLATFORMS=platforms, RIG_BUILD_RESULT=str(self.report), DOCKER_FAIL="1")
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(len(calls), 1)
                self.assertFalse(self.report.exists())

    def test_missing_digest_fails_instead_of_reporting_an_unpinned_image(self):
        result, _ = self.run_build(
            RIG_BUILD_PLATFORMS="linux/arm64,linux/amd64",
            RIG_BUILD_RESULT=str(self.report), DOCKER_NO_DIGEST="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("no published image digest", result.stderr)
        self.assertFalse(self.report.exists())

    def test_pull_can_be_disabled_without_disabling_no_cache(self):
        build, _ = self.successful_build(RIG_BUILD_NO_CACHE="1", RIG_BUILD_PULL="0")
        self.assertIn("--no-cache", build)
        self.assertNotIn("--pull", build)
        self.assertIn("GLIM_DISABLE_CCACHE=1", build)

    def test_job_cap_and_wrapper_identity_reach_the_build(self):
        build, _ = self.successful_build(GLIM_BUILD_JOBS="3")
        self.assertIn("BUILD_JOBS=3", build)
        # The temporary repo is not a git checkout: identity is explicitly unknown, never borrowed.
        self.assertIn("GLIM_SERVICE_REF=unknown", build)
        self.assertIn("GLIM_SERVICE_REV=unknown", build)


if __name__ == "__main__":
    unittest.main()
