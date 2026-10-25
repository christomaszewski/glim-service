"""tools/fetch_upstream.py against local git fixtures: pins, verification, no partial trees."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import fetch_upstream  # noqa: E402

NAMES = fetch_upstream.EXPECTED


def git(*args, cwd=None):
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True,
                          text=True).stdout.strip()


class FetchUpstreamTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.pins = {}
        for name in NAMES:
            repo = self.root / "remotes" / name
            repo.mkdir(parents=True)
            git("init", "-q", "-b", "main", cwd=repo)
            git("config", "user.email", "t@example.com", cwd=repo)
            git("config", "user.name", "t", cwd=repo)
            (repo / "CMakeLists.txt").write_text(f"# {name} pinned\n")
            git("add", ".", cwd=repo)
            git("commit", "-q", "-m", "pinned", cwd=repo)
            self.pins[name] = git("rev-parse", "HEAD", cwd=repo)
            (repo / "CMakeLists.txt").write_text(f"# {name} newer, unpinned\n")
            git("commit", "-qam", "newer", cwd=repo)   # the pin is NOT the branch head
        self.manifest = self.root / "glim.repos"
        self.write_manifest()
        self.dest = self.root / "src"

    def write_manifest(self, **changes):
        repositories = {name: {"type": "git", "url": (self.root / "remotes" / name).as_uri(),
                               "version": self.pins[name]} for name in NAMES}
        for name, spec in changes.items():
            if spec is None:
                del repositories[name]
            else:
                repositories[name].update(spec)
        self.manifest.write_text(yaml.safe_dump({"repositories": repositories}))

    def test_fetches_exact_pins_and_records_them(self):
        fetch_upstream.fetch(self.dest, self.manifest)
        for name in NAMES:
            self.assertEqual(git("rev-parse", "HEAD", cwd=self.dest / name), self.pins[name])
            self.assertIn("pinned", (self.dest / name / "CMakeLists.txt").read_text())
        sources = json.loads((self.dest / "upstream-sources.json").read_text())
        self.assertEqual(sources["version"], 1)
        self.assertEqual({k: v["rev"] for k, v in sources["sources"].items()}, self.pins)
        self.assertEqual(sorted(p.name for p in self.dest.iterdir()),
                         sorted([*NAMES, "upstream-sources.json"]))   # no staging left behind

    def test_the_real_manifest_pins_full_shas(self):
        repositories = fetch_upstream.load_manifest(ROOT / "glim.repos")
        self.assertEqual(set(repositories), set(NAMES))

    def test_rejects_moving_or_incomplete_pins(self):
        for changes, fragment in (({"glim": {"version": "master"}}, "full commit SHA"),
                                  ({"gtsam": {"version": "v1.2.2"}}, "full commit SHA"),
                                  ({"glim_ros2": None}, "must list exactly"),
                                  ({"gtsam_points": {"type": "hg"}}, "git repository")):
            with self.subTest(changes=changes):
                self.write_manifest(**changes)
                with self.assertRaisesRegex(ValueError, fragment):
                    fetch_upstream.fetch(self.dest, self.manifest)
                self.assertFalse(self.dest.exists())

    def test_malformed_manifests_are_clean_errors(self):
        for text in ("repositories: [gtsam]\n", "- a\n",
                     "repositories: {gtsam: x, gtsam_points: x, glim: x, glim_ros2: x}\n"):
            with self.subTest(text=text):
                self.manifest.write_text(text)
                with self.assertRaises(ValueError):
                    fetch_upstream.fetch(self.dest, self.manifest)

    def test_unknown_commit_leaves_no_partial_tree(self):
        fetch_upstream.FETCH_ATTEMPTS, saved = 1, fetch_upstream.FETCH_ATTEMPTS
        self.addCleanup(setattr, fetch_upstream, "FETCH_ATTEMPTS", saved)
        self.write_manifest(glim_ros2={"version": "0" * 40})
        with self.assertRaises(subprocess.CalledProcessError):
            fetch_upstream.fetch(self.dest, self.manifest)
        self.assertEqual(list(self.dest.iterdir()), [])

    def test_refuses_to_replace_a_checkout(self):
        (self.dest / "glim").mkdir(parents=True)
        (self.dest / "glim" / "local-edit.cpp").write_text("work in progress")
        with self.assertRaisesRegex(ValueError, "refusing to replace"):
            fetch_upstream.fetch(self.dest, self.manifest)
        self.assertTrue((self.dest / "glim" / "local-edit.cpp").exists())

    def test_command_line_failure_is_one_line_on_stderr(self):
        self.write_manifest(glim={"version": "master"})
        result = subprocess.run([sys.executable, str(ROOT / "tools/fetch_upstream.py"), str(self.dest),
                                 "--manifest", str(self.manifest)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, "")
        self.assertIn("fetch_upstream: ", result.stderr)


if __name__ == "__main__":
    unittest.main()
