"""glim-up against a fake `docker`: the rig launcher contract without a daemon."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

FAKE_DOCKER = r'''#!/usr/bin/env bash
target="$(cat "${GLIM_TARGET_HOST:-/nonexistent}" 2>/dev/null || echo unset)"
echo "target=${target} env_target=${GLIM_TARGET_STATE:-unset} project=${COMPOSE_PROJECT_NAME:-unset} data=${GLIM_DATA_HOST:-unset} grace=${GLIM_STOP_GRACE:-unset} docker $*" >> "$ARGV_LOG"
case "$*" in
  "compose version") exit 0 ;;
  "inspect --format"*) echo "${STOP_EXIT_CODE:-0}" ;;
  "logs --tail 40"*) echo 'shutdown log evidence' ;;
  *" config --format json")
    echo '{"name": "x", "services": {"glim": {"image": "glim:latest"}}}' ;;
  *" ps -q glim"|*" ps -aq glim") [ "${RUNNING:-}" = 1 ] && echo container; exit 0 ;;
  *" exec -T glim glim-statectl state")
    [ "${UNREACHABLE:-}" = 1 ] && exit 1
    read -r state target < "$FAKE_STATE"
    printf '{"state": "%s", "target": "%s", "detail": "fake"}\n' "$state" "$target" ;;
  *" exec -T glim glim-statectl activate") echo "active active" > "$FAKE_STATE" ;;
  *" exec -T glim glim-statectl standby") echo "standby standby" > "$FAKE_STATE" ;;
esac
exit 0
'''


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for filename in ("glim-up", "tools/render_config.py", "tools/runtime_images.py",
                         "autonomy/glim.example.yaml", "docker/compose/compose.deploy.yaml",
                         "docker/compose/compose.datadir.yaml"):
            target = self.root / filename
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / filename, target)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        docker = self.bin / "docker"
        docker.write_text(FAKE_DOCKER)
        docker.chmod(0o755)
        self.config = self.root / "autonomy/slam.yaml"
        shutil.copy2(self.root / "autonomy/glim.example.yaml", self.config)
        self.log = self.root / "argv"
        self.log.touch()
        self.state = self.root / "state"
        self.state.write_text("active active\n")
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith(("GLIM_", "RIG_", "COMPOSE_"))}
        self.env.update(PATH=f"{self.bin}:{self.env['PATH']}", ARGV_LOG=str(self.log),
                        FAKE_STATE=str(self.state), GLIM_SETTLE_TIMEOUT="4")

    def run_launcher(self, *args, **env):
        return subprocess.run([str(self.root / "glim-up"), str(self.config), *args],
                              env=dict(self.env, **env), capture_output=True, text=True, timeout=30)

    def calls(self):
        return self.log.read_text().splitlines()

    def edit_config(self, old, new):
        text = self.config.read_text()
        self.assertIn(old, text)
        self.config.write_text(text.replace(old, new))

    def test_state_of_a_down_project_is_json_and_writes_nothing(self):
        result = self.run_launcher("state")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), {"state": "down", "detail": "no containers"})
        self.assertFalse((self.root / "var").exists())
        self.assertNotIn("volume create", self.log.read_text())

    def test_state_of_a_running_project_comes_from_the_supervisor(self):
        result = self.run_launcher("state", RUNNING="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["state"], "active")

    def test_transitions_never_create_containers(self):
        for verb in ("standby", "activate"):
            result = self.run_launcher(verb)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, "")
            self.assertIn("run 'up' first", result.stderr)
        self.assertNotIn(" up", self.log.read_text())
        self.assertNotIn("volume create", self.log.read_text())

    def test_detached_up_renders_hashed_instance_and_converges(self):
        result = self.run_launcher("up", "-d", RUNNING="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        rendered = list((self.root / "var/run").glob("slam.*.instance.json"))
        self.assertEqual(len(rendered), 1)
        instance = json.loads(rendered[0].read_text())
        self.assertEqual(instance["glim"]["glim_ros"]["points_topic"], "/top/points")
        log = self.log.read_text()
        self.assertIn("docker volume create glim_slam_maps", log)
        self.assertIn("target=active env_target=unset project=glim_slam data=unset grace=270s "
                      "docker compose", log)
        self.assertIn(" up -d", log)
        self.assertNotIn("glim-statectl activate", log)   # already at the target
        self.assertNotIn(" -p ", log)

    def test_config_change_changes_the_mount_source(self):
        self.run_launcher("config")
        self.edit_config("random_downsample_target: 10000", "random_downsample_target: 5000")
        self.run_launcher("config")
        self.assertEqual(len(list((self.root / "var/run").glob("slam.*.instance.json"))), 2)

    def test_up_converges_a_container_parked_by_an_earlier_run(self):
        self.state.write_text("standby standby\n")
        result = self.run_launcher("up", "-d", RUNNING="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("glim-statectl activate", self.log.read_text())

    def test_up_reports_a_target_it_cannot_reach(self):
        self.state.write_text("transitioning active\n")
        result = self.run_launcher("up", "-d", RUNNING="1")
        self.assertEqual(result.returncode, 1)
        self.assertIn("did not reach active", result.stderr)

    def test_target_state_precedence(self):
        self.edit_config("# initial_state: standby", "initial_state: standby")
        self.state.write_text("standby standby\n")
        self.run_launcher("up", "-d", RUNNING="1")
        self.assertIn("target=standby", self.calls()[-2])
        self.state.write_text("active active\n")
        self.run_launcher("up", "-d", RUNNING="1", RIG_TARGET_STATE="active")
        self.assertIn("target=active", self.log.read_text().splitlines()[-2])

    def test_target_change_never_changes_the_compose_model(self):
        # Compose recreates on any model change; a recreate would end a running mapping session.
        def up_line():
            return [line for line in self.calls() if line.endswith(" up -d")][-1]
        self.run_launcher("up", "-d", RUNNING="1", RIG_TARGET_STATE="standby")
        first = up_line()
        self.state.write_text("active active\n")
        self.run_launcher("up", "-d", RUNNING="1")
        second = up_line()
        self.assertIn("target=standby", first)
        self.assertIn("target=active", second)
        self.assertEqual(first.split(" docker ", 1)[1], second.split(" docker ", 1)[1])
        self.assertEqual(list((self.root / "var/run").glob("*.target")),
                         [self.root / "var/run/slam.target"])

    def test_relative_data_dir_is_made_absolute(self):
        subprocess.run([str(self.root / "glim-up"), "autonomy/slam.yaml", "config"],
                       cwd=self.root, env=dict(self.env, GLIM_DATA_DIR="maps"),
                       capture_output=True, text=True, timeout=30)
        self.assertIn(f"data={self.root.resolve()}/maps", self.calls()[-1])

    def test_bogus_target_state_fails_up_but_no_other_verb(self):
        result = self.run_launcher("up", "-d", RIG_TARGET_STATE="certify-state-x")
        self.assertEqual(result.returncode, 1)
        self.assertIn("must be 'standby' or 'active'", result.stderr)
        for verb in ("config", "status", "state"):
            with self.subTest(verb=verb):
                self.assertEqual(self.run_launcher(verb, RIG_TARGET_STATE="certify-state-x").returncode, 0)

    def test_data_dir_selects_the_overlay(self):
        self.run_launcher("config", RIG_DATA_DIR="/srv/data")
        line = self.calls()[-1]
        self.assertIn("data=/srv/data", line)
        self.assertIn("compose.datadir.yaml", line)
        self.run_launcher("config", GLIM_DATA_DIR="/home/me/maps")
        self.assertIn("data=/home/me/maps", self.calls()[-1])
        self.run_launcher("config")
        self.assertNotIn("compose.datadir.yaml", self.calls()[-1])

    def test_rig_project_name_is_honored(self):
        self.run_launcher("status", COMPOSE_PROJECT_NAME="slam-vehicle-7")
        self.assertIn("project=slam-vehicle-7", self.calls()[-1])
        self.assertTrue(self.calls()[-1].endswith(" ps"))

    def test_down_reports_a_failed_map_save_and_still_tears_down(self):
        result = self.run_launcher("down", "-t", "30", RUNNING="1", STOP_EXIT_CODE="1")
        self.assertEqual(result.returncode, 1)
        self.assertIn("map may be incomplete", result.stderr)
        self.assertIn("shutdown log evidence", result.stderr)
        log = self.log.read_text()
        self.assertIn(" stop -t 30 glim", log)
        self.assertIn(" down -t 30", log)

    def test_clean_down(self):
        result = self.run_launcher("down", RUNNING="1")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_image_planning_never_reads_the_instance_config(self):
        self.config.write_text("service: glim\nname: '{{vehicle_name}}_slam'\n")  # fleet template
        result = self.run_launcher("images")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '- "glim:latest"')

    def test_invalid_config_fails_before_touching_docker_state(self):
        self.edit_config("T_lidar_imu: [0.0, 0.0, -0.10, 0.0, 0.0, 0.0, 1.0]", "T_lidar_imu: []")
        result = self.run_launcher("up", "-d")
        self.assertEqual(result.returncode, 2)
        self.assertIn("T_lidar_imu", result.stderr)
        self.assertNotIn("volume create", self.log.read_text())
        self.assertNotIn(" up", self.log.read_text())


if __name__ == "__main__":
    unittest.main()
