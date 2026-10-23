"""The control socket and the glim_supervisor CLI's stdout discipline."""
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))

from glim_supervisor import __main__ as cli  # noqa: E402
from glim_supervisor import control  # noqa: E402


class FakeController:
    def __init__(self):
        self.target = "standby"
        self.fail = None

    def state(self):
        return {"state": self.target, "target": self.target, "detail": "fake"}

    def health(self):
        return (self.fail is None, self.fail or "ok")

    def transition(self, target):
        if self.fail:
            raise RuntimeError(self.fail)
        self.target = target


class ControlSocketTests(unittest.TestCase):
    def setUp(self):
        # AF_UNIX paths are short (104 bytes on macOS): keep the directory name small.
        self.temp = tempfile.TemporaryDirectory(prefix="gs")
        self.addCleanup(self.temp.cleanup)
        self.path = str(Path(self.temp.name) / "s.sock")
        self.controller = FakeController()
        server = control.Server(self.path, control.Handler)
        server.supervisor = cli.Supervisor(self.controller)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)

    def test_round_trip(self):
        self.assertEqual(control.request("state", path=self.path)["state"], "standby")
        control.request("activate", path=self.path)
        self.assertEqual(self.controller.target, "active")
        self.assertTrue(control.request("health", path=self.path)["healthy"])

    def test_failures_carry_the_reason(self):
        self.controller.fail = "GLIM exited during startup (code 134)"
        with self.assertRaisesRegex(RuntimeError, "code 134"):
            control.request("activate", path=self.path)
        self.assertFalse(control.request("health", path=self.path)["healthy"])
        with self.assertRaisesRegex(RuntimeError, "unknown command"):
            control.request("reboot", path=self.path)

    def test_cli_state_prints_exactly_one_json_object(self):
        stdout = io.StringIO()
        with mock.patch.object(control, "SOCKET", self.path), mock.patch("sys.stdout", stdout):
            self.assertEqual(cli.main(["state"]), 0)
        self.assertEqual(json.loads(stdout.getvalue()),
                         {"state": "standby", "target": "standby", "detail": "fake"})

    def test_cli_health_exit_code(self):
        with mock.patch.object(control, "SOCKET", self.path):
            self.assertEqual(cli.main(["health"]), 0)
            self.controller.fail = "3 consecutive failed sessions"
            with mock.patch("sys.stderr", io.StringIO()):
                self.assertEqual(cli.main(["health"]), 1)

    def test_initial_target_comes_from_the_seed_file(self):
        seed = Path(self.temp.name) / "target"
        with mock.patch.object(cli, "TARGET_SEED", str(seed)), \
                mock.patch.dict("os.environ", {"GLIM_TARGET_STATE": "standby"}):
            self.assertEqual(cli.initial_target(), "standby")   # no file: env, else active
            seed.write_text("active\n")
            self.assertEqual(cli.initial_target(), "active")
            seed.write_text("parked\n")
            with self.assertRaises(ValueError):
                cli.initial_target()

    def test_cli_without_a_supervisor_fails_cleanly(self):
        with mock.patch.object(control, "SOCKET", self.path + ".missing"), \
                mock.patch("sys.stderr", io.StringIO()), mock.patch("sys.stdout", io.StringIO()) as out:
            self.assertEqual(cli.main(["state"]), 1)
        self.assertEqual(out.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
