"""The real backend process, driven as the viewer drives it (needs a built road_drive; no graphics).

Skipped when no backend is found (set ANE_BACKEND, or build road_drive into one of the usual folders).
"""
import json
import pathlib
import subprocess
import sys
import tempfile
import time
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import protocol  # noqa: E402
from ane_viewer.backend import Backend, backend_environment, find_backend  # noqa: E402

EXE = find_backend()
DEMO_C = {"scenario": "R7-A", "condition": "kerb_faults_r7", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 7, "rejoin": "unique"}


def headless_trace(config):
    out = subprocess.run([str(EXE), "drive", config["scenario"], "--condition", config["condition"], "--mode", config["mode"], "--estimator",
                          config["estimator"], "--seed", str(config["seed"]), "--rejoin", config["rejoin"]], capture_output=True, text=True,
                         env=backend_environment(EXE), check=True).stdout
    return out.split("trace=")[1].split()[0]


def wait_for(backend, pred, timeout=60.0):
    """Reads messages until one satisfies `pred`; every message read is also kept in backend.history."""
    seen = []
    t0 = time.time()
    if not hasattr(backend, "history"):
        backend.history = []
    while time.time() - t0 < timeout:
        for m in backend.poll():
            seen.append(m)
            backend.history.append(m)
            if pred(m):
                return m, seen
        time.sleep(0.005)
    raise AssertionError("timed out")


def last_tick(backend):
    return max([m["tick"] for m in getattr(backend, "history", []) if m["type"] == "tick"] + [-1])


@unittest.skipIf(EXE is None, "no road_drive backend built")
class Backend_(unittest.TestCase):
    def test_live_run_matches_the_headless_run(self):
        b = Backend(EXE, ["--speed", "0"]).start()
        try:
            hello, _ = wait_for(b, lambda m: m["type"] == "hello", 10)
            self.assertEqual(hello["protocol"], "ane-live")
            b.send(protocol.reset(DEMO_C))
            end, seen = wait_for(b, lambda m: m["type"] == "end", 120)
        finally:
            b.stop()
        ticks = [m["tick"] for m in seen if m["type"] == "tick"]
        self.assertEqual(ticks, list(range(ticks[0], ticks[0] + len(ticks))))  # consecutive tick IDs
        self.assertEqual(end["trace_hash"], headless_trace(DEMO_C))
        self.assertGreater(end["metrics"]["max_crossing"], 0.5)  # the failure is kept, not smoothed away

    def test_pause_freezes_time_and_step_advances_exactly(self):
        b = Backend(EXE, ["--speed", "1"]).start()
        try:
            b.send(protocol.reset(DEMO_C))
            wait_for(b, lambda m: m["type"] == "tick" and m["tick"] >= 50, 30)
            b.send(protocol.pause())
            wait_for(b, lambda m: m["type"] == "status" and m["state"] == "paused", 10)
            time.sleep(0.3)
            for m in b.poll():
                b.history.append(m)
            last = last_tick(b)
            time.sleep(1.0)
            later = [m for m in b.poll() if m["type"] == "tick"]
            self.assertEqual(later, [], "ticks ran while paused")
            b.send(protocol.step(7))
            m, seen = wait_for(b, lambda m: m["type"] == "tick" and m["tick"] >= last + 7, 10)
            time.sleep(0.3)
            extra = [x for x in b.poll() if x["type"] == "tick"]
            self.assertEqual(m["tick"], last + 7)
            self.assertEqual(extra, [])
            b.send(protocol.resume())
            wait_for(b, lambda m: m["type"] == "tick" and m["tick"] > last + 30, 10)
        finally:
            code = b.stop()
        self.assertEqual(code, 0)

    def test_closing_the_viewer_ends_the_backend(self):
        b = Backend(EXE, ["--speed", "1"]).start()
        b.send(protocol.reset(DEMO_C))
        wait_for(b, lambda m: m["type"] == "tick", 30)
        t0 = time.time()
        b.proc.stdin.close()  # what the operating system does when the viewer exits
        code = b.proc.wait(timeout=5)
        self.assertLess(time.time() - t0, 3.0)
        self.assertEqual(code, 0)

    def test_an_injected_session_replays_headlessly(self):
        with tempfile.TemporaryDirectory() as d:
            path = str(pathlib.Path(d) / "s.json")
            b = Backend(EXE, ["--speed", "0", "--paused", "on"]).start()
            try:
                b.send(protocol.reset({"scenario": "R7-A", "condition": "nominal_r7", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 2,
                                       "rejoin": "unique"}))
                wait_for(b, lambda m: m["type"] == "session", 30)  # paused before tick 0
                b.send(protocol.step(1500))
                wait_for(b, lambda m: m["type"] == "tick" and m["tick"] >= 1499, 30)
                b.send(protocol.gnss_outage(True))
                b.send(protocol.step(600))
                wait_for(b, lambda m: m["type"] == "tick" and m["tick"] >= 2099, 30)
                b.send(protocol.gnss_outage(False))
                b.send(protocol.kerb_blackout(True))
                b.send(protocol.resume())
                end, seen = wait_for(b, lambda m: m["type"] == "end", 120)
                applied = [m for m in b.history if m["type"] == "applied"]
                b.send(protocol.save_session(path))
                wait_for(b, lambda m: m["type"] == "saved", 10)
            finally:
                b.stop()
            self.assertEqual(len(applied), 3)
            out = subprocess.run([str(EXE), "session-replay", path], capture_output=True, text=True, env=backend_environment(EXE))
            self.assertEqual(out.returncode, 0, out.stdout + out.stderr)
            self.assertIn(end["trace_hash"], out.stdout)
            session = json.loads(pathlib.Path(path).read_text())
            self.assertEqual([c["tick"] for c in session["commands"]], [m["tick"] for m in applied])


if __name__ == "__main__":
    unittest.main()
