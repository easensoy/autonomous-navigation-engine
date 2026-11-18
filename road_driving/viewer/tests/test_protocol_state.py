"""Protocol decoding, the viewer's state and the demonstrations' captions (no Panda3D, no backend)."""
import json
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import demos, protocol  # noqa: E402
from ane_viewer.state import SimState  # noqa: E402


def tick(t, started=True, crossing=0.0, est_kerb="MATCHED", along=0.0, scanning=True):
    k = int(round(t * 100))
    return {"v": 1, "type": "tick", "tick": k, "t": t, "truth": {"x": t, "y": 0.0, "yaw": 0.0, "v": 8.9, "steer": 0.0, "accel": 0.0},
            "cmd": {"steer": 0.0, "accel": 0.0},
            "est": {"valid": True, "x": t + along, "y": 0.0, "yaw": 0.0, "v": 8.9, "pxx": 0.01, "pxy": 0.0, "pyy": 0.01, "pyawyaw": 1e-5,
                    "sigma": 0.1, "sigma_along": 0.1, "sigma_across": 0.05, "status": "OK", "gnss": "TRUSTED", "kerb": est_kerb, "verified": 1,
                    "unique": 1, "relocalisations": 0, "kerb_used": 10, "kerb_rejected": 0},
            "stack": {"started": started, "mode": "DRIVE", "element": "J00>J10/0", "s": t, "shift": 0.0, "rejoin": 0.0},
            "diag": {"along": along, "lateral": 0.0, "heading_deg": 0.0, "position": abs(along)}, "clearance": 0.5, "crossing": crossing,
            "sensors": {"gnss_available": True, "gnss_fault_m": 0.0, "multipath": False, "wheel_slip": False, "kerb_scanning": scanning,
                        "kerb_map_error": False, "injected": {"gnss_outage": False, "kerb_blackout": False, "gnss_bias": [0.0, 0.0]}}}


class Protocol(unittest.TestCase):
    def test_decodes_versioned_messages_only(self):
        self.assertEqual(protocol.decode('{"v":1,"type":"bye"}')["type"], "bye")
        for bad in ('{"v":2,"type":"tick"}', '{"v":1,"type":"nonsense"}', "not json", "[1,2]", ""):
            with self.assertRaises(protocol.ProtocolError):
                protocol.decode(bad)

    def test_commands_are_single_json_lines(self):
        for cmd in (protocol.reset({"scenario": "R7-A"}), protocol.pause(), protocol.resume(), protocol.step(4), protocol.speed(2),
                    protocol.gnss_outage(True), protocol.gnss_bias(True, 5, 90), protocol.kerb_blackout(False), protocol.closure_ahead(40),
                    protocol.closure_lift(), protocol.save_session("x.json"), protocol.replay("x.json"), protocol.quit_()):
            line = protocol.encode(cmd)
            self.assertTrue(line.endswith("\n") and "\n" not in line[:-1])
            self.assertIn("cmd", json.loads(line))


class State(unittest.TestCase):
    def test_a_session_resets_what_the_last_episode_left(self):
        s = SimState()
        s.apply({"v": 1, "type": "session", "tick": 0, "t": 0.0, "mode": "live", "config": {"scenario": "R7-A", "mode": "estimated"}})
        s.apply(tick(1.0))
        s.apply({"v": 1, "type": "gnss", "tick": 100, "t": 1.0, "x": 1, "y": 2, "sigma": 0.3})
        self.assertEqual(len(s.gnss), 1)
        s.apply({"v": 1, "type": "session", "tick": 0, "t": 0.0, "mode": "replay", "config": {"scenario": "R7-B"}})
        self.assertEqual(len(s.gnss), 0)
        self.assertIsNone(s.tick)
        self.assertEqual(s.mode, "replay")

    def test_markers_skip_the_alignment_before_the_mission(self):
        s = SimState()
        s.apply(tick(0.1, started=False))
        s.apply({"v": 1, "type": "event", "tick": 10, "t": 0.1, "source": "ESTIMATOR", "event": "KERB_CORRECTION", "detail": "unique", "unique": True})
        s.apply(tick(30.0))
        s.apply({"v": 1, "type": "event", "tick": 3000, "t": 30.0, "source": "STACK", "event": "POSE_CORRECTION", "detail": "unique;jump=1.2"})
        placed = [m for m in s.markers if m["pos"] is not None]
        self.assertEqual([m["event"] for m in placed], ["POSE_CORRECTION"])

    def test_operator_commands_join_the_events_at_their_ticks(self):
        s = SimState()
        s.apply({"v": 1, "type": "applied", "tick": 1204, "t": 12.04, "command": {"tick": 1204, "cmd": "gnss_outage", "on": True}})
        s.apply({"v": 1, "type": "applied", "tick": 2014, "t": 20.14, "command": {"tick": 2014, "cmd": "gnss_bias", "dx": -1e-16, "dy": 5.0},
                 "replay": True})
        s.apply({"v": 1, "type": "applied", "tick": 2403, "t": 24.03, "command": {"tick": 2403, "cmd": "gnss_bias", "dx": 0.0, "dy": 0.0}})
        self.assertEqual([(e["t"], e["source"], e["event"], e["detail"]) for e in s.events],
                         [(12.04, "OPERATOR", "GNSS_OUTAGE", "tick 1204; on"), (20.14, "REPLAY", "GNSS_BIAS", "tick 2014; offset +0.00, +5.00 m"),
                          (24.03, "OPERATOR", "GNSS_BIAS", "tick 2403; off")])
        self.assertEqual(len(s.applied), 3)

    def test_readout_names_every_diagnostic(self):
        s = SimState()
        s.apply({"v": 1, "type": "session", "tick": 0, "t": 0.0, "mode": "live", "config": {"scenario": "R7-A", "mode": "estimated"}})
        s.apply(tick(12.0, along=-2.7, crossing=0.3))
        labels = [r[0] for r in s.readout()]
        for need in ("time", "speed", "stack", "GNSS", "kerb scanner", "estimate", "corrections", "error (truth)", "clearance"):
            self.assertIn(need, labels)
        self.assertAlmostEqual(s.max_crossing, 0.3)


class Demonstrations(unittest.TestCase):
    def test_every_demonstration_uses_offered_options(self):
        for d in demos.DEMOS:
            c = d.config
            self.assertIn(c["scenario"], demos.ROUTES)
            self.assertIn(c["condition"], demos.CONDITIONS[demos.network_of(c["scenario"])])
            self.assertIn(c["estimator"], demos.ESTIMATORS)
            self.assertIn(c["rejoin"], demos.REJOINS)

    def test_demo_c_is_the_r7_failure_episode(self):
        c = demos.by_key("C").config
        self.assertEqual((c["scenario"], c["condition"], c["seed"], c["estimator"], c["rejoin"]), ("R7-A", "kerb_faults_r7", 7, "ekf_kerb2", "unique"))

    def test_captions_follow_what_the_backend_reports(self):
        s = SimState()
        s.apply({"v": 1, "type": "session", "tick": 0, "t": 0.0, "mode": "live", "config": demos.by_key("C").config})
        n = demos.Narrator(demos.by_key("C"))
        s.apply(tick(5.0))
        n.update(s)
        self.assertEqual(len(n.lines), 1)  # only the introduction
        s.apply(tick(50.0, scanning=False, along=-2.0))
        n.update(s)
        self.assertIn("No kerb scans", n.lines[-1])
        self.assertIn("-2.00", n.lines[-1])
        s.apply({"v": 1, "type": "event", "tick": 5436, "t": 54.36, "source": "STACK", "event": "POSE_CORRECTION", "detail": "unique;jump=-1.232"})
        s.apply(tick(54.4, along=-2.67))
        n.update(s)
        self.assertIn("1.23", n.lines[-1])
        s.apply(tick(61.4, crossing=0.876))
        n.update(s)
        self.assertIn("0.88", n.lines[-1])


if __name__ == "__main__":
    unittest.main()
