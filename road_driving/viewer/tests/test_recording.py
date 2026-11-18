"""Recording settings and the recording checks, without Panda3D or ffmpeg: lock-step tick arithmetic, the
encoder settings, and the frame-log verification catching a skipped tick, a camera inside the scenery and a
HUD clock that disagrees with its frame."""
import json
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "tools"))

from ane_viewer.capture import FrameLog, encoder_args, ticks_per_frame  # noqa: E402
from check_recording import check_log  # noqa: E402


def write_log(path, frames, end_tick):
    log = FrameLog(path)
    log.note({"type": "config", "config": {"scenario": "KEN-A"}})
    for row in frames:
        log.write(**row)
    log.note({"type": "end", "tick": end_tick, "outcome": "COMPLETED"})
    log.close()


def frame(tick, phase="run", inside=False, share=1.0, clock=None, camera="chase"):
    t = tick / 100.0
    return dict(tick=tick, t=t, phase=phase, shot=camera, camera=camera, pos=[0, 0, 3], inside=inside, share=share,
                clock=clock if clock is not None else "t = %.2f s" % t, ok=True)


class LockStep(unittest.TestCase):
    def test_ticks_per_frame(self):
        self.assertEqual(ticks_per_frame(50), 2)
        self.assertEqual(ticks_per_frame(25), 4)
        self.assertEqual(ticks_per_frame(100), 1)
        for fps in (30, 60, 24, 200):
            with self.assertRaises(ValueError):
                ticks_per_frame(fps)

    def test_encoder_settings(self):
        args = encoder_args(1920, 1080, 50, crf=18, preset="slow")
        text = " ".join(args)
        for want in ("-s 1920x1080", "-framerate 50", "-c:v libx264", "-crf 18", "-preset slow", "-pix_fmt yuv420p", "-movflags +faststart",
                     "-profile:v high", "-vf vflip"):
            self.assertIn(want, text)

    def test_a_clean_log_passes(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "c.jsonl"
            rows = [frame(2, "title")] * 3 + [frame(k) for k in range(2, 202, 2)] + [frame(200, "end")] * 2
            write_log(p, rows, 200)
            summary, problems = check_log(p, 2)
            self.assertEqual(problems, [])
            self.assertEqual((summary["run_frames"], summary["first_tick"], summary["last_tick"]), (100, 2, 200))

    def test_the_checks_catch_faults(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "c.jsonl"
            rows = [frame(k) for k in range(2, 100, 2)]
            rows[10] = frame(rows[10]["tick"] + 2)  # a skipped tick
            rows[20] = frame(rows[20]["tick"], inside=True)
            rows[30] = frame(rows[30]["tick"], share=0.25)
            rows[40] = frame(rows[40]["tick"], clock="t = 0.00 s")
            write_log(p, rows, 98)
            _, problems = check_log(p, 2)
            text = " | ".join(problems)
            for want in ("advance by", "inside the scenery", "out of view", "HUD clock"):
                self.assertIn(want, text)

    def test_an_overhead_view_may_not_see_all_points(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "c.jsonl"
            write_log(p, [frame(k, share=0.5, camera="overhead") for k in range(2, 50, 2)], 48)
            _, problems = check_log(p, 2)
            self.assertEqual(problems, [])


if __name__ == "__main__":
    unittest.main()
