#!/usr/bin/env python3
"""Records an X display as JPEG frames, each stamped with the container's monotonic time.

  python3 capture_frames.py --display :99 --out <capture dir> [--fps 10] [--quality 90]

Writes <capture dir>/frames/%05d.jpg and <capture dir>/frames.csv (index, monotonic_s: the midpoint of
the grab). ffmpeg's x11grab stamps frames with the system clock, which a Docker Desktop VM steps back by
about a second every 30 s, and a recording read as "frame i at i / fps" drifts whenever the host is too
busy to keep the rate. Here every frame carries its own monotonic time, so make_video.py places it on
the simulation clock exactly, however many frames a busy host skipped. Grabbing and JPEG encoding run
in separate threads; the rate is at most --fps. Stops on SIGTERM or SIGINT. Needs Pillow with XCB.
"""

import argparse
import queue
import signal
import threading
import time
from pathlib import Path

from PIL import ImageGrab


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--display", default=":99")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--quality", type=int, default=90)
    args = parser.parse_args()
    (args.out / "frames").mkdir(parents=True, exist_ok=True)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    pending = queue.Queue(maxsize=8)

    def writer():
        with open(args.out / "frames.csv", "w") as log:
            log.write("index,monotonic_s\n")
            while True:
                item = pending.get()
                if item is None:
                    return
                index, stamp, image = item
                image.convert("RGB").save(args.out / "frames" / f"{index:05d}.jpg", quality=args.quality)
                log.write(f"{index},{stamp:.4f}\n")
                log.flush()

    thread = threading.Thread(target=writer)
    thread.start()
    period, index = 1.0 / args.fps, 0
    next_tick = time.monotonic()
    while not stop.is_set():
        before = time.monotonic()
        image = ImageGrab.grab(xdisplay=args.display)
        after = time.monotonic()
        pending.put((index, (before + after) / 2, image))
        index += 1
        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            stop.wait(delay)
        else:
            next_tick = time.monotonic()  # behind: no catching up, the stamps say when each frame was taken
    pending.put(None)
    thread.join()


if __name__ == "__main__":
    main()
