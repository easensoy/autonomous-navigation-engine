"""The simulation backend as a child process: `road_drive live`, spoken to over its stdin and stdout.

The backend is the authority: the viewer only draws what it reports and sends operator commands, which the
backend applies at a tick it records. Closing the viewer closes the backend's input, which stops it.
"""
import collections
import os
import pathlib
import subprocess
import threading
import time

from . import protocol

REPO = pathlib.Path(__file__).resolve().parents[3]
CANDIDATES = [
    "build/road-msvc/road_driving/road_drive.exe",
    "build/road-gcc/road_driving/road_drive.exe",
    "build/road_driving/road_drive.exe",
    "build/road_driving/road_drive",
    "build/road-gcc/road_driving/road_drive",
    "build/road-msvc/road_driving/road_drive",
]
BUILD_HELP = """The simulation backend (road_drive) was not found. Build it from the repository root, for example:

  cmake -S . -B build/road-msvc -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_ROAD_DRIVING=ON
  cmake --build build/road-msvc --target road_drive

(from a Visual Studio developer prompt), or pass --backend PATH, or set ANE_BACKEND."""


def find_backend(explicit=None):
    """The backend executable: --backend, then $ANE_BACKEND, then the usual build folders. None if absent."""
    for p in (explicit, os.environ.get("ANE_BACKEND")):
        if p and pathlib.Path(p).is_file():
            return pathlib.Path(p)
    for rel in CANDIDATES:
        p = REPO / rel
        if p.is_file():
            return p
    return None


def backend_environment(exe):
    """A GCC (MSYS2 UCRT64) build needs its runtime DLLs on PATH."""
    env = dict(os.environ)
    if os.name == "nt" and "road-gcc" in str(exe).replace("\\", "/"):
        ucrt = pathlib.Path("C:/msys64/ucrt64/bin")
        if ucrt.is_dir():
            env["PATH"] = str(ucrt) + os.pathsep + env.get("PATH", "")
    return env


class Backend:
    """Starts `road_drive live`, reads its messages on a thread, writes commands."""

    def __init__(self, exe, args=(), log_path=None):
        self.exe = pathlib.Path(exe)
        self.args = list(args)
        self.log_path = log_path
        self.proc = None
        self.messages = collections.deque()
        self.errors = collections.deque(maxlen=50)
        self.lock = threading.Lock()
        self._reader = None
        self._log = None
        self.received = 0

    def start(self):
        cmd = [str(self.exe), "live"] + self.args
        self._log = open(self.log_path, "w", encoding="utf-8") if self.log_path else subprocess.DEVNULL
        flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._log, bufsize=1 << 16,
                                     env=backend_environment(self.exe), creationflags=flags)
        self._reader = threading.Thread(target=self._read, name="backend-reader", daemon=True)
        self._reader.start()
        return self

    def _read(self):
        for raw in self.proc.stdout:
            try:
                msg = protocol.decode(raw.decode("utf-8", "replace"))
            except protocol.ProtocolError as e:
                self.errors.append(str(e))
                continue
            with self.lock:
                self.messages.append(msg)
                self.received += 1

    def poll(self, limit=None):
        """Every message received since the last call (oldest first)."""
        out = []
        with self.lock:
            while self.messages and (limit is None or len(out) < limit):
                out.append(self.messages.popleft())
        return out

    def send(self, cmd):
        if not self.alive():
            return False
        try:
            self.proc.stdin.write(protocol.encode(cmd).encode("utf-8"))
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError):
            return False

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self, timeout=3.0):
        """Asks the backend to quit, closes its input, and kills it if it does not exit in time."""
        if self.proc is None:
            return None
        if self.alive():
            self.send(protocol.quit_())
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            t0 = time.time()
            while self.alive() and time.time() - t0 < timeout:
                time.sleep(0.02)
            if self.alive():
                self.proc.kill()
        code = self.proc.wait()
        if self._log not in (None, subprocess.DEVNULL):
            self._log.close()
        return code
