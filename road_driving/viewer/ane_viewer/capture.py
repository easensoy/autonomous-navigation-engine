"""Screenshots and video of the viewer's own frames.

A video is recorded in lock-step: the backend is paused and advanced exactly `ticks_per_frame` ticks per
captured frame (4 ticks of 10 ms at 25 frames per second), so the video plays at real time however long a
frame takes to render and encode. Frames go straight to ffmpeg; without ffmpeg they are saved as PNGs.
"""
import glob
import os
import pathlib
import shutil
import subprocess


def find_ffmpeg():
    exe = os.environ.get("ANE_FFMPEG")
    if exe and pathlib.Path(exe).is_file():
        return exe
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg  # noqa: F401
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:  # noqa: BLE001 - optional
        pass
    pattern = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Python", "Python3*", "Lib", "site-packages", "imageio_ffmpeg", "binaries", "ffmpeg*.exe")
    found = sorted(glob.glob(pattern))
    return found[-1] if found else None


class VideoWriter:
    def __init__(self, path, width, height, fps=25, ffmpeg=None):
        self.path = pathlib.Path(path)
        self.width, self.height, self.fps = width, height, fps
        self.frames = 0
        self.ffmpeg = ffmpeg or find_ffmpeg()
        self.proc = None
        self.frame_dir = None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.ffmpeg:
            cmd = [self.ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (width, height), "-r", str(fps),
                   "-i", "-", "-vf", "vflip", "-c:v", "libx264", "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                   str(self.path)]
            flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, creationflags=flags)
        else:
            self.frame_dir = self.path.with_suffix("")
            self.frame_dir.mkdir(parents=True, exist_ok=True)

    def add(self, base):
        tex = base.win.getScreenshot()
        if tex is None:
            return False
        if self.proc:
            data = bytes(tex.getRamImageAs("RGB"))
            if len(data) != self.width * self.height * 3:
                return False
            self.proc.stdin.write(data)
        else:
            from panda3d.core import Filename
            tex.write(Filename.fromOsSpecific(str(self.frame_dir / ("frame_%05d.png" % self.frames))))
        self.frames += 1
        return True

    def close(self):
        if self.proc:
            self.proc.stdin.close()
            self.proc.wait()
        return self.path if self.proc else self.frame_dir
