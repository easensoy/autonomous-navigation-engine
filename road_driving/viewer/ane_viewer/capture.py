"""Screenshots and video of the viewer's own frames.

A video is recorded in lock-step: the backend is paused and advanced exactly `ticks_per_frame` ticks per
captured frame (2 ticks of 10 ms at 50 frames per second, 4 at 25), so the video plays at real time however
long a frame takes to render and encode. A recording can be drawn into an offscreen multisampled buffer at
its own size (1920 x 1080 for the showcase: rendered at that size, never scaled), with the HUD drawn into it
too. Frames go straight to ffmpeg (H.264, yuv420p, fast start); without ffmpeg they are saved as PNGs. Every
frame is logged with its tick, its camera and the clock text the HUD showed.
"""
import glob
import json
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


def ticks_per_frame(fps, tick_hz=100):
    """Whole simulation ticks per video frame; the frame rate must divide the tick rate."""
    k = tick_hz / float(fps)
    if abs(k - round(k)) > 1e-9 or round(k) < 1:
        raise ValueError("%g frames per second does not divide %d ticks per second" % (fps, tick_hz))
    return int(round(k))


def encoder_args(width, height, fps, crf=18, preset="slow"):
    """ffmpeg arguments for raw RGB frames (bottom row first, as OpenGL reads them) in, H.264 out."""
    return ["-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (width, height), "-framerate", str(fps), "-i", "-",
            "-vf", "vflip", "-c:v", "libx264", "-preset", preset, "-crf", str(crf), "-profile:v", "high", "-pix_fmt", "yuv420p",
            "-g", str(int(fps)), "-colorspace", "bt709", "-color_primaries", "bt709", "-color_trc", "bt709", "-movflags", "+faststart"]


class VideoWriter:
    def __init__(self, path, width, height, fps=50, ffmpeg=None, crf=18, preset="slow"):
        self.path = pathlib.Path(path)
        self.width, self.height, self.fps = width, height, fps
        self.frames = 0
        self.ffmpeg = ffmpeg or find_ffmpeg()
        self.proc = None
        self.frame_dir = None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.ffmpeg:
            cmd = [self.ffmpeg] + encoder_args(width, height, fps, crf, preset) + [str(self.path)]
            flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, creationflags=flags)
        else:
            self.frame_dir = self.path.with_suffix("")
            self.frame_dir.mkdir(parents=True, exist_ok=True)

    def add_texture(self, tex):
        """A frame from a texture with a RAM copy (the offscreen target) or a window screenshot."""
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

    def add(self, base):
        return self.add_texture(base.win.getScreenshot())

    def close(self):
        if self.proc:
            self.proc.stdin.close()
            self.proc.wait()
        return self.path if self.proc else self.frame_dir


class OffscreenTarget:
    """A multisampled offscreen buffer of the recording's size, drawing the main camera's view and the HUD."""

    def __init__(self, base, width, height, samples=8):
        from panda3d.core import FrameBufferProperties, GraphicsOutput, GraphicsPipe, Texture, WindowProperties
        fbp = FrameBufferProperties()
        fbp.setRgbColor(True)
        fbp.setRgbaBits(8, 8, 8, 8)
        fbp.setDepthBits(24)
        fbp.setMultisamples(samples)
        self.buffer = base.graphicsEngine.makeOutput(base.pipe, "recording", -2, fbp, WindowProperties.size(width, height), GraphicsPipe.BFRefuseWindow,
                                                     base.win.getGsg(), base.win)
        if self.buffer is None:
            raise RuntimeError("could not create a %dx%d offscreen buffer" % (width, height))
        self.samples = self.buffer.getFbProperties().getMultisamples()
        self.tex = Texture("recording")
        self.buffer.addRenderTexture(self.tex, GraphicsOutput.RTMCopyRam)
        self.buffer.setClearColor(base.win.getClearColor())
        self.cam = base.makeCamera(self.buffer, lens=base.camLens)  # the same lens: the same field of view
        self.cam.reparentTo(base.camera)
        self.cam2d = base.makeCamera2d(self.buffer)
        self.width, self.height = width, height

    def close(self, base):
        base.graphicsEngine.removeWindow(self.buffer)


class FrameLog:
    def __init__(self, path):
        self.path = pathlib.Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "w", encoding="utf-8", newline="\n")
        self.n = 0

    def write(self, **row):
        row = dict(row, frame=self.n)
        self.f.write(json.dumps(row, separators=(",", ":")) + "\n")
        self.n += 1

    def note(self, obj):
        """A line that is not a frame: the run's configuration, the backend's end message."""
        self.f.write(json.dumps({"note": obj}, separators=(",", ":")) + "\n")

    def close(self):
        self.f.close()
