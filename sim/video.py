"""The pool camera as a live H.264 stream: ffmpeg in a subprocess, fed raw RGB frames from a writer thread.

Encoder settings are the ones BlueOS's camera manager and Cockpit's WebRTC want: H.264 constrained-baseline, no
B-frames, an IDR every second, parameter sets repeated, zero-latency tuning. The destination is an RTSP server
(mediamtx, deploy/docker/pool/compose.yaml) that BlueOS pulls as a "Redirect" source; a path ending in .ts or .mp4 writes a
file instead (tests, recordings).

The render loop never waits on the encoder: ``push`` replaces the one pending frame, and a slow or absent consumer
costs dropped frames (counted), not a slower simulation. If ffmpeg exits (no RTSP server yet, server restarted) it is
restarted every ``retry_s``.
"""
from __future__ import annotations

import logging
import os
import subprocess
import threading
import time

import numpy as np

log = logging.getLogger("sim.video")


def ffmpeg_exe() -> str:
    from uw import deps
    return deps.require("imageio_ffmpeg", "imageio-ffmpeg", "0.5.1").get_ffmpeg_exe()


def ffmpeg_args(ffmpeg: str, url: str, width: int, height: int, fps: float, encoder: str = "libx264",
                bitrate_kbps: int = 4000, threads: int = 2) -> list[str]:
    g = max(1, int(round(fps)))
    cmd = [ffmpeg, "-hide_banner", "-loglevel", "warning", "-y",
           "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-framerate", f"{fps:g}", "-i", "-",
           "-an", "-pix_fmt", "yuv420p", "-g", str(g), "-keyint_min", str(g), "-bf", "0",
           "-b:v", f"{bitrate_kbps}k", "-maxrate", f"{bitrate_kbps}k", "-bufsize", f"{max(bitrate_kbps // 2, 1)}k"]
    if encoder == "libx264":
        cmd += ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-profile:v", "baseline",
                "-x264-params", "repeat-headers=1", "-threads", str(int(threads))]
    elif encoder == "h264_nvenc":
        cmd += ["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull", "-zerolatency", "1", "-profile:v", "baseline"]
    else:
        raise ValueError(f"stream.encoder must be libx264 | h264_nvenc, got {encoder!r}")
    if url.startswith("rtsp://"):
        cmd += ["-f", "rtsp", "-rtsp_transport", "tcp", url]
    elif url.endswith(".ts"):
        cmd += ["-f", "mpegts", url]
    else:
        cmd += [url]
    return cmd


class VideoStream:
    def __init__(self, url: str, width: int, height: int, fps: float, encoder: str = "libx264",
                 bitrate_kbps: int = 4000, retry_s: float = 2.0, ffmpeg: str | None = None, threads: int = 2,
                 nice: int = 10):
        self.url, self.width, self.height, self.fps = url, int(width), int(height), float(fps)
        self.threads, self.nice = int(threads), int(nice)
        self.encoder, self.bitrate_kbps, self.retry_s = encoder, int(bitrate_kbps), float(retry_s)
        self.ffmpeg = ffmpeg
        self.proc = None
        self.frame = None
        self.cond = threading.Condition()
        self.stopping = False
        self.thread = None
        self.counts = {"pushed": 0, "written": 0, "dropped": 0, "restarts": 0}
        self.write_ms: list[float] = []

    def start(self) -> "VideoStream":
        self.ffmpeg = self.ffmpeg or ffmpeg_exe()
        self.thread = threading.Thread(target=self._run, name="uw-video", daemon=True)
        self.thread.start()
        log.info("video: %dx%d @ %g fps, %s %d kbit/s -> %s", self.width, self.height, self.fps, self.encoder,
                 self.bitrate_kbps, self.url)
        return self

    def push(self, rgb: np.ndarray) -> None:
        """Queue one frame (uint8 H x W x 3); replaces a frame the writer has not taken yet."""
        if rgb.shape != (self.height, self.width, 3):
            raise ValueError(f"frame {rgb.shape} != {(self.height, self.width, 3)}")
        with self.cond:
            if self.frame is not None:
                self.counts["dropped"] += 1
            self.frame = rgb
            self.counts["pushed"] += 1
            self.cond.notify()

    def _spawn(self):
        args = ffmpeg_args(self.ffmpeg, self.url, self.width, self.height, self.fps, self.encoder, self.bitrate_kbps,
                           self.threads)
        # the renderer comes first: the encoder runs niced and on a few threads, so it takes idle CPU, not Kit's
        pre = (lambda: os.nice(self.nice)) if os.name == "posix" and self.nice else None
        return subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, bufsize=0, preexec_fn=pre)

    def _run(self) -> None:
        next_try = 0.0
        while True:
            with self.cond:
                while self.frame is None and not self.stopping:
                    self.cond.wait(0.5)
                if self.stopping:
                    break
                frame, self.frame = self.frame, None
            if self.proc is None or self.proc.poll() is not None:
                if self.proc is not None:
                    log.warning("video: ffmpeg exited with %s (is the RTSP server up?); retrying every %.0f s",
                                self.proc.returncode, self.retry_s)
                    self.proc = None
                    self.counts["restarts"] += 1
                    next_try = time.monotonic() + self.retry_s
                if time.monotonic() < next_try:
                    self.counts["dropped"] += 1
                    continue
                self.proc = self._spawn()
            t = time.perf_counter()
            try:
                self.proc.stdin.write(np.ascontiguousarray(frame, np.uint8).tobytes())
                self.counts["written"] += 1
                self.write_ms.append((time.perf_counter() - t) * 1e3)
            except (BrokenPipeError, OSError):
                self.counts["dropped"] += 1

    def stats(self) -> dict:
        w = self.write_ms[-500:]
        return {**self.counts, "write_ms_mean": round(float(np.mean(w)), 2) if w else None}

    def close(self, timeout_s: float = 5.0) -> None:
        with self.cond:
            self.stopping = True
            self.cond.notify()
        if self.thread is not None:
            self.thread.join(timeout_s)
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            try:
                self.proc.wait(timeout_s)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None
