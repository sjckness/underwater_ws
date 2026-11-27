"""Where the detector's frames come from: a video URL or file through OpenCV, or a ROS image topic.

Every source fills the same one-slot mailbox (``Latest``). A live source (RTSP, UDP, HTTP, a camera device)
overwrites a frame the detector has not taken yet, so inference always runs on the newest frame and a slow model
costs frame rate, never latency; the overwritten frames are counted. A file is read losslessly instead: the reader
waits for each frame to be taken, so an offline run sees every frame.

Only numpy and OpenCV here, so the host's tests import this module without Ultralytics or ROS.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

log = logging.getLogger("yolo_detector.source")

LIVE_SCHEMES = ("rtsp://", "rtsps://", "udp://", "tcp://", "rtp://", "srt://", "http://", "https://")
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}
#: FFmpeg demuxer options for OpenCV's reader of a live stream. TCP: mediamtx in docker/pool/compose.yaml serves RTSP
#: over TCP only, and BlueOS's camera manager answers on TCP as well. nobuffer + low_delay: hand each frame over as
#: soon as it is decoded instead of filling a demuxer buffer first. Not for files: nobuffer loses a file's last frame.
FFMPEG_CAPTURE_OPTIONS = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0"
_ENV_OPTIONS = "OPENCV_FFMPEG_CAPTURE_OPTIONS"
_USER_OPTIONS = os.environ.get(_ENV_OPTIONS)        # a value set by the user wins, for live sources and files alike


@dataclass
class Frame:
    image: np.ndarray        # H x W x 3 uint8, BGR (OpenCV's order)
    stamp_ns: int            # what goes in the ROS header: the capture time (wall clock), or the source message's
    t_got: float             # time.perf_counter() when the frame was decoded: the start of the measured latency
    seq: int                 # 1, 2, ... in decode order: gaps are frames the detector never saw


class Latest:
    """A one-slot mailbox between one producer thread and the detector loop."""

    def __init__(self) -> None:
        self.cond = threading.Condition()
        self.item: Frame | None = None
        self.closed = False
        self.received = 0
        self.overwritten = 0

    def put(self, frame: Frame, wait: bool = False) -> bool:
        """Leave ``frame`` for the consumer. ``wait``: block until the previous one is taken (a file, lossless);
        otherwise replace it (a live source). False once closed."""
        with self.cond:
            while wait and self.item is not None and not self.closed:
                self.cond.wait(0.1)
            if self.closed:
                return False
            if self.item is not None:
                self.overwritten += 1
            self.item = frame
            self.received += 1
            self.cond.notify_all()
            return True

    def take(self, timeout_s: float = 0.5) -> Frame | None:
        """The newest frame, or None after ``timeout_s`` without one (check ``done`` to tell the two apart)."""
        with self.cond:
            if self.item is None and not self.closed:
                self.cond.wait(timeout_s)
            item, self.item = self.item, None
            self.cond.notify_all()
            return item

    @property
    def done(self) -> bool:
        """Closed and drained: the source ended (end of file) or was stopped."""
        with self.cond:
            return self.closed and self.item is None

    def close(self) -> None:
        with self.cond:
            self.closed = True
            self.cond.notify_all()


def is_live(source: str) -> bool:
    return source.isdigit() or source.lower().startswith(LIVE_SCHEMES)


class VideoSource:
    """OpenCV's FFmpeg reader in a thread: an RTSP/UDP/HTTP URL, a camera index, a video file or one image.

    A live source that fails to open or stops delivering (the sim not up yet, mediamtx restarted) is reopened every
    ``retry_s`` until ``stop()``. A file ends the run at its last frame.
    """

    def __init__(self, source: str, box: Latest, retry_s: float = 2.0):
        self.source, self.box, self.retry_s = str(source), box, float(retry_s)
        self.live = is_live(self.source)
        if not self.live:
            path = Path(self.source).expanduser()
            if not path.exists():
                raise FileNotFoundError(f"source {self.source!r}: no such file, and not a URL or a camera index")
            self.source = str(path.resolve())
        self.stopping = threading.Event()
        self.thread: threading.Thread | None = None
        self.opens = 0
        self.fps = 0.0

    def start(self) -> "VideoSource":
        self.thread = threading.Thread(target=self._run, name="yolo-source", daemon=True)
        self.thread.start()
        return self

    def stop(self, timeout_s: float = 2.0) -> None:
        self.stopping.set()
        self.box.close()
        if self.thread is not None:
            self.thread.join(timeout_s)

    def _open(self):
        import cv2
        if self.source.isdigit():
            return cv2.VideoCapture(int(self.source))
        opts = _USER_OPTIONS if _USER_OPTIONS is not None else (FFMPEG_CAPTURE_OPTIONS if self.live else None)
        if opts is None:
            os.environ.pop(_ENV_OPTIONS, None)
        else:
            os.environ[_ENV_OPTIONS] = opts               # OpenCV reads it at every open
        return cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)

    def _run(self) -> None:
        try:
            if Path(self.source).suffix.lower() in IMAGE_SUFFIXES and not self.live:
                self._image()
            else:
                self._video()
        except Exception:  # noqa: BLE001 - a reader crash must end the run, not hang it
            log.exception("source %s: reader stopped", self.source)
        finally:
            self.box.close()

    def _image(self) -> None:
        import cv2
        img = cv2.imread(self.source, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"cannot read image {self.source}")
        self.box.put(Frame(img, time.time_ns(), time.perf_counter(), 1), wait=True)

    def _video(self) -> None:
        import cv2
        seq = 0
        warned = False
        while not self.stopping.is_set():
            cap = self._open()
            if not cap.isOpened():
                cap.release()
                if not self.live:
                    raise ValueError(f"cannot open video {self.source}")
                if not warned:
                    log.warning("source %s: not available yet, retrying every %.0f s", self.source, self.retry_s)
                    warned = True
                self.stopping.wait(self.retry_s)
                continue
            self.opens += 1
            self.fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            log.info("source %s: open (%dx%d, %.1f fps reported)", self.source, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                     int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), self.fps)
            warned = False
            while not self.stopping.is_set():
                ok, img = cap.read()
                if not ok or img is None:
                    break
                seq += 1
                if img.ndim == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                if not self.box.put(Frame(img, time.time_ns(), time.perf_counter(), seq), wait=not self.live):
                    break
            cap.release()
            if not self.live:
                return                                    # end of file
            if not self.stopping.is_set():
                log.warning("source %s: stream ended, reopening in %.0f s", self.source, self.retry_s)
                self.stopping.wait(self.retry_s)


def image_from_msg(msg) -> np.ndarray:
    """A ``sensor_msgs/Image`` (rgb8, bgr8, rgba8, bgra8, mono8) as an H x W x 3 BGR array."""
    enc = msg.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1, "8uc3": 3}.get(enc)
    if channels is None:
        raise ValueError(f"image encoding {msg.encoding!r} is not supported (rgb8, bgr8, rgba8, bgra8, mono8)")
    h, w = int(msg.height), int(msg.width)
    data = msg.data                                   # array.array('B') from rclpy, bytes when built by hand
    buf = np.asarray(data, np.uint8) if isinstance(data, list) else np.frombuffer(data, np.uint8)
    img = buf.reshape(h, int(msg.step))[:, : w * channels].reshape(h, w, channels)
    if enc in ("rgb8",):
        return np.ascontiguousarray(img[..., ::-1])
    if enc == "rgba8":
        return np.ascontiguousarray(img[..., 2::-1])
    if enc == "bgra8":
        return np.ascontiguousarray(img[..., :3])
    if channels == 1:
        return np.repeat(img, 3, axis=2)
    return np.ascontiguousarray(img)
