"""YOLO on the ROV's camera: ``vision_msgs/Detection2DArray`` on ROS 2, and an annotated overlay to look at.

    ros/yolo_detector/run.sh                                   # the pool's stream, COCO yolo26n, ROS topics
    ros/yolo_detector/run.sh --video-out rtsp://127.0.0.1:8654/rov_yolo     # + the overlay, http://localhost:8889/rov_yolo
    ros/yolo_detector/run.sh --show --no-ros --source output/pool/snapshots/x.png   # one image, a window, no ROS

The frames come from the same place a pilot's video comes from: the H.264 stream that ``python -m uw pool`` publishes
to mediamtx (``rtsp://127.0.0.1:8654/rov``) and BlueOS relays to Cockpit, decoded here by OpenCV/FFmpeg. On the
real BlueROV2 the source is BlueOS's own RTSP stream instead, and nothing else changes: the detector sees the
encoder's artefacts and the network's delay either way, and the simulator needs no ROS for it. ``--image-topic``
takes a ``sensor_msgs/Image`` topic instead (the tank sim's ``/camera/image_raw``); any OpenCV source works too
(a video file, an image, a camera index).

Outputs, all optional except the detections:
- ``/<vehicle>/perception/camera/detections``: ``vision_msgs/Detection2DArray``, pixel boxes, ``class_id`` = the class
  name (yolo_detector/msgs.py). RELIABLE depth 10 (class R of docs/topic_map.md §3).
- ``/<vehicle>/perception/camera/overlay``: the frame with boxes, masks and a status line, ``sensor_msgs/Image``
  bgr8, sensor QoS (``--overlay-hz``, ``--no-overlay``). ``rqt_image_view`` shows it.
- ``--publish-camera``: the decoded frames as ``/<vehicle>/sensors/camera_front/image_raw`` (rgb8) and
  ``camera_info`` (a pinhole from ``--hfov``), stamped like the detections: the camera driver's part, so a bag of
  the two lines up.
- ``--video-out URL|FILE``: the overlay as H.264 (``uw.video.VideoStream``: the same encoder settings as the pool's
  stream), to mediamtx for a browser (``http://localhost:8889/<path>``) or as a second "Redirect" stream in BlueOS
  for Cockpit; a ``.ts``/``.mp4`` path writes a file. Sent at a fixed rate, the latest overlay repeated when the
  model is slower than the camera.
- ``--show``: an OpenCV window (``q`` quits).

The stamp of everything is the moment the frame was decoded here (wall clock, which is the pool's time: it runs in
real time and has no ``/clock``), or the source message's stamp with ``--image-topic``. The measured latency runs
from that moment until the detections and the overlay are out, so it leaves out the sim's encoder and the stream.

The model is any Ultralytics model with boxes (detect, segment, pose; ``.pt``, ``.onnx``, ``.engine``). A bare name
is looked up in, and downloaded by Ultralytics into, ``assets/models/`` (``UW_YOLO_MODELS``). The COCO models see
nothing in the pool, which holds no COCO object; ``--prompts "pipe,concrete block"`` with a YOLOE model
(``yoloe-26s-seg.pt``) detects by text with no training, and needs Ultralytics' CLIP (requirements.txt). Ultralytics'
automatic pip installs are off (``YOLO_AUTOINSTALL``): a missing package is an error, not an install mid-run.

The domain is ``UW_ROS_DOMAIN_ID`` (default 73), as for the other host nodes; ``ROS_DOMAIN_ID`` is ignored on purpose.
"""
from __future__ import annotations

import argparse
import collections
import contextlib
import json
import logging
import os
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

if __package__ in (None, ""):                       # run as a script: python3 ros/yolo_detector/yolo_detector/detector.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yolo_detector.sources import Frame, Latest, VideoSource, image_from_msg  # noqa: E402

log = logging.getLogger("yolo_detector")

#: the repo root when this file is in the repo (ros/yolo_detector/yolo_detector/detector.py), else None (an install)
_WS = Path(__file__).resolve().parents[3]
WS = _WS if (_WS / "uw").is_dir() else None
DEFAULT_SOURCE = "rtsp://127.0.0.1:8654/rov"
DEFAULT_MODEL = "yolo26n.pt"
CLIP_HINT = "git+https://github.com/ultralytics/CLIP.git"


def models_dir() -> Path:
    env = os.environ.get("UW_YOLO_MODELS")
    if env:
        return Path(env).expanduser()
    return WS / "assets" / "models" if WS is not None else Path.home() / ".cache" / "uw_yolo" / "models"


def resolve_model(name: str, mdir: Path) -> Path:
    """A path that exists, as is; a bare file name, in ``mdir`` (where Ultralytics downloads it if it knows it)."""
    p = Path(name).expanduser()
    if p.exists():
        return p.resolve()
    if p.parent == Path("."):
        return mdir / p.name
    raise FileNotFoundError(f"model {name}: no such file")


def parse_classes(spec: str, names: dict) -> list[int] | None:
    """``"person,3,boat"`` -> class indices; names are matched case-insensitively against the model's."""
    if not spec:
        return None
    by_name = {str(v).lower(): int(k) for k, v in names.items()}
    out = []
    for tok in (t.strip() for t in spec.split(",") if t.strip()):
        if tok.isdigit():
            out.append(int(tok))
        elif tok.lower() in by_name:
            out.append(by_name[tok.lower()])
        else:
            raise SystemExit(f"--classes: {tok!r} is not a class of this model ({len(names)} classes)")
    return out


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="yolo_detector", description=__doc__.splitlines()[0],
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    src = ap.add_argument_group("input")
    src.add_argument("--source", default=DEFAULT_SOURCE,
                     help="RTSP/UDP/HTTP URL, video or image file, or camera index (the pool's stream by default)")
    src.add_argument("--image-topic", default="", help="a sensor_msgs/Image topic instead of --source")
    m = ap.add_argument_group("model")
    m.add_argument("--model", default=DEFAULT_MODEL, help="weights: a path, or a bare name in UW_YOLO_MODELS")
    m.add_argument("--prompts", default="", help="YOLOE text prompts, comma-separated: the classes to look for")
    m.add_argument("--conf", type=float, default=0.25, help="minimum score")
    m.add_argument("--iou", type=float, default=0.7, help="NMS IoU threshold")
    m.add_argument("--imgsz", type=int, default=640, help="inference size [px]")
    m.add_argument("--max-det", type=int, default=50, help="at most this many detections per frame")
    m.add_argument("--classes", default="", help="keep only these classes: names or indices, comma-separated")
    m.add_argument("--device", default="", help="cuda:0, cpu, ...; empty: the first GPU if there is one")
    m.add_argument("--half", action="store_true", help="FP16 inference (GPU)")
    m.add_argument("--track", action="store_true", help="track across frames: Detection2D.id = the track id")
    m.add_argument("--tracker", default="bytetrack.yaml", help="Ultralytics tracker config (with --track)")
    o = ap.add_argument_group("output")
    o.add_argument("--no-ros", action="store_true", help="no ROS at all (with --show or --video-out)")
    o.add_argument("--vehicle", default="rov1", help="topic namespace and frame prefix")
    o.add_argument("--frame-id", default="", help="header frame (default <vehicle>/camera_front_optical)")
    o.add_argument("--no-overlay", action="store_true", help="do not publish the overlay image topic")
    o.add_argument("--overlay-hz", type=float, default=0.0, help="overlay topic rate cap; 0: every processed frame")
    o.add_argument("--publish-camera", action="store_true", help="also publish the decoded frames and camera_info")
    o.add_argument("--hfov", type=float, default=80.0, help="horizontal FOV for camera_info [deg] (pool: 80)")
    o.add_argument("--video-out", default="", help="the overlay as H.264: rtsp://host:8654/path, or a .ts/.mp4 file")
    o.add_argument("--video-fps", type=float, default=0.0, help="--video-out rate; 0: the source's, else 30")
    o.add_argument("--show", action="store_true", help="show the overlay in a window (q quits)")
    r = ap.add_argument_group("run")
    r.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (0: until stopped)")
    r.add_argument("--stats-every", type=float, default=10.0, help="log rates and timings every N seconds")
    r.add_argument("--stats-file", default="", help="write a JSON summary here on exit")
    args, ros_args = ap.parse_known_args(argv)
    args.ros_args = ros_args
    args.frame_id = args.frame_id or f"{args.vehicle}/camera_front_optical"
    return args


def load_model(args):
    """The Ultralytics model, loaded (and downloaded, and its text prompts encoded) inside the models directory, so
    nothing Ultralytics fetches by bare name lands in the current directory."""
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    from ultralytics import YOLO

    mdir = models_dir()
    mdir.mkdir(parents=True, exist_ok=True)
    path = resolve_model(args.model, mdir)
    with contextlib.chdir(mdir):
        model = YOLO(str(path))
        prompts = [p.strip() for p in args.prompts.split(",") if p.strip()]
        if prompts:
            if not hasattr(model, "set_classes"):
                raise SystemExit(f"--prompts needs an open-vocabulary model (YOLOE, YOLO-World), not {path.name}")
            try:
                model.set_classes(prompts)
            except ModuleNotFoundError as exc:
                raise SystemExit(f"--prompts: {exc}. Text prompts need Ultralytics' CLIP: "
                                 f"{sys.executable} -m pip install {CLIP_HINT}") from exc
    if model.task not in ("detect", "segment", "pose"):
        raise SystemExit(f"{path.name} is a {model.task} model: the detector needs boxes (detect, segment, pose)")
    return model, path


def _uw_video():
    """``uw.video`` from this checkout (the pool's encoder settings) and the system's ffmpeg."""
    import shutil
    if WS is None:
        raise SystemExit("--video-out uses uw.video from the underwater_ws checkout; run from the repo")
    if str(WS) not in sys.path:
        sys.path.append(str(WS))                      # appended: the workspace must not shadow anything installed
    from uw import video
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("--video-out needs ffmpeg on PATH")
    return video, ffmpeg


class Pacer:
    """A live source's overlay as H.264 in real time: the latest overlay goes to a ``uw.video.VideoStream`` at a fixed
    rate, repeated while the model is slower than the camera. The encoder is told a frame rate, and a stream that
    delivers fewer frames than it declares plays back in slow motion."""

    def __init__(self, url: str, fps: float):
        self.url, self.fps = url, float(fps)
        self.stream = None
        self.frame = None
        self.lock = threading.Lock()
        self.stop_ev = threading.Event()
        self.thread = threading.Thread(target=self._run, name="yolo-video-out", daemon=True)

    def set(self, bgr: np.ndarray) -> None:
        with self.lock:
            if self.stream is None:
                video, ffmpeg = _uw_video()
                self.stream = video.VideoStream(self.url, bgr.shape[1], bgr.shape[0], self.fps, ffmpeg=ffmpeg,
                                                threads=2).start()
                self.thread.start()
            self.frame = bgr

    def _run(self) -> None:
        period = 1.0 / self.fps
        nxt = time.perf_counter()
        while not self.stop_ev.is_set():
            with self.lock:
                f = self.frame
            if f is not None:
                self.stream.push(np.ascontiguousarray(f[..., ::-1]))
            nxt += period
            delay = nxt - time.perf_counter()
            if delay > 0:
                self.stop_ev.wait(delay)
            elif delay < -period:
                nxt = time.perf_counter()

    def close(self) -> dict | None:
        self.stop_ev.set()
        if self.thread.is_alive():
            self.thread.join(2.0)
        if self.stream is None:
            return None
        self.stream.close()
        return self.stream.stats()


class FrameWriter:
    """A file source's overlay, every frame in order, blocking: an offline run's video has all its frames at the
    source's rate, however fast the model went. Same encoder settings as ``uw.video``."""

    def __init__(self, url: str, fps: float):
        self.url, self.fps = url, float(fps)
        self.proc = None
        self.written = 0

    def set(self, bgr: np.ndarray) -> None:
        import subprocess
        if self.proc is None:
            video, ffmpeg = _uw_video()
            cmd = video.ffmpeg_args(ffmpeg, self.url, bgr.shape[1], bgr.shape[0], self.fps)
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
        self.proc.stdin.write(np.ascontiguousarray(bgr[..., ::-1]).tobytes())
        self.written += 1

    def close(self) -> dict | None:
        if self.proc is None:
            return None
        self.proc.stdin.close()
        rc = self.proc.wait(30)
        return {"written": self.written, "ffmpeg_exit": rc}


class Ros:
    """The ROS side: publishers, the optional image subscription, and an executor in its own thread."""

    def __init__(self, args, box: Latest):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                               qos_profile_sensor_data)
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import CameraInfo, Image
        from vision_msgs.msg import Detection2DArray

        from yolo_detector import msgs

        self.msgs, self.args, self.box = msgs, args, box
        rclpy.init(args=args.ros_args, domain_id=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")),
                   signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy = rclpy
        self.node = rclpy.create_node("yolo_detector")
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        base = f"/{args.vehicle}"
        self.pub_det = self.node.create_publisher(Detection2DArray, f"{base}/perception/camera/detections", reliable)
        self.pub_overlay = None if args.no_overlay else self.node.create_publisher(
            Image, f"{base}/perception/camera/overlay", qos_profile_sensor_data)
        self.pub_image = self.pub_info = None
        if args.publish_camera:
            self.pub_image = self.node.create_publisher(Image, f"{base}/sensors/camera_front/image_raw",
                                                        qos_profile_sensor_data)
            self.pub_info = self.node.create_publisher(CameraInfo, f"{base}/sensors/camera_front/camera_info",
                                                       qos_profile_sensor_data)
        self.seq = 0
        if args.image_topic:
            self.node.create_subscription(Image, args.image_topic, self._on_image, qos_profile_sensor_data)
        self.last_overlay = 0.0
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name="yolo-ros", daemon=True)
        self.thread.start()
        log.info("ROS domain %s: detections on %s%s", os.environ.get("UW_ROS_DOMAIN_ID", "73"),
                 self.pub_det.topic_name, f", overlay on {self.pub_overlay.topic_name}" if self.pub_overlay else "")

    def _on_image(self, msg) -> None:
        self.seq += 1
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        try:
            img = image_from_msg(msg)
        except ValueError as exc:
            self.node.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        self.box.put(Frame(img, stamp, time.perf_counter(), self.seq))

    def publish(self, frame: Frame, det, overlay) -> None:
        m, a = self.msgs, self.args
        self.pub_det.publish(det)
        if self.pub_image is not None:
            h, w = frame.image.shape[:2]
            self.pub_image.publish(m.image_msg(np.ascontiguousarray(frame.image[..., ::-1]), frame.stamp_ns,
                                               a.frame_id, "rgb8"))
            self.pub_info.publish(m.camera_info(w, h, a.hfov, frame.stamp_ns, a.frame_id))
        if self.pub_overlay is not None and overlay is not None:
            now = time.perf_counter()
            if a.overlay_hz <= 0 or now - self.last_overlay >= 1.0 / a.overlay_hz:
                self.last_overlay = now
                self.pub_overlay.publish(m.image_msg(overlay, frame.stamp_ns, a.frame_id))

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def draw_status(img: np.ndarray, text: str) -> None:
    """One status line in the bottom-left corner, white on a dark band (labels are drawn at the top of boxes)."""
    import cv2
    font, scale, th = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
    (tw, tht), base = cv2.getTextSize(text, font, scale, th)
    y = img.shape[0] - 8
    cv2.rectangle(img, (0, y - tht - 8), (tw + 16, img.shape[0]), (32, 32, 32), -1)
    cv2.putText(img, text, (8, y - 2), font, scale, (255, 255, 255), th, cv2.LINE_AA)


def _summ(v: list[float]) -> dict:
    return {"mean": round(float(np.mean(v)), 2), "p95": round(float(np.percentile(v, 95)), 2)} if v else {}


def run(args) -> int:  # noqa: C901 - one flat loop, like uw/pool.py
    if args.no_ros and not (args.show or args.video_out):
        log.warning("--no-ros without --show or --video-out: the detections go nowhere but the log")
    model, path = load_model(args)
    names = model.names
    classes = parse_classes(args.classes, names)
    kw = {"conf": args.conf, "iou": args.iou, "imgsz": args.imgsz, "max_det": args.max_det, "classes": classes,
          "half": args.half, "verbose": False}
    if args.device:
        kw["device"] = args.device
    t0 = time.perf_counter()
    model.predict(np.zeros((args.imgsz, args.imgsz, 3), np.uint8), **kw)        # CUDA init, kernel autotuning
    log.info("model %s: %s, %d classes%s, device %s, warm-up %.1f s", path.name, model.task, len(names),
             f" ({', '.join(str(v) for v in names.values())})" if len(names) <= 12 else "",
             getattr(model.predictor, "device", "?") if getattr(model, "predictor", None) else "?",
             time.perf_counter() - t0)

    box = Latest()
    stop = {"flag": False}

    def _sig(_s, _f):
        stop["flag"] = True
        box.close()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    ros = None if args.no_ros else Ros(args, box)
    source = None
    if not args.image_topic:
        source = VideoSource(args.source, box).start()
        log.info("source %s", args.source)
    elif ros is None:
        raise SystemExit("--image-topic needs ROS (drop --no-ros)")
    vout = None
    show = args.show
    infer = model.track if args.track else model.predict
    if args.track:
        kw.update(persist=True, tracker=args.tracker)

    count = {"processed": 0, "detections": 0}
    times = {"inference": [], "latency": []}
    per_class: collections.Counter = collections.Counter()
    wall0 = last_t = time.perf_counter()
    last = {"processed": 0, "received": 0, "overwritten": 0, "detections": 0}
    period_class: collections.Counter = collections.Counter()
    fps_ema = prev_t = None
    exit_reason = "stopped"
    try:
        while not stop["flag"]:
            frame = box.take(0.5)
            now = time.perf_counter()
            if frame is not None:
                t_i = time.perf_counter()
                res = infer(frame.image, **kw)[0]
                inf_ms = (time.perf_counter() - t_i) * 1e3
                b = res.boxes
                xyxy = b.xyxy.cpu().numpy() if len(b) else np.zeros((0, 4))
                conf = b.conf.cpu().numpy() if len(b) else np.zeros(0)
                cls = b.cls.cpu().numpy().astype(int) if len(b) else np.zeros(0, int)
                ids = b.id.cpu().numpy().astype(int) if (args.track and b.id is not None) else None
                if args.track and ids is None and len(b):
                    ids = np.full(len(b), -1)              # boxes the tracker has not confirmed yet
                if prev_t is not None and now > prev_t:
                    fps_ema = 1.0 / (now - prev_t) if fps_ema is None else 0.9 * fps_ema + 0.1 / (now - prev_t)
                prev_t = now
                overlay = None
                if show or args.video_out or (ros is not None and ros.pub_overlay is not None):
                    overlay = res.plot(line_width=2)
                    draw_status(overlay, f"{path.stem}  {fps_ema or 0:.0f} fps  {inf_ms:.0f} ms  {len(b)} det")
                if ros is not None:
                    det = ros.msgs.detection_array(frame.stamp_ns, args.frame_id, xyxy, conf, cls, names, ids)
                    ros.publish(frame, det, overlay)
                times["latency"].append((time.perf_counter() - frame.t_got) * 1e3)
                times["inference"].append(inf_ms)
                count["processed"] += 1
                count["detections"] += len(b)
                for c in cls:
                    per_class[str(names[int(c)])] += 1
                    period_class[str(names[int(c)])] += 1
                if args.video_out and overlay is not None:
                    if vout is None:
                        fps = args.video_fps or (source.fps if source is not None and 1.0 <= source.fps <= 120.0
                                                 else 30.0)  # RTSP may report 0 or a clock rate
                        live = source is None or source.live
                        vout = (Pacer if live else FrameWriter)(args.video_out, fps)
                        log.info("overlay video -> %s at %.0f fps (%s)", args.video_out, fps,
                                 "real time" if live else "every frame")
                    vout.set(overlay)
                if show:
                    import cv2
                    try:
                        cv2.imshow("yolo_detector", overlay)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            exit_reason = "q"
                            break
                    except cv2.error as exc:
                        log.warning("--show: no window (%s); continuing without it", str(exc).strip()[:120])
                        show = False
            elif box.done:
                exit_reason = "end of source"
                break
            now = time.perf_counter()
            if now - last_t >= args.stats_every:
                span = now - last_t
                n = count["processed"] - last["processed"]
                inf, lat = times["inference"][-n:] if n else [], times["latency"][-n:] if n else []
                log.info("in %.1f fps, processed %.1f fps (%d overwritten), inference %s ms, latency %s ms, "
                         "%.2f det/frame%s", (box.received - last["received"]) / span, n / span,
                         box.overwritten - last["overwritten"],
                         f"{np.mean(inf):.1f} (p95 {np.percentile(inf, 95):.1f})" if inf else "-",
                         f"{np.mean(lat):.1f} (p95 {np.percentile(lat, 95):.1f})" if lat else "-",
                         (count["detections"] - last["detections"]) / n if n else 0.0,
                         (": " + ", ".join(f"{k} {v}" for k, v in period_class.most_common(6))) if period_class else "")
                last = {"processed": count["processed"], "received": box.received, "overwritten": box.overwritten,
                        "detections": count["detections"]}
                period_class.clear()
                last_t = now
            if args.duration and now - wall0 >= args.duration:
                exit_reason = "duration"
                break
    finally:
        wall = time.perf_counter() - wall0
        if source is not None:
            source.stop()
        video = vout.close() if vout is not None else None
        if ros is not None:
            ros.close()
        if args.show:
            with contextlib.suppress(Exception):
                import cv2
                cv2.destroyAllWindows()
        summary = {"exit_reason": exit_reason, "wall_s": round(wall, 2), "model": str(path), "task": model.task,
                   "source": args.image_topic or args.source, "received": box.received,
                   "overwritten": box.overwritten, **count,
                   "processed_fps": round(count["processed"] / wall, 2) if wall > 0 else 0.0,
                   "inference_ms": _summ(times["inference"]), "latency_ms": _summ(times["latency"]),
                   "per_class": dict(per_class.most_common()), "video": video}
        log.info("stopped (%s): %d frames processed of %d received in %.1f s, %d detections%s", exit_reason,
                 count["processed"], box.received, wall, count["detections"],
                 (": " + ", ".join(f"{k} {v}" for k, v in per_class.most_common(8))) if per_class else "")
        if args.stats_file:
            Path(args.stats_file).write_text(json.dumps(summary, indent=1, default=str))
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
