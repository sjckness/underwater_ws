"""ros/src/yolo_detector without a model: the frame mailbox, the OpenCV reader on a file, the message builders.

Host only. Ultralytics is never imported; the message tests need ROS 2 Jazzy sourced (vision_msgs) and skip
otherwise, the reader test needs OpenCV.
"""
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "ros" / "src" / "yolo_detector"))
from yolo_detector import detector  # noqa: E402 - after sys.path
from yolo_detector.sources import Frame, Latest, VideoSource, image_from_msg, is_live  # noqa: E402


def _frame(seq):
    return Frame(np.full((4, 6, 3), seq, np.uint8), seq, time.perf_counter(), seq)


def test_a_live_source_overwrites_and_counts():
    box = Latest()
    for s in (1, 2, 3):
        assert box.put(_frame(s))
    assert box.take(0.01).seq == 3                     # the newest only
    assert (box.received, box.overwritten) == (3, 2)
    assert box.take(0.01) is None and not box.done
    box.close()
    assert box.done and not box.put(_frame(4))


def test_a_file_source_waits_for_each_frame():
    box = Latest()
    got = []

    def consumer():
        while not box.done:
            f = box.take(0.05)
            if f is not None:
                got.append(f.seq)
                time.sleep(0.005)                      # slower than the producer
    t = threading.Thread(target=consumer)
    t.start()
    for s in range(1, 21):
        box.put(_frame(s), wait=True)
    box.close()
    t.join(5)
    assert got == list(range(1, 21)) and box.overwritten == 0


def test_live_or_file():
    assert is_live("rtsp://127.0.0.1:8654/rov") and is_live("udp://0.0.0.0:5600") and is_live("0")
    assert not is_live("output/pool/x.mp4") and not is_live("/tmp/a.png")


def test_the_reader_plays_a_file_to_the_end(tmp_path):
    cv2 = pytest.importorskip("cv2")
    path = tmp_path / "clip.avi"
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (64, 48))
    if not w.isOpened():
        pytest.skip("this OpenCV cannot write MJPG")
    for i in range(12):
        w.write(np.full((48, 64, 3), 20 * i, np.uint8))
    w.release()
    box = Latest()
    src = VideoSource(str(path), box).start()
    seqs = []
    t0 = time.monotonic()
    while not box.done and time.monotonic() - t0 < 10:
        f = box.take(0.1)
        if f is not None:
            assert f.image.shape == (48, 64, 3) and f.image.dtype == np.uint8
            seqs.append(f.seq)
    src.stop()
    assert seqs == list(range(1, 13))


def test_a_missing_file_fails_at_once():
    with pytest.raises(FileNotFoundError):
        VideoSource("no/such/video.mp4", Latest())


@pytest.mark.parametrize("enc", ["rgb8", "bgr8", "rgba8", "bgra8", "mono8"])
def test_image_messages_become_bgr(enc):
    h, w = 3, 5
    bgr = np.random.default_rng(0).integers(0, 255, (h, w, 3), dtype=np.uint8)
    if enc == "mono8":
        src = bgr[..., :1]
        want = np.repeat(src, 3, axis=2)
    else:
        src = {"rgb8": bgr[..., ::-1], "bgr8": bgr}.get(enc)
        if src is None:
            alpha = np.full((h, w, 1), 255, np.uint8)
            src = np.concatenate([bgr[..., ::-1] if enc == "rgba8" else bgr, alpha], axis=2)
        want = bgr
    pad = 4                                           # a row stride wider than the pixels, as a driver may send
    rows = np.zeros((h, src.shape[1] * src.shape[2] + pad), np.uint8)
    rows[:, : src.shape[1] * src.shape[2]] = src.reshape(h, -1)
    msg = SimpleNamespace(encoding=enc, height=h, width=w, step=rows.shape[1], data=rows.tobytes())
    np.testing.assert_array_equal(image_from_msg(msg), want)


def test_unsupported_encodings_are_refused():
    msg = SimpleNamespace(encoding="16UC1", height=1, width=1, step=2, data=b"\0\0")
    with pytest.raises(ValueError, match="16UC1"):
        image_from_msg(msg)


def test_models_resolve_by_path_or_into_the_models_dir(tmp_path):
    w = tmp_path / "mine.pt"
    w.write_bytes(b"x")
    assert detector.resolve_model(str(w), tmp_path / "m") == w.resolve()
    assert detector.resolve_model("yolo26n.pt", tmp_path / "m") == tmp_path / "m" / "yolo26n.pt"
    with pytest.raises(FileNotFoundError):
        detector.resolve_model("runs/x/best.pt", tmp_path / "m")


def test_class_filters_take_names_and_indices():
    names = {0: "person", 1: "bicycle", 8: "boat"}
    assert detector.parse_classes("", names) is None
    assert detector.parse_classes("Person, 3,boat", names) == [0, 3, 8]
    with pytest.raises(SystemExit):
        detector.parse_classes("pipe", names)


def test_defaults_point_at_the_pool():
    """The default source is the pool's stream and the default camera_info its render camera."""
    from uw import config
    cfg = config.load(config.POOL_CONFIG)
    args = detector.parse_args([])
    assert args.source == cfg["stream"]["url"]
    assert args.hfov == cfg["camera"]["hfov_deg"]
    assert args.frame_id == "rov1/camera_front_optical"


def test_detection_array():
    pytest.importorskip("vision_msgs.msg", reason="host ROS 2 Jazzy with vision_msgs, sourced")
    from rclpy.serialization import deserialize_message, serialize_message
    from vision_msgs.msg import Detection2DArray

    from yolo_detector import msgs
    xyxy = [[10, 20, 50, 80], [100.5, 0, 101.5, 2]]
    out = msgs.detection_array(1_795_000_000_123_456_789, "rov1/camera_front_optical", xyxy, [0.9, 0.3], [8, 0],
                               {0: "person", 8: "boat"}, track_ids=[7, -1])
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (1_795_000_000, 123_456_789)
    a, b = out.detections
    assert (a.bbox.center.position.x, a.bbox.center.position.y, a.bbox.size_x, a.bbox.size_y) == (30, 50, 40, 60)
    assert a.results[0].hypothesis.class_id == "boat" and a.results[0].hypothesis.score == pytest.approx(0.9)
    assert (a.id, b.id) == ("7", "-1")
    assert b.results[0].hypothesis.class_id == "person" and b.bbox.size_x == pytest.approx(1.0)
    assert a.header.frame_id == "rov1/camera_front_optical"
    back = deserialize_message(serialize_message(out), Detection2DArray)
    assert len(back.detections) == 2 and back.detections[0].results[0].hypothesis.class_id == "boat"
    empty = msgs.detection_array(0, "f", np.zeros((0, 4)), [], [], ["x"])
    assert empty.detections == []
    assert msgs.detection_array(0, "f", [[0, 0, 1, 1]], [0.5], [0], ["x"]).detections[0].id == ""
    with pytest.raises(ValueError):
        msgs.detection_array(0, "f", [[0, 0, 1, 1]], [0.5, 0.6], [0], ["x"])


def test_camera_info_is_the_pool_render_camera():
    pytest.importorskip("sensor_msgs.msg", reason="host ROS 2 Jazzy, sourced")
    from yolo_detector import msgs
    info = msgs.camera_info(1280, 720, 80.0, 5, "cam")
    f = 640 / math.tan(math.radians(40))              # uw.pool_scene.camera_intrinsics, without pxr
    np.testing.assert_allclose(np.reshape(info.k, (3, 3)), [[f, 0, 640], [0, f, 360], [0, 0, 1]])
    np.testing.assert_allclose(np.reshape(info.p, (3, 4))[:, :3], np.reshape(info.k, (3, 3)))
    assert (info.width, info.height, info.header.frame_id) == (1280, 720, "cam")


def test_image_msg_round_trip():
    pytest.importorskip("sensor_msgs.msg", reason="host ROS 2 Jazzy, sourced")
    from yolo_detector import msgs
    img = np.random.default_rng(1).integers(0, 255, (7, 9, 3), dtype=np.uint8)
    m = msgs.image_msg(img, 1, "cam")
    assert (m.height, m.width, m.step, m.encoding) == (7, 9, 27, "bgr8")
    np.testing.assert_array_equal(image_from_msg(m), img)
    m = msgs.image_msg(np.ascontiguousarray(img[..., ::-1]), 1, "cam", "rgb8")
    np.testing.assert_array_equal(image_from_msg(m), img)
