"""The detector's ROS messages, built from plain arrays (pure: tested on the host without a ROS graph or a model).

Detections are ``vision_msgs/Detection2DArray`` in pixel coordinates of the image they were found in: bounding box
centre and size in pixels, ``theta`` 0, one hypothesis per box whose ``class_id`` is the model's class *name*
(``"person"``, ``"pipe"``), so a consumer does not need the model's index table. With tracking on, ``Detection2D.id``
is the tracker's id as a string, stable while the object stays in view; empty otherwise.
"""
from __future__ import annotations

import math

import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


def stamp_header(header, stamp_ns: int, frame_id: str) -> None:
    header.stamp.sec = int(stamp_ns // 1_000_000_000)
    header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    header.frame_id = frame_id


def detection_array(stamp_ns: int, frame_id: str, xyxy, scores, class_ids, names, track_ids=None) -> Detection2DArray:
    """N boxes ``xyxy`` (N x 4, pixels: x0, y0, x1, y1), ``scores`` (N), ``class_ids`` (N, indices into ``names``,
    a list or a dict as Ultralytics gives it) and optional ``track_ids`` (N, or None) as one message."""
    xyxy = np.asarray(xyxy, float).reshape(-1, 4)
    scores = np.asarray(scores, float).reshape(-1)
    class_ids = np.asarray(class_ids).reshape(-1).astype(int)
    if not len(xyxy) == len(scores) == len(class_ids):
        raise ValueError(f"{len(xyxy)} boxes, {len(scores)} scores, {len(class_ids)} classes")
    if track_ids is not None:
        track_ids = np.asarray(track_ids).reshape(-1).astype(int)
        if len(track_ids) != len(xyxy):
            raise ValueError(f"{len(xyxy)} boxes, {len(track_ids)} track ids")
    out = Detection2DArray()
    stamp_header(out.header, stamp_ns, frame_id)
    for i, (x0, y0, x1, y1) in enumerate(xyxy):
        det = Detection2D()
        det.header = out.header
        bbox = BoundingBox2D()
        bbox.center.position.x = float((x0 + x1) / 2.0)
        bbox.center.position.y = float((y0 + y1) / 2.0)
        bbox.center.theta = 0.0
        bbox.size_x = float(x1 - x0)
        bbox.size_y = float(y1 - y0)
        det.bbox = bbox
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(names[int(class_ids[i])])
        hyp.hypothesis.score = float(scores[i])
        det.results = [hyp]
        det.id = str(int(track_ids[i])) if track_ids is not None else ""
        out.detections.append(det)
    return out


def image_msg(img: np.ndarray, stamp_ns: int, frame_id: str, encoding: str = "bgr8") -> Image:
    """An H x W x 3 uint8 array as ``sensor_msgs/Image`` (no copy beyond the one into the message)."""
    if img.ndim != 3 or img.shape[2] != 3 or img.dtype != np.uint8:
        raise ValueError(f"expected H x W x 3 uint8, got {img.shape} {img.dtype}")
    msg = Image()
    stamp_header(msg.header, stamp_ns, frame_id)
    msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(img.shape[1] * 3)
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


def camera_info(width: int, height: int, hfov_deg: float, stamp_ns: int, frame_id: str) -> CameraInfo:
    """A distortion-free pinhole with square pixels and the principal point at the centre: the pool's render camera
    (``camera.hfov_deg`` in sim/config/pool/pool.yaml, the K of ``uw.pool_scene.camera_intrinsics``), not a
    calibration of the real one."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    cx, cy = width / 2.0, height / 2.0
    msg = CameraInfo()
    stamp_header(msg.header, stamp_ns, frame_id)
    msg.width, msg.height = int(width), int(height)
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg
