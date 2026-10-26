"""Where sensor outputs go (plan: SensorSink = file | ros | zenoh; later oculus_emulator).

A sensor produces ``Message``s; a sink delivers them. ``file`` writes a dataset; ``ros`` publishes live through the
Isaac ROS bridge's internal Jazzy (uw/ros_bridge.py); ``zenoh`` (M7) will.
"""
from __future__ import annotations

import json
import struct
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Message:
    sensor: str                                     # e.g. "sonar"
    seq: int
    stamp: float                                    # sim time [s]
    arrays: dict = field(default_factory=dict)      # name -> ndarray: raw data, saved together as <seq>.npz
    images: dict = field(default_factory=dict)      # name -> uint8 (H, W) or (H, W, 3): one PNG each
    meta: dict = field(default_factory=dict)        # JSON-serialisable


class SensorSink:
    def publish(self, msg: Message) -> None:
        raise NotImplementedError

    def clock(self, t: float) -> None:
        """Sim time; only live sinks use it."""

    def static_transforms(self, t: float, transforms: list) -> None:
        """Fixed frames [(parent, child, rotation 3x3, translation)]; only live sinks use them."""

    def transforms(self, t: float, transforms: list) -> None:
        """Moving frames (a mobile sonar), same form; only live sinks use them."""

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class NullSink(SensorSink):
    """Delivers nothing (e.g. a GUI session without ROS)."""

    def publish(self, msg: Message) -> None:
        pass


class FileSink(SensorSink):
    """A dataset on disk: <out>/<sensor>/<seq>.npz, <seq>_<image>.png, <seq>.json; manifest.json on close."""

    def __init__(self, out_dir, info: dict | None = None):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.info = info or {}
        self.index: dict[str, list] = {}
        self.t0 = time.time()

    def publish(self, msg: Message) -> None:
        d = self.out / msg.sensor
        d.mkdir(exist_ok=True)
        stem = f"{msg.seq:06d}"
        files = []
        if msg.arrays:
            np.savez_compressed(d / f"{stem}.npz", **msg.arrays)
            files.append(f"{stem}.npz")
        for name, img in msg.images.items():
            write_png(d / f"{stem}_{name}.png", img)
            files.append(f"{stem}_{name}.png")
        (d / f"{stem}.json").write_text(json.dumps({"seq": msg.seq, "stamp": msg.stamp, **msg.meta}, indent=1))
        files.append(f"{stem}.json")
        self.index.setdefault(msg.sensor, []).append({"seq": msg.seq, "stamp": msg.stamp, "files": files})

    def close(self) -> None:
        manifest = {**self.info, "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "wall_clock_s": time.time() - self.t0,
                    "sensors": {s: {"count": len(v), "messages": v} for s, v in self.index.items()}}
        (self.out / "manifest.json").write_text(json.dumps(manifest, indent=1))


def make_sink(kind: str, **kwargs) -> SensorSink:
    if kind == "file":
        return FileSink(**kwargs)
    if kind == "none":
        return NullSink()
    if kind == "ros":
        from .ros_bridge import RosSink
        return RosSink(**kwargs)
    if kind == "zenoh":
        raise NotImplementedError("SensorSink 'zenoh' is milestone M7")
    if kind == "oculus_emulator":
        raise NotImplementedError("SensorSink 'oculus_emulator' is not scheduled yet (after M7)")
    raise ValueError(f"unknown SensorSink {kind!r}: file | ros | none | zenoh | oculus_emulator")


def write_png(path, img: np.ndarray, level: int = 3) -> None:
    """8-bit grey (H, W), RGB (H, W, 3) or RGBA (H, W, 4) PNG, standard library only."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape[:2]
    colour = 0 if img.ndim == 2 else {3: 2, 4: 6}[img.shape[2]]          # PNG colour type: grey, RGB, RGBA
    rows = np.concatenate([np.zeros((h, 1), np.uint8), img.reshape(h, -1)], 1)     # filter byte 0 per row

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, colour, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(rows.tobytes(), level)) + chunk(b"IEND", b""))
