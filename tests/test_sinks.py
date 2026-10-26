"""SensorSink (host): the file backend writes a readable dataset; the others name their milestone."""
import json
import struct
import zlib

import numpy as np
import pytest

from uw import sinks


def _read_png(path):
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    w, h, depth, colour = struct.unpack(">IIBB", data[16:26])
    idat = data.index(b"IDAT")
    n = struct.unpack(">I", data[idat - 4:idat])[0]
    ch = {0: 1, 2: 3, 6: 4}[colour]
    raw = np.frombuffer(zlib.decompress(data[idat + 4:idat + 4 + n]), np.uint8).reshape(h, 1 + w * ch)
    assert depth == 8 and np.all(raw[:, 0] == 0)
    return raw[:, 1:].reshape(h, w, ch).squeeze()


def test_file_sink_writes_a_dataset(tmp_path):
    grey = (np.arange(12 * 7) % 256).astype(np.uint8).reshape(12, 7)
    rgb = np.random.default_rng(0).integers(0, 256, (5, 4, 3), dtype=np.uint8)
    with sinks.make_sink("file", out_dir=tmp_path / "ds", info={"name": "t"}) as sink:
        for seq in range(2):
            sink.publish(sinks.Message("sonar", seq, 0.1 * seq, arrays={"db": np.full((3, 2), seq, np.float32)},
                                       images={"polar": grey, "fan": rgb}, meta={"range": 1.5}))
    d = tmp_path / "ds" / "sonar"
    assert np.array_equal(_read_png(d / "000001_polar.png"), grey)
    assert np.array_equal(_read_png(d / "000001_fan.png"), rgb)
    assert np.all(np.load(d / "000001.npz")["db"] == 1)
    assert json.loads((d / "000001.json").read_text()) == {"seq": 1, "stamp": 0.1, "range": 1.5}
    manifest = json.loads((tmp_path / "ds" / "manifest.json").read_text())
    assert manifest["name"] == "t" and manifest["sensors"]["sonar"]["count"] == 2


def test_zenoh_sink_is_a_later_milestone():
    with pytest.raises(NotImplementedError, match="M7"):
        sinks.make_sink("zenoh")


def test_file_sink_ignores_time_and_frames(tmp_path):
    with sinks.make_sink("file", out_dir=tmp_path) as sink:
        sink.clock(1.0)
        sink.static_transforms(0.0, [("tank", "oculus", np.eye(3), (0.0, 0.0, 1.0))])
    assert not (tmp_path / "clock").exists()


def test_flat_layout_for_sweeps(tmp_path):
    grey = np.zeros((4, 3), np.uint8)
    with sinks.make_sink("file", out_dir=tmp_path, layout="flat") as sink:
        sink.publish(sinks.Message("sonar", 7, 0.7, arrays={"db": np.zeros(2)}, images={"polar": grey,
                                                                                      "mask_geom": grey}))
    names = sorted(p.name for p in tmp_path.iterdir())
    assert names == ["manifest.json", "ping_00007.json", "ping_00007.npz", "ping_00007.png",
                     "ping_00007_mask_geom.png"]

