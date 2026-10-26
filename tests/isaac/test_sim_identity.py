"""The sonar inside the Isaac app (`sim`, RTX camera and bubble instancer running) gives the same pings as
`sonar-only` for the same config and seed: bit-identical arrays, images, stamps and labels.

Starts the Kit app: runs only with `python -m uw test --isaac` (about 2 min).
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

WS = Path(__file__).resolve().parents[2]
PINGS = 10


def _run(*args):
    r = subprocess.run([sys.executable, "-m", "uw.runner", *args], cwd=WS, capture_output=True, text=True,
                       timeout=1800)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    return r


@pytest.mark.isaac
def test_sim_pings_equal_sonar_only(tmp_path):
    if not os.environ.get("UW_IN_CONTAINER"):
        pytest.skip("runs in the Isaac container")
    _run("sim", "--sink", "file", "--out", str(tmp_path / "sim"), "--duration", str(PINGS / 10), "--no-realtime")
    _run("sonar-only", "--pings", str(PINGS), "--out", str(tmp_path / "so"))
    sim, so = tmp_path / "sim" / "sonar", tmp_path / "so" / "sonar"
    assert len(list(sim.glob("*.npz"))) == len(list(so.glob("*.npz"))) == PINGS
    assert len(list((tmp_path / "sim" / "camera").glob("*_rgb.png"))) >= PINGS - 3     # the camera ran too
    for i in range(PINGS):
        a, b = np.load(sim / f"{i:06d}.npz"), np.load(so / f"{i:06d}.npz")
        assert a.files == b.files
        for k in a.files:
            assert np.array_equal(a[k], b[k], equal_nan=True), (i, k)
        for png in ("polar", "mask_all", "mask_direct", "mask_geom"):
            assert (sim / f"{i:06d}_{png}.png").read_bytes() == (so / f"{i:06d}_{png}.png").read_bytes(), (i, png)
        ja, jb = (json.loads((d / f"{i:06d}.json").read_text()) for d in (sim, so))
        ja.pop("timing_ms"), jb.pop("timing_ms")
        assert ja == jb
