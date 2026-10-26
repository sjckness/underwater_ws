"""Dataset messages from sonar pings (minimal, M2; sweeps come in M5).

Per ping (SensorSink "sonar"): <seq>.npz with polar_db (float16, dB re 1 uPa, row 0 = range 0, NaN in the blind
zone), plume_share (float16; clipped at 1000, as destructive interference can push it above 1), range_m and
bearing_deg; PNGs polar (far range at the top), fan, mask (plume label: bubbles' energy share above
sonar.plume_mask_threshold); JSON with the leak's range/bearing in the sonar frame, the gas flow, the sonar pose, the
config hash and the seed.
"""
from __future__ import annotations

import hashlib
import json

import numpy as np

from .config import get
from .sinks import Message


def config_hash(cfg: dict) -> str:
    return hashlib.sha256(json.dumps(cfg, sort_keys=True, default=str).encode()).hexdigest()[:16]


def leak_position(scene) -> np.ndarray | None:
    """Where the gas leaves: the hole, or the mean of the sediment vents of a buried leak."""
    leak = scene.leak
    if leak is None:
        return None
    return leak.vents.mean(0) if leak.buried else leak.hole


def sonar_message(seq: int, stamp: float, ping, son, cfg: dict, scene, sim=None) -> Message:
    son_cfg = get(cfg, "sonar")
    meta = {"sonar": {"position": [float(v) for v in son.position], "rpy_deg": son_cfg["rpy_deg"],
                      "mode": son_cfg["mode"], "range": son.range, "focus": bool(son.focus),
                      "noise_floor_db": son.nf_db, "display_db": son.display},
            "config_hash": config_hash(cfg), "seed": cfg["seed"],
            "plume_mask_threshold": son.mask_threshold, "plume_pixels": int(np.count_nonzero(ping.plume_mask)),
            "timing_ms": {k: round(v, 3) for k, v in ping.timing_ms.items()}}
    pos = leak_position(scene)
    if pos is not None:
        r, b, e = son.to_sonar_frame(pos)[0]
        meta["leak"] = {"position": [float(v) for v in pos], "range_m": float(r), "bearing_deg": float(b),
                        "elevation_deg": float(e), "buried": bool(scene.leak.buried),
                        "flow_lpm": sim.source.q_gas * 6e4 if sim is not None else 0.0,
                        "mass_flow_g_s": sim.source.mdot * 1e3 if sim is not None else 0.0}
    return Message("sonar", seq, stamp,
                   arrays={"polar_db": ping.db.astype(np.float16),
                           "plume_share": np.clip(ping.plume_share, 0, 1e3).astype(np.float16),
                           "range_m": ping.range_m.astype(np.float32), "bearing_deg": ping.bearing_deg.astype(np.float32)},
                   images={"polar": ping.image[::-1], "fan": son.fan(ping.image), "mask": ping.plume_mask[::-1]},
                   meta=meta)
