"""Dataset messages from sonar pings (minimal, M2; sweeps come in M5).

Per ping (SensorSink "sonar"), compressed:
- <seq>.npz: polar_db (float16, dB re 1 uPa, row 0 = range 0, NaN in the blind zone), plume_share_all_u8 and
  plume_share_direct_u8 (bubbles' energy share, uint8 = round(255 min(share, 1))), range_m, bearing_deg;
- PNGs: polar (far range at the top), mask_all, mask_direct (the plume labels, share > sonar.plume_mask_threshold),
  fan if recorder.fan_png (or ``fan=True``: the ROS sink publishes it);
- <seq>.json: the leak's range/bearing in the sonar frame, the gas flow, the sonar pose, the config hash, the seed.
"""
from __future__ import annotations

import hashlib
import json

import numpy as np

from .config import get
from .sinks import Message

LABELS = {
    "mask_all": "bubbles' share of the echo energy over all their paths (direct and surface ghosts; it also covers "
                "the upper plume seen through the vertical sidelobes, in the plume's own shadow) > threshold",
    "mask_direct": "share of the bubbles' direct-path echo only > threshold",
}


def _u8(share):
    return np.round(255 * np.clip(share, 0.0, 1.0)).astype(np.uint8)


def config_hash(cfg: dict) -> str:
    return hashlib.sha256(json.dumps(cfg, sort_keys=True, default=str).encode()).hexdigest()[:16]


def leak_position(scene) -> np.ndarray | None:
    """Where the gas leaves: the hole, or the mean of the sediment vents of a buried leak."""
    leak = scene.leak
    if leak is None:
        return None
    return leak.vents.mean(0) if leak.buried else leak.hole


def sonar_message(seq: int, stamp: float, ping, son, cfg: dict, scene, sim=None, fan: bool | None = None) -> Message:
    son_cfg = get(cfg, "sonar")
    meta = {"sonar": {"position": [float(v) for v in son.position],
                      **{f"{k}_deg": son_cfg[f"{k}_deg"] for k in ("roll", "pitch", "yaw")},
                      "mode": son_cfg["mode"], "range": son.range, "focus": bool(son.focus),
                      "noise_floor_db": son.nf_db, "display_db": son.display,
                      "frequency_hz": son.frequency, "sound_speed": son.c, "beamwidth_h_deg": son.beamwidth_deg,
                      "fov_v_deg": son.fov_v_deg},
            "config_hash": config_hash(cfg), "seed": cfg["seed"],
            "plume_mask_threshold": son.mask_threshold,
            "plume_pixels": {"all": int(np.count_nonzero(ping.plume_mask_all)),
                             "direct": int(np.count_nonzero(ping.plume_mask_direct))},
            "timing_ms": {k: round(v, 3) for k, v in ping.timing_ms.items()}}
    pos = leak_position(scene)
    if pos is not None:
        r, b, e = son.to_sonar_frame(pos)[0]
        meta["leak"] = {"position": [float(v) for v in pos], "range_m": float(r), "bearing_deg": float(b),
                        "elevation_deg": float(e), "buried": bool(scene.leak.buried),
                        "flow_lpm": sim.source.q_gas * 6e4 if sim is not None else 0.0,
                        "mass_flow_g_s": sim.source.mdot * 1e3 if sim is not None else 0.0}
    images = {"polar": ping.image[::-1], "mask_all": ping.plume_mask_all[::-1],
              "mask_direct": ping.plume_mask_direct[::-1]}
    if get(cfg, "recorder.fan_png") if fan is None else fan:
        images["fan"] = son.fan(ping.image, width=int(get(cfg, "ros.fan_width")))
    return Message("sonar", seq, stamp,
                   arrays={"polar_db": ping.db.astype(np.float16), "plume_share_all_u8": _u8(ping.plume_share_all),
                           "plume_share_direct_u8": _u8(ping.plume_share_direct),
                           "range_m": ping.range_m.astype(np.float32), "bearing_deg": ping.bearing_deg.astype(np.float32)},
                   images=images, meta=meta)
