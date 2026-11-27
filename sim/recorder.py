"""Dataset messages from sonar pings (minimal, M2; sweeps come in M5).

Per ping (SensorSink "sonar"), compressed:
- <seq>.npz: polar_db (float16, dB re 1 uPa, row 0 = range 0, NaN in the blind zone), plume_share_all_u8 and
  plume_share_direct_u8 (bubbles' energy share, uint8 = round(255 min(share, 1))), range_m, bearing_deg;
- PNGs: polar (far range at the top), the plume labels mask_all, mask_direct (energy share > sonar.plume_mask_threshold)
  and mask_geom (geometric; LABELS, also in the manifest), fan if recorder.fan_png (or ``fan=True``: the ROS sink
  publishes it);
- <seq>.json: the sonar's pose at this ping (position, roll/pitch/yaw, rotation), the leak's range/bearing/elevation
  in that pose's frame, the gas flow, the config hash, the seed.
"""
from __future__ import annotations

import functools
import hashlib
import json
import os
import subprocess
from pathlib import Path

import numpy as np

from .config import get
from .geometry import rpy_deg_of
from .sinks import Message

LABELS = {
    "mask_all": "bubbles' share of the echo energy over all their paths (direct and surface ghosts; it also covers "
                "the upper plume seen through the vertical sidelobes, in the plume's own shadow) > threshold",
    "mask_direct": "share of the bubbles' direct-path echo only > threshold (energy based: includes their leakage "
                   "through the array's sidelobes, e.g. arcs across the fan from a strong plume echo)",
    "mask_geom": "geometric: cells whose main lobe contains an alive bubble in line of sight from the sonar (range "
                 "bins of its direct echo; beams within half the -3 dB beam width in u; |elevation| <= fov_v / 2); "
                 "independent of echo levels, so no sidelobe leakage, and also bubbles too weak to see",
}


def _u8(share):
    return np.round(255 * np.clip(share, 0.0, 1.0)).astype(np.uint8)


def sonar_frame(pts, position, rot) -> np.ndarray:
    """World points -> (range, bearing [deg, + starboard], elevation [deg]) for a sonar pose."""
    rel = (np.atleast_2d(pts) - np.asarray(position)) @ np.asarray(rot)
    r = np.linalg.norm(rel, axis=1)
    return np.stack([r, np.degrees(np.arctan2(-rel[:, 1], rel[:, 0])), np.degrees(np.arcsin(rel[:, 2] / r))], 1)


def config_hash(cfg: dict) -> str:
    return hashlib.sha256(json.dumps(cfg, sort_keys=True, default=str).encode()).hexdigest()[:16]


@functools.lru_cache(maxsize=1)
def git_version() -> str:
    """Short HEAD, plus ``+dirty`` when tracked files differ from it. Cached: it costs two subprocesses and it
    cannot change while a dataset is being written, whereas it would otherwise be paid per ping.

    ``UW_GIT_VERSION`` wins when set, because pings are generated **inside the Isaac container, which has no
    git binary** -- `.git` is mounted but unreadable without one, so asking here returns "unknown" and every
    containerised dataset would record no provenance at all. `uw/launcher.py` reads it on the host, where the
    repository is, and passes it in. Measured: without this, a `sonar-only` run stamps
    `m750d-1/unknown/<hash>`.
    """
    env = os.environ.get("UW_GIT_VERSION", "").strip()
    if env:
        return env
    ws = Path(__file__).resolve().parents[1]
    try:
        h = subprocess.run(["git", "-C", str(ws), "rev-parse", "--short", "HEAD"], capture_output=True,
                           text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(ws), "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True).stdout.strip()
        return (h + ("+dirty" if dirty else "")) or "unknown"
    except OSError:
        return "unknown"


def generator_version(model_version: str, cfg: dict) -> str:
    """What generated a ping, in one string: ``<sonar model>/<git>/<config hash>``.

    Joined with ``/`` rather than ``+`` because `git_version` appends ``+dirty``, which would make a
    ``+``-joined string impossible to split back into its three parts. None of the parts can contain a slash.

    Each part answers a question the other two cannot. The model version says whether the acoustic model
    changed; the git hash pins the whole tree, including the parts of the pipeline outside `uw/sonar.py`; the
    config hash distinguishes two datasets from identical code with different parameters. A dataset is
    traceable to its generator only if all three are recorded, which is why this goes into every ping rather
    than into the sweep manifest alone -- a ping that gets copied, converted or recompressed keeps it.
    """
    return f"{model_version}/{git_version()}/{config_hash(cfg)}"


def leak_position(scene) -> np.ndarray | None:
    """Where the gas leaves: the hole, or the mean of the sediment vents of a buried leak."""
    leak = scene.leak
    if leak is None:
        return None
    return leak.vents.mean(0) if leak.buried else leak.hole


def dvl_message(report, seq: int, cfg: dict, instrument: dict | None = None) -> Message:
    """One DVL velocity-and-transducer report as a sink Message. Pure: no ROS, no Isaac, no scene.

    ``meta["dvl"]`` is the device's own JSON report, which is exactly what ``/sim/dvl/report`` carries and
    what ``ros/dvl_relay`` consumes, so the recorded stream and the live one are the same bytes. Nothing
    here is ground truth: the report has already been through the noise, the bottom-lock gate and the
    quantisation, which is what makes it a measurement rather than the answer (docs/ros_hil.md 2.5).
    """
    return Message("dvl", seq, report.stamp,
                   arrays={"beam_unit_vec": report.beam_unit_vec.astype(np.float32),
                           "beam_range": report.beam_range.astype(np.float32),
                           "beam_velocity": report.beam_velocity.astype(np.float32),
                           "rssi_dbm": report.rssi_dbm.astype(np.float32)},
                   meta={"dvl": report.as_json(), "instrument": instrument or {},
                         "num_good_beams": report.num_good_beams,
                         "config_hash": config_hash(cfg), "seed": cfg["seed"]})


def sonar_message(seq: int, stamp: float, ping, son, cfg: dict, scene, sim=None, fan: bool | None = None) -> Message:
    son_cfg = get(cfg, "sonar")
    rpy = rpy_deg_of(ping.rot)
    meta = {"sonar": {"position": [round(float(v), 6) for v in ping.position],
                      **{f"{k}_deg": round(float(a), 4) for k, a in zip(("roll", "pitch", "yaw"), rpy)},
                      "rotation": [[round(float(v), 7) for v in row] for row in ping.rot],
                      "pose_source": son_cfg["pose_source"], "mode": son_cfg["mode"], "range": son.range, "focus": bool(son.focus),
                      "noise_floor_db": son.nf_db, "display_db": son.display,
                      # the gain chain, without which the 8-bit polar image is not invertible (.../sonar_fls/meta)
                      "tvg_db_per_decade": son.tvg_k, "absorption_db_per_m": son.alpha_db,
                      "frequency_hz": son.frequency, "sound_speed": son.c, "beamwidth_h_deg": son.beamwidth_deg,
                      "fov_v_deg": son.fov_v_deg},
            "config_hash": config_hash(cfg), "seed": cfg["seed"],
            "generator_version": generator_version(son.model_version, cfg),
            "plume_mask_threshold": son.mask_threshold,
            "plume_pixels": {"all": int(np.count_nonzero(ping.plume_mask_all)),
                             "direct": int(np.count_nonzero(ping.plume_mask_direct)),
                             "geom": int(np.count_nonzero(ping.plume_mask_geom))},
            "timing_ms": {k: round(v, 3) for k, v in ping.timing_ms.items()}}
    pos = leak_position(scene)
    if pos is not None:
        r, b, e = sonar_frame(pos, ping.position, ping.rot)[0]
        meta["leak"] = {"position": [float(v) for v in pos], "range_m": float(r), "bearing_deg": float(b),
                        "elevation_deg": float(e), "buried": bool(scene.leak.buried),
                        "flow_lpm": sim.flow_now_lpm() if sim is not None else 0.0,
                        "flow_nominal_lpm": sim.source.q_gas * 6e4 if sim is not None else 0.0,
                        "mass_flow_g_s": sim.source.mdot * 1e3 if sim is not None else 0.0}
    images = {"polar": ping.image[::-1], "mask_all": ping.plume_mask_all[::-1],
              "mask_direct": ping.plume_mask_direct[::-1], "mask_geom": ping.plume_mask_geom[::-1]}
    if get(cfg, "recorder.fan_png") if fan is None else fan:
        images["fan"] = son.fan(ping.image, width=int(get(cfg, "ros.fan_width")))
    return Message("sonar", seq, stamp,
                   arrays={"polar_db": ping.db.astype(np.float16), "plume_share_all_u8": _u8(ping.plume_share_all),
                           "plume_share_direct_u8": _u8(ping.plume_share_direct),
                           "range_m": ping.range_m.astype(np.float32), "bearing_deg": ping.bearing_deg.astype(np.float32)},
                   images=images, meta=meta)
