"""Training loader for sweep datasets (tools/sweep.py): a PyTorch Dataset of pings.

    from uw.dataset import SonarDataset
    ds = SonarDataset("/HDD2T/underwater_data/v0.1", split="train", size=(512, 256))
    item = ds[0]      # {"image": 1xHxW float, "masks": 3xHxW float, "shares": 2xHxW float, "targets": {...}, "meta"}

Every array is in the polar layout of the npz: row 0 = range 0 (the PNGs, far range at the top, are flipped back),
columns = beams from port to starboard. ``image``: "polar" (the 8-bit display image / 255: TVG and the 0-80 dB window
above the noise floor) or "db" (raw dB re 1 uPa minus the noise floor, divided by 80; the blind zone is 0).
``masks``: the three labels as 0/1 in ``mask_names`` order (default mask_geom, mask_direct, mask_all); ``shares``:
the bubbles' energy share, all paths and direct, 0-1. ``targets``: leak_present, flow_lpm (now; 0 without a leak),
log10_flow (log10 of flow + 0.01), leak_range_m, leak_bearing_deg, leak_elevation_deg (NaN without a leak), in_view
(the geometric label marks a pixel). Pings differ in size (LF/HF, tank/basin): ``size`` resizes (bilinear for the
image and the shares, nearest for the masks), else a batch needs equal sizes (e.g. a sampler per mode).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .sinks import read_png

MASKS = ("mask_geom", "mask_direct", "mask_all")


def ping_files(root, split: str | None = None, kinds=None) -> list:
    """(scenario id, scenario dir, ping stem) of every ping of the finished scenarios of ``split``."""
    root = Path(root)
    manifest = json.loads((root / "manifest.json").read_text())
    out = []
    for sc in manifest["scenarios"]:
        if sc.get("status") != "done" or (split is not None and sc["split"] != split):
            continue
        if kinds is not None and sc["kind"] not in kinds:
            continue
        d = root / sc["id"]
        out += [(sc["id"], d, p.stem) for p in sorted(d.glob("ping_*.npz"))]
    return out


def load_ping(d: Path, stem: str, image: str = "polar", mask_names=MASKS) -> dict:
    """One ping as numpy arrays (see the module doc)."""
    meta = json.loads((d / f"{stem}.json").read_text())
    with np.load(d / f"{stem}.npz") as z:
        db = z["polar_db"].astype(np.float32)
        shares = np.stack([z["plume_share_all_u8"], z["plume_share_direct_u8"]]).astype(np.float32) / 255.0
    if image == "polar":
        img = read_png(d / f"{stem}.png")[::-1].astype(np.float32) / 255.0
    elif image == "db":
        img = np.nan_to_num((db - float(meta["sonar"]["noise_floor_db"])) / 80.0, nan=0.0)
    else:
        raise ValueError(f"image must be polar or db, got {image!r}")
    masks = np.stack([read_png(d / f"{stem}_{m}.png")[::-1] > 0 for m in mask_names]).astype(np.float32)
    leak = meta.get("leak")
    flow = float(leak["flow_lpm"]) if leak else 0.0
    targets = {"leak_present": float(leak is not None and flow > 0.0), "flow_lpm": flow,
               "log10_flow": math.log10(flow + 0.01),
               "leak_range_m": float(leak["range_m"]) if leak else math.nan,
               "leak_bearing_deg": float(leak["bearing_deg"]) if leak else math.nan,
               "leak_elevation_deg": float(leak["elevation_deg"]) if leak else math.nan,
               "in_view": float(meta["plume_pixels"]["geom"] > 0)}
    return {"image": img[None], "masks": masks, "shares": shares, "targets": targets, "meta": meta}


class SonarDataset:
    """Pings of one split. Subclass of torch.utils.data.Dataset when torch is importable (the class is built at
    import), else a plain sequence of numpy items."""

    def __init__(self, root, split: str | None = "train", image: str = "polar", mask_names=MASKS, size=None,
                 kinds=None):
        self.files = ping_files(root, split, kinds)
        self.image, self.mask_names, self.size = image, tuple(mask_names), size

    def __len__(self) -> int:
        return len(self.files)

    def __getitem__(self, i: int) -> dict:
        sid, d, stem = self.files[i]
        item = load_ping(d, stem, self.image, self.mask_names)
        item["meta"] = {"scenario": sid, "ping": stem, "mode": item["meta"]["sonar"]["mode"],
                        "stamp": item["meta"]["stamp"]}
        try:
            import torch
        except ImportError:
            return item
        out = {k: torch.from_numpy(np.ascontiguousarray(item[k])) for k in ("image", "masks", "shares")}
        if self.size is not None:
            import torch.nn.functional as F
            size = tuple(int(v) for v in self.size)
            out["image"] = F.interpolate(out["image"][None], size=size, mode="bilinear", align_corners=False)[0]
            out["shares"] = F.interpolate(out["shares"][None], size=size, mode="bilinear", align_corners=False)[0]
            out["masks"] = F.interpolate(out["masks"][None], size=size, mode="nearest")[0]
        out["targets"] = {k: torch.tensor(v, dtype=torch.float32) for k, v in item["targets"].items()}
        out["meta"] = item["meta"]
        return out


try:                                    # a real torch Dataset when torch is there
    import torch.utils.data as _tud

    SonarDataset = type("SonarDataset", (SonarDataset, _tud.Dataset), {"__doc__": SonarDataset.__doc__})
except ImportError:
    pass
