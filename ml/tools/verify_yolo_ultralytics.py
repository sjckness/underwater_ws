"""Build a YOLO-seg dataset through Ultralytics' own loader, with no model and no training.

    <venv-with-ultralytics>/bin/python -I tools/verify_yolo_ultralytics.py DIR/data.yaml

Needs ultralytics, which the repo's own interpreter does not have; on linus use
/home/andreas/dev_ws/kumi_stack/.venv. This is the exact code path `yolo segment train` takes to read a
dataset -- check_det_dataset for the yaml, build_yolo_dataset for the scan, ds[0] for the label parse, the
polygon rasterisation and the letterbox -- so it raises on everything a malformed dataset raises on, costs no
GPU and no download, and its only side effect is the labels/<split>.cache Ultralytics always writes.

It prints the per-split "N images, M backgrounds, K corrupt" line. Any K above zero means images were dropped.
"""
import sys

from ultralytics.cfg import get_cfg
from ultralytics.data.build import build_yolo_dataset
from ultralytics.data.utils import check_det_dataset
from ultralytics.utils import DEFAULT_CFG

data = check_det_dataset(sys.argv[1])
print(f"names={data['names']} nc={data.get('nc')} path={data.get('path')}")
cfg = get_cfg(DEFAULT_CFG, {"imgsz": int(sys.argv[2]) if len(sys.argv) > 2 else 640,
                            "task": "segment", "mode": "train", "rect": True})
rc = 0
for split in ("train", "val", "test"):
    if not data.get(split):
        continue
    ds = build_yolo_dataset(cfg, data[split], 16, data, mode="train" if split == "train" else "val",
                            rect=True, stride=32)
    empty = sum(1 for lb in ds.labels if len(lb["cls"]) == 0)
    item = ds[0]
    print(f"{split}: {len(ds)} items, {empty} backgrounds, img {tuple(item['img'].shape)}, "
          f"masks {tuple(item['masks'].shape)}, instances in item 0 {tuple(item['cls'].shape)}")
    if len(ds) == 0:
        print(f"FAIL {split}: no items")
        rc = 1
sys.exit(rc)
