"""Validate a COCO _annotations.coco.json the way RF-DETR and pycocotools will read it.

    <venv-with-pycocotools>/bin/python -I tools/verify_coco.py DIR/_annotations.coco.json ...

Needs pycocotools, which the repo's own interpreter does not have; on linus the only venv that does is
/home/andreas/image_processing/datasets_gSEM/autodistill-env. ``COCO(path)`` itself raises on almost nothing --
it is json.load plus one isinstance check and six unguarded index loops -- so everything that matters is
asserted here instead. The last check is the important one: an odd-length or degenerate polygon passes every
structural test and then rasterises to nothing, which is invisible until a model fails to learn.
"""
import json
import sys
from collections import Counter

from pycocotools.coco import COCO
import pycocotools.mask as cm

bad = 0
for path in sys.argv[1:]:
    raw = json.load(open(path, encoding="utf-8"))
    coco = COCO(path)
    fail = []
    img_ids = [i["id"] for i in raw["images"]]
    ann_ids = [a["id"] for a in raw["annotations"]]
    if len(img_ids) != len(set(img_ids)):
        fail.append(f"duplicate image ids: {[k for k, v in Counter(img_ids).items() if v > 1][:5]}")
    if len(ann_ids) != len(set(ann_ids)):
        fail.append(f"duplicate annotation ids: {[k for k, v in Counter(ann_ids).items() if v > 1][:5]}")
    known, cats = set(img_ids), {c["id"] for c in raw["categories"]}
    names = [c["name"] for c in raw["categories"]]
    if 0 in cats:
        fail.append("a category sits at id 0, the slot Roboflow reserves for its synthetic root")
    if len(names) != len(set(names)):
        fail.append(f"duplicate category names {names}: RF-DETR would train one class per entry")
    empty = 0
    for a in raw["annotations"]:
        if a["image_id"] not in known:
            fail.append(f"annotation {a['id']} refers to the missing image {a['image_id']}")
        if a["category_id"] not in cats:
            fail.append(f"annotation {a['id']} has the unknown category {a['category_id']}")
        for key in ("area", "bbox", "iscrowd"):
            if key not in a:
                fail.append(f"annotation {a['id']} has no {key}; RF-DETR reads it without a default")
        if a.get("iscrowd") != 0:
            fail.append(f"annotation {a['id']} iscrowd={a.get('iscrowd')}: dropped from supervision")
        x, y, w, h = a["bbox"]
        if not (w > 0 and h > 0 and a["area"] > 0):
            fail.append(f"annotation {a['id']} is degenerate: bbox={a['bbox']} area={a['area']}")
        for k, poly in enumerate(a.get("segmentation", [])):
            if len(poly) % 2 or len(poly) < 6 or len(poly) == 4:
                fail.append(f"annotation {a['id']} polygon {k} has {len(poly)} values; an even count >= 6, "
                            f"and never exactly 4, which frPyObjects reads as a bbox")
        if a.get("segmentation"):
            im = coco.imgs[a["image_id"]]
            if cm.decode(cm.frPyObjects(a["segmentation"], im["height"], im["width"])).sum() == 0:
                empty += 1
                fail.append(f"annotation {a['id']}: its polygons rasterise to an empty mask")
    tag = "/".join(path.split("/")[-2:])
    print(f"{'FAIL' if fail else 'OK  '} {tag}: images={len(raw['images'])} annotations={len(raw['annotations'])} "
          f"negatives={len(raw['images']) - len(coco.imgToAnns)} categories={sorted(cats)} {names} "
          f"empty_rasters={empty}")
    for f in fail[:10]:
        print(f"     {f}")
    if len(fail) > 10:
        print(f"     ... and {len(fail) - 10} more")
    bad += len(fail)
sys.exit(1 if bad else 0)
