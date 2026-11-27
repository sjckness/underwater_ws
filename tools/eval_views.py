"""M12 P4: how well a stock detector sees the pool's COCO objects. Runs an Ultralytics model on the frames of
`python -m uw pool --views NAME` and scores it against Replicator's 2-D boxes.

    UW_YOLO_PYTHON=/HDD2T/dev_ws/py_venv/bin/python    # linus: a Python with ultralytics (ros/yolo_detector)
    $UW_YOLO_PYTHON tools/eval_views.py clear [murky ...] [--model yolo26n.pt] [--conf 0.25]

For every view, the target object (the one the camera was aimed at) is found if a detection of its COCO class
overlaps its ground-truth box with IoU >= --iou. Recall is reported per class and per distance; a miss says what
the detector saw there instead (the best-overlapping detection of another class, or nothing). Objects the camera
merely happens to see are scored too, in a second table, by distance bins from their box height. Output:
output/pool/views/<NAME>/eval_<model>.json and a markdown table on stdout.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
VIEWS = REPO / "output" / "pool" / "views"
sys.path.insert(0, str(REPO / "ros" / "yolo_detector"))


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def score(view: dict, dets: list, thr: float) -> dict:
    """The target's outcome in one view: hit / miss with what was there instead / not visible (no GT box)."""
    want = view["coco"]
    gts = [g for g in view["gt"] if g["coco"].replace("_", " ") == want]
    if not gts:
        return {"outcome": "not_visible"}
    gt = max(gts, key=lambda g: (g["box"][2] - g["box"][0]) * (g["box"][3] - g["box"][1]))
    best_same = max((iou(gt["box"], d["box"]) for d in dets if d["cls"] == want), default=0.0)
    if best_same >= thr:
        conf = max(d["conf"] for d in dets if d["cls"] == want and iou(gt["box"], d["box"]) >= thr)
        return {"outcome": "hit", "conf": conf, "iou": best_same, "box_h": gt["box"][3] - gt["box"][1]}
    other = max(((iou(gt["box"], d["box"]), d) for d in dets if d["cls"] != want), default=(0.0, None),
                key=lambda x: x[0])
    instead = other[1]["cls"] if other[1] is not None and other[0] >= thr else None
    return {"outcome": "miss", "instead": instead, "box_h": gt["box"][3] - gt["box"][1]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="+", help="output/pool/views/<NAME> of `uw pool --views NAME`")
    ap.add_argument("--model", default="yolo26n.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="")
    args = ap.parse_args(argv)

    os.environ.setdefault("YOLO_OFFLINE", "1")
    from ultralytics import YOLO
    from yolo_detector.detector import models_dir, resolve_model
    model = YOLO(str(resolve_model(args.model, models_dir())))
    names = model.names
    report = {}
    for run in args.runs:
        d = VIEWS / run
        index = json.loads((d / "index.json").read_text())
        results = []
        for v in index["views"]:
            r = model.predict(str(d / v["file"]), conf=args.conf, imgsz=args.imgsz, device=args.device or None,
                              verbose=False)[0]
            dets = [{"cls": names[int(c)], "conf": float(p), "box": [float(x) for x in b]}
                    for b, c, p in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist())]
            results.append({**{k: v[k] for k in ("file", "object", "coco", "distance_m", "azimuth_deg")},
                            **score(v, dets, args.iou), "detections": dets})
        table = defaultdict(lambda: defaultdict(lambda: [0, 0]))
        instead = defaultdict(lambda: defaultdict(int))
        for r in results:
            if r["outcome"] == "not_visible":
                continue
            cell = table[r["coco"]][r["distance_m"]]
            cell[1] += 1
            if r["outcome"] == "hit":
                cell[0] += 1
            else:
                instead[r["coco"]][r.get("instead") or "nothing"] += 1
        dists = sorted({r["distance_m"] for r in results})
        print(f"\n### {run} (visibility {index['config']['visibility_m']:.1f} m), {args.model}, conf {args.conf}, "
              f"IoU {args.iou}\n")
        print("| class | " + " | ".join(f"{x:g} m" for x in dists) + " | all | missed as |")
        print("|---|" + "---|" * (len(dists) + 2))
        summary = {}
        for cls in sorted(table):
            row = table[cls]
            hit = sum(row[x][0] for x in dists)
            tot = sum(row[x][1] for x in dists)
            cells = [f"{row[x][0]}/{row[x][1]}" if row[x][1] else "-" for x in dists]
            miss = ", ".join(f"{k} {n}" for k, n in sorted(instead[cls].items(), key=lambda kv: -kv[1]))
            print(f"| {cls} | " + " | ".join(cells) + f" | {hit}/{tot} | {miss or '-'} |")
            summary[cls] = {"recall": hit / tot if tot else None, "hits": hit, "views": tot,
                            "by_distance": {str(x): row[x] for x in dists}, "missed_as": dict(instead[cls])}
        out = {"model": args.model, "conf": args.conf, "iou": args.iou, "config": index["config"],
               "summary": summary, "views": results}
        (d / f"eval_{Path(args.model).stem}.json").write_text(json.dumps(out, indent=1))
        report[run] = summary
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
