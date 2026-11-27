"""tools/eval_views.py and uw.pool.view_poses (M12 P4): the scoring and the camera poses, without Kit or a model."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

import eval_views as ev  # noqa: E402
from uw import config  # noqa: E402


def test_iou():
    assert ev.iou([0, 0, 10, 10], [0, 0, 10, 10]) == 1.0
    assert ev.iou([0, 0, 10, 10], [5, 0, 15, 10]) == pytest.approx(50 / 150)
    assert ev.iou([0, 0, 1, 1], [2, 2, 3, 3]) == 0.0


def test_a_view_is_a_hit_a_miss_or_not_visible():
    view = {"coco": "sports ball", "gt": [{"coco": "sports_ball", "box": [100, 100, 140, 140]},
                                           {"coco": "chair", "box": [300, 100, 400, 260]}]}
    hit = ev.score(view, [{"cls": "sports ball", "conf": 0.8, "box": [101, 99, 141, 139]}], 0.5)
    assert hit["outcome"] == "hit" and hit["conf"] == 0.8
    miss = ev.score(view, [{"cls": "orange", "conf": 0.6, "box": [100, 100, 140, 140]},
                           {"cls": "sports ball", "conf": 0.9, "box": [300, 100, 400, 260]}], 0.5)
    assert miss == {"outcome": "miss", "instead": "orange", "box_h": 40}
    assert ev.score({"coco": "boat", "gt": view["gt"]}, [], 0.5) == {"outcome": "not_visible"}


def test_the_view_poses_aim_the_camera_at_each_object():
    from uw import pool_geometry
    from uw.pool import view_poses
    cfg = config.load(config.POOL_CONFIG)
    cam_off = np.asarray(cfg["vehicle"]["camera"]["position_body"], float)
    objs = {pl["name"]: pl for pl, _ in pool_geometry.placed_objects(cfg)}
    poses = list(view_poses(cfg, [2.0, 6.0], 4, -0.2))
    assert {pl["name"] for pl, *_ in poses} == set(objs)
    for pl, d, az, body, rot, tilt in poses:
        cam = body + rot @ cam_off
        cx, cy, _ = pl["xyz"]
        assert math.hypot(cam[0] - cx, cam[1] - cy) == pytest.approx(d)
        fwd = rot[:, 0]
        to_obj = np.array([cx - cam[0], cy - cam[1]]) / d
        assert fwd[:2] @ to_obj == pytest.approx(1.0) and cam[2] >= 0.35 - 1e-9 and -45.0 <= tilt <= 45.0
