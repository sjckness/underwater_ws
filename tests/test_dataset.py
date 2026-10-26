"""Sweep dataset tooling (host): scenario sampling and splits (tools/sweep.py), the training loader (uw/dataset.py)
on a small synthetic dataset written through the flat FileSink."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from uw import config, dataset, sinks

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import sweep  # noqa: E402


@pytest.fixture(scope="module")
def sampled(tmp_path_factory):
    sw = yaml.safe_load((WS / "config" / "sweeps" / "v0.1.yaml").read_text())
    sw["scenarios"] = 120
    return sw, sweep.sample(sw, tmp_path_factory.mktemp("sweep"))


def test_sampled_scenarios_are_valid_configs(sampled):
    sw, scenarios = sampled
    for sc in scenarios:
        cfg = config.load(presets=sc["presets"], overrides=sc["overrides"])
        config.sonar_mode(cfg)
        if sc["kind"] == "leak":
            lo, hi = config.get(cfg, "leak.flow_range_lpm")
            assert lo <= config.get(cfg, "leak.flow_lpm") <= hi
        else:
            assert config.get(cfg, "leak.enabled") is False
    assert len({sc["params"]["seed"] for sc in scenarios}) == len(scenarios)


def test_splits_by_scenario_and_stratified(sampled, tmp_path):
    sw, scenarios = sampled
    counts = {s: sum(sc["split"] == s for sc in scenarios) for s in sw["splits"]}
    for s, f in sw["splits"].items():
        assert counts[s] == pytest.approx(f * len(scenarios), abs=0.06 * len(scenarios))
    by = {}
    for sc in scenarios:
        by.setdefault(sc["stratum"], []).append(sc["split"])
    for splits in by.values():
        if len(splits) >= 10:
            assert set(splits) == {"train", "val", "test"}
    again = sweep.sample(sw, tmp_path)
    assert [s["overrides"] for s in again] == [s["overrides"] for s in scenarios]   # deterministic


@pytest.fixture(scope="module")
def sampled_v02(tmp_path_factory):
    sw = yaml.safe_load((WS / "config" / "sweeps" / "v0.2.yaml").read_text())
    sw["scenarios"] = 60
    return sw, sweep.sample(sw, tmp_path_factory.mktemp("sweep02"))


def test_v02_places_the_sonar_on_a_wall_and_the_leak_at_a_distance(sampled_v02):
    sw, scenarios = sampled_v02
    pl = sw["sonar"]["placement"]
    walls = set()
    for sc in scenarios:
        cfg = config.load(presets=sc["presets"], overrides=sc["overrides"])
        config.sonar_mode(cfg)
        L, W, _ = config.get(cfg, "tank.size")
        size, wall = sc["params"]["size"], sc["params"]["wall"]
        walls.add(wall)
        px, along = config.get(cfg, "pipe.x"), config.get(cfg, "leak.position_along")
        assert pl["pipe_margin_m"] - 1e-9 <= px <= L - pl["pipe_margin_m"] + 1e-9
        assert 0.4 - 1e-9 <= along <= W - 0.4 + 1e-9
        d = {"x0": px, "x1": L - px, "y0": along, "y1": W - along}[wall]
        lo, hi = pl["leak_from_wall_m"][size]
        assert d <= hi + 1e-9 and (d >= lo - 1e-9 or d == pytest.approx(min(lo, L - pl["pipe_margin_m"], W - 0.4)))
        son = config.get(cfg, "sonar")
        views = sc["params"].get("views", [])
        for p in [son["position"]] + [v["position"] for v in views]:
            assert 0 < p[0] < L and 0 < p[1] < W and 0.6 <= p[2] <= 2.2
        if sc["params"]["motion"] == "mobile":
            assert son["pose_source"] == "trajectory" and son["static_cache"] == "none"
        else:
            n = sw["sonar"]["fixed"]["poses"]
            tc = son["trajectory"]
            assert son["pose_source"] == "trajectory" and tc["preset"] == "poses" and len(tc["poses"]) == n
            assert son["static_cache"] == "on_motion" and tc["hold_s"] * n * son["ping_rate_hz"] == sw["pings"]
    assert walls == {"x0", "x1", "y0", "y1"}


def test_v02_aimed_views_put_the_aim_point_inside_the_fan(sampled_v02):
    from uw import geometry, pose
    sw, scenarios = sampled_v02
    aimed = 0
    for sc in scenarios:
        if "views" not in sc["params"]:
            continue
        cfg = config.load(presets=sc["presets"], overrides=sc["overrides"] + ["leak.enabled=true"])
        mode = config.sonar_mode(cfg)
        target = pose.leak_target(geometry.build_scene(cfg, full=False))
        sp = pose.sensor_pose(cfg)
        per = sw["pings"] // len(sc["params"]["views"])
        for k, v in enumerate(sc["params"]["views"]):
            q = sp.at((k * per + 1) / config.get(cfg, "sonar.ping_rate_hz"))       # the view's first ping
            if not v["aimed"]:
                continue
            aimed += 1
            _, b, e = pose.in_sonar_frame(q.position, q.rot, target + [0.0, 0.0, v["aim"]["height_m"]])
            assert (b, e) == pytest.approx((v["aim"]["bearing_deg"], v["aim"]["elevation_deg"]), abs=0.05)
            assert abs(b) <= mode["fov_h_deg"] / 2 and abs(e) <= mode["fov_v_deg"] / 2
    n = sum(len(sc["params"].get("views", [])) for sc in scenarios)
    assert aimed == pytest.approx(sw["sonar"]["fixed"]["aim"] * n, abs=0.12 * n)


def _fake_dataset(root: Path):
    rng = np.random.default_rng(0)
    rows = []
    for k, (kind, split, leak) in enumerate((("leak", "train", True), ("negative", "train", False),
                                             ("leak", "val", True))):
        sid = f"s{k:04d}"
        with sinks.make_sink("file", out_dir=root / sid, layout="flat") as sink:
            for i in range(3):
                polar = rng.integers(0, 256, (20, 8), dtype=np.uint8)
                mask = np.zeros((20, 8), np.uint8)
                mask[5:8, 2:4] = 255 if leak else 0
                db = rng.normal(100, 5, (20, 8)).astype(np.float16)
                db[:2] = np.nan
                meta = {"sonar": {"noise_floor_db": 80.0, "mode": "lf"},
                        "plume_pixels": {"geom": int(mask.astype(bool).sum())}}
                if leak:
                    meta["leak"] = {"flow_lpm": 12.0, "range_m": 1.5, "bearing_deg": -3.0, "elevation_deg": 1.0}
                sink.publish(sinks.Message("sonar", i, 0.1 * i, arrays={
                    "polar_db": db, "plume_share_all_u8": mask, "plume_share_direct_u8": mask,
                    "range_m": np.arange(20) * 0.004, "bearing_deg": np.linspace(-65, 65, 8)},
                    images={"polar": polar[::-1], "mask_geom": mask[::-1], "mask_direct": mask[::-1],
                            "mask_all": mask[::-1]}, meta=meta))
        rows.append({"id": sid, "kind": kind, "split": split, "status": "done"})
    (root / "manifest.json").write_text(json.dumps({"scenarios": rows}))


def test_loader_items(tmp_path):
    _fake_dataset(tmp_path)
    train = dataset.SonarDataset(tmp_path, "train")
    assert len(train) == 6 and len(dataset.SonarDataset(tmp_path, "val")) == 3
    item = train[0]
    img, masks = np.asarray(item["image"]), np.asarray(item["masks"])
    assert img.shape == (1, 20, 8) and masks.shape == (3, 20, 8)
    assert masks[0, 5:8, 2:4].all() and masks[0].sum() == 6           # row 0 = range 0 (PNG flipped back)
    t = {k: float(v) for k, v in item["targets"].items()}
    assert t["leak_present"] == 1.0 and t["flow_lpm"] == 12.0 and t["in_view"] == 1.0
    neg = train[3]
    assert float(neg["targets"]["leak_present"]) == 0.0 and math.isnan(float(neg["targets"]["leak_range_m"]))
    db = dataset.SonarDataset(tmp_path, "train", image="db")[0]["image"]
    assert np.all(np.asarray(db)[0, :2] == 0)                          # blind zone (NaN) -> 0
    torch = pytest.importorskip("torch")
    resized = dataset.SonarDataset(tmp_path, "train", size=(40, 16))[0]
    assert tuple(resized["image"].shape) == (1, 40, 16) and tuple(resized["masks"].shape) == (3, 40, 16)
    assert isinstance(dataset.SonarDataset(tmp_path, "train"), torch.utils.data.Dataset)
