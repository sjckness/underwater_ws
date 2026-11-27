"""The stock config must keep producing the same ping (GPU; `python -m uw test`).

This is the guard that lets the calibration work proceed at all. Every sonar parameter added from here on
defaults to today's behaviour, so that dataset v0.2 -- and the detector trained on its frozen test split --
stay comparable with anything generated later. A default that drifts would invalidate that comparison
silently, looking exactly like a clean run.

What is compared is a set of summary statistics of `polar_db` against `golden_ping.json`, not the array
itself. Two reasons. `*.npz` is gitignored, so a reference array could not be committed beside the test. And
bit-identity is only promised *within* a machine: echoes are summed in 64-bit fixed point so the order of
atomics cannot change the sum (`physics/sonar.py`), which makes a run reproducible here and `sonar-only` identical
to `sim` (`tests/isaac/test_sim_identity.py` checks that), but a different GPU or driver can still differ in
the last bits of the transcendentals feeding it. The tolerances below are tight enough that any real change
to the model trips them -- 0.05 dB on a percentile of a 876x512 image -- and loose enough to survive that.

If the reference is missing the test writes it and fails once, telling you to commit it. It is regenerated
deliberately, never automatically: a test that rewrites its own expectation when it fails guards nothing.
"""
import json
from pathlib import Path

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")

from physics import geometry, sonar  # noqa: E402

GOLDEN = Path(__file__).with_name("golden_ping.json")
# dB tolerance per statistic. A changed scattering term, beam pattern, ring-down or TVG moves these by far
# more; cross-GPU noise in the last bits of a transcendental moves them by far less.
TOL_DB = 0.05


def summary(ping) -> dict:
    db = np.asarray(ping.db, float)
    ok = np.isfinite(db)
    q = np.percentile(db[ok], [1, 5, 25, 50, 75, 95, 99])
    return {"shape": list(db.shape), "finite": int(ok.sum()),
            "model_version": None,                       # filled by the caller
            "percentiles_db": [float(v) for v in q],
            "mean_db": float(db[ok].mean()), "max_db": float(db[ok].max()),
            "blind_rows": int((~ok).all(1).sum())}


def test_the_stock_config_still_makes_the_same_ping(cfg):
    scene = geometry.build_scene(cfg)
    sc = geometry.scatterers(cfg, scene)
    son = sonar.Sonar(cfg, scene, sc)
    got = summary(son.ping())
    got["model_version"] = son.model_version

    if not GOLDEN.is_file():
        GOLDEN.write_text(json.dumps(got, indent=1) + "\n")
        pytest.fail(f"no reference ping yet: wrote {GOLDEN.name}. Inspect it and commit it.")

    want = json.loads(GOLDEN.read_text())
    assert got["shape"] == want["shape"], "the ping changed size: a mode or range default moved"
    assert got["blind_rows"] == want["blind_rows"], "the blind zone moved"
    assert got["finite"] == want["finite"], "the number of range bins carrying data changed"
    assert got["model_version"] == want["model_version"], (
        f"SONAR_MODEL_VERSION went {want['model_version']} -> {got['model_version']}: if the model really "
        f"changed, regenerate {GOLDEN.name} deliberately and say so in docs/validation.md")
    for key in ("mean_db", "max_db"):
        assert got[key] == pytest.approx(want[key], abs=TOL_DB), f"{key} moved: a default changed"
    assert got["percentiles_db"] == pytest.approx(want["percentiles_db"], abs=TOL_DB), (
        "the level distribution of the stock ping moved: a sonar default changed. That breaks comparability "
        "with v0.2 and the detector trained on it -- new behaviour belongs behind a new parameter whose "
        "default reproduces this.")
