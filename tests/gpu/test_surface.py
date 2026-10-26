"""Surface boil backscatter (uw/surface.py) on the GPU: the Kirchhoff level of a saturated cell, zero without bursts,
and its echo in the sonar (all-paths label only)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")

from uw import bubbles, config, geometry, sonar, surface  # noqa: E402


def _fake_sim(counts, cell=0.01):
    return SimpleNamespace(bursts=wp.array(counts.astype(np.int32), dtype=wp.int32), t=0.0, device="cuda:0",
                           burst_origin=(1.0, 1.0), burst_cell=cell, water_level=2.9)


def test_kirchhoff_level_of_a_saturated_cell():
    cfg = config.load()
    counts = np.zeros((4, 4))
    counts[1, 2] = 10 ** 6                                  # saturated: f = 1
    sim = _fake_sim(counts)
    boil = surface.Boil(config.merge(cfg, {"sonar": {"boil_backscatter": {"time_s": 1e-9}}}), sim)
    sim.bursts.assign(counts.astype(np.int32))              # Boil() cleared it: refill
    sim.t = 0.1
    (pos, sig, labels), = boil.update([1.015, 1.025, 1.0])  # straight below the cell
    s = sig.numpy().reshape(4, 4)
    s2 = float(config.get(cfg, "sonar.boil_backscatter.rms_slope")) ** 2
    assert s[1, 2] == pytest.approx(0.01 ** 2 / (8 * math.pi * s2), rel=0.01)      # theta ~ 0
    assert np.count_nonzero(s) == 1 and labels == 1
    assert np.all(sim.bursts.numpy() == 0)                  # read and cleared
    p = pos.numpy().reshape(4, 4, 3)[1, 2]
    assert 1.01 <= p[0] <= 1.02 and 1.02 <= p[1] <= 1.03 and p[2] == pytest.approx(2.9, abs=1e-3)


def test_boil_echo_goes_into_the_all_paths_label():
    """Fan vertical, looking up at where the plume breaks the surface: the boil adds energy there, counted in the
    all-paths plume label and not in the direct-path one."""
    cfg = config.load(overrides=["sonar.roll_deg=90", "sonar.pitch_deg=-35", "sonar.speckle_sigma_rad=0"])
    scene = geometry.build_scene(cfg)
    sim = bubbles.BubbleSim(cfg, scene)
    sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    son = sonar.Sonar(cfg, scene, None, plume=sonar.plume_region(sim))
    boil = surface.Boil(cfg, sim)
    sim.step(0.5)
    extra = boil.update(son.position)
    assert float(extra[0][1].numpy().max()) > 0
    with_boil = son.ping(bubbles=sim, extra=extra)
    without = son.ping(bubbles=sim)
    gain = with_boil.power.sum() - without.power.sum()
    assert gain > 0
    assert np.sum(with_boil.power * with_boil.plume_share_all) - np.sum(without.power * without.plume_share_all) \
        == pytest.approx(gain, rel=0.2)
    assert np.sum(with_boil.power * with_boil.plume_share_direct) == pytest.approx(
        np.sum(without.power * without.plume_share_direct), rel=0.2)
