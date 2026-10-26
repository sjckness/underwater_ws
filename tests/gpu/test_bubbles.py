"""Bubble plume on the GPU: 12 s warm-up + 20 s for both pipe presets, then gas balance, bursts, finiteness.

Runs in the container (scripts/uw.sh test); skipped where Warp is not importable (host pytest).
"""
import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: scripts/uw.sh test")

from uw import bubbles, config, geometry  # noqa: E402

RUN_S = 20.0


@pytest.fixture(scope="module", params=["exposed", "buried"])
def run(request):
    cfg = config.load(presets=[request.param])
    sim = bubbles.BubbleSim(cfg, geometry.build_scene(cfg))
    sim.warmup(float(config.get(cfg, "plume.warmup_s")))
    before = sim.counters()
    sim.step(RUN_S)
    return sim, before, sim.counters()


def test_no_nans(run):
    sim, _, _ = run
    snap = sim.snapshot()
    assert len(snap["pos"]) > 1000
    for k in ("pos", "radius", "vel", "age"):
        assert np.all(np.isfinite(snap[k])), k


def test_gas_is_conserved(run):
    """Every bubble's gas amount p*V is accounted for: alive (from its current radius and depth, so the expansion law
    is checked too), burst at the surface, or dropped from a full pool (which must not happen here)."""
    sim, _, _ = run
    b = sim.gas_budget()
    assert b["lost"] == 0.0
    assert b["alive"] + b["burst"] == pytest.approx(b["emitted"], rel=1e-5)


def test_bubbles_burst_at_the_surface(run):
    """In the established plume the gas bursting at the surface balances the gas released (storage fluctuates with
    the meander and the vents), the plume reaches the surface, and no bubble is above it."""
    sim, before, after = run
    burst = after["burst_gas"] - before["burst_gas"]
    emitted = after["emitted_gas"] - before["emitted_gas"]
    assert burst == pytest.approx(emitted, rel=0.10)
    z = sim.snapshot()["pos"][:, 2]
    assert z.max() <= sim.water_level
    assert z.max() > sim.water_level - 0.05


def test_gas_balance_when_the_pool_overflows():
    """A pool far too small for the plume: bubbles are dropped, and their gas is still accounted for."""
    cfg = config.load(overrides=["plume.max_bubbles=3000"])
    sim = bubbles.BubbleSim(cfg, geometry.build_scene(cfg))
    sim.step(3.0)
    b = sim.gas_budget()
    assert b["lost"] > 0.0
    assert b["alive"] + b["burst"] + b["lost"] == pytest.approx(b["emitted"], rel=1e-5)


def test_lbm_flow_field_is_m6():
    cfg = config.load(overrides=["plume.flow_field=lbm"])
    with pytest.raises(NotImplementedError, match="M6"):
        bubbles.BubbleSim(cfg, geometry.build_scene(cfg))


@pytest.mark.parametrize("preset", ["exposed", "buried"])
def test_cuda_graph_and_batching_do_not_change_the_bubbles(preset):
    """Substeps captured as a CUDA graph or launched one by one, in one batch or one substep at a time: the same
    random numbers in the same order, the same kernels, so bit-identical pools."""
    states = []
    for graph, chunks in ((True, [0.1]), (False, [0.1]), (True, [1 / 240] * 24)):
        cfg = config.load(presets=[preset], overrides=[f"plume.cuda_graph={str(graph).lower()}"])
        sim = bubbles.BubbleSim(cfg, geometry.build_scene(cfg))
        sim.warmup(1.0)
        for dt in chunks * 3:
            sim.step(dt)
        states.append([getattr(sim, k).numpy() for k in ("pos", "r", "alive", "vel", "turb", "stats")]
                      + [np.array([sim.t, sim.head, sim.emitted, sim.emitted_gas])])
    for other in states[1:]:
        for a, b in zip(states[0], other):
            assert np.array_equal(a, b)
