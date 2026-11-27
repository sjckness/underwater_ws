"""M11 pool: config, the underwater post-process reference, the PWM mappings, the launcher entry (host, numpy only).

The Warp kernel, the USD stage and the encoder are tested in tests/gpu/test_pool_gpu.py (container).
"""
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import camera_uw  # noqa: E402
from uw import config  # noqa: E402

POOL = Path(config.POOL_CONFIG)


def leaves(d, prefix=""):
    out = set()
    for k, v in d.items():
        out |= leaves(v, f"{prefix}{k}.") if isinstance(v, dict) else {prefix + k}
    return out


def test_pool_config_loads_with_every_profile():
    for prof in ("linus", "workstation"):
        cfg = config.load(POOL, profile=prof)
        assert cfg["profile"]["name"] == prof
        assert cfg["vehicle"]["params"] == "vehicles/bluerov2_heavy/vehicle.yaml"
        assert cfg["physics"]["surface_model"] is True
    with pytest.raises(config.ConfigError, match="no such key"):
        config.load(POOL, overrides=["camera.ratehz=10"])


def test_pool_profiles_set_the_same_keys():
    a, b = (yaml.safe_load((POOL.parent / "profiles" / f"{p}.yaml").read_text()) for p in ("linus", "workstation"))
    assert leaves(a) == leaves(b)


@pytest.mark.parametrize("presets", [[], ["pool10"]])
def test_pool_start_is_inside_the_tank_and_at_the_surface(presets):
    cfg = config.load(POOL, presets=presets)
    x, y, z, _ = cfg["vehicle"]["start"]
    L, W, H = cfg["pool"]["size"]
    assert 0.5 < x < L - 0.5 and 0.5 < y < W - 0.5
    assert abs(cfg["pool"]["water_level"] - z) < 0.2, "the vehicle starts floating, as one is launched"
    assert cfg["pool"]["water_level"] < H, "the walls stand out of the water"
    assert cfg["camera"]["clip_m"][1] >= np.hypot(L, W), "the far plane reaches the far corner"


def test_water_model_limits():
    """d = 0: the rendered colour unchanged; d -> inf: the veiling light; a miss: the veiling light."""
    bd, bb, binf = (np.array(v) for v in ([0.4, 0.075, 0.05], [0.36, 0.065, 0.045], [0.035, 0.16, 0.22]))
    rgb = np.array([[[200, 120, 40], [10, 250, 90], [0, 0, 0]]], np.uint8)
    out = camera_uw.apply_numpy(rgb, np.array([[1e-9, 1e9, np.inf]]), bd, bb, binf)
    assert np.abs(out[0, 0].astype(int) - rgb[0, 0]).max() <= 1
    veil = np.round(camera_uw._to_srgb(binf) * 255).astype(int)
    assert np.abs(out[0, 1].astype(int) - veil).max() <= 1
    assert np.abs(out[0, 2].astype(int) - veil).max() <= 1


def test_water_model_attenuates_red_first():
    """A white wall fades to blue-green with distance: red is gone after a few metres, blue lasts longest."""
    bd, bb, binf = np.array([0.4, 0.075, 0.05]), np.array([0.36, 0.065, 0.045]), np.zeros(3)
    white = np.full((1, 1, 3), 255, np.uint8)
    out = [camera_uw.apply_numpy(white, np.array([[d]]), bd, bb, binf)[0, 0] for d in (1.0, 5.0)]
    assert out[0][0] < out[0][1] < 255 and out[0][2] > out[0][1]
    assert out[1][0] < 0.5 * out[1][2]


def test_tilt_and_lights_follow_the_pwm():
    from sim.pool import lights_of_pwm, tilt_of_pwm
    cfg = config.load(POOL)
    lo, hi = cfg["vehicle"]["camera"]["tilt_range_deg"]
    assert tilt_of_pwm(cfg, 1100) == lo and tilt_of_pwm(cfg, 1900) == hi
    assert tilt_of_pwm(cfg, 1500) == pytest.approx(0.5 * (lo + hi))
    assert tilt_of_pwm(cfg, 2500) == hi
    assert lights_of_pwm(1100) == 0.0 and lights_of_pwm(1900) == 1.0


def test_pool_is_a_launcher_command():
    from uw import launcher
    assert "pool" in launcher.COMMANDS


def test_turbidity_adds_particle_scattering_and_greys_the_veil():
    """sim/camera_uw.py: b_p on both coefficients, the veiling light pulled towards turbidity_veil; 0 is the clear
    pool exactly."""
    from sim import camera_uw
    clear = config.load(config.POOL_CONFIG)
    bd0, bb0, bi0 = camera_uw.coefficients(clear)
    assert np.allclose(bd0, clear["water"]["beta_d"]) and np.allclose(bi0, clear["water"]["b_inf"])
    tau = 0.6
    cfg = config.load(config.POOL_CONFIG, overrides=[f"water.turbidity_per_m={tau}"])
    bd, bb, bi = camera_uw.coefficients(cfg)
    assert np.allclose(bd, bd0 + tau) and np.allclose(bb, bb0 + tau)
    veil = np.asarray(cfg["water"]["turbidity_veil"])
    assert np.allclose(bi, (bb0 * bi0 + tau * veil) / (bb0 + tau), atol=1e-6)
    very = config.load(config.POOL_CONFIG, overrides=["water.turbidity_per_m=1000"])
    assert np.allclose(camera_uw.coefficients(very)[2], veil, atol=1e-3), "all particles: the veil is theirs"


def test_turbidity_presets_and_visibility():
    from sim import camera_uw
    vis = {name: camera_uw.visibility_m(config.load(config.POOL_CONFIG, presets=pre))
           for name, pre in (("clear", []), ("turbid", ["turbid"]), ("murky", ["murky"]))}
    assert vis["clear"] > 40.0 and 5.0 < vis["turbid"] < 7.0 and 2.0 < vis["murky"] < 3.0


def test_negative_turbidity_is_refused():
    from sim import camera_uw
    with pytest.raises(config.ConfigError, match="turbidity"):
        camera_uw.coefficients(config.load(config.POOL_CONFIG, overrides=["water.turbidity_per_m=-0.1"]))
