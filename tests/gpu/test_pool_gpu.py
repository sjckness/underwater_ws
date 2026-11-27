"""M11 pool: the USD stage (in memory, no Kit app), the Warp water post-process against its numpy reference, and the
H.264 encoder writing a file.

Runs in the container (python -m uw test): needs pxr (Isaac's USD libraries) and Warp; the encoder test needs the
imageio-ffmpeg binary in ~/.cache/uw_pydeps (installed by the first `uw pool` or physics/tools/sonar_preview.py run).
"""
import math

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: python -m uw test")
pxr = pytest.importorskip("pxr", reason="needs Isaac's USD libraries: python -m uw test")

from pxr import Usd, UsdGeom, UsdLux  # noqa: E402

from sim import camera_uw, pool_scene  # noqa: E402
from uw import config  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load(config.POOL_CONFIG)


@pytest.fixture(scope="module")
def built(cfg, tmp_path_factory):
    stage = Usd.Stage.CreateInMemory()
    info = pool_scene.build(stage, cfg, texture_dir=tmp_path_factory.mktemp("textures"))
    return stage, info


def _world(stage, path):
    return np.array(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())).T


def test_the_stage_has_the_tank_the_vehicle_and_its_camera(built, cfg):
    stage, info = built
    for path in ("/World/pool/floor", "/World/pool/wall_y0", "/World/pool/water_surface", "/World/pool/ladder",
                 "/World/pool/props/board", "/World/rov/body", info.camera, info.chase):
        assert stage.GetPrimAtPath(path).IsValid(), path
    assert len(info.lamps) == len(cfg["vehicle"]["lights"]["lamps"])
    surf = UsdGeom.PrimvarsAPI(stage.GetPrimAtPath("/World/pool/water_surface"))
    assert surf.GetPrimvar("doNotCastShadows").Get() is True, "the surface must not shade the hall lights"
    w, h = info.camera_resolution
    assert (w, h) == tuple(cfg["camera"]["resolution"])


def test_the_camera_rides_the_vehicle_and_tilts(built, cfg):
    stage, info = built
    p = np.array([2.0, 3.0, 1.5])
    yaw = math.radians(30.0)
    rot = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]])
    pool_scene.set_vehicle(stage, p, rot)
    pool_scene.set_tilt(stage, 0.0)
    m = _world(stage, info.camera)
    cam_pos = m[:3, 3]
    expect = p + rot @ np.asarray(cfg["vehicle"]["camera"]["position_body"], float)
    assert np.allclose(cam_pos, expect, atol=1e-6)
    look = -m[:3, 2]                                    # a USD camera looks along its -z
    assert np.allclose(look, rot[:, 0], atol=1e-6), "level tilt: the camera looks along the body x"
    pool_scene.set_tilt(stage, 30.0)
    look = -_world(stage, info.camera)[:3, 2]
    assert look[2] == pytest.approx(math.sin(math.radians(30.0)), abs=1e-6), "positive tilt looks up"
    assert np.allclose(_world(stage, info.camera)[:3, 3], expect, atol=1e-6), "the tilt turns about the camera"


def test_lights_dim_with_the_output(built):
    stage, info = built
    pool_scene.set_lights(stage, info, 0.25)
    for path in info.lamps:
        assert UsdLux.SphereLight(stage.GetPrimAtPath(path)).GetIntensityAttr().Get() == pytest.approx(
            0.25 * info.lamp_intensity)


def test_warp_water_matches_the_reference(cfg):
    rng = np.random.default_rng(3)
    rgb = rng.integers(0, 256, (48, 64, 4), dtype=np.uint8)
    dist = rng.uniform(0.0, 12.0, (48, 64)).astype(np.float32)
    dist[0, :5] = np.inf
    dist[1, :5] = 0.0
    post = camera_uw.WaterPost(cfg)
    got = post(rgb, dist)
    ref = camera_uw.apply_numpy(rgb, dist, *camera_uw.coefficients(cfg))
    assert got.shape == (48, 64, 3) and got.dtype == np.uint8
    assert np.abs(got.astype(int) - ref.astype(int)).max() <= 1


def test_the_encoder_writes_a_baseline_h264_file(tmp_path):
    from sim import video
    out = tmp_path / "clip.ts"
    s = video.VideoStream(str(out), 320, 240, 25).start()
    for i in range(50):
        frame = np.zeros((240, 320, 3), np.uint8)
        frame[:, : 6 * i] = 200
        s.push(frame)
        s.thread.join(0.02)                              # let the writer take every frame
    s.close()
    assert out.stat().st_size > 1000
    assert s.stats()["written"] >= 40
    # The SPS NAL units, read from the bytes (the bundled static ffmpeg segfaults when asked to decode in this
    # environment): profile_idc 66 with constraint_set1 = Constrained Baseline, and one per IDR (repeat-headers),
    # i.e. one a second, so a decoder joining the stream late (BlueOS's camera manager) starts within a second.
    data = out.read_bytes()
    sps = [i for i in range(len(data) - 6) if data[i:i + 4] == b"\x00\x00\x01\x67"]
    assert len(sps) >= 2, f"{len(sps)} SPS in 2 s of 25 fps with an IDR every 25 frames"
    assert all(data[i + 4] == 66 and data[i + 5] & 0x40 for i in sps), "not Constrained Baseline"
