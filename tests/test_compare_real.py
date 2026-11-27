"""The real-data comparison's loaders and gain chain (host, numpy + PIL): synthetic inputs, known answers.

These were the untested half of the comparison. Every number in `docs/validation.md`'s sim-vs-real table went
through one of these functions, and two of the surprises in that table turned out to be reading artifacts
rather than properties of a sonar -- so the loaders are now pinned to inputs whose answers are known by
construction.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "physics" / "tools"))
import compare_real as cr                                                                # noqa: E402

from uw import config                                                                    # noqa: E402

RNG = np.random.default_rng(7)


# --------------------------------------------------------------------------------- the pixel law
def test_pixels_to_db_is_the_law_it_documents():
    v = np.array([1.0, 10.0, 100.0], np.float32)
    assert cr.pixels_to_db(v, 1.0) == pytest.approx(20 * np.log10(v), abs=1e-4)
    assert cr.pixels_to_db(v, 0.5) == pytest.approx(40 * np.log10(v), abs=1e-4)


def test_pixels_to_db_floors_zero_instead_of_dropping_it():
    """A real image is 0 wherever the display clipped, which is data. The floor is half an LSB, so the value
    is finite and far below any real sample rather than -inf."""
    out = cr.pixels_to_db(np.array([0.0, 0.4]), 1.0)
    assert np.all(np.isfinite(out))
    assert out[0] == pytest.approx(20 * np.log10(0.5), abs=1e-4)
    assert out[1] == out[0]


# --------------------------------------------------------------------------------- colour map inversion
def test_colormap_lut_recovers_the_index_of_a_colourmapped_ramp(tmp_path):
    import matplotlib
    idx = np.arange(256, dtype=np.uint8)
    rgb = (np.asarray(matplotlib.colormaps["inferno"](idx / 255.0))[:, :3] * 255).astype(np.uint8)
    f = tmp_path / "ramp.png"
    Image.fromarray(np.tile(rgb, (8, 1, 1))).save(f)
    got = cr.read_gray(f, cr.colormap_lut("inferno"))
    # The cube is a 32^3 nearest-neighbour inverse, so a few levels alias; the ramp must still be recovered
    # monotonically and closely, which is all that undoing a display colour map can promise.
    assert got.shape == (8, 256)
    assert np.abs(got[0] - idx).mean() < 4.0
    assert np.corrcoef(got[0], idx)[0, 1] > 0.999


def test_read_gray_without_a_cube_is_plain_luminance(tmp_path):
    """The default path, and the one that produced basin-2's contrast 0.33: a colour-mapped image read as
    luminance is the luminance of the ramp, not the sample the renderer encoded."""
    import matplotlib
    rgb = (np.asarray(matplotlib.colormaps["inferno"](np.linspace(0, 1, 256)))[:, :3] * 255).astype(np.uint8)
    f = tmp_path / "ramp.png"
    Image.fromarray(np.tile(rgb, (4, 1, 1))).save(f)
    lum = cr.read_gray(f)
    idx = np.arange(256)
    # Luminance of a colour ramp is not the index: inferno starts near-black and ends yellow, so the curve
    # is compressive at the bright end. That is the whole mechanism behind basin-2's contrast 0.33.
    assert np.corrcoef(lum[0], idx)[0, 1] < 0.999
    assert np.abs(lum[0] - idx).max() > 20
    assert np.abs(np.diff(lum[0])).max() > 2 * np.abs(np.diff(lum[0])).mean()


# --------------------------------------------------------------------------------- fan geometry
def draw_sector(radius: int, half_angle: float, value, pad: int = 12) -> np.ndarray:
    """A rendered fan with its apex at the bottom centre, the convention `fan_geometry` fits and
    `fan_to_polar` samples: x = apex_x + r sin(b), y = apex_y - r cos(b)."""
    w = 2 * int(np.ceil(radius * np.sin(np.radians(half_angle)))) + 2 * pad
    h = radius + 2 * pad
    x0, y0 = w / 2.0, h - pad - 1.0
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - x0, y0 - yy)
    b = np.degrees(np.arctan2(xx - x0, np.maximum(y0 - yy, 1e-9)))
    inside = (r <= radius) & (np.abs(b) <= half_angle)
    return np.where(inside, value(r / radius, b), 0.0).astype(np.float32), (x0, y0)


def test_fan_geometry_recovers_a_drawn_sector():
    img, (x0, y0) = draw_sector(300, 35.0, lambda rr, b: 200.0)
    g = cr.fan_geometry(img)
    assert g["apex_x"] == pytest.approx(x0, abs=2.0)
    assert g["apex_y"] == pytest.approx(y0, abs=3.0)
    assert g["r_max_px"] == pytest.approx(300, abs=4.0)
    assert g["half_angle_deg"] == pytest.approx(35.0, abs=1.0)


def test_fan_to_polar_recovers_a_planted_radial_ramp():
    """The resampling must put a value back at the range it was drawn at: a ramp in r/r_max comes out as the
    same ramp in the polar row index, which is what makes a range profile off a fan image mean anything."""
    img, _ = draw_sector(400, 30.0, lambda rr, b: 40.0 + 200.0 * rr)
    g = cr.fan_geometry(img)
    polar = cr.fan_to_polar(img, g, 200, 120)
    prof = np.nanmean(polar, 1)
    want = 40.0 + 200.0 * (np.arange(200) + 0.5) / 200
    mid = slice(20, 180)                         # the ends straddle the apex and the arc
    assert np.abs(prof[mid] - want[mid]).max() < 12.0


def test_fan_to_polar_marks_samples_off_the_image_as_missing():
    """The guard that keeps a wrong sector fit from inventing data: samples that land outside the image are
    NaN, not clamped to its edge pixel. A polar grid built from a *correct* fit lands wholly inside, so the
    radius is inflated here to make some samples over-reach."""
    img, _ = draw_sector(300, 20.0, lambda rr, b: 150.0)
    g = cr.fan_geometry(img)
    assert np.isfinite(cr.fan_to_polar(img, g, 120, 200)).all()      # a correct fit: nothing missing
    over = dict(g, r_max_px=g["r_max_px"] * 2.5)
    polar = cr.fan_to_polar(img, over, 120, 200)
    assert np.isfinite(polar).any() and not np.isfinite(polar).all()


# --------------------------------------------------------------------------------- UATD
def write_uatd(root: Path, n: int, rows: int = 400, beams: int = 256, r_max=9.5, fov=120.0, power=1.0):
    (root / "images").mkdir(parents=True)
    (root / "annotations").mkdir(parents=True)
    for i in range(n):
        amp = RNG.rayleigh(size=(rows, beams))               # Rayleigh amplitude == exponential intensity
        v = np.clip((amp / 6.0) ** power * 255.0, 0, 255).astype(np.uint8)
        Image.fromarray(v, mode="L").save(root / "images" / f"{i:05d}.bmp")
        (root / "annotations" / f"{i:05d}.xml").write_text(
            f"<annotation><sonar><range>{r_max}</range><azimuth>{fov}</azimuth><elevation>12</elevation>"
            f"<soundspeed>1468.6</soundspeed><frequency>1200k</frequency></sonar></annotation>")


def test_load_uatd_takes_its_geometry_from_the_xml(tmp_path):
    write_uatd(tmp_path, 2, rows=400, beams=256, r_max=9.5, fov=120.0)
    db, rng, brg = next(iter(cr.load_uatd(tmp_path, 1, 1.0)))
    assert db.shape == (400, 256)
    assert rng[0] == pytest.approx(9.5 / 400 / 2, abs=1e-6)      # bin centres from range 0
    assert rng[-1] == pytest.approx(9.5 * (1 - 1 / 800), abs=1e-6)
    assert brg[0] == pytest.approx(-60 + 120 / 256 / 2, abs=1e-4)
    assert brg[-1] == pytest.approx(60 - 120 / 256 / 2, abs=1e-4)
    assert cr.detect(tmp_path) == "uatd"


@pytest.mark.parametrize("power", [1.0, 0.5])
def test_fit_exponent_recovers_a_planted_pixel_law(tmp_path, power):
    """The fit is what measured p = 1.0-1.1 for two real Gemini recordings. On native polar data, where no
    resampling has touched the speckle, it must return the exponent that was planted."""
    write_uatd(tmp_path, 1, rows=600, beams=400, power=power)
    p = cr.fit_exponent(tmp_path, "uatd", 1.0, "", 0.0, (2.0, 8.0), (-40.0, 40.0))
    assert p == pytest.approx(power, rel=0.12)


def test_fit_exponent_forces_contrast_to_one_and_so_proves_nothing_by_itself(tmp_path):
    """Guards the trap basin-2 fell into: after --fit-pixel-law the contrast is 1 by construction, whatever
    the data. Only the KS distance and the other statistics can still disagree."""
    write_uatd(tmp_path, 1, rows=600, beams=400, power=1.0)
    p = cr.fit_exponent(tmp_path, "uatd", 1.0, "", 0.0, (2.0, 8.0), (-40.0, 40.0))
    db, rng, brg = next(iter(cr.load_uatd(tmp_path, 1, p)))
    from physics import sonar_stats
    assert sonar_stats.background(db, rng, brg, (2.0, 8.0), (-40.0, 40.0))["contrast"] == pytest.approx(1.0, abs=0.02)


# --------------------------------------------------------------------------------- the sim's gain chain
GAIN = {"noise_floor_db": 84.3, "display_db": [0.0, 80.0], "tvg_db_per_decade": 30.0,
        "absorption_db_per_m": 0.23, "min_range": 0.1}


def test_uw_sensor_units_reproduces_the_display_kernel_formula():
    """The same expression `display_kernel` evaluates on the GPU (physics/sonar.py): level + k log10 R + 2 alpha R
    - nf_db. If these drift apart, the comparison silently stops measuring what the sonar displays."""
    rng = np.array([0.05, 0.5, 2.0, 3.5], np.float32)
    db = np.full((4, 3), 100.0, np.float32)
    got = cr.uw_sensor_units(db, rng, GAIN, "tvg", 1.0)
    rr = np.maximum(rng, GAIN["min_range"])[:, None]
    want = np.broadcast_to(100.0 + 30.0 * np.log10(rr) + 2 * 0.23 * rr - 84.3, db.shape)
    assert got.shape == db.shape
    assert got == pytest.approx(want, abs=1e-3)


def test_uw_sensor_units_img_saturates_and_floors_like_an_8_bit_head():
    rng = np.array([2.0], np.float32)
    db = np.array([[0.0, 100.0, 400.0]], np.float32)       # far below the gain, inside it, far above
    img = cr.uw_sensor_units(db, rng, GAIN, "img", 1.0, amp_top_db=40.0)
    floor = cr.pixels_to_db(np.array([0.5]), 1.0)[0]
    ceil = cr.pixels_to_db(np.array([255.0]), 1.0)[0]
    assert img[0, 0] == pytest.approx(floor, abs=1e-3)
    assert img[0, 2] == pytest.approx(ceil, abs=1e-3)
    assert floor < img[0, 1] < ceil


def test_uw_sensor_units_img_is_linear_in_amplitude_not_in_db():
    """A real Oculus or Gemini 8-bit sample is linear in amplitude (fitted p = 1.0-1.1 on two Geminis), while
    our own display_db window is linear in dB. Encoding ours the wrong way compressed 15 dB of sand into 4 dB
    and took the measured speckle contrast from 2.13 to 0.43 -- an artifact of the conversion. So a dB step in
    the level must come out as the same dB step after the round trip, away from the floor and the ceiling."""
    rng = np.array([2.0], np.float32)
    level = np.array([[100.0, 106.0]], np.float32)         # 6 dB apart
    out = cr.uw_sensor_units(level, rng, GAIN, "img", 1.0, amp_top_db=40.0)
    assert float(out[0, 1] - out[0, 0]) == pytest.approx(6.0, abs=0.3)


def test_uw_sensor_units_img_gain_shifts_the_image_without_changing_its_contrast():
    """The gain is a level, not a shape: raising it darkens everything by the same number of dB."""
    rng = np.array([2.0], np.float32)
    level = np.array([[100.0, 103.0, 106.0]], np.float32)
    a = cr.uw_sensor_units(level, rng, GAIN, "img", 1.0, amp_top_db=40.0)
    b = cr.uw_sensor_units(level, rng, GAIN, "img", 1.0, amp_top_db=50.0)
    assert float((a - b).mean()) == pytest.approx(10.0, abs=0.4)
    # The steps survive the shift, but only to within the quantisation: at a 50 dB gain these samples land on
    # pixels 15-31, where one level is ~0.3 dB. An amplitude display losing resolution in the dark is the real
    # behaviour being emulated, not slack in the test.
    assert np.diff(a[0]) == pytest.approx(np.diff(b[0]), abs=0.5)


def test_uw_sensor_units_img_quantisation_is_coarse_in_the_dark():
    """Why the gain has to put the seabed mid-scale: 40 dB below the saturation level, one 8-bit step is
    several dB, so speckle there is destroyed by the encoding rather than measured through it."""
    rng = np.array([2.0], np.float32)
    bright = cr.uw_sensor_units(np.array([[120.0, 120.5]], np.float32), rng, GAIN, "img", 1.0, amp_top_db=50.0)
    dark = cr.uw_sensor_units(np.array([[80.0, 80.5]], np.float32), rng, GAIN, "img", 1.0, amp_top_db=50.0)
    # Near saturation (pixels 155 and 164) the 0.5 dB survives. Forty dB down both samples land on pixel 2,
    # so it is gone entirely -- not merely coarse.
    assert float(np.diff(bright[0])[0]) == pytest.approx(0.5, abs=0.2)
    assert abs(float(np.diff(dark[0])[0]) - 0.5) > 0.4


def test_uw_sensor_units_keeps_the_blind_zone_missing():
    """Rows inside the blind zone are NaN in `polar_db` because the sonar returns nothing there. A gain is
    not data: it must not turn them into a number."""
    rng = np.array([0.02, 2.0], np.float32)
    db = np.array([[np.nan, np.nan], [100.0, 100.0]], np.float32)
    for channel in ("tvg", "img"):
        out = cr.uw_sensor_units(db, rng, GAIN, channel, 1.0, amp_top_db=40.0)
        assert np.isnan(out[0]).all()
        assert np.isfinite(out[1]).all()


def test_uw_gain_falls_back_to_the_datasets_own_config(tmp_path):
    """Datasets written before sim/recorder.py exported the gain chain carry only the noise floor and the
    display window; the rest has to come from the resolved config in the manifest."""
    cfg = config.load()
    (tmp_path / "sonar").mkdir()
    (tmp_path / "manifest.json").write_text(json.dumps({"config": cfg}, default=str))
    (tmp_path / "sonar" / "000000.json").write_text(json.dumps(
        {"sonar": {"noise_floor_db": 84.3, "display_db": [0.0, 80.0]}}))
    g = cr.uw_gain(tmp_path)
    assert g["tvg_db_per_decade"] == pytest.approx(cfg["sonar"]["tvg_db_per_decade"])
    assert g["min_range"] == pytest.approx(config.sonar_mode(cfg)["min_range"])
    assert 0.0 < g["absorption_db_per_m"] < 1.0           # 750 kHz in sea water is ~0.23 dB/m
    assert g["noise_floor_db"] == pytest.approx(84.3)


# --------------------------------------------------------------------------------- the split gate
def test_only_filters_to_a_split_and_passes_everything_without_one(tmp_path):
    files = []
    for i in range(4):
        f = tmp_path / f"{i}.png"
        f.write_bytes(b"")
        files.append(f)
    assert cr.only(files, None) == files
    keep = {files[1].resolve(), files[3].resolve()}
    assert cr.only(files, keep) == [files[1], files[3]]


def holdout_file(tmp_path: Path, root: Path, cal, hold) -> Path:
    man = {"threshold": 0.95, "datasets": {"demo": {
        "root": str(root), "frames": len(cal) + len(hold), "counts": {"calibrate": len(cal), "holdout": len(hold)},
        "split": {"calibrate": cal, "holdout": hold}}}}
    f = tmp_path / "holdout.json"
    f.write_text(json.dumps(man))
    return f


def test_split_files_returns_one_half(tmp_path):
    root = tmp_path / "real"
    (root / "d" / "sonar").mkdir(parents=True)
    f = holdout_file(tmp_path, root, ["d/sonar/a.png", "d/sonar/b.png"], ["d/sonar/c.png"])
    got = cr.split_files(f, root / "d" / "sonar", "holdout")
    assert got == {(root / "d" / "sonar" / "c.png").resolve()}
    assert len(cr.split_files(f, root / "d" / "sonar", "calibrate")) == 2


def test_split_files_refuses_an_unknown_corpus_instead_of_using_all_of_it(tmp_path):
    root = tmp_path / "real"
    (root / "d" / "sonar").mkdir(parents=True)
    (root / "other").mkdir()
    f = holdout_file(tmp_path, root, ["d/sonar/a.png"], ["d/sonar/b.png"])
    with pytest.raises(SystemExit, match="not covered by"):
        cr.split_files(f, root / "other", "holdout")


def test_split_files_refuses_a_corpus_whose_holdout_half_is_empty(tmp_path):
    """A corpus that refused to be split (one continuous recording) must not silently verify on its own
    calibration frames."""
    root = tmp_path / "real"
    (root / "d" / "sonar").mkdir(parents=True)
    f = holdout_file(tmp_path, root, ["d/sonar/a.png", "d/sonar/b.png"], [])
    with pytest.raises(SystemExit, match="no frames in the 'holdout' half"):
        cr.split_files(f, root / "d" / "sonar", "holdout")


# --------------------------------------------------------------------------------- profile averaging
def test_mean_profile_averages_frames_that_share_no_range_axis():
    """UATD's range setting changes from ping to ping. Two frames of the same underlying ramp, sampled on
    different axes, must average to that ramp instead of to one frame's copy of it."""
    grid = np.linspace(0.0, 4.0, 41)
    frames = [{"range_m": list(np.linspace(0, 5, 60)), "mean_db": list(2.0 * np.linspace(0, 5, 60))},
              {"range_m": list(np.linspace(0, 9, 31)), "mean_db": list(2.0 * np.linspace(0, 9, 31))}]
    got = np.array(cr.mean_profile(frames, "mean_db", grid), float)
    assert got == pytest.approx(2.0 * grid, abs=1e-6)


def test_mean_profile_leaves_a_point_no_frame_reaches_as_null():
    grid = np.linspace(0.0, 10.0, 11)
    frames = [{"range_m": [0.0, 1.0, 2.0], "mean_db": [0.0, 1.0, 2.0]}]
    got = cr.mean_profile(frames, "mean_db", grid)
    assert got[0] is not None and got[1] is not None
    assert all(v is None for v in got[3:])


def test_mean_profile_ignores_a_frame_with_no_data():
    grid = np.linspace(0.0, 2.0, 11)
    frames = [{"range_m": [0.0, 1.0, 2.0], "mean_db": [4.0, 4.0, 4.0]},
              {"range_m": [0.0, 1.0, 2.0], "mean_db": [None, None, None]}]
    assert cr.mean_profile(frames, "mean_db", grid) == pytest.approx([4.0] * 11)
