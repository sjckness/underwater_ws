"""vehicles.common.dvl: the DVL A50 model (host, numpy, no GPU and no ROS).

Two kinds of assertion live here. The exact ones pin what the geometry and the algebra must do - beam
directions, the least-squares recovery of a known velocity, the lever arm, quantisation, determinism -
and they are exact because nothing in them is uncertain. The loose ones pin behaviour whose scale comes
from an UNVERIFIED config value (the per-sample noise, the bias, the echo thresholds): there the job is
to catch a sign error or a dead code path, not to pin a number nobody has measured.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from uw import config
from physics import geometry
from vehicles.common import dvl as D

LEVEL = np.eye(3)                        # body FLU -> tank ENU for a level vehicle heading +x
MID = np.array([1.5, 1.5, 1.3])          # mid-tank, about 1 m over the sand


@pytest.fixture(scope="module")
def scene():
    return geometry.build_scene(config.load())


def make(scene, **overrides):
    """A DVL from the shipped config with ``--set``-style overrides."""
    cfg = config.load(overrides=[f"{k}={v}" for k, v in overrides.items()])
    return D.from_config(cfg, scene)


@pytest.fixture(scope="module")
def clean(scene):
    """Noise and dropout off: the model reduces to its geometry and its algebra."""
    return make(scene, **{"dvl.noise": "false", "dvl.dropout": "false"})


# ------------------------------------------------------------------------------------ beam geometry

def test_beams_are_unit_vectors_at_the_configured_tilt_and_azimuth(clean):
    p = clean.p
    b = clean.beams
    assert b.shape == (4, 3)
    assert np.allclose(np.linalg.norm(b, axis=1), 1.0)
    assert np.allclose(np.degrees(np.arccos(b[:, 2])), p.tilt_deg)        # tilt from the instrument z
    assert np.all(b[:, 2] > 0)                                            # FRD: every beam points down
    az = np.degrees(np.arctan2(b[:, 1], b[:, 0])) % 360.0
    assert np.allclose(np.sort(az), np.sort(np.asarray(p.azimuths_deg) % 360.0))


def test_beam_vectors_are_not_the_third_party_drivers_swapped_ones(clean):
    """``ndahn/dvl_a50`` hardcodes (+-0.65328, +-0.65328, 0.38268), which is 22.5 deg above the
    HORIZONTAL - sin and cos swapped. The config takes only the azimuths from that driver."""
    b = clean.beams
    assert np.allclose(np.abs(b), [[0.27060, 0.27060, 0.92388]] * 4, atol=1e-5)
    assert not np.allclose(np.abs(b), [[0.65328, 0.65328, 0.38268]] * 4, atol=1e-3)


def test_the_instrument_frame_is_frd_in_the_tank(clean):
    """Beam 0 is at azimuth 45 deg, forward and to starboard. For a level vehicle heading +x in the
    tank's ENU, starboard is -y and down is -z: the one place FRD meets ENU, through vehicles.common.frames."""
    origin, dirs = clean.geometry_at(MID, LEVEL)
    assert np.allclose(origin, MID + np.array([0.0, 0.0, -clean.mount.position_frd_m[2]]))
    assert dirs[0][0] > 0 and dirs[0][1] < 0                  # forward, starboard
    assert np.all(dirs[:, 2] < 0)                             # all four point at the bottom
    assert np.allclose(np.linalg.norm(dirs, axis=1), 1.0)


# -------------------------------------------------------------------------------- the velocity solve

@pytest.mark.parametrize("v", [(0.0, 0.0, 0.0), (0.3, -0.1, 0.05), (-0.2, 0.4, -0.15), (1.0, 0.0, 0.0)])
def test_a_known_velocity_comes_back_exactly(clean, v):
    r = clean.sample(0.0, MID, LEVEL, np.asarray(v), np.zeros(3))
    assert r.velocity_valid and r.num_good_beams == 4
    assert np.max(np.abs(r.velocity - np.asarray(v))) <= clean.p.resolution_mps


def test_the_beam_velocities_are_the_projection_of_the_instrument_velocity(clean):
    v = np.array([0.25, -0.15, 0.08])
    r = clean.sample(0.0, MID, LEVEL, v, np.zeros(3))
    assert np.allclose(r.beam_velocity, clean.beams @ v)


def test_the_lever_arm_is_applied_at_the_source(clean):
    """v_dvl = v_body + omega x r. With r along the instrument's z, a yaw rate produces nothing and a
    pitch rate produces exactly w_y * r_z of surge."""
    rz = float(clean.mount.position_frd_m[2])
    assert rz > 0, "this test needs a DVL mounted off the body origin"
    yaw = clean.sample(0.0, MID, LEVEL, np.zeros(3), np.array([0.0, 0.0, 0.5]))
    assert np.allclose(yaw.velocity, 0.0, atol=clean.p.resolution_mps)
    pitch = clean.sample(0.0, MID, LEVEL, np.zeros(3), np.array([0.0, 0.4, 0.0]))
    assert pitch.velocity[0] == pytest.approx(0.4 * rz, abs=clean.p.resolution_mps)
    assert np.allclose(pitch.velocity[1:], 0.0, atol=clean.p.resolution_mps)


def test_the_output_is_quantised_to_the_datasheet_resolution(scene):
    d = make(scene)
    r = d.sample(0.0, MID, LEVEL, np.array([0.31415926, -0.0271828, 0.1]), np.zeros(3))
    step = d.p.resolution_mps
    assert np.allclose(r.velocity / step, np.round(r.velocity / step))


def test_the_altitude_is_the_mean_beam_range_along_z(clean):
    r = clean.sample(0.0, MID, LEVEL, np.zeros(3), np.zeros(3))
    expected = float(np.mean(r.beam_range[r.beam_valid])) * math.cos(clean.p.tilt_rad)
    assert r.altitude == pytest.approx(expected)
    geometric = MID[2] - clean.mount.position_frd_m[2] - float(clean.scene.sand.height(MID[0], MID[1]))
    assert r.altitude == pytest.approx(geometric, abs=0.15)     # the sand is not flat under the beams


# ------------------------------------------------------------------------------- loss of bottom lock

def test_a_beam_pointing_up_never_returns(clean):
    """Rolled past 90 deg the beams look at the surface, which is not an occluder."""
    upside_down = np.diag([1.0, -1.0, -1.0])                   # roll 180 deg, FLU -> ENU
    r = clean.sample(0.0, MID, upside_down, np.zeros(3), np.zeros(3))
    assert r.num_good_beams == 0 and not r.velocity_valid
    assert not np.isfinite(r.altitude)


def test_fewer_than_three_beams_is_no_fix(clean):
    """Against the +x wall the outward beams leave the tank footprint, and the walls are not occluders,
    so they simply do not come back."""
    found = False
    for x in np.linspace(2.99, 2.70, 30):
        r = clean.sample(0.0, np.array([x, 1.5, 0.55]), LEVEL, np.zeros(3), np.zeros(3))
        if r.num_good_beams < 3:
            found = True
            assert not r.velocity_valid
            assert np.array_equal(r.covariance, np.full((3, 3), -1.0))
        elif found:
            break
    assert found, "no pose near the wall lost enough beams: the footprint test is not biting"


def test_the_minimum_altitude_gate_bites(clean):
    """0.05 m is the published bottom of the envelope and the one end the tank actually reaches."""
    sand = float(clean.scene.sand.height(MID[0], MID[1]))
    rz = float(clean.mount.position_frd_m[2])
    high = clean.sample(0.0, np.array([MID[0], MID[1], sand + rz + 0.30]), LEVEL, np.zeros(3), np.zeros(3))
    assert high.velocity_valid
    low = clean.sample(0.0, np.array([MID[0], MID[1], sand + rz + 0.01]), LEVEL, np.zeros(3), np.zeros(3))
    assert low.num_good_beams == 0 and not low.velocity_valid


def test_above_the_speed_envelope_the_lock_is_lost(clean):
    fast = clean.p.max_speed_mps * 1.5
    r = clean.sample(0.0, MID, LEVEL, np.array([fast, 0.0, 0.0]), np.zeros(3))
    assert not r.velocity_valid


def test_a_beam_can_land_on_the_pipe_instead_of_the_sand(scene):
    """The exposed pipe stands proud of the sand, so a beam over it ranges short. The failure this
    guards is a model that only ever sees the height field."""
    d = make(scene, **{"dvl.noise": "false", "dvl.dropout": "false"})
    px = float(scene.pipe_axis["x"])
    hits = []
    for dx in np.linspace(-0.45, 0.45, 25):
        origin, dirs = d.geometry_at(np.array([px + dx, 1.5, 1.1]), LEVEL)
        _, _, roles = geometry.ray_bottom(scene, origin, dirs)
        hits += [r for r in roles if r == "pipe"]
    assert hits, "no beam ever landed on the pipe"


# ------------------------------------------------------------------------- noise, bias, determinism

def test_two_runs_of_the_same_scenario_are_bit_identical(scene):
    """vehicles.common is deterministic by design (tests/rov/test_dynamics.py) and a sensor must not break that:
    every error term is a pure function of the seed and the sample index or the simulated time."""
    a, b = make(scene), make(scene)
    v = np.array([0.2, 0.1, -0.05])
    for k in range(8):
        ra = a.sample(0.1 * k, MID, LEVEL, v, np.zeros(3))
        rb = b.sample(0.1 * k, MID, LEVEL, v, np.zeros(3))
        assert np.array_equal(ra.velocity, rb.velocity)
        assert np.array_equal(ra.beam_velocity, rb.beam_velocity)
        assert ra.fom == rb.fom


def test_the_bias_does_not_depend_on_which_samples_were_taken(scene):
    """A recursive bias would drift apart as soon as a report was skipped; a fixed random spectrum does
    not. This is what lets the schedule vary with the altitude without touching reproducibility."""
    every, sparse = make(scene, **{"dvl.beam_noise_mps": 0.0}), make(scene, **{"dvl.beam_noise_mps": 0.0})
    v = np.array([0.2, 0.1, -0.05])
    for k in range(10):
        every.sample(0.1 * k, MID, LEVEL, v, np.zeros(3))
    one = sparse.sample(0.9, MID, LEVEL, v, np.zeros(3))
    last = every.sample(0.9, MID, LEVEL, v, np.zeros(3))
    assert np.allclose(one.beam_velocity, last.beam_velocity)


def test_noise_moves_the_answer_but_not_far(scene):
    d = make(scene)
    v = np.array([0.3, -0.1, 0.05])
    err = np.array([d.sample(0.05 * k, MID, LEVEL, v, np.zeros(3)).velocity - v for k in range(200)])
    assert np.any(np.abs(err) > d.p.resolution_mps), "noise is on but nothing moved"
    assert np.max(np.abs(err)) < 50 * d.p.beam_noise_mps        # loose: the scale is UNVERIFIED
    assert np.max(np.abs(err.mean(0))) < 10 * d.p.beam_noise_mps


def test_the_scale_factor_is_fixed_for_a_run_and_changes_with_the_seed(scene):
    """+-1.01 % is a long-term accuracy, so it is one draw per run, not per ping."""
    d = make(scene, **{"dvl.beam_noise_mps": 0.0, "dvl.bias_sigma_mps": 0.0})
    v = np.array([1.0, 0.0, 0.0])
    s = [d.sample(0.1 * k, MID, LEVEL, v, np.zeros(3)).velocity[0] for k in range(5)]
    assert np.allclose(s, s[0], atol=d.p.resolution_mps)
    other = D.from_config(config.load(overrides=["seed=12345", "dvl.beam_noise_mps=0.0",
                                                 "dvl.bias_sigma_mps=0.0"]),
                          geometry.build_scene(config.load(overrides=["seed=12345"])))
    assert other._scale != d._scale


def test_performance_is_the_same_model_with_a_tighter_scale(scene):
    """The Performance tier is a software licence on the same hardware, so it is one knob."""
    std = make(scene, **{"dvl.version": "standard"})
    perf = make(scene, **{"dvl.version": "performance"})
    assert perf.p.scale_sigma < std.p.scale_sigma
    assert perf.beams.tolist() == std.beams.tolist()


# ------------------------------------------------------------------------------- figure of merit

def test_the_fom_rises_when_the_beams_degrade(scene):
    quiet = make(scene, **{"dvl.beam_noise_mps": 0.0005})
    loud = make(scene, **{"dvl.beam_noise_mps": 0.02})
    v = np.array([0.2, 0.0, 0.0])
    a = np.median([quiet.sample(0.1 * k, MID, LEVEL, v, np.zeros(3)).fom for k in range(20)])
    b = np.median([loud.sample(0.1 * k, MID, LEVEL, v, np.zeros(3)).fom for k in range(20)])
    assert b > a


def test_the_covariance_is_symmetric_positive_definite_when_there_is_a_fix(scene):
    d = make(scene)
    r = d.sample(0.0, MID, LEVEL, np.array([0.2, 0.0, 0.0]), np.zeros(3))
    assert r.velocity_valid
    assert np.allclose(r.covariance, r.covariance.T)
    assert np.all(np.linalg.eigvalsh(r.covariance) > 0)
    assert r.fom == pytest.approx(math.sqrt(float(np.linalg.eigvalsh(r.covariance).max())))


# ----------------------------------------------------------------------------------- the schedule

def test_the_report_rate_stays_inside_the_published_envelope(scene):
    d = make(scene)
    p = d.p
    fired = []
    t = 0.0
    while t < 20.0:
        if d.due(t):
            fired.append(t)
            d.sample(t, MID, LEVEL, np.zeros(3), np.zeros(3))
        t += 1.0 / 240.0                    # the sim loop's substep
    gaps = np.diff(fired)
    assert len(fired) > 50
    assert gaps.min() >= 1.0 / p.rate_hz_max * (1.0 - p.rate_jitter) - 1e-2
    assert gaps.max() <= 1.0 / p.rate_hz_min * (1.0 + p.rate_jitter) + 1e-2


def test_the_rate_falls_as_the_altitude_grows(scene):
    d = make(scene, **{"dvl.rate_jitter": 0.0})
    assert d._period(0.0) == pytest.approx(1.0 / d.p.rate_hz_max)
    assert d._period(d.p.rate_altitude_m * 2) == pytest.approx(1.0 / d.p.rate_hz_min)
    assert d._period(1.0) > d._period(0.5)


# ----------------------------------------------------------------------------- the report and meta

def test_the_report_json_has_the_devices_fields(scene):
    d = make(scene)
    j = d.sample(0.0, MID, LEVEL, np.array([0.2, 0.0, 0.0]), np.zeros(3)).as_json()
    assert set(j) == {"time", "vx", "vy", "vz", "fom", "covariance", "altitude", "transducers",
                      "velocity_valid", "status", "time_of_validity", "time_of_transmission",
                      "format", "type"}
    assert j["format"] == "json_v3.3" and j["type"] == "velocity"
    assert len(j["transducers"]) == 4
    assert [t["id"] for t in j["transducers"]] == [0, 1, 2, 3]          # zero-based, as the protocol
    assert set(j["transducers"][0]) == {"id", "velocity", "distance", "rssi", "nsd", "beam_valid"}
    assert np.shape(j["covariance"]) == (3, 3)


def test_the_report_json_is_serialisable(scene):
    import json
    d = make(scene)
    for k in range(3):
        json.dumps(d.sample(0.1 * k, MID, LEVEL, np.array([0.2, 0.0, 0.0]), np.zeros(3)).as_json())
    json.dumps(d.meta())


def test_the_latched_meta_leaks_no_ground_truth(scene):
    """docs/ros_hil.md 2.5 rule 1: a latched metadata topic is the worst place for truth, because a node
    that never asked for it receives it on join. The DVL's meta describes the instrument only."""
    import json
    text = json.dumps(make(scene).meta()).lower()
    for word in ("pose", "position_tank", "leak", "plume", "truth", "label", "mask", "sand", "rock"):
        assert word not in text, f"'{word}' is in the latched DVL meta"


# ------------------------------------------------------------------------------------- validation

@pytest.mark.parametrize("override, match", [
    ("dvl.beams.tilt_deg=95", "tilt_deg"),
    ("dvl.beams.azimuths_deg=[0, 180]", "azimuths_deg"),             # three beams at least (M12: N beams)
    ("dvl.beams.tilt_deg=[22.5, 22.5]", "tilt_deg"),                 # one tilt per beam, or one for all
    ("dvl.version=deluxe", "version"),
    ("dvl.rate_hz=[15, 2]", "rate_hz"),
    ("dvl.min_altitude_m=60", "min_altitude_m"),
])
def test_a_value_outside_the_datasheet_is_refused(override, match):
    with pytest.raises(config.ConfigError, match=match):
        D.params_from_config(config.load(overrides=[override]))


def test_disabled_builds_nothing(scene):
    assert D.from_config(config.load(overrides=["dvl.enabled=false"]), scene) is None


# ------------------------------------------------------------------- the sim loop, without Isaac

def test_the_loop_block_runs_end_to_end_over_a_trajectory(scene):
    """Everything sim/main.py's DVL block does, at the loop's own substep, with no Isaac and no GPU:
    BodyMotion -> due() -> sample() -> recorder.dvl_message. What this would catch is the thing a host
    test otherwise cannot - a frame or an argument mismatch between the pieces."""
    import json

    from vehicles.common import physics_udp
    from sim import recorder
    cfg = config.load(overrides=["sonar.pose_source=trajectory", "sonar.trajectory.preset=approach"])
    from sim import pose as pose_mod
    motion = pose_mod.sensor_pose(cfg, scene)
    d = D.from_config(cfg, scene)
    bm = D.BodyMotion(motion, *physics_udp.mount(cfg))
    assert not bm.exact, "a kinematic trajectory has no dynamics: the twist must be differenced"

    dt = 1.0 / 240.0                       # sim/main.py: one tick is one bubble substep
    reports, x, truth = [], [], []
    for k in range(1, int(30.0 / dt) + 1):
        t_loop = k * dt
        if not d.due(t_loop):
            continue
        p_body, rot_body, v_frd, w_frd = bm.at(t_loop)
        r = d.sample(t_loop, p_body, rot_body, v_frd, w_frd)
        msg = recorder.dvl_message(r, len(reports), cfg, d.meta())
        json.dumps(msg.meta)               # the ROS path and the FileSink both need this to hold
        reports.append(r)
        x.append(float(p_body[0]))
        truth.append(np.asarray(v_frd, float))

    assert len(reports) > 60, f"only {len(reports)} reports in 30 s of sim time"
    # Sample by sample against the twist the sensor was actually given, not against the trajectory's
    # nominal speed: the approach preset finishes its run after a few seconds and then holds, so a
    # median over 30 s is a hover, and comparing to sonar.trajectory.speed would test the schedule.
    fixed = [(v, r) for v, r in zip(truth, reports) if r.velocity_valid]
    err = np.array([r.velocity - v for v, r in fixed])
    assert np.max(np.abs(err)) < 0.05, float(np.max(np.abs(err)))
    assert np.max(np.abs(err.mean(0))) < 0.01, err.mean(0).tolist()
    # and it does fly at the configured speed while it is moving
    nominal = float(config.get(cfg, "sonar.trajectory.speed"))
    speeds = np.array([np.linalg.norm(v) for v, _ in fixed])
    assert abs(float(np.percentile(speeds, 90)) - nominal) < 0.03, float(np.percentile(speeds, 90))
    # the altitude tracks the trajectory's configured height over the floor
    assert 0.3 < float(np.median([r.altitude for _, r in fixed])) < 2.0

    # The approach preset starts the body at the -x wall, where the two aft beams leave the tank
    # footprint - and the walls are not occluders, so they do not come back at all. That is the real
    # failure mode, so pin it rather than demanding lock everywhere: the losses are all near the wall,
    # they all leave exactly two beams, and the lock holds once the vehicle is clear of it.
    lost = [(xi, r) for xi, r in zip(x, reports) if not r.velocity_valid]
    kept = [xi for xi, r in zip(x, reports) if r.velocity_valid]
    assert lost and kept
    assert max(xi for xi, _ in lost) < min(kept), "a lost lock in open water would be a bug"
    assert min(kept) < 0.6, f"lock regained only at x = {min(kept):.2f} m: the gate is too harsh"
    assert {r.num_good_beams for _, r in lost} == {2}, "two beams point at the wall, the other two down"
    for _, r in lost:
        assert np.array_equal(r.covariance, np.full((3, 3), -1.0))
        assert not np.isfinite(r.fom)


# -------------------------------------------------------------------- N beams (M12 P5): the A50 unchanged

GOLDEN_A50 = __import__("pathlib").Path(__file__).with_name("data") / "a50_reports_golden.json"
GOLDEN_POSES = [((5.0, 5.0, 2.5), (0, 0, 0)), ((3.0, 6.0, 1.0), (5, -3, 40)), ((9.6, 5.0, 2.0), (0, 10, 90)),
                ((6.5, 6.5, 0.6), (-4, 2, 200)), ((5.0, 5.0, 4.9), (0, 0, 10)), ((0.4, 0.4, 0.3), (20, 0, 135)),
                ((5.0, 5.0, 2.0), (180, 0, 0)), ((5.0, 5.0, 0.16), (0, 0, 0)), ((9.9, 9.9, 0.5), (0, 30, 45))]


def test_the_a50_reports_did_not_change():
    """The pool's A50, 72 reports over poses that lock, lose beams, look up and sit below the minimum
    altitude: bit for bit what the 4-beam model gave before it learnt N beams (tests/rov/data/)."""
    import json
    from physics.pool_geometry import PoolBottom
    from vehicles.common import frames as F
    gold = json.loads(GOLDEN_A50.read_text())["pool"]
    cfg = config.load(config.POOL_CONFIG, presets=["pool10"],          # the tank the golden was written in,
                      overrides=["pool.objects=[]"])                    #   before the COCO objects
    d = D.from_config(cfg, PoolBottom(cfg))
    rng = np.random.default_rng(11)
    t = 0.0
    hexes = lambda a: [float(x).hex() for x in np.asarray(a, float).ravel()]        # noqa: E731
    for k, g in enumerate(gold):
        pos, rpy = GOLDEN_POSES[k % len(GOLDEN_POSES)]
        t += 0.13
        r = d.sample(t, np.array(pos), F.rot_from_rpy_deg(rpy), rng.normal(0, 0.3, 3), rng.normal(0, 0.2, 3))
        for key, val in (("velocity", r.velocity), ("covariance", r.covariance), ("fom", [r.fom]),
                         ("altitude", [r.altitude]), ("beam_range", r.beam_range), ("beam_velocity", r.beam_velocity),
                         ("rssi", r.rssi_dbm), ("beam_unit_vec", r.beam_unit_vec)):
            assert hexes(val) == g[key], (k, key)
        assert (r.velocity_valid, r.num_good_beams, [bool(x) for x in r.beam_valid]) == (g["valid"], g["n_good"],
                                                                                          g["beam_valid"]), k
        assert json.loads(json.dumps(r.as_json())) == g["json"], k


def test_three_beams_need_all_three_and_tilts_may_differ():
    """A three-beam DVL (the Nucleus1000's layout, M12 P5): every beam must return, and a per-beam tilt reaches
    the beam axes and the altitude."""
    from physics.pool_geometry import PoolBottom
    cfg = config.load(config.POOL_CONFIG, overrides=["dvl.beams.azimuths_deg=[0, 120, 240]",
                                                     "dvl.beams.tilt_deg=[25, 25, 30]", "dvl.beams.min_good=3",
                                                     "dvl.noise=false", "dvl.dropout=false"])
    d = D.from_config(cfg, PoolBottom(cfg))
    assert d.beams.shape == (3, 3)
    assert np.degrees(np.arccos(d.beams[:, 2])) == pytest.approx([25.0, 25.0, 30.0])
    v = np.array([0.2, -0.1, 0.05])
    r = d.sample(0.0, np.array([5.0, 5.0, 2.0]), LEVEL, v, np.zeros(3))
    assert r.velocity_valid and r.num_good_beams == 3 and len(r.as_json()["transducers"]) == 3
    assert np.allclose(r.velocity, D.Mount(np.zeros(3), np.eye(3)).rot_body_to_dvl @ v, atol=2e-4)
    assert r.altitude == pytest.approx(2.0 - 0.14, abs=1e-6)      # the mount is 0.14 m below the CG
    # one beam on a wall at a grazing angle and lost: no fix with 3 of 3 required
    near = d.sample(0.1, np.array([0.12, 5.0, 3.5]), LEVEL, v, np.zeros(3))
    assert near.num_good_beams == 3 or not near.velocity_valid
    with pytest.raises(config.ConfigError, match="min_good"):
        D.params_from_config(config.load(config.POOL_CONFIG, overrides=["dvl.beams.min_good=5"]))
