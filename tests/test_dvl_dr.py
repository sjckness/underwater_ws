"""ros/dvl_dr: the arithmetic the dead-reckoning preprocessor does before robot_localization sees it.

Host-only and ROS-free by construction: the maths lives in ``dvl_dr/correct.py``, which imports nothing
but the standard library, so these run everywhere rather than only where ROS 2 Jazzy is. The node that
wraps them is thin on purpose.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "ros" / "dvl_dr"))

from dvl_dr.correct import NO_FIX, correct_lever_arm, cross, depth_pose, has_fix  # noqa: E402


def _cov(*diag):
    c = [0.0] * 36
    for i, v in enumerate(diag):
        c[i * 7] = v
    return c


# ------------------------------------------------------------------------------------- the lever arm

def test_a_stationary_body_with_no_rotation_needs_no_correction():
    v = (0.3, -0.1, 0.05)
    assert correct_lever_arm(v, (0.0, 0.0, 0.0), (0.0, 0.0, -0.12)) == v


def test_the_correction_undoes_exactly_what_the_sensor_model_added():
    """uw.rov.dvl adds ``omega x r`` in FRD; this removes it in FLU. The two must cancel, or the
    estimator integrates a phantom velocity that grows with every turn."""
    from uw.rov import frames as fr
    rng = np.random.default_rng(5)
    for _ in range(50):
        v_body_frd = rng.normal(0, 0.3, 3)
        w_frd = rng.normal(0, 0.5, 3)
        r_frd = rng.normal(0, 0.2, 3)
        v_dvl_frd = v_body_frd + fr.cross3(w_frd, r_frd)          # what the model measures
        # the relay flips the measurement into FLU; the lever arm is given in FLU too
        back = correct_lever_arm(tuple(fr.vec_flu_frd(v_dvl_frd)), tuple(fr.angvel_flu_frd(w_frd)),
                                 tuple(fr.vec_flu_frd(r_frd)))
        assert np.allclose(back, fr.vec_flu_frd(v_body_frd), atol=1e-12)


def test_a_yaw_rate_about_a_vertical_lever_arm_produces_nothing():
    """The shipped mount is straight under the hull, so yaw is the one rate it cannot fake."""
    assert correct_lever_arm((0.0, 0.0, 0.0), (0.0, 0.0, 0.7), (0.0, 0.0, -0.12)) == (0.0, 0.0, 0.0)


def test_a_pitch_rate_about_a_vertical_lever_arm_fakes_surge():
    r_z = -0.12
    w_y = 0.4
    v = correct_lever_arm((w_y * r_z, 0.0, 0.0), (0.0, w_y, 0.0), (0.0, 0.0, r_z))
    assert v == pytest.approx((0.0, 0.0, 0.0), abs=1e-15)


def test_cross_is_the_right_handed_one():
    assert cross((1, 0, 0), (0, 1, 0)) == (0, 0, 1)
    assert cross((0, 1, 0), (0, 0, 1)) == (1, 0, 0)


# -------------------------------------------------------------------------------- the bottom-lock gate

def test_a_report_without_bottom_lock_is_not_a_measurement():
    """It is not a noisier measurement: the beams did not come back. Dropping it makes the filter
    coast on the IMU, which is honest; fusing it with a big covariance would still pull the state."""
    assert has_fix(_cov(1e-4, 1e-4, 1e-4))
    assert not has_fix(_cov(NO_FIX, NO_FIX, NO_FIX))
    assert not has_fix(_cov(1e-4, NO_FIX, 1e-4)), "one dead axis is enough to reject the sample"
    assert not has_fix(_cov(-1.0, -1.0, -1.0)), "-1 means unknown, which is not a fix either"
    assert not has_fix(_cov(0.0, 0.0, 0.0)), "a zero covariance is a bug, not a perfect measurement"


def test_the_gate_agrees_with_what_the_relay_stamps_on_a_lost_lock():
    pytest.importorskip("geometry_msgs.msg", reason="the Isaac ROS bridge's rclpy: python -m uw test")
    pytest.importorskip("marine_acoustic_msgs.msg", reason="host ROS 2 Jazzy with ros-jazzy-marine-acoustic-msgs")
    from uw import config, geometry
    from uw.rov import dvl as dvl_model
    sys.path.insert(0, str(WS / "ros" / "dvl_relay"))
    from dvl_relay.relay import twist_msg

    cfg = config.load()
    d = dvl_model.from_config(cfg, geometry.build_scene(cfg))
    report = d.sample(0.0, np.array([1.5, 1.5, 1.3]), np.eye(3), np.zeros(3), np.zeros(3))
    assert has_fix(twist_msg(report.as_json(), "f").twist.covariance)
    lost = {**report.as_json(), "velocity_valid": False}
    assert not has_fix(twist_msg(lost, "f").twist.covariance)


# ------------------------------------------------------------------------------------------- depth

def test_depth_is_hydrostatic_and_z_is_measured_up_from_the_floor():
    rho, wl, surf = 1026.0, 2.9, 101325.0
    for depth in (0.0, 0.5, 1.0, 2.5):
        p = surf + rho * 9.80665 * depth
        z, back = depth_pose(p, surf, rho, wl)
        assert back == pytest.approx(depth)
        assert z == pytest.approx(wl - depth)


def test_depth_agrees_with_the_frame_convention_used_everywhere_else():
    """uw.rov.frames puts the NED origin on the free surface, so D = water_level - z. The baro path
    must land on the same z or the EKF and the dynamics disagree about where the vehicle is."""
    from uw.rov import frames as fr
    rho, wl, surf = 1026.0, 2.9, 101325.0
    z_true = 1.3
    p = surf + rho * 9.80665 * (wl - z_true)
    z, _ = depth_pose(p, surf, rho, wl)
    assert z == pytest.approx(z_true)
    assert fr.position_tank_to_ned([0.0, 0.0, z], wl)[2] == pytest.approx(wl - z_true)


def test_a_pressure_below_the_surface_value_does_not_put_the_vehicle_in_the_air():
    z, depth = depth_pose(90000.0, 101325.0, 1026.0, 2.9)
    assert depth == 0.0 and z == pytest.approx(2.9)


# ---------------------------------------------------------------------------- the filter's own config

def test_the_ekf_fuses_no_ground_truth_and_no_position():
    """The two properties that make the drift number mean something: nothing from /gt is an input, and
    no x or y position is measured anywhere - the track is reckoned, not observed."""
    import yaml
    cfg = yaml.safe_load((WS / "ros" / "dvl_dr" / "config" / "ekf.yaml").read_text(encoding="utf-8"))
    p = cfg["ekf_filter_node"]["ros__parameters"]
    inputs = {k: v for k, v in p.items() if k in ("imu0", "twist0", "pose0")}
    assert set(inputs) == {"imu0", "twist0", "pose0"}
    assert not any("/gt" in v for v in inputs.values()), "ground truth must never be an estimator input"
    for key in ("imu0_config", "twist0_config", "pose0_config"):
        assert len(p[key]) == 15, f"{key} is robot_localization's 15 flags"
        assert not p[key][0] and not p[key][1], f"{key} must not fuse an x or y position"
    assert p["pose0_config"][2], "the baro is the only thing observing z"
    assert p["twist0_config"][6:9] == [True, True, True], "the DVL is the only thing observing velocity"
    assert p["world_frame"] == p["odom_frame"], "odometry, not localisation: there is no map correction"
    assert not p["two_d_mode"]


# ------------------------------------------------------------------- tools/dvl_drift.py, the scoring

sys.path.insert(0, str(WS / "tools"))
import dvl_drift as DD  # noqa: E402


def test_a_pure_heading_error_gives_the_closed_form_drift():
    """A heading error rotates the whole track instead of lengthening it, so after a straight run of
    L metres the final error is 2 L sin(theta/2), whatever the speed. That is why the tank's unreliable
    magnetometer dominates, and why the tool quotes drift against distance and not against time."""
    n = 200
    t = np.linspace(0.0, 100.0, n)
    truth = np.stack([np.linspace(0.0, 10.0, n), np.zeros(n), np.full(n, 1.3)], 1)
    for deg in (1.0, 5.0, 15.0):
        th = np.radians(deg)
        rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        est = truth.copy()
        est[:, :2] = (rot @ truth[:, :2].T).T
        res = DD.drift(t, truth, t, est)
        assert res["distance_travelled_m"] == pytest.approx(10.0)
        assert res["final_error_m"] == pytest.approx(2 * 10.0 * np.sin(th / 2), rel=1e-9)
        assert res["drift_percent_of_distance"] == pytest.approx(100 * 2 * np.sin(th / 2), rel=1e-9)
        assert res["final_error_z_m"] == pytest.approx(0.0, abs=1e-12)   # the baro-aided axis


def test_drift_is_measured_from_a_common_origin():
    """A dead-reckoned track starts wherever the filter was initialised, so an absolute comparison
    would measure the initialisation instead of the drift."""
    n = 50
    t = np.linspace(0.0, 10.0, n)
    truth = np.stack([np.linspace(0.0, 5.0, n), np.zeros(n), np.full(n, 1.0)], 1)
    assert DD.drift(t, truth, t, truth + np.array([100.0, -7.0, 3.0]))["final_error_m"] == pytest.approx(0.0)


def test_a_velocity_scale_error_drifts_in_proportion_to_distance():
    """The other error the datasheet quotes: +-1.01 % of long-term accuracy is a scale factor, so it
    shows up as a drift percentage equal to the scale error itself."""
    n = 100
    t = np.linspace(0.0, 60.0, n)
    truth = np.stack([np.linspace(0.0, 8.0, n), np.zeros(n), np.ones(n)], 1)
    est = truth.copy()
    est[:, 0] *= 1.0101
    assert DD.drift(t, truth, t, est)["drift_percent_of_distance"] == pytest.approx(1.01, rel=1e-6)


def test_resample_clips_to_the_window_both_sides_were_running():
    t = np.linspace(0.0, 10.0, 11)
    p = np.stack([t, t * 0, t * 0], 1)
    out_t, out_p, keep = DD.resample(t, p, np.array([-5.0, 0.0, 5.0, 10.0, 99.0]))
    assert out_t.tolist() == [0.0, 5.0, 10.0]
    assert np.allclose(out_p[:, 0], [0.0, 5.0, 10.0])
    assert keep.tolist() == [False, True, True, True, False]


def test_too_little_overlap_is_an_error_not_a_number():
    t = np.linspace(0.0, 1.0, 5)
    p = np.zeros((5, 3))
    with pytest.raises(ValueError, match="overlap"):
        DD.drift(t, p, np.array([50.0, 51.0]), np.zeros((2, 3)))


def test_path_length_of_a_straight_run():
    assert DD.path_length(np.stack([np.linspace(0, 3, 7), np.zeros(7), np.zeros(7)], 1)) == pytest.approx(3.0)
    assert DD.path_length(np.zeros((1, 3))) == 0.0


# -------------------------------------------------------- the whole chain, offline (tools/dvl_dr_offline)

sys.path.insert(0, str(WS / "tools"))
import dvl_dr_offline as OFF  # noqa: E402


def _fly(yaw_bias_deg, duration=90.0, seed=1):
    from uw import config, geometry
    cfg = config.load(presets=["basin"], overrides=[
        "sonar.pose_source=trajectory", "sonar.trajectory.preset=lawnmower",
        "sonar.trajectory.height=0.7", "sonar.trajectory.margin=0.35",
        "sonar.trajectory.pitch=0.0", f"seed={seed}"])
    scene = geometry.build_scene(cfg)
    ahrs = OFF.Ahrs(yaw_bias_deg, 0.0, 0.0, seed)
    t, truth, est, lost = OFF.fly(cfg, scene, duration, ahrs, baro_stddev=0.0, gyro_stddev=0.0, seed=seed)
    return DD.drift(t, truth, t, est), truth, lost, len(t)


def test_the_chain_holds_bottom_lock_when_the_beams_clear_the_walls():
    """A 22.5 deg beam at 0.7 m reaches 0.29 m sideways, so a 0.35 m wall margin clears it. The same
    run with the survey's 15 deg pitch does not: the reach becomes 0.47 m. This is a real constraint on
    how a DVL survey must be flown, not a modelling artefact."""
    _, _, lost, n = _fly(0.0)
    assert n > 500
    assert lost < 0.02 * n, f"{lost}/{n} reports lost lock with the margin clearing the beams"


def test_the_sensors_own_drift_lands_near_the_datasheets_accuracy():
    """With a perfect heading, what is left is the DVL: a scale factor drawn once per run (sigma
    1.01 %), a slow bias and per-beam noise. The drift should come out in the same neighbourhood as
    that 1.01 %, which is the one end-to-end check that the error budget is wired up at all. Loose,
    because the noise and bias values are UNVERIFIED."""
    res, _, _, _ = _fly(0.0)
    assert res["distance_travelled_m"] > 8.0
    assert 0.05 < res["drift_percent_of_distance"] < 3.0, res["drift_percent_of_distance"]
    assert res["final_error_z_m"] < 0.05, "the baro pins z, so the vertical must not drift"


@pytest.mark.parametrize("bias_deg", [5.0, 10.0])
def test_a_heading_bias_rotates_the_track_by_the_closed_form(bias_deg):
    """The error of a rigid rotation goes with the NET displacement, not the distance travelled. That
    the measured error lands on 2 sin(theta/2) x net says the heading enters the chain in exactly one
    place and with the right sign - the thing most worth getting wrong silently."""
    res, truth, _, _ = _fly(bias_deg)
    net = float(np.linalg.norm(truth[-1] - truth[0]))
    expected = 2 * np.sin(np.radians(bias_deg) / 2) * net
    assert res["final_error_m"] == pytest.approx(expected, rel=0.15), (res["final_error_m"], expected)


def test_a_heading_bias_makes_things_worse_monotonically():
    errs = [_fly(b)[0]["final_error_m"] for b in (0.0, 2.0, 5.0)]
    assert errs == sorted(errs), errs
