"""uw.rov.dynamics: the physics sanity set (a)-(g) of M8a step 1, plus the structural checks.

The numbers these tests assert are deliberately loose where the underlying parameter is UNVERIFIED (see
config/rov_bluerov2.yaml): the job here is to catch a sign error, a wrong moment arm or an unstable
integrator, not to pin a coefficient nobody has measured. ``tools/rov_sanity.py`` prints and plots the same
quantities with their actual values, which is what the parameters get judged on.

Free decay in roll and pitch is a damped OSCILLATION, not a monotone decay, because the centre of buoyancy is
above the centre of gravity. So those two DOF are tested as oscillators (period, decreasing envelope) and the
monotone-decay requirement is applied to them with the CB offset removed.
"""
from __future__ import annotations

import logging
from dataclasses import replace

import numpy as np
import pytest

from uw.rov import dynamics as D
from uw.rov import frames as F

WL = 2.9                     # config/tank.yaml tank.water_level
TANK = (3.0, 3.0, 3.0)
SAND = 0.3                   # config/tank.yaml sand.thickness
MID_TANK_NED = F.position_tank_to_ned([1.5, 1.5, 1.5], WL)

FWD = np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])          # pure surge
SWAY = np.array([-1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0])       # pure sway
YAW = np.array([1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0])        # pure yaw
UP = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0])           # pure heave, upward
DOWN = -UP
ROLL = np.array([0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0])       # pure roll
PITCH = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0])      # pure pitch
ZERO = np.zeros(8)


@pytest.fixture(scope="module")
def vehicle() -> D.Vehicle:
    return D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)


@pytest.fixture(scope="module")
def params(vehicle) -> D.Params:
    return vehicle.params


def free(vehicle, *, neutral=False, no_restoring=False, no_damping=False) -> D.Vehicle:
    """The same vehicle with the soft bound off and, optionally, one effect removed, so a test can isolate
    exactly one term. Nothing here changes the shipped parameters."""
    p = vehicle.params
    if neutral:
        p = replace(p, buoyancy_n=p.weight_n)
    if no_restoring:
        p = replace(p, r_b_frd_m=np.zeros(3))
    if no_damping:
        p = replace(p, d_lin=np.zeros(6), d_quad=np.zeros(6))
    return D.Vehicle(p, water_level=vehicle.water_level, bounds=None)


def at_tank(pos_tank, rpy_enu_deg=(0.0, 0.0, 0.0), n: int = 8) -> D.State:
    """A rest state placed by tank coordinates and ENU/FLU attitude, converted once through ``frames``."""
    p_ned, q_ned = F.pose_tank_to_ned(pos_tank, F.quat_from_rpy_deg(rpy_enu_deg), WL)
    return D.State.rest(n, position_ned=p_ned, quat=q_ned)


def steady_state(vehicle, cmd, magnitude: float = 40.0, **kw) -> D.State:
    """A rest state whose thruster forces are already at an EQUAL-MAGNITUDE version of the sign pattern, so a
    test can look at the instantaneous acceleration without waiting out the first-order lag and without the
    T200's forward/reverse asymmetry leaking into another DOF (see
    ``test_the_t200_asymmetry_couples_a_symmetric_command``)."""
    s = at_tank([1.5, 1.5, 1.5], **kw)
    return replace(s, thrust_n=np.asarray(cmd, float) * magnitude)


def half_life(t, x) -> float:
    """The simulated time at which |x| first falls to half its initial value, by linear interpolation."""
    x = np.abs(np.asarray(x, float))
    target = 0.5 * x[0]
    below = np.flatnonzero(x <= target)
    if below.size == 0:
        return float("inf")
    i = int(below[0])
    if i == 0:
        return 0.0
    x0, x1, t0, t1 = x[i - 1], x[i], t[i - 1], t[i]
    return float(t0 + (x0 - target) / max(x0 - x1, 1e-18) * (t1 - t0))


# ============================================================================== structure and parameters

def test_parameters_load_and_are_self_consistent(params):
    assert params.model == "BlueROV2 Heavy"
    assert params.frame == "vectored_6dof", "docs/ros_hil.md 6.3 decides the Heavy / Vectored_6DOF frame"
    assert params.n_thrusters == 8
    assert params.rate_hz >= 400.0, "docs/ros_hil.md 3.2 requires the physics thread at >= 400 Hz"
    assert np.isclose(params.weight_n, params.mass_kg * D.G, rtol=1e-3)
    assert params.net_buoyancy_n > 0.0, "the BlueROV2 is designed slightly positively buoyant"


def test_the_published_moment_arms_come_out_of_the_geometry(params):
    """The four mounting numbers are believed because they reproduce the published allocation: yaw arm
    0.1888 m from the horizontal four, roll 0.218 m and pitch 0.120 m from the vertical four. If someone
    edits a thruster position, this is what notices."""
    t = params.allocation
    assert np.allclose(np.abs(t[5, :4]), 0.1888, atol=5e-4), f"yaw arms {t[5, :4]}"
    assert np.allclose(np.abs(t[3, 4:]), 0.218, atol=5e-4), f"roll arms {t[3, 4:]}"
    assert np.allclose(np.abs(t[4, 4:]), 0.120, atol=5e-4), f"pitch arms {t[4, 4:]}"
    assert np.allclose(t[3:5, :4], 0.0, atol=1e-12), "horizontal thrusters make no roll or pitch moment"
    assert np.allclose(t[:3, 4:][:2], 0.0, atol=1e-12), "vertical thrusters make no surge or sway force"


def test_the_frame_has_pitch_authority(params):
    """The reason for choosing the Heavy at all (docs/ros_hil.md 6.3): the 6-thruster Vectored frame has an
    all-zero pitch column, and the v0.2 dataset views need +22 to +45 deg of nose-down pitch."""
    pitch_row = params.allocation[4, :]
    assert np.abs(pitch_row).max() > 0.1, "a frame with no pitch authority cannot hold the dataset's views"
    assert np.linalg.matrix_rank(params.allocation) == 6, "all six DOF must be independently actuable"


@pytest.mark.parametrize("cmd,dof", [(FWD, 0), (SWAY, 1), (UP, 2), (ROLL, 3), (PITCH, 4), (YAW, 5)])
def test_each_axis_thrust_pattern_is_decoupled(params, cmd, dof):
    """Each of the six sign patterns excites exactly one row of the wrench, when the thrust MAGNITUDES are
    equal. That is the Vectored_6DOF property of the geometry, and it is what makes test (d) meaningful.

    Stated in thrust, not in command, because the T200 is not symmetric - see the next test.
    """
    tau = params.allocation @ (cmd * 40.0)
    assert abs(tau[dof]) > 1e-3, f"DOF {dof} not excited: {tau}"
    others = np.delete(np.abs(tau), dof)
    assert others.max() < 1e-9 * max(abs(tau[dof]), 1.0) + 1e-9, f"cross-coupling in {tau}"


@pytest.mark.parametrize("cmd,dof,leak_dof", [(SWAY, 1, 0), (ROLL, 3, 2), (PITCH, 4, 2), (YAW, 5, 0)])
def test_the_t200_asymmetry_couples_a_symmetric_command(params, cmd, dof, leak_dof):
    """A real and load-bearing property, not a defect: the T200 gives 51.5 N forward and only 40.2 N in
    reverse, so a command pattern with mixed signs does NOT produce a pure wrench.

    A symmetric roll or pitch command (the vertical four) leaks 22.6 N = 2 (51.5 - 40.2) of heave; a
    symmetric sway or yaw command (the horizontal four, mounted at 45 deg) leaks 16.0 N of surge. Open loop
    the vehicle therefore sinks while it rolls and creeps forward while it turns; ArduSub's mixer and its
    closed loop are what remove this. Any test that assumes a pure wrench has to set the thrusts, not the
    commands.
    """
    tau = params.allocation @ params.t200.thrust_of_command(cmd)
    assert abs(tau[dof]) > 1e-3
    assert abs(tau[leak_dof]) > 1.0, f"expected a real asymmetry leak in DOF {leak_dof}, got {tau}"
    expected = 2.0 * (params.t200.thrust_fwd_max_n - params.t200.thrust_rev_max_n)
    scale = 1.0 if leak_dof == 2 else np.sqrt(2.0) / 2.0
    assert np.isclose(abs(tau[leak_dof]), expected * scale, rtol=1e-6), f"leak {tau[leak_dof]:.3f} N"


def test_coriolis_is_skew_symmetric(params):
    """Fossen's C must satisfy nu' C(nu) nu == 0; the energy test below is only meaningful if it does."""
    rng = np.random.default_rng(7)
    for _ in range(20):
        nu = rng.normal(size=6)
        c = D.coriolis(params, nu)
        assert np.allclose(c, -c.T, atol=1e-12)
        assert abs(nu @ c @ nu) < 1e-12
        assert np.allclose(c @ nu, D.coriolis_nu(params, nu)), "the product form must match the matrix"


def test_restoring_is_zero_upright_and_righting_when_tilted(params):
    """Upright: weight and buoyancy make no moment. Tilted: the moment opposes the tilt, because the centre
    of buoyancy is above the centre of gravity."""
    upright = D.restoring(params, np.eye(3))
    assert np.allclose(upright[3:], 0.0, atol=1e-12)
    # restoring() returns g, which sits on the LHS of the equation; the force the body feels is -g.
    assert np.isclose(upright[2], params.net_buoyancy_n), "g carries +net buoyancy along +z"
    assert -upright[2] < 0.0, "so the applied force is along -z in FRD, i.e. upward"
    for dof, rpy in ((3, [20.0, 0.0, 0.0]), (4, [0.0, 20.0, 0.0])):
        g = D.restoring(params, F.rot_from_rpy_deg(rpy))
        applied = -g[3:]                       # the equation has g on the LHS
        assert applied[dof - 3] < 0.0, f"a +20 deg tilt in DOF {dof} must give a negative righting moment"


def test_thruster_curve_is_monotone_asymmetric_and_has_a_dead_band(params):
    t = params.t200
    u = np.linspace(-1.0, 1.0, 401)
    f = t.thrust_of_command(u)
    assert np.all(np.diff(f) >= -1e-12), "thrust must be monotone in the command"
    assert np.isclose(f[-1], t.thrust_fwd_max_n) and np.isclose(f[0], -t.thrust_rev_max_n)
    assert t.thrust_fwd_max_n > t.thrust_rev_max_n, "the T200 is stronger forward than in reverse"
    assert np.isclose(t.thrust_of_command([0.0])[0], 0.0)
    mid = t.command_of_pwm([1475.0, 1500.0, 1525.0])
    assert np.allclose(mid, 0.0), "the whole dead band maps to exactly zero command"
    assert np.allclose(t.command_of_pwm([1100.0, 1900.0]), [-1.0, 1.0])
    assert np.allclose(t.command_of_pwm([900.0, 2200.0]), [-1.0, 1.0]), "out-of-range PWM is clipped"


def test_thruster_lag_is_first_order(params, vehicle):
    """After one time constant a step command must have reached 1 - 1/e of its steady value."""
    v = free(vehicle, neutral=True)
    s = at_tank([1.5, 1.5, 1.5])
    tau = params.t200.tau_s
    s_end, _ = D.simulate(v, s, FWD, tau)
    steady = params.t200.thrust_of_command(FWD)[0]
    assert np.isclose(s_end.thrust_n[0] / steady, 1.0 - np.exp(-1.0), rtol=2e-3)


def test_bad_parameter_files_raise(tmp_path, params):
    import yaml
    from pathlib import Path
    good = yaml.safe_load((D.DEFAULT_PARAMS).read_text())

    def write(mutate) -> Path:
        cfg = yaml.safe_load(D.DEFAULT_PARAMS.read_text())
        mutate(cfg)
        p = tmp_path / "bad.yaml"
        p.write_text(yaml.safe_dump(cfg))
        return p

    with pytest.raises(D.VehicleError, match="not found"):
        D.load_params(tmp_path / "missing.yaml")
    for name, mutate, match in (
            ("cg offset", lambda c: c["body"].__setitem__("r_g_frd_m", [0.01, 0.0, 0.0]), "centre of gravity"),
            ("non-unit axis", lambda c: c["thrusters"]["list"][0].__setitem__("axis_frd", [1.0, 1.0, 0.0]),
             "unit"),
            ("positive damping", lambda c: c["damping_linear"].__setitem__("X_u", 13.7), "must be <= 0"),
            ("count mismatch", lambda c: c["thrusters"].__setitem__("count", 6), "!="),
            ("bad method", lambda c: c["integrator"].__setitem__("method", "euler"), "rk4"),
            ("weight mismatch", lambda c: c["body"].__setitem__("weight_n", 200.0), "disagrees"),
    ):
        with pytest.raises(D.VehicleError, match=match):
            D.load_params(write(mutate))
    assert good["model"] == "BlueROV2 Heavy", "the shipped file is untouched by this test"


def test_step_validates_its_input(vehicle):
    s = at_tank([1.5, 1.5, 1.5])
    with pytest.raises(D.VehicleError, match="dt"):
        vehicle.step(s, ZERO, 0.0)
    with pytest.raises(D.VehicleError, match="entries"):
        vehicle.step(s, np.zeros(6))


def test_simulation_is_deterministic_and_reads_no_clock(vehicle):
    """Two identical runs must agree bit for bit: no wall-clock term anywhere in the ROV path
    (docs/ros_hil.md 3.4)."""
    v = free(vehicle)
    s = at_tank([1.2, 1.4, 1.6], (3.0, -5.0, 20.0))
    a, _ = D.simulate(v, s, FWD, 2.0)
    b, _ = D.simulate(v, s, FWD, 2.0)
    assert np.array_equal(a.position_ned, b.position_ned)
    assert np.array_equal(a.quat_frd_ned, b.quat_frd_ned)


# ===================================================================== (a) hover with no thrust

def test_a_zero_thrust_drift_matches_the_net_buoyancy(vehicle, params):
    """(a) The parameters say +2.0 N of buoyancy, so the vehicle does NOT hover: it rises. The terminal rise
    speed must be the one the heave damping implies, and nothing else may move."""
    v = free(vehicle)
    s = at_tank([1.5, 1.5, 1.0])
    end, tr = D.simulate(v, s, ZERO, 30.0, record_every=40)

    w = float(end.velocity_frd[2])
    assert w < 0.0, "negative body-z is upward in FRD: it must rise"
    expect = np.roots([params.d_quad[2], params.d_lin[2], -params.net_buoyancy_n])
    expect = float(max(r.real for r in expect if abs(r.imag) < 1e-12 and r.real > 0))
    assert np.isclose(abs(w), expect, rtol=1e-3), f"rise speed {abs(w):.5f} vs analytic {expect:.5f} m/s"
    assert abs(w) < 0.06, "and it is a slow drift, not a surfacing"

    assert np.allclose(end.velocity_frd[:2], 0.0, atol=1e-12), "no horizontal drift"
    assert np.allclose(end.rate_frd, 0.0, atol=1e-12), "no rotation"
    assert np.allclose(end.quat_frd_ned, s.quat_frd_ned, atol=1e-12), "attitude unchanged"
    assert np.allclose(F.rpy_deg_from_quat(end.quat_frd_ned)[:2], 0.0, atol=1e-9), \
        "and still level (the heading is 90 deg, because tank +x is East)"
    rise = float(s.position_ned[2] - end.position_ned[2])
    assert 0.8 < rise < 1.6, f"risen {rise:.3f} m in 30 s"


def test_a_neutrally_buoyant_vehicle_really_does_hover(vehicle):
    """The same test with the buoyancy set equal to the weight: then it is a true hover, and the drift is
    numerical only. This separates "the model drifts" from "the vehicle is buoyant"."""
    v = free(vehicle, neutral=True)
    s = at_tank([1.5, 1.5, 1.5])
    end, _ = D.simulate(v, s, ZERO, 600.0, record_every=4000)
    assert np.allclose(end.position_ned, s.position_ned, atol=1e-9), "no drift in 10 simulated minutes"
    assert np.allclose(end.nu, 0.0, atol=1e-12)


# ===================================================================== (b) free decay per DOF

@pytest.mark.parametrize("dof,name,v0", [(0, "surge", 0.5), (1, "sway", 0.5), (2, "heave", 0.3),
                                         (5, "yaw", 1.0)])
def test_b_free_decay_is_monotone_in_the_unrestored_dof(vehicle, dof, name, v0):
    """(b) Surge, sway, heave and yaw have no restoring term, so free decay from an initial velocity must be
    monotone, keep its sign, and not excite anything else."""
    v = free(vehicle, neutral=True)
    s = at_tank([1.5, 1.5, 1.5])
    nu = np.zeros(6)
    nu[dof] = v0
    s = replace(s, velocity_frd=nu[:3].copy(), rate_frd=nu[3:].copy())
    _, tr = D.simulate(v, s, ZERO, 20.0, record_every=20)

    x = tr.velocity_frd[:, dof] if dof < 3 else tr.rate_frd[:, dof - 3]
    assert np.all(x > 0.0), f"{name}: sign flipped"
    assert np.all(np.diff(x) <= 1e-15), f"{name}: not monotone"
    t50 = half_life(tr.t, x)
    assert 0.02 < t50 < 10.0, f"{name}: half-life {t50:.3f} s is not plausible for a 12 kg ROV"
    others = [i for i in range(6) if i != dof]
    rest = np.concatenate([tr.velocity_frd, tr.rate_frd], axis=1)[:, others]
    assert np.abs(rest).max() < 1e-9, f"{name}: excited another DOF by {np.abs(rest).max():.2e}"


@pytest.mark.parametrize("dof,name", [(3, "roll"), (4, "pitch")])
def test_b_free_decay_is_monotone_in_roll_and_pitch_without_the_cb_offset(vehicle, dof, name):
    """Roll and pitch only decay monotonically once the righting moment is removed; with it they oscillate,
    which test (e) checks instead."""
    v = free(vehicle, neutral=True, no_restoring=True)
    s = at_tank([1.5, 1.5, 1.5])
    rate = np.zeros(3)
    rate[dof - 3] = 1.0
    s = replace(s, rate_frd=rate)
    _, tr = D.simulate(v, s, ZERO, 20.0, record_every=20)
    x = tr.rate_frd[:, dof - 3]
    assert np.all(x > 0.0) and np.all(np.diff(x) <= 1e-15), f"{name}: not a monotone decay"
    t50 = half_life(tr.t, x)
    assert 0.02 < t50 < 10.0, f"{name}: half-life {t50:.3f} s"


# ===================================================================== (c) terminal velocity

def test_c_surge_terminal_velocity(vehicle, params):
    """(c) Full forward thrust on the four horizontal thrusters, buoyancy neutralised so surge is isolated.

    The analytic terminal speed solves ``X_u u + X_uu u^2 = 4 cos(45 deg) T_max``. With the published damping
    and the T200's 51.5 N at 16 V that is ~0.97 m/s, against Blue Robotics' "up to 1.5 m/s" for the vehicle.
    The gap is in the damping coefficient, not the thrust: see config/rov_bluerov2.yaml damping_quadratic.
    """
    v = free(vehicle, neutral=True)
    s = at_tank([1.5, 1.5, 1.5])
    end, tr = D.simulate(v, s, FWD, 40.0, record_every=40)

    u = float(end.velocity_frd[0])
    tau_x = float((params.allocation @ params.t200.thrust_of_command(FWD))[0])
    expect = np.roots([params.d_quad[0], params.d_lin[0], -tau_x])
    expect = float(max(r.real for r in expect if abs(r.imag) < 1e-12 and r.real > 0))
    assert np.isclose(u, expect, rtol=1e-3), f"surge {u:.4f} vs analytic {expect:.4f} m/s"
    assert 0.6 < u < 1.3, f"surge terminal velocity {u:.3f} m/s is outside the plausible BlueROV2 band"
    assert np.all(np.diff(tr.velocity_frd[:, 0]) >= -1e-9), "the approach to terminal must be monotone"
    assert np.allclose(end.velocity_frd[1:], 0.0, atol=1e-9), "pure surge, no sway or heave"
    assert np.allclose(end.rate_frd, 0.0, atol=1e-9), "and no rotation"


def test_c_heave_and_yaw_terminal_rates_are_plausible(vehicle, params):
    """The same for the two other axes an inspection mission uses, reported rather than tightly bounded."""
    v = free(vehicle, neutral=True)
    s = at_tank([1.5, 1.5, 1.5])
    up, _ = D.simulate(v, s, UP, 40.0)
    yaw, _ = D.simulate(v, s, YAW, 40.0)
    assert 0.3 < abs(float(up.velocity_frd[2])) < 1.2, f"heave {up.velocity_frd[2]:.3f} m/s"
    assert 0.5 < abs(float(yaw.rate_frd[2])) < 6.0, f"yaw rate {yaw.rate_frd[2]:.3f} rad/s"


# ===================================================================== (d) pure yaw torque

def test_d_pure_yaw_torque_gives_pure_yaw_acceleration(vehicle, params):
    """(d) The yaw thrust combination, with the thrusters already at their steady force and the buoyancy
    neutralised, must accelerate yaw and nothing else. M is diagonal here, so there is no added-mass
    cross-coupling to allow for at all - the tolerance is numerical."""
    v = free(vehicle, neutral=True)
    s = steady_state(v, YAW)
    a = v.acceleration(s, YAW)
    assert abs(a[5]) > 0.1, f"no yaw acceleration: {a}"
    assert np.abs(np.delete(a, 5)).max() < 1e-12, f"cross-coupling: {a}"

    tau = params.allocation @ (YAW * 40.0)
    assert np.isclose(a[5], tau[5] / params.M[5, 5]), "and it is torque / (Izz + N_rdot)"


@pytest.mark.parametrize("cmd,dof", [(FWD, 0), (SWAY, 1), (UP, 2), (ROLL, 3), (PITCH, 4), (YAW, 5)])
def test_d_every_axis_thrust_pattern_accelerates_only_its_own_dof(vehicle, cmd, dof):
    v = free(vehicle, neutral=True)
    a = v.acceleration(steady_state(v, cmd), cmd)
    assert abs(a[dof]) > 1e-6
    assert np.abs(np.delete(a, dof)).max() < 1e-12, f"{cmd} gave {a}"


# ===================================================================== (e) restoring moment

@pytest.mark.parametrize("dof,name", [(3, "roll"), (4, "pitch")])
def test_e_a_tilt_returns_toward_upright_with_a_plausible_period(vehicle, params, dof, name):
    """(e) Released from a 20 deg tilt the vehicle must swing back through upright, and the period must be the
    one the CB offset implies: ``2 pi sqrt(M_dof / (B |z_b|))``, i.e. 3.9 s in roll and 3.5 s in pitch.

    The damping in these two DOF is almost entirely quadratic (K_p and the linear roll term are published as
    zero), so the decay is slow and the first period is close to the undamped one.
    """
    v = free(vehicle, neutral=True)
    rpy = [0.0, 0.0, 0.0]
    rpy[dof - 3] = 20.0
    s = at_tank([1.5, 1.5, 1.5], tuple(rpy))
    _, tr = D.simulate(v, s, ZERO, 30.0, record_every=20)

    ang = tr.rpy_ned_deg[:, dof - 3]
    assert ang[0] * ang[1] > 0.0 and abs(ang[1]) < abs(ang[0]), f"{name}: does not start returning"
    assert ang.min() < -5.0, f"{name}: never swings past upright (min {ang.min():.2f} deg)"

    zeros = np.flatnonzero(np.sign(ang[:-1]) != np.sign(ang[1:]))
    assert zeros.size >= 2, f"{name}: fewer than one full swing in 30 s"
    period = 2.0 * float(tr.t[zeros[1]] - tr.t[zeros[0]])
    stiffness = params.buoyancy_n * abs(params.r_b_frd_m[2])
    undamped = 2.0 * np.pi * np.sqrt(params.M[dof, dof] / stiffness)
    # The linear damping of the DOF lengthens the period by 1/sqrt(1 - zeta^2). In roll the published linear
    # coefficient is zero (zeta = 0), in pitch M_q = -0.8 gives zeta = 0.62, i.e. 3.54 s -> 4.50 s. Comparing
    # against the undamped period would be wrong for pitch by 30 %.
    zeta = params.d_lin[dof] / (2.0 * np.sqrt(stiffness * params.M[dof, dof]))
    damped = undamped / np.sqrt(max(1.0 - zeta ** 2, 1e-9))
    assert zeta < 1.0, f"{name}: zeta {zeta:.3f} is overdamped, there would be no period"
    assert np.isclose(period, damped, rtol=0.15), \
        f"{name}: period {period:.2f} s vs damped {damped:.2f} s (undamped {undamped:.2f} s, zeta {zeta:.3f})"
    assert 1.0 < period < 10.0, f"{name}: period {period:.2f} s is not plausible"

    peaks = np.abs(ang[[0] + list(zeros[1:2] + 1)])
    assert peaks[-1] <= peaks[0], f"{name}: the envelope grows"


def test_e_the_righting_stiffness_is_one_number(params):
    """Everything in test (e) rests on body.r_b_frd_m[2] alone; make that visible, because it is UNVERIFIED
    and it is the single parameter the righting period is proportional to."""
    assert np.isclose(params.buoyancy_n * abs(params.r_b_frd_m[2]), 1.148, rtol=1e-3)


# ===================================================================== (f) energy

def test_f_energy_never_grows_over_ten_simulated_minutes(vehicle):
    """(f) With zero thrust, ``E = 0.5 nu' M nu + (B - W) p_D + B (R r_b)_D`` must be non-increasing: C is
    skew-symmetric so it does no work, and D is dissipative. 10 simulated minutes at 400 Hz = 240 000 steps.
    """
    v = free(vehicle)
    s = at_tank([1.5, 1.5, 1.2], (12.0, -8.0, 35.0))
    s = replace(s, velocity_frd=np.array([0.4, -0.2, 0.1]), rate_frd=np.array([0.5, -0.3, 0.2]))
    end, tr = D.simulate(v, s, ZERO, 600.0, record_every=400)

    assert np.all(np.isfinite(tr.energy_j)), "the integrator diverged"
    rise = np.diff(tr.energy_j).max()
    assert rise <= 1e-9, f"energy grew by {rise:.3e} J between two samples"
    assert tr.energy_j[-1] < tr.energy_j[0], "and it must actually dissipate"
    assert np.isclose(np.linalg.norm(end.quat_frd_ned), 1.0, atol=1e-12), "the quaternion stayed normalised"


def test_f_energy_is_conserved_when_the_damping_is_zeroed(vehicle):
    """The sharper half of (f): with D = 0 and no thrust, E must be CONSERVED, which tests the integrator and
    the energy expression together (C does no work, the restoring wrench is conservative).

    The horizon is 5 s, not 10 minutes, and deliberately so: the undamped vehicle is exponentially unstable
    in rotation (see the next test), so beyond a few seconds any integrator's round-off is amplified off the
    energy surface. Over 5 s the statement is about the formulation; over 600 s it would be about chaos.
    """
    v = free(vehicle, neutral=True, no_damping=True)
    s = at_tank([1.5, 1.5, 1.5], (25.0, -15.0, 40.0))
    s = replace(s, velocity_frd=np.array([0.3, -0.15, 0.08]), rate_frd=np.array([0.6, -0.4, 0.3]))
    _, tr = D.simulate(v, s, ZERO, 5.0, record_every=40)
    drift = float(np.abs(tr.energy_j - tr.energy_j[0]).max())
    assert drift < 1e-9, f"energy drifted by {drift:.2e} J absolute over 5 s with no damping"


def test_f_the_undamped_vehicle_is_rotationally_unstable_but_the_damped_one_is_not(vehicle):
    """The Munk moment, measured, because it decides how much work ArduSub has to do.

    The added mass is anisotropic (Z_wdot 18.68 vs X_udot 6.36), so a body with both surge and heave feels a
    destabilising pitch moment ``(a_x - a_z) u w``, and the righting moment available to oppose it is only
    ``B |z_b| = 1.148 N m``. With the damping removed the attitude diverges exponentially; with the published
    damping it does not. The shipped model is the damped one, so the shipped model is stable - but open loop
    it is barely so, which is why M8a step 2's attitude hold is not a formality.
    """
    s = replace(at_tank([1.5, 1.5, 1.5], (0.0, 2.0, 0.0)), velocity_frd=np.array([0.5, 0.0, -0.1]))
    _, undamped = D.simulate(free(vehicle, neutral=True, no_damping=True), s, ZERO, 20.0, record_every=40)
    _, damped = D.simulate(free(vehicle, neutral=True), s, ZERO, 20.0, record_every=40)
    assert np.abs(undamped.rate_frd).max() > 10.0 * np.abs(damped.rate_frd).max(), \
        "the damping must be what stops the divergence"
    assert np.abs(damped.rate_frd[-1]).max() < 1.0, "and the damped vehicle must stay slow"
    assert np.all(np.diff(damped.energy_j) <= 1e-9), "the damped run still never gains energy"


# ===================================================================== (g) step-size sensitivity

def test_g_halving_the_step_changes_nothing_material(vehicle):
    """(g) The same thrusting manoeuvre at dt and dt/2. RK4 at 2.5 ms is 40 steps per thruster time constant,
    so the difference must be far below anything that matters for a 3 m tank."""
    # Straight-line surge, buoyancy neutralised: no heave, so no Munk moment, and no rotation, so no phase
    # to be sensitive about. This is the configuration in which "does halving the step change the answer" is
    # a question about the integrator rather than about the trajectory.
    v = free(vehicle, neutral=True)
    s = at_tank([1.0, 1.5, 1.2], (0.0, 0.0, 20.0))
    dt = v.params.dt
    runs = [D.simulate(v, s, FWD, 20.0, dt=dt / k)[0] for k in (1, 2, 4)]

    dp = float(np.linalg.norm(runs[0].position_ned - runs[1].position_ned))
    dv = float(np.linalg.norm(runs[0].velocity_frd - runs[1].velocity_frd))
    ang = np.degrees(2.0 * np.arccos(min(1.0, abs(float(runs[0].quat_frd_ned @ runs[1].quat_frd_ned)))))
    assert dp < 1e-5, f"position differs by {dp * 1e6:.3f} um over 20 s"
    assert dv < 1e-6, f"velocity differs by {dv:.2e} m/s"
    assert ang < 1e-6, f"attitude differs by {ang:.2e} deg"
    finer = float(np.linalg.norm(runs[1].position_ned - runs[2].position_ned))
    assert finer < dp, f"refining further must shrink the difference: {dp:.3e} then {finer:.3e}"


def test_g_a_turning_manoeuvre_is_phase_sensitive_not_step_sensitive(vehicle):
    """The honest companion to (g), so the number above is not over-read.

    A surge-and-yaw command spins the vehicle at ~2.8 rad/s (the published yaw damping is quadratic only,
    N_r = 0), which is nine revolutions in 20 s. Halving the step then moves the vehicle by ~1.8 m - not
    because the integrator is inaccurate, but because the final POSITION after nine turns is set by the
    accumulated phase. The attitude error per revolution is what actually bounds the integrator, so that is
    what this checks.
    """
    v = free(vehicle, neutral=True)
    s = at_tank([1.5, 1.5, 1.5])
    cmd = np.array([1.0, 0.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0])
    dt = v.params.dt
    a, _ = D.simulate(v, s, cmd, 2.0, dt=dt)
    b, _ = D.simulate(v, s, cmd, 2.0, dt=dt / 2.0)
    assert abs(float(a.rate_frd[2])) > 1.0, "the manoeuvre must really be turning"
    ang = np.degrees(2.0 * np.arccos(min(1.0, abs(float(a.quat_frd_ned @ b.quat_frd_ned)))))
    assert ang < 1e-4, f"attitude after 2 s differs by {ang:.2e} deg"
    assert np.linalg.norm(a.position_ned - b.position_ned) < 1e-4


def test_g_semi_implicit_agrees_with_rk4_to_first_order(vehicle):
    """The alternative integrator is wired up and behaves: much less accurate, but not wrong."""
    rk4 = free(vehicle)
    semi = D.Vehicle(replace(rk4.params, method="semi_implicit"), water_level=WL, bounds=None)
    s = at_tank([1.5, 1.5, 1.5])
    a, _ = D.simulate(rk4, s, FWD, 5.0)
    b, _ = D.simulate(semi, s, FWD, 5.0)
    assert np.linalg.norm(a.position_ned - b.position_ned) < 5e-3


# ===================================================================== the soft tank bound (decision 5)

def test_bound_stops_the_vehicle_at_the_wall(vehicle, caplog):
    """Decision 5's test, part 1: full forward thrust at the +x wall. The vehicle must stop inside the tank,
    the penetration must be small, and every activation must be logged and counted."""
    v = D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)
    s = at_tank([2.0, 1.5, 1.5])                        # heading along tank +x
    with caplog.at_level(logging.DEBUG, logger="uw.rov.dynamics"):
        end, tr = D.simulate(v, s, FWD, 30.0, record_every=40)

    x = np.array([F.position_ned_to_tank(p, WL)[0] for p in tr.position_ned])
    assert x.max() <= v.bounds.hi[0] + 0.12, f"penetrated the wall by {x.max() - v.bounds.hi[0]:.3f} m"
    assert x.max() < TANK[0], "and never left the tank"
    assert abs(float(end.velocity_frd[0])) < 0.05, "it must come to rest against the wall, not bounce"
    assert v.bound_activations > 0 and v.bound_max_penetration_m > 0.0
    assert any("soft tank bound active" in r.message for r in caplog.records), "the bound must log"


def test_bound_stops_the_vehicle_at_the_floor(vehicle):
    """Decision 5's test, part 2: full downward thrust."""
    v = D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)
    s = at_tank([1.5, 1.5, 1.2])
    end, tr = D.simulate(v, s, DOWN, 30.0, record_every=40)
    z = np.array([F.position_ned_to_tank(p, WL)[2] for p in tr.position_ned])
    assert z.min() >= v.bounds.lo[2] - 0.12, f"sank {v.bounds.lo[2] - z.min():.3f} m past the floor bound"
    assert z.min() > SAND, "and never went under the sand"
    assert v.bound_activations > 0


def test_bound_holds_the_vehicle_under_the_free_surface(vehicle):
    """Decision 5's test, part 3. The margin bound (z <= water_level - 0.2) binds first, so this also sets
    ``margin_m`` to zero to exercise the separate, stiffer free-surface term at z = water_level itself."""
    v = D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)
    end, tr = D.simulate(v, at_tank([1.5, 1.5, 2.4]), UP, 30.0, record_every=40)
    z = np.array([F.position_ned_to_tank(p, WL)[2] for p in tr.position_ned])
    assert z.max() <= v.bounds.hi[2] + 0.12, f"rose {z.max() - v.bounds.hi[2]:.3f} m past the margin bound"
    assert z.max() < WL, "and stayed in the water"

    p = replace(v.params, bound_margin_m=0.0, bound_sand_extra_m=0.0)
    k, c, ks, cs = p.bound_gains
    surface_only = D.Vehicle(p, water_level=WL,
                             bounds=D.TankBounds.from_tank(TANK, SAND, WL, 0.0, 0.0, k, c, ks, cs))
    _, tr2 = D.simulate(surface_only, at_tank([1.5, 1.5, 2.7]), UP, 30.0, record_every=40)
    z2 = np.array([F.position_ned_to_tank(q, WL)[2] for q in tr2.position_ned])
    assert z2.max() < WL + 0.06, f"broke the surface by {z2.max() - WL:.3f} m"
    assert surface_only.bound_activations > 0


def test_bound_can_be_switched_off_and_then_the_vehicle_leaves_the_tank(vehicle):
    """The bound is non-physical, so it must be removable - and the test shows what it is protecting against:
    without it the vehicle simply flies out through the wall, because there is no PhysX scene
    (docs/ros_hil.md 6.5)."""
    v = free(vehicle)
    assert v.bounds is None
    end, _ = D.simulate(v, at_tank([2.0, 1.5, 1.5]), FWD, 30.0)
    out = F.position_ned_to_tank(end.position_ned, WL)
    # Direction unspecified on purpose: positively buoyant and open loop, the vehicle also tumbles (the Munk
    # moment of test_f), so it leaves through whichever face it is pointing at by then. The point is only
    # that nothing stops it.
    assert np.any(out < 0.0) or np.any(out > np.array(TANK)), f"unbounded, it must leave the tank: {out}"
    assert v.bound_activations == 0


def test_bound_applies_no_moment(vehicle, params):
    """Deliberate simplification, stated in the module docstring: the bound is a pure force at the body
    origin, so pressing into a wall cannot spin the vehicle."""
    v = D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)
    s = at_tank([2.95, 1.5, 1.5], (0.0, 0.0, 30.0))
    w = v._bound_wrench(s.position_ned, s.rot_frd_ned, np.array([0.5, 0.0, 0.0]))
    assert np.linalg.norm(w[:3]) > 1.0, "the bound must be pushing"
    assert np.allclose(w[3:], 0.0), "and it must make no moment"
