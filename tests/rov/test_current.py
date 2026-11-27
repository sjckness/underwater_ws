"""vehicles.common.current and the current in vehicles.common.dynamics (M12): the field, the relative-velocity Fossen terms, the
runtime change, and that still water is still exactly the model of before.

The golden file pins the still-water path: tests/rov/data/dynamics_golden.json holds the final states of
``golden_runs()`` written by vehicles/common/dynamics.py *before* the current existed (2026-11-27, linus: equal bit for bit
there). Elsewhere a different BLAS may round differently, so the comparison allows 1e-9 relative: a change to the
model moves these numbers by far more.
"""
from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from uw import config
from vehicles.common import current as C
from vehicles.common import dynamics as D
from vehicles.common import frames as F

POOL = (10.0, 10.0, 5.5)
POOL_WL = 5.0
GOLDEN = Path(__file__).with_name("data") / "dynamics_golden.json"

FWD = np.array([-1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])        # tests/rov/test_dynamics.py's patterns
SWAY = np.array([1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
YAW = np.array([1.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0])
UP = np.array([0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0])
ZERO = np.zeros(8)


def _cmd(t, state):
    return np.clip(0.6 * np.sin(0.7 * t) * FWD + 0.4 * np.cos(1.3 * t) * SWAY + 0.3 * np.sin(2.1 * t) * YAW
                   + 0.35 * np.sin(0.9 * t + 0.4) * UP, -1.0, 1.0)


def golden_runs(current=None):
    """name -> final State of two fixed runs in the 10 x 10 pool: mid-water with every DOF excited, and a start at
    the surface driven into a wall (surface model and soft bound)."""
    out = {}
    veh = D.make_vehicle(tank_size=POOL, sand_base=0.0, water_level=POOL_WL, surface_model=True, current=current)
    p, q = F.pose_tank_to_ned([5.0, 5.0, 2.5], F.quat_from_rpy_deg((3.0, -2.0, 30.0)), POOL_WL)
    out["mid_water"], _ = D.simulate(veh, D.State.rest(8, p, q), _cmd, 3.0)
    p, q = F.pose_tank_to_ned([9.3, 1.5, 4.88], F.quat_from_rpy_deg((0.0, 0.0, 0.0)), POOL_WL)
    out["surface_wall"], _ = D.simulate(veh, D.State.rest(8, p, q), lambda t, s: np.clip(0.8 * FWD + 0.2 * SWAY, -1, 1),
                                        3.0)
    return out


@pytest.fixture(scope="module")
def base() -> D.Vehicle:
    return D.make_vehicle(tank_size=(50.0, 25.0, 5.5), sand_base=0.0, water_level=POOL_WL)


def neutral(base, current=None) -> D.Vehicle:
    """Neutrally buoyant, no soft bound: nothing but the current moves it."""
    p = replace(base.params, buoyancy_n=base.params.weight_n)
    return D.Vehicle(p, water_level=POOL_WL, bounds=None, current=current)


def at(pos_tank=(25.0, 12.0, 2.5), yaw_enu=0.0, n=8) -> D.State:
    p, q = F.pose_tank_to_ned(pos_tank, F.quat_from_rpy_deg((0.0, 0.0, yaw_enu)), POOL_WL)
    return D.State.rest(n, p, q)


def ground_ned(state) -> np.ndarray:
    return state.rot_frd_ned @ state.velocity_frd


# ---------------------------------------------------------------------------------- still water: unchanged

def test_still_water_is_the_model_of_before_the_current():
    gold = json.loads(GOLDEN.read_text())["runs"]
    for name, state in golden_runs().items():
        y = np.array([float.fromhex(h) for h in gold[name]["y"]])
        np.testing.assert_allclose(state.pack(), y, rtol=1e-9, atol=1e-12, err_msg=name)
        assert state.t == pytest.approx(float.fromhex(gold[name]["t"]), abs=1e-12)


def test_a_still_current_is_no_current_at_all():
    for still in (C.STILL, C.Current(speed_mps=0.0, direction_deg=77.0), None):
        assert D.make_vehicle(current=still).current is None
    a, b = golden_runs(), golden_runs(C.Current())
    for name in a:
        assert np.array_equal(a[name].pack(), b[name].pack())


def test_the_current_terms_reduce_exactly_to_still_water_at_zero_velocity(base):
    """The current branch of _deriv with v_c = a_c = 0 is the still-water branch operation for operation."""
    rng = np.random.default_rng(3)
    for _ in range(20):
        s = at(yaw_enu=float(rng.uniform(-180, 180)))
        s = replace(s, velocity_frd=rng.normal(0, 0.4, 3), rate_frd=rng.normal(0, 0.5, 3),
                    thrust_n=rng.normal(0, 10, 8))
        steady = base.params.t200.thrust_of_command(rng.uniform(-1, 1, 8))
        still = base._deriv(s.pack(), steady)
        zero = base._deriv(s.pack(), steady, np.zeros(3), np.zeros(3))
        assert np.array_equal(still, zero)


# ------------------------------------------------------------------------------------- the physics of it

@pytest.mark.parametrize("direction, heading", [(0.0, 0.0), (30.0, 75.0), (90.0, -120.0), (225.0, 10.0)])
def test_with_no_thrust_the_vehicle_ends_up_drifting_with_the_water(base, direction, heading):
    cur = C.Current(speed_mps=0.3, direction_deg=direction)
    veh = neutral(base, cur)
    # 50 Hz is plenty for a coast with no thrust (the slowest mode is tens of seconds) and keeps this test fast
    s, tr = D.simulate(veh, at(yaw_enu=heading), ZERO, 300.0, dt=0.02, record_every=50)
    v_c = cur.velocity_ned(s.t, 2.5)
    assert np.linalg.norm(ground_ned(s) - v_c) < 1e-3
    # and the position follows: the last 10 s move by 10 s of current
    dp = tr.position_ned[-1] - tr.position_ned[-11]
    assert np.allclose(dp, 10.0 * v_c, atol=0.02)
    # the direction convention: ENU azimuth towards which the water flows
    v_enu = F.vec_enu_ned(v_c)
    assert math.degrees(math.atan2(v_enu[1], v_enu[0])) % 360 == pytest.approx(direction % 360, abs=1e-9)


def test_a_body_at_rest_in_the_water_stays_with_it_while_it_turns(base):
    """nu_r = 0 and a yaw spin: the ground velocity must not change, d/dt (R v) = 0. That needs M_A nu_c_dot: the
    current seen from the rotating body turns, and the added mass must be told."""
    cur = C.Current(speed_mps=0.3, direction_deg=40.0)
    veh = neutral(base, cur)
    s = at(yaw_enu=10.0)
    v_c = cur.velocity_ned(0.0, 2.5)
    w = np.array([0.0, 0.0, 0.6])
    s = replace(s, velocity_frd=s.rot_frd_ned.T @ v_c, rate_frd=w)
    d = veh.derivative(s, ZERO)
    v_dot, v = d[7:10], s.velocity_frd
    assert np.allclose(s.rot_frd_ned @ (v_dot + np.cross(w, v)), 0.0, atol=1e-12)
    # the term is not negligible: dropping it would accelerate the body by this much
    p = veh.params
    missing = p.M_inv[:3, :3] @ (p.added_mass[:3] * np.cross(w, s.rot_frd_ned.T @ v_c))
    assert np.linalg.norm(missing) > 0.01


def _command_for_thrust(t200, f):
    """The normalised commands whose steady thrust is ``f`` (the T200 curve is monotone: invert it on a grid)."""
    u = np.linspace(-1.0, 1.0, 200001)
    thrust = t200.thrust_of_command(u)
    return np.interp(f, thrust, u)


def test_a_thrust_equal_to_the_drag_holds_the_vehicle_still_in_the_current(base):
    cur = C.Current(speed_mps=0.3, direction_deg=0.0)
    veh = neutral(base, cur)
    s = at(yaw_enu=0.0)                                   # facing +x, into... no: the water flows +x, the bow too
    vc_b = s.rot_frd_ned.T @ cur.velocity_ned(0.0, 2.5)
    nu_r = np.concatenate([-vc_b, np.zeros(3)])
    tau = D.damping(veh.params, nu_r)                     # what the thrusters must give to cancel the drag
    f, *_ = np.linalg.lstsq(veh.params.allocation, tau, rcond=None)
    u = _command_for_thrust(veh.params.t200, f)
    s = replace(s, thrust_n=veh.params.t200.thrust_of_command(u))
    s, _ = D.simulate(veh, s, u, 20.0, record_every=400)
    assert np.linalg.norm(ground_ned(s)) < 1e-3


def test_a_ramp_reaches_the_new_current_and_is_deterministic(base):
    def run():
        veh = neutral(base)
        s = at()
        for i in range(4000):                             # 10 s at 400 Hz
            if i == 400:
                veh.set_current(C.Current(speed_mps=0.4, direction_deg=90.0), ramp_s=3.0)
            s = veh.step(s, ZERO)
        return veh, s
    veh, s1 = run()
    _, s2 = run()
    assert np.array_equal(s1.pack(), s2.pack())
    assert np.allclose(veh.current_ned, C.Current(speed_mps=0.4, direction_deg=90.0).velocity_ned(0, 0), atol=1e-12)
    assert isinstance(veh.current, C.Current), "a finished ramp collapses to its target"
    # and back to still water: the vehicle coasts to a stop
    veh.set_current(None, ramp_s=1.0)
    for _ in range(800):
        s1 = veh.step(s1, ZERO)
    assert veh.current is None and np.array_equal(veh.current_ned, np.zeros(3))


def test_the_ramp_is_smooth_in_velocity_and_acceleration():
    old, new = C.Current(speed_mps=0.1, direction_deg=0.0), C.Current(speed_mps=0.5, direction_deg=90.0)
    r = C.Ramped(old, new, t0=10.0, ramp_s=4.0)
    assert np.allclose(r.velocity_ned(10.0, 1.0), old.velocity_ned(10.0, 1.0))
    assert np.allclose(r.velocity_ned(14.0, 1.0), new.velocity_ned(14.0, 1.0))
    assert np.allclose(r.accel_ned(10.0, 1.0), 0.0) and np.allclose(r.accel_ned(14.0, 1.0), 0.0)
    for t in (10.5, 12.0, 13.3):
        h = 1e-5
        num = (r.velocity_ned(t + h, 1.0) - r.velocity_ned(t - h, 1.0)) / (2 * h)
        assert np.allclose(r.accel_ned(t, 1.0), num, atol=1e-8)


# ------------------------------------------------------------------------------------------- the field

def test_every_profile_is_one_at_the_surface():
    for kw in ({}, {"profile": "power"}, {"profile": "linear", "bottom_factor": 0.3},
               {"profile": "table", "table": ((0.0, 1.0), (2.0, 0.8), (5.0, 0.2))}):
        cur = C.Current(speed_mps=0.2, water_depth_m=5.0, **kw)
        assert cur.factor(0.0)[0] == pytest.approx(1.0)
        assert np.linalg.norm(cur.velocity_ned(0.0, 0.0)) == pytest.approx(0.2)


def test_the_profiles_have_the_shape_they_promise():
    lin = C.Current(speed_mps=1.0, profile="linear", bottom_factor=0.3, water_depth_m=5.0)
    assert lin.factor(5.0)[0] == pytest.approx(0.3) and lin.factor(2.5)[0] == pytest.approx(0.65)
    pw = C.Current(speed_mps=1.0, profile="power", water_depth_m=5.0)
    assert pw.factor(4.5)[0] == pytest.approx((0.5 / 5.0) ** (1 / 7))
    assert pw.factor(4.999)[0] < 0.5 and pw.factor(6.0)[0] == pw.factor(5.0)[0]       # floored at the floor
    tab = C.Current(speed_mps=1.0, profile="table", table=((0.0, 1.0), (2.0, 0.8), (5.0, 0.2)))
    assert tab.factor(1.0)[0] == pytest.approx(0.9) and tab.factor(3.5)[0] == pytest.approx(0.5)
    assert tab.factor(1.0)[1] == pytest.approx(-0.1) and tab.factor(3.5)[1] == pytest.approx(-0.2)


def test_the_acceleration_is_the_derivative_along_the_path():
    """Gusts in speed and direction plus a sheared profile, seen from a vehicle descending at 0.3 m/s."""
    cur = C.Current(speed_mps=0.25, direction_deg=20.0, profile="power", water_depth_m=5.0,
                    gust_sigma_mps=0.08, gust_time_s=15.0, gust_dir_sigma_deg=15.0, seed=4)
    w = 0.3
    for t in (3.0, 17.0, 41.5):
        h = 1e-5
        d = lambda tt: 1.0 + w * tt * 0.01         # noqa: E731 - depth along the path, slowly
        num = (cur.velocity_ned(t + h, d(t + h)) - cur.velocity_ned(t - h, d(t - h))) / (2 * h)
        assert np.allclose(cur.accel_ned(t, d(t), w * 0.01), num, atol=1e-7)


def test_gusts_have_the_requested_mean_and_spread():
    cur = C.Current(speed_mps=0.25, gust_sigma_mps=0.05, gust_time_s=10.0, seed=1)
    t = np.arange(0.0, 20000.0, 0.5)
    s = np.array([np.linalg.norm(cur.velocity_ned(x, 0.0)) for x in t])
    assert s.mean() == pytest.approx(0.25, abs=0.01)
    assert s.std() == pytest.approx(0.05, rel=0.15)
    again = C.Current(speed_mps=0.25, gust_sigma_mps=0.05, gust_time_s=10.0, seed=1)
    assert np.array_equal(cur.velocity_ned(123.4, 0.0), again.velocity_ned(123.4, 0.0))
    other = C.Current(speed_mps=0.25, gust_sigma_mps=0.05, gust_time_s=10.0, seed=2)
    assert not np.array_equal(cur.velocity_ned(123.4, 0.0), other.velocity_ned(123.4, 0.0))


def test_bad_currents_are_refused():
    with pytest.raises(C.CurrentError):
        C.Current(profile="spiral")
    with pytest.raises(C.CurrentError):
        C.Current(speed_mps=-0.1)
    with pytest.raises(C.CurrentError):
        C.Current(profile="table", table=((2.0, 1.0), (1.0, 0.5)))
    with pytest.raises(C.CurrentError):
        C.from_dict({"speed": 0.3})                       # a typo is not a still pool
    with pytest.raises(C.CurrentError):
        C.from_dict({"speed_mps": 0.3, "gust": {"sigma": 0.1}})


# -------------------------------------------------------------------------------------------- the config

def test_the_pool_config_is_still_water_and_the_presets_are_not():
    assert C.from_config(config.load(config.POOL_CONFIG)).is_still
    for name, speed in (("light", 0.10), ("strong", 0.40), ("gusty", 0.25)):
        cur = C.from_config(config.load(config.POOL_CONFIG, presets=[name]))
        assert cur.speed_mps == pytest.approx(speed) and not cur.is_still
    gusty = C.from_config(config.load(config.POOL_CONFIG, presets=["gusty", "turbid"]))
    assert gusty.profile == "power" and gusty.gust_sigma_mps > 0 and gusty.water_depth_m == POOL_WL


def test_the_tank_config_has_no_current():
    assert C.from_config(config.load()).is_still
