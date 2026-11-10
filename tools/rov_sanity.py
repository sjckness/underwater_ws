"""M8a step 1: measure and plot the BlueROV2 Heavy dynamics, so the parameters can be judged.

Runs the physics sanity set (a)-(g) of the M8a step 1 brief, prints every number it measures next to what the
model predicts analytically, and writes the plots. Nothing here needs a GPU, Isaac or ROS.

    python -m uw py tools/rov_sanity.py                 # the repo's portable entry point
    python3 tools/rov_sanity.py --out output/rov        # or directly, with PYTHONPATH=<repo>

Also prints the parameter tag inventory: every value in config/rov_bluerov2.yaml marked GUESS or UNVERIFIED,
which is the list the M8a report has to carry.
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from uw.rov import dynamics as D          # noqa: E402
from uw.rov import frames as F            # noqa: E402

WL, TANK, SAND = 2.9, (3.0, 3.0, 3.0), 0.3
AXES = ("surge", "sway", "heave", "roll", "pitch", "yaw")
CMD = {
    "surge": np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
    "sway": np.array([-1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
    "heave": np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]),
    "roll": np.array([0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0]),
    "pitch": np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0]),
    "yaw": np.array([1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
}
ZERO = np.zeros(8)


def at_tank(pos_tank, rpy_enu_deg=(0.0, 0.0, 0.0)) -> D.State:
    p_ned, q_ned = F.pose_tank_to_ned(pos_tank, F.quat_from_rpy_deg(rpy_enu_deg), WL)
    return D.State.rest(8, position_ned=p_ned, quat=q_ned)


def free(v: D.Vehicle, **kw) -> D.Vehicle:
    p = v.params
    if kw.get("neutral"):
        p = replace(p, buoyancy_n=p.weight_n)
    if kw.get("no_restoring"):
        p = replace(p, r_b_frd_m=np.zeros(3))
    if kw.get("no_damping"):
        p = replace(p, d_lin=np.zeros(6), d_quad=np.zeros(6))
    return D.Vehicle(p, water_level=v.water_level, bounds=None)


def half_life(t, x) -> float:
    x = np.abs(np.asarray(x, float))
    below = np.flatnonzero(x <= 0.5 * x[0])
    if below.size == 0 or below[0] == 0:
        return float("nan")
    i = int(below[0])
    return float(t[i - 1] + (x[i - 1] - 0.5 * x[0]) / max(x[i - 1] - x[i], 1e-18) * (t[i] - t[i - 1]))


def terminal(params: D.Params, dof: int, tau: float) -> float:
    """The analytic terminal rate: ``d_lin x + d_quad x^2 = tau``."""
    roots = np.roots([params.d_quad[dof], params.d_lin[dof], -abs(tau)])
    real = [r.real for r in roots if abs(r.imag) < 1e-9 and r.real > 0.0]
    return float(max(real)) if real else float("nan")


def tank_xyz(traj) -> np.ndarray:
    return np.array([F.position_ned_to_tank(p, WL) for p in traj.position_ned])


# ----------------------------------------------------------------------------------- the measurements

def report(out: Path) -> dict:
    v = D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)
    p = v.params
    res: dict = {"runs": {}}
    line = "-" * 100

    print(line)
    print(f"M8a step 1  {p.model} / {p.frame}, {p.n_thrusters} thrusters, "
          f"integrator {p.method} at {p.rate_hz:.0f} Hz (dt = {p.dt * 1e3:.3f} ms)")
    print(line)
    print(f"mass {p.mass_kg:.2f} kg   weight {p.weight_n:.2f} N   buoyancy {p.buoyancy_n:.2f} N   "
          f"net {p.net_buoyancy_n:+.2f} N ({'positively' if p.net_buoyancy_n > 0 else 'negatively'} buoyant)")
    print(f"M diag (surge..yaw): {np.round(np.diag(p.M), 4)}")
    print(f"righting stiffness B|z_b| = {p.buoyancy_n * abs(p.r_b_frd_m[2]):.4f} N m/rad")
    print(f"allocation rank {np.linalg.matrix_rank(p.allocation)} of 6; yaw arm "
          f"{abs(p.allocation[5, 0]):.4f} m, roll arm {abs(p.allocation[3, 4]):.4f} m, "
          f"pitch arm {abs(p.allocation[4, 4]):.4f} m")

    # ---- hover pitch, the question docs/ros_hil.md 6.3 left open
    print(f"\n{line}\n(pre) HOVER PITCH the dataset needs, and whether this frame can hold it\n{line}")
    hover = {}
    for pitch_deg in (22.1, 31.2, 45.2):
        moment = p.buoyancy_n * abs(p.r_b_frd_m[2]) * np.sin(np.radians(pitch_deg))
        avail = float(np.abs(p.allocation[4, 4:]).sum() * p.t200.thrust_fwd_max_n)
        hover[pitch_deg] = (moment, avail)
        print(f"  pitch {pitch_deg:5.1f} deg: righting moment to oppose {moment:6.3f} N m, "
              f"pitch authority {avail:6.2f} N m -> margin {avail / moment:7.1f}x")
    res["hover_pitch"] = hover

    # ---- (a) zero thrust
    print(f"\n{line}\n(a) ZERO-THRUST DRIFT (bound off, as the parameters stand)\n{line}")
    start_a = at_tank([1.5, 1.5, 1.0])
    end, tr_a = D.simulate(free(v), start_a, ZERO, 60.0, record_every=40)
    rise_rate = abs(float(end.velocity_frd[2]))
    expect = terminal(p, 2, p.net_buoyancy_n)
    print(f"  rises at {rise_rate * 1000:.2f} mm/s (analytic {expect * 1000:.2f} mm/s), "
          f"{abs(float(tr_a.position_ned[-1, 2] - tr_a.position_ned[0, 2])):.3f} m in 60 s")
    tilt = np.degrees(2.0 * np.arccos(min(1.0, abs(float(end.quat_frd_ned @ start_a.quat_frd_ned)))))
    print(f"  horizontal drift {np.linalg.norm(end.position_ned[:2] - tr_a.position_ned[0, :2]):.2e} m, "
          f"attitude change {tilt:.2e} deg (initial heading is 90 deg, because tank +x is East)")
    neutral_end, _ = D.simulate(free(v, neutral=True), at_tank([1.5, 1.5, 1.5]), ZERO, 600.0,
                                record_every=4000)
    print(f"  with buoyancy = weight: drift over 10 simulated minutes "
          f"{np.linalg.norm(neutral_end.position_ned - at_tank([1.5, 1.5, 1.5]).position_ned):.2e} m")
    res["runs"]["a_zero_thrust"] = tr_a
    res["rise_mm_s"] = rise_rate * 1000.0

    # ---- (b) free decay
    print(f"\n{line}\n(b) FREE DECAY from an initial velocity (buoyancy neutralised, bound off)\n{line}")
    print(f"  {'DOF':8s} {'v0':>8s} {'half-life':>11s}  {'note'}")
    decay = {}
    for i, name in enumerate(AXES):
        restoring = name in ("roll", "pitch")
        vv = free(v, neutral=True, no_restoring=restoring)
        nu = np.zeros(6)
        nu[i] = 1.0 if i >= 3 else 0.5
        s = replace(at_tank([1.5, 1.5, 1.5]), velocity_frd=nu[:3].copy(), rate_frd=nu[3:].copy())
        _, tr = D.simulate(vv, s, ZERO, 20.0, record_every=20)
        x = tr.velocity_frd[:, i] if i < 3 else tr.rate_frd[:, i - 3]
        t50 = half_life(tr.t, x)
        mono = bool(np.all(np.diff(x) <= 1e-15)) and bool(np.all(x > 0))
        note = "monotone" if mono else "NOT MONOTONE"
        if restoring:
            note += "; CB offset removed (else it oscillates: see (e))"
        unit = "m/s" if i < 3 else "rad/s"
        print(f"  {name:8s} {x[0]:8.3f} {t50:10.3f} s  {note} [{unit}]")
        decay[name] = (float(x[0]), t50, mono)
        res["runs"][f"b_decay_{name}"] = tr
    res["decay"] = decay

    # ---- (c) terminal velocity
    print(f"\n{line}\n(c) TERMINAL RATE under full constant thrust (buoyancy neutralised, bound off)\n{line}")
    print("      magnitudes. roll and pitch are run with the CB offset removed, so their figure is the rate a")
    print("      TUMBLING vehicle reaches: full pitch authority is 24.7 N m against 1.15 N m of righting")
    print("      moment, so commanded pitch does not settle at an angle, it rotates.")
    print(f"  {'DOF':8s} {'wrench':>10s} {'measured':>11s} {'analytic':>11s}")
    term = {}
    for i, name in enumerate(AXES):
        vv = free(v, neutral=True, no_restoring=name in ("roll", "pitch"))
        end, tr = D.simulate(vv, at_tank([1.5, 1.5, 1.5]), CMD[name], 60.0, record_every=40)
        got = float(end.nu[i])
        tau = float((p.allocation @ p.t200.thrust_of_command(CMD[name]))[i])
        unit = "m/s" if i < 3 else "rad/s"
        print(f"  {name:8s} {tau:10.2f} {abs(got):11.4f} {terminal(p, i, tau):11.4f}  [{unit}]")
        term[name] = (tau, got, terminal(p, i, tau))
        res["runs"][f"c_terminal_{name}"] = tr
    res["terminal"] = term
    print(f"  surge {term['surge'][1]:.3f} m/s at {p.t200.voltage_v:.0f} V "
          f"against Blue Robotics' 'up to 1.5 m/s' for the vehicle: the gap is in damping_quadratic.X_uu,")
    print(f"  not in the thrust - full surge wrench is {term['surge'][0]:.1f} N and 1.5 m/s would need "
          f"{p.d_lin[0] * 1.5 + p.d_quad[0] * 1.5 ** 2:.0f} N.")

    # ---- (d) decoupling
    print(f"\n{line}\n(d) CROSS-COUPLING, two different questions\n{line}")
    print("  equal THRUST magnitudes (the geometry alone)        |  the real T200 COMMAND (asymmetric)")
    print(f"  {'DOF':8s} {'commanded':>10s} {'worst other':>12s} {'ratio':>9s}   |{'commanded':>10s} "
          f"{'worst other':>12s} {'ratio':>9s}  leaks into")
    coup = {}
    for i, name in enumerate(AXES):
        vv = free(v, neutral=True)
        s_geom = replace(at_tank([1.5, 1.5, 1.5]), thrust_n=CMD[name] * 40.0)
        a_g = vv.acceleration(s_geom, CMD[name])
        w_g = float(np.abs(np.delete(a_g, i)).max())
        s_cmd = replace(at_tank([1.5, 1.5, 1.5]), thrust_n=p.t200.thrust_of_command(CMD[name]))
        a_c = vv.acceleration(s_cmd, CMD[name])
        w_c = float(np.abs(np.delete(a_c, i)).max())
        leak = AXES[int(np.argmax(np.where(np.arange(6) == i, -1.0, np.abs(a_c))))] if w_c > 1e-9 else "-"
        print(f"  {name:8s} {a_g[i]:+10.4f} {w_g:12.2e} {w_g / abs(a_g[i]):9.1e}   |{a_c[i]:+10.4f} "
              f"{w_c:12.2e} {w_c / abs(a_c[i]):9.1e}  {leak}")
        coup[name] = (float(a_g[i]), w_g, float(a_c[i]), w_c)
    res["coupling"] = coup
    print("  The geometry is exactly decoupled. The command is not, because the T200 gives 51.5 N forward and")
    print("  40.2 N in reverse: a mixed-sign pattern leaves 2 x 11.3 N unbalanced. ArduSub's closed loop is")
    print("  what removes it; nothing in the model should be 'corrected' for it.")

    # ---- (e) restoring
    print(f"\n{line}\n(e) RESTORING: released from 20 deg (buoyancy neutralised, bound off)\n{line}")
    rest = {}
    for i, name in ((3, "roll"), (4, "pitch")):
        rpy = [0.0, 0.0, 0.0]
        rpy[i - 3] = 20.0
        _, tr = D.simulate(free(v, neutral=True), at_tank([1.5, 1.5, 1.5], tuple(rpy)), ZERO, 60.0,
                           record_every=20)
        ang = tr.rpy_ned_deg[:, i - 3]
        zc = np.flatnonzero(np.sign(ang[:-1]) != np.sign(ang[1:]))
        period = 2.0 * float(tr.t[zc[1]] - tr.t[zc[0]]) if zc.size >= 2 else float("nan")
        stiffness = p.buoyancy_n * abs(p.r_b_frd_m[2])
        undamped = 2.0 * np.pi * np.sqrt(p.M[i, i] / stiffness)
        zeta = p.d_lin[i] / (2.0 * np.sqrt(stiffness * p.M[i, i]))
        damped = undamped / np.sqrt(max(1.0 - zeta ** 2, 1e-9))
        absang = np.abs(ang)
        loc = 1 + np.flatnonzero((absang[1:-1] >= absang[:-2]) & (absang[1:-1] > absang[2:]))
        peaks = absang[np.r_[0, loc]][:5]
        print(f"  {name:6s} period {period:5.2f} s (undamped {undamped:5.2f} s, zeta {zeta:.3f} -> "
              f"damped {damped:5.2f} s), peak envelope {np.round(peaks, 2)} deg")
        rest[name] = (period, undamped, damped, float(zeta))
        res["runs"][f"e_restoring_{name}"] = tr
    res["restoring"] = rest

    # ---- (f) energy
    print(f"\n{line}\n(f) ENERGY over 10 simulated minutes, zero thrust\n{line}")
    s = replace(at_tank([1.5, 1.5, 1.2], (12.0, -8.0, 35.0)),
                velocity_frd=np.array([0.4, -0.2, 0.1]), rate_frd=np.array([0.5, -0.3, 0.2]))
    _, tr_f = D.simulate(free(v), s, ZERO, 600.0, record_every=400)
    print(f"  full model : E {tr_f.energy_j[0]:+.4f} -> {tr_f.energy_j[-1]:+.4f} J, "
          f"worst rise between samples {np.diff(tr_f.energy_j).max():+.3e} J, finite {np.all(np.isfinite(tr_f.energy_j))}")
    s2 = replace(at_tank([1.5, 1.5, 1.5], (25.0, -15.0, 40.0)),
                 velocity_frd=np.array([0.3, -0.15, 0.08]), rate_frd=np.array([0.6, -0.4, 0.3]))
    vu = free(v, neutral=True, no_damping=True)
    _, short = D.simulate(vu, s2, ZERO, 5.0, record_every=40)
    drift = float(np.abs(short.energy_j - short.energy_j[0]).max())
    print(f"  no damping : E conserved to {drift:.2e} J ABSOLUTE over 5 s - the formulation is exact "
          f"(C is skew-symmetric, so it does no work)")
    print("  Beyond a few seconds the UNDAMPED vehicle is exponentially unstable in rotation (the Munk")
    print("  moment: added mass 18.68 in heave against 6.36 in surge, righting moment only 1.15 N m), so")
    print("  round-off is amplified off the energy surface. Measured divergence, |omega|max [rad/s]:")
    _, dmp = D.simulate(free(v, neutral=True), s2, ZERO, 120.0, record_every=400)
    for T in (5.0, 20.0, 60.0, 120.0):
        _, u1 = D.simulate(vu, s2, ZERO, T, record_every=400)
        print(f"      t={T:6.1f} s  undamped |w|max {np.abs(u1.rate_frd).max():10.3f}   "
              f"E drift {np.abs(u1.energy_j - u1.energy_j[0]).max():10.3e} J")
    print(f"  With the published damping the same initial state stays bounded: |w|max "
          f"{np.abs(dmp.rate_frd).max():.3f} rad/s over 120 s, and E never rises "
          f"({np.diff(dmp.energy_j).max():+.2e} J worst). The SHIPPED model is the damped one.")
    tr_f2 = u1                  # the 120 s undamped run: what the plot shows diverging
    res["runs"]["f_energy"] = tr_f
    res["runs"]["f_energy_undamped"] = tr_f2
    res["energy_rise_j"] = float(np.diff(tr_f.energy_j).max())
    res["energy_drift_rel"] = drift

    # ---- (g) step size
    print(f"\n{line}\n(g) STEP SIZE: the same manoeuvre at dt and dt/2\n{line}")
    for label, cmd, dur, veh, note in (
            ("straight surge", CMD["surge"], 20.0, free(v, neutral=True), "no rotation, no heave"),
            ("surge + yaw", np.array([1.0, 0.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0]), 20.0,
             free(v, neutral=True), "~9 revolutions: POSITION is phase-sensitive, attitude is not"),
            ("tumbling", np.array([1.0, -0.6, 0.8, -0.4, 0.5, 0.3, -0.2, 0.7]), 20.0, free(v),
             "buoyant and tumbling: the trajectory itself is exponentially sensitive")):
        s = at_tank([1.0, 1.5, 1.2], (0.0, 0.0, 20.0))
        runs = [D.simulate(veh, s, cmd, dur, dt=p.dt / k)[0] for k in (1, 2, 4)]
        print(f"  {label}  ({note})")
        for lab, x, y in (("dt vs dt/2", runs[0], runs[1]), ("dt/2 vs dt/4", runs[1], runs[2])):
            dp = float(np.linalg.norm(x.position_ned - y.position_ned))
            ang = np.degrees(2.0 * np.arccos(min(1.0, abs(float(x.quat_frd_ned @ y.quat_frd_ned)))))
            print(f"      {lab:14s} position {dp * 1e6:12.3f} um   attitude {ang * 1e6:12.3f} udeg   "
                  f"velocity {np.linalg.norm(x.velocity_frd - y.velocity_frd):.2e} m/s")
        if label == "straight surge":
            res["dt_position_um"] = float(np.linalg.norm(runs[0].position_ned - runs[1].position_ned)) * 1e6

    # ---- the soft bound
    print(f"\n{line}\n(bound) SOFT TANK BOUND, non-physical (docs/ros_hil.md 6.5)\n{line}")
    print(f"  box lo {np.round(v.bounds.lo, 3)}  hi {np.round(v.bounds.hi, 3)}  surface {v.bounds.surface_z}")
    bound_runs = {}
    for name, start, cmd_b, idx, limit in (
            ("+x wall", [2.0, 1.5, 1.5], CMD["surge"], 0, v.bounds.hi[0]),
            ("floor", [1.5, 1.5, 1.2], -CMD["heave"], 2, v.bounds.lo[2]),
            ("surface", [1.5, 1.5, 2.4], CMD["heave"], 2, v.bounds.hi[2])):
        vb = D.make_vehicle(tank_size=TANK, sand_base=SAND, water_level=WL)
        _, tr = D.simulate(vb, at_tank(start), cmd_b, 40.0, record_every=40)
        xyz = tank_xyz(tr)
        over = (xyz[:, idx].max() - limit) if name != "floor" else (limit - xyz[:, idx].min())
        print(f"  {name:8s} worst penetration {over * 1000:7.2f} mm, activations {vb.bound_activations:6d}, "
              f"max recorded {vb.bound_max_penetration_m * 1000:6.2f} mm")
        bound_runs[name] = tr
    res["runs"].update({f"bound_{k.replace(' ', '_')}": t for k, t in bound_runs.items()})

    plots(out, v, res)
    tags(out)
    return res


# -------------------------------------------------------------------------------------------- plots

def plots(out: Path, v: D.Vehicle, res: dict) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                  # pragma: no cover - reported, not fatal
        print(f"\nmatplotlib unavailable ({exc}): no plots written. The numbers above are the result.")
        return
    out.mkdir(parents=True, exist_ok=True)
    p = v.params
    r = res["runs"]

    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    tr = r["a_zero_thrust"]
    ax[0, 0].plot(tr.t, tank_xyz(tr)[:, 2])
    ax[0, 0].axhline(WL, c="tab:blue", ls=":", lw=0.9, label="free surface 2.9 m (bound OFF in this run)")
    ax[0, 0].set(title=f"(a) zero thrust: {p.net_buoyancy_n:+.1f} N net buoyancy",
                 xlabel="simulated time [s]", ylabel="tank z [m]")
    ax[0, 0].legend(fontsize=8)
    ax[0, 1].plot(tr.t, -tr.velocity_frd[:, 2] * 1000.0)
    ax[0, 1].axhline(res["rise_mm_s"], ls="--", c="k", lw=0.8,
                     label=f"terminal {res['rise_mm_s']:.1f} mm/s")
    ax[0, 1].set(title="(a) rise speed", xlabel="simulated time [s]", ylabel="upward [mm/s]")
    ax[0, 1].legend()
    for name in AXES:
        tr = r[f"b_decay_{name}"]
        i = AXES.index(name)
        x = tr.velocity_frd[:, i] if i < 3 else tr.rate_frd[:, i - 3]
        ax[1, 0].plot(tr.t, x / x[0], label=f"{name} (t50 {res['decay'][name][1]:.2f} s)")
    ax[1, 0].set(title="(b) free decay, normalised", xlabel="simulated time [s]", ylabel="rate / rate(0)",
                 xlim=(0, 5))
    ax[1, 0].legend(fontsize=8)
    for name in AXES:
        tr = r[f"c_terminal_{name}"]
        i = AXES.index(name)
        x = tr.velocity_frd[:, i] if i < 3 else tr.rate_frd[:, i - 3]
        ax[1, 1].plot(tr.t, np.abs(x), label=f"{name} -> {abs(res['terminal'][name][1]):.3f}")
    ax[1, 1].set(title="(c) approach to terminal rate, full thrust", xlabel="simulated time [s]",
                 ylabel="|rate| [m/s or rad/s]", xlim=(0, 20))
    ax[1, 1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "rov_step1_abc.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    for name in ("roll", "pitch"):
        tr = r[f"e_restoring_{name}"]
        i = 0 if name == "roll" else 1
        ax[0, 0].plot(tr.t, tr.rpy_ned_deg[:, i],
                      label=f"{name}, period {res['restoring'][name][0]:.2f} s "
                            f"(undamped {res['restoring'][name][1]:.2f} s)")
    ax[0, 0].axhline(0.0, c="k", lw=0.5)
    ax[0, 0].set(title="(e) released from 20 deg: righting oscillation", xlabel="simulated time [s]",
                 ylabel="angle [deg, NED]")
    ax[0, 0].legend(fontsize=8)

    tr = r["f_energy"]
    ax[0, 1].plot(tr.t, tr.energy_j, label="full model")
    ax[0, 1].plot(r["f_energy_undamped"].t, r["f_energy_undamped"].energy_j,
                  label="damping zeroed (unstable)")
    ax[0, 1].set(title=f"(f) energy, zero thrust: shipped model never rises "
                       f"({res['energy_rise_j']:+.1e} J worst)",
                 xlabel="simulated time [s]", ylabel="E [J]", yscale="symlog")
    ax[0, 1].legend(fontsize=8)

    pwm = np.linspace(p.t200.pwm_min_us, p.t200.pwm_max_us, 801)
    thrust = p.t200.thrust_of_command(p.t200.command_of_pwm(pwm))
    ax[1, 0].plot(pwm, thrust)
    ax[1, 0].axvspan(*p.t200.deadband_us, color="0.85", label="dead band")
    ax[1, 0].axhline(p.t200.thrust_fwd_max_n, ls="--", lw=0.8, c="k",
                     label=f"fwd {p.t200.thrust_fwd_max_n:.1f} N")
    ax[1, 0].axhline(-p.t200.thrust_rev_max_n, ls=":", lw=0.8, c="k",
                     label=f"rev {p.t200.thrust_rev_max_n:.1f} N")
    ax[1, 0].set(title=f"T200 at 16 V, exponent {p.t200.curve_exponent} (GUESS)", xlabel="PWM [us]",
                 ylabel="thrust [N]")
    ax[1, 0].legend(fontsize=8)

    for name, key, idx in (("+x wall", "bound_+x_wall", 0), ("floor", "bound_floor", 2),
                           ("surface", "bound_surface", 2)):
        tr = r[key]
        ax[1, 1].plot(tr.t, tank_xyz(tr)[:, idx], label=f"{name} (tank {'xyz'[idx]})")
    for y in (v.bounds.lo[0], v.bounds.hi[0], v.bounds.lo[2], v.bounds.hi[2]):
        ax[1, 1].axhline(y, c="0.6", lw=0.6, ls="--")
    ax[1, 1].axhline(v.bounds.surface_z, c="tab:blue", lw=0.8, ls=":", label="free surface 2.9 m")
    ax[1, 1].set(title="soft bound: driven into the wall, the floor and the surface",
                 xlabel="simulated time [s]", ylabel="tank coordinate [m]")
    ax[1, 1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "rov_step1_efg.png", dpi=110)
    plt.close(fig)
    print(f"\nplots: {out / 'rov_step1_abc.png'}\n       {out / 'rov_step1_efg.png'}")


# ------------------------------------------------------------------------------- parameter tag inventory

def tags(out: Path) -> None:
    """Every GUESS / UNVERIFIED value in config/rov_bluerov2.yaml, read off the file's own comments.

    Line-based: it reports the ``key:`` on the same line as the tag, so a tag in a continuation comment is
    attributed to the key the comment belongs to only if that key is on the first line. The file is written
    with that in mind.
    """
    text = (REPO / "config" / "rov_bluerov2.yaml").read_text().splitlines()
    found: dict[str, list[str]] = {"GUESS": [], "UNVERIFIED": [], "V": []}
    key_re = re.compile(r"^\s*(?:-\s*)?\{?\s*([A-Za-z_][A-Za-z0-9_]*)\s*:")
    last_key: str | None = None
    for raw in text:
        if raw.lstrip().startswith("#"):
            continue            # a whole-line comment: the file header's own legend, or a continuation
        m = key_re.match(raw)
        if m:
            last_key = m.group(1)
        if last_key is None:
            continue
        for tag in ("GUESS", "UNVERIFIED"):
            if tag in raw:
                if last_key not in found[tag]:
                    found[tag].append(last_key)
                break
        else:
            if "#" in raw and re.search(r"\bV\b", raw.split("#", 1)[1]) and m:
                if last_key not in found["V"]:
                    found["V"].append(last_key)
    print("\n" + "-" * 100)
    print("PARAMETER TAG INVENTORY  config/rov_bluerov2.yaml")
    print("-" * 100)
    for tag in ("GUESS", "UNVERIFIED", "V"):
        keys = found[tag]
        print(f"  {tag:11s} {len(keys):3d}  {', '.join(keys) if keys else '-'}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=REPO / "output" / "rov",
                    help="where the plots go (default: output/rov, git-ignored)")
    args = ap.parse_args(argv)
    report(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
