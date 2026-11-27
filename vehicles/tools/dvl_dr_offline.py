"""The DVL dead reckoning against the truth, offline: no Isaac, no Docker, no ROS, no GPU.

This is the rehearsal for `tools/dvl_drift.py`. It flies one of the trajectory presets, runs the real
sensor model over it, and reckons the track with the same functions the live chain uses -
`dvl_relay`'s FRD-to-FLU flip and `dvl_dr.correct`'s lever arm and baro - then scores it with the same
`drift()` the live tool scores with. What it gives you is the error budget of the sensor and the
geometry, today, before the autopilot and the cable are in the loop.

**What it is not.** The live chain's filter is `robot_localization`. Here the position is a plain
integral of the velocity. That is not a shortcut: with DVL velocity, an attitude and a depth as the only
inputs, *nothing observes x or y*, so an EKF's position estimate is the integral of its velocity
estimate. The filter adds smoothing and covariance bookkeeping, not information, in the two axes the
drift lives in. The number this prints is therefore the number to expect, and if the live chain is much
worse, the filter is mistuned rather than the sensor being bad.

**The attitude.** On the vehicle it comes from the autopilot's EKF3 over its own noisy IMU, baro and
compass (`docker/ardusub_sitl/params/hil.parm`). There is no autopilot here, so it is modelled: the
true attitude plus a constant heading bias, a slow heading walk and tilt noise. The heading bias is the
point of the exercise - in a metal tank the magnetometer is the least trustworthy thing on the vehicle,
and a heading error rotates the whole track rather than lengthening it. **The truth is never an input
to the reckoning**, only to the scoring.

    python tools/dvl_dr_offline.py                              # the default sweep
    python tools/dvl_dr_offline.py --preset lawnmower --duration 180
    python tools/dvl_dr_offline.py --yaw-bias 0 2 5 --seeds 5
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
for extra in (REPO, REPO / "ros" / "dvl_dr", REPO / "tools"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

from dvl_dr.correct import correct_lever_arm, depth_pose            # noqa: E402
from dvl_drift import drift                                         # noqa: E402
from uw import config, geometry, physics_udp, pose as pose_mod      # noqa: E402
from uw.rov import dvl as dvl_model                                 # noqa: E402
from uw.rov import frames as fr                                     # noqa: E402

TICK = 1.0 / 240.0                  # uw/main.py: one loop tick is one bubble substep


class Ahrs:
    """A stand-in for what the autopilot's EKF3 reports: the truth, degraded the way a real one is.

    A fixed random spectrum rather than a recursion, for the same reason ``uw.rov.dvl._BiasField`` uses
    one: the output must be a pure function of the seed and the time, not of the sample schedule.
    """

    def __init__(self, yaw_bias_deg: float, yaw_walk_deg: float, tilt_noise_deg: float, seed: int):
        rng = np.random.default_rng([int(seed), 41653])
        self.bias = math.radians(yaw_bias_deg)
        self.a = math.radians(yaw_walk_deg) * math.sqrt(2.0 / 6)
        self.w = 2 * np.pi / (60.0 * rng.uniform(1 / 3, 3, 6))
        self.ph = rng.uniform(0, 2 * np.pi, 6)
        self.tilt = math.radians(tilt_noise_deg)
        self.rng = np.random.default_rng([int(seed), 7717])

    def at(self, t: float, rot_true: np.ndarray) -> np.ndarray:
        """The estimated body FLU -> tank ENU rotation at time ``t``."""
        yaw = self.bias + self.a * float(np.sum(np.sin(self.w * t + self.ph)))
        c, s = math.cos(yaw), math.sin(yaw)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])      # a heading error is about world z
        rot = rz @ rot_true
        if self.tilt > 0.0:
            r, p = self.rng.normal(0.0, self.tilt, 2)
            rot = rot @ geometry.rot_rpy_deg(np.degrees([r, p, 0.0]))
        return rot


def fly(cfg: dict, scene, duration: float, ahrs: Ahrs, baro_stddev: float, gyro_stddev: float,
        seed: int):
    """One run: returns (t, truth position, reckoned position, how many reports lost bottom lock)."""
    motion = pose_mod.sensor_pose(cfg, scene)
    d = dvl_model.from_config(cfg, scene)
    bm = dvl_model.BodyMotion(motion, *physics_udp.mount(cfg))
    water_level = float(config.get(cfg, "tank.water_level"))
    r_flu = tuple(fr.vec_flu_frd(d.mount.position_frd_m))
    rng = np.random.default_rng([int(seed), 9311])

    ts, truth, est = [], [], []
    p_est = None
    t_prev = None
    v_hold = np.zeros(3)            # body FLU; a zero-order hold, which is what the filter's state does
    lost = 0
    for k in range(1, int(duration / TICK) + 1):
        t = k * TICK
        if not d.due(t):
            continue
        p_true, rot_true, v_frd, w_frd = bm.at(t)
        report = d.sample(t, p_true, rot_true, v_frd, w_frd)
        if p_est is None:
            p_est = np.asarray(p_true, float).copy()        # initialised, never told the truth again
            t_prev = t
        # integrate the velocity we held over the interval just elapsed, then take the new measurement.
        # Forward, not backward: during a gap the chain coasts on the state it had, as dvl_dr does when
        # it drops a report and lets the filter propagate. Applying the post-gap sample across the gap
        # would be using a measurement from after the fact, and on a turn it is badly wrong.
        p_est = p_est + ahrs.at(t, rot_true) @ v_hold * (t - t_prev)
        t_prev = t
        if report.velocity_valid:
            # exactly the live path: the relay flips FRD -> FLU, the preprocessor removes the lever arm
            v_flu = tuple(fr.vec_flu_frd(report.velocity))
            w_meas = np.asarray(w_frd, float) + rng.normal(0.0, gyro_stddev, 3)
            v_hold = np.asarray(correct_lever_arm(v_flu, tuple(fr.angvel_flu_frd(w_meas)), r_flu))
        else:
            lost += 1
        # the baro pins z, as pose0 does in ekf.yaml
        depth_true = water_level - float(p_true[2])
        pressure = 101325.0 + 1026.0 * 9.80665 * (depth_true + rng.normal(0.0, baro_stddev))
        p_est[2], _ = depth_pose(pressure, 101325.0, 1026.0, water_level)
        ts.append(t)
        truth.append(np.asarray(p_true, float))
        est.append(p_est.copy())
    return np.array(ts), np.array(truth), np.array(est), lost


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preset", default="lawnmower", help="sonar.trajectory.preset: the path to fly")
    ap.add_argument("--size", default="basin", help="size preset: tank | basin | room9")
    ap.add_argument("--height", type=float, default=0.7, help="trajectory height over the floor [m]")
    ap.add_argument("--margin", type=float, default=0.35, help="wall margin [m]: it must clear the beams")
    ap.add_argument("--pitch", default="0.0", help="trajectory pitch [deg], or 'track' to aim at the leak. "
                                                   "A pitched vehicle swings its beams out towards the walls")
    ap.add_argument("--duration", type=float, default=180.0, help="seconds of simulated time")
    ap.add_argument("--yaw-bias", type=float, nargs="+", default=[0.0, 1.0, 2.0, 5.0, 10.0],
                    help="heading bias to sweep [deg]: the term that dominates in a metal tank")
    ap.add_argument("--yaw-walk", type=float, default=0.5, help="slow heading walk [deg]")
    ap.add_argument("--tilt-noise", type=float, default=0.2, help="roll/pitch noise [deg]")
    ap.add_argument("--baro-stddev", type=float, default=0.02, help="depth noise [m]")
    ap.add_argument("--gyro-stddev", type=float, default=0.002, help="angular rate noise [rad/s]")
    ap.add_argument("--seeds", type=int, default=3, help="runs per bias, to see the spread")
    ap.add_argument("--out", default=str(REPO / "output" / "dvl"))
    ap.add_argument("--json", action="store_true", help="also write the table as JSON")
    args = ap.parse_args(argv)

    base = ["sonar.pose_source=trajectory", f"sonar.trajectory.preset={args.preset}",
            f"sonar.trajectory.height={args.height}", f"sonar.trajectory.margin={args.margin}",
            f"sonar.trajectory.pitch={args.pitch}"]
    cfg = config.load(presets=[args.size], overrides=base)
    scene = geometry.build_scene(cfg)
    tilt = float(config.get(cfg, "dvl.beams.tilt_deg"))
    pitch = 0.0 if args.pitch == "track" else abs(float(args.pitch))
    # a pitched vehicle swings its forward beams out: the reach is set by tilt + pitch, not tilt alone
    reach = args.height * math.tan(math.radians(tilt + pitch))
    margin = float(config.get(cfg, "sonar.trajectory.margin"))
    print(f"{args.size}/{args.preset} at {args.height} m, {args.duration:.0f} s of sim time, "
          f"{args.seeds} seed(s) per bias; yaw walk {args.yaw_walk} deg, tilt noise {args.tilt_noise} deg, "
          f"baro {args.baro_stddev} m")
    print(f"beams reach {reach:.3f} m sideways at this altitude ({tilt:.1f} deg tilt + {pitch:.0f} deg "
          f"pitch); the trajectory's wall margin is {margin:.2f} m"
          + ("" if margin > reach else "  <-- the beams cross the wall at the margin"))
    head = (f"{'yaw bias':>9}{'distance':>10}{'final err':>11}{'drift':>9}{'horiz':>8}"
            f"{'z err':>8}{'no lock':>9}")
    print()
    print(head)
    print("-" * len(head))
    rows = []
    for bias in args.yaw_bias:
        per_seed = []
        for s in range(args.seeds):
            seed = int(cfg["seed"]) + 1000 * s
            cfg_s = config.load(presets=[args.size], overrides=base + [f"seed={seed}"])
            sc = scene if s == 0 else geometry.build_scene(cfg_s)
            ahrs = Ahrs(bias, args.yaw_walk, args.tilt_noise, seed)
            t, truth, est, lost = fly(cfg_s, sc, args.duration, ahrs, args.baro_stddev,
                                      args.gyro_stddev, seed)
            res = drift(t, truth, t, est)
            res["lost_lock"] = lost
            res["reports"] = int(len(t))
            # a heading error rotates the track, so its final error goes with the NET displacement,
            # while the drift percentage is quoted against the distance travelled, as DVLs are
            res["net_displacement_m"] = float(np.linalg.norm(truth[-1] - truth[0]))
            per_seed.append(res)
        med = {k: float(np.median([r[k] for r in per_seed])) for k in per_seed[0] if k != "label"}
        med["yaw_bias_deg"] = bias
        rows.append(med)
        print(f"{bias:>7.1f}°{med['distance_travelled_m']:>9.2f}m{med['final_error_m']:>10.3f}m"
              f"{med['drift_percent_of_distance']:>8.2f}%{med['drift_percent_horizontal']:>7.2f}%"
              f"{med['final_error_z_m']:>7.3f}m{int(med['lost_lock']):>9d}")
    print()
    base0 = rows[0]
    lock = 100.0 * (1.0 - base0["lost_lock"] / max(base0["reports"], 1))
    print(f"with no heading error the reckoning drifts {base0['drift_percent_of_distance']:.2f} % of "
          f"{base0['distance_travelled_m']:.1f} m travelled, over {int(base0['reports'])} reports "
          f"({lock:.0f} % with bottom lock)")
    if lock < 95.0:
        print(f"  WARNING: {100 - lock:.0f} % of the reports had no bottom lock, so this measures the "
              f"coast more than the DVL. Fly higher margins, a different preset, or a larger size.")
    if len(rows) > 1:
        net = base0["net_displacement_m"]
        print(f"net displacement {net:.2f} m (the path doubles back, so it is far short of the "
              f"{base0['distance_travelled_m']:.1f} m travelled)")
        print(f"{'yaw bias':>9}{'measured':>11}{'2 sin(t/2) x net':>18}{'excess':>9}")
        for r in rows:
            closed = 2 * math.sin(math.radians(r["yaw_bias_deg"]) / 2) * net
            print(f"{r['yaw_bias_deg']:>7.1f}°{r['final_error_m']:>10.3f}m{closed:>17.3f}m"
                  f"{r['final_error_m'] - closed:>8.3f}m")
        print("the closed form is a rigid rotation of the track; what is left over is the sensor itself")
    if args.json:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        p = out / f"dr_offline_{args.preset}.json"
        p.write_text(json.dumps({"preset": args.preset, "duration_s": args.duration, "rows": rows},
                                indent=1), encoding="utf-8")
        print(f"[dvl_dr_offline] {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
