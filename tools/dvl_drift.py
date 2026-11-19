"""How far the DVL dead reckoning walks away from the truth, as a number.

This is the evaluator, and it is the only thing in the project allowed to look at both sides at once.
It subscribes the estimate on the HIL domain and the simulator's truth on the ground-truth domain, with
a separate participant for each - the arrangement ``uw.ros_bridge`` already uses so that no node on the
HIL domain can reach ``/gt`` at all (``docs/ros_hil.md`` section 2.5). Nothing it computes goes back
into the stack; it writes a JSON and prints a table.

What it reports is the quantity the roadmap asks for: position error as a percentage of distance
travelled, which is how a DVL's accuracy is quoted and the only way to compare two runs of different
length. It reports the horizontal error separately, because that is the unobserved part - the baro sees
z, and nothing sees x and y.

Running it (host, ROS 2 Jazzy sourced; it needs no Isaac and no GPU):

    source /opt/ros/jazzy/setup.bash
    python3 tools/dvl_drift.py --duration 120 --label baseline
    python3 tools/dvl_drift.py --duration 120 --label mag20      # after SIM_MAG1_OFS in hil.parm
    python3 tools/dvl_drift.py --compare output/dvl/*.json

Heading is the term to sweep. In a metal tank the magnetometer is unreliable, and a heading error
rotates the whole dead-reckoned track rather than lengthening it, so the drift grows with the distance
travelled and not with the velocity error. Injecting it where a real one lives - SITL's compass model,
``SIM_MAG1_OFS`` in ``docker/ardusub_sitl/params/hil.parm`` - and re-running this is what turns "the
heading will dominate" from a remark in a document into a curve.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

OUT_DEFAULT = REPO / "output" / "dvl"


# ------------------------------------------------------------------------------------------ analysis

def resample(truth_t, truth_p, est_t):
    """The truth at each estimate's stamp, linearly interpolated (pure: tested without ROS)."""
    truth_t, truth_p, est_t = np.asarray(truth_t), np.asarray(truth_p), np.asarray(est_t)
    keep = (est_t >= truth_t[0]) & (est_t <= truth_t[-1])
    out = np.stack([np.interp(est_t[keep], truth_t, truth_p[:, i]) for i in range(3)], 1)
    return est_t[keep], out, keep


def path_length(p) -> float:
    p = np.asarray(p)
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))) if len(p) > 1 else 0.0


def drift(truth_t, truth_p, est_t, est_p) -> dict:
    """Error of a dead-reckoned track against the truth (pure).

    Both tracks are shifted to start at the origin before comparison. A dead-reckoned track has no
    absolute reference - the filter starts wherever it was told to - so comparing absolute positions
    would measure the initialisation, not the drift.
    """
    est_p = np.asarray(est_p, float)
    t, truth, keep = resample(truth_t, truth_p, est_t)
    est = est_p[keep]
    if len(t) < 2:
        raise ValueError(f"only {len(t)} samples overlap in time: did both sides run together?")
    err = (est - est[0]) - (truth - truth[0])
    dist = path_length(truth)
    norm = np.linalg.norm(err, axis=1)
    horiz = np.linalg.norm(err[:, :2], axis=1)
    return {
        "samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "distance_travelled_m": dist,
        "final_error_m": float(norm[-1]),
        "rms_error_m": float(np.sqrt(np.mean(norm ** 2))),
        "max_error_m": float(norm.max()),
        "final_error_horizontal_m": float(horiz[-1]),
        "rms_error_horizontal_m": float(np.sqrt(np.mean(horiz ** 2))),
        "final_error_z_m": float(abs(err[-1, 2])),
        "drift_percent_of_distance": float(100.0 * norm[-1] / dist) if dist > 1e-6 else float("nan"),
        "drift_percent_horizontal": float(100.0 * horiz[-1] / dist) if dist > 1e-6 else float("nan"),
    }


def report(res: dict, label: str) -> str:
    rows = [
        ("samples", f"{res['samples']}"),
        ("duration", f"{res['duration_s']:.1f} s"),
        ("distance travelled", f"{res['distance_travelled_m']:.2f} m"),
        ("final error", f"{res['final_error_m']:.3f} m"),
        ("  horizontal (unobserved)", f"{res['final_error_horizontal_m']:.3f} m"),
        ("  vertical (baro-aided)", f"{res['final_error_z_m']:.3f} m"),
        ("rms error", f"{res['rms_error_m']:.3f} m"),
        ("max error", f"{res['max_error_m']:.3f} m"),
        ("drift", f"{res['drift_percent_of_distance']:.2f} % of distance"),
        ("  horizontal", f"{res['drift_percent_horizontal']:.2f} %"),
    ]
    w = max(len(k) for k, _ in rows)
    head = f"DVL dead reckoning vs /gt   [{label}]"
    return "\n".join([head, "-" * len(head)] + [f"{k.ljust(w)}  {v}" for k, v in rows])


def plot(out_dir: Path, label: str, truth, est) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                        # noqa: BLE001 - a plot is a convenience, not the result
        print(f"[dvl_drift] no plot ({exc})")
        return
    truth, est = np.asarray(truth), np.asarray(est)
    fig, ax = plt.subplots(1, 2, figsize=(11, 5))
    ax[0].plot(truth[:, 0] - truth[0, 0], truth[:, 1] - truth[0, 1], label="truth", lw=2)
    ax[0].plot(est[:, 0] - est[0, 0], est[:, 1] - est[0, 1], label="dead reckoning", lw=1.2)
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("y [m]")
    ax[0].legend()
    ax[0].set_title("track, origin-aligned")
    err = np.linalg.norm((est - est[0]) - (truth - truth[0]), axis=1)
    ax[1].plot(np.cumsum(np.r_[0.0, np.linalg.norm(np.diff(truth, axis=0), axis=1)]), err)
    ax[1].set_xlabel("distance travelled [m]")
    ax[1].set_ylabel("position error [m]")
    ax[1].set_title("drift against distance, not time")
    fig.tight_layout()
    path = out_dir / f"drift_{label}.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"[dvl_drift] {path}")


# -------------------------------------------------------------------------------------- the recording

def record(duration: float, vehicle: str, domain: int, gt_domain: int):
    """Collect the estimate and the truth until ``duration`` seconds of wall clock, or Ctrl-C."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                          reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    est_t, est_p, gt_t, gt_p = [], [], [], []

    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node("dvl_drift", context=ctx)
    node.create_subscription(Odometry, f"/{vehicle}/nav/odom", lambda m: (
        est_t.append(stamp(m.header)),
        est_p.append([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])), reliable)

    gt_ctx = rclpy.context.Context()
    rclpy.init(context=gt_ctx, domain_id=gt_domain)
    gt_node = rclpy.create_node("dvl_drift_gt", context=gt_ctx)
    gt_node.create_subscription(PoseStamped, "/gt/sonar_pose", lambda m: (
        gt_t.append(stamp(m.header)),
        gt_p.append([m.pose.position.x, m.pose.position.y, m.pose.position.z])), reliable)

    ex, gt_ex = SingleThreadedExecutor(context=ctx), SingleThreadedExecutor(context=gt_ctx)
    ex.add_node(node)
    gt_ex.add_node(gt_node)
    t0 = time.monotonic()
    print(f"[dvl_drift] estimate on domain {domain}, truth on domain {gt_domain}; Ctrl-C to stop")
    try:
        while time.monotonic() - t0 < duration:
            ex.spin_once(timeout_sec=0.02)
            gt_ex.spin_once(timeout_sec=0.02)
            if len(est_t) and len(est_t) % 200 == 0:
                print(f"\r[dvl_drift] {len(est_t)} estimates, {len(gt_t)} truth poses", end="")
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        gt_node.destroy_node()
        rclpy.try_shutdown(context=ctx)
        rclpy.try_shutdown(context=gt_ctx)
    return est_t, est_p, gt_t, gt_p


def body_truth(gt_p, cfg):
    """The sonar's true pose is what /gt carries; the estimate is the body's. Undo the mount.

    Only the translation matters here and the rig is rigid, so this is the sonar's offset in the body
    frame rotated by the body's attitude - which for the tank's level poses is the configured rotation.
    ``uw.physics_udp.mount`` is the same function ``ArduSubPose`` applies in the other direction.
    """
    from uw.geometry import body_rotation
    from uw.physics_udp import mount
    _, p_mount = mount(cfg)
    return np.asarray(gt_p, float) - body_rotation(cfg) @ np.asarray(p_mount, float)


# ---------------------------------------------------------------------------------------------- cli

def compare(paths) -> str:
    rows = []
    for p in paths:
        d = json.loads(Path(p).read_text(encoding="utf-8"))
        rows.append((d.get("label", Path(p).stem), d["distance_travelled_m"], d["final_error_m"],
                     d["drift_percent_of_distance"], d["drift_percent_horizontal"]))
    head = f"{'run':<16}{'distance':>10}{'final err':>11}{'drift':>9}{'horiz':>8}"
    out = [head, "-" * len(head)]
    for name, dist, err, pct, hpct in sorted(rows, key=lambda r: r[3]):
        out.append(f"{name:<16}{dist:>9.2f}m{err:>10.3f}m{pct:>8.2f}%{hpct:>7.2f}%")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--duration", type=float, default=120.0, help="seconds of wall clock to record")
    ap.add_argument("--vehicle", default="rov1")
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    ap.add_argument("--label", default="run", help="names the JSON and the plot, e.g. the SIM_MAG1_OFS used")
    ap.add_argument("--domain", type=int, default=int(os.environ.get("UW_ROS_DOMAIN_ID", "73")))
    ap.add_argument("--gt-domain", type=int, default=74)
    ap.add_argument("--compare", nargs="+", default=None, help="tabulate finished runs instead of recording")
    args = ap.parse_args(argv)

    if args.compare:
        print(compare(args.compare))
        return 0

    from uw import config
    cfg = config.load()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    est_t, est_p, gt_t, gt_p = record(args.duration, args.vehicle, args.domain, args.gt_domain)
    if len(est_t) < 2 or len(gt_t) < 2:
        print(f"[dvl_drift] not enough data ({len(est_t)} estimates, {len(gt_t)} truth poses). Is the "
              f"estimator running, and is the sim publishing /gt on domain {args.gt_domain}?")
        return 1

    truth = body_truth(gt_p, cfg)
    res = drift(gt_t, truth, est_t, est_p)
    res["label"] = args.label
    res["vehicle"] = args.vehicle
    print(report(res, args.label))
    path = out_dir / f"drift_{args.label}.json"
    path.write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(f"[dvl_drift] {path}")
    _, truth_r, keep = resample(gt_t, truth, est_t)
    plot(out_dir, args.label, truth_r, np.asarray(est_p)[keep])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
