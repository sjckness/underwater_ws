"""Per-frame extras of a recording run (record.enabled, --sink file; started by tools/record.py).

At every published camera frame (10 Hz, the same sim stamp as the left and right images) it writes, through the same
FileSink as the sonar and the cameras:
  gt/<seq>.npz + .json   the bubbles' ground truth: bubble_id (pool slot, stable while the bubble lives), position
                         (tank frame, m), diameter_m, distance_sonar_m, distance_camera_m (to the left camera)
and keeps one row per frame for poses.csv, written on close: sim time, the ROV body (position + quaternion w x y z,
FLU -> tank ENU, only for a pose source that has a body: script), the sonar, the left and right cameras (position +
roll/pitch/yaw in degrees, the repo's rot_rpy_deg convention).
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from .config import get
from .geometry import rot_rpy_deg, rpy_deg_of, sonar_rotation
from .sinks import Message


def _quat(rot) -> list:
    from .rov.frames import rot_to_quat
    return [float(v) for v in rot_to_quat(rot)]


class FrameRecorder:
    def __init__(self, cfg: dict, out_dir, sink, sim, motion):
        self.out, self.sink, self.sim, self.motion = Path(out_dir), sink, sim, motion
        self.max_range = get(cfg, "record.gt_max_range_m")
        s0 = np.eye(4)
        s0[:3, :3], s0[:3, 3] = sonar_rotation(cfg), get(cfg, "sonar.position")
        self.s0_inv = np.linalg.inv(s0)
        self.s0 = s0
        cam_rot = rot_rpy_deg(get(cfg, "camera.rpy_deg"))
        cams = {"camera_left": np.asarray(get(cfg, "camera.position"), float)}
        if get(cfg, "vehicle.stereo.enabled"):
            cams["camera_right"] = np.asarray(get(cfg, "camera.position"), float) + cam_rot @ np.array(
                [0.0, -float(get(cfg, "vehicle.stereo.baseline_m")), 0.0])
        self.cams = {}
        for k, p in cams.items():                  # camera poses at the configured rig pose (body frame x fwd, z up)
            m = np.eye(4)
            m[:3, :3], m[:3, 3] = cam_rot, p
            self.cams[k] = m
        self.rows: list = []
        self.n_gt: list = []

    def frame(self, seq: int, t: float, t_loop: float, sonar_pose) -> None:
        """``t`` the sim stamp (as the images), ``t_loop`` the pose sources' time (0 = end of the warm-up)."""
        if sonar_pose is None:
            s = self.s0
        else:
            s = np.eye(4)
            s[:3, :3], s[:3, 3] = sonar_pose.rot, sonar_pose.position
        rig = s @ self.s0_inv                      # what usd_scene.set_rig_pose applies to /World/rig
        row = {"seq": seq, "t_sim_s": round(float(t), 6), "t_traj_s": round(float(t_loop), 6)}
        if hasattr(self.motion, "body_at"):
            pb, rb = self.motion.body_at(t_loop)
            row.update({f"rov_{a}": float(v) for a, v in zip("xyz", pb)})
            row.update({f"rov_q{a}": v for a, v in zip("wxyz", _quat(rb))})
        for name, m in (("sonar", s), *((k, rig @ c) for k, c in self.cams.items())):
            row.update({f"{name}_{a}": float(v) for a, v in zip("xyz", m[:3, 3])})
            row.update({f"{name}_{a}_deg": float(v) for a, v in zip(("roll", "pitch", "yaw"), rpy_deg_of(m[:3, :3]))})
        self.rows.append(row)
        if self.sim is None:
            return
        snap = self.sim.snapshot()
        pos = snap["pos"].astype(np.float32)
        cam = (rig @ self.cams["camera_left"])[:3, 3]
        d_s = np.linalg.norm(pos - s[:3, 3], axis=1).astype(np.float32)
        d_c = np.linalg.norm(pos - cam, axis=1).astype(np.float32)
        keep = slice(None) if self.max_range is None else (np.minimum(d_s, d_c) <= float(self.max_range))
        arrays = {"bubble_id": snap["id"][keep], "position": pos[keep],
                  "diameter_m": (2.0 * snap["radius"][keep]).astype(np.float32),
                  "distance_sonar_m": d_s[keep], "distance_camera_m": d_c[keep]}
        self.n_gt.append(int(len(arrays["bubble_id"])))
        self.sink.publish(Message("gt", seq, t, arrays=arrays, meta={
            "n_bubbles": self.n_gt[-1], "n_alive": int(len(pos)), "max_range_m": self.max_range,
            "frame": "tank (map): x, y along the walls, z up, origin at the inner bottom corner",
            "bubble_id": "pool slot: stable while a bubble lives, reused after it bursts"}))

    def close(self) -> dict:
        if self.rows:
            keys = list(self.rows[0].keys())
            with open(self.out / "poses.csv", "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
                w.writerows(self.rows)
        return {"frames": len(self.rows), "gt_bubbles_mean": float(np.mean(self.n_gt)) if self.n_gt else 0.0,
                "gt_bubbles_max": int(max(self.n_gt)) if self.n_gt else 0}
