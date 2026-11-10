"""Frames: the one place that converts between the repo's tank/body frames and ArduPilot's.

Four frames, and no other module in ``uw.rov`` may convert between them by hand.

* **tank, a.k.a. ``map`` (ENU)** - the repo's world frame: x and y along the tank walls, z up, origin at the
  inner bottom corner (``config/tank.yaml`` header). ``docs/ros_hil.md`` section 6.2 declares it ENU -
  tank +x = East, +y = North, +z = Up - and fixes the yaw offset between tank +x and true north at 0 deg.
* **body FLU** - the repo's body frame: x forward, y port, z up (ROS REP-103). This is what
  ``uw.geometry.rot_rpy_deg`` and ``uw.pose.Pose`` already use.
* **earth NED** - ArduPilot's: ``N = y_tank``, ``E = x_tank``, ``D = water_level - z_tank``
  (``docs/ros_hil.md`` 6.2), so the NED origin sits on the free surface directly above the tank's origin
  corner and D is depth below the surface.
* **body FRD** - ArduPilot's: x forward, y starboard, z down.

**Isaac's world frame is the tank frame.** ``uw/usd_scene.py`` sets ``UsdGeom.SetStageUpAxis(stage, z)`` and
``SetStageMetersPerUnit(stage, 1.0)``, and the geometry is authored in tank metres, so tank -> Isaac is the
identity. The ``isaac_*`` helpers below exist to give that seam a name rather than to compute anything; if a
future stage is Y-up or in centimetres, they are the only things that change.

Both conversions are **proper rotations** (det = +1), so handedness is preserved and angular velocity needs no
pseudovector sign fix:

* ``M_ENU_NED`` is a 180 deg rotation about ``(1, 1, 0)/sqrt(2)`` and maps ``(x, y, z) -> (y, x, -z)``.
* ``M_FLU_FRD`` is a 180 deg rotation about the forward axis and maps ``(x, y, z) -> (x, -y, -z)``.

Both are involutions (``M @ M == I``), so each direction uses the same matrix - hence one function per
quantity, not two.

**Euler angles.** ``rot_from_rpy_deg`` is the repo's ZYX convention, identical to
``uw.geometry.rot_rpy_deg`` (a test asserts that). The *same* formula reads differently in the two body
frames, because only the z direction differs:

* in FLU/ENU, ``Ry(+pitch)`` tilts forward toward -z, i.e. **pitch positive = nose down** - the repo's
  documented convention, and what ``sonar.pitch_deg: 15.0`` means;
* in FRD/NED, ``Ry(+pitch)`` tilts forward toward -z = up, i.e. **pitch positive = nose up** - the aerospace
  convention ArduPilot reports.

So the sandwich below implies, and ``tests/rov/test_frames.py`` checks:
``roll_NED = roll_ENU``, ``pitch_NED = -pitch_ENU``, ``yaw_NED = 90 deg - yaw_ENU``.

**Quaternions** are scalar-first unit quaternions ``(w, x, y, z)`` that rotate a *body* vector into its
*parent* frame, which is ArduPilot's ``Quaternion`` order (q1..q4 = w, x, y, z). ROS uses ``(x, y, z, w)``;
``quat_to_ros_xyzw`` / ``quat_from_ros_xyzw`` are the only place that order appears.

Portable: numpy and the standard library only, pathlib-free (no file access at all).
"""
from __future__ import annotations

import math

import numpy as np

# (x, y, z)_ENU <-> (y, x, -z)_NED. Involution, det = +1.
M_ENU_NED = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]])
# (x, y, z)_FLU <-> (x, -y, -z)_FRD. Involution, det = +1.
M_FLU_FRD = np.array([[1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0]])


# ------------------------------------------------------------------------------------ small linear algebra

def skew(v) -> np.ndarray:
    """The matrix with ``skew(a) @ b == cross(a, b)``."""
    x, y, z = np.asarray(v, float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rot_from_rpy_deg(rpy_deg) -> np.ndarray:
    """Body-to-parent rotation ``Rz(yaw) Ry(pitch) Rx(roll)`` from degrees.

    Identical to ``uw.geometry.rot_rpy_deg`` (duplicated so that ``uw.rov`` depends on nothing in the repo;
    ``test_frames.py`` asserts the two agree). See the module docstring for what the pitch sign means in each
    body frame.
    """
    r, p, y = np.radians(np.asarray(rpy_deg, float))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rpy_deg_from_rot(rot) -> np.ndarray:
    """Roll, pitch, yaw [deg] of a body-to-parent rotation: the inverse of ``rot_from_rpy_deg``."""
    m = np.asarray(rot, float)
    return np.degrees([math.atan2(m[2, 1], m[2, 2]),
                       math.asin(max(-1.0, min(1.0, -m[2, 0]))),
                       math.atan2(m[1, 0], m[0, 0])])


# ------------------------------------------------------------------------------------------- quaternions

def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, float)
    n = float(np.linalg.norm(q))
    if n == 0.0:
        raise ValueError("zero quaternion")
    q = q / n
    return -q if q[0] < 0.0 else q           # one representative per rotation: w >= 0


def quat_identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0])


def quat_conj(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, float)
    return np.array([w, -x, -y, -z])


def quat_mul(a, b) -> np.ndarray:
    """Hamilton product: the rotation ``b`` followed by ``a`` (both body-to-parent)."""
    aw, ax, ay, az = np.asarray(a, float)
    bw, bx, by, bz = np.asarray(b, float)
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def quat_to_rot(q) -> np.ndarray:
    """The body-to-parent rotation matrix of a unit quaternion.

    Written into a preallocated array rather than from nested lists: this sits in the 400 Hz RK4 inner loop,
    where building a 3x3 from Python lists costs more than the arithmetic.
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    xx, yy, zz = x * x, y * y, z * z
    m = np.empty((3, 3))
    m[0, 0] = 1.0 - 2.0 * (yy + zz); m[0, 1] = 2.0 * (x * y - w * z); m[0, 2] = 2.0 * (x * z + w * y)
    m[1, 0] = 2.0 * (x * y + w * z); m[1, 1] = 1.0 - 2.0 * (xx + zz); m[1, 2] = 2.0 * (y * z - w * x)
    m[2, 0] = 2.0 * (x * z - w * y); m[2, 1] = 2.0 * (y * z + w * x); m[2, 2] = 1.0 - 2.0 * (xx + yy)
    return m


def cross3(a, b) -> np.ndarray:
    """``a x b`` for two 3-vectors. ``np.cross``'s dispatch overhead dominates at this size, and this is in
    the inner loop."""
    a0, a1, a2 = float(a[0]), float(a[1]), float(a[2])
    b0, b1, b2 = float(b[0]), float(b[1]), float(b[2])
    out = np.empty(3)
    out[0] = a1 * b2 - a2 * b1
    out[1] = a2 * b0 - a0 * b2
    out[2] = a0 * b1 - a1 * b0
    return out


def rot_to_quat(rot) -> np.ndarray:
    """The unit quaternion of a rotation matrix, by the largest-component branch (no near-singular division)."""
    m = np.asarray(rot, float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] >= m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    return quat_normalize(q)


def quat_rotate(q, v) -> np.ndarray:
    """Rotate a body vector into the parent frame."""
    return quat_to_rot(q) @ np.asarray(v, float)


def quat_rotate_inv(q, v) -> np.ndarray:
    """Rotate a parent vector into the body frame."""
    return quat_to_rot(q).T @ np.asarray(v, float)


def quat_from_rpy_deg(rpy_deg) -> np.ndarray:
    return rot_to_quat(rot_from_rpy_deg(rpy_deg))


def rpy_deg_from_quat(q) -> np.ndarray:
    return rpy_deg_from_rot(quat_to_rot(q))


def quat_derivative(q, omega_body) -> np.ndarray:
    """``dq/dt = 0.5 * q (x) (0, omega_body)`` for a body-rate vector in the body frame."""
    qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    p, r, s = float(omega_body[0]), float(omega_body[1]), float(omega_body[2])
    out = np.empty(4)
    out[0] = 0.5 * (-qx * p - qy * r - qz * s)
    out[1] = 0.5 * (qw * p + qy * s - qz * r)
    out[2] = 0.5 * (qw * r - qx * s + qz * p)
    out[3] = 0.5 * (qw * s + qx * r - qy * p)
    return out


def quat_to_ros_xyzw(q) -> np.ndarray:
    """The only place the ROS ``(x, y, z, w)`` order is produced."""
    w, x, y, z = np.asarray(q, float)
    return np.array([x, y, z, w])


def quat_from_ros_xyzw(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float)
    return quat_normalize([w, x, y, z])


# ------------------------------------------------------------------------- vectors, positions, rotations

def vec_enu_ned(v) -> np.ndarray:
    """An earth-frame *vector* (velocity, acceleration, force) between ENU and NED. Its own inverse."""
    return M_ENU_NED @ np.asarray(v, float)


def vec_flu_frd(v) -> np.ndarray:
    """A body-frame *vector* between FLU and FRD. Its own inverse."""
    return M_FLU_FRD @ np.asarray(v, float)


def angvel_flu_frd(w) -> np.ndarray:
    """A body-frame angular velocity between FLU and FRD.

    The same matrix as ``vec_flu_frd``: ``M_FLU_FRD`` has det +1, so an angular velocity (a pseudovector)
    transforms exactly like a vector. A reflection would need an extra sign, which is why ``test_frames.py``
    asserts both determinants are +1.
    """
    return M_FLU_FRD @ np.asarray(w, float)


def position_tank_to_ned(p_tank, water_level: float) -> np.ndarray:
    """A tank position [m] as ArduPilot NED [m] relative to the origin on the free surface above
    ``(x=0, y=0)``: ``N = y``, ``E = x``, ``D = water_level - z`` (``docs/ros_hil.md`` 6.2)."""
    x, y, z = np.asarray(p_tank, float)
    return np.array([y, x, float(water_level) - z])


def position_ned_to_tank(p_ned, water_level: float) -> np.ndarray:
    n, e, d = np.asarray(p_ned, float)
    return np.array([e, n, float(water_level) - d])


def rot_flu_enu_to_frd_ned(rot) -> np.ndarray:
    """A body-to-world rotation from FLU->ENU to FRD->NED.

    ``R_frd_ned = M_ENU_NED @ R_flu_enu @ M_FLU_FRD``. Both matrices are involutions, so the same expression
    inverts the conversion - see ``rot_frd_ned_to_flu_enu``.
    """
    return M_ENU_NED @ np.asarray(rot, float) @ M_FLU_FRD


def rot_frd_ned_to_flu_enu(rot) -> np.ndarray:
    return M_ENU_NED @ np.asarray(rot, float) @ M_FLU_FRD


def quat_flu_enu_to_frd_ned(q) -> np.ndarray:
    return rot_to_quat(rot_flu_enu_to_frd_ned(quat_to_rot(q)))


def quat_frd_ned_to_flu_enu(q) -> np.ndarray:
    return rot_to_quat(rot_frd_ned_to_flu_enu(quat_to_rot(q)))


# ------------------------------------------------------------------------------------------- full poses

def pose_tank_to_ned(p_tank, q_flu_enu, water_level: float) -> tuple[np.ndarray, np.ndarray]:
    """A full pose (tank position, FLU->ENU attitude) as (NED position, FRD->NED attitude)."""
    return position_tank_to_ned(p_tank, water_level), quat_flu_enu_to_frd_ned(q_flu_enu)


def pose_ned_to_tank(p_ned, q_frd_ned, water_level: float) -> tuple[np.ndarray, np.ndarray]:
    return position_ned_to_tank(p_ned, water_level), quat_frd_ned_to_flu_enu(q_frd_ned)


def twist_ned_to_tank(v_body_frd, w_body_frd) -> tuple[np.ndarray, np.ndarray]:
    """A body twist from FRD to FLU (both still body-frame, as ROS ``Twist`` in ``base_link`` wants)."""
    return vec_flu_frd(v_body_frd), angvel_flu_frd(w_body_frd)


# ------------------------------------------------------------------------------------- the Isaac seam

def isaac_position_from_tank(p_tank) -> np.ndarray:
    """Tank metres -> Isaac stage units.

    Identity: ``uw/usd_scene.py`` sets the stage up axis to Z and metersPerUnit to 1.0 and authors the tank in
    metres. Named so that a Y-up or centimetre stage changes this function and nothing else.
    """
    return np.asarray(p_tank, float).copy()


def isaac_quat_from_tank(q_flu_enu) -> np.ndarray:
    """FLU->ENU attitude -> the stage orientation, scalar-first ``(w, x, y, z)`` as USD's ``Gf.Quatd`` wants.

    Identity for the same reason as ``isaac_position_from_tank``.
    """
    return quat_normalize(q_flu_enu)
