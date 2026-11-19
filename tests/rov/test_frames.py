"""uw.rov.frames: round trips, the handedness checks and the concrete repo cases.

The point of these tests is that there is exactly ONE place converting between the repo's tank/ENU + body/FLU
frames and ArduPilot's NED + FRD, and that it agrees with (a) the repo's existing rotation convention,
(b) docs/ros_hil.md section 6.2, and (c) itself in both directions.
"""
from __future__ import annotations

import numpy as np
import pytest

from uw.rov import frames as F

RNG = np.random.default_rng(20261109)


def random_quats(n: int) -> list[np.ndarray]:
    q = RNG.normal(size=(n, 4))
    return [F.quat_normalize(v) for v in q]


# ------------------------------------------------------------------------------------- handedness

def test_both_conversions_are_proper_rotations_and_involutions():
    """det = +1 (handedness preserved, so an angular velocity needs no extra sign) and M @ M = I (so one
    function serves both directions)."""
    for name, m in (("ENU<->NED", F.M_ENU_NED), ("FLU<->FRD", F.M_FLU_FRD)):
        assert np.isclose(np.linalg.det(m), 1.0), f"{name}: det = {np.linalg.det(m)}, not a proper rotation"
        assert np.allclose(m @ m.T, np.eye(3)), f"{name}: not orthogonal"
        assert np.allclose(m @ m, np.eye(3)), f"{name}: not an involution"


def test_enu_ned_is_a_180_degree_rotation_about_the_diagonal():
    """ENU<->NED is the rotation by pi about (1, 1, 0)/sqrt(2): the axis is fixed, the normal is flipped."""
    axis = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
    assert np.allclose(F.vec_enu_ned(axis), axis)
    assert np.allclose(F.vec_enu_ned([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0])


def test_yaw_plus_90_in_enu_is_heading_north_in_ned():
    """The handedness check the design asks for by name.

    Tank +y is North (docs/ros_hil.md 6.2 fixes the yaw offset at 0 deg), so a vehicle yawed +90 deg in the
    ENU/FLU convention points North, which is heading 0 in ArduPilot's NED. Its forward axis must come out as
    +N in both conventions.
    """
    r_enu = F.rot_from_rpy_deg([0.0, 0.0, 90.0])
    fwd_enu = r_enu @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(fwd_enu, [0.0, 1.0, 0.0]), "yaw +90 in ENU must point along tank +y"

    r_ned = F.rot_flu_enu_to_frd_ned(r_enu)
    assert np.allclose(F.rpy_deg_from_rot(r_ned), [0.0, 0.0, 0.0], atol=1e-9), "that is heading 0 in NED"
    fwd_ned = r_ned @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(fwd_ned, [1.0, 0.0, 0.0]), "the forward axis must be +North"
    assert np.allclose(F.vec_enu_ned(fwd_enu), fwd_ned), "and the two must be the same physical vector"


def test_yaw_sign_is_opposite_between_the_conventions():
    """ENU yaw turns East->North, NED yaw turns North->East: yaw_NED = 90 - yaw_ENU, so the senses oppose."""
    for yaw in (0.0, 30.0, 90.0, 180.0, -45.0):
        ned = F.rpy_deg_from_rot(F.rot_flu_enu_to_frd_ned(F.rot_from_rpy_deg([0.0, 0.0, yaw])))
        expect = (90.0 - yaw + 180.0) % 360.0 - 180.0
        assert np.isclose(ned[2], expect, atol=1e-9), f"yaw_ENU {yaw} -> yaw_NED {ned[2]}, expected {expect}"


@pytest.mark.parametrize("rpy_enu", [(0.0, 15.0, 0.0), (10.0, -20.0, 30.0), (-35.0, 5.0, 140.0),
                                     (60.0, -40.0, -100.0)])
def test_euler_relations_between_the_conventions(rpy_enu):
    """roll_NED = roll_ENU, pitch_NED = -pitch_ENU, yaw_NED = 90 deg - yaw_ENU.

    The pitch sign is the one that bites: the same ZYX formula is "pitch positive = nose down" in FLU/ENU (the
    repo's convention) and "nose up" in FRD/NED (ArduPilot's), purely because z flips.
    """
    rpy_ned = F.rpy_deg_from_rot(F.rot_flu_enu_to_frd_ned(F.rot_from_rpy_deg(rpy_enu)))
    assert np.isclose(rpy_ned[0], rpy_enu[0], atol=1e-9)
    assert np.isclose(rpy_ned[1], -rpy_enu[1], atol=1e-9)
    expect_yaw = (90.0 - rpy_enu[2] + 180.0) % 360.0 - 180.0
    assert np.isclose(rpy_ned[2], expect_yaw, atol=1e-9)


def test_the_configured_sonar_pitch_is_minus_15_for_ardupilot():
    """config/tank.yaml says sonar.pitch_deg: 15.0, "positive = nose down". ArduPilot must report -15."""
    rpy_ned = F.rpy_deg_from_rot(F.rot_flu_enu_to_frd_ned(F.rot_from_rpy_deg([0.0, 15.0, 0.0])))
    assert np.isclose(rpy_ned[1], -15.0, atol=1e-9)


# -------------------------------------------------------------------- agreement with the repo convention

def test_rotation_matches_the_repo_convention():
    """frames.rot_from_rpy_deg duplicates uw.geometry.rot_rpy_deg so that uw.rov imports nothing from the
    repo; this test is what keeps the duplicate honest."""
    from uw.geometry import rot_rpy_deg
    for _ in range(20):
        rpy = RNG.uniform(-180.0, 180.0, 3)
        assert np.allclose(F.rot_from_rpy_deg(rpy), rot_rpy_deg(rpy))


def test_euler_round_trip():
    for _ in range(50):
        rpy = np.array([RNG.uniform(-179, 179), RNG.uniform(-89, 89), RNG.uniform(-179, 179)])
        assert np.allclose(F.rpy_deg_from_rot(F.rot_from_rpy_deg(rpy)), rpy, atol=1e-9)


# ------------------------------------------------------------------------------- vectors and positions

def test_vector_round_trips():
    for _ in range(20):
        v = RNG.normal(size=3)
        assert np.allclose(F.vec_enu_ned(F.vec_enu_ned(v)), v)
        assert np.allclose(F.vec_flu_frd(F.vec_flu_frd(v)), v)
        assert np.allclose(F.angvel_flu_frd(F.angvel_flu_frd(v)), v)


def test_vector_conversions_preserve_length_and_angles():
    for _ in range(20):
        a, b = RNG.normal(size=3), RNG.normal(size=3)
        assert np.isclose(np.linalg.norm(F.vec_enu_ned(a)), np.linalg.norm(a))
        assert np.isclose(F.vec_enu_ned(a) @ F.vec_enu_ned(b), a @ b)
        assert np.allclose(F.cross3(F.vec_flu_frd(a), F.vec_flu_frd(b)), F.vec_flu_frd(F.cross3(a, b))), \
            "a proper rotation commutes with the cross product; a reflection would not"


def test_position_datum_is_the_free_surface():
    """N = y, E = x, D = water_level - z (docs/ros_hil.md 6.2): at the surface D = 0, at the tank floor
    D = water_level."""
    wl = 2.9
    assert np.allclose(F.position_tank_to_ned([0.0, 0.0, wl], wl), [0.0, 0.0, 0.0])
    assert np.allclose(F.position_tank_to_ned([0.0, 0.0, 0.0], wl), [0.0, 0.0, wl])
    assert np.allclose(F.position_tank_to_ned([1.0, 2.0, 0.9], wl), [2.0, 1.0, 2.0])
    assert np.allclose(F.position_tank_to_ned([0.0, 0.0, wl + 0.1], wl), [0.0, 0.0, -0.1]), \
        "above the surface D is negative"


def test_position_round_trip():
    for _ in range(20):
        p = RNG.uniform(0.0, 3.0, 3)
        assert np.allclose(F.position_ned_to_tank(F.position_tank_to_ned(p, 2.9), 2.9), p)


# -------------------------------------------------------------------------------------- quaternions

def test_quaternion_rotation_round_trip():
    for q in random_quats(50):
        assert np.allclose(F.rot_to_quat(F.quat_to_rot(q)), q, atol=1e-9)


def test_quaternion_round_trip_near_180_degrees():
    """The branchy part of rot_to_quat: a trace near -1 must not divide by something near zero."""
    for axis in np.eye(3):
        for angle in (179.0, 180.0, 181.0):
            a = np.radians(angle)
            q = F.quat_normalize(np.concatenate([[np.cos(a / 2)], np.sin(a / 2) * axis]))
            back = F.rot_to_quat(F.quat_to_rot(q))
            assert np.allclose(F.quat_to_rot(back), F.quat_to_rot(q), atol=1e-9)


def test_quaternion_rotates_like_its_matrix():
    for q in random_quats(20):
        v = RNG.normal(size=3)
        assert np.allclose(F.quat_rotate(q, v), F.quat_to_rot(q) @ v)
        assert np.allclose(F.quat_rotate_inv(q, F.quat_rotate(q, v)), v)


def test_quaternion_product_composes_rotations():
    for a, b in zip(random_quats(10), random_quats(10)):
        assert np.allclose(F.quat_to_rot(F.quat_mul(a, b)), F.quat_to_rot(a) @ F.quat_to_rot(b))


def test_quaternion_conversion_between_conventions_round_trips():
    for q in random_quats(30):
        assert np.allclose(F.quat_frd_ned_to_flu_enu(F.quat_flu_enu_to_frd_ned(q)), q, atol=1e-9)


def test_quaternion_conversion_agrees_with_the_matrix_sandwich():
    for q in random_quats(20):
        by_quat = F.quat_to_rot(F.quat_flu_enu_to_frd_ned(q))
        by_matrix = F.rot_flu_enu_to_frd_ned(F.quat_to_rot(q))
        assert np.allclose(by_quat, by_matrix, atol=1e-12)


def test_ros_quaternion_order():
    """ROS is (x, y, z, w) and everything else here is (w, x, y, z): the two helpers are the only crossing."""
    q = F.quat_from_rpy_deg([10.0, 20.0, 30.0])
    xyzw = F.quat_to_ros_xyzw(q)
    assert np.allclose(xyzw, [q[1], q[2], q[3], q[0]])
    assert np.allclose(F.quat_from_ros_xyzw(xyzw), q)


def test_quat_derivative_integrates_to_the_expected_rotation():
    """A constant body rate about one axis for one second must give exactly that angle."""
    omega = np.array([0.0, 0.0, 0.5])            # 0.5 rad/s about body z
    q = F.quat_identity()
    dt = 1e-4
    for _ in range(10_000):
        q = F.quat_normalize(q + dt * F.quat_derivative(q, omega))
    assert np.allclose(F.rpy_deg_from_quat(q), [0.0, 0.0, np.degrees(0.5)], atol=1e-4)


def test_angular_velocity_conversion_is_consistent_with_the_attitude_conversion():
    """Integrate an attitude in FLU/ENU with omega_FLU, and the same motion in FRD/NED with omega_FRD; the
    two must describe the same physical attitude at every step. This is what makes angvel_flu_frd correct
    rather than merely plausible."""
    w_flu = np.array([0.3, -0.2, 0.45])
    w_frd = F.angvel_flu_frd(w_flu)
    q_enu = F.quat_from_rpy_deg([5.0, -10.0, 25.0])
    q_ned = F.quat_flu_enu_to_frd_ned(q_enu)
    dt = 1e-4
    for _ in range(5_000):
        q_enu = F.quat_normalize(q_enu + dt * F.quat_derivative(q_enu, w_flu))
        q_ned = F.quat_normalize(q_ned + dt * F.quat_derivative(q_ned, w_frd))
    assert np.allclose(F.quat_to_rot(F.quat_flu_enu_to_frd_ned(q_enu)), F.quat_to_rot(q_ned), atol=1e-7)


# -------------------------------------------------------------------------------- poses and twists

def test_full_pose_round_trip():
    for _ in range(20):
        p = RNG.uniform(0.0, 3.0, 3)
        q = F.quat_normalize(RNG.normal(size=4))
        p2, q2 = F.pose_ned_to_tank(*F.pose_tank_to_ned(p, q, 2.9), 2.9)
        assert np.allclose(p2, p)
        assert np.allclose(F.quat_to_rot(q2), F.quat_to_rot(q), atol=1e-9)


def test_twist_conversion():
    v, w = RNG.normal(size=3), RNG.normal(size=3)
    v2, w2 = F.twist_ned_to_tank(v, w)
    assert np.allclose(v2, F.vec_flu_frd(v))
    assert np.allclose(w2, F.angvel_flu_frd(w))
    assert np.allclose(F.twist_ned_to_tank(v2, w2)[0], v)


def test_a_concrete_pose_end_to_end():
    """A vehicle at tank (1.0, 2.0, 0.9) m heading along tank +y with 15 deg nose-down, moving forward at
    0.3 m/s: what ArduPilot must see."""
    wl = 2.9
    p_ned, q_ned = F.pose_tank_to_ned([1.0, 2.0, 0.9], F.quat_from_rpy_deg([0.0, 15.0, 90.0]), wl)
    assert np.allclose(p_ned, [2.0, 1.0, 2.0]), "N = y, E = x, D = 2.9 - z"
    assert np.allclose(F.rpy_deg_from_quat(q_ned), [0.0, -15.0, 0.0], atol=1e-9), "heading 0, nose 15 down"
    v_ned = F.quat_to_rot(q_ned) @ np.array([0.3, 0.0, 0.0])
    assert v_ned[0] > 0.28 and abs(v_ned[1]) < 1e-9, "moving north"
    assert v_ned[2] > 0.0, "and downward, because it is nose-down"


# ------------------------------------------------------------------------------------- the Isaac seam

def test_isaac_seam_is_the_identity():
    """uw/usd_scene.py sets the stage up axis to Z and metersPerUnit to 1.0 and authors the tank in metres, so
    tank -> Isaac is the identity. If a future stage changes either, these two functions are the only things
    that change - this test documents the assumption rather than checking arithmetic."""
    p = np.array([1.0, 2.0, 0.9])
    q = F.quat_from_rpy_deg([3.0, 15.0, 42.0])
    assert np.allclose(F.isaac_position_from_tank(p), p)
    assert np.allclose(F.isaac_quat_from_tank(q), q)
    assert F.isaac_quat_from_tank(q).shape == (4,), "USD Gf.Quatd is scalar-first, like ours"


def test_isaac_helpers_do_not_alias_their_input():
    p = np.array([1.0, 2.0, 0.9])
    out = F.isaac_position_from_tank(p)
    out[0] = 99.0
    assert p[0] == 1.0
