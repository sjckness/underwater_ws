"""The vehicle files against ArduSub's own frame definition. Host test (numpy only).

ArduSub mixes a pilot demand (roll, pitch, yaw, climb, forward, lateral) into motor outputs with the factor table of
``AP_Motors6DOF::setup_motors``; SITL's ``SIM_Submarine`` turns a positive output of motor n into a force along
(forward, lateral, -climb) and a moment with the signs of (roll, pitch, yaw). Our physics must agree with both, or
the autopilot flies a different vehicle from the one we simulate: before M11 the two front horizontals and every
vertical were reversed, so a climb demand drove the vehicle down.

The tables below are copied from ArduPilot tag ArduSub-4.5.7 (commit 30257f01185471ab4c1ac544e47d1b4437e44c98),
libraries/AP_Motors/AP_Motors6DOF.cpp, SUB_FRAME_VECTORED and SUB_FRAME_VECTORED_6DOF; SIM_Submarine.cpp at the
same tag has the identical rows. Motor n is SERVOn (the default SERVOn_FUNCTION), i.e. JSON pwm[n - 1].
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from uw.rov import dynamics as D

CONFIG = Path(__file__).resolve().parents[2] / "config"

#            roll  pitch  yaw  climb  forward  lateral
VECTORED_6DOF = np.array([
    [0, 0, 1, 0, -1, 1],
    [0, 0, -1, 0, -1, -1],
    [0, 0, -1, 0, 1, 1],
    [0, 0, 1, 0, 1, -1],
    [1, -1, 0, -1, 0, 0],
    [-1, -1, 0, -1, 0, 0],
    [1, 1, 0, -1, 0, 0],
    [-1, 1, 0, -1, 0, 0],
], float)
VECTORED = np.vstack([VECTORED_6DOF[:4], [[1, 0, 0, -1, 0, 0], [-1, 0, 0, -1, 0, 0]]])

FILES = {
    "rov_bluerov2.yaml": ("vectored_6dof", VECTORED_6DOF),
    "rov_bluerov2_vectored.yaml": ("vectored", VECTORED),
}
DEMANDS = ("roll", "pitch", "yaw", "climb", "forward", "lateral")


def wrench_signs_of(factors: np.ndarray) -> np.ndarray:
    """The wrench (Fx, Fy, Fz, Mx, My, Mz) signs, body FRD, that SITL gives a positive output of each motor."""
    roll, pitch, yaw, climb, fwd, lat = factors.T
    return np.sign(np.stack([fwd, lat, -climb, roll, pitch, yaw], axis=1))


def mixer_pwm(factors: np.ndarray, demand: str, level: float = 0.5) -> np.ndarray:
    """ArduSub's mixer for one pure demand, without its normalisation: out = factor * level, PWM 1500 +- 400."""
    out = factors[:, DEMANDS.index(demand)] * level
    return 1500.0 + 400.0 * np.clip(out, -1.0, 1.0)


@pytest.fixture(params=sorted(FILES), scope="module")
def vehicle_file(request):
    frame, table = FILES[request.param]
    return D.load_params(CONFIG / request.param), frame, table


def test_channels_are_ardusub_motor_order(vehicle_file):
    params, frame, table = vehicle_file
    assert params.frame == frame
    assert [t.channel for t in params.thrusters] == list(range(len(table)))


def test_each_thruster_pushes_the_way_sitl_says(vehicle_file):
    """Column i of the allocation matrix has exactly the signs SIM_Submarine gives motor i + 1."""
    params, _, table = vehicle_file
    col_signs = np.sign(np.where(np.abs(params.allocation) < 1e-9, 0.0, params.allocation)).T
    expect = wrench_signs_of(table)
    for i, t in enumerate(params.thrusters):
        assert np.array_equal(col_signs[i], expect[i]), \
            f"{t.name} (channel {t.channel}): wrench signs {col_signs[i]} vs ArduSub {expect[i]}"


@pytest.mark.parametrize("demand,dof,sign", [("forward", 0, 1), ("lateral", 1, 1), ("climb", 2, -1),
                                              ("roll", 3, 1), ("pitch", 4, 1), ("yaw", 5, 1)])
def test_a_pilot_demand_moves_the_vehicle_that_way(vehicle_file, demand, dof, sign):
    """End to end through the PWM: mixer -> PWM -> T200 command -> thrust -> wrench. Climb is -z in FRD.
    The standard vehicle has no pitch column, so its pitch demand must give no wrench at all."""
    params, frame, table = vehicle_file
    pwm = mixer_pwm(table, demand)
    thrust = params.t200.thrust_of_command(params.t200.command_of_pwm(pwm))
    tau = params.allocation @ thrust
    if demand == "pitch" and frame == "vectored":
        assert np.allclose(tau, 0.0), tau
        return
    assert sign * tau[dof] > 1.0, f"{demand}: wrench {np.round(tau, 3)}"
