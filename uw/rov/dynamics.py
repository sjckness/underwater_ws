"""6-DOF Fossen dynamics of the BlueROV2 Heavy, in pure numpy.

No Isaac, no ROS, no pxr: numpy, pyyaml and the standard library only, so this module runs anywhere the branch
is checked out. Paths go through ``pathlib``.

**Frame.** Everything here is ArduPilot's: earth **NED**, body **FRD**. That is the convention the published
Fossen identifications use and the one the JSON link wants, so the dynamics needs no conversion at all and
``uw.rov.frames`` is called only at the seams (the soft tank bound, which is stated in tank coordinates, and
whatever the ROS/Isaac side asks for).

**The model** (Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2011):

    M nu_dot + C(nu) nu + D(nu) nu + g(eta) = tau

with ``nu = [v; omega]`` the body-frame twist, and

* ``M = M_RB + M_A``, both constant, so ``M^-1`` is computed once. The body origin is the centre of gravity
  (``body.r_g_frd_m`` must be zero - a non-zero value raises rather than being silently ignored), which makes
  ``M_RB = blkdiag(m I, I_b)``.
* ``C(nu) = C_RB(nu) + C_A(nu)``, both in Fossen's skew-symmetric form, so ``nu' C(nu) nu == 0`` exactly - a
  test asserts it, because that identity is what makes the energy test below meaningful.
* ``D(nu) = D_lin + D_quad |nu|``, diagonal, from the published coefficients.
* ``g(eta)`` the restoring wrench of the weight at the CG and the buoyancy at the centre of buoyancy, computed
  vectorially from the attitude (no small-angle or Euler-angle form anywhere).
* ``tau = T f``, with the 6x8 allocation ``T`` built from the thruster geometry: column i is
  ``[axis_i ; r_i x axis_i]``. It is never written out by hand.

**Energy.** With ``C`` skew-symmetric, ``d/dt (0.5 nu' M nu) = nu' (tau - D nu - g)``, so the total

    E = 0.5 nu' M nu + (B - W) p_D + B (R r_b)_D

is non-increasing for ``tau = 0`` and exactly conserved if the damping is zeroed. ``energy()`` returns it and
step 1 test (f) checks both statements over 10 simulated minutes.

**Integration.** Fixed step, RK4 by default (``integrator.method``), 400 Hz (``integrator.rate_hz``) to match
ArduSub's ``MAIN_LOOP_RATE`` so that one physics step is one JSON frame. The quaternion is carried in the state
vector and renormalised after each completed step, not inside the stages. Nothing here reads a clock: time
advances only by the ``dt`` it is given, which is what ``docs/ros_hil.md`` 3.4 requires.

**The soft tank bound** is not physics. There is no PhysX scene (``docs/ros_hil.md`` 6.5), so a thrusting
vehicle would leave the water or pass through the pipe. ``bound.*`` adds a spring-damper per tank face plus a
stiffer free surface, applied as a pure force at the body origin with no moment. Every activation is counted
and logged. Switch it off with ``bound.enabled: false`` and the vehicle is pure Fossen again.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import yaml

from . import frames

log = logging.getLogger(__name__)

G = 9.80665
DEFAULT_PARAMS = Path(__file__).resolve().parents[2] / "config" / "rov_bluerov2.yaml"


class VehicleError(RuntimeError):
    """A vehicle parameter file is missing, malformed or internally inconsistent."""


# --------------------------------------------------------------------------------------------- state

@dataclass(frozen=True)
class State:
    """The vehicle state. ``t`` is simulated seconds; nothing in this module reads a wall clock."""
    t: float
    position_ned: np.ndarray        # (3,) [m], NED relative to the origin on the free surface
    quat_frd_ned: np.ndarray        # (4,) scalar-first (w, x, y, z), body FRD -> earth NED
    velocity_frd: np.ndarray        # (3,) [m/s] body-frame linear velocity (u, v, w)
    rate_frd: np.ndarray            # (3,) [rad/s] body-frame angular velocity (p, q, r)
    thrust_n: np.ndarray            # (n,) [N] thrust actually delivered, after the first-order lag

    @staticmethod
    def rest(n_thrusters: int = 8, position_ned=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0),
             t: float = 0.0) -> "State":
        return State(float(t), np.asarray(position_ned, float).copy(), frames.quat_normalize(quat),
                     np.zeros(3), np.zeros(3), np.zeros(int(n_thrusters)))

    @property
    def nu(self) -> np.ndarray:
        """The 6-vector body twist ``[v; omega]``."""
        return np.concatenate([self.velocity_frd, self.rate_frd])

    @property
    def rot_frd_ned(self) -> np.ndarray:
        return frames.quat_to_rot(self.quat_frd_ned)

    def pack(self) -> np.ndarray:
        return np.concatenate([self.position_ned, self.quat_frd_ned, self.velocity_frd, self.rate_frd,
                               self.thrust_n])

    @staticmethod
    def unpack(y: np.ndarray, t: float) -> "State":
        return State(float(t), y[0:3].copy(), y[3:7].copy(), y[7:10].copy(), y[10:13].copy(), y[13:].copy())


# --------------------------------------------------------------------------------------- parameters

@dataclass(frozen=True)
class Thruster:
    name: str
    channel: int
    position_frd_m: np.ndarray
    axis_frd: np.ndarray


@dataclass(frozen=True)
class T200:
    pwm_min_us: float
    pwm_mid_us: float
    pwm_max_us: float
    deadband_us: tuple[float, float]
    voltage_v: float
    thrust_fwd_max_n: float
    thrust_rev_max_n: float
    curve_exponent: float
    tau_s: float
    table: np.ndarray | None = None

    def thrust_of_command(self, u) -> np.ndarray:
        """Steady thrust [N] of a normalised command in ``[-1, 1]``.

        The table wins when it is given (fill ``t200.table`` with the published CSV); otherwise the power law
        ``max * |u|^exponent``, with a different maximum forward and reverse - the asymmetry asked for in the
        design. Commands are clipped, never extrapolated.
        """
        u = np.clip(np.asarray(u, float), -1.0, 1.0)
        if self.table is not None:
            return np.interp(u, self.table[:, 0], self.table[:, 1])
        mag = np.abs(u) ** float(self.curve_exponent)
        return np.where(u >= 0.0, mag * self.thrust_fwd_max_n, -mag * self.thrust_rev_max_n)

    def command_of_pwm(self, pwm) -> np.ndarray:
        """PWM [us] -> normalised command in ``[-1, 1]``, with the dead band mapped to exactly zero.

        Below/above the dead band the command is linear in PWM from 0 at the dead-band edge to -+1 at
        ``pwm_min_us`` / ``pwm_max_us``. Out-of-range PWM is clipped, so a disarmed or failsafe value cannot
        produce more than full thrust.
        """
        p = np.clip(np.asarray(pwm, float), self.pwm_min_us, self.pwm_max_us)
        lo, hi = self.deadband_us
        out = np.zeros_like(p)
        below = p < lo
        above = p > hi
        out[below] = -(lo - p[below]) / max(lo - self.pwm_min_us, 1e-9)
        out[above] = (p[above] - hi) / max(self.pwm_max_us - hi, 1e-9)
        return np.clip(out, -1.0, 1.0)


@dataclass(frozen=True)
class TankBounds:
    """The soft bound's axis-aligned box, in **tank/ENU** metres, plus the free surface height."""
    lo: np.ndarray
    hi: np.ndarray
    surface_z: float
    stiffness: float
    damping: float
    surface_stiffness: float
    surface_damping: float

    @staticmethod
    def from_tank(size, sand_base: float, water_level: float, margin: float, sand_extra: float,
                  stiffness: float, damping: float, surface_stiffness: float,
                  surface_damping: float) -> "TankBounds":
        """The same box ``uw.pose.Bounds`` clips the kinematic trajectories to: ``margin`` from the walls,
        ``margin + sand_extra`` over the sand, ``margin`` below the surface."""
        size = np.asarray(size, float)
        lo = np.array([margin, margin, float(sand_base) + margin + sand_extra])
        hi = np.array([size[0] - margin, size[1] - margin, float(water_level) - margin])
        return TankBounds(lo, hi, float(water_level), stiffness, damping, surface_stiffness, surface_damping)


@dataclass(frozen=True)
class Params:
    model: str
    frame: str
    mass_kg: float
    weight_n: float
    buoyancy_n: float
    r_g_frd_m: np.ndarray
    r_b_frd_m: np.ndarray
    inertia: np.ndarray                 # (3, 3) about the CG, body FRD
    M: np.ndarray                       # (6, 6) = M_RB + M_A
    M_inv: np.ndarray
    added_mass: np.ndarray              # (6,) positive diagonal of M_A
    d_lin: np.ndarray                   # (6,) positive
    d_quad: np.ndarray                  # (6,) positive
    thrusters: tuple[Thruster, ...]
    allocation: np.ndarray              # (6, n)
    t200: T200
    rate_hz: float
    method: str
    bound_enabled: bool
    bound_margin_m: float
    bound_sand_extra_m: float
    bound_gains: tuple[float, float, float, float]

    @property
    def dt(self) -> float:
        return 1.0 / float(self.rate_hz)

    @property
    def n_thrusters(self) -> int:
        return len(self.thrusters)

    @property
    def net_buoyancy_n(self) -> float:
        """Positive = upward. The BlueROV2 is designed slightly positive, so this is expected to be > 0."""
        return float(self.buoyancy_n - self.weight_n)


def _vec(d: dict, key: str, n: int = 3) -> np.ndarray:
    v = np.asarray(d[key], float)
    if v.shape != (n,):
        raise VehicleError(f"{key}: expected {n} numbers, got {v.shape}")
    return v


def load_params(path: str | Path | None = None) -> Params:
    """Read ``config/rov_bluerov2.yaml`` (or ``path``) into a ``Params``, checking it for consistency.

    Checks that raise rather than producing a quietly wrong vehicle: the body origin must be the CG, the
    thrust axes must be unit, ``M`` must be positive definite, the damping coefficients must be
    non-positive as Fossen's notation requires, and ``weight_n`` must agree with ``mass_kg * g`` to 1 %.
    """
    path = Path(path) if path is not None else DEFAULT_PARAMS
    if not path.is_file():
        raise VehicleError(f"vehicle parameters not found: {path}")
    cfg = yaml.safe_load(path.read_text())
    if not isinstance(cfg, dict):
        raise VehicleError(f"{path}: expected a mapping at top level")

    body = cfg["body"]
    mass = float(body["mass_kg"])
    weight = float(body["weight_n"])
    if abs(weight - mass * G) > 0.01 * weight:
        raise VehicleError(f"body.weight_n {weight} disagrees with mass_kg * g = {mass * G:.3f} by more than 1 %")
    r_g = _vec(body, "r_g_frd_m")
    if np.any(np.abs(r_g) > 1e-12):
        raise VehicleError("body.r_g_frd_m must be zero: this module puts the body origin at the centre of "
                           "gravity, which is what makes M_RB block-diagonal. A non-zero CG offset needs the "
                           "full M_RB and C_RB of Fossen (3.56); it is not silently approximated here.")
    ixx, iyy, izz = _vec(body, "inertia_frd_kg_m2")
    ixy, ixz, iyz = _vec(body, "inertia_products_kg_m2")
    inertia = np.array([[ixx, -ixy, -ixz], [-ixy, iyy, -iyz], [-ixz, -iyz, izz]])

    am = cfg["added_mass"]
    added = -np.array([am["X_udot"], am["Y_vdot"], am["Z_wdot"], am["K_pdot"], am["M_qdot"], am["N_rdot"]],
                      float)
    if np.any(added < 0.0):
        raise VehicleError("added_mass coefficients are Fossen's and must be <= 0 (M_A = -diag(...))")
    dl = cfg["damping_linear"]
    d_lin = -np.array([dl["X_u"], dl["Y_v"], dl["Z_w"], dl["K_p"], dl["M_q"], dl["N_r"]], float)
    dq = cfg["damping_quadratic"]
    d_quad = -np.array([dq["X_uu"], dq["Y_vv"], dq["Z_ww"], dq["K_pp"], dq["M_qq"], dq["N_rr"]], float)
    for name, arr in (("damping_linear", d_lin), ("damping_quadratic", d_quad)):
        if np.any(arr < 0.0):
            raise VehicleError(f"{name} coefficients are Fossen's and must be <= 0 (D = -diag(...))")
    d_lin += 0.0                       # a published 0.0 negates to -0.0, which prints as "-0.000"
    d_quad += 0.0
    d_lin[d_lin == 0.0] = 0.0
    d_quad[d_quad == 0.0] = 0.0

    m_rb = np.zeros((6, 6))
    m_rb[:3, :3] = mass * np.eye(3)
    m_rb[3:, 3:] = inertia
    M = m_rb + np.diag(added)
    if not np.all(np.linalg.eigvalsh(0.5 * (M + M.T)) > 0.0):
        raise VehicleError("M = M_RB + M_A is not positive definite: check the inertia and added-mass rows")

    tl = cfg["thrusters"]["list"]
    if int(cfg["thrusters"]["count"]) != len(tl):
        raise VehicleError(f"thrusters.count {cfg['thrusters']['count']} != {len(tl)} entries in thrusters.list")
    thrusters = []
    for i, t in enumerate(tl):
        axis = _vec(t, "axis_frd")
        n = float(np.linalg.norm(axis))
        if abs(n - 1.0) > 1e-4:
            raise VehicleError(f"thrusters.list[{i}] ({t['name']}): axis_frd has norm {n:.6f}, must be a unit "
                               "vector (to 1e-4; it is then normalised exactly)")
        thrusters.append(Thruster(str(t["name"]), int(t["channel"]), _vec(t, "position_frd_m"), axis / n))
    thrusters = tuple(thrusters)
    allocation = allocation_matrix(thrusters)

    tc = cfg["t200"]
    table = np.asarray(tc.get("table") or [], float)
    if table.size:
        if table.ndim != 2 or table.shape[1] != 2:
            raise VehicleError("t200.table must be a list of [command, newtons] pairs")
        if np.any(np.diff(table[:, 0]) <= 0.0):
            raise VehicleError("t200.table commands must be strictly increasing")
    t200 = T200(float(tc["pwm_min_us"]), float(tc["pwm_mid_us"]), float(tc["pwm_max_us"]),
                (float(tc["deadband_us"][0]), float(tc["deadband_us"][1])), float(tc["voltage_v"]),
                float(tc["thrust_fwd_max_n"]), float(tc["thrust_rev_max_n"]),
                float(tc["curve_exponent"]), float(tc["tau_s"]),
                table if table.size else None)
    if t200.tau_s <= 0.0:
        raise VehicleError("t200.tau_s must be > 0")

    it = cfg["integrator"]
    method = str(it["method"])
    if method not in ("rk4", "semi_implicit"):
        raise VehicleError(f"integrator.method must be rk4 | semi_implicit, got {method!r}")
    bd = cfg["bound"]
    return Params(
        model=str(cfg["model"]), frame=str(cfg["frame"]), mass_kg=mass, weight_n=weight,
        buoyancy_n=float(body["buoyancy_n"]), r_g_frd_m=r_g, r_b_frd_m=_vec(body, "r_b_frd_m"),
        inertia=inertia, M=M, M_inv=np.linalg.inv(M), added_mass=added, d_lin=d_lin, d_quad=d_quad,
        thrusters=thrusters, allocation=allocation, t200=t200,
        rate_hz=float(it["rate_hz"]), method=method,
        bound_enabled=bool(bd["enabled"]), bound_margin_m=float(bd["margin_m"]),
        bound_sand_extra_m=float(bd["sand_extra_m"]),
        bound_gains=(float(bd["stiffness_n_per_m"]), float(bd["damping_n_s_per_m"]),
                     float(bd["surface_stiffness_n_per_m"]), float(bd["surface_damping_n_s_per_m"])),
    )


def allocation_matrix(thrusters) -> np.ndarray:
    """The 6xn thrust allocation ``T``: column i is ``[axis_i ; r_i x axis_i]``, so ``tau = T f``.

    Built from geometry alone, so the moment arms cannot disagree with the mounting points.
    """
    cols = []
    for t in thrusters:
        cols.append(np.concatenate([t.axis_frd, np.cross(t.position_frd_m, t.axis_frd)]))
    return np.column_stack(cols)


# ------------------------------------------------------------------------------------------- the model

def coriolis(params: Params, nu: np.ndarray) -> np.ndarray:
    """``C(nu) = C_RB(nu) + C_A(nu)`` in Fossen's skew-symmetric form (body origin at the CG).

    ``C_RB = [[0, -m S(v)], [-m S(v), -S(I w)]]`` and, for a diagonal added mass,
    ``C_A = [[0, -S(A1 v)], [-S(A1 v), -S(A2 w)]]``. Skew-symmetric by construction, hence ``nu' C nu == 0``.
    """
    v, w = nu[:3], nu[3:]
    a1, a2 = params.added_mass[:3], params.added_mass[3:]
    c = np.zeros((6, 6))
    top = -params.mass_kg * frames.skew(v) - frames.skew(a1 * v)
    c[:3, 3:] = top
    c[3:, :3] = top
    c[3:, 3:] = -frames.skew(params.inertia @ w) - frames.skew(a2 * w)
    return c


def coriolis_nu(params: Params, nu: np.ndarray) -> np.ndarray:
    """``C(nu) nu`` as four cross products instead of a 6x6 matrix product.

    Multiplying out Fossen's skew-symmetric ``C``:

        top    = m (w x v) + w x (A1 v)
        bottom = v x (A1 v) + w x (I w) + w x (A2 w)

    This is what the integrator calls; ``coriolis`` builds the matrix itself, for the skew-symmetry test and
    for anyone who wants to look at it.
    """
    v, w = nu[:3], nu[3:]
    a1v = params.added_mass[:3] * v
    a2w = params.added_mass[3:] * w
    top = params.mass_kg * frames.cross3(w, v) + frames.cross3(w, a1v)
    bot = frames.cross3(v, a1v) + frames.cross3(w, params.inertia @ w) + frames.cross3(w, a2w)
    out = np.empty(6)
    out[:3] = top
    out[3:] = bot
    return out


def damping(params: Params, nu: np.ndarray) -> np.ndarray:
    """``D(nu) nu`` with the linear and the quadratic term, both diagonal and both dissipative."""
    return (params.d_lin + params.d_quad * np.abs(nu)) * nu


def restoring(params: Params, rot_frd_ned: np.ndarray) -> np.ndarray:
    """Fossen's ``g(eta)``: the wrench the weight and the buoyancy put on the LHS of the equation.

    Vectorial, no Euler angles. ``e_D`` is the NED down axis expressed in the body frame; the weight acts at
    the CG (the body origin) and the buoyancy at ``r_b``, so with the CB above the CG the moment is a righting
    one - ``tests/rov/test_dynamics.py`` checks the sign for roll and for pitch.
    """
    e_d_body = rot_frd_ned.T @ np.array([0.0, 0.0, 1.0])
    f_w = params.weight_n * e_d_body
    f_b = -params.buoyancy_n * e_d_body
    force = -(f_w + f_b)
    moment = -(frames.cross3(params.r_g_frd_m, f_w) + frames.cross3(params.r_b_frd_m, f_b))
    return np.concatenate([force, moment])


# ----------------------------------------------------------------------------------------- the vehicle

class Vehicle:
    """The integrator. Stateless with respect to the vehicle state: ``step`` takes one and returns a new one.

    ``bounds`` is the soft tank bound (``None``, or when ``bound.enabled`` is false, means pure Fossen).
    ``water_level`` is only needed to place the NED origin on the free surface, so that ``position_ned[2]`` is
    the depth ArduSub's simulated barometer reads.
    """

    def __init__(self, params: Params, water_level: float = 2.9, bounds: TankBounds | None = None):
        self.params = params
        self.water_level = float(water_level)
        self.bounds = bounds if params.bound_enabled else None
        self.bound_activations = 0          # derivative evaluations in which the bound pushed in. RK4 calls
                                            #   the derivative 4x per step, so this is ~4x the step count.
        self.bound_max_penetration_m = 0.0
        self._last_bound_log_t = -1e18

    # ---- the public API asked for in the design: step(state, thruster_cmd, dt) -> state

    def step(self, state: State, thruster_cmd, dt: float | None = None) -> State:
        """Advance one fixed step. ``thruster_cmd`` is the normalised command per thruster in ``[-1, 1]``.

        ``dt`` defaults to ``1 / integrator.rate_hz``. Simulated time only: no clock is read and no sleep
        happens, which is what ``docs/ros_hil.md`` 3.4 requires of the whole ROV path.
        """
        dt = self.params.dt if dt is None else float(dt)
        if not dt > 0.0:
            raise VehicleError(f"dt must be > 0, got {dt}")
        u = np.clip(np.asarray(thruster_cmd, float), -1.0, 1.0)
        if u.shape != (self.params.n_thrusters,):
            raise VehicleError(f"thruster_cmd must have {self.params.n_thrusters} entries, got {u.shape}")

        y0 = state.pack()
        steady = self.params.t200.thrust_of_command(u)      # constant over the step: hoisted out of RK4
        if self.params.method == "rk4":
            k1 = self._deriv(y0, steady)
            k2 = self._deriv(y0 + 0.5 * dt * k1, steady)
            k3 = self._deriv(y0 + 0.5 * dt * k2, steady)
            k4 = self._deriv(y0 + dt * k3, steady)
            y = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        else:
            y = y0 + dt * self._deriv(y0, steady)
        y[3:7] = frames.quat_normalize(y[3:7])      # once per completed step, not inside the stages
        return State.unpack(y, state.t + dt)

    def derivative(self, state: State, thruster_cmd) -> np.ndarray:
        """The packed state derivative: exposed so tests can inspect accelerations directly."""
        u = np.clip(np.asarray(thruster_cmd, float), -1.0, 1.0)
        return self._deriv(state.pack(), self.params.t200.thrust_of_command(u))

    def acceleration(self, state: State, thruster_cmd) -> np.ndarray:
        """``nu_dot`` (6,): body linear and angular acceleration [m/s^2, rad/s^2]."""
        return self.derivative(state, thruster_cmd)[7:13]

    def energy(self, state: State) -> float:
        """``0.5 nu' M nu + (B - W) p_D + B (R r_b)_D`` [J]: non-increasing with zero thrust, and conserved
        when the damping is zeroed. The datum is the free surface, so the potential term is a depth term."""
        p = self.params
        nu = state.nu
        ke = 0.5 * float(nu @ p.M @ nu)
        pe_net = (p.buoyancy_n - p.weight_n) * float(state.position_ned[2])
        pe_couple = p.buoyancy_n * float((state.rot_frd_ned @ p.r_b_frd_m)[2])
        return ke + pe_net + pe_couple

    def thrust_of_pwm(self, pwm) -> np.ndarray:
        """Steady thrust [N] per thruster for a PWM array ordered as ``params.thrusters``.

        The map from an ArduSub servo channel to a thruster is NOT applied here: that mapping comes from the
        ArduSub frame definition and is M8a step 2's job (see ``config/rov_bluerov2.yaml``).
        """
        return self.params.t200.thrust_of_command(self.params.t200.command_of_pwm(pwm))

    def wrench(self, thrust_n) -> np.ndarray:
        """The 6-wrench ``tau = T f`` of a thrust vector [N]."""
        return self.params.allocation @ np.asarray(thrust_n, float)

    # ---- internals

    def _deriv(self, y: np.ndarray, steady_thrust: np.ndarray) -> np.ndarray:
        """The packed derivative. ``steady_thrust`` is the thrust the current command asks for, already
        converted, because it is constant across the four RK4 stages."""
        p = self.params
        q = frames.quat_normalize(y[3:7])
        rot = frames.quat_to_rot(q)
        nu = y[7:13]
        thrust = y[13:]

        tau = p.allocation @ thrust
        if self.bounds is not None:
            tau = tau + self._bound_wrench(y[0:3], rot, nu[:3])
        nu_dot = p.M_inv @ (tau - coriolis_nu(p, nu) - damping(p, nu) - restoring(p, rot))

        d = np.empty_like(y)
        d[0:3] = rot @ nu[:3]
        d[3:7] = frames.quat_derivative(q, nu[3:])
        d[7:13] = nu_dot
        d[13:] = (steady_thrust - thrust) / p.t200.tau_s
        return d

    def _bound_wrench(self, p_ned: np.ndarray, rot: np.ndarray, v_frd: np.ndarray) -> np.ndarray:
        """The soft bound's wrench in body FRD. NON-PHYSICAL - see the module docstring.

        Stated in tank coordinates (where the walls are axis-aligned), converted through ``frames``, applied
        as a pure force at the body origin: no moment, so pressing against a wall cannot spin the vehicle.
        """
        b = self.bounds
        p_tank = frames.position_ned_to_tank(p_ned, self.water_level)
        v_tank = frames.vec_enu_ned(rot @ v_frd)

        f_tank = np.zeros(3)
        worst = 0.0
        for i in range(3):
            if p_tank[i] < b.lo[i]:
                pen = b.lo[i] - p_tank[i]
                f_tank[i] += b.stiffness * pen - b.damping * min(v_tank[i], 0.0)
                worst = max(worst, pen)
            elif p_tank[i] > b.hi[i]:
                pen = p_tank[i] - b.hi[i]
                f_tank[i] -= b.stiffness * pen + b.damping * max(v_tank[i], 0.0)
                worst = max(worst, pen)
        if p_tank[2] > b.surface_z:                 # the free surface: stiffer, so the vehicle sits on it
            pen = p_tank[2] - b.surface_z
            f_tank[2] -= b.surface_stiffness * pen + b.surface_damping * max(v_tank[2], 0.0)
            worst = max(worst, pen)

        if worst > 0.0:
            self.bound_activations += 1
            self.bound_max_penetration_m = max(self.bound_max_penetration_m, worst)
            log.debug("soft tank bound active: penetration %.4f m at tank %s", worst, np.round(p_tank, 3))
        return np.concatenate([rot.T @ frames.vec_enu_ned(f_tank), np.zeros(3)])


# ------------------------------------------------------------------------------------------- helpers

def make_vehicle(params_path: str | Path | None = None, tank_size=(3.0, 3.0, 3.0), sand_base: float = 0.3,
                 water_level: float = 2.9) -> Vehicle:
    """A ``Vehicle`` with its soft bound built from the tank geometry.

    The three tank numbers are passed in rather than read from ``config/tank.yaml`` here, so that this module
    stays independent of the repo's config layering; the runner wires them from ``tank.size``,
    ``sand.thickness`` and ``tank.water_level``.
    """
    params = load_params(params_path)
    k, c, ks, cs = params.bound_gains
    bounds = TankBounds.from_tank(tank_size, sand_base, water_level, params.bound_margin_m,
                                 params.bound_sand_extra_m, k, c, ks, cs)
    return Vehicle(params, water_level=water_level, bounds=bounds)


@dataclass(frozen=True)
class Trajectory:
    """What ``simulate`` records: one row per recorded step, simulated time only."""
    t: np.ndarray
    position_ned: np.ndarray        # (n, 3)
    quat_frd_ned: np.ndarray        # (n, 4)
    velocity_frd: np.ndarray        # (n, 3)
    rate_frd: np.ndarray            # (n, 3)
    thrust_n: np.ndarray            # (n, k)
    energy_j: np.ndarray            # (n,)

    @property
    def rpy_ned_deg(self) -> np.ndarray:
        return np.array([frames.rpy_deg_from_quat(q) for q in self.quat_frd_ned])

    def position_tank(self, water_level: float) -> np.ndarray:
        """The recorded positions in tank/ENU metres (needs the water level, hence not a property)."""
        return np.array([frames.position_ned_to_tank(p, water_level) for p in self.position_ned])


def simulate(vehicle: Vehicle, state: State, command, duration: float, dt: float | None = None,
             record_every: int = 1) -> tuple[State, Trajectory]:
    """Integrate ``duration`` simulated seconds at a fixed step and record the run.

    ``command`` is either a fixed array of normalised thruster commands or a callable ``(t, state) -> array``.
    Returns the final state and the recorded ``Trajectory``. No clock, no sleep: the only notion of time is
    the accumulated ``dt``, so the same call on any machine gives the same trajectory.
    """
    dt = vehicle.params.dt if dt is None else float(dt)
    n = int(round(float(duration) / dt))
    fixed = None if callable(command) else np.asarray(command, float)
    rows: list[tuple] = []
    for i in range(n + 1):
        if i % int(record_every) == 0:
            rows.append((state.t, state.position_ned, state.quat_frd_ned, state.velocity_frd, state.rate_frd,
                         state.thrust_n, vehicle.energy(state)))
        if i == n:
            break
        u = fixed if fixed is not None else np.asarray(command(state.t, state), float)
        state = vehicle.step(state, u, dt)
    cols = list(zip(*rows))
    return state, Trajectory(np.array(cols[0]), np.array(cols[1]), np.array(cols[2]), np.array(cols[3]),
                             np.array(cols[4]), np.array(cols[5]), np.array(cols[6]))
