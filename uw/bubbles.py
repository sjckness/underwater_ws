"""Gas leak bubble plume: Lagrangian bubbles on the GPU (Warp) riding on a FlowField.

The physics that needs no GPU (source, sizes, slip, integral plume) is in plume.py. Per substep:
1. the flow field writes the liquid velocity and the local turbulence scales at every bubble (``FlowField.sample``);
   the integral model gives the Gaussian plume flow, its radial part from continuity (entrainment inflow outside);
2. ``step_kernel`` moves the bubbles: liquid velocity + Ornstein-Uhlenbeck turbulence + Fan & Tsuchiya slip,
   zigzag/helical path oscillation above 1.8 mm, isothermal expansion with the falling pressure, burst at the free
   surface.
Gas bookkeeping: each bubble carries the gas amount p0*V0 (constant, isothermal). It is summed in ``stats`` when the
bubble bursts or is dropped because the pool is full, so emitted = alive + burst + dropped.
Not modelled: coalescence/breakup after formation, dissolution (negligible over 3 m), tank recirculation.
"""
from __future__ import annotations

import math

import numpy as np
import warp as wp

from . import plume
from . import water as wat
from .config import ConfigError, get
from .plume import N_CL

# stats slots (float64 accumulators)
S_BURST = wp.constant(0)            # bubbles burst at the surface
S_BURST_GAS = wp.constant(1)        # their gas amount p0*V0 [Pa m^3]
S_BURST_AGE = wp.constant(2)        # sum of their ages at burst [s]
S_LOST = wp.constant(3)             # bubbles dropped because the pool was full
S_LOST_GAS = wp.constant(4)         # their gas amount [Pa m^3]
N_STATS = 5


# ------------------------------------------------------------------------------------- warp kernels

@wp.func
def fan_tsuchiya(d: float, rho: float, mu: float, sigma: float, kb: float) -> float:
    t1 = rho * 9.80665 * d * d / (kb * mu)
    t2 = 2.0 * 1.4 * sigma / (rho * d) + 9.80665 * d * 0.5
    return wp.pow(wp.pow(t1, -0.8) + wp.pow(t2, -0.4), -1.25)


@wp.func
def gas_amount(r0: float, z0: float, p_surf: float, rho: float, water_level: float) -> float:
    """p0 * V0 of a bubble released with radius r0 at height z0."""
    p0 = p_surf + rho * 9.80665 * (water_level - z0)
    return p0 * (4.0 / 3.0) * wp.pi * r0 * r0 * r0


@wp.kernel
def mark_free_kernel(alive: wp.array(dtype=wp.int32), free: wp.array(dtype=wp.int32)):
    i = wp.tid()
    free[i] = 1 - alive[i]


@wp.kernel
def list_free_kernel(free: wp.array(dtype=wp.int32), offset: wp.array(dtype=wp.int32),
                     free_idx: wp.array(dtype=wp.int32)):
    i = wp.tid()
    if free[i] != 0:
        free_idx[offset[i]] = i


@wp.kernel
def spawn_kernel(step: int, heads: wp.array(dtype=wp.int32), counts: wp.array(dtype=wp.int32), cap: int,
                 free: wp.array(dtype=wp.int32), offset: wp.array(dtype=wp.int32), free_idx: wp.array(dtype=wp.int32),
                 new_pos: wp.array(dtype=wp.vec3), new_r0: wp.array(dtype=float), new_misc: wp.array(dtype=wp.vec3),
                 p_surf: float, rho: float, water_level: float,
                 pos: wp.array(dtype=wp.vec3), r0: wp.array(dtype=float), z0: wp.array(dtype=float),
                 r: wp.array(dtype=float), alive: wp.array(dtype=wp.int32), turb: wp.array(dtype=wp.vec3),
                 misc: wp.array(dtype=wp.vec3), age: wp.array(dtype=float), stats: wp.array(dtype=wp.float64)):
    """The ``counts[step]`` new bubbles of a substep (rows of the batch arrays) into free slots."""
    i = wp.tid()
    if i >= counts[step]:
        return
    head = heads[step]
    n_free = offset[cap - 1] + free[cap - 1]
    k = int(0)
    if i < n_free:                 # a free slot
        k = free_idx[i]
    else:                          # pool full: overwrite a live bubble (counted); a slot freed this step is taken
        k = (head + i) % cap
        if free[k] != 0:           # by another new bubble, so this one is dropped instead (counted)
            wp.atomic_add(stats, S_LOST, wp.float64(1.0))
            wp.atomic_add(stats, S_LOST_GAS, wp.float64(gas_amount(new_r0[i], new_pos[i][2], p_surf, rho,
                                                                   water_level)))
            return
        wp.atomic_add(stats, S_LOST, wp.float64(1.0))
        wp.atomic_add(stats, S_LOST_GAS, wp.float64(gas_amount(r0[k], z0[k], p_surf, rho, water_level)))
    pos[k] = new_pos[i]
    r0[k] = new_r0[i]
    r[k] = new_r0[i]
    z0[k] = new_pos[i][2]
    alive[k] = 1
    turb[k] = wp.vec3(0.0, 0.0, 0.0)
    misc[k] = new_misc[i]          # (wobble direction angle, helicity, phase)
    age[k] = 0.0


@wp.kernel
def integral_sample_kernel(cl_pos: wp.array(dtype=wp.vec3), cl_tan: wp.array(dtype=wp.vec3),
                           cl_u: wp.array(dtype=float), cl_b: wp.array(dtype=float), cl_a: wp.array(dtype=float),
                           cl_c: wp.array(dtype=float), n_cl: int,
                           pos: wp.array(dtype=wp.vec3), alive: wp.array(dtype=wp.int32),
                           u_liq: wp.array(dtype=wp.vec3), scales: wp.array(dtype=wp.vec3)):
    i = wp.tid()
    if alive[i] == 0:
        return
    p = pos[i]
    best = float(1.0e9)
    bk = int(0)
    for k in range(n_cl):
        dd = wp.length_sq(p - cl_pos[k])
        if dd < best:
            best = dd
            bk = k
    t = cl_tan[bk]
    rel = p - cl_pos[bk]
    ax = wp.dot(rel, t)
    radial = rel - ax * t
    rr = wp.length(radial)
    b = cl_b[bk]
    uc = cl_u[bk]
    ca = cl_a[bk]
    cc = cl_c[bk]
    if ax < 0.0 and bk == 0:       # behind the source: no plume
        uc = 0.0
        ca = 0.0
        cc = 0.0
    eta2 = rr * rr / (b * b)
    g = wp.exp(-eta2)
    u = uc * g * t
    if rr > 1.0e-6:                # radial flow from continuity: outward in a decelerating core, entrainment outside
        u = u - ((ca * (1.0 - g) + cc * (1.0 - (1.0 + eta2) * g)) / rr) * (radial / rr)
    u_liq[i] = u
    scales[i] = wp.vec3(uc, b, uc * wp.exp(-0.5 * rr * rr / (b * b)))   # turbulence: velocity, length, weight


@wp.kernel
def step_kernel(dt: float, step: int, seeds: wp.array(dtype=wp.int32), u_liq: wp.array(dtype=wp.vec3),
                scales: wp.array(dtype=wp.vec3),
                turb_k: float, wobble: int, rho: float, mu: float, sigma: float, kb: float,
                p_surf: float, water_level: float, size_x: float, size_y: float,
                pos: wp.array(dtype=wp.vec3), r0: wp.array(dtype=float), z0: wp.array(dtype=float),
                r: wp.array(dtype=float), alive: wp.array(dtype=wp.int32), turb: wp.array(dtype=wp.vec3),
                misc: wp.array(dtype=wp.vec3), age: wp.array(dtype=float), vel_out: wp.array(dtype=wp.vec3),
                stats: wp.array(dtype=wp.float64), bursts: wp.array2d(dtype=wp.int32), burst_x0: float,
                burst_y0: float, burst_cell: float):
    i = wp.tid()
    if alive[i] == 0:
        return
    seed = seeds[step]
    sc = scales[i]
    u_loc = sc[0]
    b_loc = sc[1]
    w_loc = sc[2]
    # Ornstein-Uhlenbeck turbulence, intensity tied to the local plume
    state = wp.rand_init(seed, i)
    tl = wp.max(0.3 * b_loc / wp.max(u_loc, 0.02), 0.02)
    a = wp.exp(-dt / tl)
    sig = turb_k * (w_loc + 0.03 * u_loc + 0.005)
    q = sig * wp.sqrt(1.0 - a * a)
    tb = turb[i] * a + q * wp.vec3(wp.randn(state), wp.randn(state), wp.randn(state))
    turb[i] = tb
    # slip + path oscillation
    rad = r[i]
    d = 2.0 * rad
    vt = fan_tsuchiya(d, rho, mu, sigma, kb)
    v = u_liq[i] + tb + wp.vec3(0.0, 0.0, vt)
    m = misc[i]
    if wobble != 0 and d > 1.8e-3:
        amp = 0.35 * vt * wp.min(1.0, (d - 1.8e-3) / 2.0e-3)
        om = 2.0 * wp.pi * 0.1 * vt / d
        ph = om * age[i] + m[2]
        e1 = wp.vec3(wp.cos(m[0]), wp.sin(m[0]), 0.0)
        e2 = wp.vec3(-wp.sin(m[0]), wp.cos(m[0]), 0.0)
        v = v + amp * (wp.cos(ph) * e1 + m[1] * wp.sin(ph) * e2)
    p = pos[i] + v * dt
    # keep off the walls
    p = wp.vec3(wp.clamp(p[0], rad, size_x - rad), wp.clamp(p[1], rad, size_y - rad), p[2])
    pos[i] = p
    vel_out[i] = v
    age[i] = age[i] + dt
    # isothermal expansion: r = r0 (P0 / P)^(1/3)
    p0 = p_surf + rho * 9.80665 * (water_level - z0[i])
    pz = p_surf + rho * 9.80665 * wp.max(water_level - p[2], 0.0)
    r[i] = r0[i] * wp.pow(p0 / pz, 1.0 / 3.0)
    if p[2] >= water_level - r[i]:
        alive[i] = 0
        wp.atomic_add(stats, S_BURST, wp.float64(1.0))
        wp.atomic_add(stats, S_BURST_GAS, wp.float64(gas_amount(r0[i], z0[i], p_surf, rho, water_level)))
        wp.atomic_add(stats, S_BURST_AGE, wp.float64(age[i]))
        bx = int(wp.floor((p[0] - burst_x0) / burst_cell))     # where it burst: the surface boil
        by = int(wp.floor((p[1] - burst_y0) / burst_cell))
        if bx >= 0 and by >= 0 and bx < bursts.shape[0] and by < bursts.shape[1]:
            wp.atomic_add(bursts, bx, by, 1)


# -------------------------------------------------------------------------------------- flow fields

class FlowField:
    """The liquid flow the bubbles ride on (plan: FlowField = integral | lbm).

    A substep has a host part, ``advance``, and a GPU part, ``sample``, which gets the host part's result already on
    the device: BubbleSim prepares a batch of substeps on the host, uploads it at once and launches the GPU parts as
    one CUDA graph."""
    state_len = 0                       # vec3 values per substep that ``advance`` returns for ``sample``

    def advance(self, dt: float) -> np.ndarray:
        """Advance the flow's own state by one substep (host); returns its (state_len, 3) float32 GPU input."""
        raise NotImplementedError

    def sample(self, pos: wp.array, alive: wp.array, u_liq: wp.array, scales: wp.array, state: wp.array) -> None:
        """Write the liquid velocity and the turbulence scales (velocity, length, weight) at every alive bubble."""
        raise NotImplementedError


class IntegralFlow(FlowField):
    """One integral forced plume (plume.integrate_plume) from the source, with a travelling centreline meander.

    A buried leak's vents are a few cm apart, so they feed one merged pure plume with the total gas flux, starting
    at their mean position; the vents only decide where bubbles are released.
    """

    def __init__(self, cfg: dict, source: plume.Source, water_level: float, p_surf: float, dt: float,
                 rng: np.random.Generator, device: str):
        pc = get(cfg, "plume")
        self.alpha = float(pc["entrainment_coeff"])
        self.dt, self.rng, self.device = dt, rng, device
        rho = source.rho

        def q_ratio(z):            # gas expands as it rises
            return source.p_release / (p_surf + rho * wat.G * max(water_level - z, 0.0))
        self.plume = plume.integrate_plume(source.positions.mean(0), source.direction, source.momentum / rho,
                                           source.q_gas, source.slip_mean, self.alpha, float(pc["jet_radius0"]),
                                           water_level, q_ratio=q_ratio)
        # meander: OU history of 2-D offsets, sampled at the step rate
        self.meander_amp = float(pc["meander_amp"])
        self.meander_T = float(pc["meander_time_s"])
        self.hist_len = int(self.plume["tau"][-1] / dt) + 2
        self.hist = np.zeros((self.hist_len, 2))
        self.hist_head = 0
        self.cl_tan = wp.array(self.plume["tangent"].astype(np.float32), dtype=wp.vec3, device=device)
        self.cl_u = wp.array(self.plume["u_c"].astype(np.float32), dtype=float, device=device)
        self.cl_b = wp.array(self.plume["b"].astype(np.float32), dtype=float, device=device)
        # radial velocity of the Gaussian profile u_c(s) exp(-r^2/b(s)^2) from continuity:
        # v_r = -[A (1 - e) + C (1 - (1 + eta^2) e)] / r, A = b^2 u_c'/2, C = u_c b b', e = exp(-eta^2), eta = r/b;
        # far out it is -(dQ/ds) / (2 pi r), the integral model's entrainment
        u, b = self.plume["u_c"], self.plume["b"]
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.plume["pos"], axis=0), axis=1))])
        self.cl_a = wp.array((b * b * np.gradient(u, s) / 2).astype(np.float32), dtype=float, device=device)
        self.cl_c = wp.array((u * b * np.gradient(b, s)).astype(np.float32), dtype=float, device=device)
        self.cl_pos_host = self.plume["pos"].copy()

    state_len = N_CL                    # the centreline, meandering

    def advance(self, dt: float) -> np.ndarray:
        a = math.exp(-dt / self.meander_T)
        prev = self.hist[(self.hist_head - 1) % self.hist_len]
        self.hist[self.hist_head] = prev * a + math.sqrt(1 - a * a) * self.rng.normal(size=2)
        self.hist_head = (self.hist_head + 1) % self.hist_len
        pl = self.plume
        lag = np.minimum((pl["tau"] / self.dt).astype(int), self.hist_len - 1)
        off2 = self.hist[(self.hist_head - 1 - lag) % self.hist_len]
        # the source cannot move: wander grows with the distance travelled
        grow = np.clip(pl["tau"] / max(self.meander_T, 1e-3), 0, 1)
        off = np.concatenate([off2, np.zeros((N_CL, 1))], 1) * (self.meander_amp * pl["b"] * grow)[:, None]
        off -= np.sum(off * pl["tangent"], 1, keepdims=True) * pl["tangent"]
        self.cl_pos_host = pl["pos"] + off
        return self.cl_pos_host.astype(np.float32)

    def sample(self, pos, alive, u_liq, scales, state) -> None:
        wp.launch(integral_sample_kernel, dim=len(pos), device=self.device,
                  inputs=[state, self.cl_tan, self.cl_u, self.cl_b, self.cl_a, self.cl_c, N_CL,
                          pos, alive, u_liq, scales])


def make_flow_field(cfg: dict, source: plume.Source, water_level: float, p_surf: float, dt: float,
                    rng: np.random.Generator, device: str) -> FlowField:
    kind = get(cfg, "plume.flow_field")
    if kind == "integral":
        return IntegralFlow(cfg, source, water_level, p_surf, dt, rng, device)
    if kind == "lbm":
        raise NotImplementedError("plume.flow_field 'lbm' (Warp lattice Boltzmann, 2-way coupled) is milestone M6; "
                                  "use 'integral'")
    raise ConfigError(f"plume.flow_field must be integral or lbm, got {kind!r}")


# --------------------------------------------------------------------------------------- simulator

class BubbleSim:
    def __init__(self, cfg: dict, scene, device: str = "cuda:0"):
        self.device = device
        self.cfg = cfg
        lc, pc, wc = get(cfg, "leak"), get(cfg, "plume"), get(cfg, "water")
        self.rng = np.random.default_rng(int(cfg["seed"]) + 101)
        self.seed = (int(cfg["seed"]) * 1000003) % (2 ** 31)
        self.source = src = plume.leak_source(cfg, scene)
        self.rho = src.rho
        self.water_level = float(scene.water_level)
        self.size = scene.size
        self.p_surf = wat.P_ATM + self.rho * wat.G * float(wc["ambient_depth_m"])
        self.turb_k = float(pc["turbulence"])
        self.wobble = 1 if pc["wobble"] else 0
        self.dt = float(pc["dt"])
        self.cap = int(pc["max_bubbles"])
        self.jet_radius0 = float(pc["jet_radius0"])
        self.pockmark_radius = float(lc["pockmark_radius"])
        self.t = 0.0
        self.step_count = 0
        self.head = 0
        self.flow = make_flow_field(cfg, src, self.water_level, self.p_surf, self.dt, self.rng, device)
        self.emitter = plume.Emitter(src.q_gas, src.sizes, self.rng)
        prof = lc["profile"]
        if prof["kind"] not in ("constant", "ramp", "intermittent"):
            raise ConfigError(f"leak.profile.kind must be constant | ramp | intermittent, got {prof['kind']!r}")
        self.profile = (prof["kind"], float(prof["ramp_s"]), float(prof["period_s"]), float(prof["duty"]),
                        float(prof["phase"]))
        self.emitted = 0
        self.emitted_gas = 0.0                       # sum of p0*V0 [Pa m^3]
        # buried leak: intermittent vents, OU-driven soft switching of each vent's share of the gas
        k = len(src.positions)
        self.vent_state = np.zeros(k)
        self.vent_w = np.full(k, 1.0 / k)
        self.vent_tau = float(lc["vent_switch_s"])

        cap, dev = self.cap, device
        self.pos = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.vel = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.r0 = wp.zeros(cap, dtype=float, device=dev)
        self.z0 = wp.zeros(cap, dtype=float, device=dev)
        self.r = wp.zeros(cap, dtype=float, device=dev)
        self.alive = wp.zeros(cap, dtype=wp.int32, device=dev)
        self.turb = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.misc = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.age = wp.zeros(cap, dtype=float, device=dev)
        self.u_liq = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.scales = wp.zeros(cap, dtype=wp.vec3, device=dev)
        self.stats = wp.zeros(N_STATS, dtype=wp.float64, device=dev)
        self.free = wp.zeros(cap, dtype=wp.int32, device=dev)        # slot allocation: 1 = free
        self.free_offset = wp.zeros(cap, dtype=wp.int32, device=dev)
        self.free_idx = wp.zeros(cap, dtype=wp.int32, device=dev)
        # Substeps run in batches: the host part of every substep (vents, meander, new bubbles: all the host random
        # numbers, in the order of one substep after the other) goes into pinned staging arrays, one upload, then the
        # GPU part of all substeps, captured once per batch length as a CUDA graph (plume.cuda_graph). The results
        # are bit-identical to launching every substep on its own (tests/gpu/test_bubbles.py).
        # bursts per surface cell (counted until someone reads and clears them: the surface boil, uw/surface.py),
        # on a grid around where the plume reaches the surface
        self.burst_cell = float(pc["burst_grid_m"])
        pl = getattr(self.flow, "plume", None)
        if pl is not None:
            half = (3.0 + 2.0 * float(pc["meander_amp"])) * float(pl["b"][-1]) + 0.1
            centre = pl["pos"][-1, :2]
        else:
            half, centre = 0.5 * float(max(self.size[0], self.size[1])), self.size[:2] / 2
        n = int(math.ceil(2 * half / self.burst_cell))
        self.burst_origin = (float(centre[0] - half), float(centre[1] - half))
        self.bursts = wp.zeros((n, n), dtype=wp.int32, device=dev)
        self.use_graph = bool(pc["cuda_graph"]) and wp.get_device(dev).is_cuda
        self.batch_max = 240
        self.max_new = 128                           # new bubbles per substep the batch holds (grows on demand)
        self._alloc_batch()

    # ---------------------------------------------------------------------------------------- info
    def summary(self) -> dict:
        src, pl = self.source, self.flow.plume
        z_rel = pl["pos"][:, 2] - pl["pos"][0, 2]

        def at(h):
            return float(np.interp(h, z_rel, pl["u_c"])), float(np.interp(h, z_rel, pl["b"]))
        return {"gas_flow_lpm_at_source": src.q_gas * 60000, "mass_flow_kg_s": src.mdot,
                "source": {k: (bool(v) if isinstance(v, (bool, np.bool_)) else float(v)) for k, v in src.info.items()},
                "buried": src.buried, "vents": len(src.positions), "d50_mm": src.sizes.d50 * 1e3,
                "mean_bubble_volume_mm3": src.sizes.mean_volume * 1e9,
                "bubbles_per_s": src.q_gas / src.sizes.mean_volume, "slip_mean_m_s": src.slip_mean,
                "plume_uc_b_at_0.5m": at(0.5), "plume_uc_b_at_1.5m": at(1.5),
                "liquid_travel_time_s": float(pl["tau"][-1])}

    def flow_factor(self, t: float) -> float:
        """The leak's flow at sim time t over its nominal flow (leak.profile)."""
        kind, ramp, period, duty, phase = self.profile
        if kind == "ramp":
            return min(max(t / ramp, 0.0), 1.0) if ramp > 0 else 1.0
        if kind == "intermittent":
            return 1.0 if (t / period + phase) % 1.0 < duty else 0.0
        return 1.0

    def flow_now_lpm(self) -> float:
        """Gas flow leaving the leak now [L/min at the release pressure]."""
        return self.source.q_gas * self.flow_factor(self.t) * 6e4

    def counters(self) -> dict:
        """Running totals since the start: emitted and burst bubbles, gas amounts [Pa m^3], age sum at burst [s]."""
        s = self.stats.numpy()
        return {"t": self.t, "emitted": self.emitted, "emitted_gas": self.emitted_gas,
                "burst": float(s[S_BURST]), "burst_gas": float(s[S_BURST_GAS]), "burst_age": float(s[S_BURST_AGE]),
                "lost": float(s[S_LOST]), "lost_gas": float(s[S_LOST_GAS])}

    def gas_budget(self) -> dict:
        """Gas amounts p*V [Pa m^3]. ``alive`` is computed from each bubble's current radius and depth, so the
        balance emitted = alive + burst + lost also checks the expansion law."""
        snap = self.snapshot()
        p = self.p_surf + self.rho * wat.G * np.maximum(self.water_level - snap["pos"][:, 2].astype(float), 0.0)
        c = self.counters()
        return {"emitted": c["emitted_gas"],
                "alive": float(np.sum(p * 4.0 / 3.0 * math.pi * snap["radius"].astype(float) ** 3)),
                "burst": c["burst_gas"], "lost": c["lost_gas"]}

    # ---------------------------------------------------------------------------------------- step
    def _update_vents(self):
        k = len(self.vent_w)
        if k > 1:
            a = math.exp(-self.dt / self.vent_tau)
            self.vent_state = self.vent_state * a + math.sqrt(1 - a * a) * self.rng.normal(size=k)
            w = np.exp(1.8 * self.vent_state)
            self.vent_w = w / w.sum()

    def _alloc_batch(self):
        n, m, dev, nf = self.batch_max, self.max_new, self.device, max(self.flow.state_len, 1)
        self.b_state = wp.zeros((n, nf), dtype=wp.vec3, device=dev)
        self.b_pos = wp.zeros((n, m), dtype=wp.vec3, device=dev)
        self.b_r0 = wp.zeros((n, m), dtype=float, device=dev)
        self.b_misc = wp.zeros((n, m), dtype=wp.vec3, device=dev)
        self.b_count, self.b_head, self.b_seed = (wp.zeros(n, dtype=wp.int32, device=dev) for _ in range(3))
        pinned = wp.get_device(dev).is_cuda
        self.h = {k: wp.zeros(a.shape, dtype=a.dtype, device="cpu", pinned=pinned)
                  for k, a in (("state", self.b_state), ("pos", self.b_pos), ("r0", self.b_r0), ("misc", self.b_misc),
                               ("count", self.b_count), ("head", self.b_head), ("seed", self.b_seed))}
        self.h_np = {k: a.numpy() for k, a in self.h.items()}         # zero-copy views of the staging arrays
        self.uploaded = None                                            # event: the staging arrays may be reused
        self.graphs = {}

    def _new_bubbles(self, dt):
        """Host part of a substep's emission: sizes, release points, path-oscillation parameters (pos, r0, misc as
        float32), and the gas bookkeeping."""
        d = self.emitter.emit(dt)
        n = min(len(d), self.cap)
        if n == 0:
            return None
        d = d[:n]
        rad = d / 2
        src = self.source
        if not src.buried:      # bubbles form along the first 2 cm of the jet
            t = src.direction / np.linalg.norm(src.direction)
            s = self.rng.uniform(0.0, 0.02, n)
            lat = self.rng.normal(size=(n, 3)) * (self.jet_radius0 / math.sqrt(2.0) + 0.1 * s)[:, None]
            lat -= np.sum(lat * t, 1, keepdims=True) * t
            pos = src.positions[0] + s[:, None] * t + lat
        else:                   # out of the vents, inside the pockmark floor
            which = self.rng.choice(len(src.positions), size=n, p=self.vent_w)
            rr = self.pockmark_radius * 0.3 * np.sqrt(self.rng.uniform(size=n))
            th = self.rng.uniform(0, 2 * np.pi, n)
            pos = src.positions[which] + np.stack([rr * np.cos(th), rr * np.sin(th), rad], 1)
        misc = np.stack([self.rng.uniform(0, 2 * np.pi, n), self.rng.uniform(0, 1, n) ** 2,
                         self.rng.uniform(0, 2 * np.pi, n)], 1)
        pos32, rad32 = pos.astype(np.float32), rad.astype(np.float32)
        p0 = self.p_surf + self.rho * wat.G * (self.water_level - pos32[:, 2].astype(float))
        self.emitted += n
        self.emitted_gas += float(np.sum(p0 * 4.0 / 3.0 * math.pi * rad32.astype(float) ** 3))
        return pos32, rad32, misc.astype(np.float32)

    def _prepare(self, k: int) -> None:
        """Host parts of the next k substeps into the staging arrays (grown if a substep emits more than fits)."""
        if self.uploaded is not None:
            wp.synchronize_event(self.uploaded)
        rows = []
        for j in range(k):
            self._update_vents()
            self.emitter.q = self.source.q_gas * self.flow_factor(self.t)
            state = self.flow.advance(self.dt)
            rows.append((state, self._new_bubbles(self.dt), self.head, (self.seed + self.step_count) % 2 ** 31))
            n = 0 if rows[-1][1] is None else len(rows[-1][1][0])
            self.head = (self.head + n) % self.cap
            self.t += self.dt
            self.step_count += 1
        biggest = max((len(r[1][0]) for r in rows if r[1] is not None), default=0)
        if biggest > self.max_new:
            self.max_new = 1 << (biggest - 1).bit_length()
            self._alloc_batch()
        h = self.h_np
        for j, (state, new, head, seed) in enumerate(rows):
            if state is not None:
                h["state"][j] = state
            h["count"][j] = 0 if new is None else len(new[0])
            h["head"][j], h["seed"][j] = head, seed
            if new is not None:
                n = len(new[0])
                h["pos"][j, :n], h["r0"][j, :n], h["misc"][j, :n] = new
        for key in ("state", "pos", "r0", "misc"):        # only the k rows used
            dst = getattr(self, f"b_{key}")
            wp.copy(dst, self.h[key], count=k * dst.shape[1])
        for key in ("count", "head", "seed"):
            wp.copy(getattr(self, f"b_{key}"), self.h[key], count=k)
        self.uploaded = wp.record_event()

    def _launch(self, k: int) -> None:
        """GPU parts of k prepared substeps: free-slot list, spawn, flow sample, move."""
        src, dev, cap = self.source, self.device, self.cap
        for j in range(k):
            wp.launch(mark_free_kernel, dim=cap, device=dev, inputs=[self.alive, self.free])
            wp.utils.array_scan(self.free, self.free_offset, inclusive=False)
            wp.launch(list_free_kernel, dim=cap, device=dev, inputs=[self.free, self.free_offset, self.free_idx])
            wp.launch(spawn_kernel, dim=self.max_new, device=dev,
                      inputs=[j, self.b_head, self.b_count, cap, self.free, self.free_offset, self.free_idx,
                              self.b_pos[j], self.b_r0[j], self.b_misc[j], self.p_surf, self.rho, self.water_level,
                              self.pos, self.r0, self.z0, self.r, self.alive, self.turb, self.misc, self.age,
                              self.stats])
            self.flow.sample(self.pos, self.alive, self.u_liq, self.scales, self.b_state[j])
            wp.launch(step_kernel, dim=cap, device=dev,
                      inputs=[self.dt, j, self.b_seed, self.u_liq, self.scales, self.turb_k, self.wobble,
                              self.rho, src.mu, src.sigma, src.kb, self.p_surf, self.water_level,
                              float(self.size[0]), float(self.size[1]),
                              self.pos, self.r0, self.z0, self.r, self.alive, self.turb, self.misc, self.age,
                              self.vel, self.stats, self.bursts, self.burst_origin[0], self.burst_origin[1],
                              self.burst_cell])

    def step(self, dt_total: float):
        """round(dt_total / plume.dt) substeps, in batches of up to 240."""
        n_sub = max(1, int(round(dt_total / self.dt)))
        while n_sub > 0:
            k = min(n_sub, self.batch_max)
            self._prepare(k)
            if self.use_graph:
                if k not in self.graphs:
                    wp.capture_begin(self.device)
                    try:
                        self._launch(k)
                    finally:
                        self.graphs[k] = wp.capture_end(self.device)
                wp.capture_launch(self.graphs[k])
            else:
                self._launch(k)
            n_sub -= k

    def warmup(self, seconds: float):
        self.step(seconds)

    def snapshot(self) -> dict:
        """Alive bubbles on the host: pos (N,3), radius (N,), vel (N,3), age (N,)."""
        alive = self.alive.numpy().astype(bool)
        return {"pos": self.pos.numpy()[alive], "radius": self.r.numpy()[alive], "vel": self.vel.numpy()[alive],
                "age": self.age.numpy()[alive]}
