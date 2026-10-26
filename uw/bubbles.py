"""Gas leak bubble plume: integral buoyant-jet model for the liquid flow + Lagrangian bubbles on the GPU.

Physics, in order of importance for what a sonar/camera sees:
1. Source: choked or subsonic gas flow through the hole (``water.choked_leak``) gives the gas flow and
   the jet momentum flux; a buried leak releases through sediment vents with no momentum, switching
   intermittently between vents.
2. Liquid flow: a top-hat integral model of a forced bubble plume (entrainment ``alpha``; buoyancy per
   unit length = g * gas volume per unit length) gives the curved centreline, its velocity and width.
   Bubbles see the Gaussian equivalent (u_c = 2 u_tophat, b = b_tophat / sqrt 2) plus the entrainment
   inflow outside the core, Ornstein-Uhlenbeck turbulence and a travelling centreline meander.
3. Bubbles: log-normal sizes conserving the gas volume flux; slip velocity from Fan & Tsuchiya (1990,
   contaminated water); zigzag/helical path oscillation above 1.8 mm; isothermal expansion with the
   falling pressure; burst at the free surface.
Not modelled: coalescence/breakup after formation, dissolution (negligible over 3 m), tank recirculation.
"""
from __future__ import annotations

import math

import numpy as np
import warp as wp

from . import water as wat
from .config import get

N_CL = 160          # centreline samples per plume


# ------------------------------------------------------------------------------ integral plume (CPU)

def integrate_plume(origin, direction, momentum_k, q_gas, slip, alpha, b0, z_stop, ds=0.001, s_max=8.0):
    """Top-hat forced bubble plume. Returns arrays along the centreline (resampled to N_CL points):
    pos (N,3), tangent (N,3), u_c (Gaussian centreline speed), b (Gaussian width), tau (travel time)."""
    c = np.asarray(origin, float).copy()
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    u0 = max(math.sqrt(momentum_k / (math.pi * b0 * b0)) if momentum_k > 0 else 0.0, 0.05)
    Q = math.pi * b0 * b0 * u0
    M = Q * u0 * d
    g = wat.G
    rec_s, rec = [], []
    s = tau = 0.0
    while c[2] < z_stop and s < s_max:
        m = np.linalg.norm(M)
        u = m / Q
        t = M / m
        b = math.sqrt(Q / (math.pi * u))
        rec_s.append(s)
        rec.append((*c, *t, 2.0 * u, b / math.sqrt(2.0), tau))
        Q += 2 * math.pi * b * alpha * u * ds
        M = M + np.array([0.0, 0.0, g * q_gas / max(u + slip * t[2], 0.05)]) * ds
        c = c + t * ds
        s += ds
        tau += ds / u
    rec = np.array(rec)
    rec_s = np.array(rec_s)
    s_new = rec_s[-1] * (np.arange(N_CL) / (N_CL - 1)) ** 1.5      # dense near the source
    out = np.stack([np.interp(s_new, rec_s, rec[:, k]) for k in range(rec.shape[1])], 1)
    tan = out[:, 3:6] / np.linalg.norm(out[:, 3:6], axis=1, keepdims=True)
    return {"pos": out[:, 0:3], "tangent": tan, "u_c": out[:, 6], "b": out[:, 7], "tau": out[:, 8]}


def slip_velocity(d, rho, mu, sigma):
    """Fan & Tsuchiya 1990 terminal rise velocity, contaminated water (n = 0.8, c = 1.4, Kb0 = 14.7)."""
    mo = wat.G * mu ** 4 / (rho * sigma ** 3)
    kb = max(14.7 * mo ** -0.038, 12.0)
    n = 0.8
    t1 = rho * wat.G * d * d / (kb * mu)
    t2 = 2 * 1.4 * sigma / (rho * d) + wat.G * d / 2
    return (t1 ** -n + t2 ** (-n / 2)) ** (-1 / n)


# -------------------------------------------------------------------------------------- warp kernels

@wp.func
def fan_tsuchiya(d: float, rho: float, mu: float, sigma: float, kb: float) -> float:
    t1 = rho * 9.80665 * d * d / (kb * mu)
    t2 = 2.0 * 1.4 * sigma / (rho * d) + 9.80665 * d * 0.5
    return wp.pow(wp.pow(t1, -0.8) + wp.pow(t2, -0.4), -1.25)


@wp.kernel
def spawn_kernel(head: int, cap: int,
                 new_pos: wp.array(dtype=wp.vec3), new_r0: wp.array(dtype=float), new_misc: wp.array(dtype=wp.vec3),
                 pos: wp.array(dtype=wp.vec3), r0: wp.array(dtype=float), z0: wp.array(dtype=float),
                 r: wp.array(dtype=float), alive: wp.array(dtype=wp.int32), turb: wp.array(dtype=wp.vec3),
                 misc: wp.array(dtype=wp.vec3), age: wp.array(dtype=float)):
    i = wp.tid()
    k = (head + i) % cap
    pos[k] = new_pos[i]
    r0[k] = new_r0[i]
    r[k] = new_r0[i]
    z0[k] = new_pos[i][2]
    alive[k] = 1
    turb[k] = wp.vec3(0.0, 0.0, 0.0)
    misc[k] = new_misc[i]          # (wobble direction angle, helicity, phase)
    age[k] = 0.0


@wp.kernel
def step_kernel(dt: float, t_now: float, seed: int,
                cl_pos: wp.array(dtype=wp.vec3), cl_tan: wp.array(dtype=wp.vec3), cl_u: wp.array(dtype=float),
                cl_b: wp.array(dtype=float), n_plumes: int, n_cl: int,
                alpha: float, turb_k: float, wobble: int,
                rho: float, mu: float, sigma: float, kb: float,
                p_surf: float, water_level: float, size_x: float, size_y: float,
                pos: wp.array(dtype=wp.vec3), r0: wp.array(dtype=float), z0: wp.array(dtype=float),
                r: wp.array(dtype=float), alive: wp.array(dtype=wp.int32), turb: wp.array(dtype=wp.vec3),
                misc: wp.array(dtype=wp.vec3), age: wp.array(dtype=float), vel_out: wp.array(dtype=wp.vec3)):
    i = wp.tid()
    if alive[i] == 0:
        return
    p = pos[i]
    u_l = wp.vec3(0.0, 0.0, 0.0)
    u_loc = float(0.0)
    b_loc = float(0.05)
    w_loc = float(0.0)
    for pl in range(n_plumes):
        best = float(1.0e9)
        bk = int(0)
        for k in range(n_cl):
            dd = wp.length_sq(p - cl_pos[pl * n_cl + k])
            if dd < best:
                best = dd
                bk = k
        j = pl * n_cl + bk
        t = cl_tan[j]
        rel = p - cl_pos[j]
        ax = wp.dot(rel, t)
        radial = rel - ax * t
        rr = wp.length(radial)
        b = cl_b[j]
        uc = cl_u[j]
        if ax < 0.0 and bk == 0:      # behind the source: no plume
            uc = 0.0
        g = wp.exp(-rr * rr / (b * b))
        u_l = u_l + uc * g * t
        if rr > 1.0e-6:
            u_l = u_l - (alpha * uc * (b / rr) * (1.0 - g)) * (radial / rr)
        wgt = uc * wp.exp(-0.5 * rr * rr / (b * b))
        if wgt > w_loc:
            w_loc = wgt
            u_loc = uc
            b_loc = b
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
    v = u_l + tb + wp.vec3(0.0, 0.0, vt)
    m = misc[i]
    if wobble != 0 and d > 1.8e-3:
        amp = 0.35 * vt * wp.min(1.0, (d - 1.8e-3) / 2.0e-3)
        om = 2.0 * wp.pi * 0.1 * vt / d
        ph = om * age[i] + m[2]
        e1 = wp.vec3(wp.cos(m[0]), wp.sin(m[0]), 0.0)
        e2 = wp.vec3(-wp.sin(m[0]), wp.cos(m[0]), 0.0)
        v = v + amp * (wp.cos(ph) * e1 + m[1] * wp.sin(ph) * e2)
    p = p + v * dt
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


# --------------------------------------------------------------------------------------- simulator

class BubbleSim:
    def __init__(self, cfg: dict, scene, device: str = "cuda:0"):
        self.device = device
        self.cfg = cfg
        lc, pc, wc = get(cfg, "leak"), get(cfg, "plume"), get(cfg, "water")
        self.rng = np.random.default_rng(int(cfg["seed"]) + 101)
        t, s = float(wc["temperature_c"]), float(wc["salinity_psu"])
        self.rho, self.mu, self.sigma = wat.density(t, s), wat.viscosity(t, s), wat.surface_tension(t, s)
        mo = wat.G * self.mu ** 4 / (self.rho * self.sigma ** 3)
        self.kb = max(14.7 * mo ** -0.038, 12.0)
        self.water_level = float(scene.water_level)
        self.size = scene.size
        self.p_surf = wat.P_ATM + self.rho * wat.G * float(wc["ambient_depth_m"])
        self.alpha = float(pc["entrainment_coeff"])
        self.turb_k = float(pc["turbulence"])
        self.wobble = 1 if pc["wobble"] else 0
        self.dt = float(pc["dt"])
        self.cap = int(pc["max_bubbles"])
        self.t = 0.0
        self.step_count = 0
        self.head = 0
        self.vol_debt = 0.0

        leak = scene.leak
        self.leak = leak
        dmin, dmax = (x * 1e-3 for x in lc["d_range_mm"])
        self.d_range = (dmin, dmax)
        self.sigma_ln = float(lc["sigma_ln"])
        self.d50 = float(lc["d50_mm"]) * 1e-3
        src_z = leak.hole[2] if not leak.buried else float(np.mean(leak.vents[:, 2]))
        self.p_src = wat.pressure(src_z, self.water_level, self.rho, float(wc["ambient_depth_m"]))
        t_k = t + 273.15
        momentum = 0.0
        if lc["source"] == "pressure":
            p0 = float(lc["pipe_pressure_bar"]) * 1e5 + wat.pressure(leak.hole[2], self.water_level, self.rho,
                                                                      float(wc["ambient_depth_m"]))
            out = wat.choked_leak(p0, self.p_src, float(lc["hole_diameter_mm"]) * 1e-3, lc["gas"], t_k,
                                  float(lc["discharge_coeff"]))
            self.q_gas = out["q_ambient"]
            momentum = out["momentum_flux"]
            self.source_info = out
        elif lc["source"] == "flow":
            self.q_gas = float(lc["flow_lpm"]) / 60000.0
            self.source_info = {"q_ambient": self.q_gas}
        else:
            raise ValueError(f"leak.source must be pressure or flow, got {lc['source']!r}")
        if leak.buried:
            momentum = 0.0          # the sand takes the jet's momentum
            self.d50 *= float(lc["buried_size_factor"])
        # volume-weighted mean slip of the size distribution, for the buoyancy in the integral model
        ds_ = np.exp(np.linspace(np.log(dmin), np.log(dmax), 200))
        pdf = np.exp(-0.5 * (np.log(ds_ / self.d50) / self.sigma_ln) ** 2)
        wv = pdf * ds_ ** 3
        self.slip_mean = float(np.sum(wv * np.array([slip_velocity(x, self.rho, self.mu, self.sigma)
                                                     for x in ds_])) / np.sum(wv))
        self.mean_volume = float(np.sum(pdf * np.pi / 6 * ds_ ** 3) / np.sum(pdf))

        b0 = float(pc["jet_radius0"])
        mk = momentum / self.rho
        if not leak.buried:
            self.sources = leak.hole[None, :] + 0.002 * leak.direction[None, :]
            self.plumes = [integrate_plume(self.sources[0], leak.direction, mk, self.q_gas, self.slip_mean,
                                           self.alpha, b0, self.water_level)]
        else:
            self.sources = leak.vents + np.array([0.0, 0.0, 0.003])
            # pure plumes: the shape does not depend on the flow, the speed scales with (flow)^(1/3)
            self.plumes = [integrate_plume(v, [0, 0, 1], 0.0, self.q_gas, self.slip_mean, self.alpha, b0,
                                           self.water_level) for v in self.sources]
        self.n_plumes = len(self.plumes)
        k = self.n_plumes
        self.vent_state = np.zeros(k)
        self.vent_w = np.full(k, 1.0 / k)
        self.vent_tau = float(lc["vent_switch_s"])
        # meander: OU history of 2-D offsets, sampled at the step rate
        self.meander_amp = float(pc["meander_amp"])
        self.meander_T = float(pc["meander_time_s"])
        self.hist_len = int(max(p["tau"][-1] for p in self.plumes) / self.dt) + 2
        self.hist = np.zeros((self.hist_len, 2))
        self.hist_head = 0

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
        n = k * N_CL
        self.cl_pos = wp.zeros(n, dtype=wp.vec3, device=dev)
        self.cl_tan = wp.zeros(n, dtype=wp.vec3, device=dev)
        self.cl_u = wp.zeros(n, dtype=float, device=dev)
        self.cl_b = wp.zeros(n, dtype=float, device=dev)
        self.cl_pos_host = np.zeros((n, 3))

    # ---------------------------------------------------------------------------------------- info
    def summary(self) -> dict:
        pl = self.plumes[0]
        z_rel = pl["pos"][:, 2] - pl["pos"][0, 2]
        def at(h):
            return float(np.interp(h, z_rel, pl["u_c"])), float(np.interp(h, z_rel, pl["b"]))
        return {"gas_flow_lpm_at_source": self.q_gas * 60000, "source": self.source_info,
                "buried": bool(self.leak.buried), "plumes": self.n_plumes, "d50_mm": self.d50 * 1e3,
                "mean_bubble_volume_mm3": self.mean_volume * 1e9,
                "bubbles_per_s": self.q_gas / self.mean_volume, "slip_mean_m_s": self.slip_mean,
                "plume_uc_b_at_0.5m": at(0.5), "plume_uc_b_at_1.5m": at(1.5),
                "rise_time_s": float(pl["tau"][-1])}

    # ---------------------------------------------------------------------------------------- step
    def _update_centrelines(self):
        k = self.n_plumes
        if self.leak.buried and k > 1:     # intermittent vents: OU-driven soft switching
            a = math.exp(-self.dt / self.vent_tau)
            self.vent_state = self.vent_state * a + math.sqrt(1 - a * a) * self.rng.normal(size=k)
            w = np.exp(1.8 * self.vent_state)
            self.vent_w = w / w.sum()
        # meander history
        a = math.exp(-self.dt / self.meander_T)
        prev = self.hist[(self.hist_head - 1) % self.hist_len]
        self.hist[self.hist_head] = prev * a + math.sqrt(1 - a * a) * self.rng.normal(size=2)
        self.hist_head = (self.hist_head + 1) % self.hist_len
        pos, tan, uc, bb = [], [], [], []
        for i, pl in enumerate(self.plumes):
            lag = np.minimum((pl["tau"] / self.dt).astype(int), self.hist_len - 1)
            off2 = self.hist[(self.hist_head - 1 - lag) % self.hist_len]
            # the source cannot move: wander grows with the distance travelled
            grow = np.clip(pl["tau"] / max(self.meander_T, 1e-3), 0, 1)
            off = np.concatenate([off2, np.zeros((N_CL, 1))], 1) * (self.meander_amp * pl["b"] * grow)[:, None]
            off -= np.sum(off * pl["tangent"], 1, keepdims=True) * pl["tangent"]
            pos.append(pl["pos"] + off)
            tan.append(pl["tangent"])
            scale = (self.vent_w[i] * k) ** (1 / 3) if self.leak.buried else 1.0
            uc.append(pl["u_c"] * scale)
            bb.append(pl["b"])
        self.cl_pos_host = np.concatenate(pos)
        self.cl_pos.assign(self.cl_pos_host.astype(np.float32))
        self.cl_tan.assign(np.concatenate(tan).astype(np.float32))
        self.cl_u.assign(np.concatenate(uc).astype(np.float32))
        self.cl_b.assign(np.concatenate(bb).astype(np.float32))

    def _emit(self, dt):
        vol = self.q_gas * dt + self.vol_debt
        n = int(max(vol / self.mean_volume, 0))
        if n <= 0:
            self.vol_debt = vol
            return
        n += self.rng.poisson(0.0) if n > 0 else 0
        d = np.exp(np.log(self.d50) + self.sigma_ln * self.rng.normal(size=n))
        d = np.clip(d, *self.d_range)
        self.vol_debt = vol - float(np.sum(np.pi / 6 * d ** 3))
        rad = d / 2
        # back to the release pressure: q_gas is at the source's ambient pressure already
        if not self.leak.buried:
            pl = self.plumes[0]
            s_idx = self.rng.integers(0, 6, size=n)            # first few centreline samples (< ~2 cm)
            c = pl["pos"][s_idx]
            b = pl["b"][s_idx][:, None]
            t = pl["tangent"][s_idx]
            lat = self.rng.normal(size=(n, 3)) * b
            lat -= np.sum(lat * t, 1, keepdims=True) * t
            pos = c + lat
        else:
            which = self.rng.choice(self.n_plumes, size=n, p=self.vent_w)
            rr = float(get(self.cfg, "leak.pockmark_radius")) * 0.3 * np.sqrt(self.rng.uniform(size=n))
            th = self.rng.uniform(0, 2 * np.pi, n)
            pos = self.sources[which] + np.stack([rr * np.cos(th), rr * np.sin(th), rad], 1)
        misc = np.stack([self.rng.uniform(0, 2 * np.pi, n), self.rng.uniform(0, 1, n) ** 2,
                         self.rng.uniform(0, 2 * np.pi, n)], 1)
        n = min(n, self.cap)
        wp.launch(spawn_kernel, dim=n, device=self.device,
                  inputs=[self.head, self.cap,
                          wp.array(pos[:n].astype(np.float32), dtype=wp.vec3, device=self.device),
                          wp.array(rad[:n].astype(np.float32), dtype=float, device=self.device),
                          wp.array(misc[:n].astype(np.float32), dtype=wp.vec3, device=self.device),
                          self.pos, self.r0, self.z0, self.r, self.alive, self.turb, self.misc, self.age])
        self.head = (self.head + n) % self.cap

    def step(self, dt_total: float):
        n_sub = max(1, int(round(dt_total / self.dt)))
        for _ in range(n_sub):
            self._update_centrelines()
            self._emit(self.dt)
            wp.launch(step_kernel, dim=self.cap, device=self.device,
                      inputs=[self.dt, self.t, self.step_count, self.cl_pos, self.cl_tan, self.cl_u, self.cl_b,
                              self.n_plumes, N_CL, self.alpha, self.turb_k, self.wobble,
                              self.rho, self.mu, self.sigma, self.kb, self.p_surf, self.water_level,
                              float(self.size[0]), float(self.size[1]),
                              self.pos, self.r0, self.z0, self.r, self.alive, self.turb, self.misc, self.age,
                              self.vel])
            self.t += self.dt
            self.step_count += 1

    def warmup(self, seconds: float):
        self.step(seconds)

    def snapshot(self) -> dict:
        """Alive bubbles on the host: pos (N,3), radius (N,), vel (N,3)."""
        alive = self.alive.numpy().astype(bool)
        return {"pos": self.pos.numpy()[alive], "radius": self.r.numpy()[alive], "vel": self.vel.numpy()[alive]}
