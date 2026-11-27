"""The underwater look of the rendered camera image (M4's ``rtx_post``, first used by the M11 pool).

RTX renders the scene as if in clear air (no participating medium: a volume would cost far more than the pool's
budget). Water is applied afterwards, per pixel, from the distance d [m] the ray travelled to the first surface it
hit (the ``distance_to_camera`` annotator):

    I = J exp(-beta_d d) + b_inf (1 - exp(-beta_b d))          per colour channel, in linear RGB

J is the rendered radiance, the first term the direct signal attenuated along the line of sight, the second the
veiling light scattered into it (the "revised" image formation model of Akkaynak & Treibitz 2018, with separate
coefficients for the two terms). A miss (no hit, d = inf) gets the veiling light alone. The path from a light to the
surface is not attenuated, so a lamp's pool of light is brighter than in real water at range: the honest limit of a
post-process. Coefficients: sim/config/pool/pool.yaml water.*.

Turbidity (``water.turbidity_per_m``, presets ``turbid`` / ``murky``): suspended particles add a scattering
coefficient b_p, the same in every channel (particles much larger than the wavelength scatter all colours alike).
In the model's terms, with B_inf = E b / c per channel (E the ambient light in the water, b the scattering, c the
attenuation, taken as beta_b), particles that only scatter give

    beta_d' = beta_d + b_p        beta_b' = beta_b + b_p        B_inf' = (beta_b B_inf + b_p E) / (beta_b + b_p)

so the far field turns from the clear water's blue towards the colour of the lit particle cloud, E =
``water.turbidity_veil``, while everything fades sooner. What it does not do: the ROV's own lamps light the particles
right in front of the camera (backscatter cones, the dominant look of turbid water at night), and that needs the
light's path, which a post-process does not have.

``apply_numpy`` is the reference; ``WaterPost`` runs the same arithmetic as a Warp kernel on the GPU arrays the
annotators return, and hands back one host RGB frame for the encoder.
"""
from __future__ import annotations

import numpy as np

from uw.config import ConfigError, get


def coefficients(cfg: dict):
    """(beta_d, beta_b, b_inf * gain) as float32 RGB triples, turbidity included."""
    bd = np.asarray(get(cfg, "water.beta_d"), np.float64)
    bb = np.asarray(get(cfg, "water.beta_b"), np.float64)
    binf = np.asarray(get(cfg, "water.b_inf"), np.float64)
    tau = float(get(cfg, "water.turbidity_per_m"))
    if not tau >= 0.0:
        raise ConfigError(f"water.turbidity_per_m must be >= 0, got {tau}")
    if tau > 0.0:
        veil = np.asarray(get(cfg, "water.turbidity_veil"), np.float64)
        binf = (bb * binf + tau * veil) / (bb + tau)
        bd, bb = bd + tau, bb + tau
    binf = binf * float(get(cfg, "water.b_inf_gain"))
    return bd.astype(np.float32), bb.astype(np.float32), binf.astype(np.float32)


def visibility_m(cfg: dict) -> float:
    """How far a dark target stays visible in green, the channel that carries furthest: where its contrast with
    the veiling light falls to 2 % (Duntley), ln(50) / beta_d."""
    bd, _, _ = coefficients(cfg)
    return float(np.log(50.0) / float(bd[1]))


def _to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def _to_srgb(x):
    x = np.clip(x, 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


def apply_numpy(rgb: np.ndarray, distance: np.ndarray, beta_d, beta_b, b_inf) -> np.ndarray:
    """Reference implementation. ``rgb`` uint8 (H, W, 3|4) sRGB, ``distance`` (H, W) [m], inf for a miss.
    Returns uint8 (H, W, 3) sRGB."""
    j = _to_linear(rgb[..., :3].astype(np.float64) / 255.0)
    d = np.asarray(distance, np.float64).reshape(rgb.shape[:2])
    miss = ~np.isfinite(d) | (d <= 0.0)
    d = np.where(miss, 1e6, d)[..., None]
    direct = np.where(miss[..., None], 0.0, j * np.exp(-np.asarray(beta_d, np.float64) * d))
    veil = np.asarray(b_inf, np.float64) * (1.0 - np.exp(-np.asarray(beta_b, np.float64) * d))
    return np.round(_to_srgb(direct + veil) * 255.0).astype(np.uint8)


class WaterPost:
    """The Warp version: ``__call__(rgb, distance)`` with Warp (or numpy) arrays, returns host uint8 (H, W, 3)."""

    def __init__(self, cfg: dict, device: str = "cuda:0"):
        import warp as wp

        self.wp = wp
        self.device = device
        bd, bb, binf = coefficients(cfg)
        self.bd, self.bb, self.binf = (wp.vec3(*map(float, v)) for v in (bd, bb, binf))
        self.out = None
        self.kernel = _kernel()

    def __call__(self, rgb, distance) -> np.ndarray:
        wp = self.wp
        if not isinstance(rgb, wp.array):
            rgb = wp.array(np.ascontiguousarray(rgb), dtype=wp.uint8, device=self.device)
        if not isinstance(distance, wp.array):
            distance = wp.array(np.ascontiguousarray(distance, np.float32), dtype=wp.float32, device=self.device)
        h, w = int(rgb.shape[0]), int(rgb.shape[1])
        ch = int(rgb.shape[2])
        rgb = rgb.reshape((h * w * ch,))
        distance = distance.reshape((h * w,))
        if self.out is None or self.out.shape[0] != h * w * 3:
            self.out = wp.empty(h * w * 3, dtype=wp.uint8, device=self.device)
        wp.launch(self.kernel, dim=h * w, inputs=[rgb, distance, ch, self.bd, self.bb, self.binf, self.out],
                  device=self.device)
        return self.out.numpy().reshape(h, w, 3)


_KERNEL = None


def _kernel():
    global _KERNEL
    if _KERNEL is not None:
        return _KERNEL
    import warp as wp

    @wp.func
    def lin(c: float):
        if c <= 0.04045:
            return c / 12.92
        return wp.pow((c + 0.055) / 1.055, 2.4)

    @wp.func
    def srgb(x: float):
        x = wp.clamp(x, 0.0, 1.0)
        if x <= 0.0031308:
            return 12.92 * x
        return 1.055 * wp.pow(x, 1.0 / 2.4) - 0.055

    @wp.kernel
    def water(rgb: wp.array(dtype=wp.uint8), dist: wp.array(dtype=wp.float32), ch: int, bd: wp.vec3, bb: wp.vec3,
              binf: wp.vec3, out: wp.array(dtype=wp.uint8)):
        i = wp.tid()
        d = dist[i]
        miss = not wp.isfinite(d) or d <= 0.0
        if miss:
            d = 1.0e6
        for c in range(3):
            j = lin(float(rgb[i * ch + c]) / 255.0)
            direct = j * wp.exp(-bd[c] * d)
            if miss:
                direct = 0.0
            v = direct + binf[c] * (1.0 - wp.exp(-bb[c] * d))
            out[i * 3 + c] = wp.uint8(wp.int32(wp.round(srgb(v) * 255.0)))

    _KERNEL = water
    return water
