"""SimulationApp creation (headless by default) and renderer settings. The pattern is copied from ~/isaac_ws
sim/app.py (not imported). Call ``create_app`` before importing any omni module.
"""
from __future__ import annotations

import logging

log = logging.getLogger(__name__)

# /rtx/post/aa/op: the RTX real-time renderer of Kit 107 (Isaac Sim 5.1) knows only 3 = DLSS and 4 = DLAA; DLSS
# renders below the camera resolution and upscales. DLAA is the same network at render scale 1.0: native resolution.
NATIVE_AA_OP = 4
RENDER_KEYS = ["/rtx/rendermode", "/rtx/post/aa/op", "/rtx/translucency/enabled",
               "/rtx/translucency/maxRefractionBounces", "/rtx/reflections/enabled", "/app/useFabricSceneDelegate"]


DLSS_AA_OP = 3


def create_app(gui: bool, width: int = 1280, height: int = 720, refraction_bounces: int = 2, dlss: bool = False,
               translucency: bool = True, reflections: bool = True):
    """The SimulationApp: headless unless ``gui``; RTX real-time ("RaytracedLighting"). ``dlss``: DLSS (render below
    the output resolution and upscale) instead of DLAA, the pool's lever for a slower GPU."""
    from isaacsim import SimulationApp

    app = SimulationApp({
        "headless": not gui,
        "width": width,
        "height": height,
        "renderer": "RaytracedLighting",
        "anti_aliasing": DLSS_AA_OP if dlss else NATIVE_AA_OP,
        "hide_ui": not gui,
    })
    app.update()
    _settings(refraction_bounces, DLSS_AA_OP if dlss else NATIVE_AA_OP, translucency, reflections)
    return app


def _settings(refraction_bounces: int, aa_op: int = NATIVE_AA_OP, translucency: bool = True,
              reflections: bool = True) -> None:
    """Renderer settings. Translucency and reflections are on by default: the bubbles are a refractive dielectric
    (gas in water, total internal reflection at their rims); the pool's linus profile turns reflections off.
    Indirect diffuse and ambient occlusion are off (lamp-lit, dark scene; saves GPU time and VRAM)."""
    import carb

    s = carb.settings.get_settings()
    s.set("/rtx/translucency/enabled", bool(translucency))
    s.set("/rtx/translucency/maxRefractionBounces", int(refraction_bounces))
    s.set("/rtx/reflections/enabled", bool(reflections))
    s.set("/rtx/indirectDiffuse/enabled", False)
    s.set("/rtx/ambientOcclusion/enabled", False)
    s.set("/app/asyncRendering", False)
    s.set("/app/runLoops/main/rateLimitEnabled", False)     # the sim loop paces itself
    for root in ("/rtx", "/rtx-defaults"):
        s.set(f"{root}/post/aa/op", aa_op)


def render_settings_report() -> dict:
    import carb

    s = carb.settings.get_settings()
    return {k: s.get(k) for k in RENDER_KEYS}
