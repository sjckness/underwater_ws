"""GUI-only views (Kit app with --gui): the viewport camera, the sonar overlay and screenshots.

The overlay is drawn with omni.ui.scene over the viewport, in world coordinates: the wireframe of the sonar's field of
view (``fan_polylines``) and the current sonar image as coloured points on the fan's centre plane (the 2-D
image's usual projection: a return at elevation e is drawn at elevation 0). It is UI, not scene geometry, so it never
appears in the camera images or the sonar.
"""
from __future__ import annotations

import logging
import math

import numpy as np

from .config import get

log = logging.getLogger(__name__)
FAN_COLOR = [1.0, 0.85, 0.1, 0.9]
MAX_RETURN_POINTS = 6000


def fan_polylines(son, n: int = 49) -> list[np.ndarray]:
    """Wireframe of the sonar's field of view in the world frame: the far range surface's outline (constant-u edges of
    the linear array at the fan edges, constant-elevation edges at +-fov_v/2), the four corner rays and the centre
    line. Each polyline is (k, 3)."""
    u_max = math.sin(son.theta_max)
    e_max = math.radians(son.fov_v_deg / 2)

    def body(u, e):                     # direction cosine u (+ starboard) and elevation e -> unit vector (x, y, z)
        u, e = np.broadcast_arrays(np.asarray(u, float), np.asarray(e, float))
        x = np.sqrt(np.clip(1 - u ** 2 - np.sin(e) ** 2, 0.0, 1.0))
        return np.stack([x, -u, np.sin(e)], -1)

    def world(d, r):
        return son.position + (r * d) @ son.rot.T

    us, es = np.linspace(-u_max, u_max, n), np.linspace(-e_max, e_max, max(n // 4, 5))
    lines = [world(body(us, e), son.range) for e in (-e_max, e_max)]
    lines += [world(body(u, es), son.range) for u in (-u_max, u_max)]
    lines += [np.stack([son.position, world(body(u, e), son.range)]) for u in (-u_max, u_max) for e in (-e_max, e_max)]
    lines.append(np.stack([son.position, world(body(0.0, 0.0), son.range)]))
    return lines


def views(cfg: dict, son) -> dict:
    """Named viewport cameras: (eye, target, focal length [mm] for the 20.955 mm aperture)."""
    L, W, _ = (float(v) for v in get(cfg, "tank.size"))
    wl = float(get(cfg, "tank.water_level"))
    leak = np.array([float(get(cfg, "pipe.x")), float(get(cfg, "leak.position_along")), 0.3])
    mid = 0.5 * (son.position + leak)
    return {
        # inside the tank, high on the y = 0 side wall, looking 45 deg down across it: the sonar and its fan from the
        # side, the pipe, the plume column (the walls are opaque, so the eye stays inside)
        "side45": (np.array([L - 0.15, 0.15, 0.7 * wl]), np.array([mid[0] + 0.05, mid[1], 1.0]), 12.0),
        "top": (np.array([L / 2, W / 2 - 0.01, wl - 0.05]), np.array([L / 2, W / 2, 0.0]), 8.0),
        "sonar": (son.position - son.rot[:, 0] * 0.02 + son.rot[:, 2] * 0.15, son.position + son.rot[:, 0] * 2.0,
                  12.0),
    }


def set_view(cfg: dict, son, name: str, resolution=None) -> None:
    """Viewport camera pose and focal length; ``resolution``: the viewport's render size, independent of the window
    (a maximised window would otherwise render at screen size)."""
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import UsdGeom

    v = views(cfg, son)
    if name not in v:
        raise ValueError(f"sim.view must be one of {', '.join(v)}, got {name!r}")
    eye, target, focal = v[name]
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
    vp = get_active_viewport()
    UsdGeom.Camera(vp.stage.GetPrimAtPath("/OmniverseKit_Persp")).GetFocalLengthAttr().Set(focal)
    if resolution:
        vp.resolution = tuple(int(x) for x in resolution)


def returns_points(son, image: np.ndarray, threshold: int, max_points: int = MAX_RETURN_POINTS):
    """Polar image pixels >= threshold as world points on the fan's centre plane, with an orange-to-white colour
    ramp; evenly decimated to at most ``max_points``."""
    rows, cols = np.nonzero(image >= threshold)
    step = max(1, int(np.ceil(len(rows) / max_points)))
    rows, cols = rows[::step], cols[::step]
    r = son.range_m[rows]
    b = np.radians(son.bearing[cols])
    body = np.stack([r * np.cos(b), -r * np.sin(b), np.zeros_like(r)], 1)
    pts = son.position + body @ son.rot.T
    v = (image[rows, cols].astype(np.float32) - threshold) / max(255.0 - threshold, 1.0)
    colors = np.stack([np.ones_like(v), 0.35 + 0.65 * v, 0.1 + 0.9 * v ** 2, np.ones_like(v)], 1)
    return pts, colors


class Overlay:
    def __init__(self, cfg: dict, son):
        from omni.kit.viewport.utility import get_active_viewport_window
        from omni.ui import scene as sc

        self.son = son
        self.fan = bool(get(cfg, "sim.overlay_fan"))
        self.returns = bool(get(cfg, "sim.overlay_returns"))
        self.threshold = int(get(cfg, "sim.overlay_returns_min"))
        self.window = get_active_viewport_window()
        with self.window.get_frame("uw_sonar_overlay"):
            self.view = sc.SceneView()
            with self.view.scene:
                if self.fan:
                    for line in fan_polylines(son):
                        sc.Curve(line.tolist(), curve_type=sc.Curve.CurveType.LINEAR, colors=[FAN_COLOR],
                                 thicknesses=[2.0])
                self.points = sc.Points([[0.0, 0.0, 0.0]], colors=[[0.0, 0.0, 0.0, 0.0]], sizes=[1.0], visible=False)
        self.window.viewport_api.add_scene_view(self.view)

    def update(self, image: np.ndarray) -> int:
        """Show this polar image (uint8, range bins x beams); returns the number of points drawn."""
        if not self.returns:
            return 0
        pts, colors = returns_points(self.son, image, self.threshold)
        if not len(pts):
            self.points.visible = False
            return 0
        self.points.positions = pts.tolist()
        self.points.colors = colors.tolist()
        self.points.sizes = [2.5] * len(pts)
        self.points.visible = True
        return len(pts)

    def destroy(self) -> None:
        try:
            self.window.viewport_api.remove_scene_view(self.view)
            self.view.destroy()
        except Exception:  # noqa: BLE001 - the window may be gone already
            pass


def screenshot(path: str) -> None:
    """The whole app window (viewport, overlay and UI) on the next frame."""
    import omni.renderer_capture

    omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(path))


def wait_screenshot(path: str) -> str:
    """Wait for ``screenshot(path)``; also write the viewport's part of it (with the overlay) as *_viewport.png."""
    import omni.renderer_capture
    import omni.ui as ui
    from omni.kit.viewport.utility import get_active_viewport_window
    from PIL import Image

    omni.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    w = get_active_viewport_window()
    wid = w.frame                                      # the window's content: the viewport image
    k = ui.Workspace.get_dpi_scale()
    x0, y0 = wid.screen_position_x, wid.screen_position_y
    box = [int(round(v * k)) for v in (x0, y0, x0 + wid.computed_width, y0 + wid.computed_height)]
    out = path.replace("_gui.png", "_viewport.png")
    with Image.open(path) as im:
        im.crop(box).save(out)
    return out
