"""M11: fetch the BlueROV2 Heavy 3D model and convert it into the USD asset `uw pool` references.

Source: Blue Robotics' own Heavy model from their simulator BlueSim (github.com/bluerobotics/bluesim,
vehicles/bluerovheavy/"bluerov cleaned decimated.gltf"): the vehicle CAD, cleaned and decimated by Blue Robotics,
132 k triangles, in millimetres. Licence GPL-2.0 (the BlueSim repository's), which is why the asset is generated
into assets/ (not in git) instead of being committed. The full-detail STEP of the R4 (cad.bluerobotics.com) is the
optional next step if more detail is ever wanted.

    python -m uw py tools/fetch_bluerov2.py            # in the container: needs pxr (USD) and network once
    python -m uw py tools/fetch_bluerov2.py --gltf F   # from a local copy instead of downloading

The mesh's own axes, found from the geometry (the Lumen lights are at the front, the blue fairings on top) and
consistent with the glTF node's +90 deg rotation about X: +Y forward, -Z up, +X to port. It is moved to the body
frame FLU with the origin at the bounding-box centre plus --cg-offset (the CG is not in the model; the vehicle file
puts the centre of buoyancy 1 cm above it), and scaled to metres. Only the vehicle mesh is kept (the scene's second
node, a Ping360 mount, is dropped). Materials map by name onto UsdPreviewSurface (acrylic onto a clear glass).
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
BLUESIM_COMMIT = "00fa87906fcdfd67805367b1ef2d8643e1e75479"       # bluerobotics/bluesim master, 2026-11-25
GLTF_PATH = "vehicles/bluerovheavy/bluerov cleaned decimated.gltf"
URL = f"https://raw.githubusercontent.com/bluerobotics/bluesim/{BLUESIM_COMMIT}/" + urllib.parse.quote(GLTF_PATH)
OUT = REPO / "assets" / "bluerov2_heavy" / "bluerov2_heavy.usd"
VEHICLE_MESH = "BROV2-HEAVY-ASM"            # the vehicle mesh's name prefix; the Ping360 mount is not kept

# mesh axes -> body FLU, rows are the body axes in mesh coordinates: x_b = +Y, y_b = +X, z_b = -Z (det +1)
R_MESH_TO_BODY = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

# by material name: (base colour linear RGB, metallic, roughness); "glass" for the clear parts
MATERIALS = {
    "black metal": ((0.03, 0.03, 0.03), 0.8, 0.35),
    "blue plastic": ((0.01, 0.20, 0.60), 0.0, 0.45),
    "acrilic": "glass",
    "black plastic": ((0.02, 0.02, 0.02), 0.0, 0.55),
    "red metal": ((0.30, 0.01, 0.02), 0.8, 0.35),
    "lumen acrilic": ((0.85, 0.85, 0.80), 0.0, 0.15),
}


def load_gltf(path: Path):
    g = json.loads(path.read_text())
    if len(g["buffers"]) != 1 or not g["buffers"][0]["uri"].startswith("data:"):
        raise SystemExit(f"{path}: expected one embedded buffer")
    buf = base64.b64decode(g["buffers"][0]["uri"].split(",", 1)[1])

    def acc(i):
        a = g["accessors"][i]
        bv = g["bufferViews"][a["bufferView"]]
        n = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
        dt = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}[a["componentType"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        arr = np.frombuffer(buf, dt, a["count"] * n, off)
        return arr.reshape(-1, n) if n > 1 else arr

    meshes = [m for m in g["meshes"] if m.get("name", "").startswith(VEHICLE_MESH)]
    if len(meshes) != 1:
        names = [m.get("name") for m in g["meshes"]]
        raise SystemExit(f"{path}: expected one mesh named {VEHICLE_MESH}*, found {names}")
    parts = []
    for p in meshes[0]["primitives"]:
        name = g["materials"][p["material"]]["name"]
        parts.append((name, acc(p["attributes"]["POSITION"]).astype(float),
                      acc(p["attributes"]["NORMAL"]).astype(float) if "NORMAL" in p["attributes"] else None,
                      acc(p["indices"]).astype(np.int64).reshape(-1, 3)))
    return parts


def to_body(parts, cg_offset_m):
    pts = np.vstack([p[1] for p in parts])
    centre = 0.5 * (pts.min(0) + pts.max(0))
    out = []
    for name, p, n, tri in parts:
        pb = (R_MESH_TO_BODY @ ((p - centre) / 1000.0).T).T - np.asarray(cg_offset_m, float)
        nb = None if n is None else (R_MESH_TO_BODY @ n.T).T
        out.append((name, pb, nb, tri))
    return out


def write_usd(parts, out: Path, provenance: dict) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    out.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/bluerov2_heavy")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("uw:provenance", json.dumps(provenance))
    UsdGeom.Scope.Define(stage, "/bluerov2_heavy/Looks")
    for i, (name, p, n, tri) in enumerate(parts):
        safe = "".join(c if c.isalnum() else "_" for c in name)
        mesh = UsdGeom.Mesh.Define(stage, f"/bluerov2_heavy/part_{i}_{safe}")
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(p, np.float32)))
        mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(tri), 3, np.int32)))
        mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(np.ascontiguousarray(tri, np.int32).ravel()))
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, p.min(0))), Gf.Vec3f(*map(float, p.max(0)))]))
        if n is not None:
            mesh.CreateNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(n, np.float32)))
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mat = UsdShade.Material.Define(stage, f"/bluerov2_heavy/Looks/{safe}")
        spec = MATERIALS.get(name, ((0.2, 0.2, 0.2), 0.0, 0.5))
        sh = UsdShade.Shader.Define(stage, f"/bluerov2_heavy/Looks/{safe}/Shader")
        if spec == "glass":
            sh.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
            sh.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
            sh.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.49)          # PMMA
            sh.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.97, 0.98, 0.98))
            o = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
            mat.CreateSurfaceOutput("mdl").ConnectToSource(o)
            mat.CreateVolumeOutput("mdl").ConnectToSource(o)
        else:
            color, metallic, rough = spec
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    stage.GetRootLayer().Save()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--gltf", type=Path, help="a local copy of the glTF (default: download it at the pinned commit)")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--cg-offset", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"),
                    help="the CG in the bounding-box-centred body frame FLU [m] (default: the centre; GUESS)")
    args = ap.parse_args(argv)

    src = args.gltf
    if src is None:
        cache = Path.home() / ".cache" / "uw_assets" / f"bluesim-{BLUESIM_COMMIT[:12]}-heavy.gltf"
        if not cache.is_file():
            cache.parent.mkdir(parents=True, exist_ok=True)
            print(f"downloading {URL}", flush=True)
            with urllib.request.urlopen(URL, timeout=120) as r:
                data = r.read()
            cache.with_suffix(".part").write_bytes(data)
            cache.with_suffix(".part").rename(cache)
        src = cache
    sha = hashlib.sha256(src.read_bytes()).hexdigest()
    parts = to_body(load_gltf(src), args.cg_offset)
    pts = np.vstack([p[1] for p in parts])
    ntri = sum(len(p[3]) for p in parts)
    provenance = {"source": "bluerobotics/bluesim", "commit": BLUESIM_COMMIT, "path": GLTF_PATH, "licence": "GPL-2.0",
                  "sha256": sha, "frame": "body FLU, metres, origin = bounding-box centre - cg_offset",
                  "cg_offset_m": list(args.cg_offset)}
    write_usd(parts, args.out, provenance)
    print(f"{args.out}: {ntri} triangles in {len(parts)} parts; extent (FLU, m) "
          f"{np.round(pts.min(0), 3).tolist()} .. {np.round(pts.max(0), 3).tolist()} "
          f"= {np.round(pts.max(0) - pts.min(0), 3).tolist()} (BlueROV2 Heavy: 0.457 x 0.575 x 0.254)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
