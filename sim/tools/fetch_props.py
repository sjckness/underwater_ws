"""M12 P4: fetch the pool's COCO objects (sim/config/pool/objects.yaml) and turn each into the USD `uw pool` references.

    python -m uw py sim/tools/fetch_props.py                 # every object with a source; in the container (Kit, pxr)
    python -m uw py sim/tools/fetch_props.py chair ball      # only these
    python -m uw py sim/tools/fetch_props.py --list          # what is fetched, with licences (no Kit needed)

For each object: download the pinned file into assets/props/_downloads/<id>/ and check its sha256; convert it to
USD with Kit's asset converter (glTF/GLB/OBJ/FBX; a USD is taken as it is); then write assets/props/<id>/<id>.usd,
a small wrapper whose default prim puts the model in the pool's convention:

* metres, Z up (``source.up``: the model's up axis, Y for glTF) and ``source.rotate_deg`` to turn it so that its
  length runs along +x;
* scaled uniformly so that its largest dimension matches the largest of the catalogue's ``size_m``;
* the origin at the bottom centre of its bounding box, so that a placement's ``z`` is where it rests.

assets/ is not in git (third-party licences, size): assets/props/MANIFEST.json records what each file is (URL,
sha256, licence, scale, final extent) and assets/props/ATTRIBUTION.md the credits CC-BY asks for. Without a model,
`uw pool` draws the catalogue's box or sphere in its place, so nothing here is needed to run the pool.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from uw import config  # noqa: E402
from physics.pool_geometry import object_catalog  # noqa: E402

OUT = REPO / "assets" / "props"
CACHE = OUT / "_downloads"               # inside assets/ (gitignored): the container sees it as the host does
USD_EXT = (".usd", ".usda", ".usdc", ".usdz")


def _fetch(url: str, path: Path, sha256: str | None, what: str) -> None:
    """One file into the cache (once), checked against its pinned sha256."""
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"  downloading {url}", flush=True)
        req = urllib.request.Request(url, headers={"User-Agent": "uw-fetch-props/1"})
        with urllib.request.urlopen(req, timeout=300) as r:
            data = r.read()
        tmp = path.with_suffix(path.suffix + ".part")
        tmp.write_bytes(data)
        tmp.rename(path)
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if not sha256:
        raise SystemExit(f"{what}: no sha256 pinned in sim/config/pool/objects.yaml; the file has {sha}: check it, pin it")
    if sha != sha256:
        raise SystemExit(f"{what}: sha256 {sha} != the pinned {sha256} ({path}): the source changed, or a bad download")


def download(oid: str, src: dict, cache: Path = CACHE) -> Path:
    """The pinned file (and the files it needs, ``source.include``: relative path -> {url, sha256}, e.g. a glTF's
    .bin and textures), from the cache or the network. Returns the main file."""
    url = str(src["url"])
    name = src.get("filename") or url.split("?")[0].rstrip("/").split("/")[-1]
    path = cache / oid / name
    _fetch(url, path, src.get("sha256"), oid)
    for rel, inc in (src.get("include") or {}).items():
        _fetch(str(inc["url"]), cache / oid / rel, inc.get("sha256"), f"{oid}/{rel}")
    return path


def convert(app, src_path: Path, out_usd: Path) -> None:
    """Kit's asset converter, materials as UsdPreviewSurface, textures written next to the USD."""
    import asyncio

    import omni.kit.asset_converter as converter

    ctx = converter.AssetConverterContext()
    ctx.ignore_animations = True
    ctx.ignore_camera = True
    ctx.ignore_light = True
    ctx.single_mesh = False
    ctx.smooth_normals = True
    ctx.export_preview_surface = True
    ctx.use_meter_as_world_unit = True
    ctx.embed_textures = False
    out_usd.parent.mkdir(parents=True, exist_ok=True)
    task = converter.get_instance().create_converter_task(str(src_path), str(out_usd), None, ctx)
    fut = asyncio.ensure_future(task.wait_until_finished())
    while not fut.done():
        app.update()
    if not fut.result():
        raise SystemExit(f"asset converter failed on {src_path}: {task.get_error_message()}")


def wrap(oid: str, model_usd: Path, entry: dict, out_usd: Path) -> dict:
    """The wrapper USD: the model under /<oid>/place/model, turned, scaled and moved (by /<oid>/place) into the
    pool's convention."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom

    src = entry["source"]
    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{oid}")
    stage.SetDefaultPrim(root.GetPrim())
    place = UsdGeom.Xform.Define(stage, f"/{oid}/place")         # our transform; the model keeps its own ops
    model = UsdGeom.Xform.Define(stage, f"/{oid}/place/model")
    model.GetPrim().GetReferences().AddReference(str(model_usd.relative_to(out_usd.parent)))
    xf = UsdGeom.Xformable(place)
    t_op = xf.AddTranslateOp()
    s_op = xf.AddScaleOp()
    r_up = xf.AddRotateXYZOp(opSuffix="up")
    r_al = xf.AddRotateXYZOp(opSuffix="align")
    up = str(src.get("up", "Y")).upper()
    r_up.Set(Gf.Vec3f(90.0, 0.0, 0.0) if up == "Y" else Gf.Vec3f(0.0, 0.0, 0.0))   # Y up -> Z up
    r_al.Set(Gf.Vec3f(*map(float, src.get("rotate_deg", (0.0, 0.0, 0.0)))))
    # the ops apply right to left: align (in the model's own axes), up-fix, scale, translate
    xf.SetXformOpOrder([t_op, s_op, r_up, r_al])
    s_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
    t_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], True)
    box = cache.ComputeWorldBound(model.GetPrim()).ComputeAlignedRange()
    if box.IsEmpty():
        raise SystemExit(f"{oid}: the converted model has no geometry ({model_usd})")
    ext = np.array(box.GetSize(), float)
    want = np.array(entry["size_m"], float)
    k = int(np.argmax(want))
    scale = float(want[k] / ext[k]) * float(src.get("scale_fix", 1.0))
    s_op.Set(Gf.Vec3f(scale, scale, scale))
    cache.Clear()
    box = cache.ComputeWorldBound(model.GetPrim()).ComputeAlignedRange()
    lo, hi = np.array(box.GetMin(), float), np.array(box.GetMax(), float)
    t_op.Set(Gf.Vec3d(float(-(lo[0] + hi[0]) / 2), float(-(lo[1] + hi[1]) / 2), float(-lo[2])))
    opaque = _force_opaque(stage, root.GetPrim(), src.get("keep_alpha", ("cornea", "eyelash", "eye_")))
    root.GetPrim().SetCustomDataByKey("uw:provenance", json.dumps({k: src.get(k) for k in (
        "url", "sha256", "license", "author", "title", "page")}))
    stage.GetRootLayer().Save()
    final = hi - lo
    return {"scale": scale, "extent_m": [round(float(v), 4) for v in final], "size_m": list(map(float, want)),
            "fit_axis": "xyz"[k], "forced_opaque": opaque}


def _force_opaque(stage, root, keep) -> list:
    """glTF materials marked BLEND (a skin, a basket's weave) come out of the converter with a texture-driven
    opacity, and Isaac renders them as glass: the bicycle all but vanished. In the wrapper's own layer, opacity is
    disconnected and set to 1 on every UsdPreviewSurface, except materials whose name contains one of ``keep``."""
    from pxr import Sdf, Usd, UsdShade
    changed = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(prim)
        if sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        mat_name = prim.GetParent().GetName().lower()
        if any(k in mat_name for k in keep):
            continue
        inp = sh.GetInput("opacity")
        if inp and (inp.HasConnectedSource() or (inp.Get() is not None and inp.Get() < 1.0)):
            attr = inp.GetAttr()
            attr.SetConnections([])
            attr.Set(1.0)
            th = sh.GetInput("opacityThreshold") or sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float)
            th.Set(0.0)
            changed.append(prim.GetParent().GetName())
    return changed


def attribution(manifest: dict) -> str:
    lines = ["# Third-party 3-D models in assets/props (sim/tools/fetch_props.py)", ""]
    for oid, m in sorted(manifest.items()):
        lines.append(f"- **{oid}** ({m['coco']}): \"{m.get('title') or oid}\" by {m.get('author') or 'unknown'}, "
                     f"{m.get('license')}, {m.get('page') or m.get('url')}. Converted to USD, scaled and re-centred.")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("ids", nargs="*", help="objects to fetch (default: every one with a source)")
    ap.add_argument("--force", action="store_true", help="convert again even if the USD exists")
    ap.add_argument("--list", action="store_true", help="show the catalogue and what is fetched, then exit")
    args = ap.parse_args(argv)

    cfg = config.load(config.POOL_CONFIG)
    cat = object_catalog(cfg)
    manifest_path = OUT / "MANIFEST.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    if args.list:
        for oid, o in cat.items():
            src = o.get("source") or {}
            have = (OUT / oid / f"{oid}.usd").is_file()
            print(f"{oid:10s} {o['coco']:12s} {'fetched' if have else '-':8s} {src.get('license', 'no source'):14s} "
                  f"{src.get('title') or ''} {src.get('author') or ''}")
        return 0
    todo = args.ids or [k for k, o in cat.items() if o.get("source")]
    unknown = [i for i in todo if i not in cat]
    if unknown:
        raise SystemExit(f"unknown objects {unknown} (catalogue: {sorted(cat)})")
    app = None
    for oid in todo:
        entry = cat[oid]
        src = entry.get("source")
        if not src:
            print(f"{oid}: no source in sim/config/pool/objects.yaml, skipped")
            continue
        out_usd = OUT / oid / f"{oid}.usd"
        if out_usd.is_file() and not args.force:
            print(f"{oid}: {out_usd} exists (--force to redo)")
            continue
        print(f"{oid} ({entry['coco']}):", flush=True)
        path = download(oid, src)
        if path.suffix.lower() in USD_EXT:                 # copied next to the wrapper, so assets/ is complete
            import shutil
            model = OUT / oid / "model" / path.name
            model.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, model)
        else:
            if app is None:
                from sim.app import create_app
                app = create_app(gui=False)
                from isaacsim.core.utils.extensions import enable_extension
                enable_extension("omni.kit.asset_converter")
                app.update()
            model = OUT / oid / "model" / f"{oid}_model.usd"
            convert(app, path, model)
        info = wrap(oid, model, entry, out_usd)
        manifest[oid] = {"coco": entry["coco"], **{k: src.get(k) for k in ("url", "sha256", "license", "author",
                                                                         "title", "page")}, **info}
        print(f"  -> {out_usd}: scale {info['scale']:.4g}, extent {info['extent_m']} m "
              f"(catalogue {info['size_m']}, fitted on {info['fit_axis']}); made opaque: {info['forced_opaque']}",
              flush=True)
        OUT.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(json.dumps(manifest, indent=1))
        (OUT / "ATTRIBUTION.md").write_text(attribution(manifest))
    if app is not None:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
