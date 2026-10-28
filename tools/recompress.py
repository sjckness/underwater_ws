"""Recompress a finished dataset in place: deflate the npz and the PNGs a sweep wrote uncompressed.

    python3 tools/recompress.py --root /HDD2T/underwater_data/datasets/v0.2 [--level 6] [--dry-run]   (host)
    python3 tools/recompress.py status --root DIR

A sweep runs with ``recorder.compress=false`` and ``recorder.png_level=0`` to keep zlib off the worker's critical
path (M5: 63 ms of a ~100 ms write). That costs about 3.5x the disk, so the compression is done afterwards, here,
as a CPU-only pass. v0.2 measured 216.6 -> 62.4 GB; see docs/validation.md "Recompressing v0.2".

Lossless by construction and verified per scenario before anything is replaced: every ping is re-encoded into
<root>/.recompress/<id>, then read back **through the dataset loader** (`uw.dataset.load_ping`) and compared with
the original, array by array and bit for bit, plus a raw comparison of each npz array and each PNG. Only if the
whole scenario matches are its files moved over the originals (one ``os.replace`` each, so an interrupted run
leaves valid files either way) and a RECOMPRESSED marker written.

Resumable: a scenario with RECOMPRESSED is skipped, anything else is redone from scratch. Idempotent, so a
half-replaced scenario is simply redone. It pauses while an f1tenth run is active, while <root>/PAUSE exists, while
a GPU job holds an open file under the dataset's disk, or while that disk or / runs low, and resumes after.

PNGs go through ``uw.sinks.write_png``, not Pillow: it writes the single-IDAT, filter-0 form that ``read_png``
decodes on its fast path, which is what the loader uses during training. Pillow's row filters would save another
13 % and cost that fast path.

The sweep's manifest.json is left untouched (it records what the sweep produced, including each scenario's
``bytes``); the new sizes go to <root>/recompress.json.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.sweep import f1tenth_active                                                    # noqa: E402
from uw.dataset import load_ping                                                           # noqa: E402
from uw.sinks import read_png, write_png                                                   # noqa: E402

MIN_FREE_ROOT_GB = 5.0        # the pass frees space, but never run the disk dry
MIN_FREE_SYSTEM_GB = 5.0
SUFFIXES = ("", "_mask_all", "_mask_direct", "_mask_geom")


def gpu_writing(root: Path) -> str:
    """Why to wait ('' if not): a process with a CUDA context holding an open file under the dataset's disk."""
    try:
        out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return ""
    dev = os.stat(root).st_dev
    me = os.getpid()
    for line in out.split():
        if not line.strip().isdigit() or int(line) == me:
            continue
        fd = Path(f"/proc/{line.strip()}/fd")
        try:
            for f in fd.iterdir():
                try:
                    if os.stat(f).st_dev == dev:
                        return f"GPU process {line.strip()} has a file open on {root}'s disk"
                except OSError:
                    continue
        except OSError:                      # not ours, or gone: nothing to see
            continue
    return ""


def wait_if_busy(root: Path) -> None:
    paused = ""
    while True:
        why = f1tenth_active() or ("PAUSE file" if (root / "PAUSE").exists() else "") or gpu_writing(root)
        free = shutil.disk_usage(root).free / 2 ** 30
        if not why and free < MIN_FREE_ROOT_GB:
            why = f"only {free:.0f} GB free on {root}"
        free_sys = shutil.disk_usage("/").free / 2 ** 30
        if not why and free_sys < MIN_FREE_SYSTEM_GB:
            why = f"only {free_sys:.0f} GB free on /"
        if not why:
            if paused:
                print(f"{time.strftime('%H:%M:%S')} resume", flush=True)
            return
        if why != paused:
            print(f"{time.strftime('%H:%M:%S')} pause: {why}", flush=True)
        paused = why
        time.sleep(30)


def _same(a, b) -> bool:
    """Bit-for-bit, NaN included (so float16 NaN in the blind zone must stay NaN in the same cells)."""
    a, b = np.asanyarray(a), np.asanyarray(b)
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def _same_item(a, b) -> bool:
    """Two `load_ping` results equal: the arrays bit for bit, the targets and the metadata by value."""
    if set(a) != set(b):
        return False
    for k in ("image", "masks", "shares"):
        if not _same(a[k], b[k]):
            return False
    if a["meta"] != b["meta"] or set(a["targets"]) != set(b["targets"]):
        return False
    return all(_same(np.float64(a["targets"][k]), np.float64(b["targets"][k])) for k in a["targets"])


def recompress_scenario(d: Path, tmp: Path, level: int) -> tuple[int, int, int]:
    """Re-encode, verify, replace. Returns (bytes before, bytes after, pings)."""
    stems = sorted(p.stem for p in d.glob("ping_*.npz"))
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    before = after = 0

    for stem in stems:
        with np.load(d / f"{stem}.npz") as z:
            arrays = {k: z[k] for k in z.files}
        before += (d / f"{stem}.npz").stat().st_size
        np.savez_compressed(tmp / f"{stem}.npz", **arrays)
        for suf in SUFFIXES:
            f = d / f"{stem}{suf}.png"
            if f.exists():
                before += f.stat().st_size
                write_png(tmp / f"{stem}{suf}.png", read_png(f), level)
        os.link(d / f"{stem}.json", tmp / f"{stem}.json")              # the loader needs it; never rewritten
        before += (d / f"{stem}.json").stat().st_size

    for stem in stems:                                                 # verify the whole scenario first
        with np.load(d / f"{stem}.npz") as a, np.load(tmp / f"{stem}.npz") as b:
            if set(a.files) != set(b.files) or not all(_same(a[k], b[k]) for k in a.files):
                raise AssertionError(f"{d.name}/{stem}: npz arrays differ")
        for suf in SUFFIXES:
            if (d / f"{stem}{suf}.png").exists() and not _same(read_png(d / f"{stem}{suf}.png"),
                                                               read_png(tmp / f"{stem}{suf}.png")):
                raise AssertionError(f"{d.name}/{stem}{suf}.png: pixels differ")
        if not _same_item(load_ping(d, stem), load_ping(tmp, stem)):
            raise AssertionError(f"{d.name}/{stem}: the loader reads the recompressed ping differently")

    for stem in stems:                                                 # only now touch the dataset
        os.replace(tmp / f"{stem}.npz", d / f"{stem}.npz")
        for suf in SUFFIXES:
            if (tmp / f"{stem}{suf}.png").exists():
                os.replace(tmp / f"{stem}{suf}.png", d / f"{stem}{suf}.png")
    for stem in stems:
        after += (d / f"{stem}.npz").stat().st_size + (d / f"{stem}.json").stat().st_size
        after += sum((d / f"{stem}{suf}.png").stat().st_size for suf in SUFFIXES if (d / f"{stem}{suf}.png").exists())
    shutil.rmtree(tmp)
    return before, after, len(stems)


def scenarios(root: Path) -> list:
    manifest = json.loads((root / "manifest.json").read_text())
    return [s["id"] for s in manifest["scenarios"] if s.get("status") == "done"]


def status(root: Path) -> dict:
    ids = scenarios(root)
    done = [i for i in ids if (root / i / "RECOMPRESSED").exists()]
    before = after = 0
    for i in done:
        m = json.loads((root / i / "RECOMPRESSED").read_text())
        before += m["bytes_before"]
        after += m["bytes_after"]
    return dict(total=len(ids), done=len(done), bytes_before=before, bytes_after=after)


def run(root: Path, level: int, dry: bool, limit: int = 0) -> int:
    ids = scenarios(root)
    todo = [i for i in ids if not (root / i / "RECOMPRESSED").exists()]
    print(f"{len(ids)} scenarios, {len(ids) - len(todo)} already recompressed, {len(todo)} to do", flush=True)
    if limit:
        todo = todo[:limit]
        print(f"--limit {limit}: stopping after {len(todo)}", flush=True)
    if dry:
        return 0
    t0 = time.time()
    st = status(root)
    before, after, pings = st["bytes_before"], st["bytes_after"], 0
    for n, sid in enumerate(todo, 1):
        wait_if_busy(root)
        d = root / sid
        try:
            b, a, p = recompress_scenario(d, root / ".recompress" / sid, level)
        except Exception as e:                                          # leave the scenario as it was and go on
            print(f"{time.strftime('%H:%M:%S')} {sid} FAILED: {type(e).__name__}: {e}", flush=True)
            (d / "RECOMPRESS_FAILED").write_text(f"{type(e).__name__}: {e}\n")
            continue
        (d / "RECOMPRESSED").write_text(json.dumps(dict(bytes_before=b, bytes_after=a, pings=p, level=level,
                                                        at=time.strftime("%Y-%m-%dT%H:%M:%S")), indent=1))
        before, after, pings = before + b, after + a, pings + p
        el = time.time() - t0
        print(f"{time.strftime('%H:%M:%S')} {n}/{len(todo)} {sid} {b / 1e6:.0f} -> {a / 1e6:.0f} MB "
              f"({b / a:.2f}x), total {before / 1e9:.1f} -> {after / 1e9:.1f} GB, "
              f"ETA {el / n * (len(todo) - n) / 3600:.2f} h", flush=True)
    st = status(root)
    rep = dict(level=level, scenarios=st["done"], total=st["total"], bytes_before=st["bytes_before"],
               bytes_after=st["bytes_after"],
               ratio=round(st["bytes_before"] / max(st["bytes_after"], 1), 3),
               finished=time.strftime("%Y-%m-%dT%H:%M:%S"))
    (root / "recompress.json").write_text(json.dumps(rep, indent=1))
    print(json.dumps(rep), flush=True)
    failed = [i for i in ids if (root / i / "RECOMPRESS_FAILED").exists()]
    if failed:
        print(f"FAILED: {len(failed)} scenarios: {failed[:10]}", flush=True)
    return 1 if failed else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", nargs="?", default="run", choices=("run", "status"))
    ap.add_argument("--root", required=True, help="dataset root (the directory with manifest.json)")
    ap.add_argument("--level", type=int, default=6, help="zlib level for the npz and the PNGs (default 6)")
    ap.add_argument("--dry-run", action="store_true", help="say what would be done and stop")
    ap.add_argument("--limit", type=int, default=0, help="stop after this many scenarios (0: all)")
    a = ap.parse_args()
    root = Path(a.root)
    if a.cmd == "status":
        st = status(root)
        print(json.dumps(st, indent=1))
        return 0
    return run(root, a.level, a.dry_run, a.limit)


if __name__ == "__main__":
    sys.exit(main())
