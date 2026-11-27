"""Split the read-only real sonar recordings into a calibrate and a hold-out half, leak-free, once.

    python -m uw py physics/tools/real_holdout.py [--threshold 0.95] [--out output/compare_real/holdout.json]

Why this is not a line of shell. The publishers' own split cannot be used: UATD ships `UATD_Test_1` and
`UATD_Test_2`, but they are a random 50/50 draw from one pool, not two sessions. Measured over the 1600
frames, 80 % of Test_1 has a near-twin in Test_2 at cosine 0.95 of mean-removed thumbnails, 34 % above 0.99,
one pair is bit-identical, and all 171 distinct `soundspeed` values split evenly between the two. Calibrating
on one and verifying on the other would be verifying on the training data.

Neither can the split be per frame: a forward-looking sonar on a moving platform returns almost the same
image for seconds at a time, which is the same argument `ml/tools/sweep.py` makes for splitting the simulated
sweep by scenario and never by ping.

So sessions are *derived*. Frames are grouped by single-linkage on the cosine similarity of 32x32
mean-removed thumbnails, which makes the guarantee the hold-out needs structural rather than hoped for: at
threshold T no pair in two different groups can exceed T, so assigning whole groups bounds the similarity
between the two halves by T, and `max_cross_similarity` in the output states the value actually achieved.

Groups are then assigned largest first to whichever half is furthest below its quota, counted both overall
and within the stratum the group belongs to (range setting and frequency for UATD, the recording set for the
DeeperSense dumps), the same largest-remainder idea as `sweep.assign_splits`.

The result is written once and is meant to be read, never regenerated: `compare_real.py --split` refuses a
split that is not in this file, and the per-file sha256 is what detects a supposedly read-only corpus that
moved under us.
"""
from __future__ import annotations

import argparse
import collections
import glob
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS))
from uw import config  # noqa: E402

REAL = Path(os.environ.get("UW_REAL", Path.home() / "underwater_data" / "real"))
THUMB = 32

# name -> (glob relative to REAL, holdout share, note). A share of 0 keeps a corpus wholly for calibration
# or diagnosis; the note says what the corpus may legitimately be used for, and is copied into the output.
DATASETS = {
    "uatd": ("uatd/x/UATD_Test_*/images/*.bmp", 0.40,
             "native polar, Tritech Gemini 1200ik: the only corpus admissible for near-range, ring-down "
             "and level-vs-range, and then as shape only (its own TVG is baked into the 8-bit data)"),
    "basin1": ("deepersense_sonar2rgb/x/basin-dataset-1/sonar/*.jpg", 0.40,
               "Cartesian fan 2896x1448, Gemini 720i: background statistics and beam width only"),
    "basin2": ("deepersense_sonar2rgb/x/basin-dataset-2/sonar/*.png", 0.0,
               "Cartesian fan 512x512 inferno, Oculus M1200d: diagnosis and the broadside stripe only, "
               "never a reference for speckle, ring-down or TVG (past the resolution floor)"),
    "divertrack": ("deepersense_divertrack/x/*/set_*/sonar_*/*.jpg", 0.40,
                   "Cartesian fan JPEG, Oculus M1200d: relative, within-frame measures only"),
}


def thumbnails(files: list[Path]) -> np.ndarray:
    """One unit-norm, mean-removed 32x32 thumbnail per file. Mean removal makes the similarity blind to the
    display gain, which differs between recordings and is not what tells two scenes apart."""
    out = np.empty((len(files), THUMB * THUMB), np.float32)
    for i, f in enumerate(files):
        v = np.asarray(Image.open(f).convert("L").resize((THUMB, THUMB), Image.BILINEAR), np.float32).ravel()
        v -= v.mean()
        n = float(np.linalg.norm(v))
        out[i] = v / n if n > 0 else v
    return out


def components(sig: np.ndarray, threshold: float) -> list[list[int]]:
    """Single-linkage groups of frames more similar than ``threshold``. No pair across two groups exceeds it."""
    sim = sig @ sig.T
    np.fill_diagonal(sim, 0.0)
    adj = sim > threshold
    seen = np.zeros(len(sig), bool)
    groups = []
    for i in range(len(sig)):
        if seen[i]:
            continue
        seen[i] = True
        stack, grp = [i], []
        while stack:
            k = stack.pop()
            grp.append(k)
            for j in np.flatnonzero(adj[k] & ~seen):
                seen[j] = True
                stack.append(j)
        groups.append(sorted(grp))
    return sorted(groups, key=len, reverse=True)


def uatd_strata(files: list[Path]) -> list[str]:
    """Range setting rounded to the nearest metre and frequency, from each frame's XML."""
    out = []
    for f in files:
        x = f.parent.parent / "annotations" / (f.stem + ".xml")
        if not x.is_file():
            out.append("unknown")
            continue
        son = ET.parse(x).getroot().find("sonar")
        out.append(f"{round(float(son.find('range').text))}m_{son.find('frequency').text}")
    return out


def path_strata(files: list[Path]) -> list[str]:
    """The recording directory, which is the only session hint the DeeperSense dumps carry."""
    return [f.parent.name for f in files]


def assign(groups: list[list[int]], strata: list[str], share: float) -> list[str]:
    """Whole groups to 'calibrate' or 'holdout', largest first, each to whichever half is furthest below its
    quota overall and within the group's own stratum (a group spanning strata is counted at its majority one)."""
    split = ["calibrate"] * sum(len(g) for g in groups)
    if share <= 0:
        return split
    want = {"holdout": share, "calibrate": 1.0 - share}
    have = collections.Counter()
    per = collections.defaultdict(collections.Counter)
    total = 0
    for g in groups:
        key = collections.Counter(strata[i] for i in g).most_common(1)[0][0]
        total += len(g)

        def debt(half: str) -> float:
            overall = want[half] * total - have[half]
            local = want[half] * (per[key]["calibrate"] + per[key]["holdout"] + len(g)) - per[key][half]
            return overall + 2.0 * local            # the stratum matters more than the global count

        half = max(want, key=debt)
        have[half] += len(g)
        per[key][half] += len(g)
        for i in g:
            split[i] = half
    return split


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def build(name: str, pattern: str, share: float, note: str, threshold: float, digest: bool) -> dict:
    files = sorted(Path(p) for p in glob.glob(str(REAL / pattern)))
    if not files:
        raise SystemExit(f"{name}: no files match {REAL / pattern}")
    print(f"{name}: {len(files)} frames", flush=True)
    sig = thumbnails(files)
    groups = components(sig, threshold)
    strata = uatd_strata(files) if name == "uatd" else path_strata(files)
    split = assign(groups, strata, share)

    cal = np.array([s == "calibrate" for s in split])
    cross = float((sig[cal] @ sig[~cal].T).max()) if cal.any() and (~cal).any() else None
    counts = collections.Counter(split)

    # A corpus can refuse to be split, and that is a finding, not a failure to report. When every frame
    # chains into one group the recording is one continuous near-identical scene: there is no subset of it
    # that is independent of the rest, so it can calibrate but it cannot verify. Saying so here is the whole
    # point -- an empty 'holdout' half that looked like a configuration slip would eventually be "fixed" by
    # splitting per frame, which is precisely the leak this tool exists to prevent.
    refused = None
    if share > 0 and not counts["holdout"]:
        refused = (f"{len(groups)} similarity group(s) at threshold {threshold}: the frames chain into one "
                   f"continuous recording, so no subset of it is independent of the rest. Calibration only.")
        print(f"  HOLD-OUT REFUSED: {refused}", flush=True)
    print(f"  {len(groups)} groups (largest {len(groups[0])}), "
          f"calibrate {counts['calibrate']}, holdout {counts['holdout']}, "
          f"max cross-split similarity {cross if cross is None else round(cross, 4)}", flush=True)
    return {
        "role": "calibrate only" if refused or share <= 0 else "calibrate and hold-out",
        "holdout_refused": refused, "holdout_share_requested": share,
        "root": str(REAL), "pattern": pattern, "note": note, "frames": len(files),
        "group_threshold": threshold, "groups": len(groups), "largest_group": len(groups[0]),
        "max_cross_similarity": cross, "counts": dict(counts),
        "strata": {k: dict(v) for k, v in sorted(
            ((s, collections.Counter(split[i] for i, t in enumerate(strata) if t == s))
             for s in sorted(set(strata))), key=lambda kv: kv[0])},
        "split": {half: [str(f.relative_to(REAL)) for f, s in zip(files, split) if s == half]
                  for half in ("calibrate", "holdout")},
        "sha256": {str(f.relative_to(REAL)): sha256(f) for f in files} if digest else {},
    }


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="python -m uw py physics/tools/real_holdout.py",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--threshold", type=float, default=0.95,
                    help="group frames more similar than this; it is also the bound on cross-split "
                         "similarity that the result guarantees (default: %(default)s)")
    ap.add_argument("--out", type=Path, default=Path(config.WS) / "output" / "compare_real" / "holdout.json")
    ap.add_argument("--no-digest", action="store_true", help="skip the per-file sha256 (faster)")
    ap.add_argument("--only", action="append", choices=sorted(DATASETS), help="one dataset; repeatable")
    args = ap.parse_args(argv)

    if args.out.exists():
        raise SystemExit(f"{args.out} exists: the split is written once and then read. Move it aside "
                         f"deliberately if it really has to be rebuilt.")
    names = args.only or sorted(DATASETS)
    out = {"threshold": args.threshold, "thumbnail": THUMB,
           "why": "UATD_Test_1/_2 are a random split of one pool (80 % of Test_1 has a near-twin in Test_2 "
                  "at cosine 0.95), so sessions are derived by single-linkage grouping and whole groups are "
                  "assigned; see this tool's docstring.",
           "datasets": {n: build(n, *DATASETS[n], args.threshold, not args.no_digest) for n in names}}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1) + "\n")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
