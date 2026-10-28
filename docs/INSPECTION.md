# Inspecting v0.2 before converting it to YOLO-seg / COCO

What `tools/sonar_to_yolo.py` is converting, what it had to decide, and what a reader should know before
training on the result. Tags follow the rest of `docs/`: **V** verified here, **U** unverified.

> **This is a simulated dataset.** Every ping in v0.2 came out of the Isaac Sim / Warp forward-looking-sonar
> model in this repo, not out of a sonar head. There is no real water, no real gas plume and no human
> annotation anywhere in it; the labels are the simulator's own ground truth, exact by construction. A model
> trained on it has seen a particular scattering model, a particular speckle model and a particular set of
> four geometry presets. **No sim-to-real claim is made or tested.** The nearest thing to a reality check in
> this repo is `tools/compare_real.py` against the DeeperSense recordings and the background-contrast work in
> `docs/validation.md`; the gap between this dataset and an Oculus M750d in a real tank is open and
> unmeasured. Treat any score on `val`/`test` here as a score on the simulator.

## 1. Where it lives

| | |
|---|---|
| source dataset | `/HDD2T/underwater_data/datasets/v0.2` (400 scenario directories + `manifest.json`) |
| HDD2 | `/dev/sda1` → `/HDD2T`, ST2000DM008 7200 rpm, 1.8 T, ~1.5 T free (**V**) |
| SSD | `/dev/sdb2` → `/`, 439 G, **39 G free (91 % full)** (**V**) |
| output | `/HDD2T/underwater_data/yolo/v0.2`, ~18 GB |

The output goes on HDD2 because it cannot go anywhere else: 36000 polar frames re-encoded as PNG measure
**17.0 GB** (**V**, extrapolated per cell from a 540-frame trial: tank/lf 301 KB, tank/hf 401, basin/lf 676,
basin/hf 1007 per frame), plus ~0.6 GB of labels and COCO json and ~0.3 GB of preview overlays. The SSD has
39 G free. The source is read-only throughout; nothing is moved or rewritten.

## 2. The label to train on

**The default label is `mask_geom`, not `mask_direct`.** The two energy-share masks mark *energy ownership*,
not plume extent, and that distinction decides what a detector learns. Measured over 400 randomly sampled
positive frames (**V**):

| | `mask_geom` | `mask_direct` | `mask_all` |
|---|---|---|---|
| beams spanned, median of 512 | **36** | **487** | 497 |
| frames spanning >= 480 of 512 beams | 0 / 400 | **214 / 400** | 229 / 400 |
| mask px, median | 1547 | 11471 | 14331 |
| connected components after a 5x5 close, median | **1** | 4 | 6 |
| positives in the whole dataset | 20645 | 22106 | 22107 |
| negatives (of 36000) | 15355 (42.7 %) | 13894 (38.6 %) | 13893 (38.6 %) |

`mask_direct` marks cells where the bubbles own *most of the energy*, and in the quiet water column above the
seabed there is almost no other energy to own, so a thin horizontal band crosses the 50 % threshold across
the whole fan. That is physics, not a bug — but a detector trained on a 487-of-512-beam band learns the water
column, not the leak, and will call a plume wherever the water is quiet. The preview overlays show it
plainly: the polygon is a band, not a plume. `mask_geom` is the main-lobe line-of-sight footprint — **median
one instance per frame**, sitting on the bright plume in the image — which is also the "one row per plume"
the conversion was asked for.

So `dataset_yolo/` and `dataset_coco/` carry **`mask_geom`**, and `dataset_yolo_mask_direct/`,
`dataset_yolo_mask_all/` and their COCO twins ship beside them as comparison arms, written in the same run
from the same pixels and the same split, at the cost of their label files only. Two further measurements bear
on anyone who wants to use them:

- **Intersecting `direct` with `geom` to clean it does not work.** The median IoU between the two is **0.132**
  (**V**), and only about a third of `mask_direct` falls inside `mask_geom`. They are different objects, and
  the intersection throws away two thirds of the label.
- **`--level-gate 128` does work.** It keeps only mask pixels whose display level exceeds 128, that is, only
  what is also *visible*: the median IoU(direct, geom) rises from 0.132 to **0.504** while keeping 84 % of
  `geom` (**V**). That is the supported way to get a plume-shaped label out of `mask_direct`. Off by default.

`mask_geom` is not free of cost. It is geometric, so it marks cells where a bubble sits in the main lobe
whether or not its echo rose above the background, and it leaves 15355 negative frames against
`mask_direct`'s 13894 (42.7 % against 38.6 %). That is the trade, and it is the one worth making.

## 3. What the raw data is

**Not rosbags.** There is no ROS in the dataset at all. `tools/sweep.py` writes a flat directory per scenario:

```
s0000/  manifest.json  DONE  RECOMPRESSED
        ping_00000.json   sidecar: pose, mode, range, noise_floor_db, display_db, plume_pixels, timings
        ping_00000.npz    polar_db float16 (bins,512) NaN in the blind zone, plume_share_{all,direct}_u8,
                          range_m float32 (bins,), bearing_deg float32 (512,)
        ping_00000.png    uint8 polar display image (bins,512)
        ping_00000_mask_{all,direct,geom}.png   uint8 0/255, same shape
```

**Every PNG is stored flipped** (row 0 = far range); the npz arrays are not (**V**, `uw/recorder.py:81-88`,
and `uw.dataset.load_ping` flips the PNGs back on the way in). The converter flips every PNG, so images,
masks and anything taken from the npz share row 0 = range 0. Mixing the two conventions mirrors the labels in
range and no loader complains — it is the one silent corruption this conversion can produce.

## 4. The sonar

A **multibeam forward-looking imaging sonar** (Oculus M750d-like), 512 beams, not an echosounder. Two modes
(**V**, `config/tank.yaml:166-169`), sampled per scenario:

| mode | frequency | horizontal FOV | vertical FOV | beamwidth | range bin | pings |
|---|---|---|---|---|---|---|
| `lf` | 750 kHz | **130°** | 20° | 1.0° | 4.0 mm | 23580 (65.5 %) |
| `hf` | 1200 kHz | 70° | 12° | 0.6° | 2.5 mm | 12420 (34.5 %) |

and two range presets, tank 3.5 m and basin 8.0 m, giving four frame shapes (**V**):

| cell | polar frame | pings | fan at width 1024 |
|---|---|---|---|
| tank / lf | 876 × 512 | 16020 | 566 × 1024 |
| tank / hf | 1401 × 512 | 9000 | 893 × 1024 |
| basin / lf | 2001 × 512 | 7560 | 566 × 1024 |
| basin / hf | 3201 × 512 | 3420 | 893 × 1024 |

Blind zone 0.1 m (zeroed). The 8-bit image is the simulator's display product: beamformed power → dB, then
30 dB/decade TVG plus absorption, referenced to that ping's noise floor and windowed to `display_db [0, 80]`.
**That window is a constant of the dataset**, identical in every scenario, so the fixed cross-session
normalisation the conversion needs already exists upstream and the converter does not touch the pixels.
`polar_db` in the npz is the *calibrated pre-TVG* level in dB re 1 µPa, which is a different quantity and not
reproducible as a display image from the dataset alone (the sidecar carries `noise_floor_db` and `display_db`
but neither `tvg_db_per_decade` nor the absorption coefficient). `--db-range` therefore produces a
*different, range-shaded representation*, not a re-window of the stored image; `--db-noise-floor` reproduces
`uw/dataset.py`'s `"db"` image instead, which is per-ping and so not a fixed normalisation.

## 5. Annotations: they already exist, and there are three of them

Nothing has to be annotated. Every ping carries three binary ground-truth masks, documented in the scenario
manifest's `labels` block:

- **`mask_geom`** — geometric: cells whose main lobe contains an alive bubble in line of sight. Independent
  of energy.
- **`mask_direct`** — cells where the bubbles' *direct-path* echo exceeds 50 % of the cell's energy, including
  leakage through the array's sidelobes.
- **`mask_all`** — as above over all paths, including surface ghosts and the plume seen through the vertical
  sidelobes.

The sidecar's `plume_pixels` is `np.count_nonzero` of each mask, so it is a free per-ping oracle; the
converter asserts the decoded mask against it on every frame, which catches a flip or a wrong file on ping 1
instead of on epoch 1.

### 5.1 Which of the three is the label

`mask_geom`, and `tools/sonar_to_yolo.py` defaults to it; the two energy masks ship as comparison
variants. The numbers behind that choice — beam coverage, component counts, the 0.132 IoU between
`direct` and `geom`, and the `--level-gate 128` recovery — are in section 2, which is where a reader
deciding what to train on should start.

### 5.2 Masks are speckle-fragmented, so a polygon per component is not an option

One positive basin/lf ping's `mask_direct` has **4118 connected components, 3801 under 10 px** (**V**).
Emitting one polygon per component does not merely produce noisy labels, it breaks both formats: a 1-px
component gives a 1-point contour, which Ultralytics counts as a corrupt label and drops *the whole image*
for, and which `pycocotools.frPyObjects` refuses outright (a 4-value polygon is read as a bbox). The mask is
therefore closed, hole-filled and cleared of components under `--min-area` before contours are taken.

Defaults measured over 200 sampled positive frames, `--min-area 64`, ε = 0.004 × perimeter (**V**):

| `--close` | instances/frame (median) | labelled area ÷ mask area | recall of the mask |
|---|---|---|---|
| 1 (off) | 5 | 0.83 | 0.67 |
| **3** | **4** | **1.18** | **0.81** |
| 5 | 4 | 1.61 | 0.90 |
| 7 | 4 | 1.98 | 0.94 |

Closing past 3 buys no reduction in instance count and pays for it in hallucinated area, so **`--close 3`** is
the default. On `mask_geom`, the label this dataset ships with, the same settings give a median of
**1 instance per frame at 1.03× the mask area** — one row per plume, which is what the conversion was asked
for, and almost no reshaping of the mask to get it. The table above is `mask_direct`, where the same settings
are a real compromise; that asymmetry is another reason the default is `geom`.

This is a **change of label definition**, not a cleanup: the dropped fragments are real sidelobe leakage that
the mask's own definition includes. `meta/frames.csv` therefore records, per frame and per mask,
`mask_px`, `components_raw`, `components_kept`, `dropped_px` and `instance_px`, and `meta/stats.json`
aggregates `labelled_area_over_mask` per split, so the trade is auditable after the fact.

Holes are filled because neither format can express one: a YOLO-seg line and a COCO polygon are both a single
ring, rasterised solid.

## 6. Scale, splits and anomalies

400 scenarios × 90 pings = **36000 frames**, 216.55 GB as generated (`manifest.totals.bytes`), being
recompressed in place to ~59 GB. Scenario kinds: `leak` 246, `distractor` 101, `negative` 53 — and
**only `leak` scenarios ever contain plume pixels** (**V**), so 38.6 % of frames are true negatives, evenly
spread across splits (train 38.5 %, val 38.3 %, test 39.1 %).

**The split is taken from the manifest, never regenerated.** It is per scenario, stratified by kind:
train 281 / val 60 / test 59 scenarios = 25290 / 5400 / 5310 pings. Neighbouring pings in a scenario are
0.1 s apart and nearly identical, so a per-frame split would leak; and reusing the shipped split keeps
anything trained here comparable with the rest of the repo. `--split ratio` / `--val-sessions` exist as
overrides and are not used.

Anomalies found, none blocking:

- **`holdout` is `false` for all 400 scenarios.** The field exists and was never used in v0.2. It is carried
  into `frames.csv` as a column and nothing is held out.
- **34 pings in 20 `leak` scenarios have a leak but an empty mask** — almost all of them `ping_00000..2`,
  before the plume has developed or while it is out of view. They are correct negatives.
- **The negative sets differ between variants by one ping**: `mask_all` has 22107 positives against
  `mask_direct`'s 22106. The two datasets share one image list and one image-id map but must not be assumed
  to share a negative set.
- `plume_pixels` quantiles over positive frames run from **1 px** to 159429 px for `mask_direct`; the
  `--min-area 64` floor turns a small number of nominally positive frames into negatives (9 of 200 sampled
  `mask_geom` positives).
- No corrupt frames, no missing files, no NaN outside the blind zone: the recompression pass verifies every
  ping through `uw.dataset.load_ping` and has produced 0 `RECOMPRESS_FAILED` markers.

## 7. Things that will bite during training, measured

- **Aspect ratio.** Ultralytics letterboxes the long side to `imgsz`, so at `imgsz=640` a basin/hf frame goes
  3201 × 512 → **640 × 103** and the 512 bearing bins are crushed to a hundred (**V**). Then `mask_ratio: 4`
  rasterises supervision at 160 × 160: a 426-px native plume becomes **2 supervision pixels**. `frames.csv`
  carries `<mask>_supervision_px` per frame for exactly this reason. Mitigations, best first: tile the polar
  frame along range into near-square crops; `imgsz=1024`; `mask_ratio: 1` or `2`; `rect=True` (measured batch
  shapes 640×384 / 640×256 / 640×192 / 640×128, a 5× compute saving at identical effective resolution — but
  it forces `mosaic=0` and makes every batch a single geometry cell).
- **Default augmentations are wrong for polar sonar.** `mosaic: 1.0` composes four polar frames into one
  meaningless image; `scale` and `translate` break the range↔row relationship that makes a TVG'd display
  image interpretable. In one measured case a positive frame with 58329 mask pixels came out of the default
  pipeline as a **background** (**V**). Start from `mosaic: 0.0, scale: 0.1, translate: 0.05, degrees: 0.0,
  flipud: 0.0, fliplr: 0.5, hsv_*: 0.0` — `fliplr` is the only geometrically defensible one, mirroring
  bearing about broadside.
- **RF-DETR resizes to a square** (312–768 px depending on variant, `square_resize_div_64: bool = True`), so
  the same squash applies, harder. Consider `square_resize_div_64=False` or an explicit `resolution`.
- **COCO `area` bins.** Nearly every instance lands in COCOeval's "small" bucket, so `AP_m`/`AP_l` come back
  `-1`. Expected, not a bug.
- **COCO json size.** The default `mask_geom` train split is a few tens of MB of polygons; the
  `mask_direct` and `mask_all` arms are around 140 MB and more, because they carry an order of magnitude more
  instances. RF-DETR loads the whole file into memory, and this box has under 3 GB free.

## 8. Representations

`--repr polar` (default) is the sonar's own product and loses nothing. `--repr fan` builds the Cartesian fan
on the CPU, geometry-identical to the simulator's GPU kernel (checked against the 44 GPU-written fan PNGs
under `output/sonar_preview/`: max 1 grey level on 6 × 10⁻⁵ of pixels, **V**), with the mask warped
nearest-neighbour at 3×3 supersampling. **It is a lossy arm, not a free alternative**: at width 1024 the fan
never samples **22.8 % of basin/lf and 28.7 % of basin/hf polar cells** (**V**), because `scale·dr < 1` — a
thin bright arc one range bin deep can disappear. It also wastes 37.7 % (lf) to 46.9 % (hf) of the output
rectangle on black outside the sector. Use it for comparison, not as the default.

`--repr echogram` **is refused**. An echogram is a time × depth waterfall from a single-beam echosounder;
this is a 512-beam imaging sonar and there is no such product to window. The tool exits with that message
rather than inventing one.

## 9. What was and was not verified

- **Ultralytics 8.4.96**: the written datasets were built through `check_det_dataset` +
  `build_yolo_dataset` + `ds[0]` — the exact path `yolo segment train` takes — 0 corrupt, and a real
  `yolo segment train epochs=1 imgsz=640 device=cpu` completed on the trial output (**V**).
- **pycocotools 2.0.11**: every `_annotations.coco.json` loads, every annotation's polygons rasterise to a
  non-empty mask, ids unique, no degenerate bbox (**V**).
- **RF-DETR was never executed.** `rfdetr` is installed in none of the 16 venvs on this box, and nothing was
  installed. The COCO contract — `valid/` not `val/`, one flat category at id 1 with no synthetic root,
  `area`/`iscrowd` mandatory, segmentation polygons read by the Seg variants — was read from upstream
  **rf-detr 1.11.2** (`7433d5919d0f`) and is validated structurally only (**U** for the loader, **V** for the
  file format against pycocotools). The flat single category matters: RF-DETR drops a parent category that
  carries no annotations, so a Roboflow-style `{id 0 root, id 1 class}` pair trains a 2-class head on
  versions before its 1.9.2 category filter, while one flat entry behaves identically either side.
- The GPU was not used for anything, and the only CUDA compute client on the box (a resident f1tenth Telegram
  service holding ~2 GB) was left alone; the smoke test runs `device=cpu`.

## 10. One hazard to remember

The output hardlinks the images rather than copying them, which is what keeps three label variants and two
formats at one copy of the pixels. A *future* in-place rewrite of the source — another recompression pass,
say — replaces files with `os.replace` and so breaks those links silently, leaving the converted dataset
pinned to stale pixels. `meta/stats.json` records the source's `git`, `config_hash` and the number of
`RECOMPRESSED` markers at conversion time so that this is detectable afterwards.
