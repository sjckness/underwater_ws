# slides_assets — manifest

Every image here is a **copy** (converted to PNG, downscaled only if wider than 1920 px,
aspect ratio kept). No original was moved, renamed or deleted. Nothing was re-simulated,
re-trained or re-rendered; Isaac Sim was not launched.

Slide map: **1** environments · **2** leak model · **3** sonar · **4** YOLO baseline ·
**5** large scene · **6** HIL.

Paths starting with `output/` are relative to `/home/andreas/underwater_ws`.

---

## Slide 1 — environments

| file | original | what it shows |
|---|---|---|
| `s1_tank_camera_leak_plume.png` | `output/sim/screenshots/default_t15_camera.png` | Tank scene, RTX camera render at t = 15 s: exposed gas pipe on the sand, bubble plume clearly visible rising from the leak hole. The best camera frame in the repo. |
| `s1_tank_camera_lowlight_t25.png` | `output/m3/camera_t25.png` | Same tank, t = 25 s, lit only by the two LED lamps — the realistic low-light look. Bubbles are visible but faint above the pipe. |
| `s1_tank_camera_lowlight_probe.png` | `output/probe/render_probe_glass.png` | Third camera frame, same low-light tank, slightly different lamp/material probe. Faint bubble column above the leak. |
| `s1_tank_viewport_bubbles_and_scatterers.png` | `output/m3/gui_side45_dome_inspect.png` | Isaac Sim viewport, 45° side view: the dense blue bubble PointInstancer, the sand/rock seabed, the pipe, the yellow sonar fan wireframe and the orange sonar scatterer points. Best single "what the scene is" picture. |
| `s1_tank_isaacsim_gui_window.png` | `output/sim/screenshots/default_t15_gui.png` | Full Isaac Sim GUI window with the tank scene loaded (same pixels as `output/m3/gui_side45_dome_inspect_window.png`). Use if the slide needs to show the tooling. |

## Slide 2 — leak model

| file | original | what it shows |
|---|---|---|
| `s2_plume_model_exposed_pipe.png` | `output/plume_preview/exposed/bubbles.png` | Gas-leak plume model, pipe preset `exposed`, t = 27 s, 61,039 live bubbles. Side (x-z) and top (x-y) scatter coloured by bubble radius, integral centreline and ±b envelope, plus the full parameter panel: methane, 2.50 m release depth, 0.360 g/s choked, 25.5 L/min at release, d50 3.0 mm, slip 0.234 m/s, 11,787 bubbles/s, rise time 5.27 s. The headline figure for this slide. |
| `s2_plume_model_buried_pipe.png` | `output/plume_preview/buried/bubbles.png` | Same figure for the `buried` pipe preset — the leak emerges through the sediment cover. |
| `s2_plume_profiles_exposed_pipe.png` | `output/plume_preview/exposed/profiles.png` | Centreline velocity / width / void-fraction profiles vs height for `exposed`, model curves against the MTT-style similarity solution. |
| `s2_plume_profiles_buried_pipe.png` | `output/plume_preview/buried/profiles.png` | Same profiles for `buried`. |

## Slide 3 — sonar

| file | original | what it shows |
|---|---|---|
| `s3_sonar_tank_4panel_static_vs_plume.png` | `output/sonar_preview/exposed/overview.png` | **The with/without-plume pair in one figure**: tank scene, fan view, 2×2 — static vs plume × focused vs unfocused. Ping 41 ms, 29 plume paths. Clearest single sonar slide image. |
| `s3_sonar_tank_fan_with_plume.png` | `output/sonar_preview/exposed/plume_focus_fan.png` | One ping, tank, **fan** (Cartesian) view, focused, plume present. |
| `s3_sonar_tank_fan_no_plume.png` | `output/sonar_preview/exposed/static_focus_fan.png` | Same scene and ping, **no plume** — the direct comparison to the file above. |
| `s3_sonar_tank_polar_with_plume.png` | `output/sonar_preview/exposed/plume_focus_polar.png` | Same ping in **polar** (range × beam) form, plume present. |
| `s3_sonar_tank_polar_no_plume.png` | `output/sonar_preview/exposed/static_focus_polar.png` | Same ping in polar form, no plume. |
| `s3_sonar_fan_gt_mask_outline.png` | `output/probe/frame_exposed.png` | **GT mask on the fan view**: tank fan image with the ground-truth `plume_direct` label drawn as an orange outline, plus a side-view inset of the sonar cone and the plume. Legend explains the label definition. |
| `s3_sonar_fan_gt_mask_grid_v0.2.png` | `/HDD2T/underwater_data/datasets/v0.2/stats/examples.png` | 20 random v0.2 pings in fan view with GT outlines (`mask_geom` cyan, `mask_direct` orange), annotated with flow, pitch and roll. Covers leak / negative / distractor scenarios. |
| `s3_sonar_polar_gt_mask_overlay_s0083_ping29.png` | generated from `/HDD2T/underwater_data/datasets/v0.2/s0083/ping_00029.png` + `_mask_direct.png` + `_mask_geom.png` | **Semi-transparent GT overlay** (alpha 0.45) on the polar ping: `mask_direct` orange, `mask_geom` cyan. Largest label of the three (26,918 geom / 80,567 direct px). Composited from the stored mask PNGs — no new simulation. |
| `s3_sonar_polar_gt_mask_overlay_s0205_ping51.png` | generated from `/HDD2T/underwater_data/datasets/v0.2/s0205/ping_00051.png` + its two mask PNGs | Same overlay, different scenario (15,059 geom / 45,733 direct px). The plume rises off the seabed return. |
| `s3_sonar_polar_gt_mask_overlay_s0336_ping78.png` | generated from `/HDD2T/underwater_data/datasets/v0.2/s0336/ping_00078.png` + its two mask PNGs | Same overlay, third scenario (5,926 geom / 42,244 direct px). |
| `s3_sonar_polar_gt_polygon_s0083_ping29.png` | `/HDD2T/underwater_data/yolo/v0.2/meta/preview/s0083_ping_00029.png` | The dataset's own preview: the exact polar image fed to YOLO with the exported `mask_geom` polygon in green — i.e. the label as the trainer sees it. |
| `s3_sonar_polar_gt_polygon_s0336_ping78.png` | `/HDD2T/underwater_data/yolo/v0.2/meta/preview/s0336_ping_00078.png` | Same, second scenario. |
| `s3_sonar_scatterer_cloud_viewport.png` | `output/m3/gui_lamps_only_viewport.png` | Isaac viewport with the scene unlit except the LED lamps: the orange sonar scatterer point cloud and the yellow fan wireframe stand alone. Shows how the acoustic model samples the scene. No bubbles in this frame. |
| `s3_real_vs_sim_speckle_blindzone_tvg.png` | `output/compare_real/compare.png` | **Real vs simulated sonar.** Three panels: background speckle pdf against a Rayleigh reference (sim `smoke_exposed` contrast 2.01, UATD_Test_1 0.99, DeeperSense basin-dataset-1 1.02, basin-dataset-2 0.33); blind zone and ring-down; level vs range (TVG profile). Written by `tools/compare_real.py`. |
| `s3_real_vs_sim_speckle_blindzone_tvg_basin2.png` | `output/compare_real/dfki_basin2/compare.png` | Same comparison restricted to the DFKI basin-dataset-2 recording. |

## Slide 4 — YOLO baseline

All from the baseline run `v0.2_geom_y11s_baseline` (YOLO11s-seg, 43 epochs, early-stopped).
Numbers in `training_summary.md`.

| file | original | what it shows |
|---|---|---|
| `s4_yolo_results_curves.png` | `.../v0.2_geom_y11s_baseline/results.png` | The standard Ultralytics results grid: train/val box, seg, cls, dfl losses and P / R / mAP50 / mAP50-95 for box and mask over the 43 epochs. |
| `s4_yolo_val_BoxPR_curve.png` | `.../v0.2_geom_y11s_baseline/BoxPR_curve.png` | Precision–recall curve, box, val split. |
| `s4_yolo_val_MaskPR_curve.png` | `.../v0.2_geom_y11s_baseline/MaskPR_curve.png` | Precision–recall curve, mask, val split. |
| `s4_yolo_val_confusion_matrix.png` | `.../v0.2_geom_y11s_baseline/confusion_matrix.png` | Confusion matrix, val split, absolute counts (2×2: `bubbles` / background). |
| `s4_yolo_val_confusion_matrix_normalized.png` | `.../v0.2_geom_y11s_baseline/confusion_matrix_normalized.png` | Same, normalised. |
| `s4_yolo_val_batch0_pred.png` | `.../v0.2_geom_y11s_baseline/val_batch0_pred.jpg` | Validation predictions, batch 0 — predicted masks and confidences on held-out polar pings. |
| `s4_yolo_val_batch1_pred.png` | `.../v0.2_geom_y11s_baseline/val_batch1_pred.jpg` | Validation predictions, batch 1. |
| `s4_yolo_val_batch2_pred.png` | `.../v0.2_geom_y11s_baseline/val_batch2_pred.jpg` | Validation predictions, batch 2. |
| `s4_yolo_val_batch0_labels_gt.png` | `.../v0.2_geom_y11s_baseline/val_batch0_labels.jpg` | The ground truth for the same batch 0 — pair it with the prediction image side by side. |
| `s4_yolo_test_batch0_pred.png` | `.../v0.2_geom_y11s_baseline_test/val_batch0_pred.jpg` | Predictions on the **test** split (the separate `best.pt` val run). |
| `s4_yolo_test_BoxPR_curve.png` | `.../v0.2_geom_y11s_baseline_test/BoxPR_curve.png` | PR curve, box, test split. |
| `s4_yolo_test_MaskPR_curve.png` | `.../v0.2_geom_y11s_baseline_test/MaskPR_curve.png` | PR curve, mask, test split. |
| `s4_dataset_v0.2_overview_stats.png` | `/HDD2T/underwater_data/datasets/v0.2/stats/overview.png` | The v0.2 dataset the baseline trained on: scenarios by split and kind, leak flow by preset, pitch/yaw per ping, sonar positions, leak in view, label sizes, generation wall time. 9 panels. |
| `training_summary.md` | — | Written from `args.yaml`, `results.csv` and the tmux log. |

## Slide 5 — large scene (basin)

| file | original | what it shows |
|---|---|---|
| `s5_sonar_basin_4panel_static_vs_plume.png` | `output/sonar_preview/exposed+basin/overview.png` | **Basin preset**, exposed pipe: 2×2 static vs plume × focused vs unfocused, fan view out to 8 m ahead and ±7 m across (vs 3.5 m in the tank). The scale contrast with the slide-3 figure is the point. |
| `s5_sonar_basin_fan_with_plume.png` | `output/sonar_preview/exposed+basin/plume_focus_fan.png` | Single basin ping, fan view, focused, plume present. |
| `s5_sonar_basin_fan_no_plume.png` | `output/sonar_preview/exposed+basin/static_focus_fan.png` | Same, no plume. |
| `s5_sonar_basin_polar_with_plume.png` | `output/sonar_preview/exposed+basin/plume_focus_polar.png` | Same basin ping in polar form (512 × 2001 — the longer range needs more range bins). |
| `s5_sonar_basin_buried_4panel_static_vs_plume.png` | `output/sonar_preview/buried+basin/overview.png` | Basin with the **buried** pipe preset, same 2×2 layout. |

## Slide 6 — HIL

| file | original | what it shows |
|---|---|---|
| `hil_measurements.md` | compiled from `docs/bridge_setup.md`, `docs/ros_hil.md`, `docs/topic_map.md`, `docs/PLAN.md`, `docs/validation.md`, `output/probe/m7/*.json` and `git log` | Every connection/HIL number already recorded, with value, unit, topic, conditions, where it was measured and file:line. Answers (a) sonar and camera bandwidth/Hz, (b) link total vs 1 GbE and `/gt` separation, (c) closed-loop latency, (d) Thor link facts, (e) `/HDD2T` write throughput and GT bag compression, (f) per-topic drops and RTF with/without camera. **Headline for the slide: every measurement is loopback on `linus`; the Jetson (thor) has never been connected, so there is no `192.168.9.x` number at all.** |

No *image* for this slide was found — see the table below.

---

## Looked for and not found

| wanted | status |
|---|---|
| **Camera frames with the GT bubble mask overlaid** (brief 1a, second half) | **Does not exist.** The camera path renders RGB only — `uw/main.py` creates a single `rgb` render product and `uw/ros_bridge.py` publishes only `/camera/image_raw`. No semantic AOV, no camera mask file anywhere on disk. `docs/PLAN.md:142` lists "Camera: frames + plume mask from instancer semantics" as planned recorder work, not built. The three raw camera frames are included; the overlays would have had to be invented, so they were not made. The closest existing substitute is `s1_tank_viewport_bubbles_and_scatterers.png`, where the bubble instancer is directly visible. |
| **Semi-transparent GT mask on the sonar *fan* view** (brief 1b) | **Partially.** The dataset stores polar images and polar masks only (`uw/recorder.py` writes `polar`, `mask_all`, `mask_direct`, `mask_geom`; fan PNGs were not enabled for v0.2), so a filled semi-transparent fan overlay cannot be composited from stored files. Supplied instead: `s3_sonar_fan_gt_mask_outline.png` (GT outline on a fan image, pre-existing) and `s3_sonar_fan_gt_mask_grid_v0.2.png` (20 fan pings with GT outlines). The filled semi-transparent overlays are provided on the polar view (`s3_sonar_polar_gt_mask_overlay_*.png`). |
| **Screenshot / render of the large basin scene** (brief 1c, second half) | **Not found.** Only sonar output exists for the basin preset (`output/sonar_preview/exposed+basin/`, `buried+basin/`). There is no Isaac viewport capture, RTX render or GUI screenshot of the basin anywhere in `output/`, `docs/` or `/HDD2T/underwater_data/`. The tank is covered by the slide-1 files. Isaac Sim was not launched to make one. |
| **BlueROV2 render** (brief 1d) | **Does not exist.** `BlueROV2` appears only as planned milestone M8a in `docs/PLAN.md:176-189` and as design text in `docs/ros_hil.md`. No USD, mesh or image of the vehicle is on disk; `/rov1/…` in `docs/topic_map.md` is a topic namespace, not a model. |
| **HIL figure for slide 6** | **Not found.** `docs/ros_hil.md` and `docs/topic_map.md` describe the HIL setup only in prose and ASCII diagrams — no exported image. `/HDD2T/underwater_data/real/deepersense_metadata/entity_relationship_diagram.png` exists but is the DeeperSense database schema, unrelated to the HIL stack. Slide 6 will need a diagram drawn from the ASCII in `docs/ros_hil.md` (the TF tree at line ~423 is the most slide-ready piece). The numbers for that slide are collected in `hil_measurements.md`. |
| **Per-class metric breakdown** | n/a — the dataset has a single class, `bubbles`, so the per-class row equals the `all` row. |
| **Peak VRAM / args.yaml for the test-split val run** | Not recorded. The test run wrote only plots and `val_batch*` images, no `args.yaml`. |

## Provenance notes worth stating on the slides

- Everything here is **simulated** sonar and camera data. The only real sonar in the repo is
  the reference recordings used in `s3_real_vs_sim_*` (UATD and DeeperSense basin datasets).
- `s4_dataset_v0.2_overview_stats.png` and `s3_sonar_fan_gt_mask_grid_v0.2.png` describe
  **v0.2** (400 scenarios, 36,000 pings), the dataset the baseline actually trained on.
  The older `output/m5/overview.png` and `output/m5/examples.png` in the repo describe
  **v0.1** (300 scenarios, 18,000 pings) and were deliberately *not* used, to avoid quoting
  the wrong dataset next to the baseline numbers.
- The baseline labels are `mask_geom` (geometric: cells whose main lobe contains an alive
  bubble in line of sight). `mask_direct` and `mask_all` are looser variants and are the
  orange regions in the overlay images.
