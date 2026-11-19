# Baseline YOLO run — training summary

Run: `v0.2_geom_y11s_baseline`
Run directory: `/HDD2T/underwater_data/runs/seg/v0.2_geom_y11s_baseline`
Test-split val directory: `/HDD2T/underwater_data/runs/seg/v0.2_geom_y11s_baseline_test`

All numbers below are copied from `args.yaml`, `results.csv` and
`v0.2_geom_y11s_baseline.tmux.log` in that directory. Nothing was re-run or re-computed,
except the per-epoch time (total ÷ epochs) where that is stated.

## Setup

| | |
|---|---|
| task | segment (instance segmentation) |
| model | `yolo11s-seg.pt` — YOLO11s-seg, 114 layers fused, 10,067,203 parameters, 32.8 GFLOPs |
| pretrained weights | yes (`pretrained: true`); "Transferred 555/561 items from pretrained weights" |
| dataset | `/home/andreas/datasets/sonar_v0.2_geom/data.yaml` (SSD copy of `/HDD2T/underwater_data/yolo/v0.2/dataset_yolo`) |
| label variant | `mask_geom` |
| classes | 1 — `bubbles` |
| split sizes (images) | train 25,290 / val 5,400 / test 5,310 |
| val instances | 4,158 |
| test instances | 4,045 |
| imgsz | 640 |
| batch | 8 |
| epochs requested | 100 |
| patience | 20 |
| optimizer | `auto` → MuSGD(lr=0.01, momentum=0.9); groups: 90 weight(decay=0.0), 101 weight(decay=0.0005), 100 bias(decay=0.0) |
| lr0 / lrf | 0.01 / 0.01 |
| amp | true |
| seed / deterministic | 0 / true |
| workers | 8 |
| close_mosaic | 10 |
| augmentation | mosaic 1.0, fliplr 0.5, scale 0.5, translate 0.1, hsv_v 0.2, erasing 0.4; hsv_h/hsv_s/degrees/shear/perspective/flipud/mixup/cutmix/copy_paste all 0 |
| framework | Ultralytics 8.4.96, Python 3.12.3, torch 2.13.0+cu130 |
| GPU | NVIDIA GeForce RTX 5050, 7672 MiB (CUDA:0) |

## Duration

| | |
|---|---|
| epochs actually run | 43 of 100 |
| early stopping | **yes** — "Training stopped early as no improvement observed in last 20 epochs. Best results observed at epoch 23, best model saved as best.pt." |
| best epoch | 23 |
| total wall-clock | 5.689 h (20,479.9 s cumulative in `results.csv`) |
| time per epoch | 476.3 s ≈ 7.94 min (20,479.9 s ÷ 43; includes the per-epoch val pass) |
| inference speed, val | 0.1 ms preprocess, 1.7 ms inference, 0.7 ms postprocess per image |
| inference speed, test | 0.3 ms preprocess, 2.9 ms inference, 0.5 ms postprocess per image |

## Metrics

Single class, so the per-class row and the `all` row are the same.

### val split (5,400 images, 4,158 instances)

| | Box P | Box R | Box mAP50 | Box mAP50-95 | Mask P | Mask R | Mask mAP50 | Mask mAP50-95 |
|---|---|---|---|---|---|---|---|---|
| final epoch (43) | 0.91887 | 0.62747 | 0.70134 | 0.38788 | 0.84339 | 0.59885 | 0.65404 | 0.34352 |
| best epoch (23) | 0.92533 | 0.63372 | 0.71261 | 0.39644 | 0.88548 | 0.60197 | 0.67111 | 0.35428 |
| final val of `best.pt` | 0.925 | 0.634 | 0.713 | 0.396 | 0.885 | 0.602 | 0.671 | 0.354 |

(The last row is the end-of-training validation pass that Ultralytics runs on `best.pt`; it is
printed to 3 decimals in the log and matches the epoch-23 row of `results.csv`.)

Epoch 1, for reference: Box 0.77294 / 0.57119 / 0.62836 / 0.29804, Mask 0.7628 / 0.54497 / 0.61623 / 0.28772.

### test split (5,310 images, 4,045 instances) — `best.pt`, run separately

| | Box P | Box R | Box mAP50 | Box mAP50-95 | Mask P | Mask R | Mask mAP50 | Mask mAP50-95 |
|---|---|---|---|---|---|---|---|---|
| test | 0.842 | 0.626 | 0.658 | 0.363 | 0.768 | 0.572 | 0.605 | 0.302 |

## Notes

- Ultralytics picks `best.pt` by its combined fitness score, which selected epoch 23
  (best Box mAP50-95, 0.39644). Mask mAP50-95 peaked marginally later, at epoch 28 with
  0.35459 versus 0.35428 at epoch 23 — a difference of 0.0003.
- The run is on **simulated** forward-looking sonar only. No real sonar is in train, val or test.
- val and test are both held-out scenarios from the same v0.2 simulation sweep, so the
  val→test drop (mask mAP50 0.671 → 0.605) is a scenario split, not a sim-to-real result.
- Items with no value in the source files: **n/a** — there is no per-class breakdown beyond
  the single `bubbles` class, no `args.yaml` for the test-split val run, and no recorded
  peak VRAM figure.
