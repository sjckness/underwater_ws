#!/usr/bin/env bash
# Wait for the in-place recompression pass to finish, then convert v0.2 to YOLO-seg + COCO and verify it.
#
#     setsid nohup flock -n /HDD2T/underwater_data/datasets/.sonar_to_yolo.lock \
#       tools/run_sonar_to_yolo.sh --pid 1387995 \
#       >/HDD2T/underwater_data/datasets/v0.2_to_yolo.log 2>&1 &
#
# The gate is deliberately not the pass's exit status: the pass is parented to an editor session, so it can
# die without finishing, and <root>/recompress.json is written only at the very end and the one on disk right
# now is a stale two-scenario roll-up. The gate that means what it says is one RECOMPRESSED marker per
# scenario in the manifest and no RECOMPRESS_FAILED anywhere. The PID is only how we wait.
#
# tmux is not installed on linus: watch it with `tail -f` on the log, and `kill -0 <pid>` for liveness.
set -u

PID=1387995
ROOT=/HDD2T/underwater_data/datasets/v0.2
OUT=/HDD2T/underwater_data/yolo/v0.2
POLL=60
UL_VENV=/home/andreas/dev_ws/kumi_stack/.venv/bin/python          # ultralytics 8.4.96, torch 2.13.0+cu130
COCO_VENV=/home/andreas/image_processing/datasets_gSEM/autodistill-env/bin/python   # the only pycocotools
WS="$(cd "$(dirname "$0")/.." && pwd)"
SCRATCH=/HDD2T/underwater_data/yolo/.smoke
CONV_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --pid)   PID="$2";   shift 2 ;;
    --root)  ROOT="$2";  shift 2 ;;
    --out)   OUT="$2";   shift 2 ;;
    --poll)  POLL="$2";  shift 2 ;;
    --)      shift; CONV_ARGS=("$@"); break ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

say() { echo "$(date '+%F %T') $*"; }

say "waiting for the recompression pass, PID $PID, polling every ${POLL}s"
if [ -d "/proc/$PID" ]; then
  while [ -d "/proc/$PID" ]; do
    sleep "$POLL"
  done
  say "PID $PID is gone"
else
  say "PID $PID is not running; checking the markers directly"
fi

WANT=$(/usr/bin/python3 -c "import json;print(sum(1 for s in json.load(open('$ROOT/manifest.json'))['scenarios'] if s.get('status')=='done'))")
DONE=$(find "$ROOT" -maxdepth 2 -name RECOMPRESSED | wc -l)
FAILED=$(find "$ROOT" -maxdepth 2 -name 'RECOMPRESS_FAILED*' | wc -l)
say "recompression markers: $DONE/$WANT done, $FAILED failed"
if [ "$FAILED" -ne 0 ]; then
  say "REFUSING: $FAILED scenarios carry RECOMPRESS_FAILED"
  find "$ROOT" -maxdepth 2 -name 'RECOMPRESS_FAILED*' | head
  exit 1
fi
if [ "$DONE" -ne "$WANT" ]; then
  say "REFUSING: the pass stopped with $DONE of $WANT scenarios recompressed; the dataset is half rewritten"
  exit 1
fi
say "gate passed, $(du -sh "$ROOT" | cut -f1) on disk"

cd "$WS" || exit 1
say "converting -> $OUT"
nice -n 10 ionice -c3 /usr/bin/python3 -I tools/sonar_to_yolo.py run --root "$ROOT" --out "$OUT" \
  ${CONV_ARGS[@]+"${CONV_ARGS[@]}"}
RC=$?
say "converter exit $RC"
[ "$RC" -ne 0 ] && exit "$RC"

FAILURES=0
say "--- structural check (repo python) ---"
nice -n 10 /usr/bin/python3 -I tools/sonar_to_yolo.py check --root "$ROOT" --out "$OUT" || FAILURES=$((FAILURES + 1))

say "--- COCO validation (pycocotools $("$COCO_VENV" -c 'import pycocotools;print(pycocotools.__version__ if hasattr(pycocotools,"__version__") else "2.x")' 2>/dev/null)) ---"
mapfile -t COCO_JSON < <(find "$OUT" -name _annotations.coco.json | sort)
if [ "${#COCO_JSON[@]}" -gt 0 ]; then
  nice -n 10 "$COCO_VENV" -I tools/verify_coco.py "${COCO_JSON[@]}" || FAILURES=$((FAILURES + 1))
else
  say "no COCO json found"
fi

say "--- Ultralytics dataset build, no model, no GPU ---"
mkdir -p "$SCRATCH/ulcfg" "$SCRATCH/runs"
export YOLO_CONFIG_DIR="$SCRATCH/ulcfg"
for Y in "$OUT"/dataset_yolo*/data.yaml; do
  say "  $Y"
  nice -n 10 "$UL_VENV" -I tools/verify_yolo_ultralytics.py "$Y" 2>&1 | grep -vE "Scanning|it/s" \
    || FAILURES=$((FAILURES + 1))
done

say "--- smoke test: yolo segment train, 1 epoch, CPU ---"
# device=cpu on purpose: the only CUDA compute client on this box is the f1tenth telegram service holding
# ~2 GB, and the project rule is no overlapping GPU jobs. fraction truncates the sorted file list rather than
# sampling, so this trains on the first few scenarios only -- it proves the dataset loads, nothing more.
( cd "$SCRATCH" && nice -n 10 "$UL_VENV" -c "
from ultralytics import YOLO
import sys
m = YOLO('yolo11n-seg.pt')
m.train(data='$OUT/dataset_yolo/data.yaml', epochs=1, imgsz=640, batch=2, workers=2, device='cpu',
        rect=True, cache=False, plots=False, fraction=0.004, project='$SCRATCH/runs', name='smoke',
        exist_ok=True, val=True)
print('SMOKE OK')
" 2>&1 | grep -vE "Scanning|it/s|^ *$" | tail -25 ) || FAILURES=$((FAILURES + 1))

say "--- RF-DETR ---"
if "$UL_VENV" -c "import rfdetr" 2>/dev/null; then
  say "rfdetr is importable; run its loader by hand"
else
  say "rfdetr is installed in no venv on this box, so the COCO half is verified structurally only "
  say "(pycocotools + the format contract read from rf-detr 1.11.2). Not installed: that needs your word."
fi

say "done, $FAILURES verification step(s) reported a failure"
say "output: $OUT   $(du -sh "$OUT" 2>/dev/null | cut -f1)"
exit "$FAILURES"
