#!/usr/bin/env bash
# The detector without building: sources the host's ROS 2 Jazzy and runs it with a Python that has Ultralytics,
# UW_YOLO_PYTHON (default ~/.cache/uw_yolo/venv/bin/python, made from requirements.txt), on the sim's domain
# (UW_ROS_DOMAIN_ID, default 73). Arguments go to the detector (--help); by default it reads the pool's stream.
set -eo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
PY="${UW_YOLO_PYTHON:-$HOME/.cache/uw_yolo/venv/bin/python}"
if [ ! -x "$PY" ]; then
  echo "yolo_detector: no Python with Ultralytics at $PY" >&2
  echo "  make one:   python3 -m venv ~/.cache/uw_yolo/venv && ~/.cache/uw_yolo/venv/bin/pip install -r $HERE/requirements.txt" >&2
  echo "  or reuse:   export UW_YOLO_PYTHON=/path/to/venv/bin/python" >&2
  exit 1
fi
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash        # not under `set -u`: the setup scripts read unset variables
fi
export PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}"
exec "$PY" -m yolo_detector.detector "$@"
