#!/bin/bash
# Thin Linux wrapper: `scripts/uw.sh CMD [args]` is `python3 -m uw CMD [args]` (uw/launcher.py has the details).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec env PYTHONPATH="$WS${PYTHONPATH:+:$PYTHONPATH}" python3 -m uw "$@"
