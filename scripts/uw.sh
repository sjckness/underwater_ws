#!/bin/bash
# Underwater tank sim: host wrapper around the Isaac Sim container (docker/compose.yaml).
#   scripts/uw.sh py SCRIPT [args]   run SCRIPT with Isaac's Python 3.11 + Warp 1.8.2 (also py -m MOD, py -c CODE)
#   scripts/uw.sh test [args]        pytest on tests/ in the container (Warp tests on the GPU)
#   scripts/uw.sh sim [--gui]        full Isaac app with camera and ROS 2 (M3)
#   scripts/uw.sh sonar-only         Warp-only bubbles + sonar, no RTX (M2/M3)
# Config options go after the command and reach the Python side unchanged:
#   --profile linus|workstation   --preset NAME (repeatable)   --set a.b=c (repeatable)
# On the host it starts a one-off container and removes it on exit, Ctrl-C, SIGTERM or SIGHUP; inside the
# container (UW_IN_CONTAINER=1) it runs the command under /isaac-sim/python.sh.
# Environment: UW_ROS_DOMAIN_ID (default 73, see README.md).
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
WARP_PATH=/isaac-sim/extscache/omni.warp.core-1.8.2+lx64
PYTEST_VERSION=7.4.4        # the host's pytest (Ubuntu 24.04), so tests behave the same on both sides
STOP_TIMEOUT=15             # docker stop grace period [s] before SIGKILL

usage() { sed -n '2,11p' "${BASH_SOURCE[0]}" | cut -c3-; }
die() { echo "uw.sh: $*" >&2; exit 2; }

CMD="${1:-}"
[ $# -gt 0 ] && shift
case "$CMD" in
    py|test) ;;
    sim) echo "uw.sh sim: not implemented until M3" >&2; exit 1 ;;
    sonar-only) echo "uw.sh sonar-only: not implemented until M2/M3" >&2; exit 1 ;;
    -h|--help|help) usage; exit 0 ;;
    "") usage >&2; exit 2 ;;
    *) usage >&2; die "unknown command '$CMD'" ;;
esac

if [ "${UW_IN_CONTAINER:-}" != 1 ]; then
    # ---- host side ----
    # The workspace is mounted at the same path in the container: make a script path absolute, and keep the
    # caller's directory (when inside the workspace) so that relative paths in the arguments still resolve.
    if [ "$CMD" = py ] && [ $# -gt 0 ] && [ -f "$1" ]; then
        script="$(realpath "$1")"
        case "$script" in "$WS"/*) ;; *) die "$1 is outside $WS, the only directory mounted in the container" ;; esac
        set -- "$script" "${@:2}"
    fi
    case "$(pwd -P)/" in "$WS"/*) workdir="$(pwd -P)" ;; *) workdir="$WS" ;; esac
    mkdir -p "$HOME/.cache/uw_warp" "$HOME/.cache/uw_pydeps"      # docker would create them root-owned
    UW_UID="$(id -u)" UW_GID="$(id -g)"
    export UW_WS="$WS" UW_UID UW_GID
    name="uw-$CMD-$$"
    # The container client runs in the background so that these traps fire while it is up. With a terminal, it
    # puts the terminal in raw mode and Ctrl-C goes straight into the container. Without one, Ctrl-C (SIGINT to
    # the process group) must reach only this script, which stops the container cleanly while the client stays
    # attached for the last output: the client gets its own session (it would otherwise exit at once on SIGINT).
    tty=() session=()
    { [ -t 0 ] && [ -t 1 ]; } || { tty=(-T); session=(setsid); }
    stop() {
        trap - INT TERM HUP             # a second Ctrl-C kills this script, and the EXIT trap removes the container
        echo "uw.sh: stopping $name (Ctrl-C again to kill it)" >&2
        docker stop -t "$STOP_TIMEOUT" "$name" >/dev/null 2>&1 &
        wait $! || true                 # in the background: a foreground command would hold off the second Ctrl-C
    }
    trap stop INT TERM HUP
    trap 'docker rm -f "$name" >/dev/null 2>&1 || true' EXIT
    "${session[@]}" docker compose -f "$WS/docker/compose.yaml" run --rm "${tty[@]}" --name "$name" -w "$workdir" \
        uw "$WS/scripts/uw.sh" "$CMD" "$@" <&0 &
    cli=$!
    while :; do
        rc=0
        wait "$cli" || rc=$?
        kill -0 "$cli" 2>/dev/null || break       # wait was cut short by a trapped signal: keep waiting
    done
    exit "$rc"
fi

# ---- inside the container ----
[ -d "$WARP_PATH" ] || die "Warp not found at $WARP_PATH (did the image change?)"
export PYTHONPATH="$WS:$WARP_PATH"
if [ "$CMD" = test ]; then
    # The image has no pytest: install it once, pinned, into ~/.cache/uw_pydeps (outside the image and the repo).
    deps="/uw_cache/pydeps/pytest-$PYTEST_VERSION"
    if [ ! -d "$deps" ]; then
        echo "uw.sh: installing pytest $PYTEST_VERSION into ~/.cache/uw_pydeps (first run only, needs network)" >&2
        rm -rf "${deps:?}.tmp"
        # --no-warn-conflicts: pip would complain about Isaac's own bundled packages, which are not ours to fix
        /isaac-sim/python.sh -m pip install --quiet --no-cache-dir --disable-pip-version-check --no-warn-conflicts \
            --target "$deps.tmp" "pytest==$PYTEST_VERSION" || die "installing pytest failed"
        mv -T "$deps.tmp" "$deps"
    fi
    # no plugin autoload: Isaac's bundled site-packages must not inject pytest plugins
    export PYTHONPATH="$deps:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    set -- -m pytest "$@"
fi

# Ctrl-C and docker stop reach Python as KeyboardInterrupt, so finally blocks run and pytest prints its summary.
# python.sh does not exec, and it runs in the background so that this shell can trap SIGTERM; a background job
# starts with SIGINT ignored, hence env --default-signal. A terminal's Ctrl-C reaches Python directly; docker
# stop's SIGTERM is forwarded to it as SIGINT.
py=
trap : INT
trap '[ -n "$py" ] && pkill -INT -P "$py" || true' TERM
env --default-signal=INT /isaac-sim/python.sh "$@" <&0 &
py=$!
while :; do
    rc=0
    wait "$py" || rc=$?
    kill -0 "$py" 2>/dev/null || break
done
exit "$rc"
