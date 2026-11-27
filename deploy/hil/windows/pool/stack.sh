#!/usr/bin/env bash
# M11 pool on the Work machine: the WSL2 half of scenario B (hil/win/pool/README.md). Isaac and the physics process
# run on Windows (run_pool.bat); here, in WSL's host networking with mirrored networking (127.0.0.1 is Windows'),
# run ArduSub SITL, mediamtx, BlueOS and the Water Linked DVL extension (docker/pool/compose.yaml, profile "work").
#
# Inside the WSL distribution, from anywhere in the repo, or from Windows through run_pool.bat:
#   bash hil/win/pool/stack.sh up            the stack up (the SITL image is built the first time, ~3 min)
#   bash hil/win/pool/stack.sh restart-sitl  a fresh ArduSub for a fresh physics process (docs/pool.md: before every
#                                            uw pool, or its EKF keeps the old vehicle's heading)
#   bash hil/win/pool/stack.sh configure     BlueOS's Manual board, the video stream and the DVL extension; waits for
#                                            the vehicle (uw pool up), up to WAIT seconds (default 600).
#                                            Log: output/pool/blueos_setup.log
#   bash hil/win/pool/stack.sh status        what answers, link by link
#   bash hil/win/pool/stack.sh smoke [--dvl] tools/pool_smoke.py: flies the vehicle through SITL and checks it
#   bash hil/win/pool/stack.sh logs [svc]    the last lines of the containers' logs
#   bash hil/win/pool/stack.sh down          everything off (BlueOS's settings, SITL's eeprom stay in volumes)
set -euo pipefail
cd "$(dirname "$0")/../../.."
COMPOSE=(docker compose -f docker/pool/compose.yaml --profile work)
BLUEOS=http://127.0.0.1
DVL_PAGE=http://127.0.0.1:9001
RTSP=rtsp://127.0.0.1:8654/rov

probe() {   # probe NAME URL: one line, ok or the reason
    local out
    if out=$(curl -fsS -m 3 "$2" 2>&1); then
        printf '  [ok]   %-34s %s\n' "$1" "$(printf '%s' "$out" | tr -d '\n' | cut -c1-90)"
    else
        printf '  [FAIL] %-34s %s\n' "$1" "$(printf '%s' "$out" | tail -1 | cut -c1-90)"
    fi
}

case "${1:-}" in
    up)
        docker image inspect uw-ardusub:4.5.7 >/dev/null 2>&1 || "${COMPOSE[@]}" build sitl
        "${COMPOSE[@]}" up -d
        ;;
    restart-sitl)
        "${COMPOSE[@]}" restart sitl
        ;;
    configure)
        shift
        mkdir -p output/pool
        python3 tools/pool_blueos_setup.py --blueos "$BLUEOS" --rtsp "$RTSP" --dvl "$DVL_PAGE" \
            --wait "${WAIT:-600}" "$@" 2>&1 | tee output/pool/blueos_setup.log
        ;;
    status)
        echo "networking: $(wslinfo --networking-mode 2>/dev/null || echo unknown) (mirrored expected)"
        docker ps -a --filter name=uw-pool --format '  {{.Names}}: {{.Status}}'
        probe "BlueOS (WSL, port 80)" "$BLUEOS/ardupilot-manager/v1.0/board"
        probe "ArduSub heartbeat in BlueOS" "$BLUEOS/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT"
        probe "A50 emulated (Windows, 8061)" "http://127.0.0.1:8061/api/v1/about"
        probe "DVL extension (WSL, 9001)" "$DVL_PAGE/get_status"
        probe "DVL gateway (bridge .3)" "http://172.31.250.3/cable-guy/v1.0/ethernet"
        probe "camera publishing (mediamtx)" "http://127.0.0.1:9997/v3/paths/get/rov"
        probe "BlueOS video streams" "$BLUEOS/mavlink-camera-manager/streams"
        ;;
    smoke)
        shift
        python3 tools/pool_smoke.py "$@"
        ;;
    logs)
        shift
        "${COMPOSE[@]}" logs --tail 40 "$@"
        ;;
    down)
        "${COMPOSE[@]}" down
        ;;
    *)
        sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
        exit 2
        ;;
esac
