#!/bin/bash
# Starts ArduSub SITL with the JSON physics backend. Everything is configured through environment variables (set by
# run.bat), so the image stays generic. Extra arguments are passed on to ardusub.
#
#   PHYSICS_HOST   host name or IP of the physics simulator            (default host.docker.internal)
#   PHYSICS_PORT   not used here: ArduSub's JSON backend sends to UDP 9002 (hard-wired, --sim-port-out to change)
#   MAVLINK_OUT    ardusub --serial0 device: where SITL pushes MAVLink  (default udpclient:192.168.200.10:14550)
#   HOME_LOC       lat,lon,alt,heading of the start point                (default 45.0703,7.6869,0,0)
#   PARAM_FILES    extra comma-separated defaults files                  (default /params/hil.parm if present)
#   SPEEDUP        SIM_SPEEDUP                                            (default 1)
#   WIPE           1 = wipe the eeprom (parameters) at every start       (default 1)
set -eu

PHYSICS_HOST="${PHYSICS_HOST:-host.docker.internal}"
MAVLINK_OUT="${MAVLINK_OUT:-udpclient:192.168.200.10:14550}"
HOME_LOC="${HOME_LOC:-45.0703,7.6869,0,0}"
SPEEDUP="${SPEEDUP:-1}"
WIPE="${WIPE:-1}"

# ArduPilot's socket code wants an IPv4 literal for --model JSON:<addr>, not a name: resolve it here.
PHYSICS_IP="$(getent ahostsv4 "$PHYSICS_HOST" | awk 'NR==1 {print $1}')"
if [ -z "$PHYSICS_IP" ]; then
    echo "[entrypoint] cannot resolve PHYSICS_HOST=$PHYSICS_HOST" >&2
    exit 1
fi

DEFAULTS="/opt/ardupilot/default_params/sub.parm"
if [ -n "${PARAM_FILES:-}" ]; then
    DEFAULTS="$DEFAULTS,$PARAM_FILES"
elif [ -f /params/hil.parm ]; then
    DEFAULTS="$DEFAULTS,/params/hil.parm"
fi

ARGS=(--model "JSON:${PHYSICS_IP}" --speedup "$SPEEDUP" --home "$HOME_LOC" --defaults "$DEFAULTS" --serial0 "$MAVLINK_OUT")
[ "$WIPE" = "1" ] && ARGS+=(--wipe)

echo "[entrypoint] ArduSub $(cat /opt/ardupilot/COMMIT)"
echo "[entrypoint] physics  : JSON backend -> ${PHYSICS_HOST} (${PHYSICS_IP}) UDP 9002"
echo "[entrypoint] mavlink  : ${MAVLINK_OUT}"
echo "[entrypoint] defaults : ${DEFAULTS}"
exec /opt/ardupilot/bin/ardusub "${ARGS[@]}" "$@"
