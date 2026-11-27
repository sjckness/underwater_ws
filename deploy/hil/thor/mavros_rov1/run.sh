#!/usr/bin/env bash
# Starts MAVROS on the Thor for the ArduSub SITL running on the work machine, under /<vehicle>/mavros/**.
# Run it in the dedicated HIL terminal (it sources ~/hil_env.sh: domain 73, the direct cable's Fast DDS
# profile; the F1Tenth terminals are not affected).
#     bash run.sh                                  # sim time (Isaac running and publishing /clock)
#     USE_SIM_TIME=false bash run.sh               # smoke test without Isaac: no /clock, sim time would stall
#     VEHICLE=rov2 FCU_URL=udp://192.168.200.10:14550@ bash run.sh
#
# Link: SITL PUSHES MAVLink to the Thor (ardusub --serial0 udpclient:192.168.200.10:14550). MAVROS therefore
# only binds UDP 14550 on the Thor's end of the cable and learns the work machine's address from the first
# packet ("udp://BIND_HOST:PORT@", empty remote part). Nothing is bound on any other interface.
#
# Read-only telemetry for the DVL dead reckoning (imu/data, imu/static_pressure, state). Commanding is M8b.
set -eo pipefail

HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$HOME/hil_env.sh"

VEHICLE="${VEHICLE:-rov1}"
FCU_URL="${FCU_URL:-udp://192.168.200.10:14550@}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "[mavros-run] mavros not found: run install.sh first"; exit 1
fi
if ss -uln 2>/dev/null | awk '{print $5}' | grep -q ':14550$'; then
    echo "[mavros-run] UDP 14550 is already bound on this machine (another MAVROS or a GCS?). Stop it first."; exit 1
fi
echo "[mavros-run] vehicle=$VEHICLE fcu_url=$FCU_URL use_sim_time=$USE_SIM_TIME ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 launch "$HERE/mavros.launch.py" vehicle:="$VEHICLE" fcu_url:="$FCU_URL" use_sim_time:="$USE_SIM_TIME"
