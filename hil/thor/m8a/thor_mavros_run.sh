#!/usr/bin/env bash
# Starts MAVROS on the Thor for the ArduSub SITL that runs on the work machine. Run it in the dedicated HIL terminal
# (it sources ~/hil_env.sh: domain 73, Fast DDS profile of the direct cable; F1Tenth terminals are not affected).
#     bash ~/thor_mavros_run.sh                       # sim time (Isaac running and publishing /clock)
#     USE_SIM_TIME=false bash ~/thor_mavros_run.sh    # smoke test without Isaac: there is no /clock, sim time would stall
#     FCU_URL=udp://192.168.200.10:14550@ bash ~/thor_mavros_run.sh
#
# Link: SITL PUSHES MAVLink to the Thor (ardusub --serial0 udpclient:192.168.200.10:14550). MAVROS therefore only
# binds UDP 14550 on the Thor's end of the cable and learns the work machine's address from the first packet
# ("udp://BIND_HOST:PORT@" with an empty remote part). Nothing is bound on any other interface.
# Files expected next to this script: thor_mavros.launch.py.
set -eo pipefail

HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$HOME/hil_env.sh"

FCU_URL="${FCU_URL:-udp://192.168.200.10:14550@}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "[mavros-run] mavros not found: run thor_mavros_install.sh first"; exit 1
fi
if ss -uln 2>/dev/null | awk '{print $5}' | grep -q ':14550$'; then
    echo "[mavros-run] UDP 14550 is already bound on this machine (another MAVROS or a GCS?). Stop it first."; exit 1
fi
echo "[mavros-run] fcu_url=$FCU_URL  use_sim_time=$USE_SIM_TIME  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 launch "$HERE/thor_mavros.launch.py" fcu_url:="$FCU_URL" use_sim_time:="$USE_SIM_TIME"
