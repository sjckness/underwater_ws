#!/usr/bin/env bash
# Host relay without building: sources the host's ROS 2 Jazzy and runs the node on the tank sim's domain
# (UW_ROS_DOMAIN_ID, default 73). Arguments go to relay.py (--db, --topic).
set -eo pipefail
source /opt/ros/jazzy/setup.bash          # not under `set -u`: the setup scripts read unset variables
exec python3 "$(dirname "$(readlink -f "$0")")/sonar_relay/relay.py" "$@"
