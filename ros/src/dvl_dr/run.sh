#!/usr/bin/env bash
# The preprocessor without building: sources the host's ROS 2 Jazzy and runs it on the tank sim's domain
# (UW_ROS_DOMAIN_ID, default 73). The filter itself is robot_localization, started by the launch file.
set -eo pipefail
source /opt/ros/jazzy/setup.bash          # not under `set -u`: the setup scripts read unset variables
exec python3 "$(dirname "$(readlink -f "$0")")/dvl_dr/preprocess.py" "$@"
