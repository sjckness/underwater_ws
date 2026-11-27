#!/usr/bin/env bash
# Installs MAVROS for ROS 2 Jazzy on the Thor (apt, system-wide) and the GeographicLib datasets it needs at start-up.
# Idempotent: every step checks first and skips what is there. Copy to the Thor and run as the normal user:
#     bash ~/thor_mavros_install.sh
# It calls sudo itself, only for the two steps that need it (apt install, datasets into /usr/share/GeographicLib), and
# asks before each of them. It touches no ROS environment variable and nothing of the F1Tenth setup; the packages are
# new files under /opt/ros/jazzy (ros-jazzy-mavros*, plus their dependencies that are not yet installed).
set -eo pipefail

PKGS=(ros-jazzy-mavros ros-jazzy-mavros-extras)
GEO_DIR=/usr/share/GeographicLib
GEO_SCRIPT=/opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

echo "[mavros-install] $(uname -m), $(. /etc/os-release && echo "$PRETTY_NAME")"
[ -f /opt/ros/jazzy/setup.bash ] || { echo "[mavros-install] /opt/ros/jazzy not found: this script is for Jazzy"; exit 1; }

# 1. packages ----------------------------------------------------------------------------------------------------
missing=()
for p in "${PKGS[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
done
if [ ${#missing[@]} -eq 0 ]; then
    echo "[mavros-install] already installed: ${PKGS[*]} ($(dpkg-query -W -f='${Version}' ros-jazzy-mavros))"
else
    for p in "${missing[@]}"; do
        cand="$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate:/ {print $2}')"
        if [ -z "$cand" ] || [ "$cand" = "(none)" ]; then
            echo "[mavros-install] $p has no installation candidate on this machine (arch $(dpkg --print-architecture))."
            echo "                 Check that the ROS 2 apt source is present and run: sudo apt update"
            exit 1
        fi
        echo "[mavros-install] $p candidate $cand"
    done
    read -r -p "[mavros-install] sudo apt install ${missing[*]} ? [y/N] " ans
    [ "$ans" = "y" ] || { echo "[mavros-install] stopped, nothing installed"; exit 1; }
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"
fi

# 2. GeographicLib datasets (geoid for altitude, magnetic model) --------------------------------------------------
# MAVROS exits at start-up if the EGM96 geoid is missing. The ROS script downloads them (needs internet on the Thor).
if [ -f "$GEO_DIR/geoids/egm96-5.pgm" ] && [ -d "$GEO_DIR/magnetic" ] && [ -n "$(ls -A "$GEO_DIR/magnetic" 2>/dev/null)" ]; then
    echo "[mavros-install] GeographicLib datasets already in $GEO_DIR"
else
    [ -f "$GEO_SCRIPT" ] || { echo "[mavros-install] $GEO_SCRIPT missing: is ros-jazzy-mavros installed?"; exit 1; }
    read -r -p "[mavros-install] sudo $GEO_SCRIPT (downloads ~10 MB into $GEO_DIR) ? [y/N] " ans
    [ "$ans" = "y" ] || { echo "[mavros-install] datasets skipped: MAVROS will not start without them"; exit 1; }
    sudo bash "$GEO_SCRIPT"
fi

# 3. check -----------------------------------------------------------------------------------------------------
set +e
( source /opt/ros/jazzy/setup.bash && ros2 pkg prefix mavros && ros2 pkg prefix mavros_extras && ros2 pkg prefix mavros_msgs )
rc=$?
[ $rc -eq 0 ] && echo "[mavros-install] OK" || echo "[mavros-install] ros2 does not see the packages (exit $rc)"
exit $rc
