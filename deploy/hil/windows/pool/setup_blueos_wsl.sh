#!/usr/bin/env bash
# M11: prepare an Ubuntu WSL2 distribution on the Work machine for the pool stack (BlueOS, ArduSub SITL, mediamtx,
# the DVL extension: hil/win/pool/README.md). Run INSIDE the distribution (e.g. `wsl -d Ubuntu-24.04`), from the
# repo: bash hil/win/pool/setup_blueos_wsl.sh
#
# What it does, and nothing else: enables systemd in /etc/wsl.conf, installs Docker Engine from Docker's apt
# repository (not Docker Desktop: BlueOS needs real host networking), adds you to the docker group, installs the
# Python the setup and smoke tools need (python3-pip for pymavlink, numpy, yaml), and creates the BlueOS state
# directories under ~/.config/uw_blueos. Then checks what scenario B depends on. Idempotent. Needs sudo and network.
set -euo pipefail

if ! grep -qi microsoft /proc/version; then
    echo "[setup] this is not WSL: on a Linux host install Docker as usual" >&2
    exit 1
fi

# 1. systemd (docker as a service, BlueOS's journald binds)
if ! grep -q "^systemd=true" /etc/wsl.conf 2>/dev/null; then
    printf '[boot]\nsystemd=true\n' | sudo tee -a /etc/wsl.conf >/dev/null
    echo "[setup] systemd enabled: run 'wsl --shutdown' in Windows, reopen the distro and run this script again"
    exit 0
fi

# 2. Docker Engine from Docker's repository
if ! command -v docker >/dev/null 2>&1 || docker version 2>&1 | grep -qi "docker desktop"; then
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    . /etc/os-release
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${VERSION_CODENAME} stable" \
        | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi
sudo systemctl enable --now docker
# tools/pool_blueos_setup.py is stdlib only; tools/pool_smoke.py imports uw (numpy, yaml) and installs pymavlink
# with pip --target into ~/.cache/uw_pydeps on first use (uw/deps.py)
sudo apt-get install -y python3-pip python3-numpy python3-yaml curl
if ! id -nG "$USER" | grep -qw docker; then
    sudo usermod -aG docker "$USER"
    echo "[setup] added $USER to the docker group: close and reopen the distro"
fi

# 3. BlueOS state, outside the repo (the compose file mounts these)
mkdir -p ~/.config/uw_blueos/{config,etc,bin,extensions,userdata,logs}
[ -s /etc/machine-id ] || sudo systemd-machine-id-setup

# 4. the checks that matter for scenario B
mode=$(wslinfo --networking-mode 2>/dev/null || echo "unknown (wslinfo missing)")
echo "[setup] networking mode: $mode"
[ "$mode" = mirrored ] || echo "[setup] WARNING: not mirrored: copy hil/win/pool/wslconfig.example to %UserProfile%\.wslconfig, wsl --shutdown"
# Ports the WSL side opens (BlueOS's main ones, mediamtx, the DVL extension). In mirrored mode a port Windows
# already holds cannot be had here either, and `ss` only sees Linux: ask Windows too. Meant for before
# `stack.sh up`: with the stack running, its own ports may show up on the Windows side as well.
ports="80 8000 6020 6021 8554 8654 8889 9001 9997 14550 14551 18000 18001"
win=$(/mnt/c/Windows/System32/netstat.exe -ano 2>/dev/null | tr -d '\r' | awk '$1 ~ /TCP|UDP/ {print $2}' || true)
for p in $ports; do
    if ss -Hltnu "( sport = :$p )" | grep -q .; then echo "[setup] note: port $p in use in WSL (fine if it is the uw-pool stack)"; fi
    if printf '%s\n' "$win" | grep -Eq "^(0\.0\.0\.0|127\.0\.0\.1|\[::\]|\[::1\]):$p\$"; then
        echo "[setup] WARNING: port $p is taken on Windows: netstat -ano | findstr :$p (then tasklist /fi \"pid eq <PID>\")"
    fi
done
# Line endings: the repo is shared with Windows; a CRLF here breaks the SITL image's entrypoint and these scripts
for f in docker/ardusub_sitl/entrypoint.sh docker/ardusub_sitl/params/pool.parm docker/pool/compose.yaml \
         docker/pool/dvl_gateway.conf.template hil/win/pool/stack.sh; do
    if [ -f "$f" ] && grep -q $'\r' "$f"; then
        echo "[setup] WARNING: $f has CRLF line endings: see hil/win/pool/README.md, 'Fine riga'"
    fi
done
echo "[setup] done. Next (hil/win/pool/README.md): bash hil/win/pool/stack.sh up"
