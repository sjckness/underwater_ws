#!/usr/bin/env bash
# M11: prepare an Ubuntu WSL2 distribution on the Work machine for the pool stack (BlueOS, ArduSub SITL, mediamtx).
# Run INSIDE the distribution (e.g. `wsl -d Ubuntu-24.04`), from the repo: bash hil/win/pool/setup_blueos_wsl.sh
#
# What it does, and nothing else: enables systemd in /etc/wsl.conf, installs Docker Engine from Docker's apt
# repository (not Docker Desktop: BlueOS needs real host networking), adds you to the docker group, and creates the
# BlueOS state directories under ~/.config/uw_blueos. Idempotent. Needs sudo and network.
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
if ! id -nG "$USER" | grep -qw docker; then
    sudo usermod -aG docker "$USER"
    echo "[setup] added $USER to the docker group: close and reopen the distro"
fi

# 3. BlueOS state, outside the repo (the compose file mounts these)
mkdir -p ~/.config/uw_blueos/{config,etc,bin,extensions,userdata,logs}
[ -s /etc/machine-id ] || sudo systemd-machine-id-setup

# 4. the checks that matter for mirrored networking
echo "[setup] networking mode: $(wslinfo --networking-mode 2>/dev/null || echo 'unknown (wslinfo missing)')"
for p in 80 8000 6020 6021 8554 14550 14551; do
    if ss -Hltnu "( sport = :$p )" | grep -q .; then echo "[setup] WARNING: port $p already in use"; fi
done
echo "[setup] done. Next (docs/pool.md): docker compose -f docker/pool/compose.yaml --profile blueos up -d"
