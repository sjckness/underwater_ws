"""The pool on the Work machine in one command (M12): checks, one-time setup, assets, then run_pool.bat.

    pool.bat                    (repo root; runs this file with the Isaac environment's Python)
    pool.bat --gui              every option after the ones below goes to `uw pool` (--gui, --preset turbid, ...)
    pool.bat --check            only report what is ready and what is not; change nothing
    pool.bat --no-props         do not fetch the COCO objects' models (the pool draws boxes in their place)
    pool.bat --no-cockpit       do not open Cockpit

What it does, each step only when needed, so that the second run goes straight to the pool:
 1. the repo: the files WSL reads must have LF line endings (fixed by re-checkout when `git status` is clean);
 2. WSL: version 2 with mirrored networking (%UserProfile%\\.wslconfig gets the keys of wslconfig.example that it lacks,
    then `wsl --shutdown`); the distribution UW_WSL_DISTRO (Ubuntu-24.04), installed if missing (it asks for a user);
 3. inside it, Docker Engine and systemd: setup_blueos_wsl.sh, restarting the distribution when that script asks for it
    (systemd just enabled, the docker group just joined). sudo asks for the WSL password the first time;
 4. the Windows ports the stack and Isaac need (BlueOS's 80 and 8000 first);
 5. the 3-D models: the ROV (vehicles/bluerov2_heavy/fetch_bluerov2.py) and the COCO objects (sim/tools/fetch_props.py);
 6. Cockpit: the desktop app if installed (UW_COCKPIT, or the usual install folders), else BlueOS in the browser;
 7. run_pool.bat: the WSL stack up, a fresh SITL, BlueOS and the DVL configured in the background, Isaac.

Windows only for the actions; the decisions are plain functions (tests/test_pool_up.py runs them on linus).
Standard library only: it runs before anything else is known to be installed.
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
DISTRO = os.environ.get("UW_WSL_DISTRO", "Ubuntu-24.04")
WSLCONFIG = Path(os.environ.get("USERPROFILE", str(Path.home()))) / ".wslconfig"
# What the WSL side reads from this checkout (CRLF there breaks bash and the SITL entrypoint)
LF_PATHS = ("deploy/docker", "deploy/hil/windows/pool")
# Windows ports: BlueOS in WSL's host network (mirrored) and the Isaac side
STACK_PORTS = {80: "BlueOS", 8000: "BlueOS ardupilot-manager", 8654: "mediamtx RTSP", 8889: "mediamtx WebRTC",
               14551: "BlueOS MAVLink"}
ISAAC_PORTS = {9002: "physics (UDP)", 16171: "A50 emulated", 8061: "A50 HTTP API", 9020: "control channel"}
ROV_ASSET = REPO / "vehicles" / "bluerov2_heavy" / "assets" / "bluerov2_heavy.usd"
COCKPIT_CANDIDATES = (r"%LOCALAPPDATA%\Programs\Cockpit\Cockpit.exe", r"%ProgramFiles%\Cockpit\Cockpit.exe",
                      r"%LOCALAPPDATA%\Programs\cockpit\Cockpit.exe")


# ------------------------------------------------------------------------------------------------ decisions

def decode(raw: bytes) -> str:
    """wsl.exe writes UTF-16LE (with NULs between the letters) for its own messages, UTF-8 for a Linux command's."""
    if b"\x00" in raw:
        return raw.decode("utf-16-le", errors="replace").replace("﻿", "")
    return raw.decode("utf-8", errors="replace")


def wsl_major_version(text: str) -> int | None:
    """``wsl --version``'s first line, "WSL version: 2.3.26.0" (localised label, so only the number counts)."""
    m = re.search(r"(\d+)\.\d+\.\d+", text)
    return int(m.group(1)) if m else None


def distros(text: str) -> list[str]:
    """``wsl -l -q``: one name per line."""
    return [ln.strip() for ln in text.splitlines() if ln.strip()]


def merge_wslconfig(current: str | None, example: str) -> tuple[str, list[str]]:
    """The .wslconfig with every key of the example present: a missing file becomes the example; an existing one
    keeps its own keys and values and gets the missing ones appended to their section (created if absent).
    Returns (text, the keys added as "section.key")."""
    if current is None:
        return example, ["(the whole example)"]

    def keys(text):
        out, sec = {}, None
        for ln in text.splitlines():
            s = ln.strip()
            if s.startswith("[") and s.endswith("]"):
                sec = s[1:-1].strip().lower()
                out.setdefault(sec, {})
            elif sec is not None and "=" in s and not s.startswith(("#", ";")):
                k, v = s.split("=", 1)
                out[sec][k.strip().lower()] = (k.strip(), v.strip())
        return out

    have, want = keys(current), keys(example)
    lines = current.splitlines()
    added = []
    for sec, kv in want.items():
        missing = [orig for k, orig in kv.items() if k not in have.get(sec, {})]
        if not missing:
            continue
        added += [f"{sec}.{k}" for k, _ in missing]
        new = [f"{k}={v}" for k, v in missing]
        idx = next((i for i, ln in enumerate(lines) if ln.strip().lower() == f"[{sec}]"), None)
        if idx is None:
            lines += ["", f"[{sec}]"] + new
        else:
            end = idx + 1
            while end < len(lines) and not lines[end].strip().startswith("["):
                end += 1
            while end > idx + 1 and not lines[end - 1].strip():
                end -= 1
            lines[end:end] = new
    return "\n".join(lines) + "\n", added


def networking_mode(wslconfig: str) -> str | None:
    """``[wsl2] networkingMode`` of a .wslconfig, as written (None when absent)."""
    sec = None
    for ln in wslconfig.splitlines():
        s = ln.strip()
        if s.startswith("[") and s.endswith("]"):
            sec = s[1:-1].strip().lower()
        elif sec == "wsl2" and "=" in s and not s.startswith(("#", ";")):
            k, v = s.split("=", 1)
            if k.strip().lower() == "networkingmode":
                return v.strip()
    return None


def crlf_files(ls_files_eol: str) -> list[str]:
    """``git ls-files --eol``: the files checked out with CRLF although git keeps them LF."""
    out = []
    for ln in ls_files_eol.splitlines():
        info, sep, path = ln.partition("\t")                 # "i/lf w/crlf attr/text eol=lf<TAB>path"
        parts = info.split()
        if sep and len(parts) >= 2 and parts[0] == "i/lf" and parts[1] == "w/crlf":
            out.append(path.strip())
    return out


def listening_ports(netstat: str) -> dict[int, str]:
    """``netstat -ano``: local port -> PID, for listening TCP sockets and bound UDP ones on any or loopback."""
    out = {}
    for ln in netstat.splitlines():
        p = ln.split()
        if len(p) >= 4 and p[0] in ("TCP", "UDP"):
            addr = p[1]
            host, _, port = addr.rpartition(":")
            if host not in ("0.0.0.0", "127.0.0.1", "[::]", "[::1]") or not port.isdigit():
                continue
            if p[0] == "TCP" and (len(p) < 5 or p[3] != "LISTENING"):
                continue
            out[int(port)] = p[-1]
    return out


def missing_props(cfg_text_ids: list[str], props_dir: Path) -> list[str]:
    return [i for i in cfg_text_ids if not (props_dir / i / f"{i}.usd").is_file()]


# -------------------------------------------------------------------------------------------------- actions

class Step:
    def __init__(self, check_only: bool):
        self.check_only = check_only
        self.problems: list[str] = []

    def ok(self, what: str) -> None:
        print(f"[pool] ok    {what}", flush=True)

    def do(self, what: str) -> bool:
        """Announce an action; False in --check mode (the caller then only records it)."""
        print(f"[pool] {'TODO ' if self.check_only else 'fix  '} {what}", flush=True)
        if self.check_only:
            self.problems.append(what)
        return not self.check_only

    def fail(self, what: str, how: str) -> None:
        print(f"[pool] FAIL  {what}\n             {how}", flush=True)
        raise SystemExit(1)


def run(cmd, **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, **kw)


def wsl(*args, distro: str | None = DISTRO, interactive: bool = False) -> subprocess.CompletedProcess:
    cmd = ["wsl"] + (["-d", distro] if distro else []) + list(args)
    if interactive:                                      # sudo prompts, the distro's first-user dialogue
        return subprocess.run(cmd)
    return run(cmd)


def wsl_bash(script: str, interactive: bool = False) -> subprocess.CompletedProcess:
    return wsl("--cd", str(REPO), "--", "bash", "-lc", script, interactive=interactive)


def check_repo(st: Step) -> None:
    if shutil.which("git") is None:
        print("[pool] note  git is not on PATH: line endings not checked (README §2.2)", flush=True)
        return
    eol = run(["git", "ls-files", "--eol", *LF_PATHS], cwd=REPO)
    bad = crlf_files(decode(eol.stdout))
    if not bad:
        st.ok("line endings (LF where WSL reads)")
        return
    dirty = decode(run(["git", "status", "--porcelain"], cwd=REPO).stdout).strip()
    if dirty:
        st.fail(f"{len(bad)} files with CRLF ({bad[0]}, ...) and local changes in the checkout",
                "commit nothing on the Work: save the changes elsewhere, then `git rm -rq --cached .` and "
                "`git reset -q --hard` (deploy/hil/windows/pool/README.md, 'Fine riga')")
    if st.do(f"{len(bad)} files with CRLF: checking them out again with LF (.gitattributes)"):
        run(["git", "rm", "-rq", "--cached", "."], cwd=REPO)
        run(["git", "reset", "-q", "--hard"], cwd=REPO)


def check_wsl(st: Step) -> None:
    v = wsl_major_version(decode(run(["wsl", "--version"]).stdout))
    if v is None or v < 2:
        st.fail(f"WSL {v or 'too old'}: the store version 2.x is needed", "`wsl --update` (as administrator), then again")
    st.ok(f"WSL {v}")
    example = (HERE / "wslconfig.example").read_text()
    current = WSLCONFIG.read_text() if WSLCONFIG.is_file() else None
    text, added = merge_wslconfig(current, example)
    mode = networking_mode(text)
    if mode is not None and mode.lower() != "mirrored":
        st.fail(f"{WSLCONFIG} sets networkingMode={mode}; the pool needs mirrored",
                "change it to networkingMode=mirrored (it applies to every WSL distribution), then `wsl --shutdown`")
    if not added:
        st.ok(f"{WSLCONFIG} (mirrored networking)")
    elif st.do(f"{WSLCONFIG}: adding {', '.join(added)}; then wsl --shutdown"):
        if current is not None:
            shutil.copyfile(WSLCONFIG, WSLCONFIG.parent / ".wslconfig.bak")
        WSLCONFIG.write_text(text)
        run(["wsl", "--shutdown"])
    names = distros(decode(run(["wsl", "-l", "-q"]).stdout))
    if DISTRO not in names:
        if st.do(f"the WSL distribution {DISTRO}: installing it (it asks for a Linux user name and password)"):
            subprocess.run(["wsl", "--install", "-d", DISTRO])
            if DISTRO not in distros(decode(run(["wsl", "-l", "-q"]).stdout)):
                st.fail(f"{DISTRO} is not installed", "`wsl --install -d Ubuntu-24.04`, create the user, then again")
    else:
        st.ok(f"distribution {DISTRO}")


def docker_ready() -> tuple[bool, str]:
    r = wsl_bash('[ "$(ps -p 1 -o comm=)" = systemd ] || { echo NO_SYSTEMD; exit 0; }; '
                 'docker info >/dev/null 2>&1 && echo DOCKER_OK || echo NO_DOCKER; '
                 'echo "MODE=$(wslinfo --networking-mode 2>/dev/null)"')
    out = decode(r.stdout)
    return "DOCKER_OK" in out, out


def check_docker(st: Step) -> None:
    for attempt in range(4):
        ready, out = docker_ready()
        if "MODE=" in out and "MODE=mirrored" not in out:
            st.fail(f"WSL networking is not mirrored ({out.split('MODE=')[-1].strip() or 'unknown'})",
                    "Windows 11 22H2 or later is needed; check %UserProfile%\\.wslconfig, then `wsl --shutdown`")
        if ready:
            st.ok("Docker Engine and systemd in WSL")
            return
        if not st.do("Docker and systemd in WSL: setup_blueos_wsl.sh (sudo asks for the WSL password)"):
            return
        wsl_bash("bash deploy/hil/windows/pool/setup_blueos_wsl.sh", interactive=True)
        print(f"[pool]       restarting {DISTRO} (systemd, the docker group)", flush=True)
        run(["wsl", "--terminate", DISTRO])
        time.sleep(3)
    st.fail("Docker is still not usable in WSL", "run `bash deploy/hil/windows/pool/setup_blueos_wsl.sh` in the "
            "distribution and read its messages")


def check_ports(st: Step) -> None:
    held = listening_ports(decode(run(["netstat", "-ano"]).stdout))
    stack_up = "Up" in decode(wsl_bash("docker ps --filter name=uw-pool-blueos --format '{{.Status}}'").stdout)
    for port, what in {**({} if stack_up else STACK_PORTS), **ISAAC_PORTS}.items():
        if port in held:
            pid = held[port]
            name = decode(run(["tasklist", "/fi", f"pid eq {pid}", "/fo", "csv", "/nh"]).stdout).split(",")[0]
            st.fail(f"port {port} ({what}) is taken on Windows by PID {pid} {name}",
                    "close that program (or `uw pool` still running?); for 80 see deploy/hil/windows/pool/README.md §5. "
                    "Nothing is uninstalled or disabled for you")
    st.ok("Windows ports free" + (" (the stack is already up)" if stack_up else ""))


def uw_py(args: list[str]) -> int:
    """`python -m uw py ...` in the native runtime, as run_pool.bat sets it up."""
    env = dict(os.environ, UW_RUNTIME="native", UW_PYTHON=sys.executable, OMNI_KIT_ACCEPT_EULA="YES",
               PIP_CONFIG_FILE="nul")
    return subprocess.run([sys.executable, "-m", "uw", "py", *args], cwd=REPO, env=env).returncode


def placed_object_ids() -> list[str]:
    sys.path.insert(0, str(REPO))
    from physics.pool_geometry import placed_objects
    from uw import config
    return sorted({pl["id"] for pl, _ in placed_objects(config.load(config.POOL_CONFIG, profile="workstation"))})


def check_assets(st: Step, props: bool) -> None:
    if ROV_ASSET.is_file():
        st.ok("the ROV's 3-D model")
    elif st.do("the ROV's 3-D model: vehicles/bluerov2_heavy/fetch_bluerov2.py (network, a minute)"):
        if uw_py(["vehicles/bluerov2_heavy/fetch_bluerov2.py"]) != 0:
            print("[pool] WARN  the ROV model did not convert: the pool shows a box instead", flush=True)
    if not props:
        return
    ids = missing_props(placed_object_ids(), REPO / "assets" / "props")
    if not ids:
        st.ok("the COCO objects' models")
    elif st.do(f"the COCO objects' models ({', '.join(ids)}): sim/tools/fetch_props.py (network, Kit, a few minutes)"):
        if uw_py(["sim/tools/fetch_props.py", *ids]) != 0:
            print("[pool] WARN  some objects did not convert: boxes stand in for them", flush=True)


def find_cockpit() -> Path | None:
    for c in ([os.environ["UW_COCKPIT"]] if os.environ.get("UW_COCKPIT") else []) + list(COCKPIT_CANDIDATES):
        p = Path(os.path.expandvars(c))
        if p.is_file():
            return p
    return None


def open_cockpit(st: Step) -> None:
    exe = find_cockpit()
    if exe is not None:
        st.ok(f"Cockpit: {exe} (vehicle address 127.0.0.1, Settings > General, once)")
        if not st.check_only:
            subprocess.Popen([str(exe)], close_fds=True)
        return
    print("[pool] note  no Cockpit desktop app found (UW_COCKPIT, or install 1.19.0 from github.com/bluerobotics/cockpit/"
          "releases); BlueOS opens in the browser: Extensions > Cockpit > Install", flush=True)
    if not st.check_only:
        os.startfile("http://localhost")  # noqa: S606 - Windows only, the default browser


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="pool.bat", description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--check", action="store_true", help="only report; change nothing")
    ap.add_argument("--no-props", action="store_true", help="do not fetch the COCO objects' models")
    ap.add_argument("--no-cockpit", action="store_true", help="do not open Cockpit")
    args, pool_args = ap.parse_known_args(argv)
    if os.name != "nt":
        raise SystemExit("pool.bat is for the Work machine (Windows); on linus see docs/pool.md, scenario C")
    st = Step(args.check)
    print(f"[pool] repo {REPO}, WSL distribution {DISTRO}", flush=True)
    check_repo(st)
    check_wsl(st)
    check_docker(st)
    check_ports(st)
    check_assets(st, props=not args.no_props)
    if args.check:
        print(f"[pool] {len(st.problems)} things to do" if st.problems else "[pool] all ready: pool.bat starts it")
        return 1 if st.problems else 0
    if not args.no_cockpit:
        open_cockpit(st)
    print("[pool] starting: deploy\\hil\\windows\\pool\\run_pool.bat " + " ".join(pool_args), flush=True)
    return subprocess.run(["cmd", "/c", str(HERE / "run_pool.bat"), *pool_args], cwd=REPO).returncode


if __name__ == "__main__":
    sys.exit(main())
