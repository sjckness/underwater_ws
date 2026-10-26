"""`python -m uw`: the entry point. Runs a command in the Isaac Sim container (Linux) or with a local Python (Windows
venv, native Isaac), as config/runtime.yaml says.

    python -m uw py SCRIPT [args]        also py -m MODULE, py -c CODE
    python -m uw test [pytest args]
    python -m uw sim [--gui]             M3
    python -m uw sonar-only [args]
Config options go after the command and reach the Python side unchanged: --profile linus|workstation,
--preset NAME (repeatable), --set a.b=c (repeatable).

Docker: a one-off container, removed on exit, Ctrl-C, SIGTERM or SIGHUP. With a terminal, Ctrl-C goes straight into
the container; without one it lands here and stops the container (a second Ctrl-C kills it at once). The target
side (uw/runner.py) turns Ctrl-C and docker stop into a KeyboardInterrupt.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
from pathlib import Path

import yaml

from .config import merge

WS = Path(__file__).resolve().parents[1]
COMMANDS = ("py", "test", "sim", "sonar-only")
STUBS = {"sim": "M3"}                           # command: milestone that implements it

# In the image: what python.sh does, except that Python is exec'd so that signals reach it ($0 = Isaac root).
BOOT = ('source "$0/setup_python_env.sh"; '
        'export CARB_APP_PATH="$0/kit" ISAAC_PATH="$0" EXP_PATH="$0/apps" LD_PRELOAD="$0/kit/libcarb.so" '
        'PYTHONPATH="$UW_WS${PYTHONPATH:+:$PYTHONPATH}"; '
        'exec "$0/kit/python/bin/python3" -m uw.runner "$@"')


def runtime() -> dict:
    """config/runtime.yaml, with config/runtime.local.yaml merged over it; UW_RUNTIME and UW_PYTHON override."""
    rt = yaml.safe_load((WS / "config" / "runtime.yaml").read_text())
    local = WS / "config" / "runtime.local.yaml"
    if local.is_file():
        rt = merge(rt, yaml.safe_load(local.read_text()) or {})
    rt["mode"] = os.environ.get("UW_RUNTIME", rt["mode"])
    if os.environ.get("UW_PYTHON"):
        rt["native"]["python"] = os.environ["UW_PYTHON"]
    return rt


def warp_path(rt: dict, isaac_root: str | None) -> str:
    return rt["warp_path"].format(isaac_root=isaac_root) if isaac_root else ""


def docker_argv(rt: dict, cmd: str, args: list[str], *, tty: bool, workdir: Path, name: str) -> list[str]:
    d = rt["docker"]
    root = d["isaac_root"]
    env = {"UW_PYDEPS": d["pydeps"], "UW_WARP_PATH": warp_path(rt, root), "UW_PYTEST_VERSION": str(rt["pytest_version"])}
    return ["docker", "compose", "-f", str(WS / d["compose_file"]), "run", "--rm", *([] if tty else ["-T"]),
            "--name", name, "-w", str(workdir), *[x for k, v in env.items() for x in ("-e", f"{k}={v}")],
            d["service"], "-c", BOOT, root, cmd, *args]


def run_docker(rt: dict, cmd: str, args: list[str]) -> int:
    # The workspace is mounted at the same path in the container: make a script path absolute, and keep the
    # caller's directory (when inside the workspace) so that relative paths in the arguments still resolve.
    if cmd == "py" and args and Path(args[0]).is_file():
        script = Path(args[0]).resolve()
        if not script.is_relative_to(WS):
            print(f"uw: {args[0]} is outside {WS}, the only directory mounted in the container", file=sys.stderr)
            return 2
        args = [str(script), *args[1:]]
    cwd = Path.cwd().resolve()
    workdir = cwd if cwd.is_relative_to(WS) else WS
    for d in ("uw_warp", "uw_cuda", "uw_pydeps"):       # docker would create them root-owned
        (Path.home() / ".cache" / d).mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "UW_WS": str(WS), "UW_UID": str(os.getuid()), "UW_GID": str(os.getgid())}
    name = f"uw-{cmd}-{os.getpid()}"
    tty = sys.stdin.isatty() and sys.stdout.isatty()
    # Without a terminal the client gets its own session: a Ctrl-C (SIGINT to the process group) then reaches only
    # this process, which stops the container cleanly while the client stays attached for the last output.
    proc = subprocess.Popen(docker_argv(rt, cmd, args, tty=tty, workdir=workdir, name=name), env=env,
                            start_new_session=not tty)
    stopping = False

    def on_signal(signum, _frame):
        nonlocal stopping
        if stopping:                                    # second signal: give up; `finally` removes the container
            raise SystemExit(128 + signum)
        stopping = True
        print(f"uw: stopping {name} (Ctrl-C again to kill it)", file=sys.stderr)
        subprocess.Popen(["docker", "stop", "-t", str(rt["docker"]["stop_timeout_s"]), name],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, on_signal)
    try:
        return proc.wait()
    finally:
        subprocess.run(["docker", "rm", "-f", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_native(rt: dict, cmd: str, args: list[str]) -> int:
    n = rt["native"]
    env = {**os.environ, "UW_WS": str(WS), "UW_PYDEPS": str(Path(n["pydeps"]).expanduser()),
           "UW_WARP_PATH": warp_path(rt, n.get("isaac_root")), "UW_PYTEST_VERSION": str(rt["pytest_version"]),
           "PYTHONPATH": os.pathsep.join([str(WS), *filter(None, [os.environ.get("PYTHONPATH")])])}
    proc = subprocess.Popen([n["python"], "-m", "uw.runner", cmd, *args], env=env)
    if os.name == "posix":                              # SIGTERM to us: interrupt the target like Ctrl-C would
        signal.signal(signal.SIGTERM, lambda *_: proc.send_signal(signal.SIGINT))
    interrupted = False
    while True:
        try:
            return proc.wait()
        except KeyboardInterrupt:                       # the target got the Ctrl-C too: let it finish, or kill it
            if interrupted:
                proc.kill()
            interrupted = True


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__, file=sys.stdout if argv else sys.stderr)
        return 0 if argv else 2
    cmd, args = argv[0], argv[1:]
    if cmd not in COMMANDS:
        print(f"uw: unknown command {cmd!r}; one of {', '.join(COMMANDS)}", file=sys.stderr)
        return 2
    if cmd in STUBS:
        print(f"uw {cmd}: not implemented until {STUBS[cmd]}", file=sys.stderr)
        return 1
    rt = runtime()
    if rt["mode"] == "docker":
        return run_docker(rt, cmd, args)
    if rt["mode"] == "native":
        return run_native(rt, cmd, args)
    print(f"uw: runtime mode must be docker or native, got {rt['mode']!r}", file=sys.stderr)
    return 2
