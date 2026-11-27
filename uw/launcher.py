"""`python -m uw`: the entry point. Runs a command in the Isaac Sim container (Linux) or with a local Python (Windows
venv, native Isaac), as uw/runtime.yaml says.

    python -m uw py SCRIPT [args]        also py -m MODULE, py -c CODE
    python -m uw test [pytest args]      no args: also the ROS 2 stage on the host (--no-ros, --ros, --ros-only)
    python -m uw sim [--gui] [args]      the Isaac app: USD stage, RTX camera, sonar in the loop, ROS 2
    python -m uw sonar-only [args]
    python -m uw pool [--gui] [args]     the M11 test pool (sim/pool.py)
Config options go after the command and reach the Python side unchanged: --profile linus|workstation,
--preset NAME (repeatable), --set a.b=c (repeatable).

Docker: a one-off container, removed on exit, Ctrl-C, SIGTERM or SIGHUP. With a terminal, Ctrl-C goes straight into
the container; without one it lands here and stops the container (a second Ctrl-C kills it at once). The target
side (uw/runner.py) turns Ctrl-C and docker stop into a KeyboardInterrupt.

Host commands run here, on the machine you type them on (they drive Docker, SSH, git):

    python -m uw image dev|thor|rov | promote | list    images tagged with the commit (uw/image.py)
    python -m uw deploy work|thor|rov [--dry-run]      bring a machine to this commit (uw/deploy.py)
    python -m uw smoke rov                             heartbeat, DISARMED, topic rates, CPU (uw/smoke.py)
    python -m uw logs pull rov                         .BIN + bags off the vehicle, indexed (uw/logs.py)
    python -m uw sysid bluerov2_heavy [--write]        logs -> vehicle.yaml (sysid/cli.py, skeleton)
`rov-sim` is the vehicle's stand-in on linus for all of these (docs/ARCHITECTURE.md).
"""
from __future__ import annotations

import os
import shlex
import signal
import subprocess
import sys
from pathlib import Path

import yaml

from .config import merge

WS = Path(__file__).resolve().parents[1]
COMMANDS = ("py", "test", "sim", "sonar-only", "pool")
STUBS: dict[str, str] = {}                      # command: milestone that implements it
#: commands that run on this host, not in the Isaac container: command -> module with main(argv) -> int
HOST = {"image": "uw.image", "deploy": "uw.deploy", "smoke": "uw.smoke", "logs": "uw.logs", "sysid": "sysid.cli"}

# In the image: what python.sh does, except that Python is exec'd so that signals reach it ($0 = Isaac root); plus
# the ROS bridge's internal Jazzy libraries (read at process start, so they cannot be added from Python).
BOOT = ('source "$0/setup_python_env.sh"; '
        'export CARB_APP_PATH="$0/kit" ISAAC_PATH="$0" EXP_PATH="$0/apps" LD_PRELOAD="$0/kit/libcarb.so" '
        'PYTHONPATH="$UW_WS${PYTHONPATH:+:$PYTHONPATH}" '
        'LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$0/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib:$UW_USD_PATH/bin"; '
        'exec "$0/kit/python/bin/python3" -m uw.runner "$@"')


def runtime() -> dict:
    """uw/runtime.yaml, with uw/runtime.local.yaml merged over it; UW_RUNTIME and UW_PYTHON override."""
    rt = yaml.safe_load((WS / "uw" / "runtime.yaml").read_text())
    local = WS / "uw" / "runtime.local.yaml"
    if (WS / "config" / "runtime.local.yaml").is_file() and not local.is_file():
        print("uw: config/runtime.local.yaml is not read any more: move it to uw/runtime.local.yaml "
              "(docs/MIGRATION.md)", file=sys.stderr)
    if local.is_file():
        rt = merge(rt, yaml.safe_load(local.read_text()) or {})
    rt["mode"] = os.environ.get("UW_RUNTIME", rt["mode"])
    if os.environ.get("UW_PYTHON"):
        rt["native"]["python"] = os.environ["UW_PYTHON"]
    return rt


def warp_path(rt: dict, isaac_root: str | None, key: str = "warp_path") -> str:
    return rt[key].format(isaac_root=isaac_root) if isaac_root else ""


def git_version() -> str:
    """The repository's version, resolved here on the host and passed to the target as ``UW_GIT_VERSION``.

    The Isaac container mounts `.git` but ships no git binary, so a ping generated inside it cannot work this
    out for itself and would record its provenance as "unknown". `sim.recorder.git_version` prefers the
    variable this sets.
    """
    from sim.recorder import git_version as resolve
    return resolve()


def docker_argv(rt: dict, cmd: str, args: list[str], *, tty: bool, workdir: Path, name: str,
                extra: list[str] = ()) -> list[str]:
    d = rt["docker"]
    root = d["isaac_root"]
    env = {"UW_PYDEPS": d["pydeps"], "UW_WARP_PATH": warp_path(rt, root), "UW_USD_PATH": warp_path(rt, root, "usd_path"),
           "UW_PYTEST_VERSION": str(rt["pytest_version"]), "UW_ISAAC_ROOT": root,
           "UW_GIT_VERSION": git_version()}
    return ["docker", "compose", "-f", str(WS / d["compose_file"]), "run", "--rm", *([] if tty else ["-T"]),
            "--name", name, "-w", str(workdir), *[x for k, v in env.items() for x in ("-e", f"{k}={v}")], *extra,
            d["service"], "-c", BOOT, root, cmd, *args]


def x11_args() -> list[str]:
    """docker run options for a GUI: the X11 socket, the X authority file, DISPLAY."""
    display = os.environ.get("DISPLAY")
    if not display:
        raise SystemExit("uw: --gui needs an X display (DISPLAY is not set)")
    xauth = Path(os.environ.get("XAUTHORITY") or Path.home() / ".Xauthority")
    out = ["-e", f"DISPLAY={display}", "-v", "/tmp/.X11-unix:/tmp/.X11-unix:ro"]
    if xauth.is_file():
        out += ["-v", f"{xauth}:/uw_xauth:ro", "-e", "XAUTHORITY=/uw_xauth"]
    return out


def _mkdir(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        return True
    except OSError:
        return False


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
    for d in ("uw_warp", "uw_cuda", "uw_pydeps", "uw_home/kit_cache", "uw_home/kit_data"):   # else root-owned
        (Path.home() / ".cache" / d).mkdir(parents=True, exist_ok=True)
    extra = x11_args() if cmd in ("sim", "pool") and "--gui" in args else []
    data = Path(os.path.expanduser(str(rt.get("data_root") or "")))
    if str(data) not in ("", ".") and (data.is_dir() or _mkdir(data)):
        extra += ["-v", f"{data}:{data}:rw", "-e", f"UW_DATA={data}"]
    env = {**os.environ, "UW_WS": str(WS), "UW_UID": str(os.getuid()), "UW_GID": str(os.getgid())}
    name = f"uw-{cmd}-{os.getpid()}"
    tty = sys.stdin.isatty() and sys.stdout.isatty()
    # Without a terminal the client gets its own session: a Ctrl-C (SIGINT to the process group) then reaches only
    # this process, which stops the container cleanly while the client stays attached for the last output.
    proc = subprocess.Popen(docker_argv(rt, cmd, args, tty=tty, workdir=workdir, name=name, extra=extra), env=env,
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
    env = {**os.environ, "UW_WS": str(WS), "UW_GIT_VERSION": git_version(), "UW_PYDEPS": str(Path(n["pydeps"]).expanduser()),
           "UW_WARP_PATH": warp_path(rt, n.get("isaac_root")), "UW_PYTEST_VERSION": str(rt["pytest_version"]),
           "UW_USD_PATH": warp_path(rt, n.get("isaac_root"), "usd_path"), "UW_ISAAC_ROOT": n.get("isaac_root") or "",
           "UW_DATA": os.path.expanduser(str(rt.get("data_root") or "")),
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


ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
#: the tests that need host ROS 2 Jazzy (rclpy, rosbag2, launch, message packages) and a built ros/src
ROS_TESTS = ["tests/test_rov_bringup.py", "tests/test_uw_ops.py", "tests/test_ros_messages.py", "tests/test_dvl_dr.py",
             "tests/test_pose_cmd.py"]


def run_ros_stage() -> int:
    """`uw test`, second stage, on the host: colcon build of ros/src (into output/colcon, not in git), then the ROS 2
    tests with host Jazzy and that build sourced. The first stage (the Isaac container) has no /opt/ros."""
    out = WS / "output" / "colcon"
    out.mkdir(parents=True, exist_ok=True)
    q = shlex.quote
    script = (f"source {q(str(ROS_SETUP))} && cd {q(str(WS))} && "
              f"colcon --log-base {q(str(out / 'log'))} build --base-paths ros/src --build-base {q(str(out / 'build'))} "
              f"--install-base {q(str(out / 'install'))} > {q(str(out / 'build.txt'))} 2>&1; b=$?; tail -2 {q(str(out / 'build.txt'))}; "
              f"[ $b -eq 0 ] || exit $b; source {q(str(out / 'install' / 'setup.bash'))} && "
              f"python3 -m pytest -q -p no:cacheprovider {' '.join(ROS_TESTS)}")
    print("uw test: ROS 2 stage (host Jazzy): colcon build ros/src, then " + " ".join(ROS_TESTS), file=sys.stderr)
    return subprocess.run(["bash", "-c", script]).returncode


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__, file=sys.stdout if argv else sys.stderr)
        return 0 if argv else 2
    cmd, args = argv[0], argv[1:]
    if cmd in HOST:
        import importlib
        return int(importlib.import_module(HOST[cmd]).main(args) or 0)
    if cmd not in COMMANDS:
        print(f"uw: unknown command {cmd!r}; one of {', '.join([*COMMANDS, *HOST])}", file=sys.stderr)
        return 2
    if cmd in STUBS:
        print(f"uw {cmd}: not implemented until {STUBS[cmd]}", file=sys.stderr)
        return 1
    ros = False
    if cmd == "test":
        flags = [a for a in args if a in ("--ros", "--no-ros", "--ros-only")]
        args = [a for a in args if a not in flags]
        ros = ("--ros" in flags or "--ros-only" in flags or not args) and "--no-ros" not in flags
        if ros and not (os.name == "posix" and ROS_SETUP.is_file()):
            print(f"uw test: no {ROS_SETUP} on this host: the ROS 2 stage is skipped", file=sys.stderr)
            ros = False
        if "--ros-only" in flags:
            return run_ros_stage() if ros else 1
    rt = runtime()
    if rt["mode"] == "docker":
        rc = run_docker(rt, cmd, args)
    elif rt["mode"] == "native":
        rc = run_native(rt, cmd, args)
    else:
        print(f"uw: runtime mode must be docker or native, got {rt['mode']!r}", file=sys.stderr)
        return 2
    if ros:
        rc_ros = run_ros_stage()
        print(f"uw test: suite exit {rc}, ROS 2 stage exit {rc_ros}", file=sys.stderr)
        rc = rc or rc_ros
    return rc
