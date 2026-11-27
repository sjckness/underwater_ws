"""`uw deploy work|thor|rov|rov-sim`: bring a machine to this checkout's code.

    uw deploy rov [--dry-run] [--yes] [--discover] [--start] [--no-restart]
    uw deploy thor|work [--dry-run] [--yes]

rov (and rov-sim, its test double on linus): the vehicle runs our ROS code only inside the BlueOS ROS 2 extension.
1. read the target (deploy/targets/rov.local over rov.example); with --discover, find the extension's container
   and its persistent folder on the Pi (`docker ps`, `docker inspect`) and write them to rov.local;
2. check through mavlink2rest that ArduSub is live and DISARMED — otherwise stop, nothing changed;
3. ask (or --yes);
4. rsync the target's `packages` from ros/src to <persistent>/<workspace>/src (only those folders, --delete inside
   them), `colcon build` them inside the extension container;
5. check DISARMED again and restart the extension container (unless --no-restart), so no node of the old build
   keeps running; with --start, launch `rov_bringup bringup.launch.py` detached in it.
Only the extension container and its persistent folder are touched: never blueos-core, never ArduSub's parameters.

thor / work: they only pull. Over SSH: `git pull --ff-only` in the target's checkout (and, on thor, rebuild the
stack image). --dry-run prints the commands.
"""
from __future__ import annotations

import argparse
import json
import shlex
import sys

from . import targets
from .remote import Remote, RemoteError, require_disarmed

WS = targets.WS


def confirm(question: str, yes: bool) -> bool:
    if yes:
        return True
    if not sys.stdin.isatty():
        print("uw deploy: not a terminal and no --yes: stopping", file=sys.stderr)
        return False
    return input(f"{question} [y/N] ").strip().lower() in ("y", "yes", "s", "si", "sì")


def discover(r: Remote, t: dict) -> dict:
    """The extension container (by image) and the host folder mounted at its persistent path."""
    out = r.sh(f"{r.docker()} ps --format '{{{{.Names}}}}\t{{{{.Image}}}}'")
    image_hint = t.get("extension_image", "blueos-ros2")
    hits = [ln.split("\t") for ln in out.splitlines() if image_hint in ln]
    if len(hits) != 1:
        raise RemoteError(f"expected one running container from an image matching {image_hint!r}, found "
                          f"{[h[0] for h in hits] or 'none'}:\n{out}")
    name, image = hits[0]
    mounts = json.loads(r.sh(f"{r.docker()} inspect --format '{{{{json .Mounts}}}}' {shlex.quote(name)}"))
    dest = t.get("persistent_container", "/root/persistent_ws").rstrip("/")
    src = [m["Source"] for m in mounts if m.get("Destination", "").rstrip("/") == dest]
    if not src:
        raise RemoteError(f"{name} has no mount at {dest}: {[m.get('Destination') for m in mounts]}")
    return {"container": name, "extension_image_found": image, "persistent_host": src[0].rstrip("/")}


def deploy_rov(t: dict, args) -> int:
    r = Remote(t, dry_run=args.dry_run)
    print(f"uw deploy {t['name']}: {r.where()}{' (dry run)' if args.dry_run else ''}")
    if args.discover or not t.get("container"):
        found = discover(r, t)
        print(f"  found: container {found['container']} ({found['extension_image_found']}), "
              f"persistent {found['persistent_host']}")
        if not args.dry_run:
            print(f"  wrote deploy/targets/{targets.update_local(t['name'], found).name}")
        t = {**t, **found}
        r = Remote(t, dry_run=args.dry_run)
    c, pc = t["container"], t.get("persistent_container", "/root/persistent_ws").rstrip("/")
    ws_host = f"{r.path(t['persistent_host']).rstrip('/')}/{t.get('workspace', 'ros2_ws')}"
    ws_cont = f"{pc}/{t.get('workspace', 'ros2_ws')}"
    pkgs = list(t.get("packages") or [])
    missing = [p for p in pkgs if not (WS / "ros" / "src" / p / "package.xml").is_file()]
    if not pkgs or missing:
        raise RemoteError(f"target packages must be folders of ros/src with a package.xml; missing: {missing or 'all'}")

    require_disarmed(t)
    print(f"  plan: rsync ros/src/{{{','.join(pkgs)}}} -> {ws_host}/src, colcon build in {c}"
          + ("" if args.no_restart else f", restart {c}") + (", start rov_bringup" if args.start else ""))
    if not confirm(f"Deploy to {r.where()}?", args.yes or args.dry_run):
        return 1

    r.sh(f"{t.get('sudo', '')} mkdir -p {shlex.quote(ws_host + '/src')}".strip(), mutating=True)
    for p in pkgs:
        r.rsync(f"{WS / 'ros' / 'src' / p}/", f"{ws_host}/src/{p}/",
                ["--delete", "--exclude", "__pycache__", "--exclude", "*.pyc"], sudo=bool(t.get("sudo")))
    build = (f"source /opt/ros/jazzy/setup.bash && cd {shlex.quote(ws_cont)} && "
             f"colcon build --packages-select {' '.join(pkgs)} {' '.join(map(str, t.get('colcon_args', [])))}")
    print(f"  build in {c}: {build}")
    out = r.exec_in(c, build, mutating=True, timeout=1800)
    if out:
        print("  " + "\n  ".join(out.strip().splitlines()[-4:]))
    if not args.no_restart:
        require_disarmed(t)                     # again: a build on the Pi takes minutes
        r.sh(f"{r.docker()} restart {shlex.quote(c)}", mutating=True, timeout=180)
        if not args.dry_run:
            print(f"  restarted {c}")
    if args.start:
        launch = (f"source /opt/ros/jazzy/setup.bash && source {shlex.quote(ws_cont)}/install/setup.bash && "
                  f"export ROS_DOMAIN_ID={int(t.get('ros_domain_id', 73))} && "
                  f"exec ros2 launch rov_bringup bringup.launch.py vehicle:={t.get('vehicle', 'rov1')} "
                  f"fcu_url:={shlex.quote(str(t.get('fcu_url', 'tcp://127.0.0.1:5777')))} "
                  f"> {shlex.quote(pc)}/bringup.log 2>&1")
        r.sh(f"{r.docker()} exec -d {shlex.quote(c)} bash -lc {shlex.quote(launch)}", mutating=True)
        if not args.dry_run:
            print(f"  started rov_bringup in {c} (log: {pc}/bringup.log)")
    print("uw deploy: done" + (" (dry run: nothing changed)" if args.dry_run else ""))
    return 0


def deploy_pull(t: dict, args) -> int:
    """work / thor: they never commit; they fast-forward to origin/main (or the target's branch)."""
    r = Remote(t, dry_run=args.dry_run)
    repo = t["repo"]
    branch = t.get("branch", "main")
    steps = [f"git -C {shlex.quote(repo)} fetch --prune",
             f"git -C {shlex.quote(repo)} checkout {shlex.quote(branch)}",
             f"git -C {shlex.quote(repo)} pull --ff-only"]
    steps += [f"cd {shlex.quote(repo)} && {s}" for s in t.get("after_pull", [])]
    print(f"uw deploy {t['name']}: {r.where()}, {repo} -> {branch}")
    for s in steps:
        print(f"  {s}")
    if not confirm(f"Run these on {r.where()}?", args.yes or args.dry_run):
        return 1
    for s in steps:
        out = r.sh(s, mutating=True, timeout=1800)
        if out.strip():
            print("  " + "\n  ".join(out.strip().splitlines()[-3:]))
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="uw deploy", description=__doc__.splitlines()[0])
    ap.add_argument("target", help=f"one of {', '.join(targets.names())}")
    ap.add_argument("--dry-run", action="store_true", help="print what would change, change nothing")
    ap.add_argument("--yes", action="store_true", help="do not ask")
    ap.add_argument("--discover", action="store_true", help="rov: find container and paths, write <target>.local")
    ap.add_argument("--start", action="store_true", help="rov: launch rov_bringup after the restart")
    ap.add_argument("--no-restart", action="store_true", help="rov: build only, keep the container running")
    args = ap.parse_args(argv)
    try:
        t = targets.load(args.target)
        kind = t.get("kind", args.target)
        if kind == "rov":
            return deploy_rov(t, args)
        if kind in ("work", "thor", "pull"):
            return deploy_pull(t, args)
        print(f"uw deploy: target {args.target!r} has kind {kind!r}: rov | work | thor", file=sys.stderr)
        return 2
    except (RemoteError, targets.TargetError) as e:
        print(f"uw deploy: {e}", file=sys.stderr)
        return 1
