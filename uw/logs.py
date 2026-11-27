"""`uw logs pull rov|rov-sim`: copy the vehicle's logs off it (pull only, never delete there) and index them.

    uw logs pull rov [--dry-run] [--only bin|bags]

From the target's `logs`:
- `bin_dir`: ArduSub's dataflash logs (.BIN) — on a BlueOS Pi under /root/.config/blueos/ardupilot-manager/
  firmware/logs, root-owned, hence `sudo: true` for that copy;
- `bag_dir`: the rosbag2 recordings of rov_bringup (/usr/blueos/extensions/ros2/bags on the Pi);
to `dest` (outside git: /HDD2T or the logs machine), as <dest>/bin/ and <dest>/bags/, with rsync over SSH
(--ignore-existing: a file already pulled is never overwritten). Then robot/bluerov2/logs/INDEX.md gets one row per
new log (a .BIN, or a bag folder): name, size, modification time, where it is. The index is the only thing that
goes in git; a target can point `logs.index` elsewhere (rov-sim does, so test logs never reach the real index).
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import shlex
import sys
from pathlib import Path

from . import targets
from .remote import Remote, RemoteError

INDEX = targets.WS / "robot" / "bluerov2" / "logs" / "INDEX.md"
HEAD = """# Log del BlueROV2 reale — indice

I file stanno fuori da git, nella cartella `dest` del target (`deploy/targets/rov.local`). Questo indice lo aggiorna
`uw logs pull rov` (una riga per file nuovo); la colonna *Note* si compila a mano (checklist post-immersione).

| Scaricato | File | Tipo | Dimensione | Modificato (sul veicolo) | Dove | Note |
|---|---|---|---|---|---|---|
"""


def _size(n: int) -> str:
    for unit in ("B", "kB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024.0
    return str(n)


def _listing(root: Path) -> dict:
    out = {}
    if root.is_dir():
        for p in root.rglob("*"):
            if p.is_file():
                out[str(p.relative_to(root))] = p.stat()
    return out


def index_new(dest: Path, before: dict, target: str) -> list[str]:
    """One row per new .BIN file and per new bag (a bag is a folder: its files are summed)."""
    rows = []
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    for sub, kind in (("bin", "ArduSub .BIN"), ("bags", "rosbag2")):
        groups: dict[str, list] = {}
        for rel, st in sorted(_listing(dest / sub).items()):
            if f"{sub}/{rel}" in before:
                continue
            key = rel.split("/")[0] if sub == "bags" else rel
            groups.setdefault(key, []).append(st)
        for key, sts in groups.items():
            mtime = dt.datetime.fromtimestamp(max(s.st_mtime for s in sts)).strftime("%Y-%m-%d %H:%M")
            size = _size(sum(s.st_size for s in sts))
            rows.append(f"| {now} | `{key}` | {kind} | {size} | {mtime} | `{dest / sub}` ({target}) | |")
    return rows


def index_path(t: dict) -> Path:
    """The target's logs.index (default robot/bluerov2/logs/INDEX.md, the real vehicle's, in git)."""
    p = Path(os.path.expanduser(str(t.get("logs", {}).get("index") or INDEX)))
    return p if p.is_absolute() else targets.WS / p


def pull(t: dict, args) -> int:
    lg = t.get("logs", {})
    dest = Path(os.path.expanduser(str(lg.get("dest", "")))).resolve()
    if not lg.get("dest"):
        raise RemoteError("the target has no logs.dest")
    r = Remote(t, dry_run=args.dry_run)
    print(f"uw logs pull {t['name']}: {r.where()} -> {dest}{' (dry run)' if args.dry_run else ''}")
    before = {f"{s}/{k}": v for s in ("bin", "bags") for k, v in _listing(dest / s).items()}
    jobs = []
    if args.only in (None, "bin") and lg.get("bin_dir"):
        jobs.append(("bin", lg["bin_dir"], bool(lg.get("bin_sudo", False))))
    if args.only in (None, "bags") and lg.get("bag_dir"):
        jobs.append(("bags", lg["bag_dir"], bool(lg.get("bag_sudo", False))))
    for sub, src, sudo in jobs:
        src = r.path(src)
        exists = r.sh(f"{'sudo -n ' if sudo and not r.local else ''}test -d {shlex.quote(src)} && echo yes || echo no"
                      ).strip()
        if exists != "yes":
            print(f"  {sub}: {src} does not exist on {r.where()} — skipped (check logs.{sub}_dir)")
            continue
        if not args.dry_run:
            (dest / sub).mkdir(parents=True, exist_ok=True)
        r.rsync(src.rstrip("/") + "/", str(dest / sub) + "/", ["--ignore-existing", "--prune-empty-dirs"],
                pull=True, sudo=sudo)
    if args.dry_run:
        print("uw logs: dry run, nothing copied, index untouched")
        return 0
    rows = index_new(dest, before, t["name"])
    index = index_path(t)
    if rows:
        index.parent.mkdir(parents=True, exist_ok=True)
        text = index.read_text(encoding="utf-8") if index.is_file() else HEAD
        index.write_text(text.rstrip("\n") + "\n" + "\n".join(rows) + "\n", encoding="utf-8")
    in_repo = index.is_relative_to(targets.WS)
    print(f"uw logs: {len(rows)} new log(s); index {index.relative_to(targets.WS) if in_repo else index}"
          + (" — commit it from linus" if rows and in_repo else ""))
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="uw logs", description=__doc__.splitlines()[0])
    ap.add_argument("action", choices=["pull"])
    ap.add_argument("target")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", choices=["bin", "bags"], default=None)
    args = ap.parse_args(argv)
    try:
        return pull(targets.load(args.target), args)
    except (RemoteError, targets.TargetError) as e:
        print(f"uw logs: {e}", file=sys.stderr)
        return 1
