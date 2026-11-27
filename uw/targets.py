"""Machines the repo deploys to (linus, work, thor, rov, rov-sim): deploy/targets/<name>.example in git, .local not.

A target is a YAML mapping. ``<name>.example`` holds the documented defaults and is in git; ``<name>.local`` (ignored
by git, one per machine) is merged over it key by key, so a .local only needs what differs (or what `uw deploy rov
--discover` found on the vehicle). Without a .local the example is used as is, with a note on stderr.
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml

from .config import merge

WS = Path(__file__).resolve().parents[1]
DIR = WS / "deploy" / "targets"


class TargetError(RuntimeError):
    pass


def names() -> list[str]:
    return sorted({p.name.split(".")[0] for p in DIR.glob("*.example")} | {p.name.split(".")[0] for p in DIR.glob("*.local")})


def load(name: str, quiet: bool = False) -> dict:
    example, local = DIR / f"{name}.example", DIR / f"{name}.local"
    if not example.is_file() and not local.is_file():
        raise TargetError(f"no target {name!r}: expected {example.relative_to(WS)} (targets: {', '.join(names())})")
    cfg = yaml.safe_load(example.read_text(encoding="utf-8")) if example.is_file() else {}
    if local.is_file():
        cfg = merge(cfg or {}, yaml.safe_load(local.read_text(encoding="utf-8")) or {})
    elif not quiet:
        print(f"uw: {local.relative_to(WS)} not found, using {example.name} as is", file=sys.stderr)
    cfg["name"] = name
    return cfg


def update_local(name: str, values: dict) -> Path:
    """Merge ``values`` into deploy/targets/<name>.local (created if missing), keeping what is already there."""
    local = DIR / f"{name}.local"
    cur = yaml.safe_load(local.read_text(encoding="utf-8")) if local.is_file() else None
    new = merge(cur or {}, values)
    head = (f"# {local.name}: this machine's settings for target {name!r}, merged over {name}.example.\n"
            "# Not in git (*.local). Written by `uw deploy --discover`, edit freely.\n")
    local.write_text(head + yaml.safe_dump(new, sort_keys=False), encoding="utf-8")
    return local
