"""Configuration: config/tank.yaml, a hardware profile, presets, then ``--set a.b.c=value`` overrides.

Layers, later ones win:
1. ``config/tank.yaml``: every scene and sensor parameter (it is the ``exposed`` + ``tank`` scene);
2. ``config/profiles/<name>.yaml`` (``--profile``, default ``linus``): fidelity knobs for the hardware;
3. ``config/presets/<group>/<name>.yaml`` (``--preset``, at most one per group): ``pipe`` = exposed | buried,
   ``size`` = tank | basin;
4. ``--set a.b.c=value``.
Profiles and presets may add keys (backend selectors enter the config that way); ``--set`` only changes
existing ones.

Values stay plain nested dicts. ``get(cfg, "a.b.c")`` fails with the dotted key when it is missing,
so a typo in a config edit is reported instead of silently falling back to something else.
"""
from __future__ import annotations

import copy
import glob
import os
from typing import Any, Callable, Iterable

import yaml

WS = os.environ.get("UW_WS", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CONFIG = os.path.join(WS, "config", "tank.yaml")
DEFAULT_PROFILE = "linus"


class ConfigError(RuntimeError):
    """A config file is missing, malformed, or lacks a required key."""


def get(cfg: dict, dotted: str) -> Any:
    cur: Any = cfg
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            raise ConfigError(f"missing config key '{dotted}'")
        cur = cur[part]
    return cur


def apply_overrides(cfg: dict, overrides: Iterable[str]) -> dict:
    """Return a copy of ``cfg`` with each ``a.b.c=value`` applied; the value is parsed as YAML.

    Only existing keys can be overridden: a misspelt key raises.
    """
    out = copy.deepcopy(cfg)
    for item in overrides:
        if "=" not in item:
            raise ConfigError(f"override '{item}' is not key=value")
        key, raw = item.split("=", 1)
        parts = key.strip().split(".")
        cur = out
        for part in parts[:-1]:
            if not isinstance(cur, dict) or part not in cur:
                raise ConfigError(f"override '{key}': no such key")
            cur = cur[part]
        if not isinstance(cur, dict) or parts[-1] not in cur:
            raise ConfigError(f"override '{key}': no such key")
        cur[parts[-1]] = yaml.safe_load(raw)
    return out


def merge(base: dict, overlay: dict) -> dict:
    """A copy of ``base`` with ``overlay`` merged in: mappings merge key by key, any other value (a list too)
    replaces. New keys are allowed; a mapping and a plain value cannot replace each other."""
    out = copy.deepcopy(base)
    _merge_into(out, overlay, "")
    return out


def _merge_into(dst: dict, src: dict, prefix: str) -> None:
    for k, v in src.items():
        key = prefix + str(k)
        if isinstance(dst.get(k), dict) and isinstance(v, dict):
            _merge_into(dst[k], v, key + ".")
        elif k in dst and (isinstance(dst[k], dict) or isinstance(v, dict)):
            raise ConfigError(f"'{key}': a mapping and a plain value cannot replace each other")
        else:
            dst[k] = copy.deepcopy(v)


def load(path: str = DEFAULT_CONFIG, overrides: Iterable[str] = (), profile: str | None = DEFAULT_PROFILE,
         presets: Iterable[str] = ()) -> dict:
    """``path``, then ``profile`` and ``presets`` from the profiles/ and presets/ directories next to it, then
    ``overrides``. ``profile=None`` skips the profile layer."""
    cfg = _read(path)
    root = os.path.dirname(os.path.abspath(path))
    if profile:
        cfg = _layer(cfg, _profile_file(root, profile))
    groups: dict[str, str] = {}
    for name in presets:
        group, file = _preset_file(root, name)
        if group in groups:
            raise ConfigError(f"presets '{groups[group]}' and '{name}' are both '{group}' presets: pick one")
        groups[group] = name
        cfg = _layer(cfg, file)
    return apply_overrides(cfg, overrides)


def _read(path: str) -> dict:
    if not os.path.isfile(path):
        raise ConfigError(f"config file not found: {path}")
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ConfigError(f"{path}: expected a mapping at top level")
    return cfg


def _layer(cfg: dict, path: str) -> dict:
    overlay = _read(path)
    try:
        return merge(cfg, overlay)
    except ConfigError as e:
        raise ConfigError(f"{path}: {e}") from None


def _profile_file(root: str, name: str) -> str:
    path = os.path.join(root, "profiles", f"{name}.yaml")
    if not os.path.isfile(path):
        raise ConfigError(f"unknown profile '{name}' (available: {_names(root, 'profiles', '*.yaml')})")
    return path


def _preset_file(root: str, name: str) -> tuple[str, str]:
    """(group, path) of ``presets/<group>/<name>.yaml``."""
    hits = glob.glob(os.path.join(root, "presets", "*", glob.escape(name) + ".yaml"))
    if len(hits) != 1:
        why = "ambiguous" if hits else "unknown"
        raise ConfigError(f"{why} preset '{name}' (available: {_names(root, 'presets', '*', '*.yaml')})")
    return os.path.basename(os.path.dirname(hits[0])), hits[0]


def _names(root: str, *pattern: str) -> str:
    found = sorted(os.path.basename(p)[:-len(".yaml")] for p in glob.glob(os.path.join(root, *pattern)))
    return ", ".join(found) or "none"


def add_args(add_argument: Callable[..., Any]) -> None:
    """Register --profile, --preset and --set with ``parser.add_argument`` (argparse) or ``parser.addoption``
    (pytest); ``from_args`` builds the config from the parsed options."""
    add_argument("--profile", default=DEFAULT_PROFILE,
                 help="hardware profile, config/profiles/<name>.yaml (default: %(default)s)")
    add_argument("--preset", action="append", default=[], metavar="NAME",
                 help="config/presets/<group>/<NAME>.yaml, e.g. buried or basin; repeatable")
    add_argument("--set", action="append", default=[], dest="overrides", metavar="KEY=VALUE",
                 help="override one value, e.g. --set leak.flow_lpm=30; repeatable")


def from_args(args: Any) -> dict:
    return load(overrides=args.overrides, profile=args.profile, presets=args.preset)


def sonar_mode(cfg: dict) -> dict:
    """The active sonar mode (``sonar.modes[sonar.mode]``) merged over the sonar section."""
    son = get(cfg, "sonar")
    mode = son["mode"]
    if mode not in son["modes"]:
        raise ConfigError(f"sonar.mode '{mode}' not in sonar.modes {sorted(son['modes'])}")
    merged = {k: v for k, v in son.items() if k != "modes"}
    merged.update(son["modes"][mode])
    return merged
