"""Configuration: one YAML file (config/tank.yaml) plus ``--set a.b.c=value`` overrides.

Values stay plain nested dicts. ``get(cfg, "a.b.c")`` fails with the dotted key when it is missing,
so a typo in a config edit is reported instead of silently falling back to something else.
"""
from __future__ import annotations

import copy
import os
from typing import Any, Iterable

import yaml

WS = os.environ.get("UW_WS", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CONFIG = os.path.join(WS, "config", "tank.yaml")


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


def load(path: str = DEFAULT_CONFIG, overrides: Iterable[str] = ()) -> dict:
    if not os.path.isfile(path):
        raise ConfigError(f"config file not found: {path}")
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ConfigError(f"{path}: expected a mapping at top level")
    return apply_overrides(cfg, overrides)


def sonar_mode(cfg: dict) -> dict:
    """The active sonar mode (``sonar.modes[sonar.mode]``) merged over the sonar section."""
    son = get(cfg, "sonar")
    mode = son["mode"]
    if mode not in son["modes"]:
        raise ConfigError(f"sonar.mode '{mode}' not in sonar.modes {sorted(son['modes'])}")
    merged = {k: v for k, v in son.items() if k != "modes"}
    merged.update(son["modes"][mode])
    return merged
