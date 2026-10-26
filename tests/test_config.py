"""Config layering (host, pure Python): base file -> hardware profile -> presets -> --set overrides."""
import argparse
import glob
import os

import pytest
import yaml

from uw import config

CONFIG_DIR = os.path.join(config.WS, "config")


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data))


def _leaves(d, prefix=""):
    """Dotted paths of every value that is not a mapping."""
    out = set()
    for k, v in d.items():
        out |= _leaves(v, f"{prefix}{k}.") if isinstance(v, dict) else {prefix + k}
    return out


def _leaves_of(path):
    with open(path) as f:
        return _leaves(yaml.safe_load(f))


@pytest.fixture
def base(tmp_path):
    """A small config tree: base file, one profile, presets in two groups."""
    _write(tmp_path / "base.yaml", {"a": 0, "b": 0, "c": 0, "sub": {"x": 0, "y": [0, 0]}})
    _write(tmp_path / "profiles" / "p.yaml", {"a": 1, "b": 1, "c": 1, "sub": {"x": 1}, "new": {"k": 1}})
    _write(tmp_path / "presets" / "g" / "q.yaml", {"b": 2, "c": 2})
    _write(tmp_path / "presets" / "g" / "r.yaml", {"b": 3})
    _write(tmp_path / "presets" / "h" / "s.yaml", {"sub": {"y": [9]}})
    return str(tmp_path / "base.yaml")


# ------------------------------------------------------------------ layering rules, on a synthetic tree

def test_order_is_profile_then_preset_then_set(base):
    cfg = config.load(base, ["c=3"], profile="p", presets=["q"])
    assert (cfg["a"], cfg["b"], cfg["c"]) == (1, 2, 3)


def test_mappings_merge_and_lists_replace(base):
    cfg = config.load(base, profile="p", presets=["s"])
    assert cfg["sub"] == {"x": 1, "y": [9]}


def test_profile_may_add_keys_set_may_not(base):
    assert config.load(base, ["new.k=5"], profile="p")["new"]["k"] == 5
    with pytest.raises(config.ConfigError, match="no such key"):
        config.load(base, ["new.k=5"], profile=None)


def test_one_preset_per_group(base):
    assert config.load(base, profile=None, presets=["q", "s"])["b"] == 2
    with pytest.raises(config.ConfigError, match="both 'g' presets"):
        config.load(base, profile=None, presets=["q", "r"])


def test_unknown_names_list_the_available_ones(base):
    with pytest.raises(config.ConfigError, match=r"unknown profile 'x' \(available: p\)"):
        config.load(base, profile="x")
    with pytest.raises(config.ConfigError, match=r"unknown preset 'x' \(available: q, r, s\)"):
        config.load(base, profile=None, presets=["x"])


def test_mapping_and_value_do_not_replace_each_other():
    with pytest.raises(config.ConfigError, match="'sub'"):
        config.merge({"sub": {"x": 1}}, {"sub": 2})


def test_merge_leaves_its_inputs_alone():
    base, overlay = {"sub": {"x": 1}}, {"sub": {"y": [1]}}
    config.merge(base, overlay)["sub"]["y"].append(2)
    assert base == {"sub": {"x": 1}} and overlay == {"sub": {"y": [1]}}


# --------------------------------------------------------------------------- the repository's files

def test_linus_is_the_default_profile():
    assert config.load()["profile"]["name"] == "linus"


def test_profiles_set_the_same_knobs():
    assert _leaves_of(os.path.join(CONFIG_DIR, "profiles", "linus.yaml")) == \
        _leaves_of(os.path.join(CONFIG_DIR, "profiles", "workstation.yaml"))


@pytest.mark.parametrize("group", ["pipe", "size"])
def test_presets_of_a_group_set_the_same_keys(group):
    files = sorted(glob.glob(os.path.join(CONFIG_DIR, "presets", group, "*.yaml")))
    assert len(files) >= 2
    first = _leaves_of(files[0])
    assert all(_leaves_of(f) == first for f in files[1:])


def test_default_presets_are_the_base_config():
    assert config.load(presets=["exposed", "tank"]) == config.load()


def test_all_layers_on_the_real_files():
    cfg = config.load(overrides=["pipe.cover=0.1", "plume.max_bubbles=1234"], profile="workstation",
                      presets=["buried", "basin"])
    assert cfg["profile"]["name"] == "workstation"
    assert cfg["sonar"]["scatterer_spacing_mode"] == "fixed"     # profile
    assert cfg["tank"]["size"] == [8.0, 5.0, 4.0]                 # preset
    assert cfg["pipe"]["cover"] == 0.1                            # --set over the buried preset's 0.05
    assert cfg["plume"]["max_bubbles"] == 1234                    # --set over the profile's 1 M


def test_command_line_options():
    parser = argparse.ArgumentParser()
    config.add_args(parser.add_argument)
    args = parser.parse_args(["--profile", "workstation", "--preset", "buried", "--set", "pipe.cover=0.2"])
    cfg = config.from_args(args)
    assert (cfg["profile"]["name"], cfg["pipe"]["cover"]) == ("workstation", 0.2)


def test_session_config_follows_the_command_line(cfg, pytestconfig):
    """`pytest --profile workstation` (or `scripts/uw.sh test --profile workstation`) reaches the cfg fixture."""
    assert cfg["profile"]["name"] == pytestconfig.option.profile


def test_sonar_mode_limits_from_the_datasheet():
    """Range and ping rate within the mode's datasheet limits (M750d: 120 m LF, 40 m HF, 40 Hz)."""
    assert config.sonar_mode(config.load(overrides=["sonar.mode=hf", "sonar.range=40"]))["max_range"] == 40.0
    with pytest.raises(config.ConfigError, match="max|exceeds"):
        config.sonar_mode(config.load(overrides=["sonar.mode=hf", "sonar.range=41"]))
    with pytest.raises(config.ConfigError, match="exceeds"):
        config.sonar_mode(config.load(overrides=["sonar.ping_rate_hz=50"]))

