"""Provenance of a generated ping (host, numpy): `generator_version` and the mixed-dataset warning.

A dataset is only usable as evidence if what made it can be named. `config_hash` alone cannot do that -- it
hashes the resolved config, so a change to the maths in `physics/sonar.py` leaves it untouched -- which is why a
sonar model version and the git hash are recorded beside it.
"""
import ast
import json
import re
from pathlib import Path

import numpy as np
import pytest

from uw import config
from ml import dataset
from sim import recorder

WS = Path(__file__).resolve().parents[1]


def test_generator_version_names_the_model_the_tree_and_the_config():
    cfg = config.load()
    v = recorder.generator_version("m750d-1", cfg)
    model, git, chash = v.split("/")
    assert model == "m750d-1"
    assert git == recorder.git_version()                  # '+dirty' and all: it is one field
    assert chash == recorder.config_hash(cfg)
    assert v == recorder.generator_version("m750d-1", cfg)   # stable for the same inputs


def test_generator_version_stays_parseable_when_the_tree_is_dirty():
    """`git_version` appends '+dirty', so '+' cannot be the separator: the string has to survive being split
    back into exactly the three fields it promises."""
    assert len(recorder.generator_version("m750d-1", config.load()).split("/")) == 3
    assert "/" not in recorder.git_version()
    assert "/" not in recorder.config_hash(config.load())


def test_generator_version_changes_when_any_of_the_three_does():
    cfg = config.load()
    other = config.load(overrides=["sonar.ringdown_db=55"])
    base = recorder.generator_version("m750d-1", cfg)
    assert recorder.generator_version("m750d-2", cfg) != base          # the model changed
    assert recorder.generator_version("m750d-1", other) != base        # the parameters changed


def test_git_version_is_cached_so_it_is_not_paid_per_ping():
    """Two subprocesses per ping would be a measurable share of the 100 ms ping budget."""
    recorder.git_version()
    before = recorder.git_version.cache_info().hits
    for _ in range(50):
        recorder.git_version()
    assert recorder.git_version.cache_info().hits >= before + 50


def test_the_sonar_declares_a_model_version_and_hands_it_to_the_recorder():
    """Read from the source: `physics/sonar.py` needs warp and torch, which the host suite does not have."""
    src = (WS / "physics" / "sonar.py").read_text()
    m = re.search(r'^SONAR_MODEL_VERSION = "([^"]+)"', src, re.M)
    assert m, "physics/sonar.py must define SONAR_MODEL_VERSION at module level"
    assert m.group(1).strip()
    assert "self.model_version = SONAR_MODEL_VERSION" in src, "Sonar must expose it as .model_version"
    # and the recorder must stamp it, not re-derive it
    rec = (WS / "sim" / "recorder.py").read_text()
    assert "generator_version(son.model_version, cfg)" in rec


def test_there_is_only_one_git_version_implementation():
    """Two could disagree about whether the tree is dirty, and then a dataset's manifest and its own pings
    would claim different provenance."""
    tree = ast.parse((WS / "ml" / "tools" / "sweep.py").read_text())
    defined = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "git_version" not in defined, "ml/tools/sweep.py should reuse sim.recorder.git_version"


# ------------------------------------------------------------------ the mixed-dataset warning
def fake_dataset(root: Path, versions: dict) -> None:
    """A dataset of one ping per scenario, each stamped with the given generator_version."""
    scenarios = []
    for i, (sid, version) in enumerate(versions.items()):
        d = root / sid
        d.mkdir(parents=True)
        np.savez(d / "ping_00000.npz", polar_db=np.zeros((4, 4), np.float16),
                 range_m=np.arange(4, dtype=np.float32), bearing_deg=np.arange(4, dtype=np.float32))
        meta = {"seq": 0, "stamp": 0.0, "config_hash": "0" * 16, "sonar": {"mode": "lf"}}
        if version is not None:
            meta["generator_version"] = version
        (d / "ping_00000.json").write_text(json.dumps(meta))
        scenarios.append({"id": sid, "status": "done", "split": "train", "kind": "leak"})
    (root / "manifest.json").write_text(json.dumps({"scenarios": scenarios}))


def test_one_generator_is_quiet(tmp_path, recwarn):
    fake_dataset(tmp_path, {"s0001": "m750d-1/abc1234/deadbeefdeadbeef",
                            "s0002": "m750d-1/abc1234/deadbeefdeadbeef"})
    got = dataset.check_one_generator(tmp_path, "train")
    assert list(got) == ["m750d-1/abc1234/deadbeefdeadbeef"]
    assert not [w for w in recwarn if w.category is RuntimeWarning]


def test_two_generators_in_one_split_warn(tmp_path):
    fake_dataset(tmp_path, {"s0001": "m750d-1/abc1234/deadbeefdeadbeef",
                            "s0002": "m750d-2/abc1234/deadbeefdeadbeef"})
    with pytest.warns(RuntimeWarning, match="2 different generators"):
        got = dataset.check_one_generator(tmp_path, "train")
    assert len(got) == 2


def test_a_dataset_written_before_the_field_existed_reports_unknown(tmp_path):
    fake_dataset(tmp_path, {"s0001": None})
    assert list(dataset.generator_versions(tmp_path, "train")) == ["unknown"]


def test_generator_versions_reads_one_ping_per_scenario(tmp_path):
    """Not all 36 000 of them: a scenario is written by one process in one run, so its pings cannot disagree."""
    fake_dataset(tmp_path, {"s0001": "v1", "s0002": "v1"})
    d = tmp_path / "s0001"
    for i in (1, 2, 3):
        np.savez(d / f"ping_{i:05d}.npz", polar_db=np.zeros((4, 4), np.float16))
        (d / f"ping_{i:05d}.json").write_text(json.dumps({"generator_version": "SHOULD-NOT-BE-READ"}))
    assert list(dataset.generator_versions(tmp_path, "train")) == ["v1"]
