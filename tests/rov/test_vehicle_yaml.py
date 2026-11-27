"""vehicles/<name>/vehicle.yaml: the schema, and the dynamics read from the moved files exactly as before (host, numpy).

The golden file was written at e245944, before the restructure, from config/rov_bluerov2.yaml and
config/rov_bluerov2_vectored.yaml with uw/rov/dynamics.py, by the very run below (`command`: a fixed sinusoid per
thruster, 3 s from rest at 2 m depth in a 9 m room, recorded every 200 steps). The same run on today's files and
code must give the same matrices and the same trajectory.
"""
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from vehicles.common import dynamics as D
from vehicles.common import schema

WS = Path(__file__).resolve().parents[2]
GOLDEN = json.loads((Path(__file__).with_name("golden_vehicle_dynamics.json")).read_text())
FILES = {"bluerov2_heavy": WS / "vehicles" / "bluerov2_heavy" / "vehicle.yaml",
         "bluerov2": WS / "vehicles" / "bluerov2" / "vehicle.yaml"}


def command(t, s):
    k = np.arange(len(s.thrust_n))
    return 0.6 * np.sin(2 * np.pi * (0.3 + 0.1 * k) * t + k)


@pytest.mark.parametrize("name", sorted(FILES))
def test_every_vehicle_file_matches_the_schema(name):
    assert schema.validate(yaml.safe_load(FILES[name].read_text())) == []


def test_the_default_vehicle_is_the_heavy():
    assert D.DEFAULT_PARAMS == FILES["bluerov2_heavy"]
    assert D.load_params().model == "BlueROV2 Heavy"


@pytest.mark.parametrize("name", sorted(FILES))
def test_dynamics_from_the_moved_file_are_unchanged(name):
    g = GOLDEN["runs"][name]
    p = D.load_params(FILES[name])
    for key in ("M", "allocation", "d_lin", "d_quad"):
        np.testing.assert_allclose(getattr(p, key), np.array(g[key]), rtol=0, atol=1e-12, err_msg=key)
    assert (p.buoyancy_n, p.weight_n, p.dt) == (g["buoyancy_n"], g["weight_n"], g["dt"])
    v = D.make_vehicle(FILES[name], tank_size=(9.0, 9.0, 9.0), sand_base=0.3, water_level=8.9)
    _, tr = D.simulate(v, D.State.rest(len(p.thrusters), position_ned=(0.0, 0.0, 2.0)), command, 3.0,
                       record_every=200)
    for key in ("t", "position_ned", "quat_frd_ned", "velocity_frd", "rate_frd", "thrust_n"):
        np.testing.assert_allclose(getattr(tr, key), np.array(g[key]), rtol=1e-9, atol=1e-12, err_msg=key)


def heavy():
    return yaml.safe_load(FILES["bluerov2_heavy"].read_text())


def test_a_missing_or_null_required_value_is_refused_by_name(tmp_path):
    cfg = heavy()
    del cfg["added_mass"]["Z_wdot"]
    cfg["body"]["mass_kg"] = None
    cfg["thrusters"]["list"][2]["axis_frd"] = [1.0, 0.0]
    p = tmp_path / "vehicle.yaml"
    p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(schema.SchemaError) as e:
        schema.load(p)
    joined = " ".join(e.value.problems)
    assert "added_mass.Z_wdot: missing" in joined
    assert "body.mass_kg: null, but it is required" in joined
    assert "thrusters.list[2].axis_frd" in joined
    with pytest.raises(D.VehicleError, match="does not match vehicles/common/schema.py"):
        D.load_params(p)


def test_nullable_values_may_be_unknown(tmp_path):
    cfg = heavy()
    cfg["body"]["volume_m3"] = None
    cfg["body"]["hull_size_m"] = None
    cfg["t200"]["table"] = None
    p = tmp_path / "vehicle.yaml"
    p.write_text(yaml.safe_dump(cfg))
    assert schema.load(p)["body"]["volume_m3"] is None
    assert D.load_params(p).t200.table is None


def test_every_field_has_a_unit_or_is_text():
    for f in schema.FIELDS + schema.THRUSTER_FIELDS:
        assert f.unit or f.kind in ("text", "bool"), f.key


def test_the_pool_box_size_comes_from_the_vehicle_file():
    pytest.importorskip("pxr")
    from sim import pool_scene
    from uw import config
    cfg = config.load(config.POOL_CONFIG)
    assert pool_scene.hull_size(cfg) == (0.457, 0.575, 0.254)
