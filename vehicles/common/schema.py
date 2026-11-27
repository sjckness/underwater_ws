"""The schema of ``vehicles/<name>/vehicle.yaml``: every key, its SI unit, its shape, and whether it may be unknown.

One vehicle = one folder under ``vehicles/`` with a ``vehicle.yaml``. The file is read by
``vehicles.common.dynamics.load_params`` (the Fossen model, the ArduSub JSON physics, the pool) and written, only on
request, by ``sysid`` (``uw sysid <vehicle> --write``). This module is the contract between them:

- **units are SI and explicit**: a key's unit is in ``FIELDS`` (and, for most keys, in the key's suffix:
  ``_kg``, ``_n``, ``_m``, ``_s``, ``_us``, ``_hz``). Fossen's hydrodynamic coefficients follow his notation, so
  their units come from the derivative they are (``X_u`` is N per m/s, ``K_pp`` is N m per (rad/s)^2);
- **frames**: every vector is in the body frame FRD (x forward, y starboard, z down), origin at the centre of
  gravity; ``vehicles.common.frames`` is the only place that converts to the repo's tank/ENU and body/FLU frames;
- **unknown values are ``null`` with a ``# TODO``** in the file, never a made-up number. A ``null`` is accepted only
  where ``FIELDS`` marks the key ``nullable``; anywhere else the loader refuses the file and says which key;
- **provenance stays in the file**: each value carries its source as a comment (the V / UNVERIFIED / GUESS tags and
  the [BENZON22]-style references of ``vehicles/bluerov2_heavy/vehicle.yaml``). YAML comments are not data, so
  this module cannot check them: a review can.

The thruster allocation matrix is not a key: it is built from each thruster's position and axis
(``dynamics.allocation_matrix``), so the geometry and the matrix cannot disagree. The thruster curve is the
``t200`` block (PWM range, deadband, maxima at a voltage, curve exponent or table, first-order lag).

    python -m vehicles.common.schema [vehicles/bluerov2_heavy/vehicle.yaml]     # check a file, print its values
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import yaml


class SchemaError(ValueError):
    """A vehicle.yaml does not match the schema. ``problems`` lists every mismatch, not only the first."""

    def __init__(self, path, problems: list[str]):
        self.problems = problems
        super().__init__(f"{path}: " + "; ".join(problems))


@dataclass(frozen=True)
class Field:
    key: str            # dotted path in the YAML
    unit: str           # SI unit; "1" dimensionless, "" for text/flags
    kind: str           # number | int | bool | text | vec3 | pair | table
    nullable: bool = False
    doc: str = ""


_ADDED = [("X_udot", "kg"), ("Y_vdot", "kg"), ("Z_wdot", "kg"),
          ("K_pdot", "kg m^2/rad"), ("M_qdot", "kg m^2/rad"), ("N_rdot", "kg m^2/rad")]
_DLIN = [("X_u", "N s/m"), ("Y_v", "N s/m"), ("Z_w", "N s/m"),
         ("K_p", "N m s/rad"), ("M_q", "N m s/rad"), ("N_r", "N m s/rad")]
_DQUAD = [("X_uu", "N s^2/m^2"), ("Y_vv", "N s^2/m^2"), ("Z_ww", "N s^2/m^2"),
          ("K_pp", "N m s^2/rad^2"), ("M_qq", "N m s^2/rad^2"), ("N_rr", "N m s^2/rad^2")]

FIELDS: tuple[Field, ...] = (
    Field("model", "", "text", doc="vehicle name"),
    Field("frame", "", "text", doc="ArduSub frame (vectored_6dof = Heavy, vectored = standard)"),
    Field("body.mass_kg", "kg", "number", doc="dry mass"),
    Field("body.weight_n", "N", "number", doc="mass_kg * g, checked to 1 %"),
    Field("body.buoyancy_n", "N", "number", doc="rho g V plus trim, as a force (net buoyancy = this - weight)"),
    Field("body.volume_m3", "m^3", "number", nullable=True, doc="displaced volume; informational"),
    Field("body.hull_height_m", "m", "number", nullable=True, doc="partial-submersion surface model"),
    Field("body.hull_size_m", "m", "vec3", nullable=True, doc="length, width, height: the stand-in box of the pool scene"),
    Field("body.r_g_frd_m", "m", "vec3", doc="centre of gravity; must be zero (the body origin)"),
    Field("body.r_b_frd_m", "m", "vec3", doc="centre of buoyancy"),
    Field("body.inertia_frd_kg_m2", "kg m^2", "vec3", doc="Ixx, Iyy, Izz about the CG"),
    Field("body.inertia_products_kg_m2", "kg m^2", "vec3", doc="Ixy, Ixz, Iyz"),
    *[Field(f"added_mass.{k}", u, "number", doc="Fossen added mass, <= 0") for k, u in _ADDED],
    *[Field(f"damping_linear.{k}", u, "number", doc="Fossen linear damping, <= 0") for k, u in _DLIN],
    *[Field(f"damping_quadratic.{k}", u, "number", doc="Fossen quadratic damping, <= 0") for k, u in _DQUAD],
    Field("thrusters.count", "1", "int", doc="number of entries in thrusters.list"),
    Field("t200.pwm_min_us", "us", "number"),
    Field("t200.pwm_mid_us", "us", "number"),
    Field("t200.pwm_max_us", "us", "number"),
    Field("t200.deadband_us", "us", "pair", doc="no rotation between these two"),
    Field("t200.voltage_v", "V", "number", doc="the voltage the two maxima belong to"),
    Field("t200.thrust_fwd_max_n", "N", "number"),
    Field("t200.thrust_rev_max_n", "N", "number"),
    Field("t200.curve_exponent", "1", "number", doc="thrust = max * |u|^exponent"),
    Field("t200.table", "1, N", "table", nullable=True, doc="[[u, N], ...] overriding the exponent; [] or null = none"),
    Field("t200.tau_s", "s", "number", doc="first-order thrust lag"),
    Field("integrator.rate_hz", "Hz", "number", doc="simulation step rate"),
    Field("integrator.method", "", "text", doc="rk4 | semi_implicit"),
    Field("bound.enabled", "", "bool", doc="soft walls of the tank (simulation only)"),
    Field("bound.margin_m", "m", "number"),
    Field("bound.sand_extra_m", "m", "number"),
    Field("bound.stiffness_n_per_m", "N/m", "number"),
    Field("bound.damping_n_s_per_m", "N s/m", "number"),
    Field("bound.surface_stiffness_n_per_m", "N/m", "number"),
    Field("bound.surface_damping_n_s_per_m", "N s/m", "number"),
)

THRUSTER_FIELDS: tuple[Field, ...] = (
    Field("name", "", "text"),
    Field("channel", "1", "int", doc="ArduSub motor number - 1 (JSON pwm index)"),
    Field("position_frd_m", "m", "vec3", doc="thrust application point"),
    Field("axis_frd", "1", "vec3", doc="unit vector a positive command pushes the body along"),
)

_MISSING = object()


def _get(cfg: dict, key: str):
    node = cfg
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return _MISSING
        node = node[part]
    return node


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _check(value, field: Field, where: str) -> list[str]:
    if value is _MISSING:
        return [] if field.nullable else [f"{where}: missing ({field.unit or field.kind})"]
    if value is None:
        return [] if field.nullable else [f"{where}: null, but it is required ({field.unit or field.kind}); "
                                          "find the value or keep the vehicle out of the simulation"]
    k = field.kind
    ok = {
        "number": _is_number(value),
        "int": isinstance(value, int) and not isinstance(value, bool),
        "bool": isinstance(value, bool),
        "text": isinstance(value, str) and value != "",
        "vec3": isinstance(value, list) and len(value) == 3 and all(_is_number(x) for x in value),
        "pair": isinstance(value, list) and len(value) == 2 and all(_is_number(x) for x in value),
        "table": isinstance(value, list) and all(isinstance(r, list) and len(r) == 2 and all(_is_number(x) for x in r)
                                                 for r in value),
    }[k]
    return [] if ok else [f"{where}: expected {k} [{field.unit}], got {value!r}"]


def validate(cfg) -> list[str]:
    """Every way ``cfg`` (a parsed vehicle.yaml) breaks the schema; empty if it matches."""
    if not isinstance(cfg, dict):
        return ["expected a mapping at top level"]
    problems = []
    for f in FIELDS:
        problems += _check(_get(cfg, f.key), f, f.key)
    lst = _get(cfg, "thrusters.list")
    if not isinstance(lst, list) or not lst:
        problems.append("thrusters.list: expected a non-empty list of thrusters")
    else:
        for i, t in enumerate(lst):
            if not isinstance(t, dict):
                problems.append(f"thrusters.list[{i}]: expected a mapping")
                continue
            for f in THRUSTER_FIELDS:
                problems += _check(t.get(f.key, _MISSING), f, f"thrusters.list[{i}].{f.key}")
    return problems


def load(path: str | Path) -> dict:
    """Parse ``path`` and check it against the schema; raises ``SchemaError`` listing every problem."""
    path = Path(path)
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    problems = validate(cfg)
    if problems:
        raise SchemaError(path, problems)
    return cfg


def table(cfg: dict) -> str:
    """The file's values with their units, as a Markdown table (what the CLI prints)."""
    rows = ["| key | value | unit |", "|---|---|---|"]
    for f in FIELDS:
        v = _get(cfg, f.key)
        rows.append(f"| `{f.key}` | {'(absent)' if v is _MISSING else v} | {f.unit} |")
    for i, t in enumerate(_get(cfg, "thrusters.list") or []):
        rows.append(f"| `thrusters.list[{i}]` | {t.get('name')} ch {t.get('channel')} at {t.get('position_frd_m')} "
                    f"along {t.get('axis_frd')} | m, 1 |")
    return "\n".join(rows)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    default = Path(__file__).resolve().parents[1] / "bluerov2_heavy" / "vehicle.yaml"
    path = Path(argv[0]) if argv else default
    try:
        cfg = load(path)
    except SchemaError as e:
        print("\n".join([f"{path}: does not match the schema"] + [f"  - {p}" for p in e.problems]), file=sys.stderr)
        return 1
    print(table(cfg))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
