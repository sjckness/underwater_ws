"""The sonar must not learn about bubbles (host, ast only).

`physics/sonar.py` and `physics/scattering.py` model an acoustic sensor and the scattering of sound. `physics/bubbles.py`
models a gas plume. The sonar takes bubbles as a duck-typed argument -- any object with `pos`, `r`, `alive`
and `cap` -- so the two can be developed, tested and replaced apart, and a sonar calibration run needs no
plume at all.

Checked by reading the import statements rather than by importing the modules: `physics/sonar.py` needs torch and
warp and so cannot be imported on a machine without them, while the boundary it is asked to respect is a
property of its source. A static check also catches an import added inside a function, which is where an
accidental dependency usually appears first.
"""
import ast
from pathlib import Path

import pytest

UW = Path(__file__).resolve().parents[1] / "physics"


def imported_modules(path: Path) -> set:
    """Every module `path` imports, at module level or inside a function, as dotted names."""
    tree = ast.parse(path.read_text())
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            base = ("." * node.level) + (node.module or "")
            out.add(base)
            out |= {f"{base}.{a.name}" if not base.endswith(".") else f"{base}{a.name}" for a in node.names}
    return out


def plain(names: set) -> set:
    """Last components, so that `from . import bubbles`, `from .bubbles import X` and `import physics.bubbles` all
    reduce to the same thing."""
    return {n.replace(".", " ").split()[-1] for n in names if n.strip(".")}


@pytest.mark.parametrize("module", ["sonar.py", "scattering.py"])
def test_the_sonar_side_does_not_import_bubbles_or_the_plume(module):
    got = plain(imported_modules(UW / module))
    assert "bubbles" not in got, f"uw/{module} imports bubbles; pass them to Sonar.ping(bubbles=...) instead"
    assert "plume" not in got, f"uw/{module} imports plume; it is the plume's caller, not its client"


def test_bubbles_does_not_import_the_sonar_either():
    """The other direction, so the independence cannot be satisfied by simply reversing the edge."""
    got = plain(imported_modules(UW / "bubbles.py"))
    assert "sonar" not in got
    assert "scattering" not in got


def test_scattering_is_a_leaf_of_the_uw_package():
    """It is pure acoustics with no state: water properties are the only thing it may need from us."""
    local = {n for n in imported_modules(UW / "scattering.py") if n.startswith(".")}
    assert plain(local) <= {"water"}, f"physics/scattering.py grew local imports: {sorted(plain(local))}"


def test_the_sonar_only_reaches_for_what_it_models():
    """A guard on scope rather than on layering: the list is small on purpose, so that adding to it is a
    deliberate act with a reason, not a drive-by import."""
    local = plain({n for n in imported_modules(UW / "sonar.py") if n.startswith(".")})
    assert local <= {"scattering", "water", "wat", "config", "geometry", "ConfigError", "get", "sonar_mode",
                     "sonar_rotation", "surface_materials"}, f"unexpected: {sorted(local)}"
