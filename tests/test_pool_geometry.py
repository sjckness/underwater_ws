"""physics/pool_geometry.py: the pool's solids and the DVL's ray casting against them (host, numpy only)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uw import config  # noqa: E402
from physics.pool_geometry import Box, Cylinder, PoolBottom, Quad, solids  # noqa: E402

DOWN = np.array([[0.0, 0.0, -1.0]])


@pytest.fixture(scope="module")
def cfg():
    return config.load(config.POOL_CONFIG)


@pytest.fixture(scope="module")
def bottom(cfg):
    return PoolBottom(cfg)


def hit(bottom, origin, direction):
    t, n, roles = bottom.ray_bottom(np.asarray(origin, float), np.atleast_2d(np.asarray(direction, float)))
    return float(t[0]), n[0], roles[0]


def test_straight_down_on_the_floor(bottom):
    t, n, role = hit(bottom, (5.0, 3.0, 2.5), DOWN)
    assert role == "floor" and t == pytest.approx(2.5) and np.allclose(n, [0, 0, 1])


@pytest.mark.parametrize("origin, direction, normal", [
    ((0.3, 5.0, 2.0), (-1.0, 0.0, -0.1), (1, 0, 0)),
    ((9.7, 5.0, 2.0), (1.0, 0.0, -0.1), (-1, 0, 0)),
    ((5.0, 0.3, 2.0), (0.0, -1.0, -0.1), (0, 1, 0)),
    ((2.0, 9.7, 2.0), (0.0, 1.0, -0.1), (0, -1, 0)),
])
def test_the_walls_stop_a_beam_and_face_the_water(bottom, origin, direction, normal):
    """Unlike the tank's (physics.geometry.ray_bottom), the pool's walls are occluders: a tilted beam near one hits it."""
    d = np.asarray(direction) / np.linalg.norm(direction)
    t, n, role = hit(bottom, origin, d)
    assert role == "wall" and np.allclose(n, normal)
    p = np.asarray(origin) + t * d
    k = int(np.flatnonzero(normal)[0])
    assert p[k] == pytest.approx(0.0 if normal[k] > 0 else 10.0)


def test_a_beam_up_leaves_through_the_surface(bottom):
    t, n, role = hit(bottom, (5.0, 5.0, 2.0), (0.1, 0.0, 1.0))
    assert math.isinf(t) and role is None and not n.any()


def test_a_beam_beyond_t_max_is_no_return(bottom):
    t, _, role = bottom.ray_bottom(np.array([5.0, 3.0, 2.5]), DOWN, t_max=2.0)
    assert math.isinf(t[0]) and role[0] is None


@pytest.mark.parametrize("xy, role, top", [((3.0, 6.0), "block", 0.2), ((6.5, 6.5), "pipe", 0.3),
                                           ((6.0, 6.5), "pipe", 0.3)])
def test_props_on_the_floor_are_hit_from_above(bottom, xy, role, top):
    t, n, r = hit(bottom, (*xy, 1.0), DOWN)
    assert r == role and t == pytest.approx(1.0 - top) and np.allclose(n, [0, 0, 1])


def test_the_pipe_end_cap_and_its_side(bottom):
    t, n, role = hit(bottom, (5.0, 6.5, 0.15), (1.0, 0.0, 0.0))
    assert role == "pipe" and t == pytest.approx(0.75) and np.allclose(n, [-1, 0, 0])
    t, n, role = hit(bottom, (6.5, 6.0, 0.15), (0.0, 1.0, 0.0))
    assert role == "pipe" and t == pytest.approx(0.35) and np.allclose(n, [0, -1, 0])


def test_the_marker_board_faces_whoever_looks_at_it(bottom):
    t, n, role = hit(bottom, (5.0, 7.0, 0.3), (0.0, 1.0, 0.0))
    assert role == "board" and t == pytest.approx(1.5) and np.allclose(n, [0, -1, 0])
    t, n, role = hit(bottom, (5.0, 9.5, 0.3), (0.0, -1.0, 0.0))
    assert role == "board" and t == pytest.approx(1.0) and np.allclose(n, [0, 1, 0])


def test_the_ladder_is_there(bottom):
    t, _, role = hit(bottom, (8.0, 1.0, 4.0), (0.0, -1.0, 0.0))     # rail a at (8.0, 0.15), radius 0.02
    assert role == "ladder" and t == pytest.approx(1.0 - 0.15 - 0.02)


def test_the_solids_follow_the_switches(cfg):
    kinds = {type(s) for s in solids(cfg)}
    assert kinds == {Box, Cylinder, Quad}
    off = config.load(config.POOL_CONFIG, overrides=["pool.props=false", "pool.ladder=false"])
    assert solids(off) == []
    t, _, role = hit(PoolBottom(off), (3.0, 6.0, 1.0), DOWN)
    assert role == "floor" and t == pytest.approx(1.0)


def test_every_role_needs_a_material(cfg):
    import copy
    bad = copy.deepcopy(cfg)
    del bad["acoustics"]["roles"]["pipe"]
    with pytest.raises(config.ConfigError, match="pipe"):
        PoolBottom(bad)
