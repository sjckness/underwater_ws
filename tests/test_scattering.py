"""Bubble scattering (host): closed forms against the Anderson modal series, geometric limits, lookup tables."""
import numpy as np
import pytest

from uw import scattering as sc
from uw import water as wat

# methane at the leak depth (1.27 bar), seawater 15 C
G, H = sc.gas_ratios(wat.density(15.0, 35.0), wat.sound_speed(15.0, 35.0), 1.27e5, 288.15, "methane")
KA = np.logspace(-1, np.log10(20.0), 1500)


@pytest.fixture(scope="module")
def modal():
    return (np.array([sc.modal_backscatter(x, G, H) for x in KA]),
            np.array([sc.modal_extinction(x, G, H) for x in KA]))


def test_backscatter_closed_form_vs_modal_series(modal):
    """Stanton (1989) high-pass vs the modal series, ka 0.1-20. Measured: mean |error| 0.7 dB, bias -0.2 dB,
    95 % of points within 1.7 dB; the maximum (12 dB) is a narrow interior resonance of the ideal sphere."""
    err = 10 * np.log10(sc.stanton_backscatter(KA, G, H) / modal[0])
    assert np.mean(np.abs(err)) < 1.0
    assert abs(np.mean(err)) < 0.5
    assert np.percentile(np.abs(err), 95) < 2.0


def test_extinction_fit_vs_modal_series(modal):
    """Two-parameter fit vs the modal series, ka 0.1-20: mean |error| 0.1 dB."""
    err = 10 * np.log10(sc.extinction_fit(KA) / modal[1])
    assert np.mean(np.abs(err)) < 0.2
    assert np.percentile(np.abs(err), 95) < 0.5


def test_geometric_limits():
    """ka >> 1: sigma_bs -> a^2/4 (specular reflection from a sphere), sigma_ext -> 2 pi a^2 (extinction paradox)."""
    big = np.linspace(60.0, 100.0, 200)
    assert np.mean([sc.modal_backscatter(x, G, H) for x in big]) == pytest.approx(0.25, rel=0.05)
    assert float(sc.stanton_backscatter(1e3, G, H)) == pytest.approx(0.25, rel=0.01)
    assert sc.modal_extinction(1000.0, G, H) == pytest.approx(2.0, rel=0.02)
    assert float(sc.extinction_fit(1e4)) == pytest.approx(2.0, rel=0.005)


def test_lookup_tables():
    t = sc.Tables.build(G, H)
    ka = np.array([0.05, 1.6, 7.3, 16.0])
    assert t.lookup(t.backscatter, ka) == pytest.approx(sc.stanton_backscatter(ka, G, H), rel=2e-3)
    assert t.lookup(t.extinction, ka) == pytest.approx(sc.extinction_fit(ka), rel=2e-3)
