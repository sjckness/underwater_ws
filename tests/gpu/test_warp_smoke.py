"""Placeholder Warp test (M0): a kernel compiles, runs on the GPU and returns the right numbers.

Runs in the container (scripts/uw.sh test); skipped where Warp is not importable (host pytest).
"""
import os

import numpy as np
import pytest

wp = pytest.importorskip("warp", reason="Warp tests run in the container: scripts/uw.sh test")


@wp.kernel
def axpy(a: float, x: wp.array(dtype=float), y: wp.array(dtype=float)):
    i = wp.tid()
    y[i] = a * x[i] + y[i]


def test_kernel_runs_on_the_gpu():
    wp.init()
    assert wp.is_cuda_available(), "Warp sees no CUDA device"
    dev = wp.get_device("cuda:0")
    x = np.arange(4096, dtype=np.float32)
    xd = wp.array(x, dtype=float, device=dev)
    yd = wp.ones(len(x), dtype=float, device=dev)
    wp.launch(axpy, dim=len(x), inputs=[2.0, xd, yd], device=dev)
    np.testing.assert_allclose(yd.numpy(), 2.0 * x + 1.0)


def test_kernel_cache_is_the_mounted_one():
    """docker/compose.yaml points WARP_CACHE_PATH at ~/.cache/uw_warp; compiled kernels must land there."""
    want = os.environ.get("WARP_CACHE_PATH")
    if not want:
        pytest.skip("WARP_CACHE_PATH not set (not in the uw container)")
    wp.init()
    assert os.path.realpath(wp.config.kernel_cache_dir) == os.path.realpath(want)
