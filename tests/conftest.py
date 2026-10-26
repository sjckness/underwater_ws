"""Shared test setup: the uw config options (--profile, --preset, --set) and a Warp line in the report header."""
import importlib.util

import pytest

from uw import config as uw_config


def pytest_addoption(parser):
    uw_config.add_args(parser.addoption)


def pytest_report_header(config):
    opt = config.option
    lines = [f"uw config: profile={opt.profile} presets={opt.preset} set={opt.overrides}"]
    if importlib.util.find_spec("warp") is None:
        lines.append("warp: not importable here, Warp tests skip (run them with scripts/uw.sh test)")
    else:
        import warp as wp
        wp.config.quiet = True
        wp.init()
        devices = ", ".join(f"{d.alias} {d.name} (sm_{d.arch})" for d in wp.get_cuda_devices()) or "no CUDA device"
        lines.append(f"warp {wp.__version__}: {devices}; kernel cache {wp.config.kernel_cache_dir}")
    return lines


@pytest.fixture(scope="session")
def cfg(pytestconfig):
    """The config the command line selects (--profile, --preset, --set), built as every entry point builds it."""
    return uw_config.from_args(pytestconfig.option)
