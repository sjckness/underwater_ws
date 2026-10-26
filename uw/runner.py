"""Target side of `python -m uw`, in the container or in the native Python: runs the command in this process.

SIGTERM (docker stop) becomes a KeyboardInterrupt, like Ctrl-C, so that finally blocks run and pytest reports.
Started by uw/launcher.py, which passes UW_WARP_PATH, UW_PYDEPS and UW_PYTEST_VERSION in the environment.
"""
from __future__ import annotations

import os
import runpy
import shutil
import signal
import subprocess
import sys
from pathlib import Path


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt


def run_py(args: list[str]) -> int:
    if not args:
        import code
        code.interact()
    elif args[0] == "-m":
        sys.argv = [args[1], *args[2:]]
        runpy.run_module(args[1], run_name="__main__", alter_sys=True)
    elif args[0] == "-c":
        sys.argv = ["-c", *args[2:]]
        exec(compile(args[1], "<string>", "exec"), {"__name__": "__main__"})
    else:                                               # like `python SCRIPT`: its directory first on sys.path
        script = Path(args[0]).resolve()
        sys.argv = [str(script), *args[1:]]
        sys.path[0] = str(script.parent)
        runpy.run_path(str(script), run_name="__main__")
    return 0


def _pytest():
    """pytest from this Python, or installed once (pinned) into UW_PYDEPS."""
    try:
        import pytest
        return pytest
    except ImportError:
        pass
    version = os.environ.get("UW_PYTEST_VERSION", "7.4.4")
    deps = Path(os.environ.get("UW_PYDEPS", Path.home() / ".cache" / "uw_pydeps")) / f"pytest-{version}"
    if not deps.is_dir():
        print(f"uw: installing pytest {version} into {deps} (first run only, needs network)", file=sys.stderr)
        tmp = deps.with_name(f"pytest-{version}.tmp")
        shutil.rmtree(tmp, ignore_errors=True)
        # --no-warn-conflicts: pip would complain about Isaac's own bundled packages, which are not ours to fix
        subprocess.run([sys.executable, "-m", "pip", "install", "--quiet", "--no-cache-dir", "--disable-pip-version-check",
                        "--no-warn-conflicts", "--target", str(tmp), f"pytest=={version}"], check=True)
        tmp.rename(deps)
    sys.path.insert(0, str(deps))
    import pytest
    return pytest


def run_test(args: list[str]) -> int:
    # no plugin autoload: Isaac's bundled site-packages (or a sourced ROS) must not inject pytest plugins
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    return int(_pytest().main(args))


def main(argv: list[str]) -> int:
    if os.name == "posix":
        signal.signal(signal.SIGTERM, _interrupt)
    warp = os.environ.get("UW_WARP_PATH")
    if warp and Path(warp).is_dir() and warp not in sys.path:
        sys.path.insert(1, warp)
    cmd, args = argv[0], argv[1:]
    try:
        if cmd == "py":
            return run_py(args)
        if cmd == "test":
            return run_test(args)
    except KeyboardInterrupt:
        print("uw: interrupted", file=sys.stderr)
        return 130
    print(f"uw.runner: unknown command {cmd!r}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
