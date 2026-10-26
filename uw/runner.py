"""Target side of `python -m uw`, in the container or in the native Python: runs the command in this process.

SIGTERM (docker stop) becomes a KeyboardInterrupt, like Ctrl-C, so that finally blocks run and pytest reports
(`sim` installs its own handlers: it stops its loop and closes the Kit app).
Started by uw/launcher.py, which passes UW_WARP_PATH, UW_USD_PATH, UW_ISAAC_ROOT, UW_PYDEPS and UW_PYTEST_VERSION in
the environment.
"""
from __future__ import annotations

import os
import runpy
import signal
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


def run_test(args: list[str]) -> int:
    from .deps import require
    # no plugin autoload: Isaac's bundled site-packages (or a sourced ROS) must not inject pytest plugins
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    return int(require("pytest", "pytest", os.environ.get("UW_PYTEST_VERSION", "7.4.4")).main(args))


def main(argv: list[str]) -> int:
    if os.name == "posix":
        signal.signal(signal.SIGTERM, _interrupt)
    cmd, args = argv[0], argv[1:]
    # Warp from the Isaac install; USD (pxr) too, except in the Kit app, which sets up its own
    for path in [os.environ.get("UW_WARP_PATH")] + ([] if cmd == "sim" else [os.environ.get("UW_USD_PATH")]):
        if path and Path(path).is_dir() and path not in sys.path:
            sys.path.insert(1, path)
    try:        # cache native code (cubin), not PTX: else the driver re-JITs every module in every new process
        import warp as wp
        wp.config.cuda_output = "cubin"
    except ImportError:
        pass
    try:
        if cmd == "py":
            return run_py(args)
        if cmd == "test":
            return run_test(args)
        if cmd == "sonar-only":
            from . import sonar_only
            return sonar_only.main(args)
        if cmd == "sim":
            from . import main as sim_main
            return sim_main.main(args)
    except KeyboardInterrupt:
        print("uw: interrupted", file=sys.stderr)
        return 130
    print(f"uw.runner: unknown command {cmd!r}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
