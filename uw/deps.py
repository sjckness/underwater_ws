"""Python packages the target environment may lack (the Isaac image has no pytest, no x264 ffmpeg): installed once,
pinned, into UW_PYDEPS (~/.cache/uw_pydeps; outside the image and the repo), and put on sys.path."""
from __future__ import annotations

import importlib
import os
import shutil
import subprocess
import sys
from pathlib import Path


def require(module: str, package: str, version: str, *, install: bool = True, no_deps: bool = False):
    """``import module``, installing ``package==version`` into UW_PYDEPS first if it is missing (needs network
    once). ``install`` False: raise ImportError instead of installing, for callers that would rather skip (a
    test must not reach the network). ``no_deps``: pass --no-deps, for a package whose dependencies the target
    environment already has.

    The directory carries the **interpreter's ABI tag**, because this cache is shared between the host (Python
    3.12) and the Isaac container (3.11) through ~/.cache/uw_pydeps: a compiled wheel installed by one is
    unimportable in the other, and the failure is a confusing "No module named <pkg>._<ext>" rather than
    anything about versions.

    ``no_deps`` matters for the same reason the path is prepended: a dependency installed into this directory
    **shadows the target environment's own**. Installing simplejpeg without it pulls a numpy in beside it, which
    would then take precedence over Isaac's for the whole sim."""
    try:
        return importlib.import_module(module)
    except ImportError:
        pass
    tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    deps = Path(os.environ.get("UW_PYDEPS", Path.home() / ".cache" / "uw_pydeps")) / f"{package}-{version}-{tag}"
    if not deps.is_dir():
        if not install:
            raise ImportError(f"{module} is not installed and {deps} does not exist")
        print(f"uw: installing {package} {version} into {deps} (first use only, needs network)", file=sys.stderr)
        tmp = deps.with_name(f"{package}-{version}-{tag}.tmp")
        shutil.rmtree(tmp, ignore_errors=True)
        # --no-warn-conflicts: pip would complain about Isaac's own bundled packages, which are not ours to fix
        subprocess.run([sys.executable, "-m", "pip", "install", "--quiet", "--no-cache-dir", "--disable-pip-version-check",
                        "--no-warn-conflicts", *(["--no-deps"] if no_deps else []),
                        "--target", str(tmp), f"{package}=={version}"], check=True)
        tmp.rename(deps)
    sys.path.insert(0, str(deps))
    return importlib.import_module(module)
