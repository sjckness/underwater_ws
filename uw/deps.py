"""Python packages the target environment may lack (the Isaac image has no pytest, no x264 ffmpeg): installed once,
pinned, into UW_PYDEPS (~/.cache/uw_pydeps; outside the image and the repo), and put on sys.path."""
from __future__ import annotations

import importlib
import os
import shutil
import subprocess
import sys
from pathlib import Path


def require(module: str, package: str, version: str):
    """``import module``, installing ``package==version`` into UW_PYDEPS first if it is missing (needs network once)."""
    try:
        return importlib.import_module(module)
    except ImportError:
        pass
    deps = Path(os.environ.get("UW_PYDEPS", Path.home() / ".cache" / "uw_pydeps")) / f"{package}-{version}"
    if not deps.is_dir():
        print(f"uw: installing {package} {version} into {deps} (first use only, needs network)", file=sys.stderr)
        tmp = deps.with_name(f"{package}-{version}.tmp")
        shutil.rmtree(tmp, ignore_errors=True)
        # --no-warn-conflicts: pip would complain about Isaac's own bundled packages, which are not ours to fix
        subprocess.run([sys.executable, "-m", "pip", "install", "--quiet", "--no-cache-dir", "--disable-pip-version-check",
                        "--no-warn-conflicts", "--target", str(tmp), f"{package}=={version}"], check=True)
        tmp.rename(deps)
    sys.path.insert(0, str(deps))
    return importlib.import_module(module)
