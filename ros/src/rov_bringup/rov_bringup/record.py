"""rosbag2 recording profiles of the real ROV (profiles/<name>.yaml): the topic list and the MCAP writer options.

    ros2 run rov_bringup record [--profile light] [--vehicle rov1] [--out /root/persistent_ws/bags] [--print]

Builds and execs `ros2 bag record` for a profile, so the launch file, a terminal on the Pi and the tests all use one
definition. Pure Python apart from the final exec: `command()` is what the tests check.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import yaml

#: where the bags go inside the BlueOS ROS 2 extension: the persistent folder, i.e. /usr/blueos/extensions/ros2/bags
#: on the Pi, which `uw logs pull rov` copies off the vehicle
DEFAULT_OUT = "/root/persistent_ws/bags"


def share_dir() -> Path:
    """The installed share/rov_bringup, or the source package when run from a checkout (tests)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("rov_bringup"))
    except Exception:                                   # not installed: the source tree
        return Path(__file__).resolve().parents[1]


def load_profile(name: str, share: Path | None = None) -> dict:
    share = share or share_dir()
    path = Path(name) if name.endswith(".yaml") else share / "profiles" / f"{name}.yaml"
    prof = yaml.safe_load(path.read_text(encoding="utf-8"))
    if prof.get("storage") != "mcap" or not prof.get("topics"):
        raise ValueError(f"{path}: a profile needs storage: mcap and a topics mapping")
    prof["_path"] = str(path)
    return prof


def topics(prof: dict, vehicle: str) -> dict:
    """{topic: type} with the vehicle namespace filled in."""
    return {t.format(vehicle=vehicle): ty for t, ty in prof["topics"].items()}


def command(profile: str = "light", vehicle: str = "rov1", out: str = DEFAULT_OUT, stamp: str | None = None,
            share: Path | None = None) -> list[str]:
    share = share or share_dir()
    prof = load_profile(profile, share)
    stamp = stamp or time.strftime("%Y-%m-%d_%H-%M-%S")
    name = Path(prof["_path"]).stem
    storage_cfg = share / "config" / prof["storage_config"]
    return ["ros2", "bag", "record", "-s", "mcap", "--storage-config-file", str(storage_cfg),
            "-o", f"{out}/{stamp}_{name}", "--max-bag-duration", str(int(prof.get("max_bag_duration_s", 0))),
            "--topics", *topics(prof, vehicle)]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="ros2 run rov_bringup record", description=__doc__.splitlines()[0])
    ap.add_argument("--profile", default="light", help="profiles/<name>.yaml, or a path to a profile")
    ap.add_argument("--vehicle", default="rov1", help="namespace: /<vehicle>/mavros/...")
    ap.add_argument("--out", default=DEFAULT_OUT, help="parent directory of the bag")
    ap.add_argument("--print", action="store_true", help="print the command instead of running it")
    args = ap.parse_args(argv)
    cmd = command(args.profile, args.vehicle, args.out)
    if args.print:
        print(" ".join(cmd))
        return 0
    Path(args.out).mkdir(parents=True, exist_ok=True)
    os.execvp(cmd[0], cmd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
