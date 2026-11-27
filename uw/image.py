"""`uw image dev|thor|rov`: build an image tagged with the commit's short SHA; `uw image promote`: move a tag.

    uw image dev|thor|rov [--platform P[,P]] [--push REGISTRY] [--dry-run]
    uw image promote rov <sha> hil-ok|rov [--dry-run]
    uw image list [dev|thor|rov]

| image | Dockerfile | default platform | what it is |
|---|---|---|---|
| uw-dev  | deploy/docker/Dockerfile.stack | linux/amd64 | the ROS 2 Jazzy stack on linus |
| uw-thor | deploy/docker/Dockerfile.stack | linux/arm64 | the same file on the Thor (one multi-arch Dockerfile) |
| uw-rov  | deploy/docker/Dockerfile.rov   | linux/arm64 | the BlueOS ROS 2 extension with ros/src built in |

The tag is `git rev-parse --short HEAD`; a dirty tree gets `-dirty` and cannot be promoted. One platform is loaded
into the local docker; several (`--platform linux/amd64,linux/arm64`) need `--push REGISTRY` (buildx cannot load a
multi-platform image into the classic store). Building arm64 on linus needs QEMU binfmt
(`docker run --privileged --rm tonistiigi/binfmt --install arm64`, not done by this tool); on the Thor it is native.

Promotion is explicit and in order: `:<sha>` -> `:hil-ok` (it passed HIL) -> `:rov` (it may go on the vehicle).
`promote ... rov` refuses unless `:hil-ok` is the same image as `:<sha>`.
"""
from __future__ import annotations

import argparse
import shlex
import subprocess
import sys

from .targets import WS

IMAGES = {
    "dev": ("uw-dev", "deploy/docker/Dockerfile.stack", "deploy/docker", "linux/amd64"),
    "thor": ("uw-thor", "deploy/docker/Dockerfile.stack", "deploy/docker", "linux/arm64"),
    "rov": ("uw-rov", "deploy/docker/Dockerfile.rov", ".", "linux/arm64"),
}
STAGES = ("hil-ok", "rov")


def git_sha() -> str:
    sha = subprocess.run(["git", "-C", str(WS), "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                         check=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(WS), "status", "--porcelain", "--untracked-files=no"],
                           capture_output=True, text=True).stdout.strip()
    return sha + ("-dirty" if dirty else "")


def run(argv: list[str], dry: bool) -> int:
    print("  $ " + shlex.join(argv))
    return 0 if dry else subprocess.run(argv).returncode


def image_id(ref: str) -> str | None:
    p = subprocess.run(["docker", "image", "inspect", "-f", "{{.Id}}", ref], capture_output=True, text=True)
    return p.stdout.strip() if p.returncode == 0 else None


def build(kind: str, args) -> int:
    repo, dockerfile, context, platform = IMAGES[kind]
    platform = args.platform or platform
    tag = git_sha()
    ref = f"{args.push.rstrip('/')}/{repo}:{tag}" if args.push else f"{repo}:{tag}"
    multi = "," in platform
    if multi and not args.push:
        print("uw image: several platforms need --push REGISTRY (buildx cannot --load them)", file=sys.stderr)
        return 2
    argv = ["docker", "buildx", "build", "--platform", platform, "-f", str(WS / dockerfile), "-t", ref,
            "--build-arg", f"UW_REVISION={tag}", "--label", f"org.opencontainers.image.revision={tag}",
            "--push" if args.push else "--load",
            str(WS / context)]
    print(f"uw image {kind}: {ref} ({platform})")
    return run(argv, args.dry_run)


def promote(kind: str, sha: str, stage: str, dry: bool) -> int:
    repo = IMAGES[kind][0]
    if stage not in STAGES:
        print(f"uw image promote: stage must be one of {STAGES}", file=sys.stderr)
        return 2
    if sha.endswith("-dirty"):
        print("uw image promote: a -dirty image is never promoted", file=sys.stderr)
        return 2
    src = image_id(f"{repo}:{sha}")
    if not src:
        print(f"uw image promote: {repo}:{sha} not found locally", file=sys.stderr)
        return 1
    if stage == "rov" and image_id(f"{repo}:hil-ok") != src:
        print(f"uw image promote: {repo}:hil-ok is not {repo}:{sha}; promote it to hil-ok first (after HIL)",
              file=sys.stderr)
        return 1
    return run(["docker", "tag", f"{repo}:{sha}", f"{repo}:{stage}"], dry)


def main(argv: list[str]) -> int:
    if argv[:1] == ["promote"]:
        ap = argparse.ArgumentParser(prog="uw image promote")
        ap.add_argument("kind", choices=IMAGES)
        ap.add_argument("sha")
        ap.add_argument("stage", choices=STAGES)
        ap.add_argument("--dry-run", action="store_true")
        a = ap.parse_args(argv[1:])
        return promote(a.kind, a.sha, a.stage, a.dry_run)
    if argv[:1] == ["list"]:
        for k in argv[1:] or list(IMAGES):
            subprocess.run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.CreatedSince}}",
                            IMAGES[k][0]])
        return 0
    ap = argparse.ArgumentParser(prog="uw image", description=__doc__.splitlines()[0])
    ap.add_argument("kind", choices=IMAGES)
    ap.add_argument("--platform", default=None, help="override, e.g. linux/amd64 to test the rov image on linus")
    ap.add_argument("--push", default="", metavar="REGISTRY", help="push instead of loading (needed for multi-arch)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
    return build(args.kind, args)
