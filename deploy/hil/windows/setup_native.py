"""Genera uw/runtime.local.yaml per far girare underwater_ws in modalita' nativa su Windows.

Rileva da se':
  - isaac_root  : la cartella del pacchetto isaacsim installato con pip
  - warp_path   : la cartella omni.warp.core-* dentro extscache (il template del repo ha +lx64, solo Linux)
  - usd_path    : la cartella omni.usd.libs-* dentro extscache
  - ros2 bridge : quali distribuzioni ROS 2 sono incluse (per il sink ros, piu' avanti)

Uso, dalla radice di underwater_ws, con l'ambiente (isaac) attivo:

    python deploy/hil/windows/setup_native.py                      mostra cosa ha trovato, senza scrivere
    python deploy/hil/windows/setup_native.py --write               scrive uw/runtime.local.yaml
    python deploy/hil/windows/setup_native.py --write --data-root F:/underwater_data

Scrive un solo file, uw/runtime.local.yaml, che e' nel .gitignore del repo.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[3]
LOCAL = WS / "uw" / "runtime.local.yaml"


def isaac_root() -> Path:
    """La radice dell'installazione pip di Isaac Sim: la cartella che contiene extscache."""
    try:
        import isaacsim
    except ImportError:
        sys.exit("setup_native: 'import isaacsim' non funziona. Attiva l'ambiente con Isaac Sim installato.")
    pkg = Path(isaacsim.__file__).resolve().parent
    for cand in (pkg, pkg.parent, *pkg.parent.glob("isaacsim*")):
        if (cand / "extscache").is_dir():
            return cand
    sys.exit(f"setup_native: nessuna cartella 'extscache' trovata vicino a {pkg}.\n"
             "Reinstalla con l'extra [extscache]:\n"
             '  pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com')


def pick(cache: Path, prefix: str) -> Path | None:
    """La cartella <prefix>-* dentro extscache; se ce ne sono piu' di una, la piu' recente per nome."""
    hits = sorted((p for p in cache.glob(prefix + "-*") if p.is_dir()), key=lambda p: p.name)
    return hits[-1] if hits else None


def ros_distros(root: Path) -> list[str]:
    bridge = root / "exts" / "isaacsim.ros2.bridge"
    if not bridge.is_dir():
        return []
    return sorted(p.name for p in bridge.iterdir() if p.is_dir() and (p / "rclpy").is_dir())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="scrive uw/runtime.local.yaml")
    ap.add_argument("--data-root", default="F:/underwater_data",
                    help="dove scrivono i dataset, fuori dal repo (default: F:/underwater_data)")
    args = ap.parse_args()

    root = isaac_root()
    cache = root / "extscache"
    warp = pick(cache, "omni.warp.core")
    usd = pick(cache, "omni.usd.libs")
    distros = ros_distros(root)

    def mark(ok: bool) -> str:
        return "ok     " if ok else "MANCA  "

    print()
    print(f"  python       {sys.executable}")
    print(f"  {mark(True)}isaac_root   {root}")
    print(f"  {mark(bool(warp))}warp         {warp.name if warp else '(nessuna omni.warp.core-* in extscache)'}")
    print(f"  {mark(bool(usd))}usd          {usd.name if usd else '(nessuna omni.usd.libs-* in extscache)'}")
    print(f"  {mark(bool(distros))}ros2 bridge  {', '.join(distros) if distros else '(nessuna distribuzione con rclpy)'}")
    print()

    if not warp:
        print("Senza la cartella di Warp, 'sim' e 'sonar-only' non partono. Controlla il contenuto di:")
        print(f"  {cache}")
        return 1

    lines = [
        "# Generato da deploy/hil/windows/setup_native.py: modalita' nativa su Windows (Isaac Sim installato con pip).",
        "# Sovrascrive uw/runtime.yaml, che punta a percorsi Linux (+lx64) e alla modalita' docker.",
        "mode: native",
        "",
        "native:",
        f"  python: {Path(sys.executable).as_posix()}",
        f"  isaac_root: {root.as_posix()}",
        "",
        "# Percorsi Windows reali: il template del repo usa +lx64, che qui non esiste.",
        f'warp_path: "{{isaac_root}}/extscache/{warp.name}"',
    ]
    if usd:
        lines.append(f'usd_path: "{{isaac_root}}/extscache/{usd.name}"')
    lines += [
        "",
        "# Fuori dal repo, su un disco con spazio.",
        f"data_root: {args.data_root}",
        "",
    ]
    text = "\n".join(lines)

    if not args.write:
        print("Anteprima di uw/runtime.local.yaml (rilancia con --write per scriverlo):")
        print()
        print(text)
        return 0

    if LOCAL.exists():
        backup = LOCAL.with_suffix(".yaml.bak")
        backup.write_text(LOCAL.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"Il file esisteva: copia salvata in {backup}")
    LOCAL.parent.mkdir(parents=True, exist_ok=True)
    LOCAL.write_text(text, encoding="utf-8")
    print(f"Scritto {LOCAL}")

    data = Path(args.data_root)
    try:
        data.mkdir(parents=True, exist_ok=True)
        print(f"Cartella dati pronta: {data}")
    except OSError as e:
        print(f"Non ho potuto creare {data}: {e}. Cambiala con --data-root.")

    print()
    print("Prossimo comando:")
    print("  python -m uw sim --gui --sink none --set lights.ambient_dome=300")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
