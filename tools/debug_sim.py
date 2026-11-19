"""Avvia `uw sim` come fa il launcher nativo, ma senza perdere gli errori.

Perche': uw/main.py chiude l'app Kit in un `finally`; se run() solleva un'eccezione durante il setup, la chiusura
puo' terminare il processo prima che Python stampi il traceback. Qui:
  - il traceback viene stampato subito;
  - i log INFO di uw.* (setup, running, stopped) vanno in console (Kit mostra solo i Warning);
  - a fine run viene letto il file delle statistiche (exit_reason);
  - la finestra resta aperta finche' non premi Invio (mentre aspetta, la GUI risulta "non risponde": e' normale).

Uso, dal terminale aperto da uw.bat:
    python tools\\debug_sim.py
    python tools\\debug_sim.py --gui --sink none --set lights.ambient_dome=300
Senza argomenti usa questi (GUI, nessun sink, luce ambiente 300).
"""
from __future__ import annotations

import json
import logging
import os
import sys
import traceback
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS))

from uw import launcher, main as sim_main, runner  # noqa: E402

# Stesso ambiente che launcher.run_native() passa al processo figlio.
rt = launcher.runtime()
n = rt["native"]
os.environ.update({
    "UW_WS": str(WS),
    "UW_PYDEPS": str(Path(n["pydeps"]).expanduser()),
    "UW_WARP_PATH": launcher.warp_path(rt, n.get("isaac_root")),
    "UW_PYTEST_VERSION": str(rt["pytest_version"]),
    "UW_USD_PATH": launcher.warp_path(rt, n.get("isaac_root"), "usd_path"),
    "UW_ISAAC_ROOT": n.get("isaac_root") or "",
    "UW_DATA": os.path.expanduser(str(rt.get("data_root") or "")),
})

# Log INFO di uw.* in console.
handler = logging.StreamHandler(sys.stderr)
handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
uw_log = logging.getLogger("uw")
uw_log.addHandler(handler)
uw_log.setLevel(logging.INFO)

args = sys.argv[1:] or ["--gui", "--sink", "none", "--set", "lights.ambient_dome=300"]
if "--stats-file" in args:
    stats_path = Path(args[args.index("--stats-file") + 1])
else:
    stats_path = WS / "output" / "debug_stats.json"
    stats_path.parent.mkdir(parents=True, exist_ok=True)
    args += ["--stats-file", str(stats_path)]
stats_path.unlink(missing_ok=True)          # un file vecchio darebbe un verdetto sbagliato

_run = sim_main.run


def run(app, cfg, a, stop):
    print("[debug] run(): inizio setup", flush=True)
    try:
        rc = _run(app, cfg, a, stop)
        print(f"[debug] run() terminata senza eccezioni, codice {rc}", flush=True)
        return rc
    except BaseException:
        print("[debug] ECCEZIONE in run():", flush=True)
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if stats_path.is_file():
            s = json.loads(stats_path.read_text())
            print("[debug] il ciclo principale e' partito. Statistiche:",
                  {k: s.get(k) for k in ("exit_reason", "setup_s", "sim_time_s", "wall_time_s", "counts")}, flush=True)
        else:
            print("[debug] nessun file di statistiche: il ciclo principale non e' mai partito (errore nel setup)",
                  flush=True)
        try:
            input("[debug] premi Invio per chiudere Isaac... ")
        except EOFError:
            pass


sim_main.run = run          # main.main() cerca `run` nel modulo a ogni chiamata: la sostituzione ha effetto
raise SystemExit(runner.main(["sim", *args]))
