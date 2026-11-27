# Migrazione a monorepo (branch `restructure`)

Questo documento è il piano e il registro della ristrutturazione: dove è finito ogni file, cosa è cambiato nel
codice per seguirlo e cosa va fatto a mano su Work, Thor e Pi del ROV dopo il pull.

Partenza: `main` a `e245944` (2026-11-27). Il lavoro non committato di `m11-yolo` (`ros/yolo_detector/`,
`tests/test_yolo_detector.py`, modifiche a README e docs) **non** è incluso: resta nel checkout principale di linus.
Quando verrà committato andrà in `ros/src/yolo_detector/` e il suo `parents[3]` diventerà `parents[4]` (vedi §3.4).

Regole seguite: solo `git mv` nel primo commit (la storia dei file resta, `git log --follow`); nel secondo solo
import e percorsi; nessuna modifica alla logica di fisica, sonar o dinamica; nessuna nuova dipendenza.

## 1. Struttura

```
underwater_ws/
├── physics/            acustica, bolle, plume, acqua, geometria della vasca/piscina — numpy/Warp, niente ROS
│   └── tools/          anteprime e confronto con i dati reali
├── vehicles/
│   ├── common/         dinamica Fossen, frame, DVL A50, server fisico ArduSub JSON, schema vehicle.yaml
│   ├── bluerov2_heavy/ vehicle.yaml (+ asset Isaac generati, non in git)
│   ├── bluerov2/       vehicle.yaml del BlueROV2 standard a 6 thruster (HIL)            ← non previsto, §2.2
│   ├── momorov/        segnaposto (il repo sjckness/momoROV è vuoto)
│   └── tools/          misure della dinamica e della deriva del DVL
├── sim/                app Isaac, scene (vasca, room9, piscina), sink, ROS bridge di Isaac, pose/traiettorie
│   ├── config/         tank.yaml, preset, profili hardware, piscina, traiettorie
│   └── tools/          registrazioni, smoke test della piscina, stub fisico per SITL
├── ros/src/            workspace colcon (Jazzy)
├── robot/bluerov2/     il veicolo reale: parametri ArduSub, BlueOS, versioni, checklist
├── sysid/              dai log (.BIN + bag) ai parametri di vehicles/bluerov2_heavy
├── ml/                 dataset (loader, sweep, conversione YOLO/COCO), sweep config
├── deploy/
│   ├── docker/         immagini e compose (Isaac, stack ROS, ROV, ArduSub SITL, piscina)
│   ├── hil/            windows/, thor/, sysctl, generatore di profili DDS
│   └── targets/        linus/work/thor/rov: *.example in git, *.local ignorati
├── docs/
├── tests/              tutti i test (un solo pytest.ini)                                 ← resta in radice, §2.5
├── uw/                 CLI (`python -m uw …`), configurazione, runtime.yaml               (+ slam/, §2.3)
├── uw.bat, uw.sh       ingressi Windows e Linux
```

## 2. Scostamenti dalla struttura proposta

1. **I pacchetti Python sono le cartelle.** `physics`, `sim`, `vehicles`, `ml` diventano pacchetti di primo livello
   (`from physics import sonar`, `from vehicles.common import dynamics`), come `uw` oggi. Nessun nome collide con
   pacchetti installati sull'host, nell'ambiente ROS Jazzy o nel Python di Isaac (verificato). `uw.config` resta in
   `uw/`: è la configurazione di tutto il sistema e la importano tutti.
2. **`vehicles/bluerov2/`** (non previsto): `config/rov_bluerov2_vectored.yaml` è il BlueROV2 *standard* a 6
   thruster che l'HIL vola (`hil.parm`, FRAME_CONFIG 1). Non è il Heavy, quindi non può stare in `bluerov2_heavy/`;
   una cartella per veicolo è la regola di `vehicles/`.
3. **`uw/slam/` resta dov'è — senza casa ovvia.** È il front end SLAM dal sonar (stima, non fisica, non sim, non
   ROS). Candidati: una cartella `nav/` quando la stima crescerà, o un nodo in `ros/src/`. Da decidere.
4. **`.sjck/console/chat.json` resta — senza casa ovvia.** È il log di chat di un altro strumento, tracciato per
   errore? Non l'ho tolto da git (sarebbe una cancellazione).
5. **`tests/` resta in radice.** Un solo `pytest.ini`, i marcatori `gpu/` e `isaac/` sono per requisito di
   runtime, non per area, e molti test incrociano le aree (sim + veicolo + fisica). Dividerli per area si può fare
   dopo senza toccare il codice.
6. **Una sola immagine per linus e Thor.** `docker/stack/Dockerfile` è già multi-arch su `ros:jazzy-ros-base`:
   diventa `deploy/docker/Dockerfile.stack` e serve sia `dev` (amd64) sia `thor` (arm64). Un `Dockerfile.thor`
   separato sarebbe una copia. `Dockerfile.rov` è diverso (base `itskalvik/blueos-ros2`) ed è nuovo (Fase 3.4).
7. **`ros/mavros_rov1/` → `deploy/hil/thor/mavros_rov1/`.** Non è un pacchetto (niente `package.xml`): è il launch
   MAVROS dell'HIL sul Thor con i suoi script d'installazione. Accanto ci sono le copie vecchie che il suo README
   dichiara superate (`deploy/hil/thor/m8a/thor_mavros_*`): non le ho cancellate.
8. **Gli script di `tools/` vanno nell'area a cui servono** (`physics/tools`, `sim/tools`, `ml/tools`,
   `vehicles/tools`, `deploy/hil`), perché `tools/` non c'è nella struttura proposta e così ogni area ha i suoi.
9. **Asset del Heavy**: `tools/fetch_bluerov2.py` li generava in `assets/bluerov2_heavy/` (ignorato, licenze di
   terzi); ora va in `vehicles/bluerov2_heavy/` e scrive in `vehicles/bluerov2_heavy/assets/` (ignorato).
   `assets/models/` (pesi YOLO, non in git) resta dov'è.
10. **`output/`** resta una cartella di lavoro locale ignorata (4 GB su linus). I dati veri stanno già fuori (§6).

## 3. Vecchio percorso → nuovo percorso

170 file su 242 si spostano. Le cartelle intere sono una riga sola (numero di file tra parentesi).

### 3.1 Codice Python di `uw/`

| Vecchio | Nuovo |
|---|---|
| `uw/bubbles.py`, `plume.py`, `scattering.py`, `sonar.py`, `sonar_stats.py`, `surface.py`, `water.py`, `geometry.py`, `pool_geometry.py` | `physics/` (stesso nome) |
| `uw/app.py`, `main.py`, `gui.py`, `usd_scene.py`, `pool.py`, `pool_scene.py`, `camera_uw.py`, `video.py`, `sonar_only.py`, `sinks.py`, `ros_bridge.py`, `recorder.py`, `record.py`, `pose.py`, `trajectory.py` | `sim/` (stesso nome) |
| `uw/rov/` (6: `__init__`, `dynamics`, `frames`, `dvl`, `a50`, `physics_proc`) | `vehicles/common/` |
| `uw/physics_udp.py` | `vehicles/common/physics_udp.py` |
| `uw/dataset.py` | `ml/dataset.py` |
| `uw/__init__.py`, `__main__.py`, `launcher.py`, `runner.py`, `deps.py`, `config.py` | restano in `uw/` |
| `uw/slam/` (2) | resta (§2.3) |

### 3.2 Configurazione

| Vecchio | Nuovo |
|---|---|
| `config/rov_bluerov2.yaml` | `vehicles/bluerov2_heavy/vehicle.yaml` |
| `config/rov_bluerov2_vectored.yaml` | `vehicles/bluerov2/vehicle.yaml` |
| `config/runtime.yaml` | `uw/runtime.yaml` |
| `config/runtime.local.yaml` (locale, ignorato) | `uw/runtime.local.yaml` (locale, ignorato) |
| `config/tank.yaml` | `sim/config/tank.yaml` |
| `config/presets/` (8) | `sim/config/presets/` |
| `config/profiles/` (2) | `sim/config/profiles/` |
| `config/pool/` (5) | `sim/config/pool/` |
| `config/trajectories/` (3) | `sim/config/trajectories/` |
| `config/sweeps/` (2) | `ml/sweeps/` |
| `config/sysctl/60-ros2-hil.conf` | `deploy/hil/sysctl/60-ros2-hil.conf` |
| `config/dds/*.xml` (generati, ignorati) | `deploy/hil/dds/*.xml` (generati, ignorati) |

### 3.3 Docker, HIL, ROS, script

| Vecchio | Nuovo |
|---|---|
| `docker/compose.yaml` | `deploy/docker/compose.isaac.yaml` |
| `docker/compose.stack.yaml` | `deploy/docker/compose.stack.yaml` |
| `docker/stack/Dockerfile` | `deploy/docker/Dockerfile.stack` |
| `docker/ardusub_sitl/` (7) | `deploy/docker/ardusub_sitl/` |
| `docker/pool/` (2) | `deploy/docker/pool/` |
| `docker/pool/.env` (locale, ignorato) | `deploy/docker/pool/.env` |
| `hil/win/` (11, con `pool/`) | `deploy/hil/windows/` |
| `hil/thor/m7/` (13), `hil/thor/m8a/` (8) | `deploy/hil/thor/m7/`, `deploy/hil/thor/m8a/` |
| `ros/dvl_dr/` (10), `ros/dvl_relay/` (7), `ros/link_monitor/` (7), `ros/pose_cmd/` (7), `ros/sonar_relay/` (7) | `ros/src/<stesso nome>/` |
| `ros/mavros_rov1/` (4) | `deploy/hil/thor/mavros_rov1/` |
| `scripts/uw.sh` | `uw.sh` |
| `setup_native.py` | `deploy/hil/windows/setup_native.py` |
| `uw.bat` | resta in radice |
| `tools/plume_preview.py`, `sonar_preview.py`, `compare_real.py`, `real_holdout.py`, `deepersense_index.py` | `physics/tools/` |
| `tools/sweep.py`, `recompress.py`, `dataset_stats.py`, `sonar_to_yolo.py`, `run_sonar_to_yolo.sh`, `verify_coco.py`, `verify_yolo_ultralytics.py` | `ml/tools/` |
| `tools/record.py`, `record_report.py`, `record_video.py`, `debug_sim.py`, `gpu_mem.py`, `pool_smoke.py`, `pool_blueos_setup.py`, `stub_physics.py` | `sim/tools/` |
| `tools/rov_sanity.py`, `dvl_drift.py`, `dvl_dr_offline.py` | `vehicles/tools/` |
| `tools/fetch_bluerov2.py` | `vehicles/bluerov2_heavy/fetch_bluerov2.py` |
| `tools/dds_profile.py` | `deploy/hil/dds_profile.py` |
| `assets/bluerov2_heavy/` (generato, ignorato) | `vehicles/bluerov2_heavy/assets/` (generato, ignorato) |

### 3.4 Restano dove sono

`README.md`, `.gitignore`, `.gitattributes`, `pytest.ini`, `uw.bat`, `docs/` (tutto, compresi `records/` e
`slides_assets/`), `tests/` (tutto), `uw/` (CLI e config), `uw/slam/`, `.sjck/`; ignorati: `output/`,
`third_party/`, `assets/models/`.

## 4. Cosa cambia nel codice (commit B)

Nessun cambiamento di comportamento: stessi comandi, stesse opzioni, stessi file letti, solo da un altro posto.

- **Import.** Dentro un pacchetto gli import relativi restano relativi (`from . import plume` in `physics/`).
  Tra pacchetti diventano assoluti: `from .config import get` → `from uw.config import get`,
  `from .rov import frames` → `from vehicles.common import frames`, `from ..pool_geometry import PoolBottom` →
  `from physics.pool_geometry import PoolBottom`. Fuori (`tools`, `tests`, `ros`): `from uw import config, geometry`
  → `from uw import config` + `from physics import geometry`, `from uw.rov import …` → `from vehicles.common import …`.
  Le stringhe con nomi di moduli (`mock.patch("uw.sonar…")`, docstring) seguono lo stesso schema.
- **Entry point.** `python -m uw py|test|sim|sonar-only|pool` invariato; `uw/runner.py` importa `sim.main`,
  `sim.sonar_only`, `sim.pool`. `uw.sh` e `uw.bat` in radice.
- **Percorsi relativi al file** (`Path(__file__).parents[n]`): `n` aumenta di uno per gli script scesi di un
  livello (`physics/tools/*.py`, `ros/src/pose_cmd/pose_cmd/cmd.py`: `parents[3]` → `parents[4]`, …).
- **Percorsi verso la configurazione**: `uw/config.py` (`sim/config/…`), `uw/launcher.py` (`uw/runtime*.yaml`,
  compose in `deploy/docker/compose.isaac.yaml`), `vehicles/common/dynamics.py`
  (`vehicles/bluerov2_heavy/vehicle.yaml`), `sim/trajectory.py`, `sim/config/pool/pool.yaml` (asset),
  `deploy/hil/windows/setup_native.py`, `ml/tools/sweep.py`.
- **Test.** `tests/test_module_boundaries.py` guarda `physics/`; i test che caricano `tools/` o `ros/<pkg>` dal
  `sys.path` puntano ai nuovi percorsi.
- **Launch file.** `ros/src/dvl_dr/launch/dvl_dr.launch.py` usa la propria cartella: invariato.
- **Script Windows** (la radice del repo è calcolata risalendo dalla cartella dello script, ora un livello più giù):
  `deploy/hil/windows/run_room9.bat`, `record_all.bat` (`%~dp0..\..` → `%~dp0..\..\..`),
  `deploy/hil/windows/pool/run_pool.bat` (`..\..\..` → `..\..\..\..`, e il percorso `bash …/stack.sh`),
  `deploy/docker/ardusub_sitl/run.bat` (`%HERE%..\..` → `%HERE%..\..\..`), `uw.bat` (`setup_native.py`).
- **Script Linux**: `uw.sh` (ora in radice), `deploy/hil/windows/pool/stack.sh` (`../../..` → `../../../..`,
  compose della piscina), `ml/tools/run_sonar_to_yolo.sh`.
- **Compose**: `compose.stack.yaml` costruisce `Dockerfile.stack` dalla sua cartella; `deploy/docker/pool/compose.yaml`
  usa ancora `../ardusub_sitl` (stessa posizione relativa).
- **Documentazione**: ogni percorso e nome di modulo citato in `docs/`, README e commenti è aggiornato con la stessa
  tabella.

## 5. Impatto su Work e Thor

Dopo `git pull --ff-only` il codice è a posto; i file **fuori da git** no. Per ognuno: cosa fare.

### Work (Windows 10, Isaac nativo)

- [ ] `config\runtime.local.yaml` → spostalo in `uw\runtime.local.yaml` (oppure rigenera:
      `python deploy\hil\windows\setup_native.py --write`). Senza, `python -m uw` proverebbe Docker; il launcher
      avvisa se trova ancora il file vecchio.
- [ ] `docker\pool\.env` (se l'hai creato per la piscina) → `deploy\docker\pool\.env`.
- [ ] `config\dds\*.xml` (profili DDS generati) → `deploy\hil\dds\`, oppure rigenera con
      `python deploy\hil\dds_profile.py … --out deploy\hil\dds\fastdds_hil.xml`.
- [ ] `assets\bluerov2_heavy\` → `vehicles\bluerov2_heavy\assets\` (oppure rilancia
      `python vehicles\bluerov2_heavy\fetch_bluerov2.py`).
- [ ] `output\hil\*.bat` (copie locali di `run_room9.bat`, `hil_env.bat`, `record_all.bat`): da buttare e
      rilanciare dal repo: `deploy\hil\windows\run_room9.bat`, `deploy\hil\thor\m8a\hil_env.bat`.
- [ ] Collegamenti o comandi salvati che puntano a `hil\win\…`, `tools\…`, `docker\ardusub_sitl\run.bat`,
      `setup_native.py`: aggiornali con la tabella §3.3.
- [ ] WSL (piscina tutto-su-Work): `bash deploy/hil/windows/pool/stack.sh …`.
- [ ] `F:\envs\isaac` resta il default in `uw.bat` e negli script: nulla da fare se non cambia.
- [ ] Verifica: `uw.bat` → `python -m uw test` (stessi risultati di prima).

### Thor (Jazzy, arm64)

- [ ] `~/hil_env.sh`, `~/thor_mavros_*.sh|launch.py` (copie fuori dal repo): sostituiscile con
      `deploy/hil/thor/mavros_rov1/` (`run.sh`, `mavros.launch.py`) e `deploy/hil/thor/m8a/thor_hil_env.sh`.
- [ ] Profili DDS generati in `config/dds/` → `deploy/hil/dds/` (o rigenera con `deploy/hil/dds_profile.py`).
- [ ] sysctl: il file installato in `/etc/sysctl.d/` non cambia; la sorgente ora è `deploy/hil/sysctl/`.
- [ ] Nodi lanciati con `ros/<pkg>/run.sh` → `ros/src/<pkg>/run.sh`.
- [ ] Immagine dello stack: `docker compose -f deploy/docker/compose.stack.yaml build` (o `uw image thor`).

## 6. Dove stanno i dati (fuori da git)

| Dati | Dove | Nel repo |
|---|---|---|
| dataset sintetici, YOLO, run, dati reali di terzi | linus: `/HDD2T/underwater_data/` (`data_root` di `uw/runtime.yaml`, già così) | manifest e report generati dai tool |
| log del ROV: `.BIN` di ArduSub e rosbag MCAP | macchina dei log (`logs_dest` in `deploy/targets/rov.local`; su linus `/HDD2T/underwater_data/rov_logs/`) | `robot/bluerov2/logs/INDEX.md`, aggiornato da `uw logs pull rov` |
| uscite locali, anteprime | `output/` (ignorato) | — |
| asset 3D generati | `vehicles/bluerov2_heavy/assets/` (ignorato) | lo script che li genera |
| modelli addestrati | `/HDD2T/underwater_data/yolo/…`, `assets/models/` (ignorato) | — |

**Da chiarire:** "HDD di `home`" per i log del ROV: su linus `/home` sta sul disco di sistema (SSD, 89 % pieno),
non su un HDD, e nella configurazione SSH non c'è un host `home`. La destinazione è quindi un parametro
(`logs_dest`) con default su `/HDD2T`.

## 7. Debiti noti, non toccati

La divisione rende visibili dipendenze che vanno "all'indietro"; sono tutte import ritardati dentro funzioni (come
prima), quindi nessun ciclo all'import, ma andranno ripulite in un lavoro separato:

- `physics/geometry.py` → `sim.pose` (in `build_scene`, per la posa del sensore);
- `sim/pose.py` → `vehicles.common.physics_udp` (sorgente di posa `ardusub`);
- `vehicles/common/physics_udp.py` → `sim.pose`, `physics.geometry`;
- `vehicles/common/dvl.py`, `a50.py` → `physics.geometry`, `physics.pool_geometry`, `physics.water`.

## 8. Test di riferimento

Prima di spostare qualsiasi cosa (`e245944`, worktree pulito su linus, RTX 3080):

- `pytest` sull'host: tutto verde tranne `test_physics_udp::test_neutral_pwm_hovers_and_reports_rest`, instabile
  sotto carico (399/400 pacchetti; da solo passa).
- `python -m uw test`: **21 falliti + 6 errori, tutti in `tests/gpu/`**, riproducibili: `test_golden_ping` (0,14 dB
  fuori tolleranza), poi `illegal memory access` di CUDA in `sonar.py` `ping()` che avvelena il contesto per tutti i
  test GPU successivi. È un problema preesistente, da indagare a parte.

Criterio usato in ogni fase: **stessi risultati della baseline, test per test** (nessun test nuovo rotto, nessuno
sparito), oltre a `colcon build` verde.

## 9. Aggiunto dopo lo spostamento (Fase 3)

| Percorso | Cosa |
|---|---|
| `vehicles/common/schema.py`, `vehicles/README.md` | schema di `vehicle.yaml` (unità, `null` ammessi); `body.hull_size_m` nuovo (prima costante in `sim/pool_scene.py`) |
| `vehicles/momorov/README.md` | segnaposto: `sjckness/momoROV` è vuoto |
| `tests/rov/test_vehicle_yaml.py`, `tests/rov/golden_vehicle_dynamics.json` | dinamica dai file spostati = dinamica a `e245944` |
| `robot/bluerov2/` | accesso, regole, `VERSIONS.md`, parametri, BlueOS, checklist, `logs/INDEX.md` |
| `ros/src/rov_bringup/` | MAVROS in sola lettura + profilo di registrazione MCAP `light` |
| `deploy/docker/Dockerfile.rov`, `compose.rov-sim.yaml` | immagine dell'estensione con i nostri pacchetti; ambiente di prova su linus |
| `deploy/targets/*.example` | `rov`, `rov-sim`, `thor`, `work`, `linus` |
| `uw/image.py`, `deploy.py`, `smoke.py`, `logs.py`, `remote.py`, `targets.py`, `sysid/` | i comandi `uw image|deploy|smoke|logs|sysid` |
| `docs/ARCHITECTURE.md` | struttura, macchine, ciclo, dati, versioni ROS, ambiente di prova |
