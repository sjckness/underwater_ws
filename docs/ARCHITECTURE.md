# Architettura

Come stanno insieme le parti del progetto underwater, dove gira cosa, dove vivono i dati.
La mappa file per file della ristrutturazione è in [MIGRATION.md](MIGRATION.md).

## 1. Il ciclo

```
 BlueROV2 reale ──(uw logs pull rov)──► log .BIN + rosbag MCAP ──(uw sysid)──► vehicles/<veicolo>/vehicle.yaml
       ▲                                  (fuori da git, indice in git)            │
       │                                                                           ▼
 (uw deploy rov) ◄── immagini / ros/src ◄── HIL (work + Thor) ◄── simulazione (linus / work: Isaac, SITL)
```

1. **Veicolo reale** (`robot/bluerov2/`): BlueOS + ArduSub sulla Pi; il nostro ROS 2 gira solo nell'estensione
   ROS 2 di BlueOS (`ros/src/rov_bringup`: MAVROS in sola lettura, registrazione MCAP leggera).
2. **Log** (`uw logs pull rov`): `.BIN` di ArduSub e bag, copiati dal veicolo (solo pull) in una cartella fuori da
   git; in git c'è solo l'indice `robot/bluerov2/logs/INDEX.md`.
3. **Identificazione** (`sysid/`, `uw sysid bluerov2_heavy`): dai log ai parametri di Fossen. Oggi è uno scheletro
   (inventario, prontezza dei dati, report); scrive in `vehicle.yaml` solo con `--write`.
4. **Parametri del veicolo** (`vehicles/`): `vehicle.yaml` con unità SI esplicite, provenienza di ogni valore,
   `null` + `# TODO` dove non si sa; lo schema è `vehicles/common/schema.py`.
5. **Simulazione e HIL** (`physics/`, `sim/`, `vehicles/common/`): la dinamica legge `vehicle.yaml`; ArduSub SITL
   chiude l'anello sul backend JSON; l'HIL porta i sensori simulati sul Thor via DDS.
6. **Software ROS** (`ros/src/`): un solo workspace colcon Jazzy per Thor, linus e Pi.
7. **Deploy** (`deploy/`, `uw image`, `uw deploy`): immagini con tag = SHA del commit, promosse a mano
   `:<sha>` → `:hil-ok` → `:rov`; sul veicolo `uw deploy rov` (solo a veicolo disarmato).

## 2. Struttura del repo

| Cartella | Contenuto | Dipende da |
|---|---|---|
| `physics/` | bolle, plume, scattering, sonar M750d, acqua, geometria | numpy, Warp; `uw.config` |
| `vehicles/` | `common/` (Fossen, frame, DVL, fisica JSON per SITL, schema), un veicolo per cartella | numpy, `physics` |
| `sim/` | app Isaac, scene, pose/traiettorie, sink (file, ROS), registrazione; `sim/config/` | Isaac, `physics`, `vehicles` |
| `ros/src/` | pacchetti ROS 2 Jazzy (`ament_python`) | ROS 2; `pose_cmd` importa `uw`/`physics`/`sim` dal checkout |
| `robot/bluerov2/` | il veicolo reale: accesso, regole, parametri, BlueOS, versioni, checklist, indice dei log | — |
| `sysid/` | log → parametri | `vehicles.common.schema` |
| `ml/` | dataset, sweep, conversione per YOLO/COCO | `physics`, `sim` |
| `deploy/` | `docker/` (Isaac, stack ROS, ROV, SITL, piscina, rov-sim), `hil/`, `targets/` | — |
| `uw/` | CLI `python -m uw`, `config.py`, `runtime.yaml` | — |
| `tests/` | tutti i test; `gpu/` e `isaac/` per requisito di runtime | — |

Debiti noti (import ritardati che vanno "all'indietro", es. `physics/geometry.py` → `sim.pose`):
[MIGRATION.md](MIGRATION.md) §7.

## 3. Le macchine

| Macchina | Ruolo | Cosa ci gira | Rete |
|---|---|---|---|
| **linus** (Ubuntu, RTX) | **sviluppo**: l'unica dove si scrive codice e si committa | Isaac in Docker (`deploy/docker/compose.isaac.yaml`), ROS 2 Jazzy sull'host, piscina M11 (SITL + BlueOS + DVL), rov-sim | LAN; `192.168.2.1` su un adattatore USB-Ethernet (convenzione del topside BlueROV) |
| **work** (Windows 10) | **test e HIL**: Isaac Sim 5.1 nativo, ROS in Docker/WSL | `uw.bat`, `deploy/hil/windows/` | `192.168.200.2` sul cavo HIL, dominio 73 |
| **Thor** (Jetson AGX Thor, Jazzy, arm64) | **HIL**: lo stack ROS che riceverà i sensori veri | `deploy/docker/Dockerfile.stack` (arm64), `deploy/hil/thor/` | `192.168.200.10` |
| **Pi del ROV** (Raspberry Pi 4 + Navigator, arm64) | **il veicolo** | BlueOS, ArduSub; il nostro ROS solo nell'estensione `itskalvik/blueos-ros2:0.1.0` | `192.168.2.2` sul tether; topside `192.168.2.1` |

Regola: work, Thor e Pi **non committano mai**: `git pull --ff-only` (o `uw deploy`) e basta. Cosa serve su
una macchina arriva da `main`.

## 4. Versioni di ROS

- **Jazzy** per tutto il nostro codice: `ros/src`, lo stack sul Thor e su linus (`ros:jazzy-ros-base`), l'estensione
  sulla Pi (`itskalvik/blueos-ros2:0.1.0`, Ubuntu 24.04, MAVROS 2.10.1).
- **Humble** solo dentro Isaac Sim 5.1 su Windows (rclpy interno al bridge di Isaac, l'unica distribuzione che Isaac
  5.1 porta su Windows). Il bridge Humble (work) parla con i nodi Jazzy (Thor) via DDS sul dominio 73: verificato con
  i messaggi standard (sensor_msgs, std_msgs, rosgraph_msgs) in M7 ([hil_work_thor_setup.md](hil_work_thor_setup.md),
  `deploy/hil/thor/m7/results/`). Su linus Isaac usa il proprio rclpy Jazzy nel container.

## 5. Dove vivono i dati

Mai in git. In git solo indici, manifest e report piccoli.

| Dati | Dove |
|---|---|
| dataset sintetici, run, YOLO, dati reali di terzi | linus `/HDD2T/underwater_data/` (`data_root` di `uw/runtime.yaml`) |
| log del ROV (`.BIN`, bag) | `logs.dest` di `deploy/targets/rov.local` (default `/HDD2T/underwater_data/rov_logs/`); indice `robot/bluerov2/logs/INDEX.md` |
| uscite locali, anteprime | `output/` (ignorato) |
| asset 3D generati, pesi dei modelli | `vehicles/*/assets/`, `assets/models/` (ignorati) |
| build colcon | `output/colcon/` (`uw test`), `ros/build|install|log` (ignorati) |

## 6. Sicurezza del veicolo reale

Riassunto delle regole di `robot/bluerov2/README.md`, come le applica il codice:

- nessun comando del repo arma il veicolo o scrive parametri: MAVROS carica solo plugin di lettura
  (`ros/src/rov_bringup/config/mavros_pluginlists.yaml`, verificato da `tests/test_rov_bringup.py`);
- `uw deploy rov` e `uw smoke rov` leggono lo stato da mavlink2rest (`GET` dell'ultimo HEARTBEAT, bit 128 di
  `base_mode`) e si fermano se il veicolo è armato o se ArduSub non è vivo; il deploy ricontrolla prima del riavvio;
- si tocca solo il container dell'estensione ROS e `/usr/blueos/extensions/ros2/`; mai `blueos-core`, mai gli
  aggiornamenti di BlueOS;
- `uw logs pull` copia soltanto: `--ignore-existing`, niente cancellazioni sul veicolo.

## 7. Ambiente di prova: rov-sim su linus

Per sviluppare `uw deploy | smoke | logs pull` senza il ROV. Il "veicolo" è la piscina M11 già su linus (BlueOS core
a `172.31.250.2` + ArduSub SITL, `deploy/docker/pool/compose.yaml`, profilo `blueos-local`); la "Pi" è linus stesso
(`transport: local` in `deploy/targets/rov-sim.example`: i comandi girano qui invece che via SSH); l'estensione è lo
stesso container della Pi, con lo stesso nome.

```bash
# 1. il veicolo: la piscina (se non è già su) e una fisica che faccia girare SITL
#    - con Isaac: python -m uw pool   (docs/pool.md)
#    - senza GPU: python3 sim/tools/stub_physics.py --duration 1200   (un veicolo fermo a 1 m)
curl -s http://172.31.250.2/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT | head -c 200

# 2. l'estensione ROS 2 (cartella persistente in ~/.cache/uw_rov_sim; creala prima, se no Docker la crea di root)
mkdir -p ~/.cache/uw_rov_sim/extensions/ros2 ~/.cache/uw_rov_sim/ardupilot_logs
docker compose -f deploy/docker/compose.rov-sim.yaml up -d

# 3. qualche .BIN di SITL da "scaricare"
docker cp uw-pool-sitl:/home/ardu/sitl/logs/00000010.BIN ~/.cache/uw_rov_sim/ardupilot_logs/

# 4. le prove
python -m uw deploy rov-sim --discover --yes --start      # disarmato? rsync, colcon build, restart, MAVROS su
python -m uw smoke rov-sim                                # heartbeat, disarmato, frequenze, CPU
docker exec extension-itskalvikblueosros2010 bash -c 'source /opt/ros/jazzy/setup.bash &&
  source /root/persistent_ws/ros2_ws/install/setup.bash && ROS_DOMAIN_ID=73 timeout -s INT 15 ros2 run rov_bringup record'
python -m uw logs pull rov-sim                            # -> ~/.cache/uw_rov_sim/pulled, indice lì (non in robot/)
python -m uw sysid bluerov2_heavy --logs ~/.cache/uw_rov_sim/pulled

# 5. fine
docker compose -f deploy/docker/compose.rov-sim.yaml down
```

Differenze dal veicolo vero, da tenere presenti:

- niente SSH (`transport: local`): il trasporto SSH si prova con un `rov-sim.local` che metta `transport: ssh`,
  `host: localhost` dopo `ssh-copy-id localhost`;
- MAVROS legge la seconda uscita MAVLink di SITL (`udp://:14560@`), non il link interno di BlueOS
  (`tcp://127.0.0.1:5777`), perché qui BlueOS sta su un bridge e quel link è sul suo loopback. La porta 14560 è anche
  quella di `sim/tools/pool_smoke.py`: non usarli insieme;
- CPU e temperatura sono quelle di linus, non della Pi;
- con la fisica stub l'EKF non ha origine: `local_position/pose` e `altitude` restano vuoti.
