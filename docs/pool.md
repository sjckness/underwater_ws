# M11 — La vasca prove: BlueROV2 Heavy pilotato con BlueOS, Cockpit e gamepad

Una vasca 10 × 10 × 5 m in Isaac Sim con un BlueROV2 Heavy che si pilota **come quello vero**:
- gamepad Bluetooth sul PC;
- Cockpit e BlueOS aperti nel browser;
- ArduSub che comanda gli otto motori.

Isaac fa solo la fisica del veicolo e la camera di bordo, più un DVL Water Linked A50 emulato che alimenta ArduSub attraverso la vera estensione DVL di BlueOS (POSHOLD come sul ROV vero). Niente bolle, niente sonar.

Questo file è la guida operativa. Il resto è nel codice:
- l'app: `uw/pool.py`, `uw/pool_scene.py`, `uw/camera_uw.py`, `uw/video.py`;
- la fisica: `uw/rov/physics_proc.py`;
- il DVL: `uw/rov/a50.py` (il dispositivo in rete), `uw/rov/dvl.py` (fasci, rumore, aggancio al fondo), `uw/pool_geometry.py` (cosa colpiscono i fasci);
- lo stack: `docker/pool/compose.yaml`;
- la configurazione: `config/pool/`.

## Come è fatto

```
 PC operatore ─ browser: BlueOS + Cockpit  ◄── gamepad Bluetooth (sulla Work: http://localhost)
        ▲ video WebRTC            │ MAVLink via mavlink2rest (MANUAL_CONTROL a 25 Hz)
 BlueOS 1.5 beta, board "Manual" ── in WSL2 sulla Work (rete mirrored)
        │ pull RTSP (sorgente "Redirect")   ▲ MAVLink UDP 14551
 ── macchina della simulazione (linus in docker, oppure la Work nativa) ─────────────────────────────
  mediamtx :8654 ◄── H.264 ── python -m uw pool (Isaac headless: vasca, ROV, camera, acqua, encoder)
                                     ▲ shared memory: posa, tilt camera, luci
                  uw.rov.physics_proc (Fossen, 400 Hz) ◄── UDP 9002 lockstep ──► ArduSub SITL 4.5.7 (Heavy)
                    └─ DVL A50 emulato: TCP 16171 (JSON), HTTP 8061 ──► estensione Water Linked DVL di BlueOS
                                                                          └─► VISION_POSITION_DELTA ad ArduSub
```

- **La fisica gira in un processo separato** e segue l'orologio di parete. SITL, BlueOS e il pilota vedono sempre il tempo reale: se il render rallenta perde fotogrammi, non tempo di simulazione.
- **SITL resta accanto alla fisica**: il link JSON è un andata-e-ritorno lockstep a 400 Hz, quindi su loopback. Tra le macchine passano solo MAVLink (UDP, leggero) e il video (RTSP su TCP). Vanno bene LAN o Tailscale.
- **BlueOS su linus.** In rete host BlueOS vuole le porte 80 (UI) e 8000 (ardupilot-manager), entrambe occupate su linus (80 da OnlyOffice).
  - Sulla Work va in WSL2 con la rete host: profilo `work` (tutto sulla Work, con l'estensione DVL) o `blueos` (solo BlueOS, per lo scenario A).
  - Su linus gira in un container non privilegiato su una sua rete bridge, all'indirizzo fisso 172.31.250.2 (profilo `blueos-local`, scenario C).
- **Perché la board "Manual".** Esiste da BlueOS 1.5.0-beta.3: BlueOS non lancia un autopilota suo e prende il MAVLink da SITL. La board "SITL" delle 1.4 stabili lancerebbe ArduSub dentro BlueOS, lontano dalla fisica.

## Preparazione (una volta)

**Sulla macchina della simulazione:**
1. **Modello 3D del ROV.** Lancia `python -m uw py tools/fetch_bluerov2.py`.
   - Scarica la Heavy di Blue Robotics da BlueSim (ricavata dal loro CAD, già decimata, 126 k triangoli) e la converte in `assets/bluerov2_heavy/bluerov2_heavy.usd`.
   - Il file non va in git, per licenza (GPL-2.0) e dimensione.
   - Senza questo file la vasca mostra un parallelepipedo delle stesse misure.
2. **Immagine di ArduSub SITL.** Lancia `docker compose -f docker/pool/compose.yaml build sitl` (circa 3 minuti).

**Sulla Work:** la guida passo per passo è [hil/win/pool/README.md](../hil/win/pool/README.md) (installazione, avvio, verifica, problemi noti). In breve:
1. Copia `hil/win/pool/wslconfig.example` in `%UserProfile%\.wslconfig`, poi esegui `wsl --shutdown`.
2. In una distro Ubuntu WSL dedicata, dal repo: `bash hil/win/pool/setup_blueos_wsl.sh`.
   - Abilita systemd e installa Docker Engine, non Docker Desktop: BlueOS vuole la vera rete host.
   - La prima volta chiede di riavviare la distro: rilancialo dopo.
3. Solo se SITL o il browser stanno su un'altra macchina: `hil\win\pool\firewall_pool.bat`, da amministratore.

## Avvio

### Scenario A: simulazione su linus, BlueOS sulla Work

**Su linus:**
```bash
BLUEOS_HOST=<IPv4 della Work, LAN o Tailscale> docker compose -f docker/pool/compose.yaml up -d sitl mediamtx
python -m uw pool                 # headless; --gui per vedere il viewport, --view chase per la vista da dietro
sudo ufw allow in on tailscale0 to any port 8654 proto tcp    # una volta: la Work deve leggere lo stream
```

**Sulla Work, in WSL:**
```bash
docker compose -f docker/pool/compose.yaml --profile blueos up -d blueos
python3 tools/pool_blueos_setup.py --blueos http://localhost --rtsp rtsp://<IP di linus>:8654/rov
```

### Scenario B: tutto sulla Work

Isaac e la fisica nativi su Windows; SITL, mediamtx, BlueOS e l'estensione DVL in WSL2 (profilo `work`). Guida completa: [hil/win/pool/README.md](../hil/win/pool/README.md).

```bat
hil\win\pool\run_pool.bat --gui
```

Fa tutto lui:
- porta su lo stack in WSL (`hil/win/pool/stack.sh up`);
- riavvia SITL;
- configura BlueOS e il DVL in background, appena il veicolo c'è;
- lancia Isaac con il profilo `workstation` (30 fps, riflessi accesi).

Con la rete mirrored di WSL, il `127.0.0.1` di WSL è quello di Windows:
- SITL raggiunge la fisica su 9002;
- l'estensione DVL raggiunge l'A50 emulato su 16171 e 8061, che sulla Work ascolta solo su loopback;
- Isaac pubblica su mediamtx su 8654.

**Da verificare** al primo avvio (non provato). Le alternative, senza toccare file in git, sono nella guida, §5:
- `docker/pool/.env` con `PHYSICS_HOST` e `DVL_HOST`;
- `--set physics.bind=0.0.0.0` e `--set dvl.device.bind=0.0.0.0`.

### Scenario C: tutto su linus (Isaac con la GUI, BlueOS locale, Cockpit desktop)

```bash
BLUEOS_HOST=172.31.250.2 docker compose -f docker/pool/compose.yaml --profile blueos-local up -d
python3 -m uw pool --gui          # viewport alle spalle del ROV; --gui-view onboard|free per cambiarlo
python3 tools/pool_blueos_setup.py --blueos http://127.0.0.1:28080 --rtsp rtsp://172.31.250.1:8654/rov \
    --dvl http://172.31.250.3     # in un altro terminale, con uw pool già partito (serve il DVL)
~/.local/opt/cockpit-1.19.0/Cockpit-linux-x86_64-1.19.0.AppImage --no-sandbox
```

- **BlueOS** è su `http://localhost:28080` (oppure `http://172.31.250.2`).
- **La pagina dell'estensione DVL** è su `http://172.31.250.3`: stato, assetto, traccia sulla mappa. Il profilo avvia anche l'estensione (`dvl`) e il suo gateway (`dvl-gateway`): vedi "Il DVL" sotto.
- **Cockpit desktop** (v1.19.0, AppImage da github.com/bluerobotics/cockpit/releases) si collega con *Settings → General → Global vehicle address* = `172.31.250.2`.
  - `--no-sandbox` serve su Ubuntu 24.04, che vieta i namespace utente senza privilegi (`kernel.apparmor_restrict_unprivileged_userns = 1`): senza, Electron non parte.
- **La camera di bordo con l'acqua** è in Cockpit, oppure nel browser su `http://localhost:8889/rov` (il viewer WebRTC di mediamtx). Il viewport di Isaac mostra il render grezzo, senza il post-processing dell'acqua.
- **Gamepad:** un controller USB o Bluetooth collegato a linus, letto da Cockpit.
- **Stop:** `Ctrl-C` su `uw pool`, poi `docker compose -f docker/pool/compose.yaml --profile blueos-local down`. BlueOS tiene la configurazione nel volume `uw-pool_blueos_local_config`, l'estensione DVL in `uw-pool_dvl_settings`.
- **Riavvia SITL insieme alla fisica.** Un nuovo `uw pool` rimette il ROV al punto di partenza, ma l'EKF di ArduSub resta dov'era. La prua stimata ci mette decine di secondi a raggiungere quella vera, e intanto POSHOLD e la navigazione stimata lavorano su una prua sbagliata (misurato: 20–25° di errore, 45 cm su 1 m di traccia). Quindi, prima di ogni `uw pool`: `docker compose -f docker/pool/compose.yaml restart sitl`. `pool_smoke.py` lo controlla.

### Pilotare

- **Browser.** Apri `http://localhost` sulla Work (o `http://<IP della Work>` da un altro PC) e poi Cockpit.
  - Cockpit si installa da BlueOS: *Extensions → Cockpit → Install*.
  - In alternativa c'è l'app desktop di Cockpit, che legge il joystick con SDL.
- **Gamepad.** Va accoppiato in Bluetooth al PC dove gira il browser.
  - Su `http://localhost` la Gamepad API funziona senza https.
- **Focus.** In Cockpit attiva l'opzione che continua a mandare l'input quando la finestra perde il focus. Altrimenti dopo 3 s scatta `FS_PILOT_INPUT` e il ROV si disarma.
- **Mappa di partenza** (quella di `sub.parm`, che Cockpit riprende):
  - stick per avanti, laterale, yaw e salita/discesa;
  - tasti per arm/disarm, MANUAL / STABILIZE / DEPTH HOLD;
  - tilt della camera (SERVO9) e luci Lumen (SERVO10).

  La simulazione legge tilt e luci dal PWM, come sul ROV vero.
- **Modi.** MANUAL, STABILIZE e ALT_HOLD (depth hold) funzionano sempre. POSHOLD funziona con il DVL, come sul ROV vero: nello scenario C (provato) e nel B (profilo `work`, non ancora provato sulla Work). Senza DVL l'EKF non ha posizione orizzontale e POSHOLD non si attiva.
- **Acqua torbida.** `python -m uw pool --preset turbid` (visibilità circa 5,8 m) oppure `--preset murky` (circa 2,5 m); per un valore qualsiasi `--set water.turbidity_per_m=X`. Vedi "Torbidità" sotto.
- **Vista da fuori.** `python -m uw pool --view chase` manda nello stream una camera alle spalle del ROV invece di quella di bordo.
  - Con `--gui` il viewport di Isaac è alle spalle del ROV per default (`--gui-view chase`); lo stream resta quello di bordo.

### Controllo senza BlueOS

`python -m uw py tools/pool_smoke.py` pilota il veicolo via MAVLink sulla seconda uscita di SITL (UDP 14560), come farebbe Cockpit, e verifica:
- i parametri;
- l'armamento;
- la discesa;
- ALT_HOLD;
- avanti, laterale e yaw;
- la profondità del baro rispetto a quella simulata;
- la risalita;
- luci e tilt;
- il disarmo.

Con `--dvl` (estensione DVL attiva e configurata) verifica anche: l'EKF ha la velocità orizzontale, POSHOLD si attiva e tiene il posto, si sposta con lo stick e tiene il nuovo posto, e la posizione stimata da ArduSub segue quella vera. Prima di tutto controlla che la prua stimata da ArduSub coincida con quella vera (entro 3° per 5 s).

Per provare SITL senza GPU c'è la fisica da sola: `python -m uw py -m uw.rov.physics_proc` (il ROV è invisibile ma vola, e il DVL emulato c'è anche qui).

## Torbidità

Un solo parametro, `water.turbidity_per_m` (b_p, in 1/m): il coefficiente di diffusione delle particelle sospese, uguale nei tre canali, perché particelle molto più grandi della lunghezza d'onda diffondono tutti i colori allo stesso modo (`uw/camera_uw.py`).
- Si somma all'attenuazione del segnale diretto e a quella della luce diffusa.
- Sposta la luce diffusa di fondo dal blu dell'acqua limpida verso `water.turbidity_veil`, il colore della nuvola di particelle illuminata (grigio-verde, a occhio).

| | b_p [1/m] | un oggetto scuro si vede fino a |
|---|---|---|
| limpida (default) | 0 | ~52 m (la piscina è tutta visibile) |
| `--preset turbid` | 0,6 | ~5,8 m |
| `--preset murky` | 1,5 | ~2,5 m |

La visibilità è ln(50)/β_d nel verde, il canale che arriva più lontano: dove il contrasto di un oggetto scuro scende al 2 %.

Con acqua torbida, dalla superficie guardando dritto non si vede niente: il fondo è a quasi 5 m. Scendi o inclina la camera verso il basso.

Cosa non c'è: le lampade del ROV illuminano le particelle subito davanti alla camera, ed è l'effetto più tipico dell'acqua torbida (i "coni" e la neve). Serve il percorso della luce, che un post-processing non ha.

## Il DVL

Un Water Linked DVL A50 sotto la Heavy, a 0,14 m sotto il baricentro, rivolto in basso. Sono tre pezzi:

1. **Il sensore** (`uw/rov/dvl.py`, `docs/dvl.md`): lo stesso modello della vasca del gas.
   - Quattro fasci a 22,5°, aggancio al fondo con il rapporto segnale/rumore, rumore, bias lento, fattore di scala.
   - I fasci colpiscono la geometria della piscina (`uw/pool_geometry.py`): fondo, pareti, scaletta, oggetti. Sono la stessa lista di solidi della scena USD, così quello che vede la camera e quello che misura il DVL non possono divergere.
   - Le pareti fermano i fasci. Vicino a una parete un fascio la colpisce di striscio, l'eco è debole e il DVL può perdere l'aggancio, come quello vero. Lo stesso succede sul tubo.
2. **Il dispositivo in rete** (`uw/rov/a50.py`), dentro il processo della fisica.
   - Protocollo JSON di Water Linked `json_v3.3` su TCP 16171: report di velocità, dead reckoning `position_local` a 5 Hz, comandi (`get_config`, `set_config`, `reset_dead_reckoning`, `trigger_ping`, …).
   - L'API HTTP che l'estensione interroga, sulla porta 8061 perché la 80 su linus è occupata.
   - Timestamp in tempo Unix, come il dispositivo vero.
3. **L'estensione DVL di BlueOS**, la stessa del ROV vero (`bluerobotics/blueos-water-linked-dvl:v1.0.11`), senza modifiche.
   - Legge il DVL e manda ad ArduSub `VISION_POSITION_DELTA`, l'altitudine come rangefinder e l'origine EKF.
   - Sul ROV vero raggiunge BlueOS come `host.docker.internal`. Qui al suo posto c'è `dvl-gateway` (un nginx, `docker/pool/dvl_gateway.conf.template`), per due motivi: il BlueOS locale non ha il socket docker per le estensioni, e cable_guy è spento (gestirebbe la rete dell'host), mentre l'estensione lo aspetta all'avvio.
   - Il gateway inoltra mavlink2rest a BlueOS, risponde al posto di cable_guy e serve la pagina dell'estensione su `http://172.31.250.3`.
   - Sulla Work (profilo `work`) l'estensione gira in rete host accanto a BlueOS (servizio `dvl-host`, pagina su `http://localhost:9001`). Il gateway (`dvl-gateway-host`) resta da solo sulla rete bridge e da lì raggiunge BlueOS su 172.31.250.1. Su linus questa variante è stata provata con BlueOS locale: `pool_smoke.py --dvl` 18/18.

**Lato ArduSub** (`docker/ardusub_sitl/params/pool.parm`): EKF3 con posizione e velocità orizzontali dal DVL (`EK3_SRC1_POSXY/VELXY 6`), `VISO_TYPE 1`, `RNGFND1_TYPE 10`. Sono i valori che imposta l'estensione stessa, messi già come default perché `VISO_TYPE` vale solo dopo un riavvio. `pool_blueos_setup.py --dvl` fa quello che farebbe un operatore sulla pagina dell'estensione:
- carica i parametri "DVL";
- sceglie `POSITION_DELTA`;
- attiva il rangefinder;
- fissa l'origine EKF alla home di SITL.

**Un crash di SITL evitato: `SIM_FLOAT_EXCEPT 0`.** Il "set current position" dell'estensione manda `MAV_CMD_EXTERNAL_POSITION_ESTIMATE` con accuratezza e quota NaN. Lo fa apposta: ArduPilot legge NaN come "sconosciuto". SITL però per default si ferma alla prima eccezione floating point, e quel comando da solo abortiva ArduSub 4.5.7 (provato il 2026-11-26). Il controllore di volo vero non intercetta le eccezioni, quindi ora SITL si comporta come lui.

**Misurato** (2026-11-26, linus, `pool_smoke.py --dvl`, SITL riavviato con la fisica):

| Cosa | Valore |
|---|---|
| POSHOLD fermo per 10 s | 1–2 cm dal punto |
| Dopo 1 m di stick | tiene il nuovo posto entro 2–3 cm per 8 s |
| Posizione stimata da ArduSub dopo 1,1 m | 6–7 cm dalla vera |
| Report di velocità | 2 Hz a galla, fino a 15 Hz vicino al fondo |

L'errore della posizione stimata viene da due scale che si sommano:
- la velocità del suono del DVL, che parte da 1475 m/s mentre l'acqua a 27 °C ne ha circa 1500 (-1,8 %);
- il fattore di scala estratto per la corsa (±1 %).

Sul ROV vero la velocità del suono si imposta dalla pagina web dell'A50; qui con `set_config`.

**Ancora incerti** (UNVERIFIED in `config/pool/pool.yaml`):
- come cala la frequenza dei report con l'altitudine. Water Linked pubblica solo l'intervallo 2–15 Hz, e la forma conta per la qualità di POSHOLD;
- il riferimento d'eco (`rssi_ref_dbm`);
- i materiali acustici della piscina;
- dove va montato il DVL sul ROV vero.

## Riconoscimento con YOLO

`ros/yolo_detector` fa girare un modello Ultralytics sulla camera di bordo, sull'host con ROS 2 Jazzy. Legge lo stesso stream H.264 che BlueOS passa a Cockpit (`rtsp://127.0.0.1:8654/rov`), quindi:
- la simulazione non cambia e non le serve ROS;
- il detector vede gli artefatti dell'encoder e il ritardo della rete, come sul ROV vero;
- sul ROV vero cambia solo `--source` (lo stream RTSP di BlueOS).

**Il Python.** Serve un Python 3.12 con Ultralytics e torch CUDA, separato da quello di sistema: `ros/yolo_detector/requirements.txt`.
- Su linus c'è già: `export UW_YOLO_PYTHON=/HDD2T/dev_ws/py_venv/bin/python`.
- Altrove: `python3 -m venv ~/.cache/uw_yolo/venv && ~/.cache/uw_yolo/venv/bin/pip install -r ros/yolo_detector/requirements.txt` (circa 5 GB, per torch). `run.sh` lo usa da solo.
- numpy resta sotto la 2: rclpy di Jazzy è compilato con numpy 1.26.

**Avvio**, con `uw pool` già partito:
```bash
ros/yolo_detector/run.sh                                                # COCO yolo26n, topic ROS sul dominio 73
ros/yolo_detector/run.sh --video-out rtsp://127.0.0.1:8654/rov_yolo     # + l'overlay nel browser: http://localhost:8889/rov_yolo
ros/yolo_detector/run.sh --track --publish-camera                       # id di tracking, e la camera come topic ROS
ros/yolo_detector/run.sh --no-ros --source video.mp4 --video-out out.mp4   # offline, senza ROS: ogni fotogramma
```
- **Topic** (dominio `UW_ROS_DOMAIN_ID`, default 73): `/rov1/perception/camera/detections` (`vision_msgs/Detection2DArray`: box in pixel, `class_id` = nome della classe, `id` = id di tracking con `--track`) e `/rov1/perception/camera/overlay` (l'immagine annotata, per `rqt_image_view`).
- **In Cockpit** (non provato): in BlueOS aggiungi `rtsp://172.31.250.1:8654/rov_yolo` come secondo stream "Redirect", come quello della camera.
- **Il modello:** `--model` accetta un percorso o un nome. Un nome viene cercato in `assets/models/` (fuori da git), dove Ultralytics lo scarica la prima volta. Vanno bene `.pt`, `.onnx` e `.engine`, nei task detect, segment e pose.
- **Le installazioni automatiche di Ultralytics sono spente:** un pacchetto che manca è un errore, non un `pip install` a metà corsa.
- Tutte le opzioni: `ros/yolo_detector/run.sh --help`.

**Cosa riconosce.** I modelli COCO (`yolo26n.pt`, il default) in piscina **non trovano niente**: non c'è nessun oggetto COCO (provato su tutti gli snapshot). Le strade sono due:
- **Prompt di testo, senza addestramento:** `--model yoloe-26s-seg.pt --prompts "pipe,concrete block,checkerboard,ladder"`. Serve il CLIP di Ultralytics: `$UW_YOLO_PYTHON -m pip install git+https://github.com/ultralytics/CLIP.git` (non ancora installato nel venv di linus, quindi non provato). Il modello YOLOE "prompt-free" (`-pf`) sulla piscina dà solo etichette a caso.
- **Un modello addestrato sulla piscina:** le etichette le dà la simulazione (i box 2-D di Replicator) e poi si fa un fine-tuning di YOLO. Non ancora fatto.

**Misurato** (2026-11-26, linus, RTX 3080 secondo `nvidia-smi`, accanto a `uw pool --preset turbid` in esecuzione):

| Cosa | Valore |
|---|---|
| Frame elaborati | 30 fps su 30 (stream della vasca) |
| Inferenza `yolo26n` 1280×720 → 640 | 13–15 ms |
| Latenza dalla decodifica alla pubblicazione | 22–24 ms (non comprende encoder e stream della vasca) |

## Porte

| Dove | Porta | Cosa |
|---|---|---|
| macchina sim | UDP 9002 (loopback) | fisica ← ArduSub SITL (JSON) |
| macchina sim | UDP 14560 (loopback) | SITL → strumenti locali (`pool_smoke.py`, un QGC) |
| macchina sim | TCP 8654 | RTSP di mediamtx (solo TCP); 8889 è il viewer WebRTC di mediamtx per un controllo veloce |
| macchina sim | TCP 16171, TCP 8061 | il DVL A50 emulato: protocollo JSON e API HTTP (la 80 del dispositivo vero) |
| linus, scenario C | 172.31.250.3:80 | `dvl-gateway`: la pagina dell'estensione DVL |
| Work, scenario B | TCP 9001 | la pagina dell'estensione DVL (`dvl-host`) |
| macchina sim | TCP 127.0.0.1:9997 | API di mediamtx: la camera sta pubblicando? (`stack.sh status`) |
| BlueOS | UDP 14551 | MAVLink da SITL (master della board Manual) |
| BlueOS | TCP 80, 6020, 6021; UDP 3478 + WebRTC | UI, camera manager, segnalazione e media WebRTC per Cockpit |
| BlueOS | UDP 14550 | "GCS Server Link" di BlueOS: un QGC si collega qui |

## Prestazioni misurate (linus, RTX 5050 8 GB, 2026-11-25)

Misure con 1280×720, profilo `linus`:

| Cosa | Valore |
|---|---|
| Stream | **30,0 fps** costanti |
| Render | 7 ms a fotogramma |
| Post-processing dell'acqua | 1,7 ms |
| Fisica | 400 pacchetti/s, 0 stalli |
| Video | 2 fotogrammi persi su 3288 |
| VRAM | picco 2,4 GB del processo |

Le leve che contano, misurate:
- **DLSS invece di DLAA:** 55 → 8 ms.
- **Un solo rimbalzo di rifrazione invece di due:** 31 → 8,5 ms con la superficie in vista, per 0,2/255 di differenza media nell'immagine.
- **Riflessi spenti sul profilo linus:** 26 → 38 fps limitati dalla GPU, per 1/255 di differenza.

Sulla Work restano accesi (profilo `workstation`).

## Cosa è simulato e cosa no

**Simulato:**
- dinamica Fossen 6-DOF della Heavy da von Benzon et al. 2022, Tab. A1 (misurati sulla Heavy: 13,5 kg, ∇ 0,0134 m³);
- otto T200 con banda morta e asimmetria;
- galleggiamento parziale in superficie: il ROV galleggia con il tetto appena fuori;
- pareti e fondo come limite morbido;
- camera con tilt e luci comandati da ArduSub;
- l'acqua sull'immagine: attenuazione e luce diffusa per canale, in funzione della distanza, con la torbidità regolabile.

**Non simulato:**
- collisioni con scaletta e oggetti;
- caustiche e coni di luce nell'acqua;
- la luce delle lampade attenuata nel tragitto verso l'oggetto (solo quello verso la camera);
- la distorsione del dome;
- il cavo ombelicale.

**Correzioni fatte in M11**, che valgono anche per l'HIL della room9 (`docs/ros_hil.md` §6.3):
- La mappa dei motori ora è quella di ArduSub. Prima 6 motori su 8 della Heavy e entrambi i verticali della versione a 6 avevano il verso sbagliato: stick su → il ROV scendeva.
- L'integratore RK4 bloccava la prua quando passava per 180° (sud).

**Il sensore di profondità di SITL era troppo rumoroso.** SITL aggiunge `SIM_BARn_RND` metri di rumore uniforme alla quota prima di convertirla in pressione d'acqua. Il default 0,2 significa ±20 cm di profondità (±20 mbar), mentre un Bar30 risolve 2 mm. Con quel rumore:
- ALT_HOLD oscillava di 5–17 cm picco-picco. Prima lo attribuivamo alla banda morta dei T200, ma era il rumore;
- la calibrazione della pressione a terra all'avvio poteva sbagliare fino a mezzo metro. Visto il 2026-11-26: Cockpit segnava 0,64 m con il ROV a 0,12 m.

`pool.parm` ora mette 0,005 m (stessa classe del Bar30). Misurato con `pool_smoke.py --dvl`: ALT_HOLD tiene **1 cm picco-picco**, e la profondità di ArduSub è entro 3 cm da quella vera.

## Da verificare (non ancora provato)

Tutto lo scenario B, con la tabella di verifica di [hil/win/pool/README.md](../hil/win/pool/README.md) (§4). In particolare:

- BlueOS in WSL2 con la rete mirrored sulla Work, e Cockpit con il video WebRTC e il gamepad. Sono state verificate su linus, con un BlueOS 1.5.0-beta.43 isolato:
  - la board Manual;
  - il MAVLink fino a mavlink2rest;
  - lo stream Redirect decodificato dalla camera manager.
- Lo scenario B: loopback mirrored tra WSL e Windows per la fisica e l'RTSP.
- Il `run_pool.bat` nativo e il profilo `workstation`.
- Il DVL sulla Work. L'estensione dallo store di BlueOS si bloccherebbe: aspetta cable_guy, che lì è spento. Per questo il profilo `work` ha `dvl-host` e `dvl-gateway-host`. La variante è provata su linus, ma non ancora sulla Work: restano da vedere la rete bridge docker con la rete mirrored e il TCP verso l'A50 su Windows.
