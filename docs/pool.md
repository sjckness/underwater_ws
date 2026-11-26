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
  - Sulla Work va in WSL2 con la rete host (profilo `blueos`).
  - Su linus gira in un container non privilegiato su una sua rete bridge, all'indirizzo fisso 172.31.250.2 (profilo `blueos-local`, scenario C).
- **Perché la board "Manual".** Esiste da BlueOS 1.5.0-beta.3: BlueOS non lancia un autopilota suo e prende il MAVLink da SITL. La board "SITL" delle 1.4 stabili lancerebbe ArduSub dentro BlueOS, lontano dalla fisica.

## Preparazione (una volta)

**Sulla macchina della simulazione:**
1. **Modello 3D del ROV.** Lancia `python -m uw py tools/fetch_bluerov2.py`.
   - Scarica la Heavy di Blue Robotics da BlueSim (ricavata dal loro CAD, già decimata, 126 k triangoli) e la converte in `assets/bluerov2_heavy/bluerov2_heavy.usd`.
   - Il file non va in git, per licenza (GPL-2.0) e dimensione.
   - Senza questo file la vasca mostra un parallelepipedo delle stesse misure.
2. **Immagine di ArduSub SITL.** Lancia `docker compose -f docker/pool/compose.yaml build sitl` (circa 3 minuti).

**Sulla Work, per BlueOS:**
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

**In WSL:**
```bash
docker compose -f docker/pool/compose.yaml build sitl          # la prima volta
docker compose -f docker/pool/compose.yaml --profile blueos up -d
python3 tools/pool_blueos_setup.py --blueos http://localhost --rtsp rtsp://127.0.0.1:8654/rov
```

**In Windows:** `hil\win\pool\run_pool.bat`. Usa il profilo `workstation` (30 fps, riflessi accesi).

Con la rete mirrored di WSL:
- il `127.0.0.1:9002` di SITL è quello di Windows, dove ascolta la fisica;
- il `127.0.0.1:8654` di mediamtx è quello dove Isaac pubblica.

**Da verificare** al primo avvio (non provato). Se il loopback non passa:
- `PHYSICS_HOST=<IP di Windows>` per SITL;
- `--set physics.bind=0.0.0.0` per la fisica.

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
- **Modi.** MANUAL, STABILIZE e ALT_HOLD (depth hold) funzionano sempre. POSHOLD funziona con il DVL, come sul ROV vero: nello scenario C sì; sulla Work l'estensione DVL non è ancora sistemata (vedi "Da verificare"). Senza DVL l'EKF non ha posizione orizzontale e POSHOLD non si attiva.
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

## Porte

| Dove | Porta | Cosa |
|---|---|---|
| macchina sim | UDP 9002 (loopback) | fisica ← ArduSub SITL (JSON) |
| macchina sim | UDP 14560 (loopback) | SITL → strumenti locali (`pool_smoke.py`, un QGC) |
| macchina sim | TCP 8654 | RTSP di mediamtx (solo TCP); 8889 è il viewer WebRTC di mediamtx per un controllo veloce |
| macchina sim | TCP 16171, TCP 8061 | il DVL A50 emulato: protocollo JSON e API HTTP (la 80 del dispositivo vero) |
| linus, scenario C | 172.31.250.3:80 | `dvl-gateway`: la pagina dell'estensione DVL |
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
- l'acqua sull'immagine: attenuazione e luce diffusa per canale, in funzione della distanza.

**Non simulato:**
- collisioni con scaletta e oggetti;
- caustiche e coni di luce nell'acqua;
- la luce delle lampade attenuata nel tragitto verso l'oggetto (solo quello verso la camera);
- la distorsione del dome;
- il cavo ombelicale.

**Correzioni fatte in M11**, che valgono anche per l'HIL della room9 (`docs/ros_hil.md` §6.3):
- La mappa dei motori ora è quella di ArduSub. Prima 6 motori su 8 della Heavy e entrambi i verticali della versione a 6 avevano il verso sbagliato: stick su → il ROV scendeva.
- L'integratore RK4 bloccava la prua quando passava per 180° (sud).

**ALT_HOLD non sta fermo.** La banda morta ±25 µs dei T200 e i +2 N di galleggiamento (servono circa 1540 µs per compensarli, appena fuori banda) fanno oscillare il controllo di profondità di ArduSub di 5–17 cm picco-picco, con periodo di circa 4 s. È coerente con un ROV leggermente positivo: si riduce zavorrando più vicino al neutro.

## Da verificare (non ancora provato)

- BlueOS in WSL2 con la rete mirrored sulla Work, e Cockpit con il video WebRTC e il gamepad. Sono state verificate su linus, con un BlueOS 1.5.0-beta.43 isolato:
  - la board Manual;
  - il MAVLink fino a mavlink2rest;
  - lo stream Redirect decodificato dalla camera manager.
- Lo scenario B: loopback mirrored tra WSL e Windows per la fisica e l'RTSP.
- Il `run_pool.bat` nativo e il profilo `workstation`.
- Il DVL sulla Work. L'estensione dallo store di BlueOS si blocca: aspetta cable_guy, che lì è spento. La strada è la stessa di linus: i servizi `dvl` e `dvl-gateway` del compose, con il gateway verso il BlueOS in rete host e `DVL_HOST` all'indirizzo di Windows. Non ancora preparato né provato.
