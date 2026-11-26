# M11 — La vasca prove: BlueROV2 Heavy pilotato con BlueOS, Cockpit e gamepad

Una vasca 10 × 10 × 5 m in Isaac Sim con un BlueROV2 Heavy che si pilota **come quello vero**:
- gamepad Bluetooth sul PC;
- Cockpit e BlueOS aperti nel browser;
- ArduSub che comanda gli otto motori.

Isaac fa solo la fisica del veicolo e la camera di bordo. Niente bolle, niente sonar.

Questo file è la guida operativa. Il resto è nel codice:
- l'app: `uw/pool.py`, `uw/pool_scene.py`, `uw/camera_uw.py`, `uw/video.py`;
- la fisica: `uw/rov/physics_proc.py`;
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
python3 tools/pool_blueos_setup.py --blueos http://127.0.0.1:28080 --rtsp rtsp://172.31.250.1:8654/rov
python3 -m uw pool --gui          # viewport alle spalle del ROV; --gui-view onboard|free per cambiarlo
~/.local/opt/cockpit-1.19.0/Cockpit-linux-x86_64-1.19.0.AppImage --no-sandbox
```

- **BlueOS** è su `http://localhost:28080` (oppure `http://172.31.250.2`).
- **Cockpit desktop** (v1.19.0, AppImage da github.com/bluerobotics/cockpit/releases) si collega con *Settings → General → Global vehicle address* = `172.31.250.2`.
  - `--no-sandbox` serve su Ubuntu 24.04, che vieta i namespace utente senza privilegi (`kernel.apparmor_restrict_unprivileged_userns = 1`): senza, Electron non parte.
- **La camera di bordo con l'acqua** è in Cockpit, oppure nel browser su `http://localhost:8889/rov` (il viewer WebRTC di mediamtx). Il viewport di Isaac mostra il render grezzo, senza il post-processing dell'acqua.
- **Gamepad:** un controller USB o Bluetooth collegato a linus, letto da Cockpit.
- **Stop:** `Ctrl-C` su `uw pool`, poi `docker compose -f docker/pool/compose.yaml --profile blueos-local down`. BlueOS tiene la configurazione nel volume `uw-pool_blueos_local_config`.

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
- **Modi.** MANUAL, STABILIZE e ALT_HOLD (depth hold) funzionano. POSHOLD richiede una sorgente di posizione (un DVL), che qui manca come sul ROV senza DVL.
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

Per provare SITL senza GPU c'è la fisica da sola: `python -m uw py -m uw.rov.physics_proc` (il ROV è invisibile ma vola).

## Porte

| Dove | Porta | Cosa |
|---|---|---|
| macchina sim | UDP 9002 (loopback) | fisica ← ArduSub SITL (JSON) |
| macchina sim | UDP 14560 (loopback) | SITL → strumenti locali (`pool_smoke.py`, un QGC) |
| macchina sim | TCP 8654 | RTSP di mediamtx (solo TCP); 8889 è il viewer WebRTC di mediamtx per un controllo veloce |
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
