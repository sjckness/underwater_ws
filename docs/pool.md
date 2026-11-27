# M11 — La vasca prove: BlueROV2 Heavy pilotato con BlueOS, Cockpit e gamepad

Una vasca 50 × 25 m con 5 m d'acqua (dal 2026-11-27; la 10 × 10 × 5 m di M11 con `--preset pool10`) in Isaac Sim, con un BlueROV2 Heavy che si pilota **come quello vero**:
- gamepad Bluetooth sul PC;
- Cockpit e BlueOS aperti nel browser;
- ArduSub che comanda gli otto motori.

Isaac fa solo la fisica del veicolo e la camera di bordo, più un DVL Water Linked A50 emulato che alimenta ArduSub attraverso la vera estensione DVL di BlueOS (POSHOLD come sul ROV vero). Niente bolle, niente sonar.

In acqua ci sono otto oggetti che un YOLO COCO riconosce (un sub, una canoa, una bici, una sedia, una bottiglia, una valigia, uno zaino, un pallone) e si può accendere una corrente, anche dal vivo.

Questo file è la guida operativa. Il resto è nel codice:
- l'app: `uw/pool.py`, `uw/pool_scene.py`, `uw/camera_uw.py`, `uw/video.py`;
- la fisica: `uw/rov/physics_proc.py`;
- il DVL: `uw/rov/a50.py` (il dispositivo in rete), `uw/rov/dvl.py` (fasci, rumore, aggancio al fondo), `uw/pool_geometry.py` (cosa colpiscono i fasci);
- lo stack: `docker/pool/compose.yaml`;
- la configurazione: `config/pool/` (gli oggetti in `config/pool/objects.yaml`);
- la corrente e il canale di controllo: `uw/rov/current.py`, `uw/rov/simctl.py`, `tools/simctl.py`.

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

## Dimensioni della vasca

`pool.size` in `config/pool/pool.yaml`: **50 × 25 m** con 5 m d'acqua, la pianta di una vasca olimpica con la profondità di una vasca da tuffi.
- `--preset pool10` riporta la 10 × 10 m di M11, con i suoi oggetti dove erano.
- Seguono la dimensione da soli: pareti, fondo, griglia, superficie, scaletta (2 m dalla parete x = L), DVL e limite morbido della fisica. Le luci del soffitto sono una ogni `lights.ceiling.spacing_m` (5 m: 2 × 2 nella piccola, 10 × 5 nella grande). Gli oggetti sono messi in frazioni di L e W.
- Non seguono la dimensione, quindi stanno nel preset: partenza del ROV, piano lontano della camera, disposizione degli oggetti COCO.
- In entrambe il ROV parte a galla vicino alla parete y = 0, a metà lunghezza, rivolto verso la parete opposta. Gli oggetti sono davanti, tra 3 e 15 m.

**Costo misurato** (2026-11-27, linus, RTX 3080, 150 s di `uw pool` dalla partenza, profilo `linus`):

| | 10 × 10 | 50 × 25 |
|---|---|---|
| Stream | 29,92 fps | 29,92 fps |
| Render, media / p95 | 7,27 / 8,16 ms | 7,39 / 8,30 ms |
| Post-processing dell'acqua | 1,55 ms | 1,56 ms |
| VRAM del processo, picco | 3,02 GB | 3,12 GB |
| Stalli della fisica | 0 | 0 |

Il render costa per pixel, non per metro quadro. La vasca grande aggiunge solo 46 luci rettangolari e qualche quad, quindi è diventata la default.

## Oggetti COCO

Otto oggetti che hanno senso sott'acqua e che un YOLO addestrato su COCO conosce. Il catalogo è `config/pool/objects.yaml`: per ogni oggetto la classe COCO, le dimensioni reali, il materiale acustico e la sorgente del modello 3-D. La disposizione è `pool.objects`.

| id | classe COCO | modello | licenza |
|---|---|---|---|
| `diver` | person | "Deep Diver", sub con muta, bombola e pinne, in posa di nuoto, a 0,5 m dal fondo | CC-BY-4.0, Miguelangelo Rosario |
| `canoe` | boat | "Canoe", canoa in legno, 3,65 m | CC-BY-4.0, BoatUS Foundation |
| `bicycle` | bicycle | "Bike", bici da città con cestino | CC-BY-4.0, krichwow |
| `chair` | chair | "Plastic Monobloc Chair 01", la sedia bianca da bordo piscina | CC0, Kuutti Siitonen (Poly Haven) |
| `bottle` | bottle | "Water Bottle", bottiglia d'acqua da 1,5 l | CC-BY-4.0, Microsoft |
| `suitcase` | suitcase | "Luggage", trolley nero con la maniglia alzata | CC-BY-4.0, KoteTheCat |
| `daypack` | backpack | "Backpack", zainetto grigio | CC-BY-4.0, typhomnt |
| `ball` | sports ball | "Soccer Ball", pallone bianco e nero | CC-BY-4.0, typhomnt |

Nel catalogo ci sono anche `rowboat` (una barca a remi) e `backpack` (uno zaino tattico, da fotogrammetria): provati, YOLO li riconosce peggio.

**Preparazione (una volta):**

```bash
python -m uw py tools/fetch_props.py          # scarica, controlla lo sha256, converte in USD (Kit), scala e centra
python3 tools/fetch_props.py --list           # cosa c'è, con le licenze
```

- I file vanno in `assets/props/<id>/<id>.usd`, fuori da git per licenza e dimensione. `assets/props/ATTRIBUTION.md` ha i crediti chiesti da CC-BY.
- Le sorgenti sono fissate: Objaverse a una revisione di Hugging Face, Poly Haven con lo sha256 di ogni file.
- Il modello è scalato in modo uniforme sulla dimensione più grande del catalogo e messo con l'origine al centro della base.
- I materiali glTF trasparenti ("BLEND") diventano opachi: la bici altrimenti spariva.
- **Senza i modelli** la vasca mostra al loro posto un parallelepipedo (o una sfera) delle stesse misure.

**Il DVL li vede**: il catalogo dà a ogni oggetto un solido (box ruotato o sfera) delle sue misure, nella stessa lista di scaletta e props (`uw/pool_geometry.py`). I materiali acustici sono `plastic`, `steel`, `neoprene`, `fabric`.

**Ogni oggetto ha la sua etichetta semantica** (la classe COCO), quindi Replicator ne dà i box 2-D: la base per misurare un detector, o per addestrarne uno.

### Quanto li vede YOLO

`uw pool --views NOME` non fa partire né la fisica né lo stream. Mette la camera davanti a ogni oggetto, a 2, 3, 4, 6 e 8 m da quattro direzioni, e scrive i fotogrammi con l'acqua applicata, più i box veri di Replicator. Poi `tools/eval_views.py` fa girare il modello e conta i colpi: classe giusta, IoU ≥ 0,5 con il box vero.

```bash
python -m uw pool --views lit --view-height -0.2 --view-lights 1.0 [--preset turbid]   # camera a 20 cm sotto la cima dell'oggetto, luci accese
$UW_YOLO_PYTHON tools/eval_views.py lit --model yolo26m.pt                              # tabella per classe e distanza
```

**Misurato** (2026-11-27, linus): `yolo26m`, conf 0,25, camera di fianco con le Lumen accese. Colpi su viste a 2 / 3 / 4 / 6 / 8 m (4 viste per distanza).

| Classe | Acqua limpida | `--preset turbid` (visibilità 5,8 m) | `--preset murky` (2,5 m) |
|---|---|---|---|
| person (il sub) | 4 4 4 4 3 | 4 4 2 0 0 | 2 0 0 0 0 |
| bicycle | 4 4 4 4 4 | 4 4 4 0 0 | 4 0 0 0 0 |
| chair | 4 4 4 4 4 | 4 4 4 0 0 | 4 0 0 0 0 |
| suitcase | 4 4 4 4 2 | 4 4 2 0 0 | 2 0 0 0 0 |
| bottle | 3 4 4 0 0 | 4 3 1 0 0 | 2 0 0 0 0 |
| sports ball | 4 4 3 1 1 | 4 4 0 0 0 | 0 0 0 0 0 |
| boat (la canoa) | 2 2 1 1 3 | 1 0 0 0 0 | 0 0 0 0 0 |
| backpack | 0 0 0 0 0 (preso per *suitcase*) | 0 | 0 |

Cosa se ne ricava:
- **Il modello conta più delle luci.** In acqua limpida `yolo26n` (il default del detector) trova bici e sedia sempre, il sub una volta su due (lo scambia per un *bird*), e il resto poco: valigia 3/20, bottiglia 1/18. `yolo26m` porta sub, valigia, bottiglia e pallone ai livelli della tabella. Per la vasca usa `--model yolo26m.pt` (`assets/models/`).
- **Meglio di lato che dall'alto.** Con la camera 1 m sopra l'oggetto, cioè guardando in giù di 20–30°, la canoa e la bottiglia quasi spariscono. I dataset COCO sono foto prese all'altezza degli occhi.
- **Le Lumen aiutano poco**: +1 colpo sulla canoa, +2 sul pallone.
- **La torbidità decide la distanza.** Con l'acqua torbida si riconosce fino a 3–4 m, con quella molto torbida solo a 2 m. Oltre la visibilità non si vede niente, come sul ROV vero.
- **La canoa** viene letta spesso come *surfboard*, e la barca a remi come *airplane*.
- **Lo zaino non lo prende mai** (nessuno dei due modelli): è il candidato per un fine-tuning sui box di Replicator.

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

## Corrente

L'acqua può muoversi: una corrente orizzontale, uguale in tutta la vasca in x e y, che entra solo nella dinamica del veicolo (`uw/rov/current.py`, `uw/rov/dynamics.py`). Non si vede nell'immagine: niente particelle che scorrono.

**Il modello.** È quello di Fossen (2021, §10.3) per una corrente irrotazionale:
- massa aggiunta e smorzamento vedono la velocità **rispetto all'acqua**;
- massa e inerzia del corpo rigido vedono quella **rispetto al fondo**, che è anche quella da cui si integra la posizione.

Quindi, senza spinta, il ROV finisce per muoversi con l'acqua. Il DVL (bottom track) misura la velocità rispetto al fondo e vede la deriva, come quello vero. Ci sono due termini in più, che contano solo quando il ROV ruota o la corrente cambia:
- la corrente vista dal corpo che gira;
- la spinta dell'acqua che accelera (Froude-Krylov).

Senza il primo, un ROV fermo nell'acqua che ruota verrebbe "lanciato" fuori dalla sua particella d'acqua.

**I parametri** (`current:` in `config/pool/pool.yaml`):

| Chiave | Cosa |
|---|---|
| `speed_mps` | velocità in superficie [m/s]; 0 = acqua ferma (il default) |
| `direction_deg` | **verso dove** scorre l'acqua, azimut ENU della vasca: 0 = +x, 90 = +y (come la prua di `vehicle.start`) |
| `profile` | `uniform`; `power` (legge 1/7 dal fondo, `power_exponent`); `linear` (`bottom_factor` sul fondo); `table` (`[[profondità, fattore], ...]`). Il fattore vale sempre 1 in superficie |
| `gust.sigma_mps`, `gust.dir_sigma_deg`, `gust.time_s` | raffiche su velocità e direzione: uno spettro casuale fisso (sei sinusoidi), deterministico per un dato `seed` |

**Preset** (gruppo `current`, si combinano con quelli dell'acqua):

```bash
python -m uw pool --preset light                  # 0,10 m/s verso +y, uniforme
python -m uw pool --preset strong --preset turbid  # 0,40 m/s verso +x
python -m uw pool --preset gusty                   # 0,25 ± 0,08 m/s verso +x, ± 15°, profilo 1/7
python -m uw pool --set current.speed_mps=0.3 --set current.direction_deg=45
```

La corrente vale anche per la fisica da sola (`python -m uw py -m uw.rov.physics_proc --preset strong`), che la scrive nella riga di log ogni 2 s ("water … m/s").

**A simulazione avviata** si cambia con il canale di controllo, senza riavviare niente (vedi sotto). Il passaggio dal vecchio valore al nuovo è una rampa smoothstep di `--ramp` secondi, contati nel tempo simulato.

## Canale di controllo (`simctl`)

Il processo della fisica (`uw pool`, oppure `uw.rov.physics_proc` da solo) ascolta su **TCP 9020**. Il protocollo è `uwsim/1`: JSON una riga per richiesta, solo stdlib (`uw/rov/simctl.py`, client `uw/rov/simctl_client.py`).
- **Su linus** ascolta su `127.0.0.1` e sul bridge docker `172.31.250.1`, da dove lo raggiunge il container ROS.
- **Sulla Work** ascolta solo sul loopback, che WSL vede in modalità mirrored.

```bash
python3 tools/simctl.py hello                                   # versione, vasca, corrente, dispositivi, SITL vivo?
python3 tools/simctl.py current --speed 0.3 --dir 90 --ramp 3   # corrente dal vivo
python3 tools/simctl.py still --ramp 2                          # acqua ferma
python3 tools/simctl.py stream --rate 5                         # posizione vera, velocità sul fondo e dell'acqua
python3 tools/simctl.py reposition 5 5                          # sposta il ROV in orizzontale
python3 tools/simctl.py device a50 acoustic_enabled=false       # spegne il ping dell'A50 (dropout)
python3 tools/simctl.py stats                                   # tempo di passo p50/p95/max, pacchetti, stalli
```

- **`reposition` è solo orizzontale.** Profondità e assetto restano, velocità e spinte vanno a zero, così baro, bussola, giroscopi e DVL non vedono niente e l'EKF di ArduSub non se ne accorge. Il target di POSHOLD però è in coordinate EKF: riposiziona in ALT_HOLD, non in POSHOLD.
- **Niente cambio di DVL, vasca o veicolo a runtime.** Cambierebbero porte, estensioni e parametri di ArduSub: per quelli si riavviano fisica e SITL insieme.
- `pool_smoke.py --dvl --current 0.3 [--current-dir 0]` usa il canale per accendere la corrente mentre POSHOLD tiene il posto.

**Misurato** (linus): con uno stream a 50 Hz aperto per 10 minuti la fisica tiene 400 pacchetti/s, nessuno stallo, tempo di passo p50 0,33 ms, p95 0,41 ms, max 1,3 ms.

**Senza corrente il modello è identico bit per bit a prima.** Lo verifica `tests/rov/test_current.py` con un golden scritto dal codice precedente.

**Costo:** circa 50 µs a passo in più (100 con raffiche e profilo), su 250. A 400 Hz sono 2–4 % di un core.

**Misurato** (2026-11-27, linus, fisica da sola + SITL + estensione DVL; la corrente accesa dal vivo con `simctl`):

| Cosa | Valore |
|---|---|
| ROV disarmato in superficie, corrente 0,20 m/s verso +y | deriva a 0,200 m/s, in direzione +y |
| `pool_smoke.py --dvl` con acqua ferma | 18/18 (identico a prima) |
| POSHOLD (A50), corrente 0,30 m/s di traverso accesa in 3 s | picco 12,6 cm, poi entro 2,0 cm dal punto negli ultimi 10 s su 30 |
| Lo stesso, 10 s dopo aver fermato l'acqua | 13,7 cm dal punto: l'integratore di POSHOLD spinge ancora contro la corrente che non c'è più |

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

## Nucleus1000

Il secondo DVL del confronto: un Nortek Nucleus1000 (DVL a tre fasci, AHRS e INS nello stesso strumento) emulato nel processo della fisica come l'A50 (`uw/rov/nucleus.py`). Lo leggono, senza modifiche, i tre client di Nortek:
- il driver Python `nucleus_driver` (1.7.0 e 1.7.8);
- l'estensione BlueOS `nortekgroup/blueos_nucleus:2.0.11`;
- il nodo ROS 2.

**Come è fatto.** È ricavato dal sorgente di quei client e dal manuale N3015-033; dettagli in `docs/dvl.md`.
- **Rete:** TCP 9000 con password (`nortek`), comandi ASCII o NMEA, un client alla volta; TCP 9002 solo dati.
- **Pacchetti:** AHRS 0xD2 a 20 Hz, bottom track 0xB4 e altimetro 0xAA sulla frequenza di trigger (8 Hz, un ping su quattro all'altimetro), INS 0xDC a 10 Hz.
- **Fasci:** il modello di `uw/rov/dvl.py`, con tre fasci a 25° e tutti e tre necessari (config `nucleus:`; valori UNVERIFIED dove il datasheet non li dà).
- **AHRS:** l'assetto vero con rumore e un errore di prua lento.
- **INS:** integra la velocità sul fondo con l'assetto dell'AHRS, come lo strumento.

**Avvio** (al posto dell'A50, scenario C):

```bash
docker compose -f docker/pool/compose.yaml stop dvl                       # l'estensione Water Linked
SITL_PARAM_FILES=/opt/ardupilot/default_params/sub-6dof.parm,/params/pool.parm,/params/nucleus.parm SITL_WIPE=1 \
  BLUEOS_HOST=172.31.250.2 docker compose -f docker/pool/compose.yaml up -d --force-recreate sitl
BLUEOS_HOST=172.31.250.2 docker compose -f docker/pool/compose.yaml --profile blueos-local --profile nucleus up -d nucleus
python3 -m uw pool --preset nucleus           # oppure la fisica da sola: python3 -m uw.rov.physics_proc --preset nucleus
python3 tools/pool_blueos_setup.py --blueos http://127.0.0.1:28080 --rtsp rtsp://172.31.250.1:8654/rov --nucleus http://172.31.250.7
```

- **L'estensione di Nortek** si collega e parte da sola: compose le scrive `autoconnect` e `autostart`. La sua pagina è `http://172.31.250.7`.
  - Raggiunge BlueOS attraverso `dvl-gateway`, come l'estensione Water Linked. Il gateway ora risponde anche al controllo `info.connected` di cable_guy che lei fa.
  - Manda ad ArduSub `VISION_POSITION_DELTA` (circa 6 Hz), `VISION_POSITION_ESTIMATE` (10 Hz, dall'INS) e `DISTANCE_SENSOR`.
- **`nucleus.parm`** è la configurazione che il README di Nortek chiede all'operatore: posizione, velocità **e prua** dalla navigazione esterna (`EK3_SRC1_YAW 6`), la bussola come seconda sorgente di prua. Per questo SITL va riavviato con `WIPE=1`.
- **Per tornare all'A50:** fermare `nucleus`, ricreare `sitl` con i parametri di default e `SITL_WIPE=1`, poi rilanciare `dvl`.
- `--preset both_dvl` mette sul ROV entrambi i DVL, ciascuno con le sue porte e i suoi errori. Una sola estensione però deve alimentare ArduSub.

**Misurato** (2026-11-27, linus, fisica da sola + SITL + estensione Nortek, `pool_smoke.py --dvl --current 0.3`): **19/19**.

| Cosa | A50 | Nucleus1000 |
|---|---|---|
| Prua di ArduSub contro quella vera | +0,8° | +0,1° |
| POSHOLD fermo per 10 s | 5,9 cm | 0,8 cm |
| Dopo 1 m di stick, tenuta per 8 s | 4,9 cm | 2,0 cm |
| Navigazione stimata dopo ~1,1 m | 9,6 cm | 8,5 cm |
| POSHOLD con 0,30 m/s di traverso: picco | 12,6 cm | 14,9 cm |
| Lo stesso, negli ultimi 10 s su 30 | entro 2,0 cm | entro 9,8 cm |

Una prova per DVL: il confronto vero lo farà il banco (P9), con le ripetizioni.

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

**Cosa riconosce.** Dal 2026-11-27 in vasca ci sono gli oggetti COCO: vedi "Oggetti COCO" sopra, con le misure per classe. In breve: con `--model yolo26m.pt` sei classi su otto si riconoscono bene in acqua limpida: fino a 4 m bottiglia e pallone, fino a 8 m sub, bici, sedia e valigia. La canoa si riconosce a fatica, lo zaino mai. Sulla vasca nuda (pipe, blocco, scacchiera, scaletta) un modello COCO non trova niente. Per quella le strade sono due:
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
| macchina sim | TCP 9020 | canale di controllo della simulazione (`tools/simctl.py`): loopback, e su linus anche 172.31.250.1 |
| macchina sim | TCP 9000, TCP 9002 | il Nucleus1000 emulato (`--preset nucleus`): comandi con password, solo dati |
| linus | 172.31.250.7:80 | la pagina dell'estensione Nortek (profilo `nucleus`) |
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
- la corrente d'acqua nella dinamica (Fossen, velocità relativa all'acqua), con profilo in profondità e raffiche.

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
