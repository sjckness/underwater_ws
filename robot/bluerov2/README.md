# BlueROV2 reale

Configurazione, firmware e procedure del ROV vero: Raspberry Pi 4 con Navigator, BlueOS, ArduSub, nessun GPS.
Il modello simulato dello stesso veicolo è in `vehicles/bluerov2_heavy/`; i parametri identificati dai log
arrivano lì da `sysid/`.

| Cartella / file | Cosa contiene |
|---|---|
| `VERSIONS.md` | versioni fissate di BlueOS, ArduSub, estensione ROS, SD: da compilare a ogni cambio |
| `ardusub/params/` | file `.params` esportati, uno per modifica (`AAAA-MM-GG_<motivo>.params`) |
| `blueos/` | impostazioni dell'estensione ROS 2, profilo Cockpit, impostazioni esportate di BlueOS |
| `checklists/` | pre-immersione e post-immersione |
| `logs/INDEX.md` | indice dei log scaricati (`uw logs pull rov`); i log stanno fuori da git |

## Rete e accesso

Computer a terra (topside) e Pi sono sulla `192.168.2.0/24` del tether (Fathom-X / USB-Ethernet):

| Macchina | Indirizzo |
|---|---|
| topside | `192.168.2.1`, **statico** (BlueOS manda MAVLink ai GCS su `192.168.2.1:14550`) |
| Pi del ROV | `192.168.2.2` |

- **Web UI di BlueOS**: <http://192.168.2.2>. Cockpit è un'estensione (o l'app desktop) che si collega lì.
- **Pirate mode**: menu delle impostazioni di BlueOS (in alto a destra) → interruttore *Pirate Mode*. Sblocca terminale, gestore delle
  estensioni, file browser. Usarla solo per leggere o per gestire la *nostra* estensione.
- **Terminale**: BlueOS → *Terminal* (pirate mode). Si apre **dentro il container `blueos-core`**; `red-pill`
  porta nella shell dell'host della Pi. Da lì `docker ps` mostra i container.
- **SSH**: `ssh pi@192.168.2.2` (password di default dell'immagine BlueOS: `raspberry`; cambiala e scrivilo in
  `VERSIONS.md`). Copia la chiave del topside: `ssh-copy-id pi@192.168.2.2` — `uw deploy rov`, `uw smoke rov`
  e `uw logs pull rov` usano SSH con chiave.
- **Estensione ROS 2** ([itskalvik/blueos-ros2](https://github.com/itskalvik/blueos-ros2), ROS 2 Jazzy):
  - immagine `itskalvik/blueos-ros2:0.1.0`, l'unico tag Jazzy (amd64, arm64);
  - container sulla Pi: `extension-itskalvikblueosros2010` (regola di BlueOS kraken:
    `"extension-" + docker + tag` senza caratteri non alfanumerici) — **verificalo** con `docker ps`:
    `uw deploy rov --discover` lo scrive in `deploy/targets/rov.local`;
  - cartella persistente: host `/usr/blueos/extensions/ros2/` = container `/root/persistent_ws/`. Il nostro
    workspace va in `/root/persistent_ws/ros2_ws/` (`src/` sincronizzato da `ros/src`, poi `colcon build`).
    L'estensione **non** compila né carica `persistent_ws` da sola: la sua shell carica solo `/root/ros2_ws`;
  - terminale web dell'estensione (ttyd) e nginx sulla porta 4717; MAVROS e `foxglove_bridge` sono già
    nell'immagine.
- **Log di ArduSub (.BIN)** sulla Pi: `/root/.config/blueos/ardupilot-manager/firmware/logs/` (dedotto dal
  sorgente di BlueOS: `--log-directory {firmware}/logs/` nel core, `/root/.config/blueos` montato come
  `/root/.config`). In BlueOS: *Log Browser* (`/vehicle/logs`) o file browser
  (`/tools/file-browser/ardupilot_logs/firmware/logs`). `uw logs pull rov` verifica il percorso prima di copiare.
- **MAVLink per i nostri programmi**: il link interno di BlueOS è un server TCP su `127.0.0.1:5777` (MAVROS:
  `fcu_url:=tcp://127.0.0.1:5777@`, l'estensione è in host network). Lo stato (armato/disarmato) si legge via
  HTTP da mavlink2rest: `http://192.168.2.2/mavlink2rest/mavlink/vehicles/1/components/1/messages/HEARTBEAT`,
  armato se `message.base_mode.bits & 128`.

## Regole per non rompere BlueOS

1. **Prima di tutto, la copia della SD** (immagine intera, `dd` o Raspberry Pi Imager → *Read*), con data in
   `VERSIONS.md`. Una SD rovinata senza copia = ROV fermo.
2. Si toccano **solo** il container della nostra estensione ROS e `/usr/blueos/extensions/ros2/`. Mai:
   riavviare, fermare o aggiornare `blueos-core` / `blueos-bootstrap`, `docker system prune`, `apt` sull'host
   della Pi, file sotto `/root/.config/blueos/` (se non per leggere i log).
3. Aggiornamenti di BlueOS o di ArduSub: solo a mano, dalla web UI, dopo la copia della SD e con
   `VERSIONS.md` aggiornato prima e dopo.
4. **Parametri di ArduSub**: li cambia una persona (Cockpit, QGroundControl o BlueOS), mai uno script. Prima e
   dopo ogni modifica: export in `ardusub/params/`.
5. **Nessuno script arma il veicolo.** `uw deploy rov` e `uw smoke rov` controllano che sia **disarmato** e si
   fermano altrimenti; non mandano comandi di movimento né parametri.
6. Tutto quello che gira sulla Pi passa da `uw deploy rov` (rsync di `ros/src` + `colcon build` nel container
   dell'estensione): niente modifiche a mano dentro il container, si perdono al primo aggiornamento.

## Ponte Tailscale sul topside

Per raggiungere `192.168.2.x` da fuori (linus, Work) passando dal topside:

```bash
# sul topside, una volta
sudo nmcli con mod "<tether>" ipv4.method manual ipv4.addresses 192.168.2.1/24   # IP fisso sul tether
echo 'net.ipv4.ip_forward = 1' | sudo tee /etc/sysctl.d/99-tailscale.conf && sudo sysctl --system
sudo tailscale up --advertise-routes=192.168.2.0/24
# nella console di Tailscale: approva la route del topside
# sui client (linus, work)
sudo tailscale up --accept-routes
ping 192.168.2.2
```

Tailscale fa SNAT delle route annunciate (default su Linux): la Pi vede il traffico arrivare da `192.168.2.1` e
non serve toccarne la rete. Se il nome dell'interfaccia del tether cambia, l'IP fisso va rifatto.
