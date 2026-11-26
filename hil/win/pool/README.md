# Vasca prove sulla Work (scenario B): installazione, avvio, verifica

Tutta la simulazione M11 sulla Work, senza Thor e senza hardware: Isaac, la fisica, ArduSub SITL, BlueOS, il DVL emulato con la sua estensione, Cockpit e il gamepad. Il quadro generale e il perché delle scelte sono in [docs/pool.md](../../../docs/pool.md); qui ci sono solo i passi per la Work.

È scritto anche per Claude Code lanciato sulla Work: i passi sono in ordine, ognuno con il risultato atteso.

**Regola.** Sulla Work non si fa commit: solo `git pull`. Se per far funzionare qualcosa serve cambiare un file, lo si cambia in locale e si prova. Poi si annotano la modifica (`git diff`) e il motivo in `output/pool/work_notes.md`, che non va in git, e si porta tutto su linus (vedi §6).

## 1. Come è messo

```
 Windows                                              WSL2: distro Ubuntu-24.04, Docker Engine, rete "mirrored"
 run_pool.bat ─► python -m uw pool (Isaac, nativo)    stack.sh up ─► docker compose --profile work
   fisica   UDP 127.0.0.1:9002  ◄──── lockstep ────►    sitl (ArduSub 4.5.7) ── MAVLink UDP 14551 ──► blueos (:80)
   A50      TCP 127.0.0.1:16171, HTTP :8061 ◄───────    dvl-host (estensione DVL, :9001)
                                                          └─ host.docker.internal = dvl-gateway-host (bridge .3)
                                                               └─► mavlink2rest di BlueOS ──► VISION_POSITION_DELTA
   ffmpeg   RTSP ─────────────────────────────────►     mediamtx :8654 ◄── pull ── camera manager di BlueOS
 Cockpit (app desktop o estensione) + gamepad ─────►    BlueOS su http://localhost
```

- Con la rete mirrored di WSL, il `127.0.0.1` di WSL è quello di Windows. Tra i due lati passano solo connessioni su loopback:
  - la fisica (UDP 9002);
  - il DVL (TCP 16171 e 8061);
  - il video (RTSP 8654);
  - browser e Cockpit verso BlueOS.
- BlueOS e l'estensione DVL girano in rete host dentro WSL. Il gateway dell'estensione sta su una rete bridge docker (172.31.250.0/24), dove vede WSL come 172.31.250.1.
- Su linus la stessa catena è provata: `pool_smoke.py --dvl` passa 18/18 con l'estensione in rete host, il 2026-11-26. **Sulla Work niente è ancora provato:** i punti a rischio sono in §5.

## 2. Preparazione (una volta)

**Requisiti:**
- Windows 11 22H2 o successivo (serve per la rete mirrored) e `wsl --version` ≥ 2.0.
- L'ambiente Isaac nativo della room9: `F:\envs\isaac`, con `python setup_native.py --write` già fatto (`hil/win/README.md`).

1. **Repo aggiornato.** `git pull` su `main`, con `git status` pulito.
2. **Fine riga.** Lo stack in WSL legge script e parametri da questa stessa copia del repo, attraverso `/mnt/<disco>/...`. Se Git per Windows li ha scritti con fine riga CRLF, l'immagine di SITL e gli script bash si rompono.
   - `.gitattributes` li fissa a LF, ma una copia fatta prima può averli ancora sbagliati.
   - Controllo: `git ls-files --eol docker hil/win/pool | findstr "w/crlf" | findstr "eol=lf"` non deve stampare nulla.
   - Se stampa qualcosa, e solo con `git status` pulito: `git rm -rq --cached .` e poi `git reset -q --hard`.
3. **WSL in modalità mirrored.**
   - Se `%UserProfile%\.wslconfig` non esiste, copiaci `hil\win\pool\wslconfig.example`.
   - Se esiste, aggiungi le chiavi dell'esempio senza togliere le altre: valgono per tutte le distro.
   - Poi `wsl --shutdown`.
4. **La distro.** `wsl -l -v`; se manca, `wsl --install -d Ubuntu-24.04`.
   - Con un altro nome, imposta `UW_WSL_DISTRO=<nome>` prima di `run_pool.bat`.
   - Se c'è Docker Desktop con l'integrazione WSL attiva su questa distro, disattiva l'integrazione per lei: BlueOS vuole Docker Engine dentro la distro, con la vera rete host.
5. **Dentro la distro**, dalla cartella del repo (es. `cd /mnt/f/<percorso>/underwater_ws`): `bash hil/win/pool/setup_blueos_wsl.sh`.
   - La prima volta abilita systemd e chiede `wsl --shutdown`: rilancialo dopo.
   - Poi installa Docker e i pacchetti Python, e aggiunge l'utente al gruppo docker: chiudi e riapri la distro.
   - Alla fine stampa gli avvisi da leggere: rete non mirrored, porte già prese su Windows, fine riga CRLF.
6. **Lo stack:** `bash hil/win/pool/stack.sh up`.
   - La prima volta costruisce l'immagine di SITL (circa 3 minuti) e scarica BlueOS, l'estensione, nginx e mediamtx (circa 2 GB).
   - Con BlueOS partito (1–2 minuti), `bash hil/win/pool/stack.sh status` deve dare `[ok]` su BlueOS e sul gateway DVL. Heartbeat, A50 e camera restano `FAIL` finché Isaac non gira: è normale.
7. **Il modello 3D del ROV**, su Windows, nella shell di `uw.bat`: `python -m uw py tools/fetch_bluerov2.py`. Scrive `assets\bluerov2_heavy\bluerov2_heavy.usd`; senza, la vasca mostra un parallelepipedo.
8. **Cockpit**, a scelta:
   - **App desktop per Windows**, dalle release di github.com/bluerobotics/cockpit, versione 1.19.0 come su linus. Si collega con *Settings → General → Global vehicle address* = `127.0.0.1`.
   - **Estensione di BlueOS:** `http://localhost` → *Extensions* → *Cockpit* → *Install*. Poi si apre dal menu di BlueOS.

   Il gamepad va collegato a Windows (USB o Bluetooth).

## 3. Ogni sessione

```bat
hil\win\pool\run_pool.bat --gui
```

Altre opzioni:
- `--preset turbid` o `--preset murky`: acqua torbida;
- `--view chase`: stream con la camera alle spalle del ROV;
- senza `--gui`: headless.

`run_pool.bat`, in ordine:
1. porta su lo stack in WSL (`stack.sh up`; se è già su non fa nulla);
2. riavvia SITL, perché una fisica nuova vuole un EKF nuovo (docs/pool.md, "Riavvia SITL insieme alla fisica");
3. apre una finestra ridotta a icona che aspetta il veicolo e poi configura BlueOS e il DVL (`stack.sh configure`, log in `output\pool\blueos_setup.log`);
4. lancia Isaac, con il log in `output\pool\run.log`.

Cosa aspettarsi:
- **Isaac** è pronto in 1–3 minuti. La prima volta compila gli shader per diversi minuti a 0 fps (su linus circa 3).
- **`run.log`:** ogni 10 s una riga di statistiche: fps, pacchetti fisici/s (400), stalli (0), report del DVL.
- **`blueos_setup.log`:** finisce con `done.`
- **Poi:** Cockpit, arm, e si vola.

**Stop:** `Ctrl+C` nella finestra di `run_pool.bat`. Lo stack in WSL può restare su; per spegnerlo, `wsl -d Ubuntu-24.04 -- bash hil/win/pool/stack.sh down` dalla cartella del repo.

## 4. Verifica (la prima volta, in ordine)

Al primo errore fermati e vai a §5.

| # | Cosa | Come | Atteso |
|---|---|---|---|
| 1 | rete WSL | `wslinfo --networking-mode` (in WSL) | `mirrored` |
| 2 | stack completo, con Isaac su | `bash hil/win/pool/stack.sh status` | tutte le righe `[ok]` |
| 3 | fisica ↔ SITL | statistiche in `output\pool\run.log`; `stack.sh logs sitl` | 400 pacchetti/s, 0 stalli; SITL non ripete "No JSON sensor message" |
| 4 | video | browser su Windows: `http://localhost:8889/rov` | la camera di bordo con l'acqua |
| 5 | BlueOS | `http://localhost` | veicolo collegato; in *Video* lo stream "Isaac ROV camera" running |
| 6 | DVL | `http://localhost:9001` | stato "Running", la traccia sulla mappa |
| 7 | prova di volo automatica | in WSL, nessuno che pilota: `bash hil/win/pool/stack.sh smoke --dvl` (circa 4 minuti) | `18/18 checks passed` |
| 8 | pilota | Cockpit + gamepad | video; arm, discesa, DEPTH HOLD, POSHOLD, luci, tilt |
| 9 | prestazioni | `output\pool\run_stats.json`, `nvidia-smi` | 30 fps; annota VRAM e ms di render |

## 5. Se qualcosa non va

I punti dove la Work può differire da linus, dal più probabile.

**Regolazioni locali senza toccare file in git:**
- Lo stack legge `docker/pool/.env`, che è in `.gitignore`: variabili come `PHYSICS_HOST=...` e `DVL_HOST=...`, una per riga.
- Isaac prende `--set chiave=valore` in coda a `run_pool.bat`.

| Sintomo | Causa probabile | Cosa fare |
|---|---|---|
| BlueOS non parte; `stack.sh logs blueos` dice "address already in use" | porta 80 o 8000 presa su Windows | `netstat -ano \| findstr :80` e `tasklist /fi "pid eq <PID>"`. Con IIS: `net stop w3svc` (da amministratore). Con PID 4 (http.sys): `netsh http show servicestate`. **Non disinstallare né disattivare nulla in modo permanente senza chiedere all'utente.** |
| `run.log`: 0 pacchetti fisici/s; SITL aspetta la fisica | l'UDP sul loopback mirrored non passa | In `docker/pool/.env`: `PHYSICS_HOST=<IPv4 di Windows>`. Poi `run_pool.bat --set physics.bind=0.0.0.0`. Windows può chiedere il permesso del firewall per python.exe: consentilo sulle reti private. |
| `status`: "A50 emulated" FAIL con Isaac su | il TCP verso Windows non passa sul loopback | In `docker/pool/.env`: `DVL_HOST=<IPv4 di Windows>:8061`. Poi `--set dvl.device.bind=0.0.0.0`. |
| `status`: "DVL gateway" FAIL; l'estensione resta su "waiting for cable-guy" | la rete bridge docker non funziona con la rete mirrored | Annota `ip addr` e `docker network inspect uw-pool-blueos` e riporta a linus: serve un'altra strada. |
| `status`: "camera publishing" FAIL con Isaac su | ffmpeg non raggiunge mediamtx | In `run.log` cerca l'errore di `stream`. Prova `--set stream.url=rtsp://<IPv4 di Windows>:8654/rov`. |
| Video ok in `:8889` ma non in Cockpit | WebRTC (UDP) tra Windows e WSL | `hostAddressLoopback=true` in `.wslconfig` (c'è nell'esempio), poi `wsl --shutdown`. Prova anche l'altra variante di Cockpit (§2.8). |
| `blueos_setup.log`: "no live heartbeat" | SITL non manda a BlueOS, o la fisica non gira | `stack.sh logs sitl`: SITL manda a `udpclient:127.0.0.1:14551`; BlueOS ascolta su 14551 (board Manual). Rilancia `stack.sh configure`. |
| Il ROV si disarma dopo 3 s in Cockpit | il failsafe `FS_PILOT_INPUT` | In Cockpit attiva l'invio del joystick anche con la finestra senza focus. |
| POSHOLD deriva, prua sbagliata in ArduSub | SITL non riavviato insieme alla fisica | `run_pool.bat` lo fa sempre. Se lanci `uw pool` a mano, prima `stack.sh restart-sitl`. |
| `stack.sh` o l'entrypoint di SITL: `$'\r': command not found` | file con fine riga CRLF | §2.2, poi `docker compose -f docker/pool/compose.yaml build --no-cache sitl` |

## 6. Cosa riportare a linus

In `output/pool/work_notes.md`:
- **data e versioni:** build di Windows (`winver`), `wsl --version`, driver NVIDIA (`nvidia-smi`);
- **verifiche:** l'esito di ogni riga di §4 con i numeri: fps, VRAM, ms di render, risultato di `smoke --dvl`;
- **regolazioni:** il contenuto esatto di `docker/pool/.env` e i `--set` usati;
- **modifiche:** `git diff` delle modifiche locali, con il motivo di ciascuna;
- **errori:** per ogni errore, le righe rilevanti di `run.log`, `blueos_setup.log` e `stack.sh logs <servizio>`.
