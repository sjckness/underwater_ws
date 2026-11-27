# HIL work ↔ Thor — stato al 9 novembre 2026

Progetto `underwater_ws` (rilevamento bolle/gas): Isaac Sim 5.1 nativo sulla work (Windows) e Jetson Thor collegato con un cavo diretto. Questo file riassume la sessione del 9 novembre: cosa funziona, come si avvia, cosa è stato risolto e cosa resta da fare.

## Stato in sintesi

| Cosa | Stato |
|---|---|
| Scena della vasca in Isaac (GUI) con sink ROS | funziona, setup di circa 3 s |
| Bridge ROS 2 su Windows (rclpy Humble interno a Isaac) | funziona, nodo `uw_sim` sul dominio 73 |
| Collegamento ROS 2 work → Thor (DDS su UDP unicast) | funziona dopo la sistemazione del firewall (confermato) |
| Scena in tempo reale | **no**: RTF 0,82–0,95 con GUI a 20 fps |
| IP `192.168.200.10` permanente sul Thor | non verificato |
| Throughput e RTT del cavo (`netcheck`, Windows client → Thor server) | non ancora misurati |
| Frequenze e banda dei topic reali sul Thor, anello chiuso | da fare |

## Rete e configurazione

| | Work (Windows) | Thor |
|---|---|---|
| Link diretto | Ethernet 2, `192.168.200.2/24` | `enP2p1s0`, `192.168.200.10/24` (connessione "Wired connection 1", altri indirizzi lasciati com'erano) |
| ROS 2 | rclpy **Humble** interno al bridge di Isaac | **Jazzy**, `rmw_fastrtps_cpp` |
| Dominio | `ROS_DOMAIN_ID=73` (convenzione del repo) | `73`, solo nel terminale dove si fa `source ~/hil_env.sh` |

- L'ambiente F1Tenth del Thor (dominio 0, discovery server `127.0.0.1:11811`) non è stato toccato: `hil_env.sh` toglie le sue variabili solo nel terminale in cui viene caricato.
- Fast DDS è configurato con **solo UDP unicast** sul link diretto, una whitelist sull'IP del link e il peer iniziale all'altro capo. Le altre interfacce (rete di lavoro, Tailscale, VMware) restano fuori dal DDS.
- Con il dominio 73 la discovery parte dalla porta 25650. Windows annuncia ogni 3 s verso le porte 25660–25666 del Thor.

### Topic pubblicati dalla scena

`/oculus/polar`, `/oculus/polar_db`, `/oculus/fan`, `/oculus/meta` (latched), `/camera/image_raw`, `/camera/camera_info`, `/sim/leak`, `/sim/leak/flow_lpm` (latched), `/sim/plume`, `/tf_static`, `/clock`. Sonar e camera a 10 Hz.

## File creati

Sulla work, in `underwater_ws` (`output\` è ignorato da git):

| File | A cosa serve |
|---|---|
| `sim\tools\debug_sim.py` | avvia `uw sim` come il launcher nativo, ma stampa subito il traceback, mostra i log INFO e scrive le statistiche in `output\debug_stats.json` |
| `output\hil\hil_env.bat` | variabili ROS 2 per Windows (distribuzione, dominio, profilo DDS, DLL del bridge) |
| `output\hil\fastdds_win.xml` | profilo Fast DDS della work |
| `output\hil\netcheck.py` | test di rete TCP/UDP (throughput, RTT, perdite), solo libreria standard |
| `output\hil\thor_*.{sh,xml}` | copie dei file per il Thor |

Sul Thor, nella home: `~/hil_env.sh`, `~/hil_fastdds.xml`, `~/netcheck.py`.

## Come si avvia

**Work**, da `underwater_ws`:

```bat
.\uw.bat
call output\hil\hil_env.bat
python sim\tools\debug_sim.py --gui --sink ros --set lights.ambient_dome=300 > output\hil\run.log 2>&1
```

Il terminale resta muto perché il log va su file. Per fermare la scena: chiudere la finestra di Isaac e premere Invio nel terminale (scrive `output\debug_stats.json`).

**Thor**, in un terminale dedicato:

```bash
source ~/hil_env.sh
ros2 topic list --no-daemon --spin-time 5
ros2 topic hz /oculus/polar
ros2 topic echo /oculus/meta --once --qos-durability transient_local
```

In Jazzy `ros2 topic hz` non ha opzioni QoS e si sottoscrive già in best-effort.

## Problemi incontrati e cause

| Sintomo | Causa | Soluzione |
|---|---|---|
| La scena si chiudeva dopo 3 s, senza errori | `torch 2.7.0+cpu`: il sonar alloca tensori su CUDA | `pip install torch==2.7.0+cu126 --index-url https://download.pytorch.org/whl/cu126` (il suffisso `+cu126` è necessario, altrimenti pip considera soddisfatto `2.7.0+cpu`) |
| Nessun traceback, mancavano le righe `setup` e `running` | `main.main()` chiude Kit in un `finally`, e Kit in console mostra solo i Warning | `sim\tools\debug_sim.py` |
| `uw.bat` stampava sempre `uscita: 0` | `%ERRORLEVEL%` viene espanso dentro il blocco `( )` prima che Python parta | correzione qui sotto (da applicare) |
| `ros2 topic list` sul Thor vedeva solo `/rosout` e `/parameter_events` | il firewall di Windows scartava il traffico in ingresso dal Thor | vedi sezione Firewall |

Correzione di `uw.bat`: sostituire le righe 39–51 (`if "%~1"=="" ( … ) else ( … )`) con

```bat
if not "%~1"=="" goto :run
echo [uw] Pronto. Esempi:
echo        python deploy\hil\windows\setup_native.py --write
echo        python -m uw sim --gui --sink none --set lights.ambient_dome=300
echo        python sim\tools\debug_sim.py
echo.
cmd /k
exit /b

:run
python -m uw %*
set "UW_RC=%ERRORLEVEL%"
echo.
echo [uw] uscita: %UW_RC%
cmd /k
```

## Firewall

**Diagnosi.** Il test TCP in ingresso andava in timeout (nessun rifiuto, quindi pacchetti scartati). Una cattura con `tcpdump` sul Thor mostrava gli annunci di Windows arrivare e il Thor rispondere, senza risposta da Windows. Fast DDS invia da una porta di partenza effimera, quindi le risposte del Thor non rientrano nel flusso aperto da Windows: serve un consenso esplicito in ingresso.

**Causa finale.** Windows aveva creato da solo due regole di *blocco* (UDP e TCP, profili Privato e Pubblico) per il `python.exe` di `uv` (`…\uv\python\cpython-3.11.18-windows-x86_64-none\python.exe`), quando l'avviso del firewall è stato chiuso senza consentire. Un blocco batte sempre un consenso.

**Intervento (da amministratore):**

```bat
netsh advfirewall firewall delete rule name=python.exe program="%APPDATA%\uv\python\cpython-3.11.18-windows-x86_64-none\python.exe" dir=in
netsh advfirewall firewall add rule name="HIL python Thor (UDP)" dir=in action=allow protocol=UDP program="%APPDATA%\uv\python\cpython-3.11.18-windows-x86_64-none\python.exe" localip=192.168.200.2 remoteip=192.168.200.10 profile=any
```

Più la regola generica `HIL Thor in ingresso (UDP)` (stesso scopo di IP, senza programma), già verificata con `netsh advfirewall firewall show rule`. Dopo l'intervento il collegamento ROS 2 è stato confermato funzionante; le due regole nuove non sono state riverificate con `show rule`. Il TCP in ingresso è volutamente chiuso: il DDS usa solo UDP, quindi la parte TCP di `netcheck` in ingresso continuerà ad andare in timeout.

**Per togliere le regole:**

```bat
netsh advfirewall firewall delete rule name="HIL python Thor (UDP)"
netsh advfirewall firewall delete rule name="HIL Thor in ingresso (UDP)"
```

Se Windows mostra di nuovo l'avviso del firewall per `python.exe`, non annullarlo né chiuderlo (ricreerebbe i blocchi): lasciarlo aperto.

## Prestazioni della scena (GUI a 20 fps, sink ROS)

Prova di 8 minuti: RTF **0,88** (430 s di simulazione in 486 s). Una seconda prova di 14 minuti, ancora a 20 fps, ha dato RTF tra 0,82 e 0,95 (113 s di ritardo dopo 860 s). Sonar e camera erano alle frequenze nominali (10 Hz), nessun ping fuori budget.

| Fase | Tempo totale | Quota del tempo reale |
|---|---|---|
| Render RTX (27,4 ms a frame, 20 frame/s) | 236 s | 48% |
| Calcolo del ping sonar (13,5 ms, budget 100) | 60 s | 12% |
| Attesa della GPU prima del ping | 57 s | 12% |
| Pubblicazione ROS dei ping (12,9 ms) | 55 s | 11% |
| Lettura e pubblicazione camera | 23 s | 5% |
| Passi di simulazione delle bolle | 13 s | 3% |
| Non misurato (ciclo Python, `/clock`, plume) | 43 s | 9% |

VRAM usata a fine prova: circa 5,3 GiB su 24. Il render è il collo di bottiglia: GUI a 20 fps più camera a 10 Hz, con circa 62.000 bolle rifrangenti.

**Prova da fare:** `--set sim.gui_fps=10`, che dimezza i render (da 20 a 10 al secondo). La stima è un RTF vicino a 1,17 (cioè tempo reale con margine), ma non è ancora verificata. Leve successive, tutte solo per la GUI: `sim.viewport_resolution` (ora 1280×720) e `sim.overlay_returns=false`. Da non toccare: `bubble_stride`, risoluzione e frequenza della camera, perché cambiano ciò che il Thor vede.

Per l'HIL, finché l'RTF è sotto 1 il Thor deve seguire `/clock` (`use_sim_time`), non l'orologio di parete. Il costo di pubblicazione ROS potrebbe cambiare ora che il Thor si sottoscrive ai topic: da rimisurare.

## Da fare

- Verificare sul Thor che l'IP sia permanente: `nmcli -g ipv4.addresses con show "Wired connection 1"` (deve comparire `192.168.200.10/24`).
- Misurare il cavo: `netcheck` con server sul Thor e client sulla work.
- Provare `--set sim.gui_fps=10` e rileggere `output\debug_stats.json`.
- Applicare la correzione di `uw.bat`.
- Misurare frequenza e banda dei topic reali (`ros2 topic hz`, `ros2 topic bw`), in particolare `/oculus/polar_db` e `/camera/image_raw`.
- Differenza Humble (work) ↔ Jazzy (Thor): finora i messaggi standard comunicano; controllare i QoS nell'anello chiuso.
- Anello chiuso: il Thor risponde e Isaac usa l'uscita (`sonar.pose_source: ros`, topic `/uw/sonar_pose`).
- Fuori ambito per ora: dataset e training, Docker.
