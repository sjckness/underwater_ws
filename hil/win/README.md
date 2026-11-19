# HIL su Windows (macchina "work")

File di ambiente per il link HIL diretto work <-> Thor. I corrispettivi per il Thor stanno in
`hil/thor/m8a/`. La sottorete del link e il dominio DDS sono documentati in `docs/ros_hil.md`.

| File | A cosa serve |
|---|---|
| `hil_env.bat` | variabili d'ambiente ROS 2 / Fast DDS della sessione HIL (dominio 73) |
| `fastdds_win.xml` | profilo Fast DDS: solo UDP unicast sul link diretto, niente multicast |
| `firewall_sitl.bat` | regole in ingresso per SITL (richiede privilegi di amministratore) |
| `netcheck.py` | diagnostica del link: interfacce, raggiungibilita', porte UDP/TCP |
| `run_room9.bat` | avvio della scena `room9` in modalita' nativa |
| `record_all.bat` | batch delle registrazioni headless |

## Ambiente Python nativo

L'ambiente sta fuori dal repo (`uw.bat` punta a `ISAAC_ENV`, per default `F:\envs\isaac`);
`python setup_native.py --write` genera `config/runtime.local.yaml`, che non e' in git.

**Pin di torch.** Serve la build CUDA:

```bat
pip install torch==2.7.0+cu126 --index-url https://download.pytorch.org/whl/cu126
```

Il suffisso `+cu126` e' obbligatorio: senza di esso pip considera il requisito `torch==2.7.0`
soddisfatto da `2.7.0+cpu`, e con la build CPU la scena si chiude dopo circa 3 secondi senza
errori, perche' il sonar allora alloca i tensori su CUDA. Dettagli e diagnosi in
`docs/hil_work_thor_setup.md`.
