# vehicles/

Un veicolo = una cartella con un `vehicle.yaml`. Il file è il contratto tra la simulazione (che lo legge) e
l'identificazione dai log reali (`uw sysid <veicolo> --write`, che lo aggiorna).

| Cartella | Cosa |
|---|---|
| `common/` | dinamica Fossen a 6 gradi di libertà (`dynamics.py`), frame (`frames.py`), DVL A50 (`dvl.py`, `a50.py`), server fisico per il backend JSON di ArduSub SITL (`physics_udp.py`, `physics_proc.py`), **schema di `vehicle.yaml`** (`schema.py`) |
| `bluerov2_heavy/` | BlueROV2 Heavy (8 × T200, FRAME_CONFIG 2): `vehicle.yaml`, lo script che genera l'asset 3D (`assets/`, non in git) |
| `bluerov2/` | BlueROV2 standard (6 × T200, FRAME_CONFIG 1), quello che vola l'HIL della room9 |
| `momorov/` | segnaposto: il repo `sjckness/momoROV` è vuoto |
| `tools/` | misure della dinamica (`rov_sanity.py`) e del dead reckoning col DVL |

## vehicle.yaml

- **Unità SI**, scritte nel nome della chiave (`_kg`, `_n`, `_m`, `_s`, `_us`, `_hz`) e, per tutte, in
  `common/schema.py` (`FIELDS`). I coefficienti idrodinamici seguono la notazione di Fossen (≤ 0).
- **Frame**: corpo FRD (x avanti, y dritta, z giù), origine nel baricentro.
- **Provenienza**: ogni valore ha il suo commento con la fonte (`V` verificato, `UNVERIFIED`, `GUESS`,
  riferimenti come `[BENZON22]`).
- **Valori sconosciuti**: `null` + `# TODO`, solo dove lo schema lo permette; mai un numero inventato.
- La matrice di allocazione non si scrive: si ricava da posizione e asse di ogni thruster.

```bash
python -m vehicles.common.schema vehicles/bluerov2_heavy/vehicle.yaml   # controlla il file e stampa valori e unità
```

I risultati della dinamica letta da questi file coincidono con quelli di prima della ristrutturazione
(`tests/rov/test_vehicle_yaml.py`, contro `tests/rov/golden_vehicle_dynamics.json` generato a `e245944`).
