# sysid

Dai log del veicolo reale (`.BIN` di ArduSub + bag MCAP di `rov_bringup`) ai parametri di
`vehicles/<veicolo>/vehicle.yaml`.

```bash
python -m uw logs pull rov                      # prima: i log sul disco dei dati, indicizzati
python -m uw sysid bluerov2_heavy               # inventario, dati pronti/mancanti, report in output/sysid/
python -m uw sysid bluerov2_heavy --write       # quando ci sarà uno stimatore: scrive solo le chiavi stimate
```

**Stato: scheletro.** `estimate()` non stima ancora niente, quindi `--write` non scrive. Quando lo farà, sostituisce
in `vehicle.yaml` solo le righe delle chiavi stimate, lascia tutto il resto (commenti e provenienze compresi) e
annota ogni valore nuovo con data e log di origine (`write_estimates`, provato in `tests/test_uw_ops.py`).

Ingressi che servono (profilo `light` di `rov_bringup`): PWM ai motori (`rc/out`), IMU grezza, assetto, pressione.
Il `.BIN` ha le stesse grandezze a frequenza più alta; leggerlo richiede un parser dataflash (pymavlink o uno
scritto qui): da decidere quando si scrive lo stimatore.
