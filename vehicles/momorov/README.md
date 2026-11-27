# momoROV — segnaposto

Il prototipo che cambia forma. Questa cartella è vuota di codice perché il repo `sjckness/momoROV` esiste ma è
**vuoto** (nessun commit, nessun branch, verificato il 2026-11-27): `git subtree add` non ha niente da importare.
Quando il codice sarà lì:

```bash
# su linus, da un branch: conserva la storia del repo separato
git subtree add --prefix=vehicles/momorov git@github.com:sjckness/momoROV.git main
```

e poi il repo separato si archivia (GitHub → Settings → Archive), con un README che rimanda qui.
Il codice di `rov_work` sulla macchina di work **non** va toccato da qui.

## Quello che si sa

- **Due forme**: *hover* = esagono (manovra, stazionamento), *survey* = rettangolo (avanzamento efficiente).
  Il passaggio tra le due (morphing) cambia massa aggiunta, smorzamento e allocazione dei thruster: un
  `vehicle.yaml` per forma (`hover.yaml`, `survey.yaml`, con lo schema di `vehicles/common/schema.py`) più la
  descrizione della transizione.
- **Dinamica propria**: `seamorph_core`. In simulazione PhysX serve solo per i contatti, la dinamica idrodinamica
  la calcola `seamorph_core` (non `vehicles/common/dynamics.py`, che è il Fossen del BlueROV2).
- **Sensori**: DVL Water Linked A50 (lo stesso modello emulato in `vehicles/common/a50.py` e `dvl.py`).

## Da fare quando arriva il codice

- [ ] `git subtree add` come sopra; archiviare `sjckness/momoROV`.
- [ ] `hover.yaml` / `survey.yaml` validi per lo schema (valori `null` + `# TODO` dove non si sanno).
- [ ] Decidere se `seamorph_core` diventa un pacchetto Python importabile (`vehicles.momorov`) o resta un
      sottoprogetto con il suo build.
