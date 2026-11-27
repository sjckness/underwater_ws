# Parametri di ArduSub

Un file per stato dei parametri, mai sovrascritto: `AAAA-MM-GG_<motivo>.params`, per esempio
`2026-12-03_iniziale.params`, `2026-12-10_pid-profondita.params`. Il motivo in minuscolo, con trattini.

## Quando esportare

- la prima volta, prima di cambiare qualsiasi cosa (`…_iniziale.params`);
- prima e dopo ogni modifica fatta a mano (`…_prima-<cosa>.params`, `…_<cosa>.params`);
- dopo ogni aggiornamento di ArduSub (i default cambiano tra versioni);
- a fine immersione se qualcosa è cambiato (checklist post-immersione).

## Come esportare

I parametri li legge e li cambia una persona, mai uno script del repo.

- **QGroundControl**: *Vehicle Setup → Parameters → Tools → Save to file…*.
- **BlueOS** (web UI, `192.168.2.2`): pagina *Autopilot Parameters*, se la tua versione ha l'esportazione su file.
- **Cockpit**: impostazioni del veicolo, se la versione installata permette l'export.

Poi: copia il file qui, aggiungi la riga in `../../VERSIONS.md` (colonna *Parametri*) e committa da linus.
Confronto tra due stati: `diff <(sort a.params) <(sort b.params)`.

## Da ricordare

- **Origine EKF / posizione dell'ufficio** (senza GPS ArduSub non arma finché non ha un'origine). In ArduSub 4.5.7
  i parametri esistono (`*_ORIGIN_LAT`, `*_ORIGIN_LON`, `*_ORIGIN_ALT`; verificato nel binario SITL 4.5.7 — il
  prefisso esatto controllalo nell'export). Scrivi qui le coordinate dell'ufficio usate in Cockpit come
  ORIGIN_LAT / ORIGIN_LON, così ogni export si può verificare:

  | Luogo | ORIGIN_LAT | ORIGIN_LON | Note |
  |---|---|---|---|
  | ufficio | `TODO` | `TODO` | |

- In simulazione la piscina usa `45.0703, 7.6869` (`HOME_LOC` di `deploy/docker/pool/compose.yaml`): non è
  l'ufficio.
- Frame: BlueROV2 Heavy = `FRAME_CONFIG 2` (Vectored 6DOF, 8 motori). `MOTn_DIRECTION` va verificato sul veicolo
  vero: in SITL deve restare 1 (`vehicles/bluerov2_heavy/vehicle.yaml`, commento sui thruster).
