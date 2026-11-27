# Versioni del BlueROV2 reale

Una riga per ogni stato del veicolo che è andato in acqua. Si aggiunge una riga **prima** di cambiare qualcosa
(aggiornamento di BlueOS, ArduSub, estensione, SD nuova) e si compila dopo.

| Data | BlueOS | ArduSub (versione, frame) | Estensione ROS 2 (immagine:tag, digest) | Nostro codice (`git` SHA in `persistent_ws`) | SD (modello, immagine di backup) | Parametri | Note |
|---|---|---|---|---|---|---|---|
| AAAA-MM-GG | 1.x.y | 4.x.y, Heavy (FRAME_CONFIG 2) | `itskalvik/blueos-ros2:0.1.0` | `abcdef1` | 32 GB, `sd_AAAA-MM-GG.img.zst` su … | `ardusub/params/AAAA-MM-GG_….params` | |

## Riferimenti fissati nel repo

| Cosa | Valore | Dove è usato |
|---|---|---|
| immagine base dell'estensione | `itskalvik/blueos-ros2:0.1.0` — l'unico tag Jazzy su Docker Hub al 2026-11-27 (amd64 + arm64; indice `sha256:810fb3f0bd310e5c75d8b84873e6afffc06f2b900e8bebc07de82957aef17406`) | `deploy/docker/Dockerfile.rov` |
| container dell'estensione sulla Pi | `extension-itskalvikblueosros2010` (da verificare con `docker ps`) | `deploy/targets/rov.local` |
| ArduSub in simulazione | 4.5.7 | `deploy/docker/ardusub_sitl/` |
| BlueOS in simulazione | `bluerobotics/blueos-core:1.5.0-beta.43` | `deploy/docker/pool/compose.yaml` |

Nota sul tag dell'estensione: il tag git `0.1.0` del repo dell'estensione non coincide esattamente con l'immagine
pubblicata (l'immagine ha già i limiti di CPU/memoria di un commit successivo). Per riprodurre l'immagine si usa il
digest, non il tag.
