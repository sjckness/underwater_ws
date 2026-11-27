# Target

Una macchina su cui `uw deploy | smoke | logs` lavorano. `<nome>.example` è in git e documenta i valori;
`<nome>.local` (ignorato da git, uno per macchina) viene fuso sopra chiave per chiave, quindi basta che contenga
quello che cambia. `uw deploy rov --discover` scrive da solo in `rov.local` il nome del container dell'estensione e
la cartella persistente trovati sulla Pi.

| Target | Cosa | `kind` |
|---|---|---|
| `rov` | la Pi del BlueROV2 (`192.168.2.2`), estensione ROS 2 di BlueOS | `rov` |
| `rov-sim` | la stessa cosa su linus: BlueOS + SITL della piscina, estensione in `deploy/docker/compose.rov-sim.yaml` | `rov` |
| `thor` | Jetson Thor, HIL; solo `git pull --ff-only` + immagine dello stack | `thor` |
| `work` | Windows 10 + Isaac; solo `git pull --ff-only` (serve un server SSH su Windows) | `work` |
| `linus` | la macchina di sviluppo: nessun deploy, solo dove stanno i dati | — |
