# BlueOS del BlueROV2 reale

Cosa versionare qui (file esportati dalla Pi, mai modificati a mano e ricaricati senza averli letti):

| File | Cosa | Come ottenerlo |
|---|---|---|
| `extensions/ros2.json` | l'estensione ROS 2 come la installa BlueOS (immagine, tag, permessi) | qui sotto; confrontalo con *Extensions → ROS2 → Edit* |
| `cockpit/<AAAA-MM-GG>.json` | profilo di Cockpit (layout, joystick, widget) | Cockpit → impostazioni → esporta |
| `settings/<AAAA-MM-GG>_*.json` | impostazioni esportate di BlueOS (endpoint MAVLink, rete) | pagine di BlueOS che hanno l'export; altrimenti uno screenshot con data |

## Estensione ROS 2

`extensions/ros2.json` riporta la voce dello store delle estensioni di BlueOS per `itskalvik.ros2` alla versione
0.1.0 (la stessa di `deploy/docker/Dockerfile.rov`). Punti da sapere:

- `NetworkMode: host` e `Privileged: true`: il container vede la rete della Pi (MAVLink su `127.0.0.1:5777`, DDS) e
  i dispositivi (`/dev`).
- `/usr/blueos/extensions/ros2/` (host) è `/root/persistent_ws/` (container): l'unica cosa che sopravvive a un
  aggiornamento dell'estensione. Il nostro workspace sta in `/root/persistent_ws/ros2_ws/`.
- Limiti: 2 CPU (`CpuQuota 200000` su `CpuPeriod 100000`) e ~1 GB di memoria. `colcon build` sulla Pi va fatto con
  `--parallel-workers 1` (lo fa `uw deploy rov`).

Per usare un'immagine nostra (`deploy/docker/Dockerfile.rov`) al posto di quella dello store: in pirate mode,
*Extensions → Installed → ROS2 → Edit*, cambiare solo `docker`/`tag` e lasciare uguali i permessi. Non è ancora
stato fatto: per ora `uw deploy rov` usa l'immagine dello store e compila `ros/src` dentro `persistent_ws`.
