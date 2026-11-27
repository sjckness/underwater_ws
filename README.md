# underwater_ws

A simulated underwater test tank in Isaac Sim 5.1: sand, rocks and a gas pipe with a leak, a physically based
forward-looking sonar (Oculus M750d) modelled in NVIDIA Warp, and an optional underwater camera. It outputs
live ROS 2 topics and labelled datasets. Plan and milestones: [docs/PLAN.md](docs/PLAN.md).

## Running

The entry point is `python -m uw` (from the workspace; `scripts/uw.sh` is the same thing as a Linux wrapper). On
Linux it runs everything in the Isaac Sim container (image `isaac_ws_devcontainer-isaac-sim`, built for
`~/isaac_ws`; this repo never builds or pulls it):

| Command | What it does |
|---|---|
| `python -m uw py tools/x.py [args]` | runs a script with Isaac's Python 3.11, Warp 1.8.2, torch, numpy; `py -m mod` and `py -c code` work too |
| `python -m uw test [pytest args]` | pytest on `tests/` in the container, Warp tests on the GPU |
| `python -m uw sim [--gui]` | the Isaac app: USD stage, bubbles in the RTX scene, sonar in the loop, RTX camera, ROS 2 topics (see below) |
| `python -m uw sonar-only --pings N --out output/datasets/NAME` | Warp-only bubbles + sonar without RTX: writes a dataset (`--sink ros`: the sonar topics instead) |
| `python -m uw pool [--gui] [--view chase]` | M11 test pool: a BlueROV2 Heavy flown through ArduSub SITL, BlueOS and Cockpit with a gamepad; the onboard camera as a live H.264 stream ([docs/pool.md](docs/pool.md)) |

How commands run is set in `config/runtime.yaml`: `mode: docker` (Linux) or `native` (a local Python, e.g. a
Windows venv with `warp-lang`, or a native Isaac install), and the install paths. Override per machine in
`config/runtime.local.yaml` (not in git), or with `UW_RUNTIME=native` and `UW_PYTHON=/path/to/python`.

On the host, `pytest` runs the same tests without Warp (pure Python/numpy); the Warp tests are skipped there.
`python -m uw test --isaac` also runs `tests/isaac/` (starts the Kit app, about a minute): the in-Isaac sonar gives
bit-identical pings to `sonar-only` for the same config and seed.

The tests that need ROS 2 (`ros/dvl_relay`, the host-side ROS message tests) run on the host with `pytest tests`, which has ROS 2 Jazzy and `marine_acoustic_msgs`; `python -m uw test` in the container skips them, because its rclpy lacks that package.

Tools (run with `python -m uw py`): `tools/plume_preview.py` (bubble plume, both pipe presets), `tools/sonar_preview.py` (sonar images, static and with the plume, focused and unfocused, timing; `--video N`: an H.264 MP4 of N pings with axes in metres, the pose, the outlines of the plume labels and a side view of the tank), `tools/compare_real.py` (speckle, sidelobes, blind zone, TVG profile of a recording). `tools/dvl_dr_offline.py` runs the DVL and its dead reckoning against the truth with no Isaac, no ROS and no GPU, and `tools/dvl_drift.py` (host, ROS 2 sourced) does the same on the live chain, recording the estimate and the truth on their two domains; both report the drift as a percentage of distance travelled. On the host: `python3 tools/gpu_mem.py OUT.json` samples the peak GPU memory of the running `uw` processes (nvidia-smi) until Ctrl-C. What the models were checked against, and where they disagree: [docs/validation.md](docs/validation.md).

The first `python -m uw test` installs pytest 7.4.4 (the host's version) into `~/.cache/uw_pydeps`, which needs network
once; the image has no pytest.

Scripts must live inside the workspace (the only directory mounted in the container, at the same path as on the
host). Run from inside the workspace and relative paths in the arguments resolve as they do on the host.

### Config

`config/tank.yaml` holds every scene and sensor parameter. Options after the command add layers, in this order:

1. `--profile linus|workstation` (default `linus`): hardware profile, `config/profiles/`. Fidelity knobs only:
   scatterer spacing mode, bubble pool, backends, ping budget, dataset processes.
2. `--preset NAME`, repeatable, at most one per group (`config/presets/<group>/`): pipe `exposed` (default) |
   `buried`; size `tank` (default) | `basin`.
3. `--set a.b=c`, repeatable: one value, parsed as YAML. The key must exist, so a typo fails instead of being
   ignored.

```bash
python -m uw py tools/plume_preview.py --preset buried --set leak.pipe_pressure_bar=5
python -m uw test --profile workstation --preset basin     # the tests' `cfg` fixture follows these options
```

### Stopping

Ctrl-C stops the run: Python gets a `KeyboardInterrupt`, so `finally` blocks run and pytest prints its summary.
SIGTERM or SIGHUP to `python -m uw` does the same. If Python does not exit within 15 s, the container is killed; without
a terminal (e.g. piped output), a second Ctrl-C kills it at once. The container (`uw-<command>-<pid>`) is removed
in every case; `docker ps -a --filter name=uw-` should list nothing after a run.

## Sensor outputs

Sensors hand their messages to a `SensorSink` (`uw/sinks.py`): `file` writes a dataset (npz + PNG + JSON per
message, `manifest.json`); `ros` publishes live (`uw/ros_bridge.py`); `none` drops them; `zenoh` is M7.

### The Isaac app (`python -m uw sim`)

```bash
python -m uw sim                                   # headless, ROS 2 topics, real time (sim.realtime)
python -m uw sim --gui                             # + Isaac GUI with the sonar overlay
python -m uw sim --gui --set lights.ambient_dome=300 --set sim.bubble_material=inspect   # see the whole tank
python -m uw sim --sink file --out output/datasets/NAME --duration 60 --no-realtime      # camera + sonar dataset
```

A moving sonar (M3.5): `--set sonar.pose_source=trajectory --set sonar.trajectory.preset=approach|lawnmower|orbit|hover
--set sonar.static_cache=none` (or `on_motion`); `pose_source=ros` takes the pose from `sonar.ros_pose_topic`
(PoseStamped in the tank frame). The same options work for `sonar-only` and `tools/sonar_preview.py`. Each ping's pose
is in its JSON and on `/tf`.

Options: `--duration S` (sim seconds), `--no-realtime`, `--no-camera`, `--view side45|top|sonar` (GUI start view),
`--screenshot T` (repeatable: at sim time T, the GUI window, its viewport and the camera image go to
`output/sim/screenshots/`), `--stats-file F` (per-stage timing, JSON), plus `--profile/--preset/--set`.

- One tick is one bubble substep (1/240 s); the sonar pings every 1/`sonar.ping_rate_hz`, the RTX camera renders
  every 1/`camera.rate_hz`, the GUI viewport every 1/`sim.gui_fps`. Sim time is the time since the leak started
  (the plume warm-up runs first) and goes out on `/clock`.
- Real time: the loop sleeps when ahead of the wall clock. The watchdog logs pings over
  `sonar.ping_budget_ms.static` and sim time more than `sim.max_lag_s` behind; it never skips work, so a run that
  cannot keep up just runs slower (the real-time factor is logged every 10 s).
- The USD stage is built from the same numpy geometry as the sonar, and every start checks that the occluders' USD
  meshes and the sonar's Warp BVH mesh have identical vertices and triangles.
- GUI overlay (omni.ui.scene, never in the camera images): the wireframe of the sonar's field of view and the current
  sonar image as points on the fan's centre plane (`sim.overlay_*`).
- The scene is lit by the two LED lamps only (deep water): outside their cones the GUI shows black. For looking around,
  add a dome (`--set lights.ambient_dome=300`) and draw the bubbles opaque (`--set sim.bubble_material=inspect`);
  both change the camera images too.

### ROS 2 topics

From inside the container, through the Isaac ROS bridge's internal Jazzy (`uw/ros_bridge.py`; no host ROS needed):

| Topic | Type | Rate |
|---|---|---|
| `/oculus/polar` | `sensor_msgs/Image` mono8, rows = range bins (row 0 = range 0), columns = beams (port to starboard) | ping rate |
| `/oculus/polar_db` | `sensor_msgs/Image` 32FC1, same layout, dB re 1 µPa, NaN in the blind zone | ping rate |
| `/oculus/fan` | `sensor_msgs/Image` mono8, Cartesian fan (`ros.fan_width` px) | ping rate |
| `/oculus/meta` | `std_msgs/String` JSON, latched: ranges, bearings, frequency, sound speed, beam widths, pose | once |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image` rgb8, `CameraInfo` (frame `camera_optical`) | `camera.rate_hz` |
| `/sim/dvl/report`, `/sim/dvl/meta` | `std_msgs/String` JSON: the Water Linked DVL A50's own velocity report, and the instrument's geometry (latched) | 2-15 Hz, falling with altitude |
| `/sim/leak`, `/sim/leak/flow_lpm` | `geometry_msgs/PoseStamped` (the hole, x along its normal), `std_msgs/Float32`; latched | `ros.leak_hz` |
| `/sim/plume` | `visualization_msgs/MarkerArray`: bubbles as one SPHERE_LIST, at most `ros.plume_markers` | `ros.plume_hz` |
| `/tf_static` | `tank` → `oculus` → `oculus_projected`, `tank` → `camera` → `camera_optical` | latched |
| `/clock` | `rosgraph_msgs/Clock`, sim time | `ros.clock_hz` |

`ros/dvl_relay` (same host ROS 2) stands in for the real Water Linked driver: it turns `/sim/dvl/report` into
`marine_acoustic_msgs/Dvl`, a REP-103 `TwistWithCovarianceStamped` and a `sensor_msgs/Range` under
`/rov1/sensors/dvl/`, so the vehicle and the simulator present the same interface. `ros/dvl_dr` then dead-reckons
from it with the autopilot's IMU and baro, and `tools/dvl_drift.py` scores the drift against `/gt`. The sensor
model, the measured hardware figures and what is still unverified: [docs/dvl.md](docs/dvl.md).

`ros/sonar_relay` (host ROS 2 Jazzy with `ros-jazzy-marine-acoustic-msgs`) republishes `/oculus/polar` +
`/oculus/meta` as `marine_acoustic_msgs/ProjectedSonarImage` on `/oculus/sonar_image`, the message of the Oculus ROS
drivers (`--db`: the float dB image instead). Run it without building: `ros/sonar_relay/run.sh`; or build it with
colcon (`ros2 run sonar_relay relay`).

## The test pool (`python -m uw pool`, M11)

A 10 x 10 x 5 m tiled test tank with a BlueROV2 Heavy (Blue Robotics' own model, fetched once by
`python -m uw py tools/fetch_bluerov2.py`), piloted exactly like the real vehicle: ArduSub SITL 4.5.7 runs the
autopilot, BlueOS (1.5 beta, "Manual" board) and Cockpit are the operator interface, a Bluetooth gamepad flies it.
Camera, vehicle dynamics and a DVL: no bubbles, no sonar. Operator guide, ports and measurements: [docs/pool.md](docs/pool.md).

- **Physics** (`uw/rov/physics_proc.py`): the Fossen model of `uw/rov/dynamics.py` (von Benzon 2022, the Heavy's
  measured set) in its own process, lockstep with SITL over the JSON backend (UDP 9002) and paced to the wall
  clock, with partial-submersion buoyancy at the surface. The render loop reads pose, camera tilt and lights from
  shared memory.
- **Camera** (`uw/pool.py`, `uw/camera_uw.py`, `uw/video.py`): RTX render, then the water as a per-pixel
  attenuation and veiling light from the distance annotator, then H.264 (constrained baseline) over RTSP to mediamtx,
  which BlueOS pulls as a "Redirect" stream for Cockpit. linus: 30 fps at 1280 x 720, 7 ms a frame, 2.4 GB VRAM.
- **Stack** (`docker/pool/compose.yaml`): `sitl` and `mediamtx` next to the sim; `blueos` (profile) on the host
  network in WSL2 on the Work machine, or `blueos-local` (profile) on its own bridge at 172.31.250.2 where ports 80
  and 8000 are taken (linus), flown from the Cockpit desktop app. `tools/pool_blueos_setup.py` configures BlueOS;
  `tools/pool_smoke.py` flies the vehicle over MAVLink and checks every axis, ALT_HOLD, lights and tilt.
- **DVL** (`uw/rov/a50.py`, `uw/pool_geometry.py`): the A50 model of `uw/rov/dvl.py` cast at the pool, served as the
  device itself (Water Linked's JSON protocol on TCP 16171 and its HTTP API) from the physics process, so BlueOS's
  unmodified Water Linked DVL extension feeds ArduSub VISION_POSITION_DELTA: POSHOLD works as on the real vehicle.
- Config: `config/pool/pool.yaml` with its own profiles (`config/pool/profiles/`), `--set` as for the tank.
- **YOLO on the camera** (`ros/yolo_detector`, host ROS 2 + a Python with Ultralytics): reads the same H.264
  stream BlueOS relays (`rtsp://127.0.0.1:8654/rov`), runs an Ultralytics model and publishes
  `vision_msgs/Detection2DArray` on `/rov1/perception/camera/detections` and an annotated
  `/rov1/perception/camera/overlay`; `--video-out rtsp://127.0.0.1:8654/rov_yolo` puts the overlay in mediamtx
  (browser: `http://localhost:8889/rov_yolo`). On the real BlueROV2 only the source URL changes. Setup and options:
  docs/pool.md, "Riconoscimento con YOLO".

## Datasets (sweeps)

`tools/sweep.py` generates a labelled sonar dataset from a sweep file (`config/sweeps/v0.1.yaml`: every randomised
parameter and its distribution) with `sonar-only` workers in the container:

```bash
python3 tools/sweep.py run                    # sample (once), generate, resume where it stopped, write manifest + card
python3 tools/sweep.py status                 # progress
touch /HDD2T/underwater_data/v0.1/PAUSE       # pause after the current scenarios; rm to go on
python3 tools/dataset_stats.py /HDD2T/underwater_data/v0.1     # stats/report.md, overview.png, examples.png
```

It runs on the host (plain Python) and keeps `workers` containers busy. Each scenario is a directory
`<root>/<id>/ping_xxxxx.{npz,png,json}` + `_mask_geom/_mask_direct/_mask_all.png`; `<root>/manifest.json` lists them
with their split (train/val/test by scenario, stratified by kind, pipe preset and flow decade) and `<root>/README.md`
is the dataset card. It stops its workers (freeing the GPU) while an f1tenth run is active and while / has less than
27 GB free (the f1tenth Isaac precheck needs 25 GB); the data go to `data_root` (config/runtime.yaml, the HDD on
linus). Training: `uw.dataset.SonarDataset(root, split)` (PyTorch: image, the three masks, the energy shares, scalar
targets such as the flow).

## Where things go

- `output/`: everything the tools write; not in git. `output/plume_preview/<preset>/`, `output/sonar_preview/<preset>/`:
  previews; `output/datasets/<name>/`: `sonar-only` datasets (per ping: npz with raw dB and the plume shares,
  polar PNG, three plume-label PNGs (`mask_all`, `mask_direct`: the bubbles' share of the echo energy; `mask_geom`: main-lobe cells of visible bubbles; described in the manifest), fan PNG if `recorder.fan_png`, JSON; `manifest.json`; about 1 MB per ping); `output/cache/`: acoustic scatterers, one npz per geometry + spacing (rebuilt automatically).
- `~/.cache/uw_warp`: Warp kernel cache (not `~/.cache/isaac-sim/cache`, which is root-owned); `~/.cache/uw_cuda`: the
  CUDA driver's JIT cache.
- `~/.cache/uw_pydeps`: pytest (and imageio-ffmpeg) for the container.
- `~/.cache/uw_home`: the container's home: Kit's shader, texture and derived-data caches, its config and logs (the
  first `sim` compiles the RTX shaders, later starts reuse them); `kit_cache/` and `kit_data/` are mounted over
  `/isaac-sim/kit/cache` and `/isaac-sim/kit/data`, which Kit writes but the image keeps read-only for us.

The container runs as your user (plus the image's `isaac-sim` group, to read `/isaac-sim`), so all of these
stay owned by you. `sim --gui` also mounts the X11 socket and your X authority file and passes `DISPLAY`.

## ROS 2 domain

The tank sim uses its own `ROS_DOMAIN_ID`, set from `UW_ROS_DOMAIN_ID` (default **73**). It deliberately ignores
a `ROS_DOMAIN_ID` exported in your shell: the f1tenth stack uses 0 (car) and 42 (`~/isaac_ws` sim), and its
batches 85 and 87. To see the topics from a host shell: `ROS_DOMAIN_ID=73 ros2 topic list --no-daemon` (without
`--no-daemon` the CLI leaves a `ros2` daemon running on that domain).

Do not run any ROS process, the `ros2` CLI included, while an f1tenth harness batch is running: its precheck
aborts the batch. `python -m uw py` and `python -m uw test` start no ROS (the message tests build message objects
only); `sim` (default sink `ros`), `sonar-only --sink ros` and `ros/sonar_relay` do.
