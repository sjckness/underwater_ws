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

How commands run is set in `config/runtime.yaml`: `mode: docker` (Linux) or `native` (a local Python, e.g. a
Windows venv with `warp-lang`, or a native Isaac install), and the install paths. Override per machine in
`config/runtime.local.yaml` (not in git), or with `UW_RUNTIME=native` and `UW_PYTHON=/path/to/python`.

On the host, `pytest` runs the same tests without Warp (pure Python/numpy); the Warp tests are skipped there.
`python -m uw test --isaac` also runs `tests/isaac/` (starts the Kit app, about a minute): the in-Isaac sonar gives
bit-identical pings to `sonar-only` for the same config and seed.

Tools (run with `python -m uw py`): `tools/plume_preview.py` (bubble plume, both pipe presets), `tools/sonar_preview.py` (sonar images, static and with the plume, focused and unfocused, timing; `--video N`: an H.264 MP4 of N pings with axes in metres, the pose, the ground-truth plume outline and a side view of the tank), `tools/compare_real.py` (speckle, sidelobes, blind zone, TVG profile of a recording). On the host: `python3 tools/gpu_mem.py OUT.json` samples the peak GPU memory of the running `uw` processes (nvidia-smi) until Ctrl-C. What the models were checked against, and where they disagree: [docs/validation.md](docs/validation.md).

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
| `/sim/leak`, `/sim/leak/flow_lpm` | `geometry_msgs/PoseStamped` (the hole, x along its normal), `std_msgs/Float32`; latched | `ros.leak_hz` |
| `/sim/plume` | `visualization_msgs/MarkerArray`: bubbles as one SPHERE_LIST, at most `ros.plume_markers` | `ros.plume_hz` |
| `/tf_static` | `tank` → `oculus` → `oculus_projected`, `tank` → `camera` → `camera_optical` | latched |
| `/clock` | `rosgraph_msgs/Clock`, sim time | `ros.clock_hz` |

`ros/sonar_relay` (host ROS 2 Jazzy with `ros-jazzy-marine-acoustic-msgs`) republishes `/oculus/polar` +
`/oculus/meta` as `marine_acoustic_msgs/ProjectedSonarImage` on `/oculus/sonar_image`, the message of the Oculus ROS
drivers (`--db`: the float dB image instead). Run it without building: `ros/sonar_relay/run.sh`; or build it with
colcon (`ros2 run sonar_relay relay`).

## Where things go

- `output/`: everything the tools write; not in git. `output/plume_preview/<preset>/`, `output/sonar_preview/<preset>/`:
  previews; `output/datasets/<name>/`: `sonar-only` datasets (per ping: npz with raw dB and the plume shares,
  polar PNG, two plume-label PNGs, fan PNG if `recorder.fan_png`, JSON; `manifest.json`; about 1 MB per ping); `output/cache/`: acoustic scatterers, one npz per geometry + spacing (rebuilt automatically).
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
