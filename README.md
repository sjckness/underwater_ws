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
| `python -m uw sim [--gui]` | full Isaac app with camera and ROS 2: not implemented until M3 |
| `python -m uw sonar-only --pings N --out output/datasets/NAME` | Warp-only bubbles + sonar without RTX: writes a dataset |

How commands run is set in `config/runtime.yaml`: `mode: docker` (Linux) or `native` (a local Python, e.g. a
Windows venv with `warp-lang`, or a native Isaac install), and the install paths. Override per machine in
`config/runtime.local.yaml` (not in git), or with `UW_RUNTIME=native` and `UW_PYTHON=/path/to/python`.

On the host, `pytest` runs the same tests without Warp (pure Python/numpy); the Warp tests are skipped there.

Tools (run with `python -m uw py`): `tools/plume_preview.py` (bubble plume, both pipe presets), `tools/sonar_preview.py` (sonar images, static and with the plume, focused and unfocused, timing), `tools/compare_real.py` (speckle, sidelobes, blind zone, TVG profile of a recording). What the models were checked against, and where they disagree: [docs/validation.md](docs/validation.md).

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
message, `manifest.json`); `ros` (M3) and `zenoh` (M7) will publish live.

## Where things go

- `output/`: everything the tools write; not in git. `output/plume_preview/<preset>/`, `output/sonar_preview/<preset>/`:
  previews; `output/datasets/<name>/`: `sonar-only` datasets (per ping: npz with raw dB and the plume shares,
  polar PNG, two plume-label PNGs, fan PNG if `recorder.fan_png`, JSON; `manifest.json`; about 1 MB per ping); `output/cache/`: acoustic scatterers, one npz per geometry + spacing (rebuilt automatically).
- `~/.cache/uw_warp`: Warp kernel cache (not `~/.cache/isaac-sim/cache`, which is root-owned).
- `~/.cache/uw_pydeps`: pytest for the container.

The container runs as your user (plus the image's `isaac-sim` group, to read `/isaac-sim`), so all of these
stay owned by you.

## ROS 2 domain

The tank sim uses its own `ROS_DOMAIN_ID`, set from `UW_ROS_DOMAIN_ID` (default **73**). It deliberately ignores
a `ROS_DOMAIN_ID` exported in your shell: the f1tenth stack uses 0 (car) and 42 (`~/isaac_ws` sim), and its
batches 85 and 87. To see the topics from a host shell (from M3): `ROS_DOMAIN_ID=73 ros2 topic list`.

Do not run any ROS process, the `ros2` CLI included, while an f1tenth harness batch is running: its precheck
aborts the batch. `python -m uw py` and `python -m uw test` start no ROS.
