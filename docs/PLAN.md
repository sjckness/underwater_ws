# Underwater gas-leak tank in Isaac Sim: FLS (+ camera) simulation

> **Status (2026-10-28):** M0–M3.5 and M5 done (infra and launcher; scene, bubble plume, adaptive scatterers;
> bubble scattering, sonar standalone, minimal recorder; Isaac integration with the bubble instancer, sonar in the
> loop, GUI overlay, ROS 2 topics and the host relay; M3.1 geometric plume label and CUDA graph bubble substeps;
> M3.5 mobile sonar; M5 dataset sweeps with datasets v0.1 and v0.2; checks in `docs/validation.md`). Real-data
> validation is adopted for the two public datasets that exist (`ab7243b`) and blocked on a real Oculus tank
> recording for the rest; M6 is deferred on that. **Current main track: M7, ROS 2 HIL over Ethernet**, whose step 1
> (`docs/ros_hil.md`) also carries the M8a vehicle design. M4 (camera underwater model) is unscheduled behind it.
> Order of work: "Where this session stopped".
> Runs on **linus** for now.

## Changes in this revision (2026-10-26)
1. **Bubble scattering**: at 750 kHz (λ ≈ 2 mm) bubbles of 1–3 mm have ka ≈ 3–10, i.e. the transition regime, not ka≫1. Replace `TS = 20 log(a/2)` and `σ_ext = 2πa²` with a high-frequency fluid-sphere model (Anderson, via the Thuraisingham / Ainslie–Leighton approximations), valid across ka. Resonance (Minnaert, a ≈ 4 µm at 750 kHz) is irrelevant for the size range and is dropped.
2. **Near-field beamforming**: an aperture giving ~1° at 750 kHz is L ≈ 10 cm, so the Fraunhofer distance L²/λ ≈ 5 m. The whole 3 m tank is in the array's near field. The beamformer gets a range-dependent quadratic phase (Fresnel) correction before the FFT, switchable to emulate a sonar *with* or *without* near-field focusing. Whether the M750d focuses is an open question to resolve (datasheet / Blueprint support).
3. **Blind zone and ring-down**: minimum range and ring-down are taken from the datasheet, not tuned by eye; they matter a lot at 1–3 m.
4. **VRAM budget**: a sonar-only dataset mode that does not start RTX rendering. Isaac is used only for the GUI and the camera. The static precompute keeps only the complex result per (range bin × u) in VRAM, not the scatterer points.
5. **Speckle realism**: a small slow temporal perturbation of scatterer phase (sediment micro-motion, turbulence), configurable, so the static speckle is not unrealistically frozen.
6. **Validation**: at least one real reference recording is an explicit M2 goal; the plume is checked against published blowout/bubble-plume correlations, not only MTT similarity.
7. **Tank size**: dimensions become a config parameter with presets; scatterer spacing becomes adaptive to the sonar resolution cell (see "Tank size").
8. **Hardware profiles**: every fidelity knob is a config parameter grouped in hardware profiles (`linus`, `workstation`). Everything is developed and must run on `linus` (8 GB), even slowly; `workstation` (e.g. RTX 4090 24 GB, 500 GB RAM) only raises the knobs, with no separate code paths (see "Hardware profiles").

## Language: Python + NVIDIA Warp, not Rust
Nearly all the compute already runs on the GPU:
- bubble step, sonar ray queries and deposits, extinction grid and camera post-process: Warp kernels (JIT-compiled to CUDA);
- beamformer FFT: torch (in the image).
Python only launches a few dozen kernels per frame. Rust would not make any of that faster. It would also cost us three things:
- **Isaac Sim's API is Python only.** pxr/USD, replicator, `isaacsim.*` and the internal Jazzy rclpy have no usable Rust bindings.
- **Warp's features.** BVH mesh queries (`mesh_query_ray`), zero-copy sharing with Isaac/torch arrays, and standalone Warp (`warp-lang`) for tests.
- **CPU code is not a bottleneck.** Init-time or tiny: scatterer sampling (cached as npz, movable to a Warp kernel), the plume integral (ms), bubble emission (vectorized numpy).

Rust would fit only outside the simulator (host-side `sonar_relay`, downstream leak-detection node). Default: the relay is a small Python node.

## Context
A simulated water tank in Isaac Sim (5.1, Docker image `isaac_ws_devcontainer-isaac-sim`, RTX 5050 8 GB on linus). Sand bottom with rocks and a gas pipe, possibly buried. A pressurized gas leak exits a hole in the pipe. Dark scene, as in deep water. Goal: **maximum realism**, initially without real data, for a **forward-looking sonar (Oculus M750d)** and optionally a camera. Sensors are **fixed in the tank**; the leak is **medium** size. Outputs: **live ROS 2 topics plus labelled datasets**. New repo **`~/underwater_ws`**, with **exposed and buried pipe presets** and **tank size presets**.

Why not OceanSim's sonar (reviewed in `~/underwater_ws/third_party/OceanSim`, BSD-3): camera depth image with intensity = reflectivity·cosθ·e^(−αd), normalized per frame. First hit only, no multipath, no beam pattern or sidelobes, no TVG, opaque bubbles, heuristic noise; its camera model assumes sunlit water. We write physically based models and keep OceanSim only as a reference.

## Tank size
Bigger is not automatically more realistic, and it does not remove the need for the near-field model (real test tanks are small, so near-field effects are part of what we want to reproduce).

| Preset | Size (L×W×D) | Purpose |
|---|---|---|
| `tank` (default) | 3×3×3 m | Matches typical lab test tanks; near-field regime; fastest |
| `basin` | 8×5×4 m | Ranges beyond the ~5 m Fraunhofer distance (far-field check), longer plume rise, less wall clutter |

- Plume: deeper water gives a longer rise and more room for the centreline meander; the jet→plume transition is complete within a few cm anyway (jet length l_M ≈ 2.6 cm for the default leak). At 3 m depth (z/D ≤ 0.7) the plume never reaches the slip-dominated far field, and in `basin` (z/D ≤ 1) neither. The main arguments for `basin` are ranges beyond the Fraunhofer distance and less wall clutter.
- Cost: at fixed 2 mm spacing, `basin` would need ~20 M+ scatterers (too much for 8 GB). So spacing becomes **adaptive**: per surface patch, spacing = max(2 mm, sqrt(R·Δθ·Δr/9)), where the resolution cell at range R is ≈ R·Δθ × Δr (Δr ≈ 4 mm), so each cell keeps about 3 × 3 scatterers (needed for fully developed speckle). Near the sonar it stays 2 mm; far away it grows. Areas are rescaled so the backscattered energy is unchanged.
- Rule: develop and validate on `tank`; add `basin` at M2 once adaptive spacing passes the speckle test.

## Hardware profiles
Selected with `--profile linus|workstation` (default `linus`), merged before presets and `--set` overrides. Target hardware now: **linus**. Design rule: a higher-fidelity option is the same code with bigger parameters or an alternative backend behind the same interface, never a fork. On linus the expensive options must still *run* (slower, smaller), so they stay tested.

| Knob | `linus` (RTX 5050 8 GB) | `workstation` (RTX 4090 24 GB, 500 GB RAM) |
|---|---|---|
| Tank preset | `tank` (basin only with adaptive spacing) | `tank`, `basin`, open seafloor patch |
| Scatterer spacing | adaptive (2 mm near) | fixed 2 mm everywhere |
| Sonar pose | fixed, static precompute; mobile runs (static recompute 150 ms tank, 350 ms basin per ping) | fixed **or mobile** |
| Beamformer | FFT on u grid + Fresnel correction | same, or **element-level** (64–128 channels, delay-and-sum) |
| Plume flow | integral plume model | integral, or **LBM in Warp** (2-way coupled, 1 cm cells) |
| Bubble pool | 150 k | 1–2 M (+ coalescence/breakup, later) |
| Camera | RTX + Warp post-process, optional | RTX path tracing with volumetric medium |
| Ping budget | ≤100 ms static; mobile/element-level allowed at ≥1 s/ping (offline datasets) | ≤300 ms mobile |
| Dataset throughput | 1 `sonar-only` process | 3–4 parallel processes, pose cache in RAM |

Interfaces that keep this parametric:
- `FlowField` (integral | LBM) used by `bubbles.py`;
- `Beamformer` (fft_fresnel | element_das) used by `sonar.py`;
- `StaticCache` (precomputed | on_motion | none) — `none` recomputes every ping, `on_motion` when the pose moved beyond thresholds; a mobile sonar needs one of them (M3.5);
- `SensorPose` (fixed | trajectory | ros): waypoint trajectories and presets (approach, lawnmower, orbit, hover) or a ROS pose topic (M3.5).
FlowField lbm and Beamformer element_das are stubs with a clear error (M6, conditional: see "Real-data validation").

## Architecture
A single geometry source (numpy) feeds two consumers:
- **Isaac/USD stage**: rendering (camera and GUI), ROS bridge, time loop.
- **Warp acoustic model** on the same GPU, Warp 1.8.2 from the Isaac install (path in `config/runtime.yaml`). BVH ray queries against the same meshes. Runs inside Isaac, and standalone (no SimulationApp) for tests, fast iteration and **sonar-only dataset mode**.

```
~/underwater_ws/
  config/tank.yaml          all parameters (+ presets: exposed, buried; tank, basin); --set a.b=c overrides
  config/runtime.yaml       how commands run: docker (Linux) | native (Windows venv / native Isaac); no hard-coded install paths
  docker/compose.yaml       same image, host net, nvidia runtime, mounts ~/underwater_ws, Warp cache ~/.cache/uw_warp
  scripts/uw.sh             thin Linux wrapper: `uw.sh CMD ...` = `python -m uw CMD ...`
  uw/__main__.py launcher.py runner.py   entry point `python -m uw py|test|sim|sonar-only` (+ --profile/--preset/--set)
  uw/config.py  water.py  plume.py  geometry.py  bubbles.py  scattering.py  sonar.py  sinks.py  camera_uw.py  usd_scene.py  ros_bridge.py  recorder.py  main.py  app.py  gui.py  sonar_only.py
  ros/sonar_relay/          host-side Jazzy node: Image+meta -> marine_acoustic_msgs/ProjectedSonarImage
  tools/                    plume_preview.py, sonar_preview.py, compare_real.py, sweep.py
  tests/                    pure-numpy tests on the host (pytest); Warp tests via uw.sh test
```
Reused from `~/isaac_ws` (copied, not imported; that repo is the F1Tenth sim):
- `launch.sh` and `.devcontainer/docker-compose.yaml`: container launch pattern and SIGTERM handling.
- `sim/app.py`: `create_app`, render settings, `_ensure_rclpy_path`.
- `sim/ros_bridge.py`: `_qos`, `_stamp`, `image()`, `camera_info()`, `clock()`.
- `sim/main.py`: `RateTimer`, signal handling, run loop.
- `sim/scene_builder.py`: `_material`, `_lights`.

## Components (physics)
1. **Scene** (`geometry.py`, draft done and tested): tank walls and floor; analytic sand height field (relief, sinuous asymmetric ripples, pockmarks at buried vents, sand mound against an exposed pipe); 14 procedural rocks partly sunk; a 6" pipe with two flanges and saddle supports; leak hole at configurable position and clock angle. Acoustic scatterers on every reachable surface with exact normals, areas and a random phase. **Change:** spacing adaptive to the sonar resolution cell (2 mm near, coarser far; walls ≥ 4 mm); tank dimensions from the preset. Current `tank` result: 4.7 M points, to be cached as npz.
2. **Water/gas** (`water.py`, draft): density, viscosity, surface tension, sound speed (Mackenzie), absorption (Ainslie–McColm, ≈0.23 dB/m at 750 kHz). Choked or subsonic leak from pipe pressure, hole size, discharge coefficient. Default: methane, 3 bar g, 1 mm hole, ≈25 L/min. Outputs: mass flow and jet momentum flux including pressure thrust.
3. **Bubble plume** (`bubbles.py`, draft, untested):
   - Liquid flow: integral top-hat forced bubble plume (entrainment α, buoyancy from gas volume per unit length) gives the curved jet→plume centreline, speed, width.
   - Bubble motion: Gaussian-equivalent flow + entrainment inflow + Ornstein–Uhlenbeck turbulence + travelling centreline meander.
   - Sizes: log-normal, conserving gas volume flux.
   - Behaviour: Fan–Tsuchiya slip velocity; zigzag/helical wobble above 1.8 mm; expansion with pressure; burst at the surface. Shape aspect (Wellek) for rendering and for the scattering model's equivalent radius.
   - Buried preset: no jet, larger bubbles, 3 sediment vents with intermittent OU switching.
   - Implementation: GPU ring buffer of 150 k bubbles, 240 Hz substeps, 12 s warm-up.
4. **Bubble acoustic scattering** (`scattering.py`, new):
   - Regime: at 750 kHz, a = 0.5–3 mm gives ka ≈ 1.5–10 (transition). Model: Stanton (1989) high-pass closed form for a gas-filled sphere (Rayleigh to geometric regime; Thuraisingham / Ainslie–Leighton only cover ka ≲ 1), checked against Anderson's modal series; extinction from a fit to the modal series. Pre-tabulated σ_bs(ka) and σ_ext(ka) as 1-D lookup textures (computed once, on the host, from the modal series for verification).
   - Extinction: σ_ext(ka) from the same model (tends to 2πa² only for ka≫1).
   - Resonance term kept in the formula but negligible here; no special handling.
   - Non-sphericity: equivalent-volume radius; optional aspect-dependent correction later.
5. **FLS model** (`sonar.py`): the core of the realism.
   - **Sonar parameters**: M750d LF mode by default (750 kHz, 130°×20°, 512 beams, 1°, 4 mm range resolution); HF mode (1.2 MHz, 70°×12°, 0.6°, 2.5 mm) switchable; min range 0.1 m, max range 120 / 40 m, up to 40 Hz: VERIFIED (Blueprint "Oculus M Series Datasheet EN"). Still UNVERIFIED: beam spacing, sidelobe level, near-field focusing, vertical aperture shape, ring-down.
   - **Surface scattering**: bistatic Lambert term (μ per material) + Kirchhoff/geometric-optics specular term (RMS slope). Bright glints on pipe and walls, diffuse sand.
   - **Propagation**: two-way spherical spreading and absorption. Vertical TX/RX patterns as sinc apertures (two-way −3 dB at ±10°), with sidelobe leakage. The 130° fan rolls off at the edges.
   - **Multipath**: image-source method for the planar boundaries: water surface (R≈−1 with Rayleigh roughness loss; rough "boil" where the plume reaches it) and the 4 walls (R from material impedance). Combinations of {direct, image} on TX and RX legs, with reflection-point validity checks and shadow rays (Warp `mesh_query_ray`). Produces surface-mirror ghosts and wall/corner returns.
   - **Bubbles**: point scatterers with σ_bs from `scattering.py`. Extinction voxel grid (2 cm, σ_ext from `scattering.py`) attenuates every path through the plume, giving the acoustic shadow behind it.
   - **Signal formation**: coherent complex sum (phase k·L) with a cos-shaped pulse envelope (energy preserving across range bins). Contributions go into a fine u = cosψ·sinφ grid per range bin.
   - **Near-field beamforming (new)**: before the FFT, apply the range-dependent Fresnel phase correction exp(−j·k·x²/(2R)) across the aperture, i.e. the contributions are formed with the exact element-to-point path length (not the plane-wave approximation). Two modes in config:
     - `focus: true` — the receiver focuses at each range bin (dynamic focusing): fasci stretti anche vicino;
     - `focus: false` — far-field steering only: beams broaden and defocus at short range, as an unfocused real array would.
     Then the FFT beamformer with Taylor-tapered array factor (−28 dB) yields sidelobe arcs, off-axis broadening and the linear-array cone effect. Speckle emerges physically.
   - **Background**: statistical volume reverberation (Sv, limited to water in each beam), transducer ring-down (amplitude and decay from datasheet/measurements, not tuned by eye), blind zone below the datasheet minimum range, receiver noise added before beamforming (correlated across beams), then TVG (30 log R + 2αR) and 8-bit log display mapping.
   - **Speckle dynamics (new)**: slow random-walk perturbation of scatterer phases (configurable σ and correlation time; default small, e.g. sediment/turbulence micro-motion) so consecutive pings decorrelate slightly, as in a real tank. Default `0` for unit tests.
   - **Speed**: the sonar is fixed, so static-scene contributions are computed once at start-up and stored as a complex (range bin × u) grid per path combination; scatterer points are then freed from VRAM. Per ping: plume-box paths (extinction), bubbles, speckle perturbation (applied to the cached grid as a phase field), noise, beamforming. Target ≤100 ms per ping.
   - **Buried pipe (phase 2)**: sediment penetration. Snell refraction above the critical angle (~28°) with sand attenuation (~0.5 dB/m/kHz), so a shallow-buried pipe shows weakly.
6. **Bubbles in render**: UsdGeom.PointInstancer of ellipsoids (aspect and tilt from velocity), gas-in-water dielectric material (IOR ≈ 0.75).
7. **Camera** (`camera_uw.py`, optional, last): RTX render lit only by the 2 LED lamps (no dome). Warp post-process from depth, lamp poses and cones:
   - per-channel attenuation along lamp→object→camera paths;
   - single-scattering backscatter integral from the lamp cones (glowing haze);
   - forward-scatter blur;
   - marine-snow particles;
   - low-light sensor noise.
8. **Run modes** (`main.py`, `uw.sh`):
   - `uw.sh sim [--gui]`: full Isaac app (USD stage, camera, ROS). Bubble step 240 Hz substeps, sonar at ping rate (10 Hz), camera 10 Hz. Sim time on `/clock`.
   - `uw.sh sonar-only` (new): Warp-only process, no SimulationApp, no RTX. Bubbles + sonar + recorder + (optional) ROS through host rclpy. This is the default for dataset generation and frees ~4–5 GB of VRAM.
9. **ROS 2**:
   - From inside Isaac (internal Jazzy rclpy, own ROS_DOMAIN_ID): `/oculus/polar` (mono8 range×beam), `/oculus/polar_db` (32FC1), `/oculus/fan` (Cartesian mono8), `/camera/image_raw` + `camera_info`, `/sim/leak` (PoseStamped hole + flow), `/sim/plume` (MarkerArray, decimated), TF, `/clock`.
   - Host relay `ros/sonar_relay` (host Jazzy, `ros-jazzy-marine-acoustic-msgs` 2.1.0) republishes as `ProjectedSonarImage`, the message the Oculus ROS drivers use.
10. **Dataset recorder** (`recorder.py`, `tools/sweep.py`):
    - Per ping: raw dB polar (npz), 8-bit polar and fan PNG, per-pixel plume labels (bubble energy share above a threshold: all paths, direct path; geometric: main-lobe cells of visible bubbles), leak range/bearing in the sonar frame, metadata (config + seed).
    - Camera: frames + plume mask from instancer semantics.
    - Sweeps randomize seed, leak rate, hole clock angle, cover depth, sonar tilt, noise, speckle dynamics, focus mode and tank preset.

## Decisions (2026-10-26)
- **(a) Entry point:** `python -m uw` (pathlib, no shell logic) replaces `scripts/uw.sh`, which stays as a thin Linux wrapper. Docker on Linux; a venv or native Isaac on Windows. No hard-coded `/isaac-sim` paths: they live in `config/runtime.yaml` (+ an untracked `runtime.local.yaml`).
- **(b) Sensor outputs** go through a `SensorSink` interface (`uw/sinks.py`): backends `file` (datasets), `ros`, `zenoh`; later `oculus_emulator`.
- **(c) Real-time mode** besides sim time: wall-clock pacing with a watchdog that reports pings over budget.
- **(d) Sim and ROS decoupled:** the sim publishes over Zenoh; a relay on linus republishes to ROS 2. ROS is optional on the sim side.
- **(e) Hardware in the loop:** milestones M7 (open loop), M8a (the ROV platform) and M8b (closed loop) below.

## Milestones (each ends runnable and checked; I ask before each commit)
- **M0 Infra**: compose file, `scripts/uw.sh` (incl. `sonar-only` stub), README, `.gitignore` (output/, third_party/), pytest skeleton.
- **M1 Scene + plume standalone**: `tools/plume_preview.py` writes side and top scatter plots plus stats (flow, bubbles/s, rise time, centreline speed/width), both pipe presets. Plume checked against MTT similarity **and** published bubble-plume / blowout correlations (e.g. Socolofsky & Adams, Bombardelli et al.: centreline velocity, spreading rate, slip-velocity ratio). Adaptive scatterer spacing implemented and cached.
- **M2 Sonar standalone**: `scattering.py` + `sonar.py`; `tools/sonar_preview.py` writes polar and fan PNGs of the static tank and tank with plume, both focus modes; timing per ping and VRAM. **Real reference goal**: obtain at least one real Oculus (ideally M750d) tank recording (Blueprint sample data, public FLS datasets, or other Oculus users) and compare with `tools/compare_real.py`: sand/background intensity histogram (Rayleigh/K-distribution fit), sidelobe level next to a bright target, blind zone and ring-down shape, TVG profile. Then enable the `basin` preset.
- **M3 Isaac integration**: USD stage from the geometry, lamps, bubble instancer, sonar in the loop, ROS topics plus host relay; GUI check. `sonar-only` mode feature-complete.
- **M3.5 Mobile sonar (kinematic ROV)**: SensorPose trajectory (presets) and ros, StaticCache on_motion / none, pose per ping in the data and on /tf, the rig (sonar, camera, lamps) moving in Isaac. Done.
- **M4 Camera underwater model**.
- **M5 Dataset recorder + sweeps**: surface boil and bistatic bubble paths, leak presets and time profiles, `tools/sweep.py` (resumable, parallel, pauses for f1tenth runs), dataset layout and card, splits by scenario, `uw/dataset.py` loader, `tools/dataset_stats.py`; first generation v0.1 (docs/validation.md). Done. Buried-pipe sediment penetration (the acoustic echo of a pipe under sand) is not done: see Known gaps.
- **M6 High-fidelity backends — DEFERRED (2026-10-26)**: element-level beamformer and LBM flow field. No real gas data is in prospect in the short term, so the gap that would justify the LBM cannot be shown; the integral plume model stays. **Reopened only by** (a) real bubble or plume data showing a plume-dynamics gap the integral model cannot close, or (b) a need for scenarios with currents. The element-level beamformer keeps its own trigger: a beamforming gap against a real recording. `FlowField lbm` and `Beamformer element_das` stay as stubs that raise. If reopened: bigger bubble pool when a flow range needs it, each backend checked on linus with a reduced scene and against the default backend on the same scene (same echo level and bearing within tolerance). (The mobile sonar, first planned here, is M3.5.)
- **M7 HIL open loop — ROS 2 over Ethernet (current main track, 2026-10-26)**: a Jetson on Ethernet receives sonar
  and camera and runs leak detection; end-to-end latency measured (sim stamp → detection). **ROS 2, not Zenoh**:
  the sim already publishes the topics (`uw/ros_bridge.py`) and the host relay already speaks the Oculus drivers'
  message, so the bags match future real Oculus bags. Zenoh stays a `SensorSink` option, unscheduled. Steps:
  1. **Design doc `docs/ros_hil.md` first**: topics, message types and QoS for sonar (raw polar beams *and* the
     Cartesian fan image, with Oculus-like metadata: range, gain, beam angles, sound speed), camera, ROV pose/TF,
     and ground-truth leak info in a **separate namespace the detector never subscribes to**; clock handling (sim
     time vs wall time), target rates, a bandwidth estimate over Ethernet, and how the sim keeps real time or
     degrades gracefully.
  2. Sim-side bridge publishing them, with the ROV pose driven by a command topic (teleop and scripted
     trajectories). `RosPose` (`sonar.pose_source ros`) is the existing half.
  3. rosbag2 profiles for dataset capture, same layout as a future real Oculus bag.
  4. Jetson side: a minimal subscriber that measures latency and drops, plus a Docker image for the Jetson.
  5. Loopback on one machine first, then over Ethernet; report latency, rate and drops.
  Rule for this track: **the sonar and scattering code stays independent of ROS** — the bridge only wraps it.
- **M8a ROV platform in the sim (BlueROV2 / BlueROV2 Heavy)**: a real vehicle to fly instead of a kinematic pose.
  The kinematic trajectories of M3.5 **stay** and remain the way datasets are generated: they are repeatable and
  cheap, and nothing in a dataset needs thruster dynamics. M8a is what M8b commands.
  1. **Isaac model**: the BlueROV2 (6 thrusters, 4 horizontal vectored + 2 vertical) and the Heavy (8 thrusters,
     4 vectored + 4 vertical) as a rigid body with the frame, floats and payload as visual geometry.
  2. **Hydrodynamics as external forces, not a custom integrator**: a Fossen model (added mass, linear and
     quadratic damping, restoring forces from the buoyancy/gravity offset) evaluated per physics step and applied
     to the body as a wrench. Isaac/PhysX integrates it. Thruster thrust curves (per-thruster, with the
     forward/reverse asymmetry of the T200) and a thrust-allocation matrix from the geometry, so the input is a
     body wrench and the output per-thruster PWM.
  3. **Payload as on the real vehicles**: the Oculus on its mount, the camera and the lamps at the places and
     orientations the real payload uses, so a pose from the sim means the same thing as a pose from the hardware.
  4. **Parameters**: mass, volume, inertia, centres of gravity and buoyancy, thrust curves and damping from
     published sources (BlueRobotics specs, the BlueROV2 papers that identify Fossen coefficients), each tagged
     **VERIFIED** (traceable to a datasheet or a paper) or **UNVERIFIED** (assumed), as the sonar table in
     docs/validation.md already does.
  5. **ArduSub SITL** through ArduPilot's external-simulator interface (the JSON/UDP sensor-and-actuator protocol):
     the sim sends IMU, depth and attitude, ArduSub returns the thruster outputs. **ArduSub stays unforked** — it is
     the low level (attitude and depth hold, arming, failsafes), configured only through parameters and Lua
     scripting. Everything above that level is **ROS 2**. Evaluate both bridges before choosing: **MAVROS**
     (mature, Sub-aware) and ArduPilot's **native DDS/microROS** backend (fewer hops, but its Sub support needs
     checking). The result of that evaluation goes in `docs/ros_hil.md`.
- **M8b HIL closed loop**: the Jetson stack commands the sim ROV through ArduSub (guided setpoints or manual
  wrenches), closing the loop sim → sonar/camera → detection → command → sim. Leak-search behaviours on top:
  lawnmower survey, bearing-following approach to a detection, hold-and-inspect at close range. Needs the
  workstation for real time (linus runs it slower than real time, which is fine for everything but latency numbers).
- **M9 Large scenes with localized physics**: today the whole scene carries full physics, which is why the tank and
  the basin are the only sizes. For open-seafloor scenes the cost has to follow the sensor, not the scene.
  - A **lightweight statistical layer everywhere**: sand and far structure as a backscatter field with the right
    first- and second-order statistics (level, Rayleigh/K contrast, correlation length) rather than resolved
    scatterers.
  - **Full physics only in the ROI**: resolved scatterers, coherent facets, bubbles and multipath inside the region
    the sonar can actually see this ping (the fan's slab plus the plume box), moving with the ROV.
  - **Continuity across the boundary is a test, not a hope**: the mean level, the speckle contrast and the
    correlation length must match across the ROI edge within tolerance, checked on a scene whose ROI boundary is
    swept through a uniform sand patch (no visible seam in the image, no step in the statistics).
  - **Scene tiling**: the seafloor and its structure as tiles generated and cached per tile, so a survey can run
    beyond one tank-sized box, with the scatterer and facet caches keyed per tile.
- **M10 Baseline leak detector on v0.2**: the dataset exists, so the detector that justifies it comes next; it is
  also what tells us which labels and which v0.3 changes matter.
  - **Baseline**: single-frame segmentation on `mask_geom` (the geometric label, free of sidelobe leakage) from the
    polar `polar_db` image; a small U-Net is enough.
  - **Main model**: temporal, because a plume is distinguished from a bright static object by how it moves — a
    stack of 4–8 consecutive pings, or a ConvLSTM / temporal transformer over the ping sequence.
  - **Optional head**: regression of the leak's gas flow (the per-ping flow is in the ping JSON), as a second task.
  - **Metrics per split** (train/val/test, and per kind and per size, which `dataset_stats` already reports), plus
    the two error modes this dataset was rebuilt to measure: **false positives on the random (non-aimed) views and
    on the distractor scenarios**.
  - **Deployment**: export for the Jetson and measure its latency, so M7's end-to-end number includes the detector.

**Open items, not yet milestones:**
- A **round (cylindrical) tank preset**, after the diffuse-wall check: a curved wall focuses the specular return and
  rings differently from flat walls, and many real test tanks are cylindrical. Sequenced after the check because the
  check is what tells us how much of the wall return is specular in the first place.
- **2-D sonar profiles for other heads**: Oculus **M1200d** (the head the DeeperSense data was recorded with, so it
  is the one we can compare against) and Tritech **Gemini 720i / 1200ik**. Same model, different parameter sets:
  frequency, beam count and spacing, fan, range and resolution, taper.
- **3-D / multibeam-volume sonar is out of scope.**

## Real-data validation (priority over new physics)
Sources to obtain:
- **DeeperSense / DFKI dataset**: Blueprint Oculus M1200d + stereo camera, tank and lakes. Metadata on Zenodo (record 10373154); the full ROS bags on request.
- **UATD** multibeam forward-looking sonar dataset (arXiv 2212.00352): background, speckle and sidelobe statistics.
- **Gas data**: the authors of "Underwater Gas Leakage Flow Detection and Classification Based on Multibeam FLS" (JMSA 2024), on request. **Not expected in the short term (2026-10-26)**; this is what defers M6.

`tools/compare_real.py` gets a reader for each format as the data arrives. Every mismatch found becomes a config or model change, logged in `docs/validation.md`.

Rule: new physics fidelity is added only when the real-data comparison shows a gap it would close. M6 (LBM flow, element-level beamforming) is **deferred** on that rule, not merely unscheduled: the gas-data source below did not materialise, so the plume-dynamics gap cannot be demonstrated either way. Its triggers are listed under M6.

## Dataset v0.2
From the v0.1 sanity check (docs/validation.md, M5). `config/sweeps/v0.2.yaml`; v0.1's file still samples its 300
scenarios exactly.

**Done** (d11a9fe, "Dataset v0.2 prep"):
- Aim most fixed poses at the leak: `fixed.aim` 0.8, a point up to 0.6 m above the hole at a random place inside the
  fan (`pose.aim`); the rest random pitch and yaw. Pilot: 14/14 aimed views and 1/1 random view in view.
- Vary the leak's range: `sonar.placement` puts the pole on any of the four walls and the leak at a distance from it
  (tank 1–2.5 m, basin 2.5–7 m) via `pipe.x` or the leak's place along the pipe.
- Distractors at 25 %.
- Several sonar poses per scene: `fixed.poses` 3 views of one scene, `pings / poses` each, same wall, static cache
  `on_motion`. The scene and scatterer sampling were 60–90 % of the time and are now shared across the three views.
- Randomise `sonar.receiver_noise_common_db` over −20 to −10 dB in half the scenarios (a stripe at bearing 0, +3 to
  +11 dB at broadside: the span an Oculus M1200d shows, docs/validation.md) until the 16-bit bags fix it.
- Faster writing: `recorder.compress` false and `recorder.png_level` 0 drop zlib from the worker's critical path
  (M5 measured 63 ms of a ~100 ms write in zlib). Disk is the trade; the sweep passes both from the scenario config.

**Generated and analysed** (6893055, 2026-10-26/27; docs/validation.md "Second generation: v0.2"):
- The sweep ran to `/HDD2T/underwater_data/datasets/v0.2`: **400/400 scenarios, 0 failed**, 36 000 pings, 97 min
  wall clock, **201.7 GiB (216.6 GB)**, splits 281/60/59 scenarios (train/val/test).
- `tools/dataset_stats.py` on v0.1 and v0.2, 1000 random pings image-checked each: leak scenarios never in view
  19.1 % → 0.4 %, all 527 aimed views in view, basin leak ranges now p50 4.15 m up to 6.71 m, splits balanced per
  kind and per size with no scenario in two splits, no image flags.
- Size came out **2.5× the 60–90 GB estimated from an 8-ping sample** (6.0 MB per ping, not ~2.4): that sample was
  tank-sized and the basin's longer range axis doubles every array. Estimate from the real range-bin count per size
  preset next time, not from one pilot scenario — and **sample the whole size range**: scenarios run 324 → 1183 MB
  (p50 518), while both this estimate and the recompression timing below looked only at the 324 and 740 MB cells. A
  **time** projection must also price in *verifying* a pass, not just doing it: the recompression encoder was timed
  alone and came out ~8× optimistic (docs/validation.md).

**Open:**
- More *kinds* of distractor: only the count was raised. Other strong point clouds (a fish school, suspended
  sediment, a second pipe) so that a detector cannot key on "anything bright above the sand".
- **Recompression of v0.2 (measured 2026-10-27; running since 2026-10-28 10:20, docs/validation.md).** `polar_db` as
  uint16 at 0.01 dB is **rejected**: deflate makes it 5–7 % *larger* than the float16 in place (quantisation noise in
  the low bits), `polar_db` goes negative so the scheme needs an offset anyway, and float16 is the coarser of the two,
  so it was never a precision win either. 0.1 dB steps or a hi/lo byte split give only ~7 %. What does pay is plain
  **deflate on what the sweep wrote uncompressed**: npz `ZIP_DEFLATED` + PNG `compress_level=6` is lossless and
  bit-identical on a round trip, and projects **216.6 → 62.4 GB (3.47×, 6.02 → 1.73 MB per ping, 154 GB freed)** for
  ~13 h of single-threaded encoding — CPU-bound on the pass's per-scenario verification, not disk-bound at the 35–50
  min first projected (docs/validation.md). Three of the four PNGs per ping are binary masks in uncompressed 8-bit
  planes (29–860×); re-encoding them as 1-bit PNG adds under 1 % on top, so it is not worth the format change. **The
  pass is running** since 2026-10-28 10:20 (single-threaded under `nice`/`ionice`; log
  `/HDD2T/underwater_data/datasets/v0.2_recompress.log`): 3.69× on the first 18 scenarios, 0 failures, ETA around
  midnight. Still open after it: the final size and ratio in a follow-up commit, and dropping the soft shares if
  unused.
- Leak flow driving the plume flow field while it changes (Known gaps).
- Scatterer sampling on the GPU (the other half of "faster setup").
- Calibrate against real data first where it exists (Real-data validation); the UNVERIFIED table in
  docs/validation.md says which parameters the public datasets do and do not constrain.

## Verification
- **Host pytest (pure numpy)**:
  - water properties against reference values (ρ = 1025.9 at 15 °C/35 psu, c ≈ 1506.7 m/s, α(750 kHz) ≈ 0.23 dB/m);
  - choked-flow example;
  - Fan–Tsuchiya against the contaminated-bubble correlation of Grace et al. (1976), in Clift et al. (1978): 1 mm ≈ 0.12, 3 mm ≈ 0.25 m/s (FT: +20 % at 1 mm, −8 % at 3 mm);
  - pure-plume integral against the MTT similarity solution (u ∝ z^−1/3, b ∝ z);
  - bubble scattering: approximation vs Anderson modal series for ka ∈ [0.1, 20] (σ_bs within 1–2 dB on average, ka≫1 limit → a²/4, σ_ext → 2πa²);
  - sand gradient against finite differences (already 1e-9);
  - adaptive spacing: total area and expected backscattered energy conserved within 1 %.
- **Warp tests (`uw.sh test`)**:
  - single point target: right range bin, echo level (SL − 40logR − 2αR + TS) and bearing;
  - beam pattern: −3 dB width ≈1° and first sidelobe ≈ −28 dB **at a far-field range**;
  - near field: at 1 m, `focus: true` keeps the −3 dB width ≈1°; `focus: false` broadens it as predicted by Fresnel theory;
  - sand speckle Rayleigh contrast ≈1 for both spacings (fine and adaptive);
  - surface-mirror ghost at the image range;
  - nothing returned inside the blind zone;
  - gas volume conserved between emitted and alive bubbles;
  - bubbles burst at the surface.
- **Real-data comparison (M2)**: see milestone; differences documented in `docs/validation.md` and turned into config changes, not code hacks.
- **Visual**: compare with published Oculus M750d imagery of tanks and bubble plumes (bright far wall and corners, sidelobe arcs, plume speckle and shadow, surface ghosts), and the camera with deep-sea ROV footage (lamp haze cone, snow).
- **Performance on linus**: ping ≤100 ms; VRAM in `sonar-only` < 3 GB; full sim with camera < 7 GB, measured with `nvidia-smi` during a GUI run.

## Where this session stopped (2026-10-28)
Done and pushed before 2026-10-28 (`origin/main` stood at `6893055`):
- `ab7243b` real-data work adopted and merged (UATD + DeeperSense loaders in `tools/compare_real.py`, which now
  runs without `PYTHONPATH`; `tools/deepersense_index.py` + `docs/deepersense_tank_recordings.md`; one "Real data:
  public sonar datasets" section in `docs/validation.md` with the stripe finding as a subsection); M6 deferred;
  Dataset v0.2 split into done/open; `recorder.compress` / `recorder.png_level`.
- `b8b786f` splits: the by-scenario rule stated and tested, optional held-out domain, `dataset_stats` split report.
- `6893055` the **v0.2 sweep ran and was analysed**: 400/400 scenarios, 0 failed, 36 000 pings, 97 min, 201.7 GiB in
  `/HDD2T/underwater_data/datasets/v0.2`, splits 281/60/59; `tools/dataset_stats.py` on v0.1 and v0.2 (see
  "Dataset v0.2" above and docs/validation.md "Second generation: v0.2").

Committed on 2026-10-27 but left unpushed, pushed on 2026-10-28:
- `9a7965f` those statistics written up in `docs/validation.md`.
- `f84513b` the recompression of v0.2 measured: uint16 rejected, plain deflate projected at 3.47×; details under
  "Dataset v0.2" → Open.
- `2dbacc5` the PLAN revision that deferred M6, made M7 the main track and added M8a/M8b, M9 and M10.

Done on 2026-10-28:
- `KeyError: 'rms_slope'` fixed with tests (`geometry.surface_materials` derives the slope from the Gaussian surface
  whatever the `coherent` flag says), the empty probe dirs deleted, and the **diffuse-scattering check run**: seven
  paired 60-ping runs attribute the tank's 2.0–3.7 background contrast to **sand relief and the specular wall image
  paths**, and clear the coherent glints and the plume entirely (±25° falls to 0.981, the Rayleigh value, once the
  ripples, the undulation and the rocks are removed). docs/validation.md, "What makes the tank's background contrast
  2.0–3.7".
- `tools/recompress.py` committed (verified per scenario through the dataset loader, resumable, CPU-only) and the
  **recompression pass started** at 10:20, still running (below). docs/validation.md and this file corrected where
  they still called the pass unapplied and 35–50 min long.

**Not started, in this order:**
1. **`docs/ros_hil.md`**: M7 step 1 (topics, message types, QoS, clock, rates, bandwidth) **plus the M8a vehicle
   design** (the Isaac/Fossen/thruster model, the ArduSub SITL interface, and the MAVROS vs native-DDS evaluation).
   CPU-only.
2. **M10 baseline detector on v0.2**: propose the training setup, then the baseline and the temporal model. The
   dataset is on disk and needs no GPU-side simulation, so it is the first thing that uses v0.2 for its purpose.
3. **M7 steps 2–5**: the sim-side bridge with the ROV pose on a command topic, rosbag2 profiles, the Jetson
   subscriber and its image, loopback then Ethernet with latency, rate and drop numbers.
4. **M8a** the ROV platform, then **M8b** the closed loop, then **M9** large scenes with localized physics.

Running in the background since 2026-10-28 10:20: the recompression pass over v0.2 (3.69× so far, ~154 GB freed, ~13 h
single-threaded, ETA around midnight; log `/HDD2T/underwater_data/datasets/v0.2_recompress.log`). It needs no
attention beyond a follow-up commit with the final size and ratio. Deferred, pick up when convenient: the remaining
"Dataset v0.2 → Open" items before a v0.3.

Environment notes: `~/underwater_data/real` is a symlink to `/HDD2T/underwater_data/real` (the 6.8 GB of DeeperSense
and UATD data, intact); the GPU is otherwise free; the f1tenth telegram bot holds 2 GB of VRAM permanently.
`/HDD2T` has 1.5 TB free and reads cold at 186 MB/s; the root disk is at 93 % (32 GB free), so nothing large goes
outside `/HDD2T`.

## For the next session
- `docs/validation.md` records what M1–M3 and M5 checked, calibrated and measured.
- Run everything through `python -m uw` (or `scripts/uw.sh`), e.g. `python -m uw test`, `python -m uw py tools/x.py`. In the container the image has numpy 1.26, scipy, torch 2.7 cu128, cv2, matplotlib and trimesh; pytest is installed into `~/.cache/uw_pydeps` on first use. The host has pytest and numpy.
- `~/.cache/isaac-sim/cache` is root-owned, so the Warp kernel cache goes to `~/.cache/uw_warp`.
- Run no ROS process on the side while the f1tenth harness batches run (see the f1tenth CLAUDE.md). The tank sim uses its own ROS_DOMAIN_ID.

## Known gaps (planned fixes; milestones proposed)
- ~~**Coherent specular reflection from smooth surfaces (pipe, walls).**~~ Done in M2.2: Kirchhoff integral with
  deterministic phase over small triangles of the smooth materials, scaled by the Rayleigh factor of their rms
  roughness; checked against the analytic echo of a flat plate (docs/validation.md).
- ~~**Wall roughness and absorption.**~~ Done in M2.2: per-wall rms roughness (Rayleigh loss of the mirror
  reflection) and an absorbing lining (reflection coefficient in config).
- ~~**Surface backscatter in the plume boil.**~~ Done in M5 (uw/surface.py): Kirchhoff backscatter of the patch the
  bursts roughen, from the burst rate per surface cell, random phase per ping. Its parameters (rms slope 0.3,
  saturation rate) are assumed; to check against a real recording of a plume reaching the surface.
- ~~**Bubble bistatic multipath.**~~ Done in M5: the bubbles' image paths over the surface and the walls, with the
  modal-series bistatic cross-section on the paths in and out by different routes (+3 ms per ping).
- **The plume's liquid flow follows the nominal leak flow.** A ramp or an intermittent leak changes the gas emitted,
  not the integral plume's flow field (computed once for the nominal flow): early in a ramp and in an intermittent
  leak's off phases the bubbles ride a stronger flow than the actual gas flux would drive. Fix: recompute or scale
  the integral solution with the flow of the last seconds (cheap), or the LBM flow field.
- **Real time with the camera on linus.** Headless with the RTX camera and the 62 k rendered bubbles the sim runs at
  0.75 x real time with the ROS sink (0.86 unpaced, no sink; without the camera 1.4), with the GUI at 0.47
  (docs/validation.md, M3 and M3.1). The bubble substeps are no longer the limit (CUDA graph: 27 → 5 ms per
  ping period); the GPU is: per camera frame the ping waits about 40 ms for the RTX render's GPU work and the render
  call takes 24 ms more, ping 36 ms. Fewer rendered bubbles (`sim.bubble_stride=2`) or a 5 Hz instancer
  (`sim.bubble_render_hz=5`) do not help, so it is the camera render itself, sharing the GPU with CUDA. Candidates:
  a smaller camera image or fewer RTX features (M4 decides the camera pipeline anyway), asynchronous rendering with
  the camera stamped one frame late, or the workstation profile.
- **The per-ping sonar kernels are not a CUDA graph.** The ping is GPU-bound (plume-box paths 27 of 36 ms) and has
  torch FFTs between its Warp kernels; a graph would save well under 1 ms of launches. Revisit for M7 (latency).
- **The tank's background contrast is 2.0–3.7, real bottom is ~1.0.** Measured out on 2026-10-28
  (docs/validation.md): it is the modelled **sand relief** (ripples, undulation, rocks — removing them takes ±25° to
  the Rayleigh 0.981) plus the **specular wall image paths** in the wall-bearing windows. The coherent glint term
  and the plume are ruled out, so this is scene content, not a defect in the scattering model. What it changes is
  the comparison: `sand.ripple_amp` and `rocks.count` are parameters the background statistics are sensitive to, and
  a like-for-like comparison with the DFKI basin needs a window free of ripples, rocks and walls, or a recording
  from a tank of our geometry.
- **`surface_materials` corner cases.** A per-wall `rms_height_m` is ignored if `tank.wall_material` is a
  slope-only material (sand/rock), and a future explicit `rms_slope` on a Gaussian material would go stale
  under a wall override — unreachable with current configs, guard is one line.
- **Sidelobe rings from strong near-normal echoes.** With the fan vertical (roll 90°), the sand straight below and the
  plume column in the fan plane leak through the array's sidelobes into every beam at their ranges: concentric rings
  across the fan (docs/validation.md, M3 addendum). The beamformer matches Taylor theory to 0.1 dB, so this is the
  configured array (−28 dB Taylor, UNVERIFIED), not a numerical artifact. Two inputs to check against a real
  recording: the M750d's actual sidelobe level (−40 dB removes most of the rings), and the sand's backscatter near
  normal incidence (Kirchhoff with rms slope 0.25 gives about −1 dB at normal incidence, likely 10 dB more than real
  sand).
- ~~**Plume labels include the sidelobe leakage.**~~ The energy labels (mask_all, mask_direct) do by definition;
  done in M3.1: a third, geometric label mask_geom (main-lobe cells of visible bubbles, no echo levels).
- **Motion within a ping and Doppler.** A moving sonar takes one pose per ping (the pose at the ping time): no motion
  during the ~5 ms of a ping's two-way travel, and no Doppler shift or smear of the echoes (at 0.15 m/s the shift
  is 150 Hz at 750 kHz, small against the 188 kHz band).
- **on_motion saves little for a moving sonar.** Static recomputes cost 150 ms (tank) / 350 ms (basin) per ping; on
  a trajectory or with a hover drift of 2 cm the pose moves beyond the default thresholds (5 mm, 0.25°) almost every
  ping. It pays for stop-and-go motion; recomputing only the paths near the scene parts that moved is a possible
  speed-up.
- **A buried pipe is acoustically invisible.** The sonar sees only the sand surface above a buried pipe; the echo of
  the pipe through the sediment (refraction beyond the critical angle, sand attenuation ~0.5 dB/m/kHz) is not
  modelled. Buried-pipe scenarios show the gas at the sand and the pockmarks, not the pipe.
- **Plume intermittency.** The plume echo is steadier than real seep footage suggests (energy ±0.4 dB from ping to
  ping): constant flow, slow meander and small-scale OU turbulence only, no large eddies or bubble clusters. To check
  against a real recording (M2 goal) before tuning; candidate fix with the LBM flow field (M6).

## Open questions
- Does the M750d apply near-field (dynamic) focusing? Beam spacing, sidelobe level (receive taper), vertical aperture shape, ring-down: not on the datasheet; ask Blueprint support or measure. (Frequencies, fans, beam count, angular and range resolution, minimum and maximum range, update rate are VERIFIED against the datasheet.)
- Source for a real Oculus tank recording (M2): Blueprint sample data, public FLS datasets, other Oculus users.
- Is there a physical lab tank we want to match? If yes, set the `tank` preset to its exact dimensions and wall material.

## Current state / assumptions to confirm later
- M0–M2.2 committed (git log). OceanSim cloned under `third_party/` for reference only.
- Assumed values, all in config: concrete tank walls; seawater at 15 °C; leak of 3 bar g methane through a 1 mm hole; source level 206 dB and noise level 52 dB; sonar on a wall pole at 1.05 m height, pitched 15° down, looking at the pipe; camera just below it; two LED lamps of about 4000 cd each (USD sphere lights, 45° cones), no ambient light.
- Not modelled: bubble coalescence and breakup, dissolution, tank recirculation, elastic echoes of the pipe, Doppler within a ping.