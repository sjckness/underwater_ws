# Underwater gas-leak tank in Isaac Sim: FLS (+ camera) simulation

> **Status (2026-10-26):** M0–M3 done (infra and launcher; scene, bubble plume, adaptive scatterers; bubble scattering, sonar standalone, minimal recorder; Isaac integration with the bubble instancer, sonar in the loop, GUI overlay, ROS 2 topics and the host relay; checks in `docs/validation.md`). Next: M4 camera underwater model.
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
| Sonar pose | fixed, static precompute | fixed **or mobile** (full recompute per ping) |
| Beamformer | FFT on u grid + Fresnel correction | same, or **element-level** (64–128 channels, delay-and-sum) |
| Plume flow | integral plume model | integral, or **LBM in Warp** (2-way coupled, 1 cm cells) |
| Bubble pool | 150 k | 1–2 M (+ coalescence/breakup, later) |
| Camera | RTX + Warp post-process, optional | RTX path tracing with volumetric medium |
| Ping budget | ≤100 ms static; mobile/element-level allowed at ≥1 s/ping (offline datasets) | ≤300 ms mobile |
| Dataset throughput | 1 `sonar-only` process | 3–4 parallel processes, pose cache in RAM |

Interfaces that keep this parametric:
- `FlowField` (integral | LBM) used by `bubbles.py`;
- `Beamformer` (fft_fresnel | element_das) used by `sonar.py`;
- `StaticCache` (precomputed | none) — `none` means recompute every ping, required for a mobile sonar;
- `SensorPose` (fixed | trajectory from config/ROS topic).
On linus, M2–M3 implement only the first option of each; the others are stubs with a clear error until a later milestone (M6).

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
   - **Sonar parameters**: M750d LF mode by default (750 kHz, 130°×20°, 512 beams, ~1°, ~4 mm range resolution); HF mode (1.2 MHz, 70°×12°) switchable. **To verify against the Blueprint datasheet**: angular resolution, beam spacing, minimum range, ping rate, and whether near-field focusing is applied.
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
    - Per ping: raw dB polar (npz), 8-bit polar and fan PNG, per-pixel plume label (bubble energy share above a threshold), leak range/bearing in the sonar frame, metadata (config + seed).
    - Camera: frames + plume mask from instancer semantics.
    - Sweeps randomize seed, leak rate, hole clock angle, cover depth, sonar tilt, noise, speckle dynamics, focus mode and tank preset.

## Decisions (2026-10-26)
- **(a) Entry point:** `python -m uw` (pathlib, no shell logic) replaces `scripts/uw.sh`, which stays as a thin Linux wrapper. Docker on Linux; a venv or native Isaac on Windows. No hard-coded `/isaac-sim` paths: they live in `config/runtime.yaml` (+ an untracked `runtime.local.yaml`).
- **(b) Sensor outputs** go through a `SensorSink` interface (`uw/sinks.py`): backends `file` (datasets), `ros`, `zenoh`; later `oculus_emulator`.
- **(c) Real-time mode** besides sim time: wall-clock pacing with a watchdog that reports pings over budget.
- **(d) Sim and ROS decoupled:** the sim publishes over Zenoh; a relay on linus republishes to ROS 2. ROS is optional on the sim side.
- **(e) Hardware in the loop:** milestones M7 (open loop) and M8 (closed loop) below.

## Milestones (each ends runnable and checked; I ask before each commit)
- **M0 Infra**: compose file, `scripts/uw.sh` (incl. `sonar-only` stub), README, `.gitignore` (output/, third_party/), pytest skeleton.
- **M1 Scene + plume standalone**: `tools/plume_preview.py` writes side and top scatter plots plus stats (flow, bubbles/s, rise time, centreline speed/width), both pipe presets. Plume checked against MTT similarity **and** published bubble-plume / blowout correlations (e.g. Socolofsky & Adams, Bombardelli et al.: centreline velocity, spreading rate, slip-velocity ratio). Adaptive scatterer spacing implemented and cached.
- **M2 Sonar standalone**: `scattering.py` + `sonar.py`; `tools/sonar_preview.py` writes polar and fan PNGs of the static tank and tank with plume, both focus modes; timing per ping and VRAM. **Real reference goal**: obtain at least one real Oculus (ideally M750d) tank recording (Blueprint sample data, public FLS datasets, or other Oculus users) and compare with `tools/compare_real.py`: sand/background intensity histogram (Rayleigh/K-distribution fit), sidelobe level next to a bright target, blind zone and ring-down shape, TVG profile. Then enable the `basin` preset.
- **M3 Isaac integration**: USD stage from the geometry, lamps, bubble instancer, sonar in the loop, ROS topics plus host relay; GUI check. `sonar-only` mode feature-complete.
- **M4 Camera underwater model**.
- **M5 Dataset recorder + sweeps; buried-pipe sediment penetration**.
- **M6 High-fidelity backends (developed on linus at small scale, run full-size on a workstation)**: mobile sonar (`StaticCache: none` + `SensorPose: trajectory`), element-level beamformer, LBM flow field, bigger bubble pool. Each checked on linus with a reduced scene, and against the default backend on the same scene (same echo level and bearing within tolerance).
- **M7 HIL open loop**: a Jetson on Ethernet receives sonar and compressed camera (Zenoh), runs leak detection; end-to-end latency measured (sim stamp → detection).
- **M8 HIL closed loop**: ROV dynamics model, commands from the Jetson, mobile sonar. Needs M6 (mobile sonar) and the workstation for real time.

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

## For the next session
- Continue at M4. `docs/validation.md` records what M1–M3 checked, calibrated and measured.
- Run everything through `python -m uw` (or `scripts/uw.sh`), e.g. `python -m uw test`, `python -m uw py tools/x.py`. In the container the image has numpy 1.26, scipy, torch 2.7 cu128, cv2, matplotlib and trimesh; pytest is installed into `~/.cache/uw_pydeps` on first use. The host has pytest and numpy.
- `~/.cache/isaac-sim/cache` is root-owned, so the Warp kernel cache goes to `~/.cache/uw_warp`.
- Run no ROS process on the side while the f1tenth harness batches run (see the f1tenth CLAUDE.md). The tank sim uses its own ROS_DOMAIN_ID.

## Known gaps (planned fixes; milestones proposed)
- ~~**Coherent specular reflection from smooth surfaces (pipe, walls).**~~ Done in M2.2: Kirchhoff integral with
  deterministic phase over small triangles of the smooth materials, scaled by the Rayleigh factor of their rms
  roughness; checked against the analytic echo of a flat plate (docs/validation.md).
- ~~**Wall roughness and absorption.**~~ Done in M2.2: per-wall rms roughness (Rayleigh loss of the mirror
  reflection) and an absorbing lining (reflection coefficient in config).
- **Surface backscatter in the plume boil.** Now the water surface is only a mirror (calm 0.2 mm rms; the boil's
  5 mm rms only weakens the coherent reflection) and scatters nothing back. Fix: scatterers on the surface inside the
  boil disc, with a roughness-dependent backscatter (Kirchhoff / small-slope, the boil's rms height and slope) updated
  every ping from the bubbles reaching the surface, plus the bubble layer just below it. M5.
- **Bubble bistatic multipath.** Now the bubbles' image paths (surface only) use the backscattering cross-section
  for bistatic angles too, and their wall paths are skipped. Fix: tabulate the gas sphere's scattering amplitude
  against ka and scattering angle (modal series, 2-D table), use it on the TX-via-mirror and RX-via-mirror paths, and
  add the wall paths (about 4x the bubble paths). M5, or M6 if it breaks the ping budget on linus.
- **Real time with the camera on linus.** Headless with the RTX camera and the 62 k rendered bubbles the sim runs at
  0.75 x real time (without the camera 1.0), with the GUI at 0.47 (docs/validation.md, M3). Per 100 ms of sim time:
  bubble substeps 25 ms of Python/launch overhead (each substep uploads small arrays from pageable memory, which
  serialises CPU and GPU), ping 53 ms, RTX render 30 ms (of which about 20 ms is the instancer's per-frame rebuild),
  camera read 7–10 ms. Candidate fixes: recorded Warp launches and pinned, double-buffered uploads for the substep;
  pipelining the ping's download behind the next substeps; asynchronous rendering with the camera stamped one frame
  late. Not needed on the workstation profile.
- **Sidelobe rings from strong near-normal echoes.** With the fan vertical (roll 90°), the sand straight below and the
  plume column in the fan plane leak through the array's sidelobes into every beam at their ranges: concentric rings
  across the fan (docs/validation.md, M3 addendum). The beamformer matches Taylor theory to 0.1 dB, so this is the
  configured array (−28 dB Taylor, UNVERIFIED), not a numerical artifact. Two inputs to check against a real
  recording: the M750d's actual sidelobe level (−40 dB removes most of the rings), and the sand's backscatter near
  normal incidence (Kirchhoff with rms slope 0.25 gives about −1 dB at normal incidence, likely 10 dB more than real
  sand).
- **Plume labels include the sidelobe leakage.** A pixel is plume when the bubbles carry most of its energy, so the
  leakage arcs of a strong plume echo are labelled too (with the fan vertical, an annulus across all beams). Right for
  "what the bubbles cause in the image"; a detector trained on it learns the arcs as plume. Option for M5: a second,
  geometric label (pixels whose resolution cell contains bubbles).
- **Plume intermittency.** The plume echo is steadier than real seep footage suggests (energy ±0.4 dB from ping to
  ping): constant flow, slow meander and small-scale OU turbulence only, no large eddies or bubble clusters. To check
  against a real recording (M2 goal) before tuning; candidate fix with the LBM flow field (M6).

## Open questions
- Does the M750d apply near-field (dynamic) focusing? Exact LF/HF angular resolution, beam spacing, minimum range, ring-down. Ask Blueprint support or check the SDK/datasheet.
- Source for a real Oculus tank recording (M2): Blueprint sample data, public FLS datasets, other Oculus users.
- Is there a physical lab tank we want to match? If yes, set the `tank` preset to its exact dimensions and wall material.

## Current state / assumptions to confirm later
- M0–M2.2 committed (git log). OceanSim cloned under `third_party/` for reference only.
- Assumed values, all in config: concrete tank walls; seawater at 15 °C; leak of 3 bar g methane through a 1 mm hole; source level 206 dB and noise level 52 dB; sonar on a wall pole at 1.05 m height, pitched 15° down, looking at the pipe; camera just below it; two LED lamps of about 4000 cd each (USD sphere lights, 45° cones), no ambient light.
- Not modelled: bubble coalescence and breakup, dissolution, tank recirculation, elastic echoes of the pipe, Doppler within a ping.