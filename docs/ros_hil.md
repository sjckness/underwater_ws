# ROS 2 HIL over Ethernet (M7 step 1) and the vehicle design (M8a)

> **Topic names (2026-10-29):** this document was written against the names the sim published then
> (`/oculus/*`, `/sim/*`). `docs/topic_map.md` is the approved map and the code now publishes it —
> `/rov1/sensors/sonar_fls/{image,meta,sonar_image}`, `/rov1/sensors/camera_front/*`, `/gt/*`, `/sim/*` — so read
> the names below as the analysis that produced it, not as what is on the wire. The decisions, bandwidth
> arithmetic and QoS classes are unchanged.
>
> **Status (2026-10-28):** design only, nothing implemented. M7 step 1 and the M8a vehicle design, as
> `docs/PLAN.md` M7/M8a ask for. Numbers tagged **V** are traceable to a file, a datasheet or upstream source and
> **U** are assumed or inferred, as in `docs/validation.md`. Repo citations are this working tree; ArduPilot and
> MAVROS citations are **master / branch `ros2`, read 2026-10-28** — pin a commit before relying on line numbers.

## 0. Scope: everything is simulated except the Jetson

| runs on **linus** | runs on the **Jetson** |
|---|---|
| Isaac Sim (scene, plume, camera), the sonar model, the ROS 2 bridge `uw/ros_bridge.py` | the leak detector (M10) |
| the BlueROV2 rigid body and the Fossen dynamics (M8a) | state estimation |
| ArduSub SITL, with the JSON/UDP external-simulator link on **loopback** | the mission layer, which sends setpoints to ArduSub |
| `ros/sonar_relay` (the `ProjectedSonarImage` assembly), rosbag2 capture | a latency/drop monitor (M7 step 4) |

There is **no physical ROV, no real sensor hardware and no sim-to-real milestone.** The Jetson is the only real
thing in the loop, and the only reason the link exists is to put the detector on the hardware it will actually run
on, across a real network, with real serialization and real jitter.

Two consequences that shape everything below:

1. **The JSON/UDP link never crosses the network.** Isaac, the dynamics and SITL are all on linus, so the
   external-simulator link is `127.0.0.1` (default `SIM_JSON.h:60`, **V**). This matters because that protocol is
   blocking lockstep with no timeout (§3) — putting it on a wire would be the worst available design choice. The
   Ethernet carries only ROS 2 sensor/GT traffic and MAVLink.
2. **Only two things cross the wire**: ROS 2 sensor topics (linus → Jetson) and MAVLink v2 UDP (Jetson ↔ SITL).
   Everything else stays host-local.

**Which machine the numbers belong to.** `docs/PLAN.md` M8b says the closed loop "needs the workstation for real
time (linus runs it slower than real time, which is fine for everything but latency numbers)". That is true *with
the camera*: linus measures RTF 0.694 headless with camera and ROS sink, and RTF **0.997** headless with the ROS
sink and **no camera** (`output/probe/sim_ros_stats.json`, `output/probe/sim_nocam_stats.json`, **V**). Since a
wall-clock delay appears inside the sim scaled by RTF (§3), the resolution is: **the M8b latency budget is measured
in the no-camera profile on linus at RTF ≈ 1.0**, and the camera profile moves to the workstation. Every latency
number in this project is quoted with its RTF and its machine, or it is meaningless.

**The link is 1 GbE at MTU 1500**, which is what every bandwidth figure below is computed against. linus's
`enp5s0` is 1000 Mb/s at MTU 1500 with software timestamping only — **no PTP hardware clock** (**V**,
measured), which sets the floor on any wall-clock latency measurement (§3.3). The Jetson's model, NIC and
topology (direct cable or switch) are still to be filled in; they do not move the bandwidth arithmetic, but
they decide arm64 package availability and whether a switch sits in the latency path (§7).

## 1. What exists today

The entire ROS surface is two files: `uw/ros_bridge.py` (sim side) and `ros/sonar_relay/sonar_relay/relay.py`
(host side). No MAVROS, ArduSub, Fossen, thruster, odometry, rosbag2 or setpoint code exists anywhere (**V**,
repo-wide grep). Published today, all stamps **sim time**, domain **73**:

| topic | type | rate |
|---|---|---|
| `/oculus/polar` | `sensor_msgs/Image` mono8, frame `oculus`, row 0 = range 0 | 10 Hz |
| `/oculus/polar_db` | `sensor_msgs/Image` 32FC1, dB re 1 µPa, NaN in the blind zone | 10 Hz |
| `/oculus/fan` | `sensor_msgs/Image` mono8, Cartesian fan | 10 Hz |
| `/oculus/meta` | `std_msgs/String` JSON, latched — **carries the true sonar pose** | on change |
| `/camera/image_raw`, `/camera/camera_info` | `Image` rgb8 1280×720, `CameraInfo`, frame `camera_optical` | 10 Hz |
| `/sim/leak`, `/sim/leak/flow_lpm` | `PoseStamped`, `Float32` — **ground truth, latched** | 1 Hz |
| `/sim/plume` | `visualization_msgs/MarkerArray`, 3000 of up to 150 000 bubbles | 2 Hz |
| `/tf`, `/tf_static`, `/clock` | `TFMessage`, `rosgraph_msgs/Clock` | per ping / once / 60 Hz |
| `/uw/sonar_pose` (**in**) | `PoseStamped`, `sonar.pose_source=ros`, polled once per ping | — |

`ros/sonar_relay` subscribes `/oculus/meta` + `/oculus/polar` and republishes
`marine_acoustic_msgs/ProjectedSonarImage` on `/oculus/sonar_image` (**V**). `RosPose` has **no producer** — that
is M7 step 2's job. There is no DDS tuning of any kind, and nothing anywhere declares `use_sim_time` (**V**).

## 2. M7 step 1: topics, types, QoS, metadata, ground truth, bandwidth

### 2.1 What crosses the link, and what does not

The design decision that makes 1 GbE comfortable is **subtraction** — but it matters *which* product is kept.
A real Oculus delivers a **polar ping**: beams × range bins of quantized intensity, after the head's own gain
and TVG chain, 8-bit in 92 of the 95 DeeperSense lab configs (**V**). The Cartesian fan is a viewer product.
So the realistic thing to put on the link is our *Oculus-like polar ping*, and what stays home is the
*calibrated* array that has no counterpart in any head's output.

| topic | crosses? | why |
|---|---|---|
| `/oculus/sonar_image` (`ProjectedSonarImage`, `DTYPE_UINT8`) | **yes** — the detector's only sonar input | matches the bag format; the relay stays on linus so the bag and the Jetson see the same message |
| `/oculus/meta` (latched) | **yes**, once per scenario | the Jetson needs the beam geometry to decode anything |
| `/oculus/polar` mono8 | its **pixels do** — inside `sonar_image` | the raw `Image` topic is redundant, not the data. This is the Oculus-like product (see below) |
| `/oculus/polar_db` 32FC1 | **no** | the **calibrated pre-TVG level**, `10·log10(power)` dB re 1 µPa (`display_kernel`, **V**) — not "polar data" but a physical quantity no head reports, since a real ping is an intensity *after* its gain chain. 64 % of the sonar bytes in every preset (**V**). GT-only: `/gt` and the bag |
| `/oculus/fan` mono8 | no | **derived from the uint8 polar on the sim side** (`Sonar.fan()` takes it as input, **V**), so it is a lossy resampling of what we already send; dropping it also leaves M10's polar-vs-Cartesian choice open |
| `/camera/image_raw` rgb8 | only in the camera profile | 221.2 Mbit/s uncompressed, and no compressed transport exists (`uw/ros_bridge.py:247`). The camera profile is not the latency profile |
| `/clock`, `/tf_static`, `/tf`, setpoints, `/gt/*` scalars | yes | small |

**What the two arrays actually are** (`display_kernel`, **V**). From the same beamformed power, per cell:

- `polar_db` = `10·log10(power)` — the raw absolute level in dB re 1 µPa, **no TVG, no gain, no noise-floor
  reference**, NaN in the blind zone. Physically meaningful and calibrated, which is exactly why no head produces
  it: a real Oculus hands you an amplitude that has already been through its analogue and digital gain.
- `polar` = `clamp((level + 30·log10 R + 2αR − nf_db − lo) / (hi − lo) · 255, 0, 255)` — spreading TVG
  (`tvg_db_per_decade: 30`, area scattering) plus absorption, referenced to the beam noise floor, windowed by
  `display_db: [0, 80]` dB and quantized to 8 bits (**V**, `config/tank.yaml:184-185`). That is structurally an
  Oculus 8-bit ping, and the relay already defaults to it (`DTYPE_UINT8`; `--db` is the float32 opt-in, **V**).

So the correct statement of the decision is: **send the quantized polar ping with its gain and range metadata;
keep the calibrated dB array as ground truth.** Not "no real Oculus produces polar" — it does, and that is the
product we send.

### 2.2 Bandwidth against 1 GbE

Payload at 10 Hz, 512 beams, from the real array shapes (**V**, `uw/sonar.py:850-852` + the size presets);
`ProjectedSonarImage` adds a constant 19 888 B (tank/lf) to 29 188 B (basin/hf) of beam geometry per ping
(`relay.py:47-51`, **V**). Framing overhead ~8 % at MTU 1500 (**U**).

| cell | polar rows | `sonar_image` | + camera | wire (+8 %) | of 1 GbE |
|---|---|---|---|---|---|
| tank / lf (3.5 m, 4 mm) | 876 | 37.5 Mbit/s | 258.7 | 40.5 / 279.4 | 4 % / 28 % |
| tank / hf (3.5 m, 2.5 mm) | 1401 | 59.1 | 280.3 | 63.8 / 302.7 | 6 % / 30 % |
| basin / lf (8 m, 4 mm) | 2001 | 83.8 | 305.0 | 90.5 / 329.4 | 9 % / 33 % |
| basin / hf (8 m, 2.5 mm) | 3201 | 133.4 | 354.6 | 144.1 / 383.0 | 14 % / 38 % |

For contrast, **the configuration this replaces**: publishing `polar` + `polar_db` + `fan` + camera is 447 Mbit/s
at tank/lf and **951.8 Mbit/s at basin/hf** (**V**, computed) — over a practical 1 GbE on the worst cell, before
any `/gt` raster. Dropping `polar_db` is what buys the headroom, and it costs the detector nothing.

**Polar instead of the fan is not always cheaper, and the decision does not rest on bytes.** The fan is
range-independent (its width is fixed at `ros.fan_width: 1024`, its height set by the fan angle), while the
polar image grows with the range-bin count:

| cell | polar | fan | cheaper |
|---|---|---|---|
| tank / lf | 35.9 Mbit/s | 46.4 | polar |
| tank / hf | 57.4 | 73.2 | polar |
| basin / lf | 82.0 | 46.4 | **fan** |
| basin / hf | 131.1 | 73.2 | **fan** |

In the basin the polar ping costs 36–58 Mbit/s more than the fan would — still a small fraction of the link.
We send polar anyway, on three grounds that outlast the byte count: it is what a real driver gives, so the
bags stay comparable; the fan is derived from it, so sending the fan means sending a resampled copy of data
we threw away; and M10 has not chosen between a polar and a Cartesian model, so the link should not choose
for it.

Two cautions. `/oculus/polar` at a long range is not safe at any MTU: the same mono8 topic is 410 Mbit/s at 40 m
and **1229 Mbit/s at 120 m** (**U**, computed) — the device's full range is not a usable HIL configuration. And a
real Oculus returns **far fewer rows than we do**: 383–709 bins at 10–39 mm, derived from the DeeperSense M1200d
bags (**U**, §7), against our 876 at a fixed 4 mm. Modelling that cap would cut the sonar topic to ~29 Mbit/s at
any range and is simultaneously the largest fidelity gap and the largest bandwidth lever we have.

### 2.3 QoS

| class | profile | note |
|---|---|---|
| sonar and camera images | `SensorDataQoS`: KEEP_LAST **5**, **BEST_EFFORT**, VOLATILE | **change from today.** The bridge publishes RELIABLE depth 2 (`ros_bridge.py:211`). Fast DDS's `max_blocking_time` defaults to **100 ms** — exactly the whole `ping_budget_ms.static` (**V**) — so one blocked write eats the entire ping budget. An image topic that blocks the ping loop is worse than a dropped frame |
| `/oculus/meta` | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | already correct; a late-joining Jetson cannot decode without it. All of it must fit one message |
| `/tf` | RELIABLE, VOLATILE, KEEP_LAST **100** | tf2's own broadcaster depth; the bridge uses 10 (compatible, less buffer than tf2 expects) |
| `/tf_static` | RELIABLE, TRANSIENT_LOCAL, depth 1 | a depth-1 transient writer replays only its **last** message, so every static transform must go in one `TFMessage` — `static_transforms()` already does |
| `/clock` | `ClockQoS`: KEEP_LAST 1, BEST_EFFORT | already correct. A dropped tick is never retransmitted, so sim time on the Jetson advances in jumps sized by loss (≤17 ms at 60 Hz). **If AP_DDS is ever used it will not match** — its `/clock` reader is RELIABLE depth 5 (§4) |
| setpoints, pose, `/gt` scalars | default: RELIABLE, VOLATILE, depth 10 | small; a dropped setpoint is a control fault |
| `/gt` per-ping rasters | BEST_EFFORT | recorded, not controlled on |

Also required for a link that leaves the host, none of which exists today (**V**): the same `ROS_DOMAIN_ID=73`,
the same `RMW_IMPLEMENTATION` and the same `FASTRTPS_DEFAULT_PROFILES_FILE` on linus (inside and outside the
container) and on the Jetson; `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (already exported on linus — `LOCALHOST`
would silently break the link); and pinned sysctls in `/etc/sysctl.d/` on both hosts, because linus's current
`net.core.rmem_max=4194304` is set by nothing on disk and may not survive a reboot (**V**, measured). For large
samples Fast DDS's documented remedy is `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`, which adds TCP for user data
while discovery stays on UDP; note it **fails participant creation** if combined with any other builtin-transport
selection. `RosSink` should also pass `domain_id` explicitly as `relay.py:85` does — today it only reads the env
var and logs a default of `0`, which is an f1tenth domain (**V**).

### 2.4 Oculus-style sonar metadata

The bag-compatibility target is **`marine_acoustic_msgs/ProjectedSonarImage`** (2.1.0, installed), because that is
what `ros/sonar_relay` already emits and what the newer APL-lineage drivers publish. State the limits plainly:

- That message carries **no ping id, no gain, no master mode, no temperature, no pressure and no row count**
  (rows are implicit in `len(data)/(beam_count·itemsize)`) (**V**). `PingInfo` has four fields only.
- The real DeeperSense bags in hand are the **ROS 1 generation**, `acoustic_msgs/SonarImage` plus exactly one
  `dynamic_reconfigure/Config` per stream carrying all 16 Oculus settings (**V**). So `docs/PLAN.md`'s "the bags
  match future real Oculus bags" holds for the *newer driver generation*, not for the bags we have.
- `relay.py` puts the full `fov_v_deg` in `rx_beamwidths` where a driver puts a −3 dB width (**V**) — those two
  fields are not comparable to a real driver's until that is settled (§7).
- **No gain field exists anywhere in the sim** (**V**), and `ProjectedSonarImage` has nowhere to put one. But
  the quantization in §2.1 is only invertible if the consumer knows the window it used, so `/oculus/meta`
  must carry `display_db` (the `[lo, hi]` window), `tvg_db_per_decade` and `noise_floor_db` alongside the
  range and mode. Those three *are* our gain chain — the real bag's analogue is the single
  `dynamic_reconfigure/Config` carrying `gain`, `gamma`, `range` and `data_16bit` (**V**). Note
  `noise_floor_db` is computed inside the sonar model and has no real counterpart (§2.5), so it travels as
  metadata for invertibility, not as something a real head would tell us.
- Our ping is **8-bit**, which matches the real norm (`data_16bit` false in 92 of 95 lab configs, **V**); the
  head can be put in 16-bit mode, so a future 16-bit profile is a metadata flag, not a redesign.
- A third model, `oculus_interfaces/Ping`, is the only one carrying ping id, gain, mode, temperature and pressure
  inline. It and `ProjectedSonarImage` are not losslessly convertible in either direction. We pick one target and
  say so rather than straddling.

### 2.5 Ground truth under `/gt`

Today ground truth sits on `/sim/*` and — worse — inside `/oculus/meta`. Six leakage paths exist (**V**); these
are the rules that close them.

**What `/gt` carries live**, all renamed out of `/sim`:

| topic | type | rate | bytes |
|---|---|---|---|
| `/gt/sonar_pose` | `PoseStamped` | per ping | small — **moved out of `/oculus/meta`** |
| `/gt/leak` | `PoseStamped` (hole + outward normal) | 1 Hz, **not latched** | small |
| `/gt/leak/flow_lpm` | `Float32` | 1 Hz | small |
| `/gt/leak/geometry` | range / bearing / elevation / buried / mass flow / `in_view` | per ping | small |
| `/gt/plume/centreline` | the 160-point integral axis: position, tangent, `u_c`, `b` | 2 Hz | ~1.9 kB — the most useful compact GT item |
| `/gt/plume` | `MarkerArray`, 3000 bubbles | 2 Hz | ~120 kB → 1.9 Mbit/s |
| `/gt/mask_geom` | `CompressedImage` (png) | per ping | ~3.5 kB tank/lf → ~0.3 Mbit/s |

`/gt/polar_db` (the calibrated level of §2.1) is recorded on the sim host and **never crossed**: 143.5 Mbit/s
(tank/lf) to 524.5 (basin/hf) of a quantity only the evaluator wants.

**What is never published live**: `mask_all`, `mask_direct` and both soft shares. Raw, the three masks plus two
shares are 179 Mbit/s (tank/lf) to **656 Mbit/s** (basin/hf) (**V**) — they do not coexist with anything. They are
already in the recorder; they go to the bag on the sim host, where the offline evaluator reads them.

**Five hard rules**, because a namespace alone is a convention and conventions leak:

1. The exact sonar pose **leaves `/oculus/meta`**. It is the one metadata topic the detector must subscribe to, and
   it currently contains `position` and `roll/pitch/yaw` (**V**). A real Oculus publishes none of those, so
   removing them is also the more faithful choice.
2. **A test asserts that `sonar_meta()` contains no label and no pose field.** It is clean today and is one
   careless `**msg.meta` away from shipping the answer — the per-ping JSON also holds `plume_pixels.geom > 0`,
   which *is* the `in_view` target (**V**).
3. `/sim/leak` and `/sim/leak/flow_lpm` **lose TRANSIENT_LOCAL**. Latched ground truth is the worst case: a node
   that never asked for it receives it on join.
4. The live `/tf` sonar pose comes from the **estimator or ArduSub**, never from the simulator. The sim's true
   transform goes on `/gt/tf` or under a `gt_` frame prefix. Any detector doing a TF lookup today reads truth.
5. **The rosbag2 profile excludes `/gt`, or replay remaps it into a dead namespace** — a written rule in step 3,
   not a convention. A bag recorded with `--all` and replayed defeats rules 1–4 at once.

Two traps that produce silently wrong `/gt` rather than a leak: the recorder stores every image `[::-1]`
(far range first) while `ros_bridge.py:236` flips `polar` back, so a `/gt` mask published straight from the stored
array is **mirrored relative to `/oculus/polar`** — publish from the unflipped `ping.plume_mask_*`. And
`load_ping(image="db")` normalises by `meta.sonar.noise_floor_db`, a quantity computed inside the sonar model that
no real head reports (**V**): the run-time path consumes `["image"]` only, and `masks`/`shares`/`targets` are
evaluator-only keys.

Enforcement by namespace is what M7 adopts. The only *middleware-enforced* alternative is a second
`ROS_DOMAIN_ID` for `/gt`, which costs the evaluator a bridge and the Jetson a second participant; hold it in
reserve for the day a leak is actually observed.

## 3. Clock and lockstep

### 3.1 One clock, and it is Isaac's

**Isaac's physics time is the single clock authority.** The same value is (a) the JSON `timestamp` field that
drives SITL and (b) the only thing published on `/clock`. Jetson nodes run `use_sim_time:=true` off that `/clock`
over the wire.

This is forced, not chosen. SITL advances **only** when a JSON frame arrives: `deltat = timestamp − last`,
`time_now_us += deltat·1e6`, and `time_advance()` (**V**, `SIM_JSON.cpp:504-525`). ArduPilot's own
`--use_sim_time` does *not* slave the autopilot to ROS `/clock` — with DDS built in, AP_DDS subscribes to `/clock`
and uses it to **replace header stamps only** (**V**, `AP_DDS_Client.cpp:273-292`); the scheduler still moves on
JSON timestamps. So there is no second clock to reconcile, provided Isaac publishes both.

### 3.2 The lockstep hazard, and the architectural consequence

`recv_fdm()` is blocking, polls with `UDP_TIMEOUT_MS = 100` and **loops forever** with no give-up path, reprinting
"No JSON sensor message received, resending servos" every 1000 ms (**V**, `SIM_JSON.cpp:35, 322-331`). A stalled
backend therefore **freezes the autopilot** — and with it MAVLink and DDS servicing, which are main-loop tasks.
The Jetson's symptom is telemetry and `/clock` stopping, not a runaway vehicle; its watchdog must alarm on a
stalled clock, not on a mode change.

Worse, the parser keeps only the **newest** complete frame, so a backlog collapses into one large `deltat`; if that
`deltat ≥ 0.1 s`, wall-clock pacing is skipped but `time_now_us` still jumps the full amount — one IMU sample
across a >100 ms gap, which is the classic EKF-divergence path (**V**).

Now put the repo's measured numbers next to that 0.1 s cliff. `uw/main.py` is **one synchronous loop**: a ping
costs 36.4 ms mean / 70.1 max with a fixed pose, and a *moving* sonar adds a static recompute of **133–154 ms
(tank)** or **291–352 ms (basin)** per ping (**V**, `output/probe/mobile_timing.json`). A ping inside the JSON
producer's loop would stall ArduSub for 1.3–3.5 ping periods and feed its 400 Hz loop a single 100–350 ms step,
every ping.

> **Decision.** The Fossen body integration and the JSON producer run in **their own thread, decoupled from the
> ping**, at ≥ 400 Hz (ArduSub's `MAIN_LOOP_RATE`, **V** `ArduSub/config.h:16`), sharing only the body state with
> the Isaac loop. `SIM_RATE_HZ` is set to the rate actually sent. The repo's single `dt` today is
> `plume.dt = 1/240 s` (**V**) and there is **no PhysX scene at all** — so `docs/PLAN.md` M8a's "evaluated per
> physics step … Isaac/PhysX integrates it" has to be read as a separate integrator beside Isaac, not as a PhysX
> force callback at 240 Hz, which would starve a 400 Hz autopilot.

`no_lockstep` and `no_time_sync` stay **false**. Both are undocumented escape hatches and both trade the real
estimator for convenience — the latter defaults `AHRS_EKF_TYPE` to 10, the simulator's truth AHRS (**V**).

A timestamp going backwards is tolerated as a "physics reset" and does **not** reset the autopilot: mode, arming
and EKF state survive an Isaac stage reload (**V**). Scenario changes therefore need an explicit disarm and mode
reset.

### 3.3 Measuring latency across the link, when stamps are sim time

Every stamp in this system is sim time, and sim time advances at RTF × wall time. A wall-clock delay `Δt` on the
Jetson appears inside the sim as `RTF · Δt` — **at RTF 0.694 a real 60 ms loop looks like 42 ms**, optimistic by
1/RTF. Hence two separate measurements, never one number:

- **sim-time chain**, stamp to stamp: ping stamp → detection → setpoint → the JSON frame that acts on it. Needs no
  inter-host clock sync and is the number that describes what the *vehicle* experiences.
- **wall-clock round trip**, sim → Jetson → setpoint → sim, measured on linus at both ends. Also needs no sync.

A one-way wall-clock number would need synced clocks, and `enp5s0` has **no PTP hardware clock** (software
timestamping only, **V**) — chrony over the LAN puts the floor at tens of µs to ~1 ms, i.e. ~1 % of a 100 ms ping
period: adequate for a 10 Hz loop, not for sub-millisecond claims. Drops are read per subscription from
`QoSMessageLostInfo` via `SubscriptionEventCallbacks(message_lost=…)`, which exists in rclpy (**V**), together
with `incompatible_qos` — the latter catches a RELIABLE-reader/BEST_EFFORT-writer mismatch that otherwise presents
as total silence. Kernel-side, sample `/proc/net/snmp` Udp `RcvbufErrors`/`InErrors` and Ip
`ReasmFails`/`ReasmTimeout` as deltas around a run; absolutes are already non-zero on linus from other work.

## 4. MAVROS or native DDS: MAVROS, and the reason is not preference

> **Decision: MAVROS 2.16.0 on the Jetson, MAVLink v2 over unicast UDP as the only control traffic on the wire.**

On ArduPilot master today, **`ArduSub/AP_ExternalControl_Sub.cpp` does not exist** — Copter, Plane and Rover each
have one, Sub does not (**V**). Every AP_DDS control path null-checks that singleton, so on an unforked ArduSub
`/ap/cmd_vel`, `/ap/cmd_gps_pose` and `/ap/arm_motors` are received, deserialised and **silently dropped**
(**V**, `AP_DDS_ExternalControl.cpp:12-17, 48-53`). The fix is PR **#33175**, open since 2026-05-23 with two
approvals (**V**). Adopting DDS for commanding before it merges means carrying a patch, against
`docs/PLAN.md`'s "ArduSub stays unforked".

What DDS *does* deliver on Sub is telemetry — `/ap/pose/filtered`, `/ap/twist/filtered`, `/ap/imu/…`, `/ap/clock`
— plus `/ap/mode_switch`, `/ap/prearm_check`, parameters, and `/ap/joy`, which maps axes into RC overrides
vehicle-agnostically. It is a usable telemetry path and a dead command path. Its cost is also real: AP_DDS is off
by default in SITL builds, `micro_ros_agent` has **no Jazzy binary release**, and `ardupilot_msgs` must be built
from the ArduPilot tree **on both hosts including arm64** (**V**). MAVROS is `apt install ros-jazzy-mavros`
(2.16.0, 2026-09-27) (**V**).

On the wire, MAVLink is also the better-behaved choice: ≤280 B per frame, one message per datagram, **never IP
fragmented**, ~14 kB/s at 50 Hz, stateless under loss (a dropped setpoint is simply superseded). AP_DDS's XRCE
session is reliable with a 4 KB window, notices a dead peer after ~1.5 s (3 missed 500 ms pings) and then re-runs
its whole create sequence (**V**).

**Correction for `docs/PLAN.md`:** it calls MAVROS "mature, Sub-aware" and native DDS "needs checking". Precisely:
MAVROS is Sub-*agnostic* and mature — the Sub-awareness lives in ArduSub's MAVLink handler — and native DDS is not
unchecked but **confirmed non-functional for commanding Sub**.

### 4.1 The command interface, and three traps

What ArduSub accepts (**V**, `ArduSub/GCS_MAVLink_Sub.cpp`):

| MAVLink | modes | MAVROS topic | note |
|---|---|---|---|
| `SET_POSITION_TARGET_LOCAL_NED` (84) | **GUIDED only** | `setpoint_raw/local` | pos / vel / posvelaccel by type_mask; FORCE rejected. No sysid gate |
| `SET_POSITION_TARGET_GLOBAL_INT` (86) | GUIDED, AUTO, **and ALT_HOLD** | `setpoint_raw/global` | in ALT_HOLD it sets depth directly — **the one position-free setpoint path** |
| `SET_ATTITUDE_TARGET` (82) | → `guided_set_angle` | `setpoint_raw/attitude` | needs both attitude and thrust |
| `MANUAL_CONTROL` (69) | any | `manual_control/send` | drives **and** feeds the pilot-input watchdog |
| `RC_CHANNELS_OVERRIDE` (70) | any | `rc/override` | GCS sysid only |

1. **Sysid.** `MAV_GCS_SYSID` defaults to 255 and gates RC override and manual control; MAVROS's `source_system`
   defaults to 1 (**V**). Out of the box `manual_control/send` and `rc/override` are **rejected**. Set
   `MAV_GCS_SYSID = 1` — which also turns MAVROS's 1 Hz heartbeat into a genuine command-link watchdog through
   `FS_GCS_ENABLE` (default: disarm after 5 s).
2. **The pilot-input failsafe disarms a polite commander.** `FS_PILOT_INPUT = 2` (disarm) after
   `FS_PILOT_TIMEOUT = 3.0 s`, reset **only** by msg 69, msg 70 or real RC — not by guided setpoints, and not
   mode-gated (**V**). A pure setpoint commander is disarmed every 3 s. **Decision:** the mission layer sends
   `MANUAL_CONTROL` as its heartbeat, since it both drives and feeds the watchdog; keep ArduSub's safety net
   rather than setting `FS_PILOT_INPUT = 1`.
3. **Arming needs a disarm path.** ArduSub refuses to arm unless a disarm or arm-toggle button function is
   assigned — `BTN4_FUNCTION 4`, which SITL's `sub.parm` already sets and a hand-rolled param set will not (**V**).
   It also warns, and fails with `ARMING_CHECK` PARAMETERS on, when the EKF has no origin.

### 4.2 The position-estimate branch, which is also a ground-truth question

SITL enables GPS instance 0 **regardless of the parameter** and feeds it from the FDM position, so a Sub in SITL
has a working horizontal fix underwater (**V**) — convenient, physically false. The honest alternatives are
`EK3_SRC1_POSXY = 6` (ExternalNav) fed by `VISION_POSITION_ESTIMATE`, or `GPS_INPUT`; both pipe the simulator's
true pose into the control path over MAVLink, **outside the `/gt` rule of §2.5**, and make the Jetson's "state
estimation" circular.

> **Decision for the first closed loop:** **ALT_HOLD + msg 86 for depth + `MANUAL_CONTROL` for horizontal.** It
> needs no position estimate, so it needs no ground truth in the control path, and depth is the axis the vehicle
> actually has authority over. GUIDED — and with it the position-source question — is a deliberate later step,
> not a default.

This matters more than it looks: GUIDED runs the WPNAV/PSC position controller with `PILOT_SPEED` 200 cm/s and
`PILOT_SPEED_UP` 500 cm/s in a **2.9 m water column inside a 3 m tank** (**V**), where a single setpoint can
command the vehicle across the whole tank in under two seconds. Those gains are unvalidated at tank scale; the
param delta the project commits to belongs in a named `.parm` file beside this doc (§7).

## 5. Latency budget for the M8b closed loop

Written against **linus, no camera, `--sink ros`, RTF 0.997** (**V**) — the only profile on this machine where
wall time and sim time coincide, so the two agree to within the measurement.

| term | mean | max | source |
|---|---|---|---|
| `gpu_wait_before_ping` (no camera) | 3.3 ms | — | **V** `sim_max--no-camera.json` |
| sonar ping compute, fixed pose, tank | 36.4 | 70.1 | **V** (plume-box paths 26.8 of it) |
| …**static recompute if the sonar moved**, tank | **+133–154** | — | **V** `mobile_timing.json` |
| …the same, basin | **+291–352** | — | **V** |
| publish, ROS sink headless | 9.0 | 21.8 | **V** `sim_ros_stats.json` |
| serialization + 1 GbE transfer, 448 kB payload | ~3.6 | — | **U**, computed at line rate |
| link RTT linus ↔ Jetson | — | — | **unmeasured** |
| Jetson decode + detector inference | — | — | **does not exist** (M10 unstarted) |
| MAVLink one-way + ArduSub reaction (next 400 Hz loop) | ~2.5 | — | **U** |
| **quantization, listed separately** | | | |
| ping period | 100 ms | | **V** |
| `/clock` tick | 16.7 ms | | **V** |
| `RosPose` polled once per ping | 100 ms | | **V** — disappears in M8a (§6) |

The sim-side chain alone is **~49 ms mean / ~95 ms max** against a 100 ms ping period and a 100 ms
`ping_budget_ms.static`. Three honest conclusions:

1. **With a moving sonar the sonar rate, not the network, is the binding constraint.** 36 + 133 ms ≈ 170 ms per
   ping in the tank means **~5.9 Hz**, and the basin ~2.6 Hz. `on_motion` does not rescue it: at the default
   0.15 m/s the pose moves 1.5 cm per ping, 3× the 5 mm threshold, and it recomputed on 40 of 40 pings on a
   trajectory (**V**). M8b runs the sonar at 5 Hz in the tank, or the static-cache recompute gets the
   partial-update fix already listed in PLAN's Known gaps.
2. **The budget is not a number yet** — the detector does not exist and the link is unmeasured. What this table
   fixes is the *method* and the sim-side terms, so that M7 step 5 and M10 fill in two columns rather than
   inventing a total.
3. **Twelve `ros2 topic hz` processes dropped the sim to RTF 0.5** (**V**): subscriber-side CPU on the publisher's
   host is itself a term. Keep the monitors on the Jetson.

## 6. M8a: the vehicle in the sim

### 6.1 Frames

Today there is **no vehicle body frame at all** — the sonar is the only moving frame, and the payload positions in
the size presets are absolute tank coordinates (sonar `[0.10, 1.50, 1.05]`, camera `[0.10, 1.50, 0.95]`, lamps
likewise) (**V**). M8a needs them vehicle-relative:

```
map (≡ tank, ENU, z up, origin at the inner bottom corner)
└── odom                     published by the estimator / ArduSub via MAVROS
    └── base_link (FLU)      the BlueROV2 body; published by the dynamics
        ├── oculus           static, from a new config block (was absolute)
        │   └── oculus_projected   static, R_SONAR_PROJECTED (exists)
        ├── camera → camera_optical  static, R_BODY_OPTICAL (exists)
        └── lamp_*           static
gt_tank → gt_oculus          the simulator's true pose, /gt only (§2.5 rule 4)
```

Who publishes which edge is the load-bearing part: `map→odom` and `odom→base_link` come from the
estimator/ArduSub, **never from the sim**, or rule 4 of §2.5 is void. The mount offsets themselves need a
BlueROV2 payload-skid dimension source or an explicit UNVERIFIED assumption (§7).

### 6.2 NED/FRD ↔ ENU/FLU, with the actual constants

Declare the tank frame as ENU: **tank +x = East, +y = North, +z = Up** (right-handed, consistent with the existing
z-up convention). Then, with home at the water surface (`water_level = 2.9 m` in a 3.0 m tank, **V**):

| quantity | conversion |
|---|---|
| earth, ENU → NED | `N = y_tank`, `E = x_tank`, `D = 2.9 − z_tank` |
| earth, NED → ENU | `x_tank = E`, `y_tank = N`, `z_tank = 2.9 − D` |
| body, FLU → FRD | `x_FRD = x_FLU`, `y_FRD = −y_FLU`, `z_FRD = −z_FLU` |
| depth sensor | ArduSub's baro is `BARO_TYPE_WATER`, pressure from `SimpleUnderWaterAtmosphere` of the FDM altitude (**V**) — Isaac controls ArduSub's depth **purely through `position[2]`** |
| `SURFACE_DEPTH = −10 cm` | `z_tank = 2.8 m` |

The yaw offset between tank +x and true north is a free choice; fixing it at 0° (tank +y = north) is the simplest
and must be written down, together with the chosen EKF origin lat/lon, because `AHRS_ORIGIN_*` gates arming.

### 6.3 Fossen dynamics and thrusters

The model: `M ν̇ + D(ν) ν + g(η) = τ`, with `M = M_RB + M_A` (added mass), `D(ν) = D_lin + D_quad|ν|`, `g(η)` the
restoring wrench from the gravity/buoyancy offset, and `τ = T · f` from the thrust-allocation matrix. Integrated in
the dedicated ≥400 Hz thread of §3.2, publishing IMU, position, attitude and velocity as JSON; `pwm` comes back
and goes through the thrust curve.

> **Not one of these coefficients exists in this repo or in any source read for this doc.** Mass, displaced
> volume, the inertia tensor, added mass, linear and quadratic damping, the CG–CB offset, the six (or eight)
> thruster positions and axes, and the T200 PWM→thrust curve with its forward/reverse asymmetry at a stated
> voltage are **all TBD**. They get a `docs/validation.md`-style table with a source per row and a VERIFIED /
> UNVERIFIED tag, built from BlueRobotics' spec sheet, the T200 performance chart and a published BlueROV2 Fossen
> identification. Until that table exists, this doc states no values rather than plausible ones.

**The frame choice is forced by our own datasets.** The default BlueROV2 mixer (`FRAME_CONFIG = 1`, Vectored, six
thrusters) has an **all-zero pitch column** — no pitch authority (**V**, `AP_Motors6DOF.cpp:172-180`). But every
v0.2 view is pitched: `pitch_deg = 15` nose-down by default and `trajectory.pitch: track` aims the axis at the
leak (**V**). A six-thruster BlueROV2 cannot hold the attitude the datasets were generated at, so closed-loop
views would not be comparable to the dataset.

> **Decision: M8a models the BlueROV2 **Heavy** (`Vectored_6DOF`, eight thrusters).** The alternative — a fixed
> tilted sonar mount with pitch uncontrolled and trimmed by ballast — stays open and is cheaper, but it changes
> what a "pose" means between the dataset and the closed loop. The hover pitch actually needed to put the leak in
> the 20° vertical fan from the configured rig position should be computed before this is final.

Read `SERVOn_FUNCTION` to find which `pwm[i]` is which thruster rather than assuming `pwm[0..5]` (**V**).
`MOT_FV_CPLNG_K` is an autopilot-side output limit — its effect arrives in the PWM, nothing to replicate.

### 6.4 The JSON/UDP interface

Loopback, SITL listening on **UDP 9002** and replying to the source of what it received (**V**).

**SITL → sim**, 40-byte little-endian struct: `magic = 18458` (the magic *is* the version handshake; 29569 with
`SERVO_32_ENABLE`), `frame_rate`, `frame_count`, `pwm[16]` µs.

**sim → SITL**, newline-framed ASCII JSON. Required: `timestamp` (absolute physics seconds), `imu.gyro`,
`imu.accel_body`, `velocity` [N,E,D], and `attitude` **or** `quaternion`; `position` [N,E,D] is not flagged
required but is the only practical depth source, so treat it as required (**V**). Three parser facts that bite:
each frame is preceded *and* terminated by `\n`; the parser takes the text **between the last two newlines**, i.e.
the newest frame, discarding any backlog; and parsing is `strstr`-based, not a real JSON parse, so keep the frame
flat and in documented key order. A missing mandatory field drops the frame with "Did not contain all mandatory
fields" — and the first connection prints the received field list, which makes a good acceptance test for the
bridge.

**Do not send** what SITL synthesises: the magnetometer (computed from position and attitude, so a steel tank's
distortion is simply not modelled), the depth/baro, the GPS, and IMU noise and bias. Water current, buoyancy,
thruster wake and umbilical drag are **not in the protocol at all** — they live in Isaac/Fossen and reach the EKF
only through the IMU and position we produce.

### 6.5 Two things nothing currently bounds

There is **no PhysX scene**, hence no contacts: an M8a vehicle under thruster commands can leave the water, pass
through the pipe or exit the tank. The kinematic presets got this free from `pose.py`'s bounds clipping with a
0.2 m margin, which will not apply to a dynamic body. Either the walls, floor and pipe get collision geometry, or
the dynamics carry an explicit soft bound and surface constraint — a decision, not an oversight to discover later.

And `mask_geom`'s line-of-sight test rays against occluders `(sand, rock, pipe, support, flange)` — **the ROV hull
is not among them** (**V**). Once a BlueROV2 body is in the scene, `mask_geom` will mark bubbles the vehicle's own
frame blocks, silently changing the label's meaning between M7 and M8a datasets. If the hull joins
`occluder_roles`, the label needs a schema version marker.

## 7. Open questions

**Blocking the first implementation step**

1. **The Jetson link: MTU 1500 on 1 GbE, which is what §2.2 is computed against** — so the bandwidth table
   stands as written, and jumbo frames are off the table unless the topology changes (they would cut the
   camera's ~1879 fragments per frame to ~310). Still **TBD**, and small: the **model**, the **NIC**
   (onboard 1 GbE or other) and the **topology** (direct cable or via switch). Those three settle arm64
   package availability (MAVROS 2.16.0 is a Jazzy binary; `marine_acoustic_msgs` and any custom message
   package need building for the Jetson's distro), whether a switch sits in the latency path, and whether the
   NIC has a PTP hardware clock — linus's `enp5s0` does not, which sets the wall-clock sync floor of §3.3.
   One `ip link; ethtool -T <nic>; ethtool <nic>` on the Jetson closes all of it.
2. **Measured RTT and jitter** linus ↔ Jetson. It is the first term of §5 and it also decides whether MAVROS's
   TIMESYNC converges at all: samples with RTT > 10 ms are rejected, and under `use_sim_time` the RTT is measured
   in *sim* time, so a stepped or slowed clock can keep the offset from ever converging — after which every
   MAVROS stamp quietly becomes `node->now()`.
3. **The Fossen / T200 parameter table** (§6.3). No coefficient exists today.
4. **Custom message types may be blocked on three targets at once**: Isaac's bundled Jazzy `rclpy` (Python 3.11,
   no `/opt/ros` in the container), host Jazzy (3.12) and the Jetson's arm64. That constraint is presumably why
   `/oculus/meta` is a `std_msgs/String` and why the relay runs on the host. Confirm the in-container
   rclpy/FastDDS version before promising any typed `/gt` or sonar-meta message — the same check settles whether
   Isaac's rclpy honours `FASTRTPS_DEFAULT_PROFILES_FILE` and `LARGE_DATA` at all.

**Decisions this doc deliberately leaves open**

5. Does GUIDED hold position at 3 m scale, and with which `PSC_*` / `WPNAV_*` / `PILOT_SPEED*` deltas? Produce a
   named `.parm` file, or state that M8b starts in ALT_HOLD with gains unvalidated.
6. The position-estimate story (§4.2): fake SITL GPS, ExternalNav with `EK3_SRC1_POSXY = 6`, or no horizontal
   estimate at all. It decides whether GUIDED exists and how circular the estimation milestone is.
7. Model the real Oculus **nRanges cap** (383–709 rows, 10–39 mm) or state plainly that our images are finer than
   any real bag's? Largest fidelity gap and largest bandwidth lever, same knob. And does the M750d share the
   M1200d's ladder at all — all of those numbers come from M1200d bags.
8. Which bag-compatibility target: `ProjectedSonarImage` (chosen here) or `oculus_interfaces/Ping` (the only one
   carrying ping id, gain, mode, temperature, pressure inline)? Also: what *is* the ROS 1
   `acoustic_msgs/SonarImage` field layout, since that is what the 151 GB of DFKI bags actually contain?
9. Should `rx_beamwidths` carry a vertical −3 dB width instead of the full `fov_v_deg` the relay puts there?
10. Do the soft shares have any run-time consumer? They are 40 % of the raw `/gt` bytes and the worst-compressing
    part; PLAN already lists dropping them.
11. Does the ROV hull become a `mask_geom` occluder (§6.5), and does the label get a schema version?
12. `MANUAL_CONTROL` heartbeat (chosen) or `FS_PILOT_INPUT = 1` with a mission-side watchdog?
13. Where is the rosbag2 recorded, and can the disk take it? Raw capture is ~5.6 MB per ping period ≈ 56 MB/s at
    10 Hz, and zlib was measured at 63 ms of a ~100 ms ping write — inline compression cannot keep up at ping
    rate. Sustained write throughput on the target volume is unmeasured.
14. **f1tenth coexistence for a long HIL session**: the hard rule is no ROS process, `ros2` CLI included, while a
    harness batch runs, and a second host now joins domain 73 on the shared subnet. Who holds the lock, and what
    does the Jetson do when a batch starts?
15. Pin the sysctls on both hosts; linus's current `rmem_max` is set by nothing on disk.

**Stale text in `docs/PLAN.md` to fix when this lands**: decision (d) still says the sim publishes over Zenoh with
a relay to ROS 2 and "ROS is optional on the sim side", which M7 overrides and `sinks.make_sink("zenoh")` raises
`NotImplementedError` for; the MAVROS "Sub-aware" / DDS "needs checking" wording (§4); and the "bags match future
real Oculus bags" claim (§2.4).

## 8. M7 steps 2–5

**Step 2 — the sim-side bridge and a commanded pose.**
Split the pose out of `/oculus/meta` into `/gt/sonar_pose` and add the §2.5 test. Rename `/sim/*` to `/gt/*`, drop
the latching, add `/gt/leak/geometry`, `/gt/plume/centreline` and `/gt/mask_geom` (unflipped). Switch the image
topics to `SensorDataQoS`. Give `RosPose` a producer: a teleop node and a scripted-trajectory node publishing
`/uw/sonar_pose`. Note that `pose_source=ros` forbids `static_cache=precomputed` and so pays ≥133 ms per ping
(**V**) — the 5 Hz consequence of §5 appears here, not in M8b. Make `RosSink` pass `domain_id` explicitly.

**Step 3 — rosbag2 profiles.**
One profile per purpose: a *detector* bag (`/oculus/sonar_image`, `/oculus/meta`, `/clock`, `/tf*`) and an
*evaluation* bag that adds `/gt/**` and the host-local `polar_db` and masks. The replay rule of §2.5 is part of
the profile, not a convention. Decide the recording host and measure the disk first (§7 item 13).

**Step 4 — the Jetson side.**
A minimal subscriber reporting rate, age from `header.stamp`, and drops from `QoSMessageLostInfo`; the kernel
counter deltas of §3.3; a Docker image for the Jetson's arch. No detector yet — this step measures the transport.

**Step 5 — loopback, then Ethernet.**
Loopback on linus first (shared memory, which is what every number measured so far used), then the real link, and
report rate, drops and both latency measurements of §3.3 with the RTF and the machine next to each one. This is
the step that turns §2.2's arithmetic and §5's table into measurements.
