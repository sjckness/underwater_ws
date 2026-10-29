# Proposed topic, namespace and frame map (M7 step 1, task 2)

> **Status (2026-10-29): a proposal. Nothing in this document is implemented, and no code was changed for it.**
> It is written against the topics that exist today (`uw/ros_bridge.py`, `ros/sonar_relay`, `config/tank.yaml`,
> all **V**) and against the decisions already taken in `docs/ros_hil.md` §2 (what crosses the link, QoS,
> metadata, `/gt`) and §6.1 (frames). Where it departs from the starting draft, §7 says so and why. Numbers
> tagged **V** are traceable to a file or a measurement; **U** are assumed or inferred.
>
> Adopting it is a rename of **every topic the project publishes** plus the frame names, so §9 lists the changes
> file by file, and §8 lists what must be decided first.

## 1. Three namespaces, and what each one promises

The requirement is a clean sim/real separation plus room for a second vehicle. One prefix is not enough for that,
because two different things have no counterpart on the real robot: **the truth** (what the detector must
estimate) and **the plumbing** (what only a simulator needs at all). They want different rules.

| prefix | contains | rule |
|---|---|---|
| `/rov1/…` | everything a real BlueROV2 with a real Oculus would publish, with the real driver's names and types | the Jetson stack subscribes **only** here, plus `/clock` and `/tf*` |
| `/gt/…` | ground truth: leak pose and flow, plume geometry, labels, the true vehicle and sonar pose | **no Jetson node ever subscribes.** Never latched. Excluded from the detector bag |
| `/sim/…` | simulator plumbing with no real counterpart: the kinematic pose command, scene parameters, RTF and loop health | no Jetson *perception or control* node subscribes; a monitor may read `/sim/diagnostics` |

Unprefixed: **`/clock`, `/tf`, `/tf_static`** only. They cannot be namespaced — `use_sim_time` reads `/clock` from
the global namespace, and tf2 needs one tree for all vehicles. Multi-vehicle separation in TF is done with
**frame prefixes** (`rov1/base_link`), not topic namespaces (§2).

Five naming rules behind the table in §4:

1. **Sensor topics carry the name and type the real driver would produce.** `/rov1/sensors/sonar_fls/sonar_image`
   is `marine_acoustic_msgs/ProjectedSonarImage` because that is what the APL-lineage Oculus drivers publish and
   what `ros/sonar_relay` already emits (**V**). The camera follows `image_pipeline`
   (`image_raw` + `camera_info`).
2. **Autopilot-provided sensors keep MAVROS's names**, under `/rov1/mavros/…`. MAVROS *is* the real driver for
   the IMU, the depth/pressure and the autopilot's own pose, so inventing `/rov1/sensors/imu` would create a
   second name for the same real topic and a remap to maintain forever.
3. **Function layer by role, one level deep**: `sensors` (payload sensors), `nav` (state estimate),
   `perception` (what the detector produces), `control` (what the mission layer commands), plus the MAVROS
   namespace. Device names below `sensors/` are **functional, not brand** (`sonar_fls`, not `oculus`), so a
   different head is a config change.
4. **One publisher per topic, named in the table.** Where that is impossible (`/tf`) the *edge* has one
   publisher (§2).
5. **Units and frames per REP-103/REP-105**: metres, seconds, radians, m/s, rad/s; right-handed; ENU world,
   FLU body, optical frames for the camera. One documented exception: gas flow stays in **L/min**, with the unit
   in the topic name (`flow_lpm`), matching the existing field (**V**) and field practice.

## 2. Frames and the TF tree

Rename `tank` → `map`, as `docs/ros_hil.md` §6.1 already states (`map ≡ tank`, ENU, origin at the inner bottom
corner, z up). Frame ids get a **vehicle prefix** and no leading slash.

```
map                                   REP-105 world frame ≡ the tank (ENU: +x East, +y North, +z Up,
│                                     origin at the inner bottom corner; water_level = 2.9 m, V)
├── rov1/odom                          ← rov1 estimator (map→odom correction; identity until a localizer exists,
│   │                                     published by a Jetson-side static publisher, NEVER by the sim)
│   └── rov1/base_link                 ← rov1 estimator, or MAVROS local_position (odom→base_link). FLU
│       ├── rov1/sonar_fls             static, mount offset from a new config block (absolute today, V)
│       │   └── rov1/sonar_fls_projected   static, R_SONAR_PROJECTED (exists, V)
│       ├── rov1/camera_front          static
│       │   └── rov1/camera_front_optical  static, R_BODY_OPTICAL (exists, V)
│       ├── rov1/imu_link              static
│       ├── rov1/pressure_link         static — the depth datum; not the same point as the CG
│       └── rov1/lamp_0, rov1/lamp_1   static
│
└── gt/rov1/base_link                  ← the sim, on /gt/tf ONLY (never /tf) — ros_hil §2.5 rule 4
    ├── gt/rov1/sonar_fls              the true sonar pose
    └── gt/leak                        the hole, +x along its outward normal
```

| edge | publisher | topic | note |
|---|---|---|---|
| `map → rov1/odom` | rov1 localizer (Jetson); a static identity until one exists | `/tf` | the sim must never publish this edge, or rule 4 is void |
| `rov1/odom → rov1/base_link` | rov1 estimator / MAVROS (Jetson) | `/tf` | the only moving edge on `/tf` in the first loop |
| `base_link → payload frames` | `robot_state_publisher` from the vehicle URDF, or the sim bridge until one exists | `/tf_static` | **all static transforms must go in one `TFMessage`**: a depth-1 TRANSIENT_LOCAL writer replays only its last sample (**V**, ros_hil §2.3) |
| `map → gt/*` | the sim | **`/gt/tf`** | a separate topic *and* a `gt/` frame prefix — either one alone leaks |

Two notes. **`rov1/odom` exists even though nothing fills it yet**, because inserting it later changes every
consumer's lookup; REP-105 wants a continuous frame distinct from `map` whether or not a correction is published.
And **the sonar frame is `rov1/sonar_fls`, not `oculus`** — the relay carries the frame name through
`/oculus/meta`'s `projected_frame_id` field (**V**), so this is a one-line change there, not a protocol change.

## 3. QoS classes

Unchanged from `docs/ros_hil.md` §2.3, named here so the table can refer to them:

| class | profile | applies to |
|---|---|---|
| **S** (sensor) | `SensorDataQoS`: KEEP_LAST 5, BEST_EFFORT, VOLATILE | sonar and camera images, GT rasters |
| **L** (latched) | RELIABLE, TRANSIENT_LOCAL, depth 1 | `meta`, `camera_info`, `/tf_static`, `/sim/env/params` |
| **R** (default) | RELIABLE, VOLATILE, depth 10 | setpoints, odometry, estimates, diagnostics, scalars |
| **T** (tf) | RELIABLE, VOLATILE, KEEP_LAST 100 | `/tf`, `/gt/tf` — tf2's own broadcaster depth |
| **C** (clock) | `ClockQoS`: KEEP_LAST 1, BEST_EFFORT | `/clock` |

**S is a change from today**: the bridge publishes images RELIABLE depth 2 (`ros_bridge.py:211`, **V**), and Fast
DDS's `max_blocking_time` default of 100 ms is the *entire* `ping_budget_ms.static` (**V**) — one blocked write
eats the whole ping budget.

## 4. The map

Columns: **msg** type · **pub** → **sub** · **QoS** class (§3) · **frame_id** · **rate** · **sim-only**, where
sim-only is `no` (a real deployment has this topic too), `yes` (simulation only), `yes*` (simulation only but
legitimately consumed by the Jetson stack).

### 4.1 Global

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | `sim_bridge` | every node (`use_sim_time:=true`) | C | — | 60 Hz (`ros.clock_hz`, **V**) | yes* |
| `/tf` | `tf2_msgs/TFMessage` | rov1 estimator / MAVROS | every node | T | — | ~50 Hz (**U**, estimator rate) | no |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` (or `sim_bridge`) | every node | L | — | once, latched | no |

### 4.2 `/rov1/sensors` — payload sensors, named as the real driver names them

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/sonar_image` | `marine_acoustic_msgs/ProjectedSonarImage` (`DTYPE_UINT8`) | `sonar_image_relay` (sim host) | `leak_detector` (thor), rosbag2 | S | `rov1/sonar_fls_projected` | 10 Hz fixed pose; **5 Hz mobile** (**V**, ros_hil §5) | no |
| `/rov1/sensors/sonar_fls/meta` | `std_msgs/String` (JSON) | `sim_bridge` | relay, detector, rosbag2 | L | `rov1/sonar_fls` | on change | no — the real analogue is one `dynamic_reconfigure/Config` per stream (**V**) |
| `/rov1/sensors/sonar_fls/image` | `sensor_msgs/Image` mono8 | `sim_bridge` | `sonar_image_relay` **same host only** | S | `rov1/sonar_fls` | 10 Hz | no — host-local; see §6 |
| `/rov1/sensors/camera_front/image_raw` | `sensor_msgs/Image` rgb8 1280×720 | `sim_bridge` | detector (optional), rosbag2 | S | `rov1/camera_front_optical` | 10 Hz (`camera.rate_hz`, **V**) | no |
| `/rov1/sensors/camera_front/camera_info` | `sensor_msgs/CameraInfo` | `sim_bridge` | same | S (at image rate, as a real driver) | `rov1/camera_front_optical` | 10 Hz | no |

`meta` replaces the draft's `sonar_fls/info`: `ProjectedSonarImage` already self-describes beam geometry
(`beam_directions`, `ranges`, `sound_speed`), so an `info` topic would duplicate it. What it does **not** carry is
our gain chain, and the 8-bit quantization is only invertible with it — so `meta` must carry `display_db`,
`tvg_db_per_decade` and `noise_floor_db` plus range and mode (**V**, ros_hil §2.4). `tvg_db_per_decade` is
**missing from `sonar_meta()` today** (**V**) and `position`/`roll`/`pitch`/`yaw` must leave it (§9).

### 4.3 `/rov1/mavros` — the autopilot's own sensors and the command wire (M8a/M8b)

MAVROS launched with namespace `/rov1/mavros`. Rates are MAVROS/ArduSub stream-rate parameters, all **U** until
measured.

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/rov1/mavros/imu/data` | `sensor_msgs/Imu` | `mavros` (thor) | estimator | S | `rov1/imu_link` | ~50 Hz (**U**) | no |
| `/rov1/mavros/imu/static_pressure` | `sensor_msgs/FluidPressure` | `mavros` | estimator | S | `rov1/pressure_link` | ~10 Hz (**U**) | no |
| `/rov1/mavros/altitude` | `mavros_msgs/Altitude` | `mavros` | estimator, mission | R | `rov1/base_link` | ~10 Hz (**U**) | no |
| `/rov1/mavros/local_position/odom` | `nav_msgs/Odometry` | `mavros` | estimator | R | `rov1/odom` | ~30 Hz (**U**) | no |
| `/rov1/mavros/state` | `mavros_msgs/State` | `mavros` | mission, diagnostics | L | — | 1 Hz | no |
| `/rov1/mavros/battery` | `sensor_msgs/BatteryState` | `mavros` | diagnostics | R | — | 1 Hz | no — **synthetic in SITL** |
| `/rov1/mavros/manual_control/send` | `mavros_msgs/ManualControl` | `mavlink_adapter` (thor) | `mavros` | R | — | ≥ 1 Hz, **must not stop** | no |
| `/rov1/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | `mavlink_adapter` | `mavros` | R | — | 10 Hz (**U**) | no |

`manual_control/send` is the heartbeat, not a convenience: `FS_PILOT_INPUT = 2` disarms after
`FS_PILOT_TIMEOUT = 3.0 s` and **only** msg 69/70 or real RC resets it (**V**, ros_hil §4.1). The first closed
loop is ALT_HOLD + msg 86 for depth + `MANUAL_CONTROL` for horizontal (**V**, §4.2), so `setpoint_raw/local` and
GUIDED are deliberately absent from this table.

### 4.4 `/rov1/nav` — the state estimate

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/rov1/nav/odom` | `nav_msgs/Odometry` | `state_estimator` (thor) | mission, detector, rosbag2 | R | header `rov1/odom`, child `rov1/base_link` | 50 Hz (**U**) | no |

**The draft's `/rov1/nav/pose` is dropped.** It would be the same information as `odom.pose` and as the
`rov1/odom → rov1/base_link` TF edge, i.e. three sources that can disagree. If a map-frame localizer appears
later it publishes `/rov1/nav/pose` as `geometry_msgs/PoseWithCovarianceStamped` in `map` *and* the `map → odom`
TF edge — different information, so it earns its own topic then.

### 4.5 `/rov1/perception` — what the detector produces (M10)

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/rov1/perception/bubbles/detections` | `vision_msgs/Detection2DArray` | `leak_detector` (thor) | mission, rosbag2, evaluator | R | `rov1/sonar_fls_projected` (polar image coords) | per ping | no |
| `/rov1/perception/bubbles/mask` | `sensor_msgs/Image` mono8 | `leak_detector` | evaluator, rosbag2 | S | `rov1/sonar_fls` | per ping | no |
| `/rov1/perception/bubbles/leak_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | `leak_detector` | mission | R | `map` | ≤ 1 Hz | no |
| `/rov1/perception/bubbles/flow_estimate` | `geometry_msgs/Vector3Stamped` (x = L/min, y = σ, z unused) | `leak_detector` | mission, evaluator | R | `rov1/sonar_fls` | ≤ 1 Hz | no |
| `/rov1/perception/debug/overlay` | `sensor_msgs/CompressedImage` (jpeg) | `leak_detector` | operator only | S | `rov1/sonar_fls` | ≤ 2 Hz, **off by default** | no |

Two deliberately ugly choices, both because **a custom message package is blocked on three build targets at
once** — Isaac's bundled Jazzy `rclpy` (Python 3.11, no `/opt/ros`), host Jazzy (3.12) and the Jetsons' arm64
(**V**, ros_hil §7 item 4):

- `flow_estimate` as `Vector3Stamped` is a stopgap. There is no stamped scalar in `std_msgs`, and under HIL the
  stamp is not optional — message age *is* a measurement (§3.3 of ros_hil). The alternative is `uw_msgs`
  (§8 item 1).
- `mask` is a plain `Image`, as `/gt/mask_geom` is, so detector output and label are directly comparable. The
  overlay is `CompressedImage` on purpose: a raw overlay at ping rate is another ~36 Mbit/s of a picture nobody
  evaluates.

### 4.6 `/rov1/control` — what the mission layer commands

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/rov1/control/cmd_vel` | `geometry_msgs/TwistStamped` | `mission` (thor) | `mavlink_adapter` | R | `rov1/base_link` (FLU) | 10 Hz (**U**) | no |
| `/rov1/control/depth_setpoint` | `geometry_msgs/PointStamped` (z used) | `mission` | `mavlink_adapter` | R | `map` | 10 Hz (**U**) | no |
| `/rov1/control/setpoint` | `geometry_msgs/PoseStamped` | `mission` | `mavlink_adapter` | R | `map` | **unused in the first loop** | no |

`TwistStamped`, not `Twist`: MAVROS's own `setpoint_velocity/cmd_vel` is stamped, and an unstamped command in a
HIL latency budget cannot be aged. `depth_setpoint` is a `PointStamped` rather than a bare `Float64` because the
frame travels with it — "depth" has a sign and a datum (`z_tank = 2.9 − D`, **V**, ros_hil §6.2), and that is
exactly the kind of thing that is silently wrong in a scalar. `control/setpoint` is listed but unused: it is the
GUIDED path, and GUIDED at 3 m tank scale has unvalidated gains (**V**, ros_hil §4.2).

### 4.7 `/rov1/diagnostics` — health

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/rov1/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `link_monitor`, `leak_detector`, `state_estimator`, `mission` (all thor) | operator, rosbag2 | R | — | 1 Hz | no |

**The draft's `/rov1/status/health` becomes `/rov1/diagnostics`.** `DiagnosticArray` on `diagnostics` is what
`diagnostic_updater`, `diagnostic_aggregator` and `rqt_runtime_monitor` already speak, and under a vehicle
namespace it is still the standard multi-robot layout; a bespoke `status/health` would need its own tooling. The
HIL link monitor of M7 step 4 (rate, age from `header.stamp`, drops from `QoSMessageLostInfo`) publishes here,
which is also what makes a **stalled clock** visible — the symptom of the lockstep hazard in ros_hil §3.2.

### 4.8 `/gt` — ground truth. No Jetson node subscribes. Never latched.

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/gt/tf` | `tf2_msgs/TFMessage` | `sim_bridge` | evaluator, rosbag2 (eval profile) | T | — | per ping | yes |
| `/gt/rov1/pose` | `geometry_msgs/PoseStamped` | `sim_bridge` | evaluator | R | `map` | per physics publish | yes |
| `/gt/sonar_pose` | `geometry_msgs/PoseStamped` | `sim_bridge` | evaluator | R | `map` | per ping | yes |
| `/gt/leak/pose` | `geometry_msgs/PoseStamped` (+x = outward normal) | `sim_bridge` | evaluator | R | `map` | 1 Hz (`ros.leak_hz`, **V**) | yes |
| `/gt/leak/flow_lpm` | `std_msgs/Float32` | `sim_bridge` | evaluator | R | — | 1 Hz | yes |
| `/gt/leak/geometry` | `std_msgs/String` (JSON: range, bearing, elevation, buried, mass flow, `in_view`) | `sim_bridge` | evaluator | R | `rov1/sonar_fls` | per ping | yes |
| `/gt/plume/centreline` | `sensor_msgs/PointCloud2` (x y z tx ty tz u_c b tau) | `sim_bridge` | evaluator | R | `map` | 2 Hz (`ros.plume_hz`, **V**) | yes |
| `/gt/plume/markers` | `visualization_msgs/MarkerArray` | `sim_bridge` | RViz, evaluator | S | `map` | 2 Hz, 3000 of up to 150 000 bubbles (**V**) | yes |
| `/gt/mask_geom` | `sensor_msgs/CompressedImage` (png) | `sim_bridge` | evaluator | S | `rov1/sonar_fls` | per ping, ~3.5 kB (**V**) | yes |
| `/gt/polar_db` | `sensor_msgs/Image` 32FC1 | `sim_bridge` | **sim-host evaluator and bag only** | S | `rov1/sonar_fls` | 10 Hz, 143.5–524.5 Mbit/s (**V**) | yes |

`centreline` as `PointCloud2` because it is the only standard message that carries arbitrary per-point fields:
160 points × 9 float32 ≈ 5.8 kB at 2 Hz, and it is the most useful compact GT item there is (**V**, ros_hil
§2.5). `nav_msgs/Path` would hold position and tangent but lose `u_c`, `b` and `tau`.

`mask_all`, `mask_direct` and both soft shares are **not published at all** — 179 to 656 Mbit/s raw (**V**). They
go to the bag on the sim host, where the offline evaluator reads them.

**A trap that produces silently wrong GT rather than a leak**: the recorder stores every image `[::-1]` (far
range first) while `ros_bridge.py` flips `polar` back (**V**), so a mask published straight from the stored array
is **mirrored relative to the sonar image** — publish from the unflipped `ping.plume_mask_geom`.

### 4.9 `/sim` — simulator plumbing with no real counterpart

| topic | msg | pub | sub | QoS | frame_id | rate | sim-only |
|---|---|---|---|---|---|---|---|
| `/sim/cmd/rov1/pose` | `geometry_msgs/PoseStamped` | `pose_cmd` (sim host) | `sim_bridge` (`RosPose`) | R | `map` | ≤ 10 Hz, polled once per ping (**V**) | yes |
| `/sim/env/params` | `std_msgs/String` (JSON: preset, tank size, water level, temperature, salinity, sound speed, seed, config hash) | `sim_bridge` | rosbag2, evaluator | L | — | once, latched | yes |
| `/sim/diagnostics` | `diagnostic_msgs/DiagnosticArray` (RTF, ping ms, budget overruns, bubble count) | `sim_bridge` | operator, rosbag2, link monitor | R | — | 1 Hz | yes* |
| `/sim/debug/sonar_fan` | `sensor_msgs/Image` mono8 | `sim_bridge` | operator only | S | `rov1/sonar_fls` | 10 Hz, **off by default** | yes |

`/sim/env/params` carries the **environment**, not the answer: the leak's position, normal and flow live under
`/gt/leak/*`, and the preset *name* is in `/sim/env/params` only because a bag is useless without knowing which
scenario it is. Sound speed and water temperature are not GT — a real Oculus is configured with them and
`ProjectedSonarImage.ping_info.sound_speed` carries it on every ping (**V**).

`/sim/diagnostics` is `yes*` for one reason that matters: **every latency number in this project is meaningless
without the RTF it was measured at** — a wall-clock delay appears inside the sim scaled by RTF, so at 0.694 a
real 60 ms loop looks like 42 ms (**V**, ros_hil §3.3). The HIL monitor must be able to read it, so it is the one
`/sim` topic a Jetson *monitor* may subscribe to. No perception or control node may.

`/sim/debug/sonar_fan` replaces today's `/oculus/fan`, off by default: the fan is **derived from the uint8 polar
on the sim side** (`Sonar.fan()` takes it as input, **V**), so publishing it means sending a resampled copy of
data already sent, and M10 has not chosen between a polar and a Cartesian model.

## 5. Nodes

| node | host | publishes | subscribes | exists? |
|---|---|---|---|---|
| `sim_bridge` (today `uw_sim`) | sim host | `/rov1/sensors/**`, `/gt/**`, `/sim/**`, `/clock`, `/tf_static` | `/sim/cmd/rov1/pose` | **V** `uw/ros_bridge.py` |
| `sonar_image_relay` (today `uw_sonar_relay`) | **sim host** | `/rov1/sensors/sonar_fls/sonar_image` | `…/image`, `…/meta` | **V** `ros/sonar_relay` |
| `pose_cmd` | sim host | `/sim/cmd/rov1/pose` | — | **V** `ros/pose_cmd`: hold / trajectory / keys |
| `rov1_dynamics` | sim host | body state → JSON/UDP to SITL (loopback, not ROS) | `/tf` of nothing | no — M8a |
| `mavros` | thor | `/rov1/mavros/**`, `/tf` | `/rov1/mavros/{manual_control/send,setpoint_raw/global}` | no — M8b |
| `mavlink_adapter` | thor | MAVROS command topics | `/rov1/control/**` | no — M8b |
| `state_estimator` | thor | `/rov1/nav/odom`, `/tf` (`odom→base_link`) | `/rov1/mavros/**` | no — M9/M10 |
| `leak_detector` | thor | `/rov1/perception/**` | `/rov1/sensors/sonar_fls/{sonar_image,meta}`, `/clock`, `/tf` | no — M10 |
| `mission` | thor | `/rov1/control/**` | `/rov1/perception/**`, `/rov1/nav/odom` | no — M8b |
| `link_monitor` | thor (and the sim host, for loopback) | a report; `/rov1/diagnostics` later | every sensor topic, with the QoS read from each publisher | **V** `ros/link_monitor`: rate, payload, age, `QoSMessageLostInfo` |
| `rosbag2` | sim host (linus) | — | per profile (§6) | no — M7 step 3 |

Every node except `sim_bridge` runs with `use_sim_time:=true`. **Nothing in the repo declares it today** (**V**).

## 6. What crosses the Ethernet

From `docs/ros_hil.md` §2.1–2.2, with the new names. Payload at 10 Hz, tank/lf preset, 512 beams, 876 rows:

| topic | crosses | payload | of 1 GbE (+8 % framing) |
|---|---|---|---|
| `/rov1/sensors/sonar_fls/sonar_image` | **yes** — the detector's only sonar input | 37.5 Mbit/s | 4 % |
| `/rov1/sensors/sonar_fls/meta` | **yes**, once per scenario | — | — |
| `/rov1/sensors/sonar_fls/image` | **no** — its *pixels* cross, inside `sonar_image` | 35.9 Mbit/s | — |
| `/rov1/sensors/camera_front/image_raw` | camera profile only | 221.2 Mbit/s | 24 % |
| `/gt/polar_db` | **never** | 143.5 Mbit/s | — |
| `/gt/plume/markers`, `/gt/mask_geom`, `/gt/**` scalars | only to the evaluator / bag | ~2.2 Mbit/s | — |
| `/clock`, `/tf*`, `/rov1/control/**`, `/rov1/diagnostics` | yes | small | — |

**How "does not cross" is actually enforced**: DDS sends a sample over the wire only when a *remote* reader has
matched it, so `…/image` and `/gt/polar_db` stay host-local as long as nothing remote subscribes. That is an
emergent property, not a guarantee — one `ros2 topic echo /gt/polar_db` from thor puts 143 Mbit/s on the link.
Hence §8's enforcement list, and hence the operator rule that monitors run on the subscriber host (**V**, ros_hil
§5: twelve `ros2 topic hz` processes dropped the sim to RTF 0.5).

**Bag profiles** (M7 step 3): a *detector* bag — `/rov1/sensors/**`, `/clock`, `/tf*`, `/rov1/perception/**`,
`/rov1/control/**`, `/rov1/diagnostics`, `/sim/env/params` — and an *evaluation* bag that adds `/gt/**` and the
host-local rasters. **A bag recorded with `--all` and replayed defeats the whole separation at once**: the replay
rule (exclude `/gt`, or remap it into a dead namespace) is part of the profile, not a convention (**V**, ros_hil
§2.5 rule 5).

## 7. Differences from the starting draft

| draft | proposal | why |
|---|---|---|
| `/sim/ground_truth/rov1/pose`, `/sim/ground_truth/bubbles/{sources,plumes}` | `/gt/rov1/pose`, `/gt/leak/*`, `/gt/plume/*` | `docs/ros_hil.md` §2.5 already decided `/gt` with five enforcement rules, and §8 step 2 says "rename `/sim/*` to `/gt/*`" in so many words (**V**). Also: `bubbles/sources` is the **leak**, and the project calls it that everywhere (`leak_position`, `/sim/leak`, leak presets) |
| `/sim/env/params` | **kept**, with the leak removed from it | the environment is not the answer; the leak is. See §4.9 |
| `/rov1/sensors/sonar_fls/{image,info}` | `sonar_image` (ProjectedSonarImage) + `meta` + a host-local `image` | `info` would duplicate the beam geometry that `ProjectedSonarImage` already carries; what is actually missing is the **gain chain** (`display_db`, `tvg_db_per_decade`, `noise_floor_db`), without which the 8-bit quantization is not invertible (**V**, ros_hil §2.4) |
| `/rov1/sensors/{imu,depth}` | `/rov1/mavros/imu/data`, `/rov1/mavros/imu/static_pressure`, `/rov1/mavros/altitude` | MAVROS **is** the real driver for these. A second name for the same real topic is a remap to maintain forever, and it breaks the "same names as the real driver" constraint rather than honouring it |
| `/rov1/nav/{odom,pose}` | `/rov1/nav/odom` only | `pose` is `odom.pose` and the `odom→base_link` TF edge restated; three sources that can disagree. It returns when a map-frame localizer exists (§4.4) |
| `/rov1/perception/bubbles/{detections,flow_estimate}` | `+ mask`, `+ leak_estimate`; `flow_estimate` as `Vector3Stamped` | `mask` makes detector output comparable with `/gt/mask_geom`; `leak_estimate` is what the mission layer actually needs; `Vector3Stamped` is a stopgap because no stamped scalar exists and custom messages are blocked on three targets (§4.5, §8 item 1) |
| `/rov1/control/{cmd_vel,setpoint}` | `cmd_vel` (TwistStamped) + `depth_setpoint` (PointStamped); `setpoint` kept but unused | the first closed loop is ALT_HOLD + msg 86 depth + `MANUAL_CONTROL`, which needs **no position estimate and no GT in the control path** (**V**, ros_hil §4.2). A single `setpoint` hides that depth is the only axis with real authority |
| `/rov1/status/health` | `/rov1/diagnostics` (`DiagnosticArray`) | the standard aggregator topic and type, so existing tooling works (§4.7) |
| `/clock`, `/tf`, `/tf_static` | unchanged, global | correct as drafted; multi-vehicle separation goes in **frame prefixes**, not topic namespaces (§2) |
| — | `+ /sim/diagnostics` (RTF) | not in the draft, and without it no latency number in this project can be interpreted (**V**, ros_hil §3.3) |
| — | `+ /gt/tf`, `+ /gt/leak/geometry`, `+ /gt/plume/centreline`, `+ /gt/mask_geom` | the §2.5 GT set, and the centreline is the most useful compact label there is |
| frames `tank`, `oculus`, `camera` | `map`, `rov1/sonar_fls`, `rov1/camera_front` + `rov1/odom`, `rov1/base_link` | REP-105 names, a body frame that does not exist today at all, and per-vehicle prefixes (**V**, ros_hil §6.1) |

## 8. Open decisions (these block implementation, in this order)

1. **`uw_msgs`, or `String`/`Vector3Stamped` forever?** **Decided 2026-10-29: standard messages only, for
   now.** A typed `SonarMeta`, `LeakGeometry` and `FlowEstimate` would remove three JSON blobs and one abused
   message, but the package must build for Isaac's bundled Jazzy `rclpy` (Python 3.11, no `/opt/ros`), host
   Jazzy (3.12) **and** arm64 on thor and orin (**V**, ros_hil §7 item 4). Reopen it when all four are proven.
2. **Does the relay stay a separate node? Settled: yes.** The bundled `rclpy` in the image ships
   `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `rosgraph_msgs`, `sensor_msgs`, `std_msgs`, `tf2_msgs`,
   `tf2_ros`, `vision_msgs` and `visualization_msgs` — and **no `marine_acoustic_msgs`** (**V**, listed in
   `isaac_ws_devcontainer-isaac-sim:latest` on 2026-10-29). So `sim_bridge` cannot publish
   `ProjectedSonarImage` itself, `…/sonar_fls/image` stays, and the relay keeps its hop. The same listing
   confirms `/sim/diagnostics` (`diagnostic_msgs`) and the detector's `vision_msgs` are available in the sim;
   the arm64 half of item 8 is still open.
3. **`rov1` as a literal, or `vehicle_ns` from config?** This map hard-codes `rov1` in strings. One config key
   (`ros.vehicle: rov1`) plus a frame prefix makes multi-ROV a launch argument; it also means no topic name in
   the code is a constant any more, which is a readability cost. Recommend the config key, since the whole point
   of the namespace is the second vehicle.
4. **Who owns `map → rov1/odom` before a localizer exists?** A static identity publisher on the Jetson is the
   honest answer (the sim must not publish it, ros_hil §2.5 rule 4) — but then `map` and `odom` are the same
   frame and nobody notices a missing localizer. Alternative: no `map → odom` edge at all until there is one, and
   the detector works in `odom`.
5. **Is `/sim/diagnostics` really allowed on the Jetson?** It is the exception that makes the rule arguable.
   Alternative: the sim publishes RTF *into* `/rov1/diagnostics` as one more `DiagnosticStatus`, which keeps the
   blacklist absolute at the cost of putting a sim quantity in a vehicle topic.
6. **The GT enforcement mechanism.** Namespace + a test + the bag rule is what this map assumes. The only
   *middleware-enforced* option is a second `ROS_DOMAIN_ID` for `/gt`, costing the evaluator a bridge and the
   Jetson a second participant — hold it in reserve until a leak is actually observed (**V**, ros_hil §2.5).
7. **Sonar rate**: 10 Hz is in every table, but a moving sonar costs 36 + 133–154 ms per ping in the tank, i.e.
   **~5.9 Hz** (**V**, ros_hil §5). Either every rate in §4.2 becomes 5 Hz for mobile runs, or the static-cache
   partial-update fix lands first. The map should not claim 10 Hz where the sim cannot deliver it.
8. **`vision_msgs` version on arm64 Jazzy** for `Detection2DArray`/`Detection3DArray`, and whether the detector's
   output is boxes, a mask, or both. M10 has not chosen (**V**, ros_hil §2.1).

## 9. What adopting this map changes in the existing code

Nothing below has been done. Grouped by file, with what it is today.

**`uw/ros_bridge.py`** — the whole topic and frame surface is hard-coded here (**V**):

- `FRAMES` (`line ~41`): `tank`→`map`, `oculus`→`rov1/sonar_fls`, `oculus_projected`→`rov1/sonar_fls_projected`,
  `camera`→`rov1/camera_front`, `camera_optical`→`rov1/camera_front_optical`; add `base_link`, `odom` and the
  `gt/` variants.
- `RosSink.__init__`'s `self.pub` dict: every one of the 11 topic strings changes. Move them to a module-level
  `TOPICS` dict (or `config/tank.yaml`'s `ros:` block) so this map has exactly one place in the code.
- QoS: images from `_qos({"depth": 2})` RELIABLE to `SensorDataQoS` (KEEP_LAST 5, BEST_EFFORT); `/tf` depth
  10→100; drop TRANSIENT_LOCAL from the leak topics.
- `sonar_meta()`: **remove** `position`, `roll_deg`, `pitch_deg`, `yaw_deg` (they become `/gt/sonar_pose`);
  **add** `tvg_db_per_decade` (missing today); keep `display_db` and `noise_floor_db`.
- `publish()`: `polar_db` moves to `/gt/polar_db`; `fan` to `/sim/debug/sonar_fan`, default off; add the
  `/gt/leak/geometry`, `/gt/plume/centreline`, `/gt/mask_geom` and `/gt/tf` publishers; publish masks from the
  **unflipped** arrays.
- `rclpy.init()`: pass `domain_id` explicitly, as `relay.py:85` does — today it only reads the env var and logs a
  default of `0`, an f1tenth domain (**V**).

**`ros/sonar_relay/sonar_relay/relay.py`** — subscribe `…/sonar_fls/{image,meta}`, default output topic
`/rov1/sensors/sonar_fls/sonar_image` (`--topic` already exists, **V**); `SENSOR` QoS becomes BEST_EFFORT to
match the publisher, or the subscription silently never matches.

**`uw/main.py`** — `static_frames()` (`line 106`) builds `world→sonar` / `world→camera` from absolute tank
coordinates (**V**); with a body frame those become `base_link→payload` plus the estimator's edges. `R_BODY_OPTICAL`
(`line 31`) is unchanged. The per-ping `sink.transforms(...)` call (`line 249`) publishes the sim's true sonar
pose on `/tf` — **that is rule 4's violation** and must move to `/gt/tf`.

**`uw/sonar_only.py`** — the same two calls with literal `"tank"`, `"oculus"`, `"oculus_projected"` strings
(`lines 71, 88`, **V**).

**`config/tank.yaml`** — the `ros:` block's comments name `/sim/plume`, `/sim/leak`, `/oculus/fan` (**V**); add
`ros.vehicle`, a `ros.topics`/`ros.frames` block if §8 item 3 is adopted, and the sonar/camera **mount offsets**
relative to `base_link` (absolute tank coordinates today, in the size presets). `sonar.ros_pose_topic` default
`/uw/sonar_pose` → `/sim/cmd/rov1/pose`.

**`tests/test_ros_messages.py`** — asserts `"oculus"`, `"oculus_projected"` and `frame_id == "tank"` at four
places (**V**); they follow the rename. Add the §2.5 test: **`sonar_meta()` contains no pose field and no label**
— it is clean today and one careless `**msg.meta` away from shipping the answer, since the per-ping JSON also
holds `plume_pixels.geom > 0`, which *is* the `in_view` target (**V**).

**New** — a `tests/test_topic_map.py` that greps the Jetson-side packages for `/gt` and `/sim` and fails on a
match, which is the only cheap part of §8 item 6 that can exist before the Jetson stack does.

**`docs/ros_hil.md`** — §1, §2.1, §2.5, §6.1 and §8 use the old names throughout; they become this map's names
when it is approved. `docs/PLAN.md`'s decision (d) still describes a Zenoh transport that
`sinks.make_sink("zenoh")` raises `NotImplementedError` for (**V**, already listed as stale text in ros_hil §7).
