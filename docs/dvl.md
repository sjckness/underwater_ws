# The DVL and the dead reckoning

A Water Linked DVL A50 in the tank sim, and the dead-reckoning chain it feeds. The point of modelling
the sensor rather than publishing the true velocity is that the chain downstream of it is the one that
runs on the vehicle, and the drift it accumulates is therefore a number worth measuring:
`tools/dvl_drift.py` prints it live, `tools/dvl_dr_offline.py` prints it without any hardware at all.

```
uw/rov/dvl.py          4 beams, raycast, least squares, noise, FOM, bottom lock   (numpy, no GPU)
      ▼
/sim/dvl/report        std_msgs/String: the A50's own JSON velocity report, host-local
      ▼
ros/dvl_relay/         stands in for the real driver (host Jazzy)
      ├── /rov1/sensors/dvl/velocity        marine_acoustic_msgs/Dvl       in rov1/dvl_instrument (FRD)
      ├── /rov1/sensors/dvl/velocity_twist  TwistWithCovarianceStamped     in rov1/dvl_link (FLU)
      ├── /rov1/sensors/dvl/altitude        sensor_msgs/Range
      └── /rov1/sensors/dvl/meta            std_msgs/String, latched
      ▼
ros/dvl_dr/preprocess  v_body = v_dvl − ω × r; depth from the baro; drops reports with no lock
      ▼
robot_localization     /rov1/nav/odom, and odom → base_link on /tf
      ▲
      └── /rov1/mavros/{imu/data, imu/static_pressure}       ros/mavros_rov1
```

On the vehicle the first two boxes are the A50 and `ndahn/dvl_a50`. Everything from
`/rov1/sensors/dvl/**` down is unchanged, which is what makes a comparison against real BlueROV2
odometry mean something.

## Why the simulator emits the device's protocol

`marine_acoustic_msgs` is not in the Isaac container's bundled rclpy (`docs/topic_map.md` §8), so a
`Dvl` message cannot be built there at all — the same constraint that produced `ros/sonar_relay`. Rather
than invent a half-message in the container, the sim publishes the A50's own JSON report verbatim and a
host node turns it into ROS types. That the relay is then interchangeable with the real driver is a
consequence, not a coincidence.

The raw report lives on `/sim/dvl/report`, not under `/rov1/sensors/`, unlike the sonar's host-local
image. A device-protocol dump is plumbing, not something a real vehicle publishes, and keeping it out
of `/rov1` leaves one fewer surface for ground truth to leak through.

## The hardware, verified and not

Checked against Water Linked's own documentation in November 2026. Their docs moved: the
`waterlinked.github.io/dvl/*` links that circulate are dead, and the current ones are
`docs.waterlinked.com/dvl/*`.

| Quantity | Value | Status |
|---|---|---|
| Transducers | 4-beam convex Janus array, 1 MHz | **V** datasheet |
| Beam tilt from vertical | **22.5°** | **V** — the A50 page's line-of-sight drawing; the 65° obstruction-free cone it asks for is consistent with 22.5° plus margin, and impossible at 67.5° |
| Half-power beam width | 4.4° | **V** A50 page |
| Beam azimuths | 45 / 135 / 225 / 315° about the DVL's +x | **U** — from the `beam_unit_vec` hardcoded in the `ndahn/dvl_a50` driver, *not* from Water Linked. A config value for exactly this reason |
| Instrument axes | FRD: x forward, y starboard, z down; the status LED marks +x, the cable points aft | **V** axes and installation pages |
| Altitude | 0.05 – 50 m | **V** datasheet |
| Max speed | 3.75 m/s | **V** |
| Velocity resolution | 0.1 mm/s | **V** |
| Report rate | 2 – 15 Hz, falling with altitude | **V** JSON protocol (the datasheet's marketing figure is 4–15) |
| Long-term accuracy | ±1.01 % Standard, ±0.1 % Performance | **V** — and Performance is a *software licence on the same hardware*, so in the model it is one knob, the scale-factor σ |
| Per-sample noise, slow bias | — | **U**: published nowhere. `dvl.beam_noise_mps`, `dvl.bias_sigma_mps`, `dvl.bias_time_s`, to be tuned against a real log |
| Echo thresholds | — | **U**: `dvl.rssi_ref_dbm`, `dvl.nsd_dbm`, `dvl.snr_threshold_db` set where echoes land in the A50's rssi range |

### A trap worth naming

The `ndahn/dvl_a50` driver hardcodes `beam_unit_vec` as `(±0.65328, ±0.65328, 0.38268)`. Those have sin
and cos swapped: they put the beams 22.5° above the *horizontal* instead of 22.5° off nadir. The
correct vector for azimuth φ is `[sin22.5·cosφ, sin22.5·sinφ, cos22.5]` = `(±0.27060, ±0.27060,
0.92388)`. Only the azimuths are taken from that driver, never the vectors, and
`tests/rov/test_dvl.py::test_beam_vectors_are_not_the_third_party_drivers_swapped_ones` keeps it that
way.

## The measurement

For each report, at a rate that falls with altitude:

1. The four beam axes are built from `dvl.beams.tilt_deg` and `dvl.beams.azimuths_deg` in the
   instrument's FRD frame, then rotated into the tank through `uw.rov.frames` — the only converter.
2. `geometry.ray_bottom` finds each beam's range. The sand is solved analytically against
   `SandField` (Newton on the height field, using its exact gradient), so the incidence angle comes
   from the true surface normal rather than the 1 cm occlusion mesh; the rocks, the pipe, its flanges
   and the supports are tested as triangles behind two levels of AABB rejection. The walls are not
   occluders, so a beam that leaves the tank footprint simply does not return — which is the honest
   model of losing lock near a wall.
3. The echo level per beam is the same backscatter law the sonar uses — monostatic Lambert plus the
   Kirchhoff term of a rough surface with Gaussian slopes, from `geometry.surface_materials` — over the
   beam's footprint, spread as 1/r⁴. A beam locks when its range is inside 0.05–50 m **and** its
   rssi − nsd clears `dvl.snr_threshold_db`.
4. The instrument's true velocity is `R_bd (v_body + ω × r)`: the lever arm is applied at the source,
   because that is where a real one acts. Each beam measures `b_i · v`.
5. Errors, in order: a scale factor drawn once per run (`±1.01 %` is a long-term accuracy, not a
   per-ping error), a slow per-beam bias, per-beam white noise that grows as the echo approaches the
   noise floor, then quantisation to 0.1 mm/s.
6. With three or more valid beams, least squares over those beams gives the velocity. With four, the
   redundant beam gives the error velocity, which inflates the covariance. `fom` is the square root of
   the covariance's largest eigenvalue — a modelling choice, since Water Linked says only that the FOM
   derives from the covariance.
7. The altitude is the mean of the valid beams' ranges times cos(tilt), matching the note in
   `marine_acoustic_msgs/Dvl` itself.

### Determinism

Every error term is a pure function of the seed and either the sample index or the simulated time.
The slow bias is a fixed random spectrum rather than an Ornstein-Uhlenbeck recursion precisely so that
it does not depend on *which* samples were taken — the report schedule varies with altitude, and a
recursion would make two runs of the same scenario diverge. `uw.rov` is bit-reproducible by design
(`tests/rov/test_dynamics.py`) and a sensor must not be the thing that breaks it.

### What the tank actually exercises

| preset | tank | basin | room9 |
|---|---|---|---|
| water over the sand | ≈ 2.6 m | ≈ 3.6 m | ≈ 8.6 m |

So the 50 m ceiling never fires, and the model's interesting end is the bottom: 0.05 m is reachable,
the exposed pipe stands 0.12 m proud of the sand at x = 1.65, there are 14 rocks of radius 0.03–0.16 m,
and near a wall a tilted beam leaves the footprint. Those are the cases the tests pin.

### Not in `sonar-only`

The DVL runs in `python -m uw sim` only, not in `python -m uw sonar-only`. That entry point exists to
generate sonar datasets: its loop is one iteration per ping, which a 2–15 Hz altitude-dependent report
schedule does not map onto, and adding a stream would change what `tools/sweep.py` writes, the dataset
card and `record.mb_per_frame` for no consumer — nothing that reads those datasets wants DVL rows. The
HIL path and the estimator both run under `sim`, so nothing needs it there. If a DVL dataset is ever
wanted, the honest way is a recorded `sim` run, not a second cadence.

### Deferred

Bubble degradation. The card this work came from asks for the plume to spoil beams that pass through
it, which is real — bubbles give spurious echoes and a Doppler bias. It is not modelled yet. The hook
is a multiplier on the beam quality fed from `sim.snapshot()`; until then a beam through the plume is
as good as any other, which is optimistic in exactly the region the gas-leak scenario cares about.

## Frames, and the one place they change

`rov1/dvl_link` is REP-103 FLU. `rov1/dvl_instrument` is the device's own FRD, which is also the frame
`marine_acoustic_msgs/Dvl` declares for itself ("z-axis in direction DVL is pointing, x-axis aligned
with vehicle forward"). The two differ by `uw.ros_bridge.R_DVL_INSTRUMENT = diag(1, −1, −1)`, published
as a static transform, and the relay is the only code that applies it — to the twist, covariance
included. Both edges hang off `rov1/sonar_fls`, like the camera's: payload on one rig, and only the
relative offset is not ground truth.

`uw/main.py:static_frames` and `uw.rov.dvl.Dvl.geometry_at` must agree on where the acoustic centre is,
or the lever arm the estimator removes is not the one the measurement was taken with.
`tests/test_sim_parts.py::test_the_dvl_static_edge_matches_the_sensor_model` asserts it.

## The dead reckoning

`robot_localization`'s `ekf_node`, configured rather than written, plus one small node for the two
things it cannot do: the lever arm and hydrostatic depth. Inputs:

| input | topic | what it observes |
|---|---|---|
| `imu0` | `/rov1/mavros/imu/data` | orientation and angular rate |
| `twist0` | `/rov1/sensors/dvl/velocity_corrected` | body linear velocity |
| `pose0` | `/rov1/nav/depth` | z only |

**Nothing observes x or y.** That is the point: the track is reckoned, and the drift is the result.
`world_frame` is `rov1/odom`, not `map`, because without a map correction there is nothing to anchor
`map → odom` and claiming otherwise would be a lie. `/rov1/nav/pose` can return once a map-frame
localizer exists (`docs/topic_map.md` §4.4).

A report with no bottom lock is **dropped**, not down-weighted. It is not a noisier measurement; the
beams did not come back. The filter then coasts on the IMU, which is honest, and the drop is counted.

### No ground truth, and what that cost

The DVL comes from the sensor model with its noise and dropout. The attitude and the depth come from
the autopilot — which is only honest if the autopilot is not itself handed the answer, and by default
it is, twice over:

1. `AHRS_EKF_TYPE 10` is SITL's "truth AHRS": the attitude MAVROS publishes *is* the simulator's.
2. SITL enables GPS instance 0 from the FDM position whatever `GPS_TYPE` says (`docs/ros_hil.md` §4.2,
   marked **V**, "convenient, physically false"). Even with a real EKF selected, a truth-derived
   horizontal fix would aid it.

`docker/ardusub_sitl/params/hil.parm` closes both: `AHRS_EKF_TYPE 3` and the `EK3_SRC1_*` sources set
to none for position and velocity, baro for z, compass for yaw. What is left is what a real BlueROV2
has underwater. The risks are the ones `docs/ros_hil.md` §4.1 already names — EKF3 with no position
source may refuse to arm, and its health has to be watched — and the revert is one line.

`/gt` stays on its own ROS domain and is read by exactly one thing: the evaluator.

### Heading is the error that matters

A velocity scale error drifts in proportion to distance and shows up as a percentage equal to the scale
error. A **heading** error rotates the whole track: after a run with a net displacement of D metres the
position error is `2·D·sin(θ/2)`, regardless of how fast it was flown or how far it wandered getting
there. In a metal tank the magnetometer is the least trustworthy thing in the vehicle, so this is the
dominant term.

Rather than hand-rolling a yaw bias in the estimator, inject it where a real one lives — SITL's compass
model, `SIM_MAG1_OFS` — and re-run `tools/dvl_drift.py`.

## Measured, offline

`tools/dvl_dr_offline.py` runs the whole chain without Isaac, Docker, ROS or a GPU: the real sensor
model over a trajectory, `dvl_relay`'s FRD→FLU flip, `dvl_dr.correct`'s lever arm and baro, then the
same `drift()` the live tool scores with. It is not `robot_localization`, but with velocity, attitude
and depth as the only inputs **nothing observes x or y**, so an EKF's position estimate *is* the
integral of its velocity estimate — the filter adds smoothing and covariance bookkeeping, not
information, in the two axes the drift lives in. These are therefore the numbers to expect, and if the
live chain comes out much worse, the filter is mistuned rather than the sensor being bad.

`basin/lawnmower` at 0.7 m, flown level, 180 s, median of 3 seeds: 26.6 m travelled, 8.7 m net, 2482
reports, bottom lock held throughout.

| heading bias | final error | drift | 2·sin(θ/2) × net | excess |
|---|---|---|---|---|
| 0° | 0.242 m | **0.91 %** | 0.000 m | 0.242 m |
| 1° | 0.294 m | 1.10 % | 0.152 m | 0.142 m |
| 2° | 0.372 m | 1.40 % | 0.304 m | 0.067 m |
| 5° | 0.761 m | 2.86 % | 0.760 m | 0.001 m |
| 10° | 1.505 m | 5.65 % | 1.519 m | −0.014 m |

Two things to read out of it. **The sensor's own drift is 0.91 % of distance travelled**, which lands
where the A50's ±1.01 % long-term accuracy says it should — the one end-to-end check that the error
budget is wired up and not merely present. And **above about 2° of heading bias the heading is the
whole story**: from 5° up the measured error matches a rigid rotation of the track to within 14 mm.
Note what that closed form is a function of: a heading error goes with the **net displacement**, while
the drift percentage is quoted against distance travelled, as DVLs are. A survey that doubles back is
therefore far more forgiving of a bad compass than a straight transit of the same length.

The vertical does not drift: the baro pins z, and the final z error stays at the baro's own noise.

### What makes it lose bottom lock

Flying a DVL is not the same as flying a sonar, and the tank is small enough that it bites. Measured
over 120 s on `basin/lawnmower` at 0.7 m, counting why each individual beam failed:

| sonar wall margin | trajectory pitch | DVL's own distance to the wall | locked | beams off the footprint | beams below the SNR gate |
|---|---|---|---|---|---|
| 0.35 m | `track` (as shipped) | 0.073 m | 88.4 % | 366 | 2 |
| 0.35 m | 0 | 0.125 m | 99.6 % | 0 | 0 |
| 0.90 m | `track` | 0.623 m | 95.9 % | 0 | 109 |
| 0.90 m | 0 | 0.675 m | 98.5 % | 0 | 42 |

**The margin is applied to the sonar, and the DVL is not the sonar.** `vehicle.body.offset` puts the
body 0.26 m behind and 0.10 m below the sonar's acoustic centre, and the DVL hangs off the body, so a
0.35 m margin on the sonar leaves the DVL within 0.07–0.13 m of a wall. A beam lands
`altitude · tan(tilt + pitch)` to the side — 0.29 m level at 0.7 m, 0.54 m at 15° — and the walls are
not occluders, so those beams return nothing at all and the report drops below three good ones.

Pitch enters twice, which is why its **sign** changes the lock rate at the same margin: it rotates that
body offset, and it leans the beams toward or away from the wall the vehicle is approaching.

Raising the margin *does* clear the walls. What then appears is a second, smaller failure: 105 of the
109 beams the SNR gate rejected at a 0.9 m margin had hit the **exposed pipe** at a median 48° off its
normal. A smooth steel cylinder at a glancing angle returns little to a monostatic sensor, which is the
right kind of failure — but it depends on `dvl.rssi_ref_dbm`, `dvl.nsd_dbm` and `dvl.snr_threshold_db`,
all UNVERIFIED, so treat the *rate* as untuned while the mechanism stands.

One corollary worth keeping in mind when reading a drift number: a **closed** path understates a
heading error, because rotating a loop maps it nearly onto itself. `orbit` shows 0.06 % drift at a 0°
bias and barely more at 5°. Measure heading sensitivity on an open path.

## In the pool: the device on the network

`python -m uw pool` (M11, `docs/pool.md`) uses this model too, cast at the pool's floor, walls, ladder and props
(`uw/pool_geometry.PoolBottom`, the same solids the USD stage is built from; unlike the tank's, the pool's walls stop
a beam). There the report does not go to ROS but to the network, as the device's own: `uw/rov/a50.py` serves Water
Linked's JSON protocol (json_v3.3) on TCP 16171 (velocity reports, the device's dead reckoning, its commands) and
the HTTP API BlueOS's Water Linked DVL extension probes, from the physics process. The extension runs unmodified
and feeds ArduSub; the `ndahn/dvl_a50` driver could connect the same way. The pool's `dvl:` section repeats this
file's instrument values (kept equal by `tests/rov/test_a50.py`) with its own mount and echo reference.

## Running it

```bash
# the model alone, host, no GPU
pytest tests/rov/test_dvl.py tests/test_dvl_dr.py tests/test_geometry.py

# the whole chain against the truth, offline: no Isaac, no Docker, no ROS
python tools/dvl_dr_offline.py --duration 180 --seeds 3
python tools/dvl_dr_offline.py --preset orbit --size room9 --yaw-bias 0 5

# the sim, recording a DVL stream
python -m uw sim --sink file --out output/datasets/dvl_smoke --duration 60 --no-realtime
#   -> output/datasets/dvl_smoke/dvl/*.json, and a DVL line in the run summary

# live, with the relay (host ROS 2 Jazzy)
python -m uw sim --set sonar.pose_source=ardusub
ros/dvl_relay/run.sh
ROS_DOMAIN_ID=73 ros2 topic hz /rov1/sensors/dvl/velocity --no-daemon       # 2-15 Hz, altitude dependent
ROS_DOMAIN_ID=73 ros2 topic echo /rov1/sensors/dvl/meta --once --qos-durability transient_local

# the whole chain, live
bash ros/mavros_rov1/run.sh                                                 # on the Thor
ros2 launch ros/dvl_dr/launch/dvl_dr.launch.py
python3 tools/dvl_drift.py --duration 120 --label baseline
```

Cost: a report costs about 1.7 ms, dominated by the beam raycast, so at 15 Hz the DVL is under 3 % of
real time. It must stay there — the scene already runs at RTF 0.82–0.95.

## Configuration

Everything is `dvl:` in `config/tank.yaml`, with a VERIFIED/UNVERIFIED tag per value in the comments.
The mount is in ArduPilot body FRD, like the thruster positions in
`config/rov_bluerov2_vectored.yaml`, and deliberately not in the size presets — it belongs to the
vehicle, not to the room. `uw.rov.dvl.params_from_config` refuses a value outside the datasheet.
