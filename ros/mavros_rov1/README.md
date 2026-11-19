# MAVROS for the HIL vehicle

MAVROS 2.16.0 on the Jetson, reading ArduSub SITL's MAVLink over the direct cable, under the namespace
`docs/topic_map.md` §4.3 reserves: `/rov1/mavros/**`.

| File | What it is |
|---|---|
| `install.sh` | installs `ros-jazzy-mavros`, `ros-jazzy-mavros-extras` and the GeographicLib datasets on the Thor. Idempotent, asks before each `sudo` step |
| `mavros.launch.py` | `apm.launch` inside `PushRosNamespace(vehicle)`, with `use_sim_time` applied to every node |
| `run.sh` | the launch in the dedicated HIL terminal, after `~/hil_env.sh` |

```bash
# on the Thor, once
bash install.sh
# then, per session (HIL terminal)
bash run.sh                                  # Isaac running: sim time from /clock
USE_SIM_TIME=false bash run.sh               # smoke test without Isaac
ros2 topic hz /rov1/mavros/state             # ~1 Hz, connected: true
```

These files were in `output/hil/` (`thor_mavros_*`), which is git-ignored. They are here because the
dead-reckoning chain's **input contract** cannot live in an untracked directory: `ros/dvl_dr` subscribes
`/rov1/mavros/imu/data` and `/rov1/mavros/imu/static_pressure`, and if the namespace or the launch
changes, that breaks silently. The copies under `output/hil/` are superseded; the ones on the Thor's
home directory should be replaced from here.

## Why the namespace changed

`thor_mavros.launch.py` included `apm.launch` bare, so the topics came out as `/mavros/**`. Every name
the estimator and the topic map use is `/rov1/mavros/**`. This is the only functional difference.

## Only telemetry

Consumed: `imu/data`, `imu/static_pressure`, `state`. Not consumed, and not configured here: command
topics, setpoints, arming. Those belong to M8b, and with them the three traps of `docs/ros_hil.md` §4.1
(`MAV_GCS_SYSID` gating manual control, the pilot-input failsafe disarming a polite commander, and the
disarm-button requirement for arming). A read-only link meets none of them.

## The autopilot must not be handed the truth

MAVROS is only honest if what ArduSub estimates is honest. Two truth paths run into SITL by default and
`docker/ardusub_sitl/params/hil.parm` closes both — see the comments there. In short: `AHRS_EKF_TYPE 10`
hands ArduSub the simulator's attitude, and SITL feeds GPS instance 0 from the FDM position whatever
`GPS_TYPE` says (`docs/ros_hil.md` §4.2, marked **V**). With `AHRS_EKF_TYPE 3` and the `EK3_SRC1_*`
sources set to none for position and velocity, ArduSub runs its own EKF3 over SITL's synthetic noisy
IMU, baro and compass — which is what a real BlueROV2 does underwater.
