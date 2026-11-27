# rov_bringup

The ROS 2 side of the real BlueROV2, run inside the BlueOS ROS 2 extension on the Pi (`uw deploy rov`).

| File | What |
|---|---|
| `launch/mavros.launch.py` | MAVROS to ArduSub through BlueOS's internal link (`tcp://127.0.0.1:5777`), namespace `/rov1/mavros` |
| `config/mavros_pluginlists.yaml` | **telemetry only**: every plugin denied, then only reading ones allowed (no `command`, `param`, `setpoint_*`, ...) |
| `launch/record.launch.py`, `rov_bringup/record.py` | `ros2 bag record` of a profile |
| `profiles/light.yaml` | IMU, attitude, depth, state, motor PWM; MCAP |
| `config/mcap_light.yaml` | MCAP writer options: **no compression** (zstd costs the Pi's CPU), 1 MiB chunks |
| `launch/bringup.launch.py` | MAVROS + (with `record:=true`) the recording |

```bash
# in the extension's terminal, after `uw deploy rov`
source /root/persistent_ws/ros2_ws/install/setup.bash
ros2 launch rov_bringup bringup.launch.py record:=true      # bags in /root/persistent_ws/bags
```

Nothing here can arm the vehicle or change an ArduSub parameter (tests/test_rov_bringup.py checks the plugin set).
