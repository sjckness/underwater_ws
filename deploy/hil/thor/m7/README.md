# M7 on the Thor: measurement and consumer scripts (sensors-only HIL leg)

Isaac on the work machine (192.168.200.2, `--sink ros`) publishes the sonar, camera and `/clock` topics. The Thor
(192.168.200.10, ROS 2 Jazzy) receives them on domain 73 over Fast DDS unicast. These scripts measure that link and
run a minimal consumer. Topic names and types are the ones in the main README ("ROS 2 topics"). `docs/ros_hil.md`
does not exist yet.

## Files

| File | What it does |
|---|---|
| `measure.py` | Rate (wall and header-stamp sim time), stamp gaps (> 1.5x median period), per-message size and bandwidth for `/oculus/polar`, `/oculus/polar_db`, `/oculus/fan`, `/camera/image_raw`; real-time factor from `/clock`; latency proxy (latest `/clock` on the Thor minus header stamp, not one-way latency) |
| `simtime_check.py` | Checks that a node with `use_sim_time:=true` takes its clock, timers and stamps from `/clock` |
| `m7_consumer.py` | Consumer (sim time): decodes `/oculus/polar_db` and `/camera/image_raw`, thresholds the sonar dB image, finds connected components, publishes `/uw/m7/detections` (`vision_msgs/msg/Detection2DArray`, layout in the docstring) |
| `peek_headers.py` | Prints one header, size and encoding per image topic |
| `peek_sonar_values.py` | dB statistics of `polar_db`, and a test of whether `polar` (mono8) is a linear mapping of `polar_db` |
| `run_with_load.sh` | Runs a command while logging tegrastats (1 Hz) and `nvidia-smi` GPU utilisation (1 Hz) |
| `summarize_load.py` | Summarises the logs written by `run_with_load.sh` |
| `results/*.json` | Summaries from the M7 run of 2026-11-09 (`smoke.json` is a 10 s smoke test) |

All scripts write to `results/` next to themselves. Raw logs (`*.log`) are ignored by `results/.gitignore`.

## Running on the Thor

Run each command in a fresh shell, from the repo root:

```bash
source ~/hil_env.sh                      # domain 73, Fast DDS, unicast profile

python3 hil/thor/m7/peek_headers.py
python3 hil/thor/m7/peek_sonar_values.py

# rates / gaps / bandwidth / RTF: 120 s window after a 5 s warm-up, with load logging
hil/thor/m7/run_with_load.sh step2 python3 hil/thor/m7/measure.py --duration 120 --warmup 5
python3 hil/thor/m7/summarize_load.py step2

python3 hil/thor/m7/simtime_check.py --ros-args -p use_sim_time:=true -p run_wall_s:=30.0

# consumer: 120 s, threshold 140 dB, minimum blob area 20 px
hil/thor/m7/run_with_load.sh step4 python3 hil/thor/m7/m7_consumer.py --ros-args \
    -p use_sim_time:=true -p threshold_db:=140.0 -p min_area_px:=20 -p run_wall_s:=120.0
python3 hil/thor/m7/summarize_load.py step4
# while it runs, in another sourced shell:
ros2 topic hz -s --window 100 /uw/m7/detections     # sim time
ros2 topic hz --window 100 /uw/m7/detections        # wall time
```

Requirements on the Thor:
- ROS 2 Jazzy: `rclpy`, `sensor_msgs`, `std_msgs`, `rosgraph_msgs`, `vision_msgs` (`ros-jazzy-vision-msgs`)
- Python: `numpy`, `opencv-python` (cv2)
- `tegrastats` and `nvidia-smi`, for `run_with_load.sh`

The M7 run used numpy 1.26.4, cv2 4.11.0 and vision_msgs 4.1.1. Do not run any of these while an f1tenth harness
batch is running (see the main README).

## M7 results (2026-11-09, 120 s, RELIABLE subscribers)

| Topic | Size | Rate wall / sim | Stamp gaps | Bandwidth |
|---|---|---|---|---|
| `/oculus/polar` | mono8 512x876, 448 KB | 9.986 / 10.000 Hz | 0 | 4.48 MB/s |
| `/oculus/polar_db` | 32FC1 512x876, 1.79 MB | 9.986 / 10.000 Hz | 0 | 17.93 MB/s |
| `/oculus/fan` | mono8 1024x566, 580 KB | 9.986 / 10.000 Hz | 0 | 5.79 MB/s |
| `/camera/image_raw` | rgb8 1280x720, 2.76 MB | 9.986 / 10.000 Hz | 0 | 27.63 MB/s |
| Total | | | | 55.8 MB/s (link: 5 Gb/s) |

- RTF from `/clock` (59.95 Hz): 0.9997, 10 s windows between 0.984 and 1.018.
- Latency proxy: image stamps equal the latest `/clock` received, to within one tick (camera: at most 4 ticks).
- `use_sim_time:=true` verified: `now()` equals `/clock`, and a 1 s timer fires at exactly 1.0 s of sim time.
- Consumer: 1200 sonar frames in, 1200 detection messages out, 0 gaps. About 4.5 ms per sonar frame and 5 ms per
  camera decode. 37 % of one core. GPU about 0 % (the consumer is CPU-only). `ros2 topic hz -s` reads 10.000 Hz.
- The largest wall interarrival (0.36 s) hit all four topics at the same moment, with no stamp gap. The cause was
  not tested.

## Known quirks

- On Jazzy, `ros2 interface show sensor_msgs/msg/Image` prints CompressedImage. This is a CLI bug: the lookup takes
  the first interface file that ends in `Image.msg`. The installed `Image.msg`/`.idl` and the Python class are the
  real Image; read them directly.
- The publishers run Humble, so on the Jazzy Thor `ros2 topic info -v` shows the node names as
  `_NODE_NAME_UNKNOWN_` and the topic type hash as `INVALID`. The history depth also shows as UNKNOWN.
- `polar_db` is NaN in rows 0-24 (range 0-0.096 m, the blind zone). `polar` is not a linear function of `polar_db`.
