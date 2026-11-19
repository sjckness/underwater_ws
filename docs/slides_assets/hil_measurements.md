# HIL and link measurements already on disk

> Fotografia anteriore al 2026-11-09. Da allora il link work↔Thor su 192.168.200.x funziona (discovery ROS 2 confermata); mancano ancora le misure di throughput, RTT e banda.

Every number below is copied verbatim from a file in the repo or from a finished run output.
Nothing was re-run, re-measured or averaged; no rounding was applied beyond what the source
file already prints. Where a source gives a value to more digits than the prose does, both are
shown. Anything absent is written **not found**, with the places that were searched.

**The single most important fact for slide 6: the Jetson (thor) has never been connected.**
`docs/bridge_setup.md:697-698` — "nothing here has been run, because the Jetson is not connected
yet". So **every measurement in this file is loopback on `linus` (container ↔ host on one
machine) and is labelled "no Thor"**. There is no `192.168.9.x` traffic measurement of any kind.

Sources, all at commit `ff2f1a4` (2026-10-29) unless stated:

| short name | path |
|---|---|
| `bridge` | `docs/bridge_setup.md` |
| `hil` | `docs/ros_hil.md` |
| `map` | `docs/topic_map.md` |
| `plan` | `docs/PLAN.md` |
| `val` | `docs/validation.md` |
| raw run outputs | `output/probe/m7/*.json` (untracked, written 2026-10-29 13:26–15:42) |

Measurement commits, all dated **2026-10-29**: `6713131`, `93d65d8`, `668a1be`,
`fbae417` ("ros: measure the loopback link on domain 73"), `c51a8cb`, `56cb673`, `ff2f1a4`.
Branch `m7-topic-map` contains no commit that is not already in `main` (`git log main..m7-topic-map`
is empty), so there are no extra measurements in its history.

---

## a. Sonar and camera topics: bandwidth and rate in the HIL configuration

### a.1 Sonar, no-camera profile — **no Thor**, loopback on linus

Run: `python3 -m uw sim --sink ros --no-camera --duration 170` in the Isaac container
(`network_mode: host`, `ipc: host`, domain 73, `rmw_fastrtps_cpp`), relay and `link_monitor` on
the host. Whole run **170.0 s sim / 170.1 s wall, RTF 0.9995**, 1700 pings, 1 over budget.
40 s monitor window (`wall_s = 40.023`). Sonar at 10 Hz, tank/lf preset, 512 beams × 876 rows,
`/clock` nominal 60 Hz.

| topic | value | unit | QoS seen | n | mean bytes | lost | source |
|---|---|---|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` (mono8 polar) | **10.020** | Hz | best_effort/volatile | 402 | 448 572 | 0 | `bridge:440`; raw `output/probe/m7/link_nocam.json` |
| `/rov1/sensors/sonar_fls/image` | **36.04** | Mbit/s | | | | | same (`mb_s` 4.506) |
| `/rov1/sensors/sonar_fls/sonar_image` (`ProjectedSonarImage`) | **10.000** | Hz | best_effort/volatile | 400 | 468 488 | 0 | `bridge:441`; `link_nocam.json` |
| `/rov1/sensors/sonar_fls/sonar_image` | **37.46** | Mbit/s | | | | | same (`mb_s` 4.682) |
| `/rov1/sensors/sonar_fls/meta` | latched, 1 sample | — | reliable/transient_local | 1 | 11 664 | 0 | `bridge:442` |
| `/clock` | **60.059** | Hz | best_effort/volatile | 2402 | 12 | 0 | `bridge:443` |
| `/sim/diagnostics` | **1.000** | Hz | reliable/volatile | 40 | 439 | 0 | `bridge:445` |

Relay overhead, measured: **19 916 B** per ping (468 488 − 448 572) against the 19 888 B computed
from `relay.py` — "28 bytes of message header" (`bridge:474-475`).

### a.2 Camera — **no Thor**, loopback on linus

25 s windows, `--duration 110`, same scene. `camera_read_publish` is the sim-side encode+publish
time per frame.

| run / conditions | camera topic | Hz | payload Mbit/s | bytes/msg | lost | RTF (window) | `camera_read_publish` | source |
|---|---|---|---|---|---|---|---|---|
| raw, render 10 Hz, publish 10 Hz | `image_raw` | 5.036 | **109.46** | 2 764 872 | **86** | 0.8340 | 7.5 ms | `bridge:606`; `cam_raw_link.json`, `cam_raw.json` |
| PNG level 1, render 10 Hz, publish 10 Hz | `image_raw/compressed` | 8.338 | **18.13** | 272 675 | 0 | 0.8378 | 15.7 ms | `bridge:607,647`; `cam_png_link.json` |
| PNG level 1, render 10 Hz, publish 2 Hz (`ros.camera_hz: 2`) | `image_raw/compressed` | 1.664 | **3.64** | 272 025 | 0 | 0.8321 | 7.8 ms | `bridge:608`; `cam_png2hz_link.json` |
| PNG level 1, render 2 Hz (`camera.rate_hz: 2`) | `image_raw/compressed` | 2.001 | **4.18** | 261 663 | 0 | **0.9989** | 15.8 ms | `bridge:609`; the §6.3 in-loop table at `bridge:649` gives 263 176 B / **4.20** Mbit/s for the same run |
| **JPEG q85, render 10 Hz** | `image_raw/compressed` | 8.400 | **2.94** | 43 581 | 0 | 0.8435 | 9.4 ms | `bridge:648`; `cam_jpeg_link.json`, `cam_jpeg.json` |
| **JPEG q85, render 2 Hz** | `image_raw/compressed` | 2.000 | **0.69** | 43 285 | 0 | **1.0000** (window) | 9.4 ms | `bridge:650`; `cam_jpeg2_link.json` |

Whole-run RTF from the sim's own stats JSONs, which differ from the window values above and are
reported separately rather than reconciled: `cam_raw.json` 0.8340466620283347,
`cam_png.json` 0.8378373960538307, `cam_png2hz.json` 0.8320524866695203,
`cam_jpeg.json` 0.8434743982978086, `cam_jpeg2.json` **0.9988954196899447**,
`cam_render2.json` 0.9988778483197581.

Codec bench, offline, one saved frame, in the sim's own interpreter (`bridge:627-639`):
png level 1 0.735 MB / 3.76× / 20.7 ms; png level 3 0.638 MB / 4.33× / 37.3 ms;
**jpeg q75 0.057 MB / 48.5× / 3.1 ms**; **jpeg q85 0.073 MB / 38.0× / 3.1 ms**;
jpeg q95 0.139 MB / 19.9× / 3.9 ms. q85 against the original: mean absolute difference
**0.74 of 255**, worst pixel **41**, **PSNR 44.8 dB** (`bridge:640`).

PNG ratio on the live dark frames was **10.1×** (272 675 B against 2 764 872 B), taking the camera
from 221.2 to **21.7 Mbit/s** at nominal 10 Hz — and the doc states explicitly that this ratio is a
property of the scene, not the setting (`bridge:613-615`).

### a.3 Predicted vs measured, per-message size × nominal 10 Hz (`bridge:468-473`)

| topic | measured bytes/msg | → at 10 Hz | `hil` §2.2 predicted | error |
|---|---|---|---|---|
| polar `image` | 448 572 | 35.89 Mbit/s | 35.9 | 0.0 % |
| `sonar_image` | 468 488 | 37.48 | 37.5 | 0.1 % |
| `polar_db` | 1 794 108 | 143.53 | 143.5 | 0.0 % |
| `camera_front/image_raw` | 2 764 872 | 221.19 | 221.2 | 0.0 % |

---

## b. Total bandwidth on the HIL link against 1 Gbit/s, and `/gt` absent on the Thor side

### b.1 Measured total — **no Thor**, stack container ↔ host on linus

The closest thing to a HIL total that exists. Stack container built on linus (amd64, 1.31 GB),
`link_monitor` **inside** the container, sim in the recommended HIL configuration
(`ros.camera_transport=compressed`, `camera.rate_hz=2`), generated loopback DDS profile
bind-mounted (`bridge:668-691`):

| topic | rate | payload | bytes/msg | lost |
|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` | 10.026 Hz | 36.12 Mbit/s | 448 572 | 0 |
| `/rov1/sensors/sonar_fls/sonar_image` | 10.000 Hz | 37.43 Mbit/s | 468 488 | 0 |
| `/rov1/sensors/camera_front/image_raw/compressed` | 2.000 Hz | 4.20 Mbit/s | 263 176 | 0 |
| `/clock` | 60.022 Hz | 0.01 Mbit/s | 12 | 0 |
| `/sim/diagnostics` | 1.000 Hz | — | 459 | 0 |

**Stated total: "78 Mbit/s of 1 GbE, zero drops, every rate nominal"** (`bridge:689`).
Whole-run RTF for that run: **0.9987823976609405**, 110.00000879999999 s sim / 110.13410834793467 s
wall (`output/probe/m7/stack.json`). The doc is explicit that this does **not** test arm64, the
Ethernet, or the profile's `interfaceWhiteList`/`initialPeersList` (`bridge:690-693`).

### b.2 Computed budget against 1 GbE (`hil:111-118`, framing overhead ~8 % at MTU 1500, marked **U**)

| cell | polar rows | `sonar_image` | + camera | wire (+8 %) | of 1 GbE |
|---|---|---|---|---|---|
| tank / lf (3.5 m, 4 mm) | 876 | 37.5 Mbit/s | 258.7 | 40.5 / 279.4 | 4 % / 28 % |
| tank / hf (3.5 m, 2.5 mm) | 1401 | 59.1 | 280.3 | 63.8 / 302.7 | 6 % / 30 % |
| basin / lf (8 m, 4 mm) | 2001 | 83.8 | 305.0 | 90.5 / 329.4 | 9 % / 33 % |
| basin / hf (8 m, 2.5 mm) | 3201 | 133.4 | 354.6 | 144.1 / 383.0 | 14 % / 38 % |

For contrast, the configuration this replaces (polar + polar_db + fan + camera): **447 Mbit/s** at
tank/lf and **951.8 Mbit/s at basin/hf** (`hil:120-122`, marked **V**, computed).
`/oculus/polar` alone is **410 Mbit/s at 40 m** and **1229 Mbit/s at 120 m** (`hil:140-141`, **U**).

### b.3 `/gt` absent on the Thor side

**Not verified from a second host — no Thor.** What exists:

- `/gt` runs on its own ROS domain, `ros.gt_domain_id: 74`, with `ros.gt_localhost_only: true`
  (`config/tank.yaml:247`; `bridge:495-500`).
- Verified on linus, one sim, both domains listed at once (`bridge:509-515`): domain 73 carries
  `/clock`, `/tf_static`, `/rov1/sensors/**`, `/sim/diagnostics`, `/sim/debug/sonar_fan` and
  **"no `/gt` topic of any kind"**; domain 74 carries `/gt/{polar_db,sonar_pose,tf}`,
  `/gt/leak/{pose,flow_lpm}`, `/gt/plume/markers`, `/clock`. Measured flowing on domain 74:
  `polar_db` **56.34 Mbit/s**, `sonar_pose` and `tf` **8.25 Hz**, `plume/markers` **1.65 Hz**.
- The doc states the discovery-range half "still has to be verified from a second host"
  (`bridge:503-505`), and the Jetson checklist item that would do it is unticked
  (`bridge:762-766`: "`ros2 topic list` … and **no `/gt` topic of any kind**").
- `map:290-293` notes the enforcement is emergent, not a guarantee: "one `ros2 topic echo
  /gt/polar_db` from thor puts 143 Mbit/s on the link".

---

## c. End-to-end latency of the minimal closed loop

**Not found — the measurement does not exist, and cannot yet.** Two independent reasons, both
stated in the repo:

1. **The detector does not exist.** `/rov1/perception/bubbles/detections`
   (`vision_msgs/Detection2DArray`, publisher `leak_detector` on thor) is specified in
   `map:166` but is **M10, unstarted**: `hil:398` lists "Jetson decode + detector inference" as
   "**does not exist** (M10 unstarted)".
2. **Thor is not connected.** `bridge:790-791`: "Latency end to end is *not* in scope: the detector
   does not exist (M10), so there is nothing to time yet."

What **is** on disk is the **sim-side half of the budget only** (`hil:377-404`), written against
**linus, no camera, `--sink ros`, RTF 0.997 — no Thor**:

| term | mean | max | source |
|---|---|---|---|
| `gpu_wait_before_ping` (no camera) | 3.3 ms | — | **V**, `output/probe/sim_max--no-camera.json` |
| sonar ping compute, fixed pose, tank | 36.4 ms | 70.1 ms | **V** (plume-box paths 26.8 ms of it) |
| static recompute if the sonar moved, tank | **+133–154 ms** | — | **V**, `output/probe/mobile_timing.json` |
| the same, basin | **+291–352 ms** | — | **V** |
| publish, ROS sink headless | 9.0 ms | 21.8 ms | **V**, `output/probe/sim_ros_stats.json` |
| serialization + 1 GbE transfer, 448 kB payload | ~3.6 ms | — | **U**, computed at line rate |
| link RTT linus ↔ Jetson | — | — | **unmeasured** |
| Jetson decode + detector inference | — | — | **does not exist** |
| MAVLink one-way + ArduSub reaction | ~2.5 ms | — | **U** |
| ping period (quantization) | 100 ms | | **V** |
| `/clock` tick (quantization) | 16.7 ms | | **V** |
| `RosPose` polled once per ping | 100 ms | | **V** |

Stated total of the sim-side chain alone: **~49 ms mean / ~95 ms max** against a 100 ms ping period
(`hil:398-399`). The doc says plainly: "**The budget is not a number yet** — the detector does not
exist and the link is unmeasured" (`hil:406`).

**Sim time vs wall clock — the method, not a result** (`hil:289-296`): "A wall-clock delay `Δt` on
the Jetson appears inside the sim as `RTF · Δt` — **at RTF 0.694 a real 60 ms loop looks like
42 ms**, optimistic by 1/RTF." Two separate measurements are required, a sim-time stamp-to-stamp
chain and a wall-clock round trip measured on linus at both ends. Neither has been run.

Also relevant to a moving sonar (`hil:401-404`): 36 + 133 ms ≈ **170 ms per ping in the tank =
~5.9 Hz**, basin **~2.6 Hz**; on a trajectory the static cache recomputed on **40 of 40 pings**
at the default 0.15 m/s (pose moves 1.5 cm per ping against a 5 mm threshold).

Where I looked and found nothing: `output/probe/m7/*` (no detection or round-trip timing),
`output/probe/hz/*` (per-topic `ros2 topic hz` text dumps only), `bridge` §6.1/§6.3/§6.4,
`hil` §3.3/§5, `map` §4.5, git log of `main` and `m7-topic-map`.

---

## d. Thor link facts: ethtool, PTP, time sync

**All three are "not found" for thor.** The checklist entries that would produce them are unticked
(`bridge:703-707`):

> - [x] **Primary: thor** (decided 2026-10-29). Still to paste in here: `cat /etc/nv_tegra_release`,
>   `ip -br link`, `ethtool <nic>`, `ethtool -T <nic>`.
> - [ ] NIC, link speed, MTU and whether it has a PTP hardware clock: `ip -br link; ethtool <nic>; ethtool -T <nic>`.

`bridge:825-826`: "**Thor's and Orin's JetPack version, NIC and PTP capability are unknown**".

| wanted | thor | linus (the workstation, for reference — no Thor) | source |
|---|---|---|---|
| `ethtool <nic>` output | **not found** | NIC `enp5s0`, **1000 Mb/s**, **MTU 1500**, MAC `a8:5e:45:17:fe:e7`, read with `ip link` (**V**) | `bridge:48` |
| `ethtool -T <nic>` / PTP hardware clock | **not found** | **none** — software timestamping only (**V**, measured earlier) | `bridge:54`, `hil:299-300` |
| which time sync was used | **not found** (no run has happened) | No sync needed for the two measurements the method specifies: both the sim-time chain and the wall-clock round trip are measured at one end. For a one-way wall-clock number the doc names **chrony over the LAN**, floor "tens of µs to ~1 ms, i.e. ~1 % of a 100 ms ping period: adequate for a 10 Hz loop, not for sub-millisecond claims" | `hil:299-302` |
| address plan | `192.168.9.2` (thor), `192.168.9.1` (linus), /24, static, no gateway, no DHCP, **direct cable, no switch** — decided 2026-10-29, **not yet used** | | `bridge:712-716`, `bridge:708-710` |
| MTU on the HIL link | planned **1500** on both ends, unticked | 1500 | `bridge:717-719` |
| RTT / jitter linus ↔ thor | **not found / unmeasured** — "the first missing term of `docs/ros_hil.md` §5" | | `bridge:823-824`, `hil:524-527` |
| line rate (`iperf3` ≥ 900 Mbit/s) | **not found**, unticked | | `bridge:756-757` |
| `/proc/net/snmp` deltas | **not found**, unticked; no snmp capture exists in `output/probe/m7/` (grepped for `snmp`, `ReasmFails`, `RcvbufErrors`: no hits) | | `bridge:758-759` |

Socket buffers on linus, which the DDS profile is capped by:
`rmem_max = wmem_max = 4194304`, `rmem_default = 212992`; **not pinned on disk** — nothing in
`/etc/sysctl.conf` or `/etc/sysctl.d/` sets them (`bridge:50-51`). `config/sysctl/60-ros2-hil.conf`
exists but is **not installed** (`bridge:490-492`, `bridge:822`). IP fragment reassembly:
`ipfrag_high_thresh = 4194304`, `ipfrag_time = 30` (`bridge:53`).

---

## e. Write throughput on `/HDD2T`, and the GT bag compression choice

### e.1 Write throughput — **not found as a direct measurement**

`hil:552-554` (open question 13): "Raw capture is ~5.6 MB per ping period ≈ **56 MB/s** at 10 Hz,
and zlib was measured at **63 ms of a ~100 ms ping write** — inline compression cannot keep up at
ping rate. **Sustained write throughput on the target volume is unmeasured.**"
Repeated at `bridge:593-595`: "sustained write throughput on that volume is **still unmeasured**".

No `fio`, `dd`, or `hdparm` result exists anywhere in the repo or in `output/` (grepped for
`fio `, `dd if=`, `hdparm`, "write speed", "write throughput", "sustained write": the only hits are
the two "unmeasured" statements above and the per-topic MB/s rows).

The **read** figure that does exist, and must not be quoted as a write figure:
`plan:362` — "`/HDD2T` has 1.6 TB free (136 GB used after the recompression) and **reads cold at
186 MB/s**"; the root disk is at 93 % (31 GB free).

Free space, **V**, 2026-10-29 (`bridge:533-535`): `/` has **25 G free of 439 G (95 % used)**,
`/HDD2T` has **1.6 T free of 1.8 T**; datasets occupy 153 G. "A root-filesystem bag would fill the
disk in roughly twenty minutes."

Indirect, from the v0.2 recompression pass (`val:869-885`, **a disk-write workload, but the number
reported is a CPU bound**): 400/400 scenarios, 0 failures, 216.5 → 58.4 GB (201.7 → 54.4 GiB),
**3.71×**, 6.02 → 1.62 MB per ping, 158 GB freed; "~113 s per scenario over the real mix, so
**~13 h single-threaded for all 400**"; "the process sits at **95 % of one core: the pass is
CPU-bound, not disk-bound at 35–50 min**". 217 GB read + 62 GB written, projected (`val:863-866`).

### e.2 GT bag: what it carries, and the compression choice

Ground-truth domain throughput from the measured sizes and rates (`bridge:540-547`):

| topic | bytes/msg | rate | MB/s | GB/hour |
|---|---|---|---|---|
| `/gt/polar_db` | 1 794 108 | 10 Hz | 17.94 | **64.6** |
| `/gt/plume/markers` | 117 363 | 2 Hz | 0.235 | 0.85 |
| `/gt/sonar_pose` + `/gt/tf` | 76 + 108 | 10 Hz | 0.002 | 0.007 |
| `/gt/leak/pose` + `/gt/leak/flow_lpm` | 76 + 8 | 1 Hz | — | — |
| `/clock` | 12 | 60 Hz | 0.001 | 0.003 |
| **total** | | | **18.2** | **65.5** |

`polar_db` is **98.6 %** of it (`bridge:549`).

zstd measured on real `polar_db` frames (12 frames of `v0.1/s0043`, the `zstd` CLI, one core;
`bridge:554-560`):

| mode | ratio | throughput | GB/hour (polar_db) | CPU at 17.9 MB/s in |
|---|---|---|---|---|
| file-level zstd -1 | 2.50× | 192.6 MB/s | 25.8 | 9.3 % of a core |
| file-level zstd -3 | 2.63× | 140.8 MB/s | 24.6 | 12.7 % |
| file-level zstd -9 | 2.95× | 31.2 MB/s | 21.9 | **57 %** |
| **message-level zstd -1** | **2.51×** | 152 MB/s | 25.8 | ~12 % |
| message-level zstd -3 | 2.64× | 114 MB/s | 24.5 | ~16 % |

Options priced (`bridge:572-580`): A raw 65.5 GB/h, 24 h on 1.6 T · **B message-level zstd -1
26.7 GB/h, 61 h, ~12 % of a core** · C message-level zstd -3 25.4 GB/h, 64 h, ~16 % ·
D `polar_db` at 2 Hz + zstd -1 6.1 GB/h, 262 h (needs an unimplemented `ros.gt_polar_db_hz` key) ·
E `polar_db` out of the bag, kept in the dataset `npz`, 0.9 GB/h, ~1800 h.

> **Final choice as recorded (`bridge:582-585`): "Recommendation: B for routine runs, D for a long
> session."** B = `ros2 bag record --compression-mode message --compression-format zstd`, written
> to `/HDD2T` (`bridge:586-590`). Message-level is chosen over file-level because it costs nothing
> in ratio at this message size (2.51× against 2.50×) and avoids file-level's transient disk spike
> (`bridge:562-566`).

Why only 2.5× (`bridge:566-571`): the topic is `32FC1` but the source array is **float16** in the
recorder, so half of every mantissa on the wire is zero padding; raw float16 is **10.76 against
21.53 MB over 12 frames** and still compresses **1.43×** at zstd -1.

The related dataset-side choice, already in force: recorder keeps `compress=false` /
`png_level=0` during a sweep (zlib was **63 of ~100 ms per write**) and recompresses afterwards as
a separate pass, measured **3.47× projected / 3.71× as run** (`val:906-908`, `val:870-871`).

The recorder/bag writer needs the §4 DDS profile like every other subscriber: `/gt/polar_db`
"dropped **64 of ~250** samples on the ground-truth domain with default socket buffers"
(`bridge:528-529`, `bridge:593-596`).

---

## f. Packet loss / dropped messages per topic, and RTF with and without the camera

All loss figures are `lost` from `QoSMessageLostInfo` per subscription, read by
`ros/link_monitor`. **All loopback on linus — no Thor.** No `/proc/net/snmp` or `ip -s link` delta
was captured in any run (see §d).

### f.1 No-camera profile, 40 s window (`bridge:439-451`; `output/probe/m7/link_nocam.json`)

| topic | n | rate | payload | mean bytes | **lost** |
|---|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` | 402 | 10.020 Hz | 36.04 Mbit/s | 448 572 | **0** |
| `/rov1/sensors/sonar_fls/sonar_image` | 400 | 10.000 Hz | 37.46 Mbit/s | 468 488 | **0** |
| `/rov1/sensors/sonar_fls/meta` | 1 | latched | — | 11 664 | **0** |
| `/clock` | 2402 | 60.059 Hz | 0.01 | 12 | **0** |
| `/tf_static` | 1 | latched | — | 356 | **0** |
| `/sim/diagnostics` | 40 | 1.000 Hz | — | 439 | **0** |
| `/gt/sonar_pose` | 400 | 10.000 Hz | — | 76 | **0** |
| `/gt/tf` | 400 | 10.000 Hz | — | 108 | **0** |
| `/gt/plume/markers` | 80 | 2.000 Hz | 1.88 | 117 363 | **0** |
| `/gt/leak/pose` | 40 | 1.000 Hz | — | 76 | **0** |
| `/gt/polar_db` | 338 | 8.467 Hz | 121.21 | 1 794 108 | **61** |

### f.2 Camera profile, same scene, 40 s window (`bridge:456-462`; `link_cam.json`)

| topic | n | rate | payload | mean bytes | **lost** |
|---|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` | 316 | 7.880 Hz | 28.34 Mbit/s | 448 572 | **0** |
| `/rov1/sensors/sonar_fls/sonar_image` | 316 | 7.879 Hz | 29.60 Mbit/s | 468 488 | **0** |
| `/rov1/sensors/camera_front/image_raw` | 215 | 5.361 Hz | 118.85 Mbit/s | 2 764 872 | **100** |
| `/rov1/sensors/camera_front/camera_info` | 315 | 7.873 Hz | 0.02 | 384 | **0** |
| `/gt/polar_db` | 269 | 6.755 Hz | 96.49 Mbit/s | 1 794 108 | **45** |
| `/clock` | 1890 | 47.311 Hz | — | 12 | **0** |

`link_cam.json` additionally records `/sim/diagnostics` 32 msgs / 0.786 Hz / 0 lost,
`/gt/sonar_pose` and `/gt/tf` 315 msgs / 7.866 Hz / 0 lost, `/gt/plume/markers` 63 msgs /
1.573 Hz / 1.47 Mbit/s / 0 lost, `/gt/leak/pose` and `/gt/leak/flow_lpm` 32 msgs / 0.786 Hz /
0 lost, `/tf_static` and `meta` latched / 0 lost.

### f.3 The cause of the loss, A/B on the same run, reader side only (`bridge:427-432`)

| reader | `/gt/polar_db` | rate | payload | lost |
|---|---|---|---|---|
| default socket buffers | 243 msgs | 9.758 Hz | 139.14 Mbit/s | **6** |
| `FASTRTPS_DEFAULT_PROFILES_FILE` with `receiveBufferSize` 4 MB | 250 msgs | **10.000 Hz** | **143.46 Mbit/s** | **0** |

Stated rule (`bridge:424-426`): "**BEST_EFFORT samples above ~1 MB are lost without a DDS profile.**
The 448 kB sonar ping lost nothing; the 1.79 MB `/gt/polar_db` and the 2.76 MB camera frame did."
The same run's 448 kB sonar topic was unaffected either way (36.0 Mbit/s, 0 lost).
Compression removes the camera loss by making the sample small: raw lost **86 of ~210**; at 273 kB
(PNG) and 43 kB (JPEG) three runs lost nothing (`bridge:620-621`).
**The camera case has not been retested with the profile** (`bridge:488`, `bridge:819-821`).

### f.4 RTF with and without the camera

| condition | RTF | sim / wall | source |
|---|---|---|---|
| no camera, `--sink ros`, whole run | **0.9995** | 170.0 s / 170.1 s | `bridge:435`; `sim_nocam.json` gives 0.9994720113262269, 170.0000136 s / 170.08981909800787 s |
| no camera, nothing attached | **1.001** | | `bridge:487` |
| no camera, relay + `link_monitor` subscribing | **0.9963** (0.4 % cost) | | `bridge:487` |
| camera raw 10 Hz, 40 s window | **0.787** | | `bridge:452` |
| camera raw 10 Hz, whole run incl. startup transient | **0.683** | | `bridge:452` |
| camera raw, separate 120 s run | **0.8340466620283347** | 120.0000096 s / 143.876853734022 s | `cam_raw.json` |
| camera PNG level 1, 10 Hz | **0.8378373960538307** | 110.0 s / 131.29040231206454 s | `cam_png.json` |
| camera PNG, publish capped 2 Hz (`ros.camera_hz`) | **0.8320524866695203** | | `cam_png2hz.json` |
| camera PNG, render 2 Hz (`camera.rate_hz`) | **0.9989** / **0.9988778483197581** | 110.0 s / 110.12358416500501 s | `bridge:609,649`; `cam_render2.json` |
| camera JPEG q85, render 10 Hz | **0.8434743982978086** | | `cam_jpeg.json` |
| **camera JPEG q85, render 2 Hz** | **1.0000** (window) / **0.9988954196899447** (whole run) | 110.0 s / 110.12164700299036 s | `bridge:650`; `cam_jpeg2.json` |
| stack container, recommended HIL config | **0.9987823976609405** | 110.0 s / 110.13410834793467 s | `stack.json` |
| sim with camera, longer run `sim_cam.json` | **0.4801943829938265** | 140.0000112 s / 291.5486231370596 s | `output/probe/m7/sim_cam.json` |
| **12 concurrent `ros2 topic hz` processes** | **0.5** | | `hil:409`, `bridge:356` |

Two findings attached to these (`bridge:418-422`, `bridge:622-626`):

- "**Every rate on the wire is nominal × RTF.**" At RTF 0.787: sonar 7.88 Hz, `/clock` 47.3 Hz
  instead of 60, plume 1.573 Hz, leak 0.786 Hz — "all 0.787 of nominal".
- "**The two rate knobs are not interchangeable.**" `ros.camera_hz` caps the *publish* rate (bytes
  off the link, RTF unchanged at 0.832); `camera.rate_hz` caps the *render* and is where the cost
  is — at 2 Hz it gives **RTF 0.9989 with the sonar at 10.028 Hz and `/clock` at 60.081 Hz**.

### f.5 The recommended HIL camera configuration, as recorded (`bridge:656-660`)

`ros.camera_transport: compressed`, `ros.camera_compression: jpeg` (q85),
`camera.rate_hz: 2` → **0.69 Mbit/s, no drops, RTF 1.0000, full sonar rate**.
Current repo defaults are still `camera_transport: raw`, `camera_compression: png`,
`camera_jpeg_quality: 85`, `camera_hz: null` (`config/tank.yaml:250-254`), with
`camera.rate_hz: 10.0`, `sonar.ping_rate_hz: 10.0`, `ros.clock_hz: 60.0`,
`ros.plume_hz: 2.0`, `ros.leak_hz: 1.0`, `ros.diagnostics_hz: 1.0`
(`config/tank.yaml:138,243,244,246,256,269`).

---

## Summary of everything marked "not found"

| # | item | why | where I looked |
|---|---|---|---|
| 1 | Any measurement over the `192.168.9.x` cable | Thor has never been connected | `bridge` §8 (`:697`), §9 (`:814-826`); `output/probe/m7/` (all runs domain 73, loopback) |
| 2 | Thor `ethtool` / `ethtool -T` / PTP capability / JetPack version / NIC | checklist unticked | `bridge:703-707`, `bridge:825-826` |
| 3 | Time sync actually used on a HIL run | no HIL run exists; chrony named as the plan only | `hil:299-302` |
| 4 | RTT, jitter, `iperf3` line rate linus ↔ thor | unticked, "unmeasured" | `bridge:756-757`, `bridge:823-824`, `hil:524-527` |
| 5 | End-to-end closed-loop latency (sim time or wall clock) | detector is M10, unstarted; Thor absent | `hil:398,406`, `bridge:790-791`, `map:166` |
| 6 | `/gt` confirmed absent **on the Thor side** | verified on linus across two domains only | `bridge:503-505,509-515`, `bridge:762-766` |
| 7 | Sustained **write** throughput on `/HDD2T` | explicitly "unmeasured"; no fio/dd/hdparm anywhere | `hil:552-554`, `bridge:593-595`; repo-wide grep |
| 8 | `/proc/net/snmp` and `ip -s link` deltas for any run | unticked; no capture in the run outputs | `bridge:758-759`, `output/probe/m7/` grep |
| 9 | Camera frame retested with the §4 DDS profile | stated as untested | `bridge:488`, `bridge:819-821` |
| 10 | Raw `link_monitor` JSON for the stack-container run (§6.4) | only `stack.json` (sim stats) and `stack.log` were kept; the §6.4 table exists in the doc only | `output/probe/m7/` |
