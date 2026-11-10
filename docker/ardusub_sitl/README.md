# ArduSub SITL in Docker (HIL, M8a/M8b prerequisite)

Unforked ArduPilot **ArduSub-4.5.7** (the last 4.5.x release; commit `30257f01185471ab4c1ac544e47d1b4437e44c98`), SITL board,
JSON physics backend, BlueROV2 frame (`FRAME_CONFIG 1`, "vectored": 4 horizontal vectored + 2 vertical thrusters;
the Heavy is `FRAME_CONFIG 2`, "vectored_6dof"). Container `uw-ardusub`, non-root (uid 1000), no `--privileged`,
no host networking, no published ports, restart policy off, all capabilities dropped. Image 124 MB, cold build 146 s.

```
   Isaac Sim (Windows, native)                 Docker Desktop                         Jetson Thor
   or tools/stub_physics.py                    container uw-ardusub                   192.168.200.10
   binds 127.0.0.1:9002   <---- servo UDP ---- ardusub (client)
                          ----- JSON state --->  (reply to the packet's source)
                                                 ardusub --serial0 ----- MAVLink v2 UDP ----> MAVROS binds :14550
                                                 udpclient:192.168.200.10:14550  <---- replies ----
```

## Direction of the physics link (differs from the first brief)
In `libraries/SITL/SIM_JSON.cpp` (tag ArduSub-4.5.7) the SITL socket is **not bound**: SITL *sends* a servo packet to
`<target_ip>:9002` (target from `--model JSON:<ip>`) and the physics simulator replies to that packet's source address.
So the **simulator listens on UDP 9002**, SITL is the client, nothing is published by Docker, and the container reaches
the host through `host.docker.internal` (the entrypoint resolves it to an IP: ArduPilot's socket code wants a literal).
Docker Desktop delivers it to the host's loopback (the stub sees the peer as 127.0.0.1): **Isaac must bind `127.0.0.1:9002`
and reply to the sender**, no firewall rule involved. Servo packet: `<HHI16H` (magic 18458; 32 channels: magic 29569,
`<HHI32H`), reply: `"\n{json}\n"` in the exact format `tools/stub_physics.py` emits (the parser is `strstr`/`sscanf`,
not a JSON parser: `"[a, b, c]"` with `", "`, no space after `:`). **SITL is lockstep with the sim**: it blocks on the reply
and takes its clock from `"timestamp"`, so the physics' real-time factor is SITL's.

## MAVLink to the Thor: SITL pushes (`--serial0 udpclient:192.168.200.10:14550`)
Pushing needs no listening port on the work machine and no inbound rule beyond the replies that belong to the outgoing UDP
flow; publishing a port would put a MAVLink listener on the work machine's network and need an inbound rule. MAVROS on the
Thor binds `192.168.200.10:14550` (cable interface only) and learns the peer from the first packet.
`udpclient` takes an IPv4 literal only (a host name silently sends nothing: tested).

## Files
| File | Purpose |
|---|---|
| `Dockerfile` | two stages: build (toolchain, pinned tag, `waf configure --board sitl && waf sub`) and a small runtime with only `ardusub` + `sub.parm` + `sub-6dof.parm` |
| `entrypoint.sh` | resolves the physics host, assembles the `ardusub` command line from environment variables |
| `params/hil.parm` | defaults on top of ArduPilot's stock `sub.parm` (`FRAME_CONFIG 1`, `SYSID_THISMAV 1`); mounted read-only at `/params`, applied at each start (`--wipe`) |
| `run.bat` | `build`, `up` (default), `logs`, `stop`, `status`; paths with spaces are fine |
| `../../tools/stub_physics.py` | stdlib-only fake physics: a motionless vehicle at 1 m depth, paced at real time (`--rtf`) |
| `../../output/hil/firewall_sitl.bat` | `add` / `remove` / `show` (you run it as administrator) |
| `../../output/hil/thor_mavros_*` , `thor_mavros.launch.py` | Thor side (git-ignored with the rest of `output/`) |

Logs: `output\ardusub_sitl\` (`build.log`, `sitl.log`, `stub.log`).

## Run
```bat
rem from underwater_ws
docker\ardusub_sitl\run.bat build
docker\ardusub_sitl\run.bat            & rem start (detached)
docker\ardusub_sitl\run.bat logs       & rem save the log, show the tail
docker\ardusub_sitl\run.bat stop       & rem save the log, stop, remove
```
The equivalent `docker run` (what `run.bat up` executes):
```bat
docker run -d --name uw-ardusub --restart no --init --cap-drop ALL --security-opt no-new-privileges ^
  -e PHYSICS_HOST=host.docker.internal -e MAVLINK_OUT=udpclient:192.168.200.10:14550 ^
  -v "F:\users\andreas steffens\work\isaac_hil\underwater_ws\docker\ardusub_sitl\params:/params:ro" ^
  uw-ardusub:4.5.7
```
Other environment variables: `HOME_LOC` (lat,lon,alt,heading), `SPEEDUP`, `PARAM_FILES`, `WIPE`. Extra arguments after the image
name go straight to `ardusub`.

## Smoke test without Isaac (pass: `ros2 topic hz /mavros/state` on the Thor ~1 Hz, `connected: true`)
Work machine, two consoles in `underwater_ws` (any Python 3; the stub is stdlib only):
```bat
python tools\stub_physics.py > output\ardusub_sitl\stub.log 2>&1
docker\ardusub_sitl\run.bat
```
Thor (HIL terminal): `USE_SIM_TIME=false bash ~/thor_mavros_run.sh`, then in a second HIL terminal
`source ~/hil_env.sh; ros2 topic hz /mavros/state; ros2 topic echo /mavros/state --once`.
Without Isaac there is no `/clock`, so MAVROS must run with `use_sim_time:=false` for this test only.

Checked on the work machine (no Thor): stub + container + a loopback MAVLink listener, 19 heartbeats in 20 s, sysid 1, type 12
(submarine), autopilot ArduPilotMega, mode MANUAL. Not checked: the cable, MAVROS, the Thor scripts.

## Known limits
- The stub is not a vehicle: thruster PWM is counted, never integrated. The ROV model belongs to the Isaac side.
- ArduSub's GUIDED needs a position estimate; with the stub's fixed pose that may be refused. `thor_mavros_test.py --mode ALT_HOLD`
  (RC override) works without one. Choosing the real position source (simulated GPS, vision pose, DVL) is an M8a decision.
- Clock: SITL time follows the physics' timestamps. If Isaac runs at RTF 0.85, MAVLink (heartbeats, telemetry) runs at 0.85 in wall time.
