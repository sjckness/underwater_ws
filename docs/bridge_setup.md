# The ROS 2 bridge: Isaac Sim (Windows) ↔ the ROS 2 containers (M7 step 1, task 1)

> **Status (2026-10-29): a runbook, not a result.** Nothing on the Windows side exists yet — the Isaac scene, the
> plume and the sonar model were built in this repo against the **Linux container on linus**
> (`docker/compose.yaml`), and the ROS 2 side of a Windows install has never been started. Everything in §1 is
> **measured on linus today**; everything in §2–§5 is a procedure with a check beside it. Numbers tagged **V** are
> traceable to a file, a datasheet, a doc page or a measurement taken here; **U** are assumed or inferred, as in
> `docs/validation.md`.
>
> §2 contains **four checks that can invalidate the premise of this task**, and the second one is not a detail:
> our sonar publishes through `rclpy` *inside the sim process*, and that path may not exist on a Windows Isaac at
> all. Read §2 before doing any of §3–§5.

Companion documents: `docs/ros_hil.md` (what crosses the link, QoS, bandwidth, clock, latency — M7 step 1's
analysis) and `docs/topic_map.md` (the proposed topic and frame map). This file is only about **making the
transport work and proving it with measurements**.

## 0. The topology this document configures

| host | hardware / OS | role | ROS 2 |
|---|---|---|---|
| **workstation** | Windows 10/11, RTX 4090 24 GB (`config/profiles/workstation.yaml`) | Isaac Sim 5.1 **native**: scene, plume, camera, the sonar model | internal bridge libraries — **which distro is check 1** |
| **thor** | Jetson AGX Thor, JetPack 7 / Ubuntu 24.04 (**U**, to confirm) | **primary** stack container: detector, estimator, mission, link monitor | ROS 2 Jazzy in a container (arm64) |
| **orin** | Jetson Orin, JetPack 6 / Ubuntu 22.04 (**U**, to confirm) | second stack container | Jazzy **in the container**, whatever the host's L4T ships |
| **linus** | Ubuntu, RTX 5050 8 GB, `enp5s0` 1 GbE | `ros/sonar_relay`, rosbag2, and the **fallback sim host** (what the repo does today) | host Jazzy at `/opt/ros/jazzy` (**V**) |

Two structural facts about the containers, because the word "the container" now means two different things:

1. **The sim container is amd64-and-NVIDIA-runtime only.** `docker/compose.yaml` runs
   `isaac_ws_devcontainer-isaac-sim:latest` with `runtime: nvidia`, `network_mode: host`, `ipc: host` and the
   workspace bind-mounted at the same path (**V**). It will not run on Thor or Orin, and it has **no `/opt/ros`**
   — it reaches ROS only through Isaac's bundled Jazzy `rclpy` (`uw/ros_bridge.py:ensure_rclpy_path`, **V**).
2. **The stack container is a plain ROS 2 Jazzy container** (arm64 on the Jetsons, amd64 on linus) that needs
   `rclpy`, `marine_acoustic_msgs` and later MAVROS — and does **not** need Isaac. It does not exist in this repo
   yet: `ros/sonar_relay` is run from the host with `run.sh`, which sources `/opt/ros/jazzy` (**V**).

So the link under test is **one publisher host → up to three subscriber hosts on a shared Ethernet**, and the
publisher host is the one variable this task actually changes.

## 1. The baseline on linus, measured 2026-10-29

| fact | value | how it was read |
|---|---|---|
| host ROS 2 | Jazzy at `/opt/ros/jazzy`, `ros2` on `PATH` | `ls /opt/ros`, `which ros2` (**V**) |
| RMW | `rmw_fastrtps_cpp` | `ros2 doctor --report` (**V**) |
| `ROS_DOMAIN_ID` in the login shell | **not set** → the CLI sits on **domain 0** | `env` (**V**) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` (exported) | `env` (**V**) — `LOCALHOST` would silently kill the link |
| NIC | `enp5s0`, 1000 Mb/s, **MTU 1500**, `a8:5e:45:17:fe:e7` | `ip link` (**V**) |
| address | `192.168.177.140/24`, gateway `.254` | `ip -4 addr` (**V**) |
| subnet occupancy | **20+ other hosts** in the ARP cache | `ip neigh` (**V**) — this is a shared lab LAN, not a HIL segment |
| socket buffers | `rmem_max = wmem_max = 4194304`, `rmem_default = 212992` | `sysctl` (**V**) |
| …pinned on disk? | **no** — nothing in `/etc/sysctl.conf` or `/etc/sysctl.d/` sets them | `grep` (**V**); they will not survive a reboot |
| IP fragment reassembly | `ipfrag_high_thresh = 4194304`, `ipfrag_time = 30` | `sysctl` (**V**) |
| PTP hardware clock on `enp5s0` | **none** (software timestamping only) | `docs/ros_hil.md` §3.3 (**V**, measured earlier) |
| DDS profile XML in the repo | **none anywhere** | `find . -name '*.xml'` → only `package.xml` (**V**) |
| traffic on the wire now | nothing: `ros2 topic list` on domains **73** and **0** returns only `/parameter_events` and `/rosout` | run today (**V**) |
| containers running | none | `docker ps -a` empty (**V**) |
| images present | only `isaac_ws_devcontainer-isaac-sim:latest` | `docker images` (**V**) |

**The first trap is already visible in that table.** The domain is set *inside* the sim container
(`UW_ROS_DOMAIN_ID`, default **73**, `docker/compose.yaml`, **V**) and passed explicitly by the relay
(`relay.py:85`, **V**), but a plain login shell on linus has **no `ROS_DOMAIN_ID`**. So `ros2 topic list` typed
without a prefix talks to **domain 0** and shows nothing, with no error — and domain 0 is an f1tenth domain
(`docker/compose.yaml`'s own comment, **V**). Every command in §6 therefore carries the domain explicitly.

## 2. Four checks on the Windows side, before anything else

### Check 1 — which internal ROS 2 distro the Windows install actually has

```powershell
$env:ISAAC = "C:\isaacsim"          # wherever Isaac Sim 5.1 is installed
dir "$env:ISAAC\exts\isaacsim.ros2.bridge"        # look for humble\ and/or jazzy\ subfolders
dir "$env:ISAAC\exts\isaacsim.ros2.bridge\*\lib"
```

NVIDIA's **5.1 installation page lists Windows 10 and Windows 11 with Humble only**, and says internal Humble
loads automatically on Windows; its Windows recipe for anything else is ROS 2 **in WSL2 on Ubuntu 22.04** (**V**,
docs.isaacsim 5.1 `install_ros.html`). Our stack is **Jazzy** — `ros/sonar_relay` runs on host Jazzy with
`ros-jazzy-marine-acoustic-msgs` (**V**), and the Jetson containers are Jazzy by choice. Community reports do
have 5.1 on Windows running with the internal **Jazzy** libraries selected in the launcher (**U**, a third-party
write-up, not NVIDIA), so the folder listing above is the only answer that counts.

If only `humble` exists, the options are: run the whole stack on Humble (changes `marine_acoustic_msgs` and MAVROS
package selection, and `docs/ros_hil.md` §7.4's three-target message problem becomes four), or publish from a
Jazzy process elsewhere (check 2), or accept a Humble-publisher ↔ Jazzy-subscriber graph — which does usually work
for standard messages over RTPS but is a combination nobody tests, and it is **not** a configuration to discover
by debugging a silent topic.

### Check 2 — is `rclpy` importable from Isaac's Python on Windows? (the blocking one)

Our entire ROS surface is a **Python publisher inside the sim process**. `RosSink` does
`ensure_rclpy_path()` → appends `{isaac_root}/exts/isaacsim.ros2.bridge/{ROS_DISTRO}/rclpy` to `sys.path`, strips
`/opt/ros`, then `import rclpy` and `create_publisher` for every topic (`uw/ros_bridge.py`, **V**). There is no
OmniGraph ROS 2 bridge node in this project at all, and there is no stock node that could publish a numpy array
produced by our Warp sonar kernel — the bridge's graph nodes publish render products, TF and clock, not arbitrary
arrays.

```powershell
cd $env:ISAAC
dir exts\isaacsim.ros2.bridge\jazzy\rclpy        # or humble\rclpy — does this directory exist on Windows?
.\python.bat -c "import sys; sys.path.append(r'$env:ISAAC\exts\isaacsim.ros2.bridge\jazzy\rclpy'); import rclpy; rclpy.init(); print(rclpy.create_node('probe').get_name())"
.\python.bat -c "import sys; print(sys.version)"  # 3.11 for Isaac Sim 5.1 (U) — the ABI any ROS 2 wheel must match
```

NVIDIA's own custom-message / `rclpy` guidance says that workflow is **"not supported" on Windows (WSL)** and is
supported on the Pixi-based install, and that a ROS 2 workspace used from Isaac's Python must be **built against
the same Python version** (**V**, docs.isaacsim `tutorial_ros2_custom_message_python.html`). A ROS 2 Windows
binary release is built for a different Python than Isaac's, so "just install ROS 2 for Windows and import it"
is not a free move either.

> **If check 2 fails, there is no sonar on ROS from a Windows Isaac**, and the three honest ways out are:
>
> **(A) Keep the sim on linus** — `docker/compose.yaml` as it is today, Isaac in the Linux container, and the
> Ethernet carries sim → thor/orin. Nothing in the repo changes, the publisher path is the one the whole project
> was measured with, and the Windows box stays a scene-authoring machine. Cost: the camera profile and the
> workstation-class scene stay off the RTX 4090, and `docs/ros_hil.md` §0's "the camera profile moves to the
> workstation" stays unfulfilled.
>
> **(B) Isaac Sim 6.x on Windows with the Pixi-based native ROS 2 Jazzy**, which is the only configuration
> NVIDIA documents as supporting custom `rclpy` work on Windows (**V**, 6.0 docs). Cost: an Isaac version bump on
> the publisher host while linus stays on 5.1, i.e. two Isaac versions to keep the scene valid against.
>
> **(C) A socket side-channel**: the Windows sim writes pings to a local TCP/UDP socket, a publisher node in WSL2
> (or on linus) turns them into ROS 2 messages. Cost: an extra serialization hop and copy inside the latency
> budget of `docs/ros_hil.md` §5, a second process to keep alive, and — worse — a second place where sim time can
> be lost, when §3.1 of that document makes Isaac's clock the single authority.
>
> **Recommendation: prove the link with (A) first.** The transport questions in §3–§6 — domain, RMW, DDS profile,
> firewall, fragmentation, QoS, bandwidth — are *identical* whichever host publishes, and (A) is the only
> configuration that can be run today. Then treat Windows-native Isaac as a separate port whose first task is
> check 2, not as a prerequisite for M7.

### Check 3 — addresses and topology

```powershell
ipconfig /all                       # Windows: IPv4 of the physical NIC, and every Hyper-V/WSL virtual adapter
```
```bash
ip -4 addr; ip link                 # thor, orin: address and MTU
ethtool <nic>; ethtool -T <nic>     # link speed; PTP hardware clock (docs/ros_hil.md §7 item 1)
ping -c 100 -i 0.01 <peer>          # RTT and jitter — the first unmeasured term of ros_hil §5
```

What is needed in writing: the Windows IPv4, thor's and orin's, whether all of them sit on `192.168.177.0/24`
with the other 20+ lab hosts or on a dedicated HIL segment, and whether a switch is in the path. On a shared LAN
the multicast discovery of **every** host on domain 73 reaches **every** other, which is exactly how the f1tenth
coexistence problem in `docs/ros_hil.md` §7 item 14 turns into a HIL problem.

### Check 4 — which side of a NAT the subscriber is on

If the stack container ever runs on the Windows box through Docker Desktop/WSL2, `network_mode: host` does **not**
mean what it means on Linux: the container is inside the WSL2 VM behind NAT, DDS multicast does not cross it, and
NVIDIA's own answer is `netsh interface portproxy` entries per DDS port (**V**, 5.1 docs) — plus the same page
stating that **Docker workflows on WSL2 are not supported**. Mirrored WSL2 networking removes the NAT, but it is
a Windows-wide setting and untested here (**U**). Given check 1's answer the stack containers live on thor, orin
and linus, which are all Linux hosts where `network_mode: host` is literal; keep it that way.

## 3. Shared configuration — every host, every terminal, no exceptions

| variable | value | why |
|---|---|---|
| `ROS_DOMAIN_ID` | `73` | the project's own domain; **never** 0, 42, 85 or 87 (f1tenth) (**V**) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | what the sim container sets and the only RMW Isaac's bundled libraries provide (**V**) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` | already on linus; `LOCALHOST` silently breaks a multi-host graph (**V**) |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | the §4 profile's absolute path | NVIDIA requires it "in every terminal that passes ROS 2 messages" for multi-machine setups (**V**) |
| `ROS_DISTRO` | `jazzy` (or `humble`, per check 1) | Isaac uses it both to find its internal libraries and to detect a sourced ROS (**V**) |
| `use_sim_time` | `true` on **every** node except the sim itself | nothing in the repo declares it today (**V**, `docs/ros_hil.md` §1) |

A mismatch in any of the first three presents as **an empty topic list with no error**, which is why §6 reads
them back rather than assuming them.

**The UDP ports for domain 73**, because a firewall rule needs numbers. Fast DDS computes
`port = 7400 + 250·domain + offset`, with offsets 0 (multicast metatraffic), 10 + 2·participantId (unicast
metatraffic), 1 (multicast user data) and 11 + 2·participantId (unicast user data) (**V**, Fast DDS port
algorithm). For domain 73 (`250·73 = 18250`):

| traffic | port |
|---|---|
| discovery, multicast — group **239.255.0.1** | **25650** |
| discovery, unicast | **25660 + 2·n** |
| user data, multicast | **25651** |
| user data, unicast | **25661 + 2·n** |

`n` is the participant id on that host, counting from 0. Opening **UDP 25650–25760** inbound and outbound covers
roughly 50 participants per host (**U**, the upper bound is a choice, not a limit).

## 4. The Fast DDS profile

Two separate reasons to need one, and they want different things in it:

1. **Two hosts.** Fast DDS's default enables shared memory, which only works within one machine; NVIDIA's
   multi-machine instruction is a profile with a single UDPv4 transport and `useBuiltinTransports` **false**
   (**V**, 5.1 docs). Without it, participants discover each other and **no data arrives** — the single most
   reported Isaac-Sim-to-another-host symptom.
2. **Multicast that does not work** — a switch without IGMP snooping, a Windows host choosing a Hyper-V/WSL
   virtual adapter, or a routed path. Then discovery needs **explicit unicast peers**, and the interface the
   participant is allowed to use needs pinning.

Save this as `config/dds/fastdds_hil.xml` (**not created in this commit**: the `<address>` entries need the
Windows, thor and orin addresses from check 3, and a profile with wrong peers fails *more* confusingly than no
profile at all):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_only</transport_id>
        <type>UDPv4</type>
        <!-- raise with net.core.rmem_max / wmem_max (4194304 on linus, V) -->
        <sendBufferSize>1048576</sendBufferSize>
        <receiveBufferSize>4194304</receiveBufferSize>
        <maxMessageSize>65500</maxMessageSize>
        <!-- one line per host, that host's own NIC address; keeps Fast DDS off Hyper-V/WSL/docker0/virbr0/tailscale0 -->
        <interfaceWhiteList>
          <address>192.168.177.140</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="hil" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_only</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
        <builtin>
          <!-- unicast discovery: uncomment when multicast is proven not to work (§7) -->
          <!--
          <metatrafficUnicastLocatorList>
            <locator><udpv4><address>192.168.177.140</address></udpv4></locator>
          </metatrafficUnicastLocatorList>
          <initialPeersList>
            <locator><udpv4><address>WINDOWS_IP</address></udpv4></locator>
            <locator><udpv4><address>THOR_IP</address></udpv4></locator>
            <locator><udpv4><address>ORIN_IP</address></udpv4></locator>
          </initialPeersList>
          -->
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
```

Three notes that save an afternoon:

- **The file is per host**, because `interfaceWhiteList` and the metatraffic locator carry that host's own
  address. Keep one file per host, same filename, or template it from the launcher.
- A peer address with no port probes only the **first few participant ids** on that host
  (`maxInitialPeersRange`, a small default) (**U**) — with several nodes per host, either raise it or list
  `<port>` explicitly. This is the usual cause of "it discovers *some* of the nodes".
- For large samples, Fast DDS's documented remedy is `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` (TCP for user data,
  UDP discovery), and it **fails participant creation if combined with any other builtin-transport selection** —
  i.e. it is an alternative to this profile's `userTransports`, not an addition (**V**, `docs/ros_hil.md` §2.3).
  Reach for it only if §7's fragmentation symptom appears.

Also pin the socket buffers on both Linux hosts, since nothing on disk sets them today (**V**):

The file is in the repo as `config/sysctl/60-ros2-hil.conf`, with the reasoning for each value and for the one
value it deliberately leaves alone (`rmem_default`, since the DDS profile sets its own socket sizes and this host
is shared with the f1tenth stacks):

```bash
sudo install -m 644 config/sysctl/60-ros2-hil.conf /etc/sysctl.d/60-ros2-hil.conf && sudo sysctl --system
sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_high_thresh    # read them back
```

## 5. Per-host procedure

### 5.1 The publisher — the sim

**(A), today, on linus:** `python -m uw sim --sink ros` (or `python -m uw sonar-only --sink ros`). The container
sets `ROS_DOMAIN_ID` from `UW_ROS_DOMAIN_ID` (default 73), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
`ROS_DISTRO=jazzy` and runs with `network_mode: host`, so DDS sees `enp5s0` directly (**V**). Two additions are
needed and neither exists yet: `FASTRTPS_DEFAULT_PROFILES_FILE` must be added to the compose `environment:` with
the file bind-mounted, and `RosSink` should pass `domain_id` explicitly to `rclpy.init()` as `relay.py:85` does —
today it only *reads* the variable and logs a default of `0` (**V**, `docs/ros_hil.md` §2.3).

**Windows, after check 2 passes:**

```powershell
$env:ISAAC = "C:\isaacsim"
$env:ROS_DISTRO = "jazzy"                       # or humble, per check 1
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
$env:ROS_DOMAIN_ID = "73"
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = "SUBNET"
$env:FASTRTPS_DEFAULT_PROFILES_FILE = "C:\path\to\fastdds_hil.xml"
$env:PATH = "$env:PATH;$env:ISAAC\exts\isaacsim.ros2.bridge\$env:ROS_DISTRO\lib"   # once per terminal only
.\isaac-sim.bat --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge
```

Appending the `lib` path more than once per terminal is called out by NVIDIA as a source of conflicts (**V**).
The repo side would run with `UW_RUNTIME=native` and `native.python` pointing at Isaac's `python.bat`
(`config/runtime.yaml`, **V**) — note that `uw/launcher.py`'s native path has had no ROS exercise at all.

### 5.2 The subscribers — thor (primary), orin, linus

The stack container is new work; the shape it needs:

```
ros:jazzy-ros-base (arm64 on the Jetsons, amd64 on linus)
+ ros-jazzy-marine-acoustic-msgs      # ros/sonar_relay's dependency (V, package.xml)
+ ros-jazzy-rosbag2-* , ros-jazzy-mavros (later, M8b)
run with: --network host --ipc host, ROS_DOMAIN_ID=73, RMW_IMPLEMENTATION=rmw_fastrtps_cpp,
          FASTRTPS_DEFAULT_PROFILES_FILE=<that host's profile>
```

`--network host` is literal on all three (Linux hosts), so no port mapping is needed anywhere. The container
decouples the ROS distro from the host L4T, which matters for orin: JetPack 6 is Ubuntu 22.04, whose native ROS 2
is **Humble**, while the container can still be Jazzy (**U**, pending the JetPack versions from check 3).
`marine_acoustic_msgs` must exist for arm64 Jazzy or be built in the image — `docs/ros_hil.md` §7 item 1 flags
exactly this.

Where the **relay** runs is a decision, not a detail: `docs/ros_hil.md` §2.1 puts it **on the sim host**, so the
bag and the Jetson see the same `ProjectedSonarImage` bytes. Keep it there; the Jetson subscribes, it does not
assemble.

### 5.3 Windows firewall

Windows blocks inbound UDP by default and NVIDIA's page says nothing about it (**V** — it only documents WSL2
port proxies). From an **administrator** PowerShell:

```powershell
New-NetFirewallRule -DisplayName "ROS2 DDS domain 73 in"  -Direction Inbound  -Protocol UDP -LocalPort 25650-25760 -Action Allow -Profile Private
New-NetFirewallRule -DisplayName "ROS2 DDS domain 73 out" -Direction Outbound -Protocol UDP -LocalPort 25650-25760 -Action Allow -Profile Private
# multicast discovery, if it is used at all
New-NetFirewallRule -DisplayName "ROS2 DDS mcast in" -Direction Inbound -Protocol UDP -RemoteAddress 239.255.0.1 -Action Allow -Profile Private
```

Check the network is classified **Private**, not Public (`Get-NetConnectionProfile`) — on Public, Windows blocks
the rules' profile and the symptom is one-way traffic. If the stack ever runs inside WSL2, add NVIDIA's
`netsh interface portproxy` entries for the §3 ports instead (**V**).

## 6. Verification — the success criterion, with the numbers to compare against

Run every command with the domain in front of it, and **one monitor at a time**: twelve concurrent
`ros2 topic hz` processes dropped the sim to **RTF 0.5** (**V**, `docs/ros_hil.md` §5), so a monitor on the
publisher host corrupts the thing it measures. Put the monitors on thor.

```bash
export ROS_DOMAIN_ID=73 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_hil.xml
ros2 doctor --report | grep -A3 -E "MIDDLEWARE|NETWORK"     # reads back what §3 claims
ros2 topic list
ros2 node list
S=/rov1/sensors/sonar_fls; C=/rov1/sensors/camera_front
ros2 topic info -v $S/image             # QoS of every endpoint, and that there IS a remote endpoint
# rate, payload, age and drops, with the reader's QoS taken from each publisher's. NOT ros2 topic hz/bw:
# they have no QoS option in Jazzy and their RELIABLE reader cannot match a BEST_EFFORT writer (§6.1)
ros/link_monitor/run.sh --duration 40 --age --json link.json
ros2 topic echo --once --qos-durability transient_local $S/meta   # latched: a late joiner must receive it
ros2 run tf2_ros tf2_echo rov1/sonar_fls rov1/sonar_fls_projected
ros2 run tf2_ros tf2_echo map rov1/sonar_fls    # MUST fail: that edge is ground truth (§2.5 rule 4)
```

Expected, from what the sim publishes today (**V**, `docs/ros_hil.md` §1) and the bandwidth arithmetic of its
§2.2 (tank/lf preset, 512 beams, 876 rows):

| topic | rate | `ros2 topic bw` ≈ | note |
|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` mono8 | 10 Hz | **4.5 MB/s** (35.9 Mbit/s) | grows with range: 2001 rows in the basin |
| `/rov1/sensors/sonar_fls/sonar_image` | 10 Hz | **4.7 MB/s** (37.5 Mbit/s) | the relay's output; the only sonar topic that crosses |
| `/rov1/sensors/sonar_fls/meta` | on change | — | must arrive on a late joiner (TRANSIENT_LOCAL) |
| `/rov1/sensors/camera_front/image_raw` rgb8 1280×720 | 10 Hz | **27.6 MB/s** (221.2 Mbit/s) | the camera profile is not the latency profile |
| `/gt/polar_db` 32FC1 | 10 Hz | **18 MB/s** (143.5 Mbit/s) | ground truth, 4× the ping: must **never** cross |
| `/gt/plume/markers` | 2 Hz | ~0.24 MB/s | 3000 markers (**V**) |
| `/gt/leak/pose`, `/gt/leak/flow_lpm` | 1 Hz | — | not latched any more |
| `/gt/sonar_pose`, `/gt/tf` | per ping | — | the true pose, out of `meta` and out of `/tf` |
| `/clock` | 60 Hz | — | `ros.clock_hz: 60.0` (**V**) |
| `/tf` | — | — | **no publisher on the sim host at all**, by design |
| `/sim/debug/sonar_fan` | off | — | `ros.publish_fan: false` |

`ros2 topic bw` reports deserialized payload per second, so compare it with the payload column above, **not**
with §2.2's "+8 % wire" column; the 8 % is framing overhead measured at the NIC, e.g. with
`ip -s link show enp5s0` deltas.

**Acceptance** (the criterion this task is judged on):

| # | pass condition |
|---|---|
| 1 | on thor, `ros2 topic list` shows the sonar, camera, `/clock`, `/tf_static` topics published by the sim host |
| 2 | `ros2 topic info -v` shows a **remote** publisher endpoint for each, with the QoS the map specifies |
| 3 | the rate is **nominal × RTF** over ≥ 30 s, and the mean `header.stamp` interval is the nominal period — see §6.1, this is not the same as "within ±5 % of 10 Hz" |
| 4 | `mbit_s` matches `docs/ros_hil.md` §2.2 within ~10 %, and the NIC counters agree within the framing overhead |
| 5 | `…/sonar_fls/meta` reaches a subscriber started **after** the sim (late-joiner latching) |
| 6 | `tf2_echo` resolves the payload chain (`rov1/sonar_fls` → `rov1/sonar_fls_projected`) on the subscriber host, and `map` → `rov1/sonar_fls` does **not** resolve — that edge is ground truth |
| 7 | the sim's **RTF is recorded next to every number**, with and without the monitors attached (§3.3 of ros_hil: a wall-clock delay appears inside the sim scaled by RTF) |
| 8 | `lost` (from `QoSMessageLostInfo`) is 0 on every topic that crosses, and the `/proc/net/snmp` UDP/IP deltas are captured for the run |

### 6.1 Measured: loopback on linus, 2026-10-29

**What ran.** `python3 -m uw sim --sink ros` in the Isaac container (`network_mode: host`, `ipc: host`, domain 73,
`rmw_fastrtps_cpp`, no DDS profile unless stated), `ros/sonar_relay/run.sh` and `ros/link_monitor/run.sh` on the
host with `/opt/ros/jazzy`. Container ↔ host on one machine, which is M7 step 5's "loopback first". Raw numbers in
`output/probe/m7/` (untracked).

**Three findings before the tables, because two of them invalidate the obvious way of taking these measurements.**

1. **`ros2 topic hz` and `ros2 topic bw` cannot measure this graph at all.** They have no QoS option in Jazzy —
   `--qos-reliability` is rejected with a usage error — and their default RELIABLE reader cannot match the
   BEST_EFFORT image writers the map specifies (§3 of `topic_map.md`). They report silence, and if the usage
   error is swallowed by a pipe it reads exactly like a dead link. `ros2 topic echo` *does* take the flag.
   `ros/link_monitor` exists for this: it reads each publisher's offered QoS with
   `get_publishers_info_by_topic` and subscribes with a matching profile.
2. **Every rate on the wire is nominal × RTF**, because the sim paces itself in sim time. With the camera at
   RTF 0.787 the 10 Hz sonar arrived at 7.88 Hz, `/clock` at 47.3 Hz instead of 60, the 2 Hz plume at 1.573 and
   the 1 Hz leak at 0.786 — all 0.787 of nominal, which is the §3.3 scaling made visible. A rate criterion
   written as "±5 % of 10 Hz" fails a perfectly healthy link.
3. **BEST_EFFORT samples above ~1 MB are lost without a DDS profile.** The 448 kB sonar ping lost nothing; the
   1.79 MB `/gt/polar_db` and the 2.76 MB camera frame did. A/B on the same run, reader side only:

   | reader | `/gt/polar_db` | rate | payload | lost |
   |---|---|---|---|---|
   | default socket buffers | 243 msgs | 9.758 Hz | 139.14 Mbit/s | **6** |
   | `FASTRTPS_DEFAULT_PROFILES_FILE` with `receiveBufferSize` 4 MB (§4) | 250 msgs | **10.000 Hz** | **143.46 Mbit/s** | **0** |

   So the §4 profile is not multi-host hygiene, it is what makes the map's QoS work, and the sysctls behind it
   are load-bearing. The same run's 448 kB sonar topic was unaffected either way (36.0 Mbit/s, 0 lost).

**No-camera profile** — `--no-camera --duration 170`, whole run **170.0 s sim / 170.1 s wall, RTF 0.9995**,
1700 pings, 1 over budget. 40 s window, relay and monitor attached:

| topic | QoS seen | n | rate | payload | mean bytes | lost |
|---|---|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` | best_effort/volatile | 402 | **10.020 Hz** | 36.04 Mbit/s | 448 572 | 0 |
| `/rov1/sensors/sonar_fls/sonar_image` | best_effort/volatile | 400 | **10.000 Hz** | 37.46 Mbit/s | 468 488 | 0 |
| `/rov1/sensors/sonar_fls/meta` | reliable/transient_local | 1 | latched | — | 11 664 | 0 |
| `/clock` | best_effort/volatile | 2402 | **60.059 Hz** | 0.01 | 12 | 0 |
| `/tf_static` | reliable/transient_local | 1 | latched | — | 356 | 0 |
| `/sim/diagnostics` | reliable/volatile | 40 | **1.000 Hz** | — | 439 | 0 |
| `/gt/sonar_pose` | reliable/volatile | 400 | 10.000 Hz | — | 76 | 0 |
| `/gt/tf` | reliable/volatile | 400 | 10.000 Hz | — | 108 | 0 |
| `/gt/plume/markers` | best_effort/volatile | 80 | 2.000 Hz | 1.88 | 117 363 | 0 |
| `/gt/leak/pose` | reliable/volatile | 40 | 1.000 Hz | — | 76 | 0 |
| `/gt/polar_db` | best_effort/volatile | 338 | 8.467 Hz | 121.21 | 1 794 108 | **61** |

**Camera profile** — same scene with the camera, RTF **0.787** in the window (0.683 over the whole run, which
includes the startup transient). 40 s window:

| topic | n | rate | payload | mean bytes | lost |
|---|---|---|---|---|---|
| `/rov1/sensors/sonar_fls/image` | 316 | 7.880 Hz | 28.34 Mbit/s | 448 572 | 0 |
| `/rov1/sensors/sonar_fls/sonar_image` | 316 | 7.879 Hz | 29.60 Mbit/s | 468 488 | 0 |
| `/rov1/sensors/camera_front/image_raw` | 215 | 5.361 Hz | 118.85 Mbit/s | 2 764 872 | **100** |
| `/rov1/sensors/camera_front/camera_info` | 315 | 7.873 Hz | 0.02 | 384 | 0 |
| `/gt/polar_db` | 269 | 6.755 Hz | 96.49 Mbit/s | 1 794 108 | **45** |
| `/clock` | 1890 | 47.311 Hz | — | 12 | 0 |

**The bandwidth arithmetic of `docs/ros_hil.md` §2.2 is confirmed**, which is the point of taking these numbers.
Per-message size × nominal 10 Hz, measured against predicted:

| topic | measured bytes/msg | → at 10 Hz | §2.2 predicted | error |
|---|---|---|---|---|
| polar `image` | 448 572 | 35.89 Mbit/s | 35.9 | 0.0 % |
| `sonar_image` (`ProjectedSonarImage`) | 468 488 | 37.48 | 37.5 | 0.1 % |
| `polar_db` | 1 794 108 | 143.53 | 143.5 | 0.0 % |
| `camera_front/image_raw` | 2 764 872 | 221.19 | 221.2 | 0.0 % |

The relay's constant overhead measures **19 916 B** per ping (468 488 − 448 572) against the 19 888 B §2.2
computes from `relay.py` — 28 bytes of message header.

**Acceptance verdict.**

| # | verdict |
|---|---|
| 1 | **pass** (loopback): `/rov1/sensors/**`, `/gt/**`, `/sim/**`, `/clock`, `/tf_static`, node `sim_bridge`, and **no `/tf` publisher at all** |
| 2 | **pass**: QoS read back per endpoint is the map's — best_effort/volatile on images, reliable/transient_local on `meta` and `/tf_static` |
| 3 | **pass**: 10.020 / 10.000 / 60.059 / 2.000 / 1.000 Hz at RTF 0.9995 |
| 4 | **pass**: 0.0–0.1 % against §2.2 (table above) |
| 5 | **pass**: `meta` and `/tf_static` both reached a monitor started minutes after the sim |
| 6 | **pass**: the mount chain resolves (RPY 180°, −90°, 0° = `R_SONAR_PROJECTED`), and `map → rov1/sonar_fls` fails with *Invalid frame ID "map" … frame does not exist* — the world-anchored edge is simply not in the live tree |
| 7 | **pass**: RTF **1.001** with nothing attached, **0.9963** with the relay and the monitor subscribing — 0.4 %, against the 50 % that twelve `ros2 topic hz` processes once cost (ros_hil §5). `ros/link_monitor` subscribes raw, which is most of the difference |
| 8 | **pass for everything that crosses the link**; `polar_db` and the camera frame drop without the §4 profile, and `polar_db` is 0 with it (finding 3). The camera case is untested with the profile |

**Not done:** the sysctls are written to `config/sysctl/60-ros2-hil.conf` but **not installed** — that needs root,
and this session has no sudo. The live values on linus are still the unpinned `rmem_max = wmem_max = 4194304`,
which is what the measurements above ran against, so installing the file raises the ceiling rather than changing
any number reported here.

### 6.2 The ground-truth domain (approved 2026-10-29)

`/gt` has its own ROS domain and its own participant. `ros.gt_domain_id: 74` makes `RosSink` create a second
rclpy `Context`, and `ros.gt_localhost_only: true` sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` around **that
init only** — rcl reads the variable when a context is initialised, so the ground-truth participant gets
LOCALHOST while the HIL participant keeps SUBNET.

Two mechanisms, and only one of them is a guarantee. **The domain** is the guarantee: a node on the HIL domain
cannot subscribe to a topic published on another one, whatever it asks for, so the five rules of
`docs/ros_hil.md` §2.5 stop depending on anyone's discipline. **The discovery range** keeps that participant from
announcing itself on the HIL NIC at all, and is the part that still has to be verified from a second host.
Neither was ever what kept `/gt/polar_db`'s 143.5 Mbit/s off the wire — DDS sends nothing for a topic with no
matched remote reader — but *"nobody subscribed"* is not a property you can design a HIL link around.

Verified on linus, one sim, both domains listed at once:

| domain 73 (HIL) | domain 74 (ground truth) |
|---|---|
| `/clock`, `/tf_static` | `/clock` (the same stamps: a bag without sim time is useless) |
| `/rov1/sensors/sonar_fls/{image,meta}` | `/gt/{polar_db,sonar_pose,tf}` |
| `/rov1/sensors/camera_front/{image_raw,image_raw/compressed,camera_info}` | `/gt/leak/{pose,flow_lpm}`, `/gt/plume/markers` |
| `/sim/diagnostics`, `/sim/debug/sonar_fan` | — |
| **no `/gt` topic of any kind** | measured flowing: `polar_db` 56.34 Mbit/s, `sonar_pose` and `tf` 8.25 Hz, `plume/markers` 1.65 Hz |

Record the ground truth on the workstation from that domain (the `--domain` flag of `link_monitor` exists for the
same reason — `ROS_DOMAIN_ID` deliberately does not select it, as in `ros/sonar_relay`):

```bash
ROS_DOMAIN_ID=74 ros2 bag record -o gt_$(date +%F_%H%M) \
  /clock /gt/sonar_pose /gt/tf /gt/leak/pose /gt/leak/flow_lpm /gt/plume/markers /gt/polar_db
ros/link_monitor/run.sh --domain 74 --duration 30        # what that bag is about to receive
```

Use the §4 profile for the recorder too: `/gt/polar_db` is a 1.79 MB sample and dropped 64 of ~250 on the
ground-truth domain with default socket buffers, exactly as it did on the HIL domain (§6.1 finding 3).

### 6.3 Camera: raw, compressed, and which knob is which

The container's Python has **no JPEG encoder at all** — no PIL, cv2, simplejpeg, turbojpeg, imageio or
torchvision (**V**, checked in the image on 2026-10-29) — so `CompressedImage` is PNG, through the stdlib zlib
writer the datasets already use (`sinks.png_bytes`). Four runs, same scene, 25 s windows, `--duration 110`:

| run | render | publish | camera topic | Hz | payload | bytes/msg | lost | RTF | `camera_read_publish` |
|---|---|---|---|---|---|---|---|---|---|
| `raw` | 10 Hz | 10 Hz | `image_raw` | 5.036 | 109.46 Mbit/s | 2 764 872 | **86** | 0.8340 | 7.5 ms |
| `compressed` | 10 Hz | 10 Hz | `image_raw/compressed` | 8.338 | **18.13** | **272 675** | **0** | 0.8378 | 15.7 ms |
| `compressed`, `ros.camera_hz: 2` | 10 Hz | 2 Hz | `image_raw/compressed` | 1.664 | 3.64 | 272 025 | 0 | 0.8321 | 7.8 ms |
| `compressed`, `camera.rate_hz: 2` | 2 Hz | 2 Hz | `image_raw/compressed` | 2.001 | 4.18 | 261 663 | 0 | **0.9989** | 15.8 ms |

Four things worth stating plainly.

1. **PNG is 10.1× on these frames** (272 675 B against 2 764 872 B), which takes the camera from 221.2 Mbit/s at
   nominal 10 Hz to **21.7**. The scene is dark and smooth, which is why; on a brighter saved frame from an
   earlier run the same encoder gave only 3.76× at level 1 and 4.33× at level 3, so **the ratio is a property of
   the scene, not of the setting** — re-measure it per scenario rather than quoting 10× as a constant.
2. **It costs 8.2 ms per published frame and nothing in RTF** (7.5 → 15.7 ms of `camera_read_publish`, RTF 0.834
   → 0.838, which is inside the run-to-run spread). The encode replaces the serialization and transport of
   2.76 MB, so the loop comes out even.
3. **It removes the drops, by making the sample small.** Raw lost 86 of ~210 frames; at 273 kB the sample is
   below the ~1 MB threshold where BEST_EFFORT loss starts (§6.1 finding 3), and three runs lost nothing.
4. **The two rate knobs are not interchangeable.** `ros.camera_hz` caps the *publish* rate: it takes bytes off
   the link and leaves RTF alone (0.832). `camera.rate_hz` caps the *render*: it is where the whole cost is, and
   at 2 Hz it gives **RTF 0.9989 with the sonar at 10.028 Hz and `/clock` at 60.081 Hz** — a camera in the
   latency profile, which `docs/ros_hil.md` §0 assumed impossible on this machine.

> **The HIL camera configuration this measurement recommends:** `ros.camera_transport: compressed` with
> `camera.rate_hz: 2`. 4.18 Mbit/s, no drops, RTF 1.00, full sonar rate. `ros.camera_hz` stays for the case where
> the render rate must stay high for the dataset while the link stays thin.

One cosmetic consequence of publishing a topic conditionally: the publisher for the *other* transport is still
created, so `ros2 topic list` shows `image_raw` (or `image_raw/compressed`) with no data behind it, as it already
does for `/sim/debug/sonar_fan`. A monitor reports `n = 0` for it rather than an error.

## 7. Troubleshooting: symptom → cause → check

| symptom | most likely cause | check |
|---|---|---|
| `ros2 topic list` empty, no error | wrong domain — a login shell on linus has **no `ROS_DOMAIN_ID`**, so it sits on domain 0 (**V**, measured) | `echo $ROS_DOMAIN_ID`; prefix every command |
| …still empty, domain correct | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, or different `RMW_IMPLEMENTATION` on the two hosts | `ros2 doctor --report` on both |
| nodes **discover** each other, **no data** | shared-memory transport between two hosts: the §4 profile is missing or not exported in that terminal | `echo $FASTRTPS_DEFAULT_PROFILES_FILE` in the *publishing* process's environment |
| only some nodes discovered | unicast `initialPeersList` without ports probes only the first few participant ids (**U**) | add `<port>`, or raise `maxInitialPeersRange` |
| one topic silent, the rest fine | incompatible QoS — a RELIABLE reader against a BEST_EFFORT writer presents as total silence | `ros2 topic info -v`; subscribe with the `incompatible_qos` event callback (**V**) |
| images arrive truncated, or stop under load | UDP fragmentation: 448 kB payloads at MTU 1500 are ~300 fragments per message; one lost fragment drops the whole sample | `/proc/net/snmp` Ip `ReasmFails`/`ReasmTimeout`, Udp `RcvbufErrors`; raise the §4 buffers, then consider `LARGE_DATA` |
| discovery works, then stops after minutes | multicast on a shared LAN with IGMP snooping/querier issues | switch to unicast peers (§4) |
| Windows: only its own nodes visible | Fast DDS bound to a Hyper-V/WSL virtual adapter | `interfaceWhiteList` with the physical NIC's address; `ipconfig /all` |
| Windows: traffic one way only | firewall profile is **Public**, or no outbound rule | `Get-NetConnectionProfile`; §5.3 |
| `/clock` present, Jetson nodes frozen or stamping wall time | `use_sim_time` not set — **nothing in the repo sets it** (**V**) | `ros2 param get <node> use_sim_time` |
| the sim slows down when you start monitoring | subscriber-side CPU on the publisher host: 12 × `ros2 topic hz` → RTF 0.5 (**V**); `ros/link_monitor` costs 0.4 % (**V**, §6.1) | move monitors to thor; log RTF per run |
| `ros2 topic hz`/`bw` print nothing on a sensor topic | they have **no `--qos-reliability` in Jazzy** and their default RELIABLE reader cannot match a BEST_EFFORT writer (**V**, §6.1). Piped into `grep`, the usage error looks like a dead link | use `ros/link_monitor/run.sh`, or `ros2 topic echo --qos-reliability best_effort` |
| every rate is low by the same factor | not a fault: rates are **nominal × RTF** (**V**, §6.1) — at RTF 0.787 a 10 Hz sonar is 7.88 Hz on the wire | divide by the RTF on `/sim/diagnostics` |
| traffic appears on domain 73 that nobody here published | another host on the shared `192.168.177.0/24` LAN | `ros2 node list`; see ros_hil §7 item 14 (f1tenth coexistence) |
| `rosidl`/`rclpy` import errors on the sim host | `ROS_DISTRO`, the bridge `lib` path or the RMW not exported in the launching shell | re-read §5.1; the bundled `rosidl_generator_py` also logs a harmless DEBUG traceback (**V**, `ros_bridge.py`) |

## 8. What this does not establish yet

- **The loopback half is done** (§6.1): container ↔ host on linus, all eight acceptance criteria, with the
  bandwidth arithmetic of ros_hil §2.2 confirmed to 0.1 %. **The Ethernet half is not**: the Windows side does
  not exist, and the thor and orin containers do not exist.
- **The §4 profile has been validated on one host only**, as a reader-side fix for large-sample loss. Its
  interface whitelist and unicast peers — the parts that matter for a real link — are still untested, and the
  camera frame has not been retested with it.
- **The sysctls are not installed**: `config/sysctl/60-ros2-hil.conf` needs root.
- **RTT and jitter linus ↔ thor ↔ orin are unmeasured** — the first missing term of `docs/ros_hil.md` §5, and
  what decides whether MAVROS's TIMESYNC converges at all (its §7 item 2).
- **Thor's and Orin's JetPack version, NIC and PTP capability are unknown** (§7 item 1 of that document); they
  decide arm64 package availability and the wall-clock sync floor.
- **The topic names used in §6 are `docs/topic_map.md`'s**, which the code now publishes; the rename landed
  before the first transport measurement, so a failure here is one unknown, not two.
