#!/usr/bin/env python3
"""Stub physics for ArduPilot's JSON SITL backend (stdlib only): a vehicle that hovers motionless at a fixed depth.

It exists to smoke-test SITL and the MAVLink path to the Thor before the ROV model is there. It is NOT a vehicle
model: the thruster outputs are read and counted, never integrated.

Protocol (ArduPilot libraries/SITL/SIM_JSON.cpp, tag ArduSub-4.5.7). SITL is the client and this program the server:
  SITL -> sim   servo packet to UDP <sim>:9002 from an ephemeral port, little endian
                16 channels: uint16 magic=18458, uint16 frame_rate, uint32 frame_count, uint16 pwm[16]
                32 channels: uint16 magic=29569, uint16 frame_rate, uint32 frame_count, uint16 pwm[32]
  sim -> SITL   one reply per servo packet, to the packet's source address: "\\n{json}\\n". The parser is not a JSON
                parser (strstr plus sscanf), so the vectors keep the "[a, b, c]" form with ", " and no space after ':'.
  Lockstep      SITL blocks on the reply and takes its clock from "timestamp", so SITL time = the timestamps sent here.
                --rtf paces the timestamps against the wall clock (1.0 = real time). RTF < 1 = a slow physics sim.

    python tools/stub_physics.py                       # listens on 127.0.0.1:9002, Ctrl+C to stop
    python tools/stub_physics.py --duration 60 --rtf 0.8
"""
import argparse
import socket
import struct
import sys
import time

MAGIC_16 = 18458
MAGIC_32 = 29569
HDR = struct.Struct("<HHI")


def reply(t, depth_m):
    # NED, relative to the home point: position z positive down, so depth_m below the surface. At rest the
    # accelerometer reads the reaction to gravity: -g on body z.
    return (
        '\n{"timestamp":%.6f,"imu":{"gyro":[0.0, 0.0, 0.0],"accel_body":[0.0, 0.0, -9.80665]},'
        '"position":[0.0, 0.0, %.3f],"attitude":[0.0, 0.0, 0.0],"velocity":[0.0, 0.0, 0.0]}\n' % (t, depth_m)
    ).encode()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bind", default="127.0.0.1", help="address to listen on (default loopback: no firewall prompt)")
    ap.add_argument("--port", type=int, default=9002)
    ap.add_argument("--depth", type=float, default=1.0, help="fixed depth in m (default 1.0)")
    ap.add_argument("--rtf", type=float, default=1.0, help="pace sim time against the wall clock (0 = unpaced)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (0 = until Ctrl+C)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # On Windows a reply to a port that has just closed raises ConnectionResetError on the next recvfrom: caught below.
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    print("[stub] listening on %s:%d, depth %.2f m, rtf %.2f" % (args.bind, args.port, args.depth, args.rtf), flush=True)

    sim_t = 0.0                 # the timestamp sent to SITL
    wall0 = None                # wall time of the first packet
    n = n_last = 0
    last_pwm = ()
    peer = None
    t_report = t_start = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if args.duration and now - t_start > args.duration:
                break
            if now - t_report >= 5.0:
                rate = (n - n_last) / (now - t_report)
                print("[stub] t_sim=%8.2f s  %6.1f packets/s  peer=%s  pwm[0:8]=%s"
                      % (sim_t, rate, peer, list(last_pwm[:8])), flush=True)
                t_report, n_last = now, n
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue
            if len(data) >= HDR.size:
                magic, rate_hz, frame_count = HDR.unpack_from(data)
                nch = 16 if magic == MAGIC_16 else 32 if magic == MAGIC_32 else 0
                if nch and len(data) >= HDR.size + 2 * nch:
                    last_pwm = struct.unpack_from("<%dH" % nch, data, HDR.size)
                    if peer != addr:
                        peer = addr
                        print("[stub] servo packets from %s:%d, %d channels, frame_rate %d Hz"
                              % (addr[0], addr[1], nch, rate_hz), flush=True)
                    sim_t += 1.0 / (rate_hz or 400)
                    if args.rtf > 0:
                        if wall0 is None:
                            wall0 = time.monotonic() - sim_t / args.rtf
                        delay = wall0 + sim_t / args.rtf - time.monotonic()
                        if delay > 0:
                            time.sleep(delay)
                    sock.sendto(reply(sim_t, args.depth), addr)
                    n += 1
    except KeyboardInterrupt:
        pass
    print("[stub] done: %d packets answered, t_sim %.2f s" % (n, sim_t), flush=True)
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
