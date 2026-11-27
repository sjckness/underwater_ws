"""vehicles.common.nucleus (M12 P5): the Nortek Nucleus1000 at the protocol level (host, numpy, sockets on loopback).

The wire format is pinned against the manual's own sample packet (N3015-033 §11.2) and NMEA examples (§7.38), and,
when Nortek's driver is installed (``nucleus-driver`` 1.7.x in uw_pydeps), against its real parser.
"""
from __future__ import annotations

import math
import socket
import struct
import time

import numpy as np
import pytest

from uw import config
from vehicles.common import nucleus as N

# The AHRS packet of the manual's §11.2, header and data (108 bytes), as printed there
def _manual_packet() -> bytes:
    hexs = """A5 0A D2 20 6C 00 8A E5 F9 C6 02 24 00 00 02 00 00 00 00 35 0C 00 00 00 00 00 04 00 00 00 00 00 00 00 02 00
              00 00 CB 82 77 3E 00 00 A0 40 AC A0 25 BF BC 74 4A BF 6B B6 8D 43 63 EC 48 BF 11 AA 0E 3C 4E 7C FB 3A F1 9E
              1E 3F 6D B9 6D 3E 7A FF 78 3F 7D E5 FE 3B 19 FB 78 BF 9A 93 6D 3E F5 72 83 3C 69 23 62 3C 7C FC 38 BC 94 F5
              7F 3F 00 00 00 00 FF FE 2D 3F"""
    return bytes(int(x, 16) for x in hexs.split())


def test_the_checksums_are_the_manuals():
    pkt = _manual_packet()
    head, data = pkt[:10], pkt[10:]
    size, data_cs, head_cs = struct.unpack_from("<HHH", head, 4)
    assert size == len(data) == 108
    assert N.checksum(data) == data_cs and N.checksum(head[:8]) == head_cs
    assert N.frame(0xD2, data) == pkt


@pytest.mark.parametrize("body, cs", [('ID,STR="Nucleus1000",SN=58', "31"), ('GETAHRS,FREQ=10,DS="ON",DF=210', "46"),
                                      ('GETTRIG,SRC="INTERNAL",FREQ=2.00,ALTI=2', "1B"), ("GETALL", "38"),
                                      ("OK", "2B"), ("ERROR", "77")])
def test_nmea_checksums_match_the_manual(body, cs):
    assert N.nmea(body) == f"$PNOR,{body}*{cs}\r\n".encode()


def _packets(data: bytes) -> list:
    out, i = [], 0
    while i + 10 <= len(data):
        if data[i] != N.SYNC:
            j = data.find(bytes([N.SYNC]), i + 1)
            if j < 0:
                break
            i = j
            continue
        size = struct.unpack_from("<H", data, i + 4)[0]
        if i + 10 + size > len(data):
            break
        head, body = data[i:i + 10], data[i + 10:i + 10 + size]
        assert N.checksum(head[:8]) == struct.unpack_from("<H", head, 8)[0]
        assert N.checksum(body) == struct.unpack_from("<H", head, 6)[0]
        out.append((head[2], body))
        i += 10 + size
    return out


@pytest.fixture
def device():
    cfg = config.load(config.POOL_CONFIG, presets=["nucleus"])
    cfg["nucleus"]["device"].update(bind="127.0.0.1", cmd_port=0, data_port=0)
    dev = N.pool_device(cfg)
    assert dev is not None
    from vehicles.common.physics_proc import make_pool_server
    srv = make_pool_server(cfg)
    dev.start()
    try:
        yield dev, srv
    finally:
        dev.close()


def _line(sock, buf=b""):
    while b"\n" not in buf:
        buf += sock.recv(4096)
    line, rest = buf.split(b"\n", 1)
    return line + b"\n", rest


def _fly(dev, srv, seconds, v_frd=(0.3, 0.0, 0.0)):
    """The physics side, without SITL: the device stepped at 400 Hz on a moving mid-water state."""
    from dataclasses import replace
    from vehicles.common import frames as F
    state = srv.state
    p, q = F.pose_tank_to_ned([25.0, 12.0, 2.5], F.quat_from_rpy_deg((0, 0, 30)), 5.0)
    state = replace(state, position_ned=p, quat_frd_ned=q, velocity_frd=np.array(v_frd, float))
    for k in range(int(seconds * 400)):
        state = replace(state, t=state.t + 0.0025)
        pose = F.pose_ned_to_tank(state.position_ned, state.quat_frd_ned, 5.0)
        dev.step(state.t, (pose[0], F.quat_to_rot(pose[1])), state)
        time.sleep(0.0)


def test_login_getall_start_and_the_data(device):
    dev, srv = device
    s = socket.create_connection(("127.0.0.1", dev.tcp_port), timeout=3)
    try:
        line, rest = _line(s)
        assert line == b"Please enter password:\r\n"
        s.sendall(b"nortek\r\n")
        line, rest = _line(s, rest)
        assert b"Welcome to Nortek" in line
        s.sendall(b'SETCLOCKSTR,TIME="2026-11-27 12:00:00"\r\n')
        line, rest = _line(s, rest)
        assert line == b"OK\r\n" and dev.posix
        s.sendall(b"$PNOR,GETALL*38\r\n")
        lines = []
        while not lines or lines[-1] != b"$PNOR,OK*2B\r\n":
            line, rest = _line(s, rest)
            lines.append(line)
        assert len(lines) >= 15 and all(b"$PNOR," in x for x in lines)
        assert [x for x in lines if b"ID" in x.split(b"*")[0]] == [lines[0]]     # only the ID line says "ID"
        for x in lines:                                                         # every NMEA checksum is right
            body, cs = x[1:].rstrip(b"\r\n").split(b"*")
            assert int(cs, 16) == np.bitwise_xor.reduce(np.frombuffer(body, np.uint8))
        s.sendall(b"GETBT\r\n")
        line, rest = _line(s, rest)
        assert line.startswith(b'MODE="AUTO"')
        line, rest = _line(s, rest)
        assert line == b"OK\r\n"
        s.sendall(b"START\r\n")
        line, rest = _line(s, rest)
        assert line == b"OK\r\n" and dev.measuring
        _fly(dev, srv, 1.0)
        time.sleep(0.3)
        s.settimeout(0.5)
        data = rest
        try:
            while True:
                chunk = s.recv(65536)
                if not chunk:
                    break
                data += chunk
        except socket.timeout:
            pass
        pk = _packets(data)
        kinds = {pid: [b for p, b in pk if p == pid] for pid in (N.ID_AHRS, N.ID_BT, N.ID_ALTI, N.ID_INS)}
        assert len(kinds[N.ID_AHRS]) == pytest.approx(20, abs=2)
        assert len(kinds[N.ID_BT]) + len(kinds[N.ID_ALTI]) == pytest.approx(8, abs=1)
        assert len(kinds[N.ID_INS]) == pytest.approx(10, abs=1)
        assert {len(b) for b in kinds[N.ID_BT]} == {128} and {len(b) for b in kinds[N.ID_ALTI]} == {44}
        assert {len(b) for b in kinds[N.ID_AHRS]} == {108} and {len(b) for b in kinds[N.ID_INS]} == {208}
        bt = kinds[N.ID_BT][-1]
        status = struct.unpack_from("<I", bt, 12)[0]
        assert status & (0b111 << 9) == 0b111 << 9                              # X, Y, Z velocity valid
        vx, vy, vz = struct.unpack_from("<3f", bt, 96)
        assert vx == pytest.approx(0.3, abs=0.02) and abs(vy) < 0.02
        assert bt[2] & 1, "POSIX time after SETCLOCKSTR"
        ahrs = kinds[N.ID_AHRS][-1]
        assert ahrs[0] == 2 and ahrs[1] == 36
        roll, pitch, heading = struct.unpack_from("<3f", ahrs, 36)
        assert heading == pytest.approx(60.0, abs=5.0)                          # ENU 30 deg is NED heading 60
        # in measurement mode only STOP, TRIG and UPDATEPOS
        s.settimeout(3)
        s.sendall(b"GETBT\r\nSTOP\r\n")
        tail = b""
        while b"OK\r\n" not in tail:
            tail += s.recv(65536)
        assert b"ERROR\r\n" in tail and not dev.measuring
    finally:
        s.close()


def test_one_client_on_the_command_port_two_on_the_data_port(device):
    dev, srv = device
    a = socket.create_connection(("127.0.0.1", dev.tcp_port), timeout=3)
    a.recv(100)
    a.sendall(b"nortek\r\n")
    a.recv(100)
    b = socket.create_connection(("127.0.0.1", dev.tcp_port), timeout=3)
    assert b.recv(100) == b"", "the second command client is closed"
    d1 = socket.create_connection(("127.0.0.1", dev.data_port), timeout=3)
    d2 = socket.create_connection(("127.0.0.1", dev.data_port), timeout=3)
    time.sleep(0.2)
    assert dev.clients == 3
    for x in (a, b, d1, d2):
        x.close()


def test_the_control_channel_can_switch_the_acoustics_off(device):
    dev, srv = device
    assert dev.command({"command": "set_config", "parameters": {"acoustic_enabled": False}})["success"]
    dev.measuring = True
    _fly(dev, srv, 1.0)
    assert dev.count["bottom_track"] == 0 and dev.count["ahrs"] > 0
    assert not dev.command({"command": "set_config", "parameters": {"speed_of_sound": 1500}})["success"]


def test_nortek_s_own_parser_reads_every_packet():
    """The contract: nucleus_driver's Parser (1.7.0, the one the BlueOS extension pins) parses what we send."""
    from uw import deps
    try:
        parser_mod = deps.require("nucleus_driver._parser", "nucleus-driver", "1.7.0", install=False, no_deps=True)
    except ImportError as exc:
        pytest.skip(f"Nortek's driver is not installed: {exc}")

    class M:
        def __getattr__(self, n):
            return lambda *a, **k: None

    class L:
        _logging = False

    class C:
        timeout = 1
    rot = np.eye(3)
    from types import SimpleNamespace
    rep = SimpleNamespace(beam_valid=[True, True, True], velocity_valid=True, beam_velocity=[0.1, 0.2, 0.3],
                          beam_range=[2.0, 2.1, 2.2], covariance=np.eye(3) * 1e-4, velocity=np.array([0.3, -0.1, 0.02]))
    stream = (N.ahrs_packet(1764245000.25, (1.0, -2.0, 350.0), rot, 3.5, 900002, True)
              + N.bt_packet(1764245000.5, rep, 1500.0, 27.0, 0.35, 900002, True, 0.125)
              + N.alti_packet(1764245000.6, 2.0, 1500.0, 27.0, 0.35, 900002, True)
              + N.ins_packet(1764245000.7, (1.0, -2.0, 10.0), rot, 3.5, (1.5, -0.5, 3.5), (0.2, 0.1, 0.0),
                             (0.2, 0.1, 0.0), (45.07, 7.68), 27.0, 0.35, 2.0, 900002, True))
    p = parser_mod.Parser(messages=M(), logger=L(), connection=C())
    p.add_data(stream)
    got = {}
    while True:
        pkt = p.read_packet(_suppress_warning=True)
        if pkt is None:
            break
        got[pkt["id"]] = pkt
    assert set(got) == {N.ID_AHRS, N.ID_BT, N.ID_ALTI, N.ID_INS}
    assert got[N.ID_AHRS]["ahrsData.heading"] == pytest.approx(350.0)
    assert got[N.ID_BT]["velocityX"] == pytest.approx(0.3) and got[N.ID_BT]["status.xVelocityValid"]
    assert got[N.ID_ALTI]["altimeterDistance"] == pytest.approx(2.0)
    assert got[N.ID_INS]["positionFrameX"] == pytest.approx(1.5) and math.isclose(got[N.ID_INS]["latitude"], 45.07)
