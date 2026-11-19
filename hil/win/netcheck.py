#!/usr/bin/env python3
"""Test di rete punto-punto per il link work <-> Thor (solo libreria standard: stesso file su Windows e Linux).

    server:  python netcheck.py server [--port 5301]
    client:  python netcheck.py client HOST [--port 5301] [--secs 8]

Il client esegue tre prove contro il server:
  1. TCP client -> server (throughput, misurato dal server)
  2. TCP server -> client (throughput, sulla stessa connessione, aperta dal client)
  3. UDP: 300 pacchetti da 64 byte a 100 Hz verso il server, che li rimanda indietro: RTT e perdite.

Per sapere se il firewall di Windows blocca il traffico IN INGRESSO (quello che serve al DDS dal Thor), il server va
avviato su Windows e il client sul Thor: se la prova 1 non si connette, o la prova 3 perde il 100%, e' bloccato.
I throughput sono indicativi: Python non satura un link da 5 Gbit/s come iperf3.
"""
from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

CHUNK = bytes(1 << 18)                  # 256 KiB di zeri
UDP_PACKETS, UDP_HZ = 300, 100


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


# ------------------------------------------------------------------------------------------------------- server
def handle(conn: socket.socket, addr) -> None:
    with conn:
        hdr = recv_exact(conn, 2)
        if hdr is None:
            return
        mode, secs = hdr[:1], hdr[1]
        print(f"[netcheck] TCP da {addr[0]}:{addr[1]} modo {mode.decode()} {secs} s", flush=True)
        if mode == b"U":                                    # il client invia, noi contiamo
            n, t0 = 0, time.perf_counter()
            while True:
                b = conn.recv(1 << 20)
                if not b:
                    break
                n += len(b)
            conn.sendall(struct.pack("!Qd", n, time.perf_counter() - t0))
        elif mode == b"D":                                  # noi inviamo per `secs` secondi, poi chiudiamo
            end = time.perf_counter() + secs
            while time.perf_counter() < end:
                conn.sendall(CHUNK)


def serve_udp(port: int) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", port))
    first = True
    while True:
        try:
            data, addr = s.recvfrom(2048)
        except ConnectionResetError:                        # Windows: ICMP di ritorno di un invio precedente
            continue
        if first:
            print(f"[netcheck] UDP da {addr[0]}:{addr[1]}", flush=True)
            first = False
        s.sendto(data, addr)


def server(port: int) -> None:
    threading.Thread(target=serve_udp, args=(port,), daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", port))
    srv.listen(8)
    print(f"[netcheck] server in ascolto su TCP+UDP {port} (Ctrl+C per uscire)", flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("[netcheck] chiuso")


# ------------------------------------------------------------------------------------------------------- client
def tcp_test(host: str, port: int, mode: bytes, secs: int) -> float:
    s = socket.create_connection((host, port), timeout=10)
    with s:
        s.sendall(mode + bytes([secs]))
        if mode == b"U":
            end = time.perf_counter() + secs
            while time.perf_counter() < end:
                s.sendall(CHUNK)
            s.shutdown(socket.SHUT_WR)
            reply = recv_exact(s, 16)
            if reply is None:
                raise OSError("il server non ha risposto")
            n, dt = struct.unpack("!Qd", reply)
        else:
            n, t0 = 0, None
            while True:
                b = s.recv(1 << 20)
                if not b:
                    break
                if t0 is None:
                    t0 = time.perf_counter()
                n += len(b)
            if n == 0 or t0 is None:
                raise OSError("nessun dato ricevuto")
            dt = time.perf_counter() - t0
    return n * 8 / dt / 1e6


def udp_test(host: str, port: int) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(False)
    rtts: list[float] = []

    def drain() -> None:
        """Legge le risposte gia' arrivate. Un solo thread, senza attese bloccanti: l'RTT non include ritardi di Python
        (un thread ricevente separato verrebbe affamato dal ciclo di attesa attiva)."""
        while True:
            try:
                data, _ = s.recvfrom(2048)
            except BlockingIOError:
                return
            except ConnectionResetError:                    # Windows: ICMP di un invio precedente
                continue
            now = time.perf_counter()
            if len(data) >= 16:
                _, ts = struct.unpack("!Qd", data[:16])
                rtts.append((now - ts) * 1e3)

    pad = bytes(48)
    t0 = time.perf_counter()
    for i in range(UDP_PACKETS):
        while time.perf_counter() < t0 + i / UDP_HZ:        # attesa attiva: il sleep() di Windows e' a ~15 ms
            drain()
        s.sendto(struct.pack("!Qd", i, time.perf_counter()) + pad, (host, port))
    end = time.perf_counter() + 1.0
    while time.perf_counter() < end:
        drain()
    got = len(rtts)
    loss = 100.0 * (1 - got / UDP_PACKETS)
    if not got:
        print(f"[netcheck] UDP echo: 0/{UDP_PACKETS} risposte (perdita 100%)")
        return
    rtts.sort()

    def pct(p: float) -> float:
        return rtts[min(got - 1, int(p * got))]

    print(f"[netcheck] UDP echo: {got}/{UDP_PACKETS} risposte, perdita {loss:.1f}%  RTT ms: "
          f"min {rtts[0]:.3f}  p50 {pct(0.50):.3f}  p95 {pct(0.95):.3f}  p99 {pct(0.99):.3f}  max {rtts[-1]:.3f}")


def client(host: str, port: int, secs: int) -> None:
    print(f"[netcheck] client -> {host}:{port}, {secs} s per prova TCP")
    for label, mode in (("TCP client -> server", b"U"), ("TCP server -> client", b"D")):
        try:
            print(f"[netcheck] {label}: {tcp_test(host, port, mode, secs):8.0f} Mbit/s", flush=True)
        except OSError as exc:
            print(f"[netcheck] {label}: FALLITA ({exc})", flush=True)
    udp_test(host, port)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("server")
    sp.add_argument("--port", type=int, default=5301)
    cp = sub.add_parser("client")
    cp.add_argument("host")
    cp.add_argument("--port", type=int, default=5301)
    cp.add_argument("--secs", type=int, default=8)
    a = ap.parse_args()
    if a.cmd == "server":
        server(a.port)
    else:
        client(a.host, a.port, a.secs)


if __name__ == "__main__":
    main()
