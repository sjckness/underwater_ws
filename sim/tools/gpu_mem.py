"""Peak GPU memory of the uw processes (host side, nvidia-smi): `python3 tools/gpu_mem.py OUT.json [interval_s]`.

Samples until SIGINT/SIGTERM; for every process whose command line contains `uw.runner` (the container's Python,
seen with its host PID), the peak of nvidia-smi's per-process used memory, plus the peak total used on the GPU.
"""
import json
import signal
import subprocess
import sys
import time


def sample():
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout
    procs = {}
    for line in out.strip().splitlines():
        pid, mem = (x.strip() for x in line.split(","))
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode()
        except OSError:
            continue
        if "uw.runner" in cmd:
            procs[pid] = (int(mem), " ".join(cmd.split()[3:6]))
    total = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True).stdout.strip()
    return procs, int(total)


def main():
    path, dt = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    peak, total_peak, stop = {}, 0, []
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.append(1))
    t0 = time.time()
    while not stop:
        procs, total = sample()
        total_peak = max(total_peak, total)
        for pid, (mem, cmd) in procs.items():
            if mem > peak.get(pid, (0, ""))[0]:
                peak[pid] = (mem, cmd)
        time.sleep(dt)
    json.dump({"duration_s": time.time() - t0, "gpu_total_used_peak_mib": total_peak,
               "uw_processes_peak_mib": {pid: {"mib": m, "cmd": c} for pid, (m, c) in peak.items()}},
              open(path, "w"), indent=1)


if __name__ == "__main__":
    main()
