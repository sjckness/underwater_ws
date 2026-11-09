#!/usr/bin/env python3
"""M7: summarize tegrastats + nvidia-smi logs written by run_with_load.sh. Usage: summarize_load.py <tag>"""
import os, re, statistics, sys
from pathlib import Path
tag = sys.argv[1]
R = Path(__file__).resolve().parent / 'results'
cpu_avg, ram, vin, gpu_pw = [], [], [], []
for line in open(f'{R}/{tag}_tegrastats.log'):
    m = re.search(r'CPU \[([^\]]*)\]', line)
    if m:
        loads = [int(x.split('%')[0]) for x in m.group(1).split(',') if '%' in x]
        cpu_avg.append(sum(loads) / len(loads))
    m = re.search(r'RAM (\d+)/(\d+)MB', line); m and ram.append(int(m.group(1)))
    m = re.search(r'VIN (\d+)mW', line); m and vin.append(int(m.group(1)))
    m = re.search(r'VDD_GPU (\d+)mW', line); m and gpu_pw.append(int(m.group(1)))
gpu = []
for line in open(f'{R}/{tag}_gpu.log'):
    p = [x.strip() for x in line.split(',')]
    if len(p) >= 2 and p[1].endswith('%'):
        gpu.append(int(p[1].rstrip(' %')))
def s(x): return f'mean {statistics.mean(x):.1f} / max {max(x):.1f} (n={len(x)})' if x else 'n/a'
print(f'[{tag}] CPU avg over 14 cores %: {s(cpu_avg)}')
print(f'[{tag}] RAM used MB: {s(ram)}')
print(f'[{tag}] GPU util % (nvidia-smi): {s(gpu)}')
print(f'[{tag}] VDD_GPU mW: {s(gpu_pw)}   VIN mW: {s(vin)}')
