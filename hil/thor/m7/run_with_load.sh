#!/usr/bin/env bash
# M7: run a command while logging Thor load (tegrastats 1 Hz, nvidia-smi GPU util 1 Hz) and the
# command's own CPU time. Usage: run_with_load.sh <tag> <command...>
# Logs go to <this script's dir>/results/<tag>_{tegrastats,gpu,proc}.log
set -u
tag=$1; shift
R="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/results"; mkdir -p "$R"
tegrastats --interval 1000 --logfile "$R/${tag}_tegrastats.log" & TP=$!
nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory --format=csv,noheader -lms 1000 > "$R/${tag}_gpu.log" 2>&1 & NP=$!
t0=$(date +%s.%N)
"$@" & CP=$!
wait $CP; rc=$?
t1=$(date +%s.%N)
kill $TP $NP 2>/dev/null; wait $TP $NP 2>/dev/null
echo "rc=$rc wall_s=$(echo "$t1 - $t0" | bc)" > "$R/${tag}_proc.log"
exit $rc
