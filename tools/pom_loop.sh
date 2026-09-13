#!/usr/bin/env bash
# Keeps the loco in the command station's refresh cycle so RailCom cutouts
# stay frequent, without hogging the controller lease (a Z21 app can keep
# driving alongside this loop).
#
# Usage: tools/pom_loop.sh [HOST] [INTERVAL_SECONDS]
#   HOST              command station IP (default 192.168.1.69)
#   INTERVAL_SECONDS  pause between reads (default 3)
HOST="${1:-192.168.1.69}"
INTERVAL="${2:-3}"
while true; do
    python3 tools/z21_pom_read.py "$HOST" --addr 3 --cv 29 --no-railcom-address --timeout 1
    sleep "$INTERVAL"
done
