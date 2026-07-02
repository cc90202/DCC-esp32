#!/usr/bin/env bash
while true; do
    python3 tools/z21_pom_read.py 192.168.1.58 --addr 3 --cv 29 --no-railcom-address --timeout 4.5
    sleep 0.5
done
