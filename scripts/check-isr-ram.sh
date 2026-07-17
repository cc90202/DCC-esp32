#!/usr/bin/env bash
set -euo pipefail

elf="${1:-target/riscv32imac-unknown-none-elf/release/dcc-esp32}"
symbols="$(readelf -sW "$elf")"

for symbol in \
    __esp_hal_internal_rmt_interrupt \
    __esp_hal_internal_cutout_timer_interrupt \
    write_data_to_rmt_ram
do
    line="$(printf '%s\n' "$symbols" | grep "$symbol" || true)"
    if [[ -z "$line" ]]; then
        printf 'missing ISR symbol: %s\n' "$symbol" >&2
        exit 1
    fi

    address="$(printf '%s\n' "$line" | awk '{print $2}')"
    if [[ "$address" != 408* ]]; then
        printf 'ISR symbol is not in internal RAM: %s at %s\n' "$symbol" "$address" >&2
        exit 1
    fi
done

printf 'RMT and cutout ISR paths are linked in internal RAM.\n'
