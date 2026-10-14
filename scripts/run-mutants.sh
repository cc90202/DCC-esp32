#!/usr/bin/env bash
# Mutation testing of the host-compiled firmware code (dcc-esp32 library tests
# on x86_64). It takes minutes, so it is not part of scripts/check-all.sh or CI.
#
# --all-features is required: without bench-diag a plain host library build
# fails on two fields of src/railcom/pipeline.rs that only that feature reads.
#
# Usage: scripts/run-mutants.sh OUTPUT_DIR [cargo-mutants options...]
# Writes OUTPUT_DIR/mutants.out (caught.txt, missed.txt, timeout.txt,
# unviable.txt, outcomes.json) and OUTPUT_DIR/mutants.log, then prints the
# missed mutants. Extra options go to cargo mutants, e.g. `--file src/dcc/**`
# to limit the run, or `--list` to only list the mutants.
# Exits non-zero when cargo mutants does (missed mutants, failed baseline).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -lt 1 || "$1" == -* ]]; then
    echo "usage: scripts/run-mutants.sh OUTPUT_DIR [cargo-mutants options...]" >&2
    exit 2
fi
out="$1"
shift

command -v cargo-mutants >/dev/null ||
    { echo "run-mutants: cargo-mutants not found (cargo install cargo-mutants)" >&2; exit 1; }
mkdir -p "$out"
out="$(cd "$out" && pwd)"

status=0
(cd "$root" && cargo mutants --package dcc-esp32 --all-features \
    --cargo-arg=--target=x86_64-unknown-linux-gnu --output "$out" "$@" \
    -- --lib -- --test-threads=1) 2>&1 | tee "$out/mutants.log" || status=$?

if [[ -f "$out/mutants.out/missed.txt" ]]; then
    missed="$(wc -l <"$out/mutants.out/missed.txt")"
    printf 'Missed mutants: %d (%s)\n' "$missed" "$out/mutants.out/missed.txt"
    sed 's/^/  /' "$out/mutants.out/missed.txt"
fi
exit "$status"
