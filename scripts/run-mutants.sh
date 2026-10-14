#!/usr/bin/env bash
# Mutation testing of the host-compiled code: the dcc-esp32 library tests and
# the dcc-ota and dcc-ota-format tests, on x86_64. It takes minutes, so it is
# not part of scripts/check-all.sh or CI.
#
# --all-features is required for dcc-esp32: without bench-diag a plain host
# library build fails on two fields of src/railcom/pipeline.rs that only that
# feature reads.
#
# Usage: scripts/run-mutants.sh OUTPUT_DIR [--changed [BASE]] [cargo-mutants options...]
# --changed limits the run to the code that differs from BASE (default
# origin/main), working tree and untracked .rs files included: the end-of-phase
# check. Without it every mutant runs (about an hour), which is only needed to
# regenerate scripts/mutants-baseline.txt.
# Writes OUTPUT_DIR/firmware and OUTPUT_DIR/ota, each with mutants.out
# (caught.txt, missed.txt, timeout.txt, unviable.txt, outcomes.json) and
# mutants.log, then prints the missed mutants and, apart, those
# scripts/mutants-baseline.txt does not list.
# Extra options go to cargo mutants, e.g. `--file src/dcc/**` to limit the
# run, or `--list` to only list the mutants.
# Exits non-zero when cargo mutants does (missed mutants, failed baseline).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
usage="usage: scripts/run-mutants.sh OUTPUT_DIR [--changed [BASE]] [cargo-mutants options...]"

if [[ $# -lt 1 || "$1" == -* ]]; then
    echo "$usage" >&2
    exit 2
fi
out="$1"
shift

command -v cargo-mutants >/dev/null ||
    { echo "run-mutants: cargo-mutants not found (cargo install cargo-mutants)" >&2; exit 1; }
mkdir -p "$out"
out="$(cd "$out" && pwd)"

if [[ "${1:-}" == "--changed" ]]; then
    shift
    base="origin/main"
    if [[ $# -gt 0 && "$1" != -* ]]; then
        base="$1"
        shift
    fi
    diff="$out/changed.diff"
    (
        cd "$root"
        git diff "$base" -- '*.rs'
        # New files are not in git diff until they are added.
        git ls-files --others --exclude-standard -- '*.rs' | while read -r file; do
            git diff --no-index -- /dev/null "$file" || true
        done
    ) >"$diff"
    printf 'Changed code: %s against %s (%d lines)\n' "${diff}" "$base" "$(wc -l <"$diff")"
    set -- --in-diff "$diff" "$@"
fi

status=0
run() {
    local name="$1"
    shift
    mkdir -p "$out/$name"
    (cd "$root" && cargo mutants --cargo-arg=--target=x86_64-unknown-linux-gnu \
        --output "$out/$name" "$@") 2>&1 | tee "$out/$name/mutants.log" || status=$?
}
run firmware --package dcc-esp32 --all-features "$@" -- --lib -- --test-threads=1
run ota --package dcc-ota --package dcc-ota-format "$@"

missed_files=()
for name in firmware ota; do
    file="$out/$name/mutants.out/missed.txt"
    [[ -f "$file" ]] && missed_files+=("$file")
done
if ((${#missed_files[@]} > 0)); then
    # missed.txt has no newline after its last line.
    missed="$(awk 1 "${missed_files[@]}")"
    printf 'Missed mutants: %d\n' "$(grep -c . <<<"$missed" || true)"
    [[ -z "$missed" ]] || sed 's/^/  /' <<<"$missed"

    # Known survivors live in scripts/mutants-baseline.txt. Line and column
    # are dropped before comparing, so edits elsewhere in a file do not make
    # an old survivor look new.
    baseline="$root/scripts/mutants-baseline.txt"
    if [[ -f "$baseline" ]]; then
        strip() { grep -v '^#' | sed -E 's/^([^:]+):[0-9]+:[0-9]+:/\1:/' | sort -u; }
        new="$(comm -13 <(strip <"$baseline") <(strip <<<"$missed"))"
        printf 'Missed mutants not in %s: %d\n' "${baseline#"$root"/}" \
            "$(grep -c . <<<"$new" || true)"
        [[ -z "$new" ]] || sed 's/^/  /' <<<"$new"
    fi
fi
exit "$status"
