#!/usr/bin/env bash
# Runs every local quality gate in order and stops at the first failure.
#
# Each gate's output goes to a log file; a passing gate prints one summary line
# with its last line of output, a failing gate prints the end of its log.
#
# Usage: scripts/check-all.sh [GATE...]
#        scripts/check-all.sh --list
# With gate names only those run, in the order below. CHECK_ALL_VERBOSE=1
# streams every gate's output as well.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_dir="${CARGO_TARGET_DIR:-$root/target}/check-all"
cd "$root"

# name|command, run with bash -c from the repository root.
gates=(
    "fmt|cargo fmt --all -- --check"
    "clippy-host|cargo clippy-host -- -D warnings"
    "clippy-esp-lib|cargo clippy --lib --target riscv32imac-unknown-none-elf -- -D warnings"
    "clippy-esp|cargo clippy-esp -- -D warnings"
    "test-host|cargo test-host"
    "docs-host|RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps --target x86_64-unknown-linux-gnu"
    "docs-esp|RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --target riscv32imac-unknown-none-elf"
    "release-build|cargo build-esp-release"
    "isr-ram|bash scripts/check-isr-ram.sh --self-test && bash scripts/check-isr-ram.sh"
    "complexity|bash scripts/check-complexity.sh"
    "module-deps|bash scripts/check-module-deps.sh --self-test && bash scripts/check-module-deps.sh"
    "duplication|bash scripts/check-duplication.sh jscpd"
    "machete|bash scripts/check-duplication.sh machete"
    "deny|cargo deny --workspace check"
    "typos|typos src crates tools scripts"
)

if [[ "${1:-}" == "--list" ]]; then
    for gate in "${gates[@]}"; do printf '%-15s %s\n' "${gate%%|*}" "${gate#*|}"; done
    exit 0
fi

selected=("$@")
for wanted in "${selected[@]}"; do
    found=0
    for gate in "${gates[@]}"; do [[ "${gate%%|*}" == "$wanted" ]] && found=1; done
    ((found)) || { echo "check-all: unknown gate '$wanted' (see --list)" >&2; exit 2; }
done

mkdir -p "$log_dir"
start_all=$SECONDS
count=0
for gate in "${gates[@]}"; do
    name="${gate%%|*}" command="${gate#*|}"
    if ((${#selected[@]} > 0)); then
        run=0
        for wanted in "${selected[@]}"; do [[ "$wanted" == "$name" ]] && run=1; done
        ((run)) || continue
    fi
    count=$((count + 1))
    log="$log_dir/$(printf '%02d' "$count")-$name.log"
    start=$SECONDS
    status=0
    if [[ -n "${CHECK_ALL_VERBOSE:-}" ]]; then
        bash -c "$command" 2>&1 | tee "$log" || status=$?
    else
        bash -c "$command" >"$log" 2>&1 || status=$?
    fi
    elapsed=$((SECONDS - start))
    if ((status != 0)); then
        printf 'FAIL  %-15s %4ds  exit %d: %s\n' "$name" "$elapsed" "$status" "$command"
        printf -- '----- last 40 lines of %s -----\n' "$log"
        tail -n 40 "$log"
        printf 'check-all: stopped at %s; the gates after it did not run.\n' "$name" >&2
        exit 1
    fi
    if grep -q '^test result:' "$log"; then
        last="$(awk '/^test result:/ { suites++; passed += $4; ignored += $8 }
            END { printf "%d tests passed, %d ignored, in %d suites", passed, ignored, suites }' "$log")"
    else
        last="$(grep -v '^[[:space:]]*$' "$log" | tail -n 1 | sed 's/^[[:space:]]*//' | cut -c1-110 || true)"
    fi
    printf 'ok    %-15s %4ds  %s\n' "$name" "$elapsed" "${last:-clean}"
done
printf 'All %d gates passed in %ds (logs in %s).\n' "$count" "$((SECONDS - start_all))" "$log_dir"
