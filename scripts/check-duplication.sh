#!/usr/bin/env bash
# Duplicated code and unused dependencies.
#
# - jscpd over src/, crates/dcc-protocol/src/ and crates/z21/src/ (Rust, clones
#   of 50 tokens or more) fails when the share of duplicated lines rises above
#   max_percentage. Lower the value when a change removes duplication; never
#   raise it.
# - cargo machete fails on a dependency that no crate of the workspace uses.
#
# Usage: scripts/check-duplication.sh [jscpd|machete]
# With an argument only that part runs. Uses jscpd from PATH when it is version 4.3.0, otherwise npx jscpd@4.3.0
# (Node.js required). Requires cargo-machete and python3.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
max_percentage="0.85"
jscpd_version="4.3.0"

part="${1:-all}"
case "$part" in
    all | jscpd | machete) ;;
    *) echo "usage: scripts/check-duplication.sh [jscpd|machete]" >&2; exit 2 ;;
esac

check_machete() {
    command -v cargo-machete >/dev/null ||
        { echo "check-duplication: cargo-machete not found (cargo install cargo-machete)" >&2; exit 1; }
    (cd "$root" && cargo machete)
}

if [[ "$part" == machete ]]; then
    check_machete
    exit
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

if command -v jscpd >/dev/null && [[ "$(jscpd --version 2>/dev/null)" == "$jscpd_version" ]]; then
    jscpd=(jscpd)
elif command -v npx >/dev/null; then
    jscpd=(npx --yes "jscpd@$jscpd_version")
else
    echo "check-duplication: jscpd $jscpd_version not found and npx is unavailable" >&2
    exit 1
fi

(cd "$root" && "${jscpd[@]}" src crates/dcc-protocol/src crates/z21/src --format rust --min-tokens 50 --silent \
    --reporters console,json --output "$tmp")
[[ -f "$tmp/jscpd-report.json" ]] || { echo "check-duplication: jscpd wrote no report" >&2; exit 1; }

python3 - "$tmp/jscpd-report.json" "$max_percentage" <<'PY'
import json
import sys

report_path, limit = sys.argv[1], float(sys.argv[2])
report = json.load(open(report_path, encoding="utf-8"))
total = report["statistics"]["total"]
if total["sources"] == 0:
    sys.exit("check-duplication: jscpd found no Rust sources")
percentage = float(total["percentage"])
if percentage > limit:
    for clone in report["duplicates"]:
        first, second = clone["firstFile"], clone["secondFile"]
        print(f"  {clone['lines']} lines: {first['name']}:{first['start']}-{first['end']}"
              f" = {second['name']}:{second['start']}-{second['end']}", file=sys.stderr)
    sys.exit(f"Duplicated lines {percentage}% are above the recorded {limit}%: "
             "remove the new clone among those listed above")
note = f" (lower max_percentage to {percentage})" if percentage < limit else ""
print(f"Duplicated lines {percentage}% within the recorded {limit}%{note}.")
PY

if [[ "$part" == all ]]; then
    check_machete
fi
