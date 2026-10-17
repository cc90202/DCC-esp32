#!/usr/bin/env bash
# Cognitive complexity ratchet for the firmware crate (src/) and the HAL-free
# crates it is built on (crates/dcc-protocol/src/, crates/z21/src/).
#
# Fails when a function has cognitive complexity above the limit and is not
# listed in scripts/complexity-allowlist.txt, when a listed function rises above
# its recorded value, or when a listed value is stale (the function dropped
# below its value or under the limit, or is gone): the list only shrinks.
# With a base revision available (COMPLEXITY_BASE, default "main"), an entry
# that is new or higher than on that revision also fails. A file renamed since
# the base (as `git diff -M --name-status` reports it) keeps its entries: the
# base entry under the old path counts as the same entry under the new one.
# Git only sees a rename once the new path is tracked (git add or git mv).
#
# Cyclomatic complexity is printed next to each function as the number of paths
# a test suite has to cover; it does not block, since a long flat `match` is
# cyclomatically large but easy to read.
#
# Usage: scripts/check-complexity.sh
# Requires rust-code-analysis-cli and python3.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
allowlist="$root/scripts/complexity-allowlist.txt"
limit="${COMPLEXITY_LIMIT:-15}"
base="${COMPLEXITY_BASE:-main}"

command -v rust-code-analysis-cli >/dev/null ||
    { echo "check-complexity: rust-code-analysis-cli not found (cargo install rust-code-analysis-cli)" >&2; exit 1; }
[[ -f "$allowlist" ]] || { echo "check-complexity: allowlist not found: $allowlist" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

(cd "$root" && rust-code-analysis-cli -m -p src -p crates/dcc-protocol/src -p crates/z21/src -O json) >"$tmp/metrics.json"
[[ -s "$tmp/metrics.json" ]] || { echo "check-complexity: no metrics produced" >&2; exit 1; }

base_list="$tmp/base-allowlist.txt"
if git -C "$root" rev-parse --verify -q "$base:scripts/complexity-allowlist.txt" >/dev/null; then
    git -C "$root" show "$base:scripts/complexity-allowlist.txt" >"$base_list"
    # "R<score> TAB old TAB new" for every file renamed since the base.
    git -C "$root" diff -M --name-status "$base" -- >"$tmp/renames.txt"
else
    base_list=""
    : >"$tmp/renames.txt"
fi

python3 - "$tmp/metrics.json" "$allowlist" "$limit" "$base_list" "$base" "$tmp/renames.txt" <<'PY'
import json
import sys

metrics_path, allowlist_path, limit, base_path, base_name, renames_path = sys.argv[1:7]
limit = int(limit)


def load_renames(path):
    """Maps each path renamed since the base to its current path."""
    renames = {}
    with open(path, encoding="utf-8") as handle:
        for raw in handle:
            fields = raw.rstrip("\n").split("\t")
            if len(fields) == 3 and fields[0].startswith("R"):
                renames[fields[1]] = fields[2]
    return renames


def renamed(entries, renames):
    """Re-keys "<file>::<function>" entries whose file was renamed."""
    result = {}
    for key, value in entries.items():
        file_name, separator, function = key.partition("::")
        result[renames.get(file_name, file_name) + separator + function] = value
    return result


def load_list(path):
    """Reads "<value> <file>::<function>" lines; "#" starts a comment."""
    entries = {}
    with open(path, encoding="utf-8") as handle:
        for number, raw in enumerate(handle, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            value, _, key = line.partition(" ")
            if not value.isdigit() or not key.strip():
                sys.exit(f"check-complexity: {path}:{number}: expected '<value> <file>::<function>'")
            entries[key.strip()] = int(value)
    return entries


# Functions are keyed by file and nesting ("Type::method::<anonymous>"); two
# functions with the same key (cfg variants of one function) keep the larger
# value.
functions = {}


def walk(space, path, file_name):
    for child in space.get("spaces", []):
        qualified = path + [child.get("name") or "<anonymous>"]
        if child["kind"] == "function":
            key = f"{file_name}::{'::'.join(qualified)}"
            cognitive = int(child["metrics"]["cognitive"]["sum"])
            cyclomatic = int(child["metrics"]["cyclomatic"]["sum"])
            old = functions.get(key)
            if old is None or (cognitive, cyclomatic) > old:
                functions[key] = (cognitive, cyclomatic)
        walk(child, qualified, file_name)


with open(metrics_path, encoding="utf-8") as handle:
    for line in handle:
        if line.strip():
            unit = json.loads(line)
            walk(unit, [], unit["name"])

if not functions:
    sys.exit("check-complexity: no functions found in the metrics")

allowed = load_list(allowlist_path)
errors = []
for key, (cognitive, _) in sorted(functions.items()):
    if key in allowed:
        recorded = allowed[key]
        if cognitive > recorded:
            errors.append(f"{key}: cognitive {cognitive}, above its recorded {recorded}")
        elif cognitive <= limit:
            errors.append(f"{key}: cognitive {cognitive}, now within the limit: remove its entry")
        elif cognitive < recorded:
            errors.append(f"{key}: cognitive {cognitive}, below its recorded {recorded}: lower the entry")
    elif cognitive > limit:
        errors.append(f"{key}: cognitive {cognitive}, above the limit of {limit} and not listed")
for key in sorted(set(allowed) - set(functions)):
    errors.append(f"{key}: listed but not found: remove its entry")

if base_path:
    previous = renamed(load_list(base_path), load_renames(renames_path))
    for key, value in sorted(allowed.items()):
        if key not in previous:
            errors.append(f"{key}: entry not present on {base_name}: the list only shrinks")
        elif value > previous[key]:
            errors.append(f"{key}: entry raised from {previous[key]} to {value} since {base_name}")

print(f"{'cog':>4} {'cyc':>4}  function (cognitive above 10)")
for key, (cognitive, cyclomatic) in sorted(functions.items(), key=lambda item: (-item[1][0], item[0])):
    if cognitive > 10:
        print(f"{cognitive:>4} {cyclomatic:>4}  {key}")

def above(index, threshold):
    return sum(1 for value in functions.values() if value[index] > threshold)

top_key, (top_value, _) = max(functions.items(), key=lambda item: item[1][0])
print(f"functions: {len(functions)}; cognitive >10: {above(0, 10)}, >{limit}: {above(0, limit)}, "
      f">25: {above(0, 25)}; highest {top_value} ({top_key})")
print(f"cyclomatic (not blocking) >10: {above(1, 10)}, >15: {above(1, 15)}, >20: {above(1, 20)}")
if not base_path:
    print(f"note: no allowlist on {base_name}; the shrink-only check against it was skipped")

if errors:
    sys.stdout.flush()
    print("Cognitive complexity check failed:", file=sys.stderr)
    for error in errors:
        print(f"  {error}", file=sys.stderr)
    sys.exit(1)
print(f"Cognitive complexity within the limit of {limit} or the recorded allowlist ({len(allowed)} entries).")
PY
