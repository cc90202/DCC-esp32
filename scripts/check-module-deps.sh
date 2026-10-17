#!/usr/bin/env bash
# Module dependency rules for the firmware crate, checked on the module graph
# of both targets (host and riscv32): much of the crate only exists on one.
# The HAL-free crates it is built on (crates/dcc-protocol, crates/z21) are
# checked the same way, with all their features on, and may have no cycle at
# all: the allowlist is the firmware's.
#
# - Only module-to-module "uses" edges count. Any edge is lifted to the two
#   sibling modules under the closest common ancestor, so a::x::y -> a::z
#   becomes a::x -> a::z, and a::b -> c becomes a -> c.
# - An edge between a module and its own ancestor or descendant is not part of
#   cycle detection: a parent that re-exports its child while the child uses
#   `super::` items is idiomatic Rust, so a -> a::b -> a is accepted. A cycle
#   that also goes through a third module is still caught, because the lifted
#   edges carry it: a -> a::b -> c -> a fails as a -> c -> a.
# - Modules mounted with #[path] are renamed after the file they live in, so
#   `dcc_runtime` (dcc/engine.rs) is checked as `dcc::engine`.
# - A forbidden direction below fails the check.
# - A forbidden crate below is checked on the source text instead: the
#   `--no-externs` graph has no edge into another crate, and a path through
#   the firmware's re-export shim (`crate::z21::X`) resolves to that crate too.
#   Comments (nested block comments too) and string, raw-string and char
#   literals are blanked first; then any line under src/<module>/ (or
#   src/<module>.rs) naming the crate as a whole word (`z21::`, `::z21`,
#   `use z21`, `{z21, ...}`) fails.
# - A cycle fails the check, unless every edge of it is listed in
#   scripts/module-cycles-allowlist.txt. That list only shrinks: a listed edge
#   that is no longer part of a cycle fails, and so does a listed edge missing
#   from the list at CYCLE_BASE_REF (default origin/main). When the base has no
#   list yet, the current one is accepted with a notice.
#
# Usage: scripts/check-module-deps.sh [--dot-dir DIR]
#        scripts/check-module-deps.sh --self-test
# --dot-dir reads host.dot and riscv32.dot from DIR instead of running
# cargo-modules (and, when present, DIR/<crate>/{host,riscv32}.dot for each
# HAL-free crate); --self-test runs the rules on built-in graphs. Requires
# cargo-modules (except with those options), git and python3.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cycles_allowlist="$root/scripts/module-cycles-allowlist.txt"

# "from -> to": no module under `from` may use a module under `to`.
forbidden=(
    "dcc -> net"
    "dcc -> z21"
    "dcc -> application"
)

# "module -> crate": no file of the firmware module may name the workspace
# crate. The modules of dcc-protocol cannot use z21 at all: Cargo rejects the
# dependency cycle.
forbidden_crates=(
    "dcc -> z21"
)

targets=(x86_64-unknown-linux-gnu:host riscv32imac-unknown-none-elf:riscv32)

# HAL-free workspace crates checked with an empty cycle list.
crates=(dcc-protocol z21)

base_ref="${CYCLE_BASE_REF:-origin/main}"

dot_dir=""
self_test=0
case "${1:-}" in
    --dot-dir) dot_dir="${2:?--dot-dir needs a directory}" ;;
    --self-test) self_test=1 ;;
    "") ;;
    *) echo "usage: scripts/check-module-deps.sh [--dot-dir DIR | --self-test]" >&2; exit 2 ;;
esac

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# Arguments: root, DOT directory, #[path] grep output, cycle list, base cycle
# list ("" when the base has none), base name, forbidden rules...
read -r -d '' checker <<'PY' || true
import os
import re
import sys

root, dot_dir, mounts_path, cycles_path, base_path, base_name, *forbidden_rules = sys.argv[1:]
labels = ["host", "riscv32"]


def file_module(path):
    """Module path of a source file under src/: src/a/b.rs -> a::b."""
    rel = os.path.relpath(path, os.path.join(root, "src"))
    parts = rel[:-3].split(os.sep)
    if parts[-1] == "mod":
        parts.pop()
    if parts in (["lib"], ["main"]):
        return []
    return parts


def parse_mounts(path):
    """Maps each #[path]-mounted module to the module path of its file."""
    renames = {}
    lines = open(path, encoding="utf-8").read().splitlines()
    for index, line in enumerate(lines):
        match = re.match(r'(.+?):\d+:\s*#\[path\s*=\s*"([^"]+)"\]', line)
        if not match:
            continue
        declaring, target = match.group(1), match.group(2)
        name = None
        for follow in lines[index + 1:index + 4]:
            found = re.search(r'\bmod\s+(\w+)\s*;', follow)
            if found:
                name = found.group(1)
                break
        if name is None:
            sys.exit(f"check-module-deps: no `mod` after #[path] in {declaring}")
        declaring_abs = os.path.join(root, declaring)
        parent = file_module(declaring_abs)
        if os.path.basename(declaring) in ("lib.rs", "main.rs", "mod.rs"):
            base_dir = os.path.dirname(declaring_abs)
        else:
            base_dir = declaring_abs[:-3]
        resolved = os.path.normpath(os.path.join(base_dir, target))
        renames["::".join(parent + [name])] = "::".join(file_module(resolved))
    return renames


renames = parse_mounts(mounts_path)


def rename(module):
    for old, new in renames.items():
        if module == old or module.startswith(old + "::"):
            return new + module[len(old):]
    return module


edge_re = re.compile(r'^\s*"([^"]+)" -> "([^"]+)" \[label="uses"')
def module_parts(node):
    """Module path of a DOT node below the crate root; the root itself is []."""
    path = "::".join(node.split("::")[1:])
    return [part for part in rename(path).split("::") if part]


# Per target: lifted sibling edge -> the original module edges behind it.
graphs = {}
for label in labels:
    edges = {}
    with open(os.path.join(dot_dir, f"{label}.dot"), encoding="utf-8") as handle:
        text = handle.read()
    if "digraph" not in text:
        sys.exit(f"check-module-deps: {label}.dot is not a DOT graph")
    for line in text.splitlines():
        match = edge_re.match(line)
        if not match:
            continue
        source, target = module_parts(match.group(1)), module_parts(match.group(2))
        common = 0
        while common < min(len(source), len(target)) and source[common] == target[common]:
            common += 1
        if common == len(source) or common == len(target):
            continue
        lifted = ("::".join(source[:common + 1]), "::".join(target[:common + 1]))
        edges.setdefault(lifted, set()).add(f"{'::'.join(source)} -> {'::'.join(target)}")
    if not edges:
        sys.exit(f"check-module-deps: no module edges in {label}.dot")
    graphs[label] = edges


def strongly_connected(edges):
    adjacency = {}
    for source, target in edges:
        adjacency.setdefault(source, []).append(target)
        adjacency.setdefault(target, [])
    index, low, stack, on_stack, components = {}, {}, [], set(), []
    counter = [0]

    def visit(node):
        index[node] = low[node] = counter[0]
        counter[0] += 1
        stack.append(node)
        on_stack.add(node)
        for successor in adjacency[node]:
            if successor not in index:
                visit(successor)
                low[node] = min(low[node], low[successor])
            elif successor in on_stack:
                low[node] = min(low[node], index[successor])
        if low[node] == index[node]:
            component = set()
            while True:
                member = stack.pop()
                on_stack.discard(member)
                component.add(member)
                if member == node:
                    break
            if len(component) > 1:
                components.append(component)

    for node in sorted(adjacency):
        if node not in index:
            visit(node)
    return components


def under(module, prefix):
    return module == prefix or module.startswith(prefix + "::")


def load_edges(path):
    """Reads "a -> b" lines; "#" starts a comment."""
    listed = set()
    with open(path, encoding="utf-8") as handle:
        for number, raw in enumerate(handle, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [part.strip() for part in line.split("->")]
            if len(parts) != 2 or not all(parts):
                sys.exit(f"check-module-deps: {path}:{number}: expected 'a -> b'")
            listed.add(tuple(parts))
    return listed


allowed_cycle_edges = load_edges(cycles_path)

errors = []
if base_path:
    for source, target in sorted(allowed_cycle_edges - load_edges(base_path)):
        errors.append(f"{source} -> {target} is listed but not in the list at {base_name}: "
                      "the list only shrinks")
else:
    print(f"notice: {base_name} has no scripts/module-cycles-allowlist.txt; "
          "the current list is accepted as the starting point")

cycle_edges_seen = set()
for label in labels:
    edges = graphs[label]
    for rule in forbidden_rules:
        source_prefix, target_prefix = (part.strip() for part in rule.split("->"))
        for source, target in sorted(edges):
            if under(source, source_prefix) and under(target, target_prefix):
                errors.append(f"{label}: {', '.join(sorted(edges[(source, target)]))} "
                              f"breaks the rule '{rule}'")
    components = strongly_connected(edges)
    for component in components:
        inside = {edge for edge in edges if edge[0] in component and edge[1] in component}
        cycle_edges_seen |= inside
        unlisted = sorted(inside - allowed_cycle_edges)
        if unlisted:
            errors.append(f"{label}: cycle among {', '.join(sorted(component))}; unlisted edges: "
                          + ", ".join(f"{a} -> {b}" for a, b in unlisted))
    print(f"{label}: {len(edges)} module edges, {len(components)} cycle(s)"
          + "".join(f"\n  cycle: {', '.join(sorted(c))}" for c in components))

for source, target in sorted(allowed_cycle_edges - cycle_edges_seen):
    errors.append(f"{source} -> {target} is listed but no longer part of a cycle: remove it")

if renames:
    print("path mounts: " + ", ".join(f"{old} = {new}" for old, new in sorted(renames.items())))
if errors:
    sys.stdout.flush()
    print("Module dependency check failed:", file=sys.stderr)
    for error in errors:
        print(f"  {error}", file=sys.stderr)
    sys.exit(1)
print(f"Module dependencies follow the {len(forbidden_rules)} direction rule(s); "
      f"only the {len(allowed_cycle_edges)} listed cycle edge(s) form cycles.")
PY

# Arguments: source root, forbidden crate rules...
read -r -d '' crate_checker <<'PY' || true
import os
import re
import sys



RAW_STRING = re.compile(r'(?:b|c)?r(#*)"')
CHAR_LITERAL = re.compile(r"'(?:\\(?:x[0-9a-fA-F]{2}|u\{[0-9a-fA-F_]*\}|.)|[^\\'])'", re.S)


def strip_rust(text):
    """Blanks comments and string, byte-string, raw-string and char literals.

    Newlines are kept so line numbers stay valid; lifetimes are left alone.
    """
    out = []
    i, n = 0, len(text)

    def blank(chunk):
        out.append("".join(c if c == "\n" else " " for c in chunk))

    while i < n:
        c = text[i]
        prev = text[i - 1] if i else ""
        ident_before = prev.isalnum() or prev == "_"
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end < 0 else end
            blank(text[i:end])
            i = end
        elif text.startswith("/*", i):
            depth, j = 1, i + 2
            while j < n and depth:
                if text.startswith("/*", j):
                    depth, j = depth + 1, j + 2
                elif text.startswith("*/", j):
                    depth, j = depth - 1, j + 2
                else:
                    j += 1
            blank(text[i:j])
            i = j
        elif not ident_before and (m := RAW_STRING.match(text, i)):
            close = '"' + m.group(1)
            end = text.find(close, m.end())
            end = n if end < 0 else end + len(close)
            blank(text[i:end])
            i = end
        elif c == '"' or (not ident_before and text.startswith(('b"', 'c"'), i)):
            j = text.index('"', i) + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            blank(text[i:j + 1])
            i = j + 1
        elif c == "'" and (m := CHAR_LITERAL.match(text, i)):
            blank(text[i:m.end()])
            i = m.end()
        else:
            out.append(c)
            i += 1
    return "".join(out)


src, *rules = sys.argv[1:]
errors = []
for rule in rules:
    module, crate = (part.strip() for part in rule.split("->"))
    pattern = re.compile(r"(?<![\w$])" + re.escape(crate) + r"(?!\w)")
    files = []
    single = os.path.join(src, module + ".rs")
    if os.path.isfile(single):
        files.append(single)
    for directory, _, names in os.walk(os.path.join(src, module)):
        files.extend(os.path.join(directory, name) for name in names if name.endswith(".rs"))
    for path in sorted(files):
        with open(path, encoding="utf-8") as handle:
            code = strip_rust(handle.read())
            for number, line in enumerate(code.split("\n"), 1):
                if pattern.search(line):
                    rel = os.path.relpath(path, os.path.dirname(src))
                    errors.append(f"{rel}:{number}: names the crate '{crate}', "
                                  f"which breaks the rule '{rule}'")
if errors:
    print("Crate dependency check failed:", file=sys.stderr)
    for error in errors:
        print(f"  {error}", file=sys.stderr)
    sys.exit(1)
print(f"No firmware module names a crate it may not use ({len(rules)} rule(s)).")
PY

# Runs the crate rules on a source root: SRC, then the rules.
check_crates() {
    python3 -c "$crate_checker" "$@"
}

# Runs the checker on: DOT directory, #[path] grep output, cycle list, base
# cycle list or "", base name, then the forbidden rules.
check() {
    python3 -c "$checker" "$root" "$@"
}

if ((self_test)); then
    failures=0
    # Writes a DOT graph with the given "a -> b" module edges to DIR/host.dot
    # and DIR/riscv32.dot; x -> y keeps every graph non-empty.
    graph() {
        local dir="$1" edge
        shift
        mkdir -p "$dir"
        {
            printf 'digraph {\n'
            for edge in "x -> y" "$@"; do
                printf '    "dcc_esp32::%s" -> "dcc_esp32::%s" [label="uses"];\n' "${edge%% -> *}" "${edge#* -> }"
            done
            printf '}\n'
        } >"$dir/host.dot"
        cp "$dir/host.dot" "$dir/riscv32.dot"
    }
    # Case: name, expected outcome (pass|fail), expected output regex, DOT
    # directory, cycle list, base list, then forbidden rules.
    expect() {
        local name="$1" outcome="$2" pattern="$3" output status=0
        shift 3
        output="$(check "$@" 2>&1)" || status=$?
        if { [[ "$outcome" == pass ]] && ((status != 0)); } ||
            { [[ "$outcome" == fail ]] && ((status == 0)); } ||
            ! grep -Eq -- "$pattern" <<<"$output"; then
            printf 'self-test FAILED: %s (expected %s, exit %d, /%s/):\n%s\n' \
                "$name" "$outcome" "$status" "$pattern" "$output" >&2
            failures=$((failures + 1))
        else
            printf 'self-test ok: %s\n' "$name"
        fi
    }
    : >"$tmp/mounts.txt"
    : >"$tmp/empty.txt"
    printf 'boot -> net\nnet -> boot\n' >"$tmp/base.txt"
    printf 'boot -> net\nnet -> boot\napplication -> boot\nboot -> application\n' >"$tmp/grown.txt"

    graph "$tmp/listed" "boot -> net" "net -> boot"
    expect "listed cycle, list equal to the base" pass 'cycle: boot, net' \
        "$tmp/listed" "$tmp/mounts.txt" "$tmp/base.txt" "$tmp/base.txt" base
    expect "base without a list" pass '^notice: ' \
        "$tmp/listed" "$tmp/mounts.txt" "$tmp/base.txt" "" base
    graph "$tmp/grown" "boot -> net" "net -> boot" "application::status -> boot" "boot::error -> application"
    expect "new cycle edges appended to the list" fail \
        'application -> boot is listed but not in the list at base' \
        "$tmp/grown" "$tmp/mounts.txt" "$tmp/grown.txt" "$tmp/base.txt" base
    expect "unlisted cycle" fail 'cycle among application, boot' \
        "$tmp/grown" "$tmp/mounts.txt" "$tmp/base.txt" "$tmp/base.txt" base
    graph "$tmp/stale" "boot -> net"
    expect "listed edge no longer in a cycle" fail 'boot -> net is listed but no longer part of a cycle' \
        "$tmp/stale" "$tmp/mounts.txt" "$tmp/base.txt" "$tmp/base.txt" base
    graph "$tmp/parent" "a -> a::child" "a::child -> a"
    expect "parent and child using each other are accepted" pass '0 cycle' \
        "$tmp/parent" "$tmp/mounts.txt" "$tmp/empty.txt" "$tmp/empty.txt" base
    graph "$tmp/through" "a -> a::b" "a::b -> c" "c -> a"
    expect "cycle through a parent, its child and a third module" fail 'cycle among a, c' \
        "$tmp/through" "$tmp/mounts.txt" "$tmp/empty.txt" "$tmp/empty.txt" base
    graph "$tmp/forbidden" "dcc::scheduler -> net::wifi"
    expect "forbidden direction" fail "dcc::scheduler -> net::wifi breaks the rule 'dcc -> net'" \
        "$tmp/forbidden" "$tmp/mounts.txt" "$tmp/empty.txt" "$tmp/empty.txt" base "${forbidden[@]}"
    printf 'src/lib.rs:1:#[path = "dcc/engine.rs"]\nsrc/lib.rs-2-pub(crate) mod dcc_runtime;\n' >"$tmp/mounts-engine.txt"
    graph "$tmp/mounted" "dcc_runtime -> net::wifi"
    expect "forbidden direction from a #[path] module" fail "dcc::engine -> net::wifi breaks the rule" \
        "$tmp/mounted" "$tmp/mounts-engine.txt" "$tmp/empty.txt" "$tmp/empty.txt" base "${forbidden[@]}"

    # Writes FILE under DIR/src with the given lines.
    source_file() {
        local dir="$1" file="$2"
        shift 2
        mkdir -p "$(dirname "$dir/src/$file")"
        printf '%s\n' "$@" >"$dir/src/$file"
    }
    # Case: name, expected outcome, output regex, source root, then rules.
    expect_crates() {
        local name="$1" outcome="$2" pattern="$3" output status=0
        shift 3
        output="$(check_crates "$@" 2>&1)" || status=$?
        if { [[ "$outcome" == pass ]] && ((status != 0)); } ||
            { [[ "$outcome" == fail ]] && ((status == 0)); } ||
            ! grep -Eq -- "$pattern" <<<"$output"; then
            printf 'self-test FAILED: %s (expected %s, exit %d, /%s/):\n%s\n' \
                "$name" "$outcome" "$status" "$pattern" "$output" >&2
            failures=$((failures + 1))
        else
            printf 'self-test ok: %s\n' "$name"
        fi
    }
    source_file "$tmp/crate-direct" dcc/channels.rs "use z21::HEADER_XBUS;"
    expect_crates "module using a forbidden crate" fail \
        "dcc/channels.rs:1: names the crate 'z21', which breaks the rule 'dcc -> z21'" \
        "$tmp/crate-direct/src" "${forbidden_crates[@]}"
    source_file "$tmp/crate-shim" dcc/scheduler/runtime.rs "fn f() {}" "    let _ = crate::z21::HEADER_XBUS;"
    expect_crates "forbidden crate through the firmware shim" fail "runtime.rs:2: names the crate 'z21'" \
        "$tmp/crate-shim/src" "${forbidden_crates[@]}"
    source_file "$tmp/crate-group" dcc.rs "use crate::{net, z21};"
    expect_crates "forbidden crate in a grouped import of src/dcc.rs" fail "dcc.rs:1: names the crate 'z21'" \
        "$tmp/crate-group/src" "${forbidden_crates[@]}"
    source_file "$tmp/crate-clean" dcc/mod.rs "// z21::X is only named in a comment" "fn z21_ready() {}" \
        "let lan_z21 = 1; // z21::Y"
    source_file "$tmp/crate-clean" net/udp.rs "use z21::HEADER_XBUS;"
    expect_crates "comments, longer names and other modules are accepted" pass '1 rule' \
        "$tmp/crate-clean/src" "${forbidden_crates[@]}"
    source_file "$tmp/crate-url" dcc/diag.rs 'let _ = ("udp://", z21::HEADER_XBUS);'
    expect_crates "forbidden crate after a string holding //" fail "diag.rs:1: names the crate 'z21'" \
        "$tmp/crate-url/src" "${forbidden_crates[@]}"
    source_file "$tmp/crate-char" dcc/diag.rs "fn f<'a>(_: &'a u8) {}" "let _ = ('\"', z21::HEADER_XBUS);"
    expect_crates "forbidden crate after a lifetime and a quote char literal" fail \
        "diag.rs:2: names the crate 'z21'" "$tmp/crate-char/src" "${forbidden_crates[@]}"
    source_file "$tmp/crate-literals" dcc/diag.rs 'let _ = "z21-udp";' '/* z21 */ fn f() {}' \
        '/* outer /* z21 */ z21 */' 'let _ = r#"a " z21 "#;' 'let _ = b"z21";' 'let _ = "two' 'lines z21";' \
        'let _ = ("//", 1); // z21::X' 'fn g() { z21_ready(); }'
    expect_crates "strings, raw strings and nested block comments are accepted" pass '1 rule' \
        "$tmp/crate-literals/src" "${forbidden_crates[@]}"

    graph "$tmp/protocol" "packet -> encoder" "encoder::tail -> packet"
    expect "protocol crate: any cycle fails against the empty list" fail 'cycle among encoder, packet' \
        "$tmp/protocol" "$tmp/mounts.txt" "$tmp/empty.txt" "$tmp/empty.txt" "(none)"

    ((failures == 0)) || { echo "check-module-deps: $failures self-test case(s) failed" >&2; exit 1; }
    echo "All self-test cases passed."
    exit 0
fi

# Writes DIR/host.dot and DIR/riscv32.dot for one package; extra arguments go
# to cargo-modules.
module_graphs() {
    local package="$1" dir="$2" entry target label
    shift 2
    mkdir -p "$dir"
    for entry in "${targets[@]}"; do
        target="${entry%%:*}" label="${entry#*:}"
        (cd "$root" && cargo modules dependencies -p "$package" --lib --target "$target" "$@" \
            --no-externs --no-sysroot --no-fns --no-traits --no-types) >"$dir/$label.dot" 2>"$dir/$label.err" ||
            { cat "$dir/$label.err" >&2; echo "check-module-deps: cargo-modules failed for $package on $target" >&2; exit 1; }
    done
}

# Crates from `crates` that have graphs, each under DOT_DIR/<crate>.
crate_dot_dirs=()
if [[ -z "$dot_dir" ]]; then
    command -v cargo-modules >/dev/null ||
        { echo "check-module-deps: cargo-modules not found (cargo install cargo-modules)" >&2; exit 1; }
    dot_dir="$tmp"
    module_graphs dcc-esp32 "$dot_dir"
    for crate in "${crates[@]}"; do
        module_graphs "$crate" "$tmp/$crate" --all-features
        crate_dot_dirs+=("$crate")
    done
else
    for crate in "${crates[@]}"; do
        [[ ! -d "$dot_dir/$crate" ]] || crate_dot_dirs+=("$crate")
    done
fi

# #[path] mounts, as grep output: the attribute line and the lines after it.
(cd "$root" && grep -rn --include='*.rs' -A3 '^[[:space:]]*#\[path *= *"' src || true) >"$tmp/path-mounts.txt"

base_list=""
if git -C "$root" cat-file -e "$base_ref:scripts/module-cycles-allowlist.txt" 2>/dev/null; then
    git -C "$root" show "$base_ref:scripts/module-cycles-allowlist.txt" >"$tmp/base-cycles.txt"
    base_list="$tmp/base-cycles.txt"
fi

# The HAL-free crates: no #[path] mounts, no direction rules, and an empty
# cycle list that is also its own base, so any cycle fails.
: >"$tmp/no-mounts.txt"
: >"$tmp/no-cycles.txt"
for crate in "${crate_dot_dirs[@]}"; do
    echo "$crate:"
    check "$dot_dir/$crate" "$tmp/no-mounts.txt" "$tmp/no-cycles.txt" "$tmp/no-cycles.txt" "(none)"
done

((${#crate_dot_dirs[@]} == 0)) || echo "dcc-esp32:"
check_crates "$root/src" "${forbidden_crates[@]}"
check "$dot_dir" "$tmp/path-mounts.txt" "$cycles_allowlist" "$base_list" "$base_ref" "${forbidden[@]}"
