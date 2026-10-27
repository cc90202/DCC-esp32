#!/usr/bin/env bash
# Checks that the DCC interrupt handlers run entirely from internal RAM.
#
# 1. Each root symbol below must be linked in internal RAM.
# 2. Every function reachable from a root is disassembled and analysed:
#    - a call or jump into RAM is followed, one into the mask ROM is accepted;
#    - a call or jump into flash fails unless scripts/isr-ram-allowlist.txt
#      names it and the call site lies on a path that ends in a panic;
#    - a flash address the function reads, or computes with, fails;
#    - a jump or call through a register fails unless it is one of the known
#      forms validated below (jump table, waker vtable, interrupt dispatch).
#
# A handler that calls into flash stalls on every flash-cache miss, so its
# latency would depend on what else the cache holds and change from build to
# build.
#
# Usage: scripts/check-isr-ram.sh [ELF]
#        scripts/check-isr-ram.sh --self-test [ELF]
# --self-test runs the analyser and the allowlist rules on built-in fixtures and
# checks that a failing disassembler (LLVM_OBJDUMP=/bin/false) makes the gate
# fail on ELF.
# LLVM_OBJDUMP, LLVM_NM and LLVM_READOBJ override the tool discovery below;
# ISR_RAM_VERBOSE=1 lists the allowlisted calls that were found.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
script_path="$script_dir/$(basename "${BASH_SOURCE[0]}")"
default_elf="${CARGO_TARGET_DIR:-target}/riscv32imac-unknown-none-elf/release/dcc-esp32"
allowlist="${ISR_RAM_ALLOWLIST:-$script_dir/isr-ram-allowlist.txt}"

# ESP32-C6 address map (esp-hal memory.x): internal SRAM, the mask ROM whose
# functions run without the cache, and the cached external flash mapping.
ram_start=0x40800000
ram_end=0x40880000
rom_start=0x40000000
rom_end=0x40050000
flash_start=0x42000000
flash_end=0x43000000

# Name substrings of the functions that must run from RAM: esp-hal's interrupt
# dispatcher, the two handlers, the RMT RAM writer, and the embassy waker the
# handlers reach through the RawWakerVTable when they signal a task.
roots=(
    handle_interrupts
    __esp_hal_internal_rmt_interrupt
    __esp_hal_internal_cutout_timer_interrupt
    write_data_to_rmt_ram
    16embassy_executor3raw5waker4wake
)

# The waker vtable: the handlers load `wake` from it on every notification.
waker_vtable=16embassy_executor3raw5waker6VTABLE
waker_wake=16embassy_executor3raw5waker4wake
# RawWakerVTable layout: clone, wake, wake_by_ref, drop (4 bytes each).
waker_wake_slot=4
# Demangled-name substrings of the functions allowed to call through a waker
# vtable: the embassy-sync notifications used by the handlers. Every waker in
# this firmware comes from the embassy executor, whose vtable is checked above.
waker_callers=(
    "embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ()>>::signal"
    "embassy_sync::channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, u8, 1>>::try_send"
)
# esp-hal calls the bound handlers through this RAM table; the DCC handlers
# are roots of their own.
dispatch_caller=handle_interrupts
dispatch_table=__EXTERNAL_INTERRUPTS

# Names of functions that never return: an allowlisted call site counts as
# cold only when every path from it reaches a call to one of these.
panic_pattern='panic|unwrap_failed|expect_failed|index_fail|index_order_fail'

fail() {
    printf 'check-isr-ram: %s\n' "$*" >&2
    exit 1
}

# Number parser shared by every awk program in this script: "0x" hex, with an
# optional leading "-"; anything without the prefix is read as decimal. Plain
# hex fields (nm, readobj) are passed as hex("0x" field).
read -r -d '' awk_hex <<'AWK' || true
function hex(s,    neg, i, c, v) {
    neg = 0
    if (substr(s, 1, 1) == "-") { neg = 1; s = substr(s, 2) }
    s = tolower(s)
    if (substr(s, 1, 2) == "0x") {
        s = substr(s, 3)
        v = 0
        for (i = 1; i <= length(s); i++) {
            c = index("0123456789abcdef", substr(s, i, 1)) - 1
            if (c < 0) break
            v = v * 16 + c
        }
    } else {
        v = s + 0
    }
    return neg ? -v : v
}
AWK

# The analyser reads one function's disassembly and prints its references:
#   call <hex> <cold>               direct call or jump leaving the function,
#                                   conditional branches included
#   arg <hex> <hex> <cold>          flash address passed in a0-a7 to the call
#                                   (second value) without being read first
#   data <hex>                      flash address read or computed with
#   jumptable <pc> <hex>            jump through a word loaded from table <hex>
#   tablecall <pc> <hex>            call through a word loaded from table <hex>
#   vcall <pc> <offset>             call through a word loaded at <offset> from
#                                   an unknown pointer (a vtable slot)
#   indirect <pc> <instruction>     any other register jump
# <cold> is 1 when every path from the instruction, the call itself included,
# reaches a call to a non-returning function (the PANICS addresses) without a
# return, a register jump or a jump out of the function on the way; 0 otherwise.
# Loops count as not cold.
#
# Registers carry abstract values: c:N constant, t:N pointer derived from the
# constant N by arithmetic, d:N word loaded from a table at N, u:O word loaded
# at offset O from an unknown pointer. Copies and add/sub/shNadd propagate
# them, a flash-derived source winning over any other operand; a join keeps a flash-derived value if any predecessor has one, so a
# flash pointer cannot be lost at a merge. Blocks reached only through a jump
# table start from the merged state of the table jumps.
read -r -d '' analyser <<'AWK' || true
function wrap(v) {
    v = v % 4294967296
    return v < 0 ? v + 4294967296 : v
}
function in_flash(v) { return v >= flash_lo && v < flash_hi }
function kind(val) { return substr(val, 1, 1) }
function num(val) { return substr(val, 3) + 0 }
function flashy(val) {
    return (kind(val) == "c" || kind(val) == "t") && in_flash(num(val))
}
# A constant or derived pointer into RAM: the base of a RAM table.
function addressy(val) {
    return (kind(val) == "c" || kind(val) == "t") && num(val) >= ram_lo && num(val) < ram_hi
}
function parse(s, arr,    n, parts, i, eq) {
    split("", arr)
    n = split(s, parts, ";")
    for (i = 1; i <= n; i++) {
        eq = index(parts[i], "=")
        if (eq > 0) arr[substr(parts[i], 1, eq - 1)] = substr(parts[i], eq + 1)
    }
}
function dump(arr,    i, s) {
    s = ""
    for (i = 1; i <= nregs; i++)
        if ((REGS[i] in arr) && arr[REGS[i]] != "") s = s REGS[i] "=" arr[REGS[i]] ";"
    return s
}
function merge(a, b,    i, r, va, vb, out) {
    parse(a, MA)
    parse(b, MB)
    split("", MO)
    for (i = 1; i <= nregs; i++) {
        r = REGS[i]
        va = (r in MA) ? MA[r] : ""
        vb = (r in MB) ? MB[r] : ""
        if (va == vb) out = va
        else if (flashy(va)) out = "t:" num(va)
        else if (flashy(vb)) out = "t:" num(vb)
        else out = ""
        MO[r] = out
    }
    return dump(MO)
}
function reg_of(operand,    r) {
    r = operand
    sub(/^.*\(/, "", r)
    sub(/\)$/, "", r)
    return r
}
function use(r) {
    if (emit && (r in R) && flashy(R[r])) printf "data %08x\n", num(R[r])
}
function access(base, offset,    v, address) {
    v = (base in R) ? R[base] : ""
    if (kind(v) == "c") {
        address = wrap(num(v) + offset)
        if (emit && in_flash(address)) printf "data %08x\n", address
        return "d:" address
    }
    if (kind(v) == "t") {
        if (emit && in_flash(num(v))) printf "data %08x\n", num(v)
        return "d:" num(v)
    }
    return "u:" offset
}
function call_out(target, i,    r, k) {
    target = wrap(target)
    if (target >= lo && target < hi) return
    if (collect) { CALLT[i] = target; return }
    if (!emit) return
    printf "call %08x %d\n", target, COLD[i]
    for (k = 0; k <= 7; k++) {
        r = "a" k
        if ((r in R) && flashy(R[r]))
            printf "arg %08x %08x %d\n", num(R[r]), target, COLD[i]
    }
}
function clobber(    k) {
    delete R["ra"]
    for (k = 0; k <= 6; k++) delete R["t" k]
    for (k = 0; k <= 7; k++) delete R["a" k]
}
# Applies instruction i to R[]; prints references when emit is set.
function step(i,    op, n, a, k, moff, mreg, rd, v, v1, v2, target, link, slot) {
    op = OP[i]
    n = split(ARGS[i], a, ",")
    moff = ""; mreg = ""
    if (n >= 1 && match(a[n], /^-?(0x)?[0-9a-f]*\(/)) {
        moff = substr(a[n], 1, RLENGTH - 1)
        if (moff == "") moff = "0"
        mreg = reg_of(a[n])
    }
    if (op ~ /^(lb|lh|lw|lbu|lhu)$/) {
        R[a[1]] = access(mreg, hex(moff))
        return
    }
    if (op ~ /^(sb|sh|sw)$/) {
        use(a[1])
        access(mreg, hex(moff))
        return
    }
    if (op ~ BRANCH) {
        for (k = 1; k < n; k++) use(a[k])
        call_out(hex(a[n]), i)
        return
    }
    if (op == "lui") { R[a[1]] = "c:" wrap(hex(a[2]) * 4096); return }
    if (op == "auipc") { R[a[1]] = "c:" wrap(PC[i] + hex(a[2]) * 4096); return }
    if (op == "li") { R[a[1]] = "c:" wrap(hex(a[2])); return }
    if (op == "mv") { R[a[1]] = (a[2] in R) ? R[a[2]] : ""; return }
    if (op == "addi") {
        v = (a[2] in R) ? R[a[2]] : ""
        if (kind(v) == "c") R[a[1]] = "c:" wrap(num(v) + hex(a[3]))
        else if (kind(v) == "t") R[a[1]] = v
        else R[a[1]] = ""
        return
    }
    if (op ~ /^(add|sub|sh1add|sh2add|sh3add)$/) {
        v1 = (a[2] in R) ? R[a[2]] : ""
        v2 = (a[3] in R) ? R[a[3]] : ""
        if (flashy(v1)) R[a[1]] = "t:" num(v1)
        else if (flashy(v2)) R[a[1]] = "t:" num(v2)
        else if (addressy(v1)) R[a[1]] = "t:" num(v1)
        else if (addressy(v2)) R[a[1]] = "t:" num(v2)
        else if (kind(v1) == "c" || kind(v1) == "t") R[a[1]] = "t:" num(v1)
        else if (kind(v2) == "c" || kind(v2) == "t") R[a[1]] = "t:" num(v2)
        else R[a[1]] = ""
        return
    }
    if (op == "andi") {
        v = (a[2] in R) ? R[a[2]] : ""
        use(a[2])
        R[a[1]] = (kind(v) == "d") ? v : ""
        return
    }
    if (op == "jal" || op == "j" || op == "call" || op == "tail") {
        call_out(hex(a[n]), i)
        if (op == "jal" || op == "call") clobber()
        return
    }
    if (op == "jalr" || op == "jr") {
        link = 0
        if (op == "jalr") {
            rd = (n >= 2) ? a[1] : "ra"
            link = (rd != "zero")
        }
        if (mreg == "") { mreg = a[n]; moff = "0" }
        if (n == 3) { mreg = a[2]; moff = a[3] }
        v = (mreg in R) ? R[mreg] : ""
        if (kind(v) == "c") {
            call_out(num(v) + hex(moff), i)
        } else if (emit && kind(v) == "d" && hex(moff) == 0) {
            printf "%s %08x %08x\n", link ? "tablecall" : "jumptable", PC[i], num(v)
        } else if (emit && kind(v) == "u" && hex(moff) == 0 && link) {
            printf "vcall %08x %d\n", PC[i], num(v)
        } else if (emit) {
            printf "indirect %08x %s %s\n", PC[i], op, ARGS[i]
        }
        if (link) clobber()
        return
    }
    if (op == "ret" || op == "mret") return
    # Generic instruction: first operand written, the others read.
    for (k = 2; k <= n; k++) use(reg_of(a[k]))
    if (n >= 1) R[a[1]] = ""
}
function falls_through(i,    op, n, a) {
    op = OP[i]
    if (op == "j" || op == "jr" || op == "tail" || op == "ret" || op == "mret") return 0
    if (op == "unimp" || op == "c.unimp") return 0
    if (op == "jalr" || op == "jal") {
        n = split(ARGS[i], a, ",")
        if (n >= 2 && a[1] == "zero") return 0
    }
    return 1
}
# Least fixpoint of "every path from i ends in a panic call".
function solve_cold(    i, changed_cold, pass, t, v, exits) {
    for (i = 1; i <= count; i++) COLD[i] = 0
    for (pass = 0; pass < 4 * count + 4; pass++) {
        changed_cold = 0
        for (i = count; i >= 1; i--) {
            if (COLD[i]) continue
            v = 1
            t = local_target(i)
            # A branch, jump or return whose path leaves the function: cold
            # only when it lands on a non-returning function.
            exits = (OP[i] ~ BRANCH || !falls_through(i)) && t <= 0
            if ((i in CALLT) && (CALLT[i] in PANIC)) {
                # A panic call, or a branch or tail jump into one: that path
                # ends here; a conditional branch still has its fall-through.
                if (!(OP[i] ~ BRANCH)) v = 1
                else v = (i < count) ? COLD[i + 1] : 0
            } else {
                if (exits) v = 0
                if (v && falls_through(i)) v = (i < count) ? COLD[i + 1] : 0
                if (v && t > 0) v = COLD[t]
            }
            if (v) { COLD[i] = 1; changed_cold = 1 }
        }
        if (!changed_cold) break
    }
}
function local_target(i,    op, n, a, t) {
    op = OP[i]
    if (!(op ~ BRANCH || op == "j")) return -1
    n = split(ARGS[i], a, ",")
    t = wrap(hex(a[n]))
    return (t in IDX) ? IDX[t] : -1
}
function propagate(from, state,    merged) {
    if (!(from in IN)) { IN[from] = state; changed = 1; return }
    merged = merge(IN[from], state)
    if (merged != IN[from]) { IN[from] = merged; changed = 1 }
}
# Fails the analysis when the register states have not settled after
# max_passes passes: the gate must not go on with an incomplete abstraction.
function solve(    i, out, t, pass) {
    for (pass = 0; pass < max_passes; pass++) {
        changed = 0
        for (i = 1; i <= count; i++) {
            if (!(i in IN)) continue
            parse(IN[i], R)
            emit = 0
            step(i)
            out = dump(R)
            if (OP[i] == "jr") { JT = (JT_SET ? merge(JT, out) : out); JT_SET = 1 }
            if (falls_through(i) && i < count) propagate(i + 1, out)
            t = local_target(i)
            if (t > 0) propagate(t, out)
        }
        if (!changed) return
    }
    printf "analyser: register states still changing after %d passes\n", max_passes > "/dev/stderr"
    exit 3
}
BEGIN {
    BRANCH = "^b(eq|ne|lt|ge|ltu|geu|eqz|nez|gez|lez|gtz|ltz|gt|le|gtu|leu)$"
    nregs = split("zero ra sp gp tp t0 t1 t2 s0 s1 a0 a1 a2 a3 a4 a5 a6 a7 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 t3 t4 t5 t6", REGS, " ")
    flash_lo = hex(flash_start); flash_hi = hex(flash_end)
    ram_lo = hex(ram_start); ram_hi = hex(ram_end)
    np = split(panics, plist, ",")
    for (k = 1; k <= np; k++) if (plist[k] != "") PANIC[wrap(hex("0x" plist[k]))] = 1
    lo = hex("0x" start); hi = hex("0x" stop)
    count = 0
}
/^ *[0-9a-f]+:/ {
    count++
    PC[count] = hex("0x" substr($1, 1, length($1) - 1))
    IDX[PC[count]] = count
    OP[count] = $2
    ops = ""
    for (i = 3; i <= NF && substr($i, 1, 1) != "<"; i++) ops = ops $i
    ARGS[count] = ops
}
END {
    if (count == 0) exit 0
    IN[1] = ""
    JT_SET = 0
    solve()
    # Code not reached yet is entered through a jump table (or never runs).
    for (;;) {
        seeded = 0
        for (i = 1; i <= count; i++) if (!(i in IN)) { IN[i] = JT_SET ? JT : ""; seeded = 1; break }
        if (!seeded) break
        solve()
    }
    collect = 1
    emit = 0
    for (i = 1; i <= count; i++) { parse(IN[i], R); step(i) }
    collect = 0
    solve_cold()
    emit = 1
    for (i = 1; i <= count; i++) { parse(IN[i], R); step(i) }
}
AWK
# Analyses one function, [START, STOP) in hex, whose disassembly is on stdin.
# PANICS is a comma-separated list of non-returning function addresses (hex);
# MAX_PASSES bounds the register-state fixpoint (64 when omitted).
analyse() {
    awk -v start="$1" -v stop="$2" -v panics="${3:-}" \
        -v flash_start="$flash_start" -v flash_end="$flash_end" \
        -v ram_start="$ram_start" -v ram_end="$ram_end" \
        -v max_passes="${4:-64}" "$awk_hex"$'\n'"$analyser"
}

# --- allowlist rules ---------------------------------------------------------

in_ram() {
    local address=$((16#$1))
    ((address >= ram_start && address < ram_end))
}

in_rom() {
    local address=$((16#$1))
    ((address >= rom_start && address < rom_end))
}

# Reads allowlist lines from stdin into allowed[]: an entry followed by
# "  # reason"; lines starting with "#" are comments. The reason needs
# whitespace before "#" because demangled closure names contain "{closure#0}".
allowed=()
load_allowlist() {
    local line entry
    allowed=()
    while IFS= read -r line || [[ -n "$line" ]]; do
        [[ "$line" =~ ^[[:space:]]*# ]] && continue
        entry="${line%%[[:space:]]#*}"
        entry="${entry#"${entry%%[![:space:]]*}"}"
        entry="${entry%"${entry##*[![:space:]]}"}"
        [[ -n "$entry" ]] && allowed+=("$entry")
    done
    return 0
}

# An allowlist entry is "callee" or "callee @ caller": each part must be a
# substring of the demangled name, and the caller part limits the entry to
# calls made from that function.
is_allowed() {
    local callee="$1" caller="$2" entry wanted_callee wanted_caller
    for entry in "${allowed[@]}"; do
        wanted_callee="$entry"
        wanted_caller=""
        if [[ "$entry" == *" @ "* ]]; then
            wanted_callee="${entry%% @ *}"
            wanted_caller="${entry#* @ }"
        fi
        [[ "$callee" == *"$wanted_callee"* && "$caller" == *"$wanted_caller"* ]] && return 0
    done
    return 1
}

is_panic() {
    [[ "$1" =~ $panic_pattern ]]
}

# Verdict on a direct call from CALLER into the flash function CALLEE, COLD
# being the analyser's cold flag for the call site: "allowed", "unlisted", or
# "hot" for a listed callee that is not a panic on a path that can return.
flash_call_verdict() {
    local callee="$1" caller="$2" cold="$3"
    if ! is_allowed "$callee" "$caller"; then
        printf 'unlisted\n'
    elif ! is_panic "$callee" && ((cold == 0)); then
        printf 'hot\n'
    else
        printf 'allowed\n'
    fi
}

# Succeeds when CALLER may pass a flash address to the callee at TARGET (hex)
# named CALLEE: only to a listed flash function on a panic path, never to RAM.
flash_arg_allowed() {
    local callee="$1" caller="$2" cold="$3" target="$4"
    ! in_ram "$target" && is_allowed "$callee" "$caller" &&
        { is_panic "$callee" || ((cold == 1)); }
}

# --- self-test ---------------------------------------------------------------

# Each case: name, expected line (regex, "!" prefix = must not appear),
# then the disassembly lines. Addresses are inside a pretend RAM function.
self_test_case() {
    local name="$1" expect="$2" output
    shift 2
    output="$(printf '%s\n' "$@" | analyse 40800000 40800100 42026000)"
    if [[ "$expect" == !* ]]; then
        if grep -Eq -- "${expect#!}" <<<"$output"; then
            printf 'self-test FAILED: %s: unexpected "%s" in:\n%s\n' "$name" "${expect#!}" "$output" >&2
            return 1
        fi
    elif ! grep -Eq -- "$expect" <<<"$output"; then
        printf 'self-test FAILED: %s: expected "%s" in:\n%s\n' "$name" "$expect" "$output" >&2
        return 1
    fi
    printf 'self-test ok: %s\n' "$name"
}

# Compares a gate rule's result with the expected one.
self_test_rule() {
    local name="$1" expect="$2" actual="$3"
    if [[ "$actual" != "$expect" ]]; then
        printf 'self-test FAILED: %s: expected "%s", got "%s"\n' "$name" "$expect" "$actual" >&2
        return 1
    fi
    printf 'self-test ok: %s\n' "$name"
}

self_test() {
    local elf="$1" failures=0
    i() { printf '%x:      \t%s\n' "$1" "$2"; }
    self_test_case "lui copied with mv, then loaded" '^data 42001020$' \
        "$(i 0x40800000 'lui	a0, 0x42001')" "$(i 0x40800004 'mv	a1, a0')" \
        "$(i 0x40800006 'lw	a2, 0x20(a1)')" "$(i 0x40800008 'ret')" || failures=$((failures + 1))
    self_test_case "auipc copied with mv, then loaded" '^data 42000000$' \
        "$(i 0x40800000 'auipc	a0, 0x1800')" "$(i 0x40800004 'mv	a1, a0')" \
        "$(i 0x40800006 'lw	a2, 0x0(a1)')" "$(i 0x40800008 'ret')" || failures=$((failures + 1))
    self_test_case "folded low part after add" '^data 42001000$' \
        "$(i 0x40800000 'lui	a5, 0x42001')" "$(i 0x40800004 'add	a4, a4, a5')" \
        "$(i 0x40800006 'lw	a3, 0x10(a4)')" "$(i 0x40800008 'ret')" || failures=$((failures + 1))
    self_test_case "sh2add index into flash table" '^data 42001000$' \
        "$(i 0x40800000 'lui	a5, 0x42001')" "$(i 0x40800004 'sh2add	a4, a0, a5')" \
        "$(i 0x40800008 'lw	a3, 0x0(a4)')" "$(i 0x4080000a 'ret')" || failures=$((failures + 1))
    self_test_case "flash pointer survives a join" '^data 4200[12]000$' \
        "$(i 0x40800000 'bnez	a0, 0x4080000a <f+0xa>')" "$(i 0x40800002 'lui	a1, 0x42001')" \
        "$(i 0x40800006 'j	0x4080000e <f+0xe>')" "$(i 0x4080000a 'lui	a1, 0x42002')" \
        "$(i 0x4080000e 'lw	a2, 0x0(a1)')" "$(i 0x40800010 'ret')" || failures=$((failures + 1))
    self_test_case "jump table in flash" '^jumptable 4080000a 42000d14$' \
        "$(i 0x40800000 'lui	a1, 0x42001')" "$(i 0x40800004 'addi	a1, a1, -0x2ec')" \
        "$(i 0x40800006 'add	a1, a1, a2')" "$(i 0x40800008 'lw	a1, 0x0(a1)')" \
        "$(i 0x4080000a 'jr	a1')" || failures=$((failures + 1))
    self_test_case "jump table read is flash data" '^data 42000d14$' \
        "$(i 0x40800000 'lui	a1, 0x42001')" "$(i 0x40800004 'addi	a1, a1, -0x2ec')" \
        "$(i 0x40800006 'add	a1, a1, a2')" "$(i 0x40800008 'lw	a1, 0x0(a1)')" \
        "$(i 0x4080000a 'jr	a1')" || failures=$((failures + 1))
    self_test_case "peripheral register access is clean" '!^data' \
        "$(i 0x40800000 'lui	a0, 0x60010')" "$(i 0x40800004 'lw	a1, 0x34(a0)')" \
        "$(i 0x40800006 'ret')" || failures=$((failures + 1))
    self_test_case "RAM table access is clean" '!^data' \
        "$(i 0x40800000 'lui	a3, 0x40802')" "$(i 0x40800004 'addi	a3, a3, 0x5c0')" \
        "$(i 0x40800008 'add	a3, a3, a0')" "$(i 0x4080000a 'lbu	a4, 0x0(a3)')" \
        "$(i 0x4080000e 'ret')" || failures=$((failures + 1))
    self_test_case "panic location passed to a call" '^arg 4200dbe4 42026000 ' \
        "$(i 0x40800000 'lui	a2, 0x4200e')" "$(i 0x40800004 'addi	a2, a2, -0x41c')" \
        "$(i 0x40800008 'auipc	ra, 0x1826')" "$(i 0x4080000c 'jalr	-0x8(ra)')" || failures=$((failures + 1))
    self_test_case "auipc and jalr call resolved" '^call 42026000 ' \
        "$(i 0x40800008 'auipc	ra, 0x1826')" "$(i 0x4080000c 'jalr	-0x8(ra)')" || failures=$((failures + 1))
    self_test_case "conditional branch out of the function" '^call 42000000 ' \
        "$(i 0x40800000 'beqz	a0, 0x42000000 <g>')" "$(i 0x40800004 'ret')" || failures=$((failures + 1))
    self_test_case "vtable slot call" '^vcall 40800004 4$' \
        "$(i 0x40800000 'lw	a1, 0x4(a0)')" "$(i 0x40800004 'jalr	a1')" \
        "$(i 0x40800006 'ret')" || failures=$((failures + 1))
    self_test_case "unknown register call" '^indirect 40800000 jalr a5$' \
        "$(i 0x40800000 'jalr	a5')" "$(i 0x40800002 'ret')" || failures=$((failures + 1))
    self_test_case "sh2add with the flash base second" '^data 42001000$' \
        "$(i 0x40800000 'li	a0, 0x1')" "$(i 0x40800002 'lui	a5, 0x42001')" \
        "$(i 0x40800006 'sh2add	a4, a0, a5')" "$(i 0x4080000a 'lw	a3, 0x0(a4)')" \
        "$(i 0x4080000c 'ret')" || failures=$((failures + 1))
    self_test_case "add with the flash base second" '^data 42001000$' \
        "$(i 0x40800000 'li	a0, 0x1')" "$(i 0x40800002 'lui	a5, 0x42001')" \
        "$(i 0x40800006 'add	a4, a0, a5')" "$(i 0x40800008 'lw	a3, 0x0(a4)')" \
        "$(i 0x4080000a 'ret')" || failures=$((failures + 1))
    # 42010000 stands for a logger call, 42026000 for a panic.
    self_test_case "logger straight into a panic is cold" '^call 42010000 1$' \
        "$(i 0x40800000 'jal	0x42010000 <log>')" "$(i 0x40800004 'jal	0x42026000 <panic>')" \
        || failures=$((failures + 1))
    self_test_case "logger with both branches panicking is cold" '^call 42010000 1$' \
        "$(i 0x40800000 'jal	0x42010000 <log>')" "$(i 0x40800004 'bnez	a0, 0x4080000c <f+0xc>')" \
        "$(i 0x40800008 'jal	0x42026000 <panic>')" "$(i 0x4080000c 'jal	0x42026000 <panic>')" \
        || failures=$((failures + 1))
    self_test_case "logger whose branch falls through to a return is not cold" '^call 42010000 0$' \
        "$(i 0x40800000 'jal	0x42010000 <log>')" "$(i 0x40800004 'bnez	a0, 0x4080000a <f+0xa>')" \
        "$(i 0x40800008 'ret')" "$(i 0x4080000a 'jal	0x42026000 <panic>')" \
        || failures=$((failures + 1))
    self_test_case "logger jumping over a panic block is not cold" '^call 42010000 0$' \
        "$(i 0x40800000 'jal	0x42010000 <log>')" "$(i 0x40800004 'j	0x4080000c <f+0xc>')" \
        "$(i 0x40800008 'jal	0x42026000 <panic>')" "$(i 0x4080000c 'ret')" \
        || failures=$((failures + 1))
    self_test_case "logger before an indirect tail jump is not cold" '^call 42010000 0$' \
        "$(i 0x40800000 'jal	0x42010000 <log>')" "$(i 0x40800004 'jalr	zero, 0x0(a5)')" \
        "$(i 0x40800008 'jal	0x42026000 <panic>')" || failures=$((failures + 1))
    self_test_case "logger in a loop is not cold" '^call 42010000 0$' \
        "$(i 0x40800000 'jal	0x42010000 <log>')" "$(i 0x40800004 'j	0x40800000 <f>')" \
        || failures=$((failures + 1))

    # A backward branch needs a second pass: one pass is not enough to settle.
    local loop
    loop="$(i 0x40800000 'lui	a1, 0x42001')"$'\n'"$(i 0x40800004 'bnez	a0, 0x40800000 <f>')"$'\n'"$(i 0x40800006 'ret')"
    if analyse 40800000 40800100 "" 1 <<<"$loop" >/dev/null 2>&1; then
        printf 'self-test FAILED: an unsettled fixpoint did not fail the analysis\n' >&2
        failures=$((failures + 1))
    else
        printf 'self-test ok: an unsettled fixpoint fails the analysis\n'
    fi
    if ! analyse 40800000 40800100 "" <<<"$loop" >/dev/null; then
        printf 'self-test FAILED: the loop did not settle within the default passes\n' >&2
        failures=$((failures + 1))
    else
        printf 'self-test ok: the loop settles within the default passes\n'
    fi

    # Gate rules on a fixture allowlist: comments, "{closure#0}" kept in an
    # entry, "callee @ caller" scoping, and the panic-path requirement.
    local saved_allowed=("${allowed[@]}")
    load_allowlist <<'LIST'
# comment
  defmt::export::fmt @ app::isr::{closure#0}  # reason with a # inside
core::panicking::panic_fmt  # panic from anywhere
LIST
    self_test_rule "allowlist parsing" "2|defmt::export::fmt @ app::isr::{closure#0}" \
        "${#allowed[@]}|${allowed[0]}" || failures=$((failures + 1))
    self_test_rule "listed callee from its caller on a panic path" allowed \
        "$(flash_call_verdict 'defmt::export::fmt::<u8>' 'app::isr::{closure#0}' 1)" || failures=$((failures + 1))
    self_test_rule "listed non-panic callee on a path that returns" hot \
        "$(flash_call_verdict 'defmt::export::fmt::<u8>' 'app::isr::{closure#0}' 0)" || failures=$((failures + 1))
    self_test_rule "listed callee from another caller" unlisted \
        "$(flash_call_verdict 'defmt::export::fmt::<u8>' app::other 1)" || failures=$((failures + 1))
    self_test_rule "listed panic callee on any path" allowed \
        "$(flash_call_verdict core::panicking::panic_fmt app::other 0)" || failures=$((failures + 1))
    self_test_rule "unlisted callee on a panic path" unlisted \
        "$(flash_call_verdict core::fmt::write 'app::isr::{closure#0}' 1)" || failures=$((failures + 1))
    self_test_rule "flash argument to a listed callee on a panic path" yes \
        "$(flash_arg_allowed 'defmt::export::fmt::<u8>' 'app::isr::{closure#0}' 1 42010000 && echo yes || echo no)" ||
        failures=$((failures + 1))
    self_test_rule "flash argument to a listed callee on a path that returns" no \
        "$(flash_arg_allowed 'defmt::export::fmt::<u8>' 'app::isr::{closure#0}' 0 42010000 && echo yes || echo no)" ||
        failures=$((failures + 1))
    self_test_rule "flash argument to a RAM function" no \
        "$(flash_arg_allowed core::panicking::panic_fmt app::other 1 40800010 && echo yes || echo no)" ||
        failures=$((failures + 1))
    allowed=("${saved_allowed[@]}")

    if [[ -f "$elf" ]]; then
        if LLVM_OBJDUMP=/bin/false bash "$script_path" "$elf" >/dev/null 2>&1; then
            printf 'self-test FAILED: the gate passed with LLVM_OBJDUMP=/bin/false\n' >&2
            failures=$((failures + 1))
        else
            printf 'self-test ok: a failing disassembler fails the gate\n'
        fi
        if LLVM_OBJDUMP=/bin/true bash "$script_path" "$elf" >/dev/null 2>&1; then
            printf 'self-test FAILED: the gate passed with an empty disassembly\n' >&2
            failures=$((failures + 1))
        else
            printf 'self-test ok: an empty disassembly fails the gate\n'
        fi
    else
        printf 'self-test FAILED: ELF not found for the disassembler cases: %s\n' "$elf" >&2
        failures=$((failures + 1))
    fi
    ((failures == 0)) || fail "$failures self-test case(s) failed"
    printf 'All self-test cases passed.\n'
}

if [[ "${1:-}" == "--self-test" ]]; then
    self_test "${2:-$default_elf}"
    exit 0
fi

# --- gate --------------------------------------------------------------------

elf="${1:-$default_elf}"

# Finds an LLVM binary: explicit override, then the active toolchain's
# llvm-tools component, then PATH, then any installed rustup toolchain. The
# disassembler only has to understand RISC-V, so any recent LLVM works.
find_llvm_tool() {
    local name="$1" override="$2" candidate sysroot host
    if [[ -n "$override" ]]; then
        [[ -x "$override" ]] || fail "$name override is not executable: $override"
        printf '%s\n' "$override"
        return
    fi
    if command -v rustc >/dev/null 2>&1; then
        sysroot="$(rustc --print sysroot 2>/dev/null || true)"
        host="$(rustc -vV 2>/dev/null | sed -n 's/^host: //p')"
        candidate="$sysroot/lib/rustlib/$host/bin/$name"
        if [[ -n "$sysroot" && -n "$host" && -x "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return
        fi
    fi
    if command -v "$name" >/dev/null 2>&1; then
        command -v "$name"
        return
    fi
    for candidate in "${RUSTUP_HOME:-$HOME/.rustup}"/toolchains/*/lib/rustlib/*/bin/"$name"; do
        if [[ -x "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return
        fi
    done
    fail "$name not found; install it with 'rustup component add llvm-tools' or set the override variable"
}

[[ -f "$elf" ]] || fail "ELF not found: $elf (build it with 'cargo build-esp-release')"
[[ -f "$allowlist" ]] || fail "allowlist not found: $allowlist"

objdump="$(find_llvm_tool llvm-objdump "${LLVM_OBJDUMP:-}")"
nm="$(find_llvm_tool llvm-nm "${LLVM_NM:-}")"
readobj="$(find_llvm_tool llvm-readobj "${LLVM_READOBJ:-}")"

nm_demangled="$("$nm" --defined-only --numeric-sort --print-size --demangle "$elf")" ||
    fail "llvm-nm failed on $elf"
mangled="$("$nm" --defined-only --print-size "$elf")" || fail "llvm-nm failed on $elf"
sections="$("$readobj" --elf-output-style=GNU --section-headers "$elf")" ||
    fail "llvm-readobj failed on $elf"

# Function symbols sorted by address: "address size name"; demangled names may
# contain spaces, so the name is everything after the second field.
symbols="$(awk 'NF >= 4 && ($3 == "t" || $3 == "T") {
        name = $0
        sub(/^[^ ]+ [^ ]+ [^ ]+ /, "", name)
        print $1, $2, name
    }' <<<"$nm_demangled")"
[[ -n "$symbols" ]] || fail "no function symbols in $elf"
# Start addresses of the non-returning functions, for the cold-path proof.
panic_addresses="$(while read -r address _ name; do
    if [[ "$name" =~ $panic_pattern ]]; then printf '%s,' "$address"; fi
done <<<"$symbols")"
# Every defined symbol address, sorted; bounds jump tables.
all_addresses="$(awk '{ print $1 }' <<<"$nm_demangled")"

load_allowlist <"$allowlist"

# Prints "start size name" of the function symbol containing ADDRESS (hex).
symbol_at() {
    awk -v want="$1" "$awk_hex"'
        BEGIN { target = hex("0x" want) }
        {
            start = hex("0x" $1)
            if (start <= target && target < start + hex("0x" $2)) { print; exit }
        }' <<<"$symbols"
}

# Prints the address (hex) of the first symbol whose mangled name matches.
symbol_address() {
    awk -v s="$1" '$NF ~ s { print $1; exit }' <<<"$mangled"
}

# Prints the 32-bit little-endian words at ADDRESS (hex), COUNT of them, one
# per line in hex, read from the ELF section that holds them.
read_words() {
    local address=$((16#$1)) count="$2" offset
    offset="$(awk -v a="$address" "$awk_hex"'
        $1 ~ /^\[/ {
            line = $0
            sub(/^ *\[ *[0-9]+\] */, "", line)
            split(line, f, " +")
            if (f[2] != "PROGBITS") next
            base = hex("0x" f[3]); off = hex("0x" f[4]); size = hex("0x" f[5])
            if (a >= base && a < base + size) { print off + (a - base); exit }
        }' <<<"$sections")"
    [[ -n "$offset" ]] || fail "no file data at 0x$1"
    od -An -v -tx4 --endian=little -j "$offset" -N "$((count * 4))" "$elf" | tr -s ' ' '\n' | sed '/^$/d'
}

declare -A visited=()
violations=()
allowed_hits=()
validated=()

queue=()
for root in "${roots[@]}"; do
    address="$(symbol_address "$root")"
    [[ -n "$address" ]] || fail "missing ISR symbol: $root"
    if in_ram "$address"; then
        queue+=("$address")
    else
        violations+=("root $root is linked in flash at 0x$address")
    fi
done

vtable="$(symbol_address "$waker_vtable")"
[[ -n "$vtable" ]] || fail "missing data symbol: $waker_vtable"
in_ram "$vtable" || violations+=("waker vtable $waker_vtable is linked in flash at 0x$vtable")
wake_entry="$(read_words "$(printf '%x' $((16#$vtable + waker_wake_slot)))" 1)"
wake_address="$(symbol_address "$waker_wake")"
((16#$wake_entry == 16#$wake_address)) ||
    fail "waker vtable slot $waker_wake_slot holds 0x$wake_entry, not the waker wake function at 0x$wake_address"
dispatch_address="$(symbol_address "$dispatch_table")"
[[ -n "$dispatch_address" ]] || fail "missing data symbol: $dispatch_table"
in_ram "$dispatch_address" || fail "interrupt table is not in internal RAM: 0x$dispatch_address"

while ((${#queue[@]} > 0)); do
    address="${queue[0]}"
    queue=("${queue[@]:1}")
    [[ -n "${visited[$address]:-}" ]] && continue
    visited[$address]=1

    entry="$(symbol_at "$address")"
    [[ -n "$entry" ]] || fail "no function symbol contains RAM address 0x$address"
    read -r start size name <<<"$entry"
    stop="$(printf '%x' $((16#$start + 16#$size)))"

    disassembly="$("$objdump" -d --no-show-raw-insn \
        --start-address="0x$start" --stop-address="0x$stop" "$elf")" ||
        fail "llvm-objdump failed on $name"
    if ((16#$size > 0)) && ! grep -q '^ *[0-9a-f]\+:' <<<"$disassembly"; then
        fail "llvm-objdump produced no instructions for $name (0x$start, $((16#$size)) bytes)"
    fi
    references="$(analyse "$start" "$stop" "$panic_addresses" <<<"$disassembly")" ||
        fail "analysis failed on $name"

    while read -r kind first second third fourth; do
        [[ -n "$kind" ]] || continue
        case "$kind" in
            call)
                target="$first" cold="$second"
                if in_rom "$target"; then
                    continue
                fi
                callee="$(symbol_at "$target")"
                callee_name="${callee#* * }"
                [[ -n "$callee" ]] || callee_name="<no symbol>"
                if in_ram "$target"; then
                    [[ -n "$callee" ]] || fail "no function symbol contains RAM address 0x$target (called from $name)"
                    queue+=("${callee%% *}")
                else
                    case "$(flash_call_verdict "$callee_name" "$name" "$cold")" in
                        allowed) allowed_hits+=("$callee_name (called from $name)") ;;
                        hot) violations+=("0x$target $callee_name (called from $name on a path that does not end in a panic)") ;;
                        *) violations+=("0x$target $callee_name (called from $name)") ;;
                    esac
                fi
                ;;
            arg)
                data_address="$first" target="$second" cold="$third"
                callee="$(symbol_at "$target")"
                callee_name="${callee#* * }"
                if flash_arg_allowed "$callee_name" "$name" "$cold" "$target"; then
                    continue
                fi
                violations+=("flash data at 0x$data_address (passed by $name to ${callee_name:-0x$target})")
                ;;
            data)
                violations+=("flash data at 0x$first (read by $name)")
                ;;
            jumptable)
                pc="$first" table="$second"
                if ! in_ram "$table"; then
                    violations+=("jump table at 0x$table outside RAM (used by $name at 0x$pc)")
                    continue
                fi
                end="$(awk -v t="$((16#$table))" "$awk_hex"'
                    hex("0x" $1) > t { print hex("0x" $1); exit }' <<<"$all_addresses")"
                [[ -n "$end" ]] || fail "cannot bound the jump table at 0x$table"
                entries=$(((end - 16#$table) / 4))
                ((entries > 0 && entries <= 1024)) || fail "jump table at 0x$table has $entries entries"
                # Read into a variable first: a failure inside a process
                # substitution would be lost and leave the table unchecked.
                words="$(read_words "$table" "$entries")" ||
                    fail "cannot read the jump table at 0x$table"
                (($(wc -w <<<"$words") == entries)) ||
                    fail "jump table at 0x$table: read fewer than $entries entries"
                while read -r word; do
                    if ((16#$word < 16#$start || 16#$word >= 16#$stop)); then
                        violations+=("jump table at 0x$table sends $name to 0x$word, outside the function")
                    fi
                done <<<"$words"
                validated+=("jump table 0x$table: $entries entries inside $name")
                ;;
            tablecall)
                pc="$first" table="$second"
                if [[ "$name" == "$dispatch_caller" ]] && ((16#$table == 16#$dispatch_address)); then
                    validated+=("$name: handler dispatch through $dispatch_table at 0x$pc")
                else
                    violations+=("call through the table at 0x$table in $name at 0x$pc")
                fi
                ;;
            vcall)
                pc="$first" slot="$second"
                caller_ok=0
                for waker_caller in "${waker_callers[@]}"; do
                    [[ "$name" == *"$waker_caller"* ]] && caller_ok=1
                done
                if ((caller_ok && slot == waker_wake_slot)); then
                    validated+=("waker wake at 0x$pc in ${name:0:120}")
                else
                    violations+=("call through slot $slot of an unknown vtable in $name at 0x$pc")
                fi
                ;;
            indirect)
                violations+=("unresolved register jump in $name at 0x$first: $second $third")
                ;;
            *)
                fail "unexpected analyser output: $kind"
                ;;
        esac
    done <<<"$references"
done

printf 'Scanned %d RAM functions reachable from the DCC interrupt handlers.\n' "${#visited[@]}"
printf 'Register jumps validated: %d.\n' "$(printf '%s\n' "${validated[@]}" | sort -u | wc -l)"
if [[ -n "${ISR_RAM_VERBOSE:-}" ]]; then
    printf '%s\n' "${validated[@]}" | sort -u | sed 's/^/  /'
fi
if ((${#allowed_hits[@]} > 0)); then
    if [[ -n "${ISR_RAM_VERBOSE:-}" ]]; then
        printf 'Allowlisted panic-path calls:\n'
        printf '%s\n' "${allowed_hits[@]}" | sort -u | sed 's/^/  /'
    else
        printf 'Allowlisted panic-path calls: %d (ISR_RAM_VERBOSE=1 lists them).\n' \
            "$(printf '%s\n' "${allowed_hits[@]}" | sort -u | wc -l)"
    fi
fi
if ((${#violations[@]} > 0)); then
    {
        printf 'Flash code or data reached from the DCC interrupt handlers:\n'
        printf '%s\n' "${violations[@]}" | sort -u | sed 's/^/  /'
        printf 'Move the callee to RAM (ld/isr_ram.x) or, for a panic path, add it to %s.\n' "$allowlist"
    } >&2
    exit 1
fi

printf 'RMT and cutout ISR paths are linked in internal RAM.\n'
