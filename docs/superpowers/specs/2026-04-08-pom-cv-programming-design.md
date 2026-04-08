# POM CV Programming Design

Programming On the Main (POM) — CV read/write via RailCom feedback, without dedicated programming track.

## Context

The DCC-esp32 command station needs CV programming capability. Two approaches exist:
- **POM (Phase 1):** Software-only, uses existing RailCom infrastructure for decoder feedback
- **Service Mode (Phase 2):** Requires hardware (relay, ACK circuit, prog track)

This spec covers Phase 1: POM via RailCom.

### References

- NMRA S-9.2.1 "DCC Extended Packet Formats" (Jan 2025) — Long Form CV Access Instruction
- NMRA S-9.3.2 "Bi-Directional Communication" — RailCom datagram IDs 0 (CvData), Ack/Nack
- Z21 LAN Protocol v1.13 — `LAN_X_CV_POM_WRITE_BYTE`, `LAN_X_CV_POM_READ_BYTE`, `LAN_X_CV_RESULT`, `LAN_X_CV_NACK`
- DCC-EX CommandStation-EX — reference implementation for POM packet construction
- `docs/specs/cv-programming-strategy.md` — project strategy document

### Existing Infrastructure

| Component | Status | Location |
|-----------|--------|----------|
| RailCom parser with `CvData(u8)`, `Ack`, `Nack` | Done | `src/railcom/parser.rs` |
| DCC packet encoder (generic via `to_bytes()`) | Done | `src/dcc/encoder.rs` |
| Scheduler with priority queue | Done | `src/dcc/scheduler.rs` |
| Z21 UDP protocol layer | Done | `src/net/z21_proto.rs` |
| `DccPacket` enum with address encoding | Done | `src/dcc/packet.rs` |

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Scope | Write + Read | Full POM capability from day one |
| Request/response correlation | `Channel<CvResponse, 1>` | Embassy-idiomatic, one POM op at a time, easy to test |
| CV range | 1-1024 | Long Form uses 10 bits natively; limiting to 256 would be artificial |
| Z21 response | Full (`CV_RESULT` / `CV_NACK`) | Without response, Z21 app hangs waiting |
| Retry policy | 3 attempts, 100ms timeout each | Typical Z21 central behavior; simple, predictable |
| Architecture | `PomService` module + scheduler one-shot queue | Clean separation; scheduler grows minimally |

---

## 1. Packet Types and Encoding

### New `DccPacket` Variants

```rust
DccPacket::PomWriteByte { address: DccAddress, cv: u16, value: u8 }
DccPacket::PomReadByte  { address: DccAddress, cv: u16 }
```

### NMRA S-9.2.1 Long Form CV Access Instruction

Wire format appended after address bytes:

```
[1110CCVV] [VVVVVVVV] [DDDDDDDD]
```

| Field | Bits | Value |
|-------|------|-------|
| `1110` | 7-4 | Fixed prefix (POM uses `1110`, Service Mode uses `0111`) |
| `CC` | 3-2 | `11` = Write Byte, `01` = Verify Byte (used for Read) |
| `VV VVVVVVVV` | 1-0 + 7-0 | **cv - 1** (10-bit, 0-1023 maps to CV 1-1024) |
| `DDDDDDDD` | 7-0 | Value to write (Write) or `0x00` (Read) |

Concrete instruction bytes:
- **Write Byte:** `0xEC | cv_high` where `cv_high = ((cv - 1) >> 8) & 0x03`
- **Read Byte:** `0xE4 | cv_high` (Verify Byte with data=0; actual value returns via RailCom)

### Complete Packet Examples

**POM Write, short address 3, CV29=6:**

```
[0x03] [0xEC] [0x1C] [0x06] [0xF5]
 addr    inst   cv_lo  value   XOR
```

5 bytes. cv-1=28=0x001C, cv_high=0, cv_low=0x1C. XOR: 0x03^0xEC^0x1C^0x06=0xF5.

**POM Read, long address 1000, CV1:**

```
[0xC3] [0xE8] [0xE4] [0x00] [0x00] [0xCF]
 ah     al     inst   cv_lo  data    XOR
```

6 bytes (maximum for `Vec<u8, 6>`). cv-1=0, data=0x00. XOR: 0xC3^0xE8^0xE4^0x00^0x00=0xCF.

### RailCom Response (already implemented in parser)

| DCC Operation | Decoder Ch2 Response | Parser Type |
|---------------|---------------------|-------------|
| POM Write (CC=11) | ACK | `RailcomChannel2Item::Ack` |
| POM Read (CC=01) | CV value | `RailcomDatagram::CvData(u8)` |
| Failure | NACK | `RailcomChannel2Item::Nack` |

### Validation

`to_bytes()` returns `Err(PacketEncodeError::InvalidCvAddress)` for cv outside 1-1024. Reuses the existing error variant.

### Note: Existing Service Mode cv-1 Bug

`encode_service_mode_cv()` at `packet.rs:453` does not subtract 1 from cv before encoding. Per NMRA S-9.2.2 and DCC-EX, the wire address should be cv-1. Out of scope for this spec — tracked separately.

---

## 2. One-Shot Queue in Scheduler

### New `SchedulerCommand`

```rust
SchedulerCommand::SendOneShot(DccPacket)
```

Generic command, not POM-specific. Reusable for any fire-and-forget packet.

### Queue in `SlotManager`

```rust
one_shot_queue: heapless::Deque<DccPacket, 4>
```

- Capacity 4: one POM operation at a time, room for 3 retries
- FIFO ordering (send order matters)

### Priority in `build_next_packet`

One-shot packets get highest priority:

```
Priority 0: One-shot queue (POM, single commands)
Priority 1: Emergency stops
Priority 2: Dirty speed
Priority 3: Dirty functions
Priority 4: Cyclic refresh
```

After transmission, the packet is removed from the queue (no refresh cycle).

### Pause Interaction

While scheduler is paused (e.g., during Service Mode), one-shot packets remain queued and are sent after resume.

---

## 3. PomService Module

New file: `src/dcc/pom.rs`

### Struct

```rust
pub struct PomService {
    scheduler_tx: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    cv_result_rx: Receiver<'static, CriticalSectionRawMutex, CvResponse, 1>,
}
```

### Types

```rust
pub enum CvResponse {
    Value(u8),  // CvData from decoder (POM Read)
    Ack,        // Write confirmed
    Nack,       // Decoder rejected
}

pub enum PomError {
    NoResponse,        // No response after 3 attempts
    Nack,              // Decoder rejected the operation
    InvalidCvAddress,  // CV outside 1-1024
}
```

### Public API

```rust
impl PomService {
    pub async fn read_cv(&self, address: DccAddress, cv: u16) -> Result<u8, PomError>;
    pub async fn write_cv(&self, address: DccAddress, cv: u16, value: u8) -> Result<(), PomError>;
}
```

### Read Flow

```
1. Validate cv (1-1024), return Err(InvalidCvAddress) if out of range
2. For attempt in 1..=3:
   a. Build DccPacket::PomReadByte { address, cv }
   b. Send SchedulerCommand::SendOneShot(packet) via scheduler_tx
   c. Await cv_result_rx with 100ms timeout
   d. Match:
      - CvResponse::Value(v) -> return Ok(v)
      - CvResponse::Nack -> return Err(Nack)
      - Timeout -> continue to next attempt
3. return Err(NoResponse)
```

### Write Flow

```
1. Validate cv (1-1024)
2. For attempt in 1..=3:
   a. Build DccPacket::PomWriteByte { address, cv, value }
   b. Send SchedulerCommand::SendOneShot(packet) via scheduler_tx
   c. Await cv_result_rx with 100ms timeout
   d. Match:
      - CvResponse::Ack -> return Ok(())
      - CvResponse::Nack -> return Err(Nack)
      - Timeout -> continue to next attempt
3. return Err(NoResponse)
```

### Design Properties

- **No internal state.** Each call is self-contained.
- **Not a separate Embassy task.** Called inline from the UDP handler (the Z21 app waits for the response anyway).
- **Channel capacity 1** naturally serializes operations — no locks or flags needed.

---

## 4. Z21 Protocol

### New Commands

```rust
Z21Command::CvPomWriteByte { address: DccAddress, cv: u16, value: u8 }
Z21Command::CvPomReadByte  { address: DccAddress, cv: u16 }
```

### Incoming Frame — `LAN_X_CV_POM_WRITE_BYTE` / `READ_BYTE`

```
[0x0C 0x00] [0x40 0x00] [0xE6] [0x30] [DB1] [DB2] [DB3] [DB4] [DB5] [XOR]
 DataLen=12   Header      XHdr   DB0    addr   addr  CV    CV   value  check
```

Parsing in `parse_xbus()` on X-Header `0xE6`:

| Field | Decoding |
|-------|----------|
| DB0 | `0x30` = loco POM |
| Address | `((DB1 & 0x3F) << 8) | DB2` (same as SetLocoDrive) |
| Operation | `DB3 & 0xFC`: `0xEC` = Write, `0xE4` = Read |
| CV | `(((DB3 & 0x03) << 8) | DB4) + 1` (Z21 is 0-based, internal is 1-based) |
| Value | DB5 (Write only) |

### Response Frames

**`LAN_X_CV_RESULT`** (success):

```
[0x0A 0x00] [0x40 0x00] [0x64] [0x14] [CV_MSB] [CV_LSB] [Value] [XOR]
 DataLen=10   Header      XHdr   DB0    (cv-1)h  (cv-1)l  data    check
```

CV on wire is 0-based (cv - 1). XOR covers bytes 4-8.

**`LAN_X_CV_NACK`** (failure):

```
[0x07 0x00] [0x40 0x00] [0x61] [0x13] [0x72]
 DataLen=7    Header      XHdr   DB0    XOR
```

Fixed frame. XOR = 0x61 ^ 0x13 = 0x72.

### New Builder Functions

```rust
fn build_cv_result(cv: u16, value: u8) -> heapless::Vec<u8, 10>
fn build_cv_nack() -> heapless::Vec<u8, 7>
```

---

## 5. Wiring and Data Flow

### End-to-End Flow

```
Z21 App (UDP)
  |
  v
udp_control: parse CvPomWriteByte/ReadByte
  |
  v
PomService::write_cv() / read_cv()        <-- await on cv_result_rx
  |                                              ^
  v                                              |
scheduler_tx.send(SendOneShot(packet))     cv_result_tx.send(CvResponse)
  |                                              |
  v                                              |
SlotManager: one_shot_queue -> build_next_packet |
  |                                              |
  v                                              |
DCC Engine: encode + RMT -> GPIO2 -> track       |
  |                                              |
  v                                              |
Decoder responds via RailCom in cutout window    |
  |                                              |
  v                                              |
UART RX -> RailCom parser -> CvData/Ack ---------+
```

### Channels

| Channel | Type | Cap | From | To |
|---------|------|-----|------|----|
| `scheduler_cmd` (existing) | `SchedulerCommand` | 32 | PomService | SlotManager |
| `cv_result` (new) | `CvResponse` | 1 | RailCom parser | PomService |

One new channel. One new `StaticCell` in `main.rs`.

### Static Allocation

```rust
static CV_RESULT_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, CvResponse, 1>> =
    StaticCell::new();
```

Initialized in `main.rs`. Sender goes to the RailCom parser task, Receiver goes to `PomService`.

### PomService Ownership

`PomService` is not a separate Embassy task. It is owned by `udp_control_task` and called inline:

```rust
// In udp_control_task
match command {
    Z21Command::CvPomReadByte { address, cv } => {
        match pom_service.read_cv(address, cv).await {
            Ok(value) => send(build_cv_result(cv, value)),
            Err(_) => send(build_cv_nack()),
        }
    }
    // ...
}
```

### RailCom Parser Modification

The parser receives an `Option<Sender<CvResponse, 1>>`. On Channel 2 decode:
- `RailcomDatagram::CvData(v)` -> `try_send(CvResponse::Value(v))`
- `RailcomChannel2Item::Ack` -> `try_send(CvResponse::Ack)`
- `RailcomChannel2Item::Nack` -> `try_send(CvResponse::Nack)`

Uses `try_send` (non-blocking). If no POM operation is in progress, the channel is full or has no receiver waiting, and the datagram is silently dropped. This is correct: unsolicited CvData is not actionable.

### No Race Conditions

The flow is inherently sequential: PomService sends a packet, then awaits the response channel. The decoder responds in the immediately following cutout window. Channel capacity 1 prevents stale responses from accumulating.

---

## 6. Testing

### Host Unit Tests (`cargo test-host`)

**`packet.rs` — POM encoding:**
- `PomWriteByte` short address: verify exact bytes (addr=3, CV29=6 -> `[0x03, 0xEC, 0x1C, 0x06, 0xF5]`)
- `PomWriteByte` long address: verify 6-byte output (addr=1000, CV1=5)
- `PomReadByte`: verify CC=01 (`0xE4`) and data=0x00
- CV boundary: CV 1, CV 1024 accepted; CV 0, CV 1025 rejected
- `DccValidator::validate_full()` on POM packets (preamble + pulse timing compliance)

**`z21_proto.rs` — Z21 parsing and encoding:**
- Parse `LAN_X_CV_POM_WRITE_BYTE` frame -> `Z21Command::CvPomWriteByte`
- Parse `LAN_X_CV_POM_READ_BYTE` frame -> `Z21Command::CvPomReadByte`
- CV 0-based in Z21 frame -> 1-based internal (cv + 1)
- `build_cv_result(cv, value)` -> verify exact bytes against spec
- `build_cv_nack()` -> verify fixed frame `[0x07, 0x00, 0x40, 0x00, 0x61, 0x13, 0x72]`
- XOR correctness on all frames

**`scheduler.rs` — one-shot queue:**
- `SendOneShot` enqueues; `build_next_packet` returns it at highest priority
- Packet removed after send (no refresh)
- One-shot not sent while paused; sent after resume

### PomService Testing Strategy

`PomService` methods are async and depend on `embassy_sync::Channel`. Testing the full async flow on host requires an executor.

Pragmatic approach: test each component synchronously (packet encoding, Z21 parsing, scheduler queue). The PomService orchestration is a simple retry loop over these components. End-to-end validation on real hardware with decoder + Z21 app.

---

## Files Modified

| File | Change |
|------|--------|
| `src/dcc/packet.rs` | Add `PomWriteByte`, `PomReadByte` variants + encoding + tests |
| `src/dcc/pom.rs` | New module: `PomService`, `CvResponse`, `PomError` |
| `src/dcc/mod.rs` | Add `pom` module, re-export public types |
| `src/dcc/scheduler.rs` | Add `SendOneShot` command, `one_shot_queue`, priority change + tests |
| `src/net/z21_proto.rs` | Parse POM commands, build CV_RESULT/CV_NACK + tests |
| `src/net/udp_control.rs` | Dispatch POM commands to `PomService` |
| `src/railcom/parser.rs` | Add `Option<Sender<CvResponse>>`, publish CvData/Ack/Nack |
| `src/bin/main.rs` | Allocate `CV_RESULT_CHANNEL`, wire PomService |
