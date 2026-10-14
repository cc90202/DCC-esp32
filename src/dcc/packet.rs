//! DCC packet types and byte encoding.
//!
//! Implements NMRA S-9.2 compliant DCC packet structures and binary encoding.
//!
//! Address, speed, service-mode CV, POM CV, and binary-state address fields use
//! validated newtypes so invalid ranges are rejected before packet construction.
//!
//! ```ignore
//! use crate::dcc::{DccAddress, DccPacket, Direction};
//!
//! let addr = DccAddress::new_short(42).expect("42 is a valid short address");
//! let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
//! let bytes = packet.to_bytes();
//! assert!(!bytes.is_empty());
//! ```
//!
//! # NMRA compliance
//!
//! - Short addresses: 1-127 (0 is broadcast only)
//! - Long addresses: 128-10239 (11-bit field)
//! - Speed28: 0-29 (0=stop, 1=e-stop, 2-29=speed)
//! - Speed128: 0-126 (0=stop, 1-126=speed; wire value 1 reserved for e-stop)
//! - Service Mode CV range: 1-256 (Direct Mode addressing)
//! - POM CV range: 1-1024 (Long Form CV access)

mod address;
#[cfg(any(test, target_arch = "riscv32"))]
mod packed;
mod values;

use heapless::Vec;

use crate::dcc::function_groups::FunctionGroupKind;
pub use crate::dcc::speed28::NmraSpeed28;
use crate::dcc::speed28::encode_nmra_instruction_speed_bits;

pub(crate) use address::HighFragment;
pub use address::{DccAddress, DccAddressKind};
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) use packed::PackedAddressFlags;
#[cfg(test)]
pub(crate) use packed::PackedDccAddress;
pub use values::{
    BinaryStateAddress, LogonGroup, LogonSessionId, NmraSpeed128, PomCv, ServiceModeCv,
};

/// Locomotive direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum Direction {
    Forward,
    Reverse,
}

/// DCC packet types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DccPacket {
    /// Idle packet - keeps DCC signal active
    Idle,
    /// Reset packet - resets all decoders
    Reset,
    /// 28-speed step format with direction
    /// Per NMRA S-9.2, bit C is speed LSB (interleaved), NOT headlight.
    /// FL (headlight) travels in function group G1.
    Speed28 {
        address: DccAddress,
        direction: Direction,
        speed: NmraSpeed28,
    },
    /// 128-speed step format (advanced operations)
    Speed128 {
        address: DccAddress,
        direction: Direction,
        speed: NmraSpeed128,
    },
    /// Function group packet: the decoder's functions of one NMRA group.
    ///
    /// `bits` holds the group's data bits as laid out on the wire (see
    /// [`FunctionGroupKind`]); bits outside the group's data mask are
    /// ignored when encoding. Build it with [`DccPacket::function_group`].
    FunctionGroup {
        address: DccAddress,
        kind: FunctionGroupKind,
        bits: u8,
    },
    /// Emergency stop for a specific decoder
    /// Encoding: address + 01DC0001 where C=0 (NMRA S-9.2 e-stop) + checksum
    EmergencyStop {
        address: DccAddress,
        direction: Direction,
    },
    /// Broadcast stop: stops all decoders' motors but preserves state
    /// Different from Reset: BroadcastStop only stops motors, Reset resets all decoder state
    /// Encoding: 0x00 (broadcast) + 01100001 (fwd e-stop, C=0) + checksum
    BroadcastStop,
    /// Broadcast feature expansion - binary state short packet.
    ///
    /// RailCom/DCC-A track search uses binary state address 2 with state=false
    /// to invite app:search feedback during the following cutout.
    BroadcastBinaryStateShort {
        bin_addr: BinaryStateAddress,
        state: bool,
    },
    /// RCN-218 automatic logon enable packet addressed to 254.
    ///
    /// `group` uses the RCN-218 two-bit value: 0=all, 1=loco, 2=accessory, 3=now.
    LogonEnable {
        group: LogonGroup,
        command_station_id: u16,
        session_id: LogonSessionId,
    },
    /// RCN-218 automatic logon select packet addressed to 254.
    LogonSelect {
        manufacturer_id: u16,
        decoder_id: u32,
        subcommand: u8,
    },
    /// Service Mode: Verify Byte (instruction 0b0111_01AA)
    /// Tests if CV equals a specific value (decoder ACKs if match)
    /// CV range: 1-256 (Direct Mode, single-byte addressing)
    ServiceModeVerifyByte { cv: ServiceModeCv, value: u8 },
    /// Service Mode: Write Byte (instruction 0b0111_11AA)
    /// Sets CV to a specific value (decoder ACKs on success)
    /// CV range: 1-256 (Direct Mode, single-byte addressing)
    ServiceModeWriteByte { cv: ServiceModeCv, value: u8 },
    /// Programming on the Main: Write Byte (Long Form CV Access).
    PomWriteByte {
        address: DccAddress,
        cv: PomCv,
        value: u8,
    },
    /// Programming on the Main: Read Byte via Verify Byte + RailCom feedback.
    PomReadByte { address: DccAddress, cv: PomCv },
}

/// Maximum number of bytes in an encoded DCC packet, including its checksum.
///
/// RCN-218 `LogonSelect` is the longest packet currently supported.
pub const DCC_MAX_PACKET_BYTES: usize = 10;

/// Builder for DCC packet byte sequences with incremental XOR checksum.
struct PacketBytes {
    bytes: Vec<u8, DCC_MAX_PACKET_BYTES>,
    checksum: u8,
}

impl PacketBytes {
    fn new() -> Self {
        Self {
            bytes: Vec::new(),
            checksum: 0,
        }
    }

    fn push(&mut self, byte: u8) {
        self.bytes
            .push(byte)
            .expect("packet payload must fit DCC_MAX_PACKET_BYTES");
        self.checksum ^= byte;
    }

    fn push_address(&mut self, address: DccAddress) {
        let address_bytes = address.to_bytes();
        for byte in address_bytes {
            self.push(byte);
        }
    }

    /// Appends the Dallas/Maxim CRC8 of the bytes pushed so far.
    ///
    /// Used by RCN-218 automatic-logon packets (`LogonSelect`), which carry a
    /// CRC8 byte ahead of the usual trailing XOR checksum.
    fn push_crc8(&mut self) {
        let crc = crate::logon::crc8_dallas_maxim(&self.bytes);
        self.push(crc);
    }

    fn finalize(mut self) -> Vec<u8, DCC_MAX_PACKET_BYTES> {
        self.bytes
            .push(self.checksum)
            .expect("packet checksum must fit DCC_MAX_PACKET_BYTES");
        self.bytes
    }
}

impl DccPacket {
    /// Address whose decoder may answer in the following RailCom cutout.
    #[must_use]
    pub const fn railcom_target_address(&self) -> Option<DccAddress> {
        match *self {
            DccPacket::Speed28 { address, .. }
            | DccPacket::Speed128 { address, .. }
            | DccPacket::FunctionGroup { address, .. }
            | DccPacket::EmergencyStop { address, .. }
            | DccPacket::PomWriteByte { address, .. }
            | DccPacket::PomReadByte { address, .. } => Some(address),
            DccPacket::Idle
            | DccPacket::Reset
            | DccPacket::BroadcastStop
            | DccPacket::BroadcastBinaryStateShort { .. }
            | DccPacket::LogonEnable { .. }
            | DccPacket::LogonSelect { .. }
            | DccPacket::ServiceModeVerifyByte { .. }
            | DccPacket::ServiceModeWriteByte { .. } => None,
        }
    }

    #[inline]
    fn bool_mask(value: bool, mask: u8) -> u8 {
        if value { mask } else { 0 }
    }

    #[inline]
    fn direction_mask(direction: Direction, mask: u8) -> u8 {
        Self::bool_mask(direction == Direction::Forward, mask)
    }

    fn encode_speed28_instruction(direction: Direction, speed: NmraSpeed28) -> u8 {
        // Instruction format: 01DCSSSS (NMRA S-9.2 §2.3.2.3)
        0b0100_0000
            | Self::direction_mask(direction, 0b0010_0000)
            | encode_nmra_instruction_speed_bits(speed)
    }

    fn encode_speed128_data(direction: Direction, speed: NmraSpeed128) -> u8 {
        let raw = speed.value();
        let wire_speed = if raw == 0 { 0 } else { raw + 1 };
        Self::direction_mask(direction, 0x80) | (wire_speed & 0x7F)
    }

    fn encode_emergency_stop_instruction(direction: Direction) -> u8 {
        // E-stop per NMRA S-9.2: 01DC0001 with C=0, SSSS=0001
        0b0100_0001 | Self::direction_mask(direction, 0b0010_0000)
    }

    /// Encodes a validated 1-based CV number into the two on-wire bytes used by
    /// Direct Mode (S-9.2.2) and POM (S-9.2.1) packets.
    fn encode_cv_addr(cv: u16, op_prefix: u8) -> (u8, u8) {
        let wire_cv = cv - 1;
        let cv_high = op_prefix | (((wire_cv >> 8) & 0b11) as u8);
        let cv_low = (wire_cv & 0xFF) as u8;
        (cv_high, cv_low)
    }

    /// Helper function for tests - create an idle packet.
    #[cfg(test)]
    pub fn idle() -> Self {
        DccPacket::Idle
    }

    /// Create a function group packet for `address`.
    ///
    /// `bits` are the group's data bits as laid out on the wire; bits outside
    /// `kind`'s data mask are dropped so equal packets compare equal.
    #[must_use]
    pub const fn function_group(address: DccAddress, kind: FunctionGroupKind, bits: u8) -> Self {
        DccPacket::FunctionGroup {
            address,
            kind,
            bits: bits & kind.data_mask(),
        }
    }

    /// Create a 28-step speed packet.
    ///
    /// Returns `None` if speed > 29 (NMRA S-9.2 §2.3.2.3).
    pub fn speed_28step(address: DccAddress, speed: u8, direction: Direction) -> Option<Self> {
        Some(DccPacket::Speed28 {
            address,
            direction,
            speed: NmraSpeed28::new(speed)?,
        })
    }

    /// Create a 128-step speed packet.
    ///
    /// Returns `None` if speed > 126 (NMRA S-9.2.1).
    pub fn speed_128step(address: DccAddress, speed: u8, direction: Direction) -> Option<Self> {
        Some(DccPacket::Speed128 {
            address,
            direction,
            speed: NmraSpeed128::new(speed)?,
        })
    }

    /// Encode packet to bytes with checksum.
    ///
    /// Returns a vector containing the complete DCC packet payload bytes
    /// followed by the error detection checksum byte. Every variant encodes:
    /// speed, address and CV ranges are enforced by the field newtypes.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8, DCC_MAX_PACKET_BYTES> {
        let mut packet = PacketBytes::new();

        match *self {
            DccPacket::Idle => {
                // Idle packet: {preamble} 0 11111111 0 00000000 0 EEEEEEEE 1
                packet.push(0xFF);
                packet.push(0x00);
            }
            DccPacket::Reset => {
                // Reset packet: {preamble} 0 00000000 0 00000000 0 EEEEEEEE 1
                packet.push(0x00);
                packet.push(0x00);
            }
            DccPacket::Speed28 {
                address,
                direction,
                speed,
            } => {
                packet.push_address(address);
                packet.push(Self::encode_speed28_instruction(direction, speed));
            }
            DccPacket::Speed128 {
                address,
                direction,
                speed,
            } => {
                // Advanced operations instruction: 0x3F
                packet.push_address(address);
                packet.push(0x3F);
                packet.push(Self::encode_speed128_data(direction, speed));
            }
            DccPacket::FunctionGroup {
                address,
                kind,
                bits,
            } => {
                let data = bits & kind.data_mask();
                packet.push_address(address);
                match kind {
                    FunctionGroupKind::G1 => packet.push(0b1000_0000 | data),
                    FunctionGroupKind::G2a => packet.push(0b1011_0000 | data),
                    FunctionGroupKind::G2b => packet.push(0b1010_0000 | data),
                    FunctionGroupKind::G3 => {
                        packet.push(0xDE);
                        packet.push(data);
                    }
                    FunctionGroupKind::G4 => {
                        packet.push(0xDF);
                        packet.push(data);
                    }
                }
            }
            DccPacket::EmergencyStop { address, direction } => {
                packet.push_address(address);
                packet.push(Self::encode_emergency_stop_instruction(direction));
            }
            DccPacket::BroadcastStop => {
                // Broadcast address 0x00 + forward e-stop
                packet.push(0x00);
                packet.push(Self::encode_emergency_stop_instruction(Direction::Forward));
            }
            DccPacket::BroadcastBinaryStateShort { bin_addr, state } => {
                packet.push(0x00);
                packet.push(0xDD);
                packet.push(Self::bool_mask(state, 0x80) | bin_addr.value());
            }
            DccPacket::LogonEnable {
                group,
                command_station_id,
                session_id,
            } => {
                packet.push(254);
                packet.push(0xFC | group.value());
                packet.push((command_station_id >> 8) as u8);
                packet.push((command_station_id & 0xFF) as u8);
                packet.push(session_id.value());
            }
            DccPacket::LogonSelect {
                manufacturer_id,
                decoder_id,
                subcommand,
            } => {
                let logon_instruction = 0xD0 | ((manufacturer_id >> 8) as u8 & 0x0F);
                packet.push(254);
                packet.push(logon_instruction);
                packet.push((manufacturer_id & 0xFF) as u8);
                packet.push(((decoder_id >> 24) & 0xFF) as u8);
                packet.push(((decoder_id >> 16) & 0xFF) as u8);
                packet.push(((decoder_id >> 8) & 0xFF) as u8);
                packet.push((decoder_id & 0xFF) as u8);
                packet.push(subcommand);
                packet.push_crc8();
            }
            DccPacket::ServiceModeVerifyByte { cv, value } => {
                // Service Mode Verify Byte per NMRA S-9.2.2
                // Packet format: [preamble] 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 [checksum] 1
                // Where:
                //   0111CCAA AAAAAAAA = CV address (Direct Mode, CC=00 for CV 1-256)
                //   DDDDDDDD = value to verify (0-255)
                //
                // For CV 1-256: CC=00 (bits 9-8), AA AAAAAAAA = bits 7-0
                // Instruction byte: 0b0111_01AA (bits 3-2 = 01 for Verify)
                let (cv_high, cv_low) = Self::encode_cv_addr(cv.value(), 0b0111_0100);
                packet.push(cv_high);
                packet.push(cv_low);
                packet.push(value);
            }
            DccPacket::ServiceModeWriteByte { cv, value } => {
                // Service Mode Write Byte per NMRA S-9.2.2
                // Packet format: same as Verify, but instruction byte bits 3-2 = 11 (Write)
                // Instruction byte: 0b0111_11AA (bits 3-2 = 11 for Write)
                let (cv_high, cv_low) = Self::encode_cv_addr(cv.value(), 0b0111_1100);
                packet.push(cv_high);
                packet.push(cv_low);
                packet.push(value);
            }
            DccPacket::PomWriteByte { address, cv, value } => {
                let (cv_high, cv_low) = Self::encode_cv_addr(cv.value(), 0b1110_1100);
                packet.push_address(address);
                packet.push(cv_high);
                packet.push(cv_low);
                packet.push(value);
            }
            DccPacket::PomReadByte { address, cv } => {
                let (cv_high, cv_low) = Self::encode_cv_addr(cv.value(), 0b1110_0100);
                packet.push_address(address);
                packet.push(cv_high);
                packet.push(cv_low);
                packet.push(0x00);
            }
        }

        packet.finalize()
    }
}

#[cfg(test)]
mod tests;
