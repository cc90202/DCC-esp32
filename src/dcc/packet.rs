//! DCC Packet Types and Encoding
//!
//! Implements NMRA S-9.2 compliant DCC packet structures and binary encoding.
//!
//! # Overview
//!
//! This module defines the complete set of DCC packet types (locomotive control,
//! functions, service mode) and provides encoding to binary format per NMRA S-9.2.
//! All addresses are validated at construction time using strong types (`DccAddress`),
//! making it impossible to create invalid packets.
//!
//! # Examples
//!
//! **Create and encode a Speed128 packet:**
//!
//! ```
//! use dcc_esp32::dcc::{DccPacket, DccAddress, Direction};
//!
//! // Create a short address (1-127)
//! let addr = DccAddress::short(42).expect("42 is valid");
//!
//! // Create a 128-step speed command: address 42, speed 75, forward
//! let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
//!
//! // Encode to binary (6 bytes max: address + data + checksum)
//! let bytes = packet.to_bytes();
//! assert!(!bytes.is_empty());
//! ```
//!
//! **Use a long address (128-10239):**
//!
//! ```
//! use dcc_esp32::dcc::{DccPacket, DccAddress, Direction};
//!
//! // Long addresses for modern decoders (multifunction)
//! let addr = DccAddress::long(1234).expect("1234 is valid");
//!
//! let packet = DccPacket::speed_128step(addr, 50, Direction::Reverse).unwrap();
//! ```
//!
//! **Control functions (headlight, horn, lights):**
//!
//! ```
//! use dcc_esp32::dcc::{DccPacket, DccAddress};
//!
//! let addr = DccAddress::short(5).unwrap();
//!
//! // Function Group 1: FL (headlight) and F1-F4
//! let packet = DccPacket::FunctionGroup1 {
//!     address: addr,
//!     fl: true,   // Headlight on
//!     f1: true,   // Horn on
//!     f2: false,
//!     f3: false,
//!     f4: false,
//! };
//! ```
//!
//! **Service Mode: Verify a CV value:**
//!
//! ```
//! use dcc_esp32::dcc::DccPacket;
//!
//! // Check if CV1 (primary address) equals 42
//! let packet = DccPacket::ServiceModeVerifyByte {
//!     cv: 1,
//!     value: 42,
//! };
//! // Decoder will send an ACK pulse if CV1 matches.
//! ```
//!
//! # Errors
//!
//! Address validation occurs at construction via `DccAddress::short()` and `DccAddress::long()`,
//! which return `Option`. Invalid CV values in Service Mode packets return `PacketEncodeError`.
//!
//! # NMRA Compliance
//!
//! - Short addresses: 1-127 (0 is broadcast only)
//! - Long addresses: 128-10239 (11-bit field)
//! - Speed28: 0-29 (0=stop, 1=e-stop, 2-29=speed)
//! - Speed128: 0-126 (0=stop, 1-126=speed; wire value 1 reserved for e-stop)
//! - CV range: 1-256 (Direct Mode addressing)

use heapless::Vec;

use crate::dcc::speed28::encode_nmra_instruction_speed_bits;

/// Packet encoding failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PacketEncodeError {
    InvalidSpeed28 { speed: u8 },
    InvalidCvAddress { cv: u16 },
}

/// DCC decoder address (opaque type enforcing NMRA address validation)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct DccAddress {
    kind: AddressKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
enum AddressKind {
    Short(u8),
    Long(u16),
}

impl DccAddress {
    /// Create a new short address (1-127)
    ///
    /// Returns `None` if address is 0 or > 127
    pub fn new_short(addr: u8) -> Option<Self> {
        if addr == 0 || addr > 127 {
            None
        } else {
            Some(DccAddress {
                kind: AddressKind::Short(addr),
            })
        }
    }

    /// Create a new long address (128-10239)
    ///
    /// Returns `None` if address is < 128 or > 10239
    pub fn new_long(addr: u16) -> Option<Self> {
        if !(128..=10239).contains(&addr) {
            None
        } else {
            Some(DccAddress {
                kind: AddressKind::Long(addr),
            })
        }
    }

    /// Alias for `new_short`
    pub fn short(addr: u8) -> Option<Self> {
        Self::new_short(addr)
    }

    /// Alias for `new_long`
    pub fn long(addr: u16) -> Option<Self> {
        Self::new_long(addr)
    }

    /// Returns `true` if this is a short address
    pub fn is_short(&self) -> bool {
        matches!(self.kind, AddressKind::Short(_))
    }

    /// Returns `true` if this is a long address
    pub fn is_long(&self) -> bool {
        matches!(self.kind, AddressKind::Long(_))
    }

    /// Returns the numeric address value
    pub fn value(&self) -> u16 {
        match self.kind {
            AddressKind::Short(addr) => addr as u16,
            AddressKind::Long(addr) => addr,
        }
    }

    /// Encode address to bytes according to NMRA S-9.2
    fn to_bytes(self) -> Vec<u8, 2> {
        let mut bytes = Vec::new();
        match self.kind {
            AddressKind::Short(addr) => {
                if bytes.push(addr).is_err() {
                    unreachable!("short address must fit in two-byte buffer");
                }
            }
            AddressKind::Long(addr) => {
                // Long address format: 11AAAAAA AAAAAAAA
                // Top 6 bits in first byte (with 0xC0 prefix)
                // Bottom 8 bits in second byte
                let high = 0xC0 | ((addr >> 8) as u8);
                let low = (addr & 0xFF) as u8;
                if bytes.push(high).is_err() {
                    unreachable!("long address high byte must fit in two-byte buffer");
                }
                if bytes.push(low).is_err() {
                    unreachable!("long address low byte must fit in two-byte buffer");
                }
            }
        }
        bytes
    }
}

/// Locomotive direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum Direction {
    Forward,
    Reverse,
}

/// Validated 28-step speed value per NMRA S-9.2 §2.3.2.3.
///
/// Range 0-29: 0=stop, 1=e-stop, 2-29=speed steps 1-28.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct NmraSpeed28(u8);

impl NmraSpeed28 {
    pub const fn new(speed: u8) -> Option<Self> {
        if speed > 29 { None } else { Some(Self(speed)) }
    }

    pub const fn value(self) -> u8 {
        self.0
    }
}

/// Validated 128-step speed value per NMRA S-9.2.1.
///
/// Range 0-126: 0=stop, 1-126=speed steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct NmraSpeed128(u8);

impl NmraSpeed128 {
    pub const fn new(speed: u8) -> Option<Self> {
        if speed > 126 { None } else { Some(Self(speed)) }
    }

    pub const fn value(self) -> u8 {
        self.0
    }
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
    /// Use FunctionGroup1 to control FL (headlight).
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
    /// Function Group One: FL (headlight), F1-F4
    /// Encoding: 100DDDDD where D4=FL, D3=F4, D2=F3, D1=F2, D0=F1
    FunctionGroup1 {
        address: DccAddress,
        fl: bool,
        f1: bool,
        f2: bool,
        f3: bool,
        f4: bool,
    },
    /// Function Group Two (F5-F8)
    /// Encoding: 1011DDDD where D3=F8, D2=F7, D1=F6, D0=F5
    FunctionGroup2A {
        address: DccAddress,
        f5: bool,
        f6: bool,
        f7: bool,
        f8: bool,
    },
    /// Function Group Two (F9-F12)
    /// Encoding: 1010DDDD where D3=F12, D2=F11, D1=F10, D0=F9
    FunctionGroup2B {
        address: DccAddress,
        f9: bool,
        f10: bool,
        f11: bool,
        f12: bool,
    },
    /// Function Group 3 (F13-F20) — Binary State Control Long Form
    /// Encoding: 0xDE DDDDDDDD where D0=F13, D7=F20
    FunctionGroup3 {
        address: DccAddress,
        f13: bool,
        f14: bool,
        f15: bool,
        f16: bool,
        f17: bool,
        f18: bool,
        f19: bool,
        f20: bool,
    },
    /// Function Group 4 (F21-F28) — Binary State Control Long Form
    /// Encoding: 0xDF DDDDDDDD where D0=F21, D7=F28
    FunctionGroup4 {
        address: DccAddress,
        f21: bool,
        f22: bool,
        f23: bool,
        f24: bool,
        f25: bool,
        f26: bool,
        f27: bool,
        f28: bool,
    },
    /// Emergency stop for a specific decoder
    /// Encoding: address + 01DC0001 where C=0 (NMRA S-9.2 e-stop) + checksum
    EmergencyStop {
        address: DccAddress,
        direction: Direction,
    },
    /// Broadcast stop — stops all decoders' motors but preserves state
    /// Different from Reset: BroadcastStop only stops motors, Reset resets all decoder state
    /// Encoding: 0x00 (broadcast) + 01100001 (fwd e-stop, C=0) + checksum
    BroadcastStop,
    /// Service Mode: Verify Byte (instruction 0b0111_01AA)
    /// Tests if CV equals a specific value (decoder ACKs if match)
    /// CV range: 1-256 (Direct Mode, single-byte addressing)
    ServiceModeVerifyByte {
        cv: u16,   // CV address 1-256
        value: u8, // Expected value 0-255
    },
    /// Service Mode: Write Byte (instruction 0b0111_11AA)
    /// Sets CV to a specific value (decoder ACKs on success)
    /// CV range: 1-256 (Direct Mode, single-byte addressing)
    ServiceModeWriteByte {
        cv: u16,   // CV address 1-256
        value: u8, // Value to write 0-255
    },
}

/// Builder for DCC packet byte sequences with incremental XOR checksum.
struct PacketBytes {
    bytes: Vec<u8, 6>,
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
        if self.bytes.push(byte).is_err() {
            unreachable!("packet payload must fit in six-byte buffer");
        }
        self.checksum ^= byte;
    }

    fn push_address(&mut self, address: DccAddress) {
        let address_bytes = address.to_bytes();
        for byte in address_bytes {
            self.push(byte);
        }
    }

    fn finalize(mut self) -> Vec<u8, 6> {
        if self.bytes.push(self.checksum).is_err() {
            unreachable!("packet checksum must fit in six-byte buffer");
        }
        self.bytes
    }
}

impl DccPacket {
    #[inline]
    fn bool_mask(value: bool, mask: u8) -> u8 {
        if value { mask } else { 0 }
    }

    fn encode_speed28_instruction(
        direction: Direction,
        speed: NmraSpeed28,
    ) -> Result<u8, PacketEncodeError> {
        // Instruction format: 01DCSSSS (NMRA S-9.2 §2.3.2.3)
        let direction_bit = if direction == Direction::Forward {
            0b0010_0000
        } else {
            0
        };

        let speed_bits = encode_nmra_instruction_speed_bits(speed.value())?;

        Ok(0b0100_0000 | direction_bit | speed_bits)
    }

    fn encode_speed128_data(direction: Direction, speed: NmraSpeed128) -> u8 {
        let raw = speed.value();
        let wire_speed = if raw == 0 { 0 } else { raw + 1 };
        let direction_bit = if direction == Direction::Forward {
            0x80
        } else {
            0
        };
        direction_bit | (wire_speed & 0x7F)
    }

    fn encode_function_group1(fl: bool, f1: bool, f2: bool, f3: bool, f4: bool) -> u8 {
        // 100DDDDD: D4=FL, D3=F4, D2=F3, D1=F2, D0=F1
        0b1000_0000
            | Self::bool_mask(fl, 0b0001_0000)
            | Self::bool_mask(f4, 0b0000_1000)
            | Self::bool_mask(f3, 0b0000_0100)
            | Self::bool_mask(f2, 0b0000_0010)
            | Self::bool_mask(f1, 0b0000_0001)
    }

    fn encode_function_group2a(f5: bool, f6: bool, f7: bool, f8: bool) -> u8 {
        // 1011DDDD: D3=F8, D2=F7, D1=F6, D0=F5
        0b1011_0000
            | Self::bool_mask(f8, 0b0000_1000)
            | Self::bool_mask(f7, 0b0000_0100)
            | Self::bool_mask(f6, 0b0000_0010)
            | Self::bool_mask(f5, 0b0000_0001)
    }

    fn encode_function_group2b(f9: bool, f10: bool, f11: bool, f12: bool) -> u8 {
        // 1010DDDD: D3=F12, D2=F11, D1=F10, D0=F9
        0b1010_0000
            | Self::bool_mask(f12, 0b0000_1000)
            | Self::bool_mask(f11, 0b0000_0100)
            | Self::bool_mask(f10, 0b0000_0010)
            | Self::bool_mask(f9, 0b0000_0001)
    }

    fn encode_function_group_data(functions: [bool; 8]) -> u8 {
        // DDDDDDDD: bit0..bit7 map to the corresponding function flags in order.
        functions
            .iter()
            .enumerate()
            .fold(0u8, |acc, (idx, enabled)| {
                acc | Self::bool_mask(*enabled, 1u8 << idx)
            })
    }

    fn encode_emergency_stop_instruction(direction: Direction) -> u8 {
        // E-stop per NMRA S-9.2: 01DC0001 with C=0, SSSS=0001
        let direction_bit = if direction == Direction::Forward {
            0b0010_0000
        } else {
            0
        };
        0b0100_0001 | direction_bit
    }

    fn encode_service_mode_cv(cv: u16, op_prefix: u8) -> Result<(u8, u8), PacketEncodeError> {
        if !(1..=256).contains(&cv) {
            return Err(PacketEncodeError::InvalidCvAddress { cv });
        }
        let cv_high = op_prefix | (((cv >> 8) & 0b11) as u8);
        let cv_low = (cv & 0xFF) as u8;
        Ok((cv_high, cv_low))
    }

    /// Helper function for tests - create an idle packet
    pub fn idle() -> Self {
        DccPacket::Idle
    }

    /// Helper function for tests - create a reset packet
    pub fn reset() -> Self {
        DccPacket::Reset
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
    /// followed by the error detection checksum byte.
    ///
    /// # Errors
    /// Returns [`PacketEncodeError::InvalidCvAddress`] when service-mode CV is outside 1..=256.
    /// Speed ranges are guaranteed valid by [`NmraSpeed28`] and [`NmraSpeed128`] newtypes.
    pub fn to_bytes(&self) -> Result<Vec<u8, 6>, PacketEncodeError> {
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
                packet.push(Self::encode_speed28_instruction(direction, speed)?);
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
            DccPacket::FunctionGroup1 {
                address,
                fl,
                f1,
                f2,
                f3,
                f4,
            } => {
                packet.push_address(address);
                packet.push(Self::encode_function_group1(fl, f1, f2, f3, f4));
            }
            DccPacket::FunctionGroup2A {
                address,
                f5,
                f6,
                f7,
                f8,
            } => {
                packet.push_address(address);
                packet.push(Self::encode_function_group2a(f5, f6, f7, f8));
            }
            DccPacket::FunctionGroup2B {
                address,
                f9,
                f10,
                f11,
                f12,
            } => {
                packet.push_address(address);
                packet.push(Self::encode_function_group2b(f9, f10, f11, f12));
            }
            DccPacket::FunctionGroup3 {
                address,
                f13,
                f14,
                f15,
                f16,
                f17,
                f18,
                f19,
                f20,
            } => {
                packet.push_address(address);
                packet.push(0xDE); // Binary state control F13-F20
                packet.push(Self::encode_function_group_data([
                    f13, f14, f15, f16, f17, f18, f19, f20,
                ]));
            }
            DccPacket::FunctionGroup4 {
                address,
                f21,
                f22,
                f23,
                f24,
                f25,
                f26,
                f27,
                f28,
            } => {
                packet.push_address(address);
                packet.push(0xDF); // Binary state control F21-F28
                packet.push(Self::encode_function_group_data([
                    f21, f22, f23, f24, f25, f26, f27, f28,
                ]));
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
            DccPacket::ServiceModeVerifyByte { cv, value } => {
                // Service Mode Verify Byte per NMRA S-9.2.2
                // Packet format: [preamble] 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 [checksum] 1
                // Where:
                //   0111CCAA AAAAAAAA = CV address (Direct Mode, CC=00 for CV 1-256)
                //   DDDDDDDD = value to verify (0-255)
                //
                // For CV 1-256: CC=00 (bits 9-8), AA AAAAAAAA = bits 7-0
                // Instruction byte: 0b0111_01AA (bits 3-2 = 01 for Verify)
                let (cv_high, cv_low) = Self::encode_service_mode_cv(cv, 0b0111_0100)?;
                packet.push(cv_high);
                packet.push(cv_low);
                packet.push(value);
            }
            DccPacket::ServiceModeWriteByte { cv, value } => {
                // Service Mode Write Byte per NMRA S-9.2.2
                // Packet format: same as Verify, but instruction byte bits 3-2 = 11 (Write)
                // Instruction byte: 0b0111_11AA (bits 3-2 = 11 for Write)
                let (cv_high, cv_low) = Self::encode_service_mode_cv(cv, 0b0111_1100)?;
                packet.push(cv_high);
                packet.push(cv_low);
                packet.push(value);
            }
        }

        Ok(packet.finalize())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_short_address_validation() {
        assert_eq!(DccAddress::new_short(0), None);
        assert_eq!(DccAddress::new_short(128), None);
        assert!(DccAddress::new_short(3).is_some());
        assert!(DccAddress::new_short(127).is_some());
        let addr = DccAddress::new_short(3).unwrap();
        assert!(addr.is_short());
        assert!(!addr.is_long());
        assert_eq!(addr.value(), 3);
    }

    #[test]
    fn test_long_address_validation() {
        assert_eq!(DccAddress::new_long(127), None);
        assert_eq!(DccAddress::new_long(10240), None);
        assert!(DccAddress::new_long(128).is_some());
        assert!(DccAddress::new_long(10239).is_some());
        let addr = DccAddress::new_long(1000).unwrap();
        assert!(addr.is_long());
        assert!(!addr.is_short());
        assert_eq!(addr.value(), 1000);
    }

    #[test]
    fn test_idle_packet_encoding() {
        let packet = DccPacket::Idle;
        let bytes = packet.to_bytes().unwrap();
        // Idle packet: [0xFF, 0x00, checksum]
        // checksum = 0xFF XOR 0x00 = 0xFF
        assert_eq!(bytes.as_slice(), &[0xFF, 0x00, 0xFF]);
    }

    #[test]
    fn test_reset_packet_encoding() {
        let packet = DccPacket::Reset;
        let bytes = packet.to_bytes().unwrap();
        assert_eq!(bytes.as_slice(), &[0x00, 0x00, 0x00]);
    }

    #[test]
    fn test_speed28_packet_encoding() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::speed_28step(addr, 14, Direction::Forward).unwrap();
        let bytes = packet.to_bytes().unwrap();

        // address = 3
        // instruction = 01DCSSSS per NMRA S-9.2
        // speed 14: step=13, SSSS=(13+3)/2=8, C=(13-1)&1=0, CSSSS=0b0_1000
        // instruction = 0b0100_0000 | 0b0010_0000 (fwd) | 0b0000_1000 = 0b0110_1000 = 0x68
        // checksum = 3 ^ 0x68 = 0x6B
        assert_eq!(bytes.len(), 3);
        assert_eq!(bytes[0], 3);
        assert_eq!(bytes[1], 0x68);
        assert_eq!(bytes[2], 0x6B);
    }

    #[test]
    fn test_speed128_packet_encoding() {
        let packet =
            DccPacket::speed_128step(DccAddress::new_long(1000).unwrap(), 64, Direction::Reverse)
                .unwrap();
        let bytes = packet.to_bytes().unwrap();

        // Expected: [addr_high, addr_low, 0x3F, speed_byte, checksum]
        // Long address 1000 = 0x03E8
        // high = 0xC0 | (0x03E8 >> 8) = 0xC0 | 0x03 = 0xC3
        // low = 0xE8
        // logical speed 64 -> wire 65 (reverse, so bit 7 = 0) = 0x41
        // checksum = 0xC3 ^ 0xE8 ^ 0x3F ^ 0x41
        let expected_checksum = 0xC3 ^ 0xE8 ^ 0x3F ^ 0x41;

        assert_eq!(bytes.len(), 5);
        assert_eq!(bytes[0], 0xC3);
        assert_eq!(bytes[1], 0xE8);
        assert_eq!(bytes[2], 0x3F);
        assert_eq!(bytes[3], 0x41);
        assert_eq!(bytes[4], expected_checksum);
    }

    #[test]
    fn test_function_group1_encoding() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::FunctionGroup1 {
            address: addr,
            fl: true,
            f1: true,
            f2: false,
            f3: true,
            f4: false,
        };
        let bytes = packet.to_bytes().unwrap();
        // instruction = 100DDDDD = 0b1000_0000 | FL(0b1_0000) | F4(0) | F3(0b100) | F2(0) | F1(0b1)
        // = 0b1001_0101 = 0x95
        assert_eq!(bytes.len(), 3);
        assert_eq!(bytes[0], 3);
        assert_eq!(bytes[1], 0b1001_0101);
        assert_eq!(bytes[2], 3 ^ 0b1001_0101);
    }

    #[test]
    fn test_function_group2a_encoding() {
        let addr = DccAddress::new_short(5).unwrap();
        let packet = DccPacket::FunctionGroup2A {
            address: addr,
            f5: true,
            f6: false,
            f7: true,
            f8: false,
        };
        let bytes = packet.to_bytes().unwrap();
        // instruction = 1011DDDD = 0b1011_0000 | F8(0) | F7(0b100) | F6(0) | F5(0b1)
        // = 0b1011_0101 = 0xB5
        assert_eq!(bytes.len(), 3);
        assert_eq!(bytes[0], 5);
        assert_eq!(bytes[1], 0b1011_0101);
        assert_eq!(bytes[2], 5 ^ 0b1011_0101);
    }

    #[test]
    fn test_function_group2b_encoding() {
        let addr = DccAddress::new_short(5).unwrap();
        let packet = DccPacket::FunctionGroup2B {
            address: addr,
            f9: false,
            f10: false,
            f11: true,
            f12: true,
        };
        let bytes = packet.to_bytes().unwrap();
        // instruction = 1010DDDD = 0b1010_0000 | F12(0b1000) | F11(0b100) | F10(0) | F9(0)
        // = 0b1010_1100 = 0xAC
        assert_eq!(bytes.len(), 3);
        assert_eq!(bytes[0], 5);
        assert_eq!(bytes[1], 0b1010_1100);
        assert_eq!(bytes[2], 5 ^ 0b1010_1100);
    }

    #[test]
    fn test_emergency_stop_encoding() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::EmergencyStop {
            address: addr,
            direction: Direction::Forward,
        };
        let bytes = packet.to_bytes().unwrap();
        // instruction = 01DC0001 (C=0): 0b0100_0001 | 0b0010_0000 (fwd) = 0b0110_0001 = 0x61
        assert_eq!(bytes.len(), 3);
        assert_eq!(bytes[0], 3);
        assert_eq!(bytes[1], 0b0110_0001);
        assert_eq!(bytes[2], 3 ^ 0b0110_0001);
    }

    #[test]
    fn test_broadcast_stop_encoding() {
        let packet = DccPacket::BroadcastStop;
        let bytes = packet.to_bytes().unwrap();
        // [0x00, 0b0110_0001, checksum]
        // 0b0110_0001 = 01DC0001, D=1 (fwd), C=0, SSSS=0001 (e-stop)
        // checksum = 0x00 ^ 0b0110_0001 = 0b0110_0001
        assert_eq!(bytes.len(), 3);
        assert_eq!(bytes[0], 0x00);
        assert_eq!(bytes[1], 0b0110_0001);
        assert_eq!(bytes[2], 0b0110_0001);
    }

    #[test]
    fn test_broadcast_stop_differs_from_reset() {
        let stop = DccPacket::BroadcastStop.to_bytes().unwrap();
        let reset = DccPacket::Reset.to_bytes().unwrap();
        // BroadcastStop has instruction byte 0x61, Reset has 0x00
        assert_ne!(stop.as_slice(), reset.as_slice());
    }

    #[test]
    fn test_function_group3_encoding() {
        let addr = DccAddress::new_short(5).unwrap();
        let packet = DccPacket::FunctionGroup3 {
            address: addr,
            f13: true,
            f14: false,
            f15: true,
            f16: false,
            f17: false,
            f18: true,
            f19: false,
            f20: true,
        };
        let bytes = packet.to_bytes().unwrap();
        // data = 0b1010_0101 (F20=1, F18=1, F15=1, F13=1)
        assert_eq!(bytes.len(), 4);
        assert_eq!(bytes[0], 5);
        assert_eq!(bytes[1], 0xDE);
        assert_eq!(bytes[2], 0b1010_0101);
        assert_eq!(bytes[3], 5 ^ 0xDE ^ 0b1010_0101);
    }

    #[test]
    fn test_function_group4_encoding() {
        let addr = DccAddress::new_short(7).unwrap();
        let packet = DccPacket::FunctionGroup4 {
            address: addr,
            f21: false,
            f22: true,
            f23: false,
            f24: true,
            f25: true,
            f26: false,
            f27: false,
            f28: true,
        };
        let bytes = packet.to_bytes().unwrap();
        // data = 0b1001_1010 (F28=1, F25=1, F24=1, F22=1)
        assert_eq!(bytes.len(), 4);
        assert_eq!(bytes[0], 7);
        assert_eq!(bytes[1], 0xDF);
        assert_eq!(bytes[2], 0b1001_1010);
        assert_eq!(bytes[3], 7 ^ 0xDF ^ 0b1001_1010);
    }

    #[test]
    fn test_function_group3_long_address_encoding() {
        let addr = DccAddress::new_long(1000).unwrap(); // 0x03E8 -> [0xC3, 0xE8]
        let packet = DccPacket::FunctionGroup3 {
            address: addr,
            f13: true,
            f14: false,
            f15: true,
            f16: false,
            f17: false,
            f18: true,
            f19: false,
            f20: true,
        };
        let bytes = packet.to_bytes().unwrap();
        assert_eq!(bytes.len(), 5);
        assert_eq!(bytes[0], 0xC3);
        assert_eq!(bytes[1], 0xE8);
        assert_eq!(bytes[2], 0xDE);
        assert_eq!(bytes[3], 0b1010_0101);
        assert_eq!(bytes[4], 0xC3 ^ 0xE8 ^ 0xDE ^ 0b1010_0101);
    }

    #[test]
    fn test_function_group4_long_address_encoding() {
        let addr = DccAddress::new_long(1000).unwrap(); // 0x03E8 -> [0xC3, 0xE8]
        let packet = DccPacket::FunctionGroup4 {
            address: addr,
            f21: false,
            f22: true,
            f23: false,
            f24: true,
            f25: true,
            f26: false,
            f27: false,
            f28: true,
        };
        let bytes = packet.to_bytes().unwrap();
        assert_eq!(bytes.len(), 5);
        assert_eq!(bytes[0], 0xC3);
        assert_eq!(bytes[1], 0xE8);
        assert_eq!(bytes[2], 0xDF);
        assert_eq!(bytes[3], 0b1001_1010);
        assert_eq!(bytes[4], 0xC3 ^ 0xE8 ^ 0xDF ^ 0b1001_1010);
    }

    #[test]
    fn test_speed28_constructor_validates_range() {
        let addr = DccAddress::new_short(3).unwrap();
        assert!(DccPacket::speed_28step(addr, 0, Direction::Forward).is_some());
        assert!(DccPacket::speed_28step(addr, 29, Direction::Forward).is_some());
        assert!(DccPacket::speed_28step(addr, 30, Direction::Forward).is_none());
    }

    /// Verify Speed28 interleaved encoding against NMRA S-9.2 §2.3.2.3.
    ///
    /// Instruction byte: 01DCSSSS
    /// - D = direction (1=forward, 0=reverse)
    /// - For speed 0: CSSSS=00000 (stop)
    /// - For speed 1: CSSSS=00001 (e-stop)
    /// - For speed 2-29: step=speed-1, SSSS=(step+3)/2, C=(step-1)&1
    #[test]
    fn test_speed28_nmra_table() {
        let addr = DccAddress::new_short(1).unwrap();

        // Verify stop (speed=0): CSSSS = 00000
        let pkt = DccPacket::speed_28step(addr, 0, Direction::Forward).unwrap();
        let bytes = pkt.to_bytes().unwrap();
        assert_eq!(bytes[1] & 0b0001_1111, 0b0_0000, "speed=0 should be stop");

        // Verify e-stop (speed=1): CSSSS = 00001
        let pkt = DccPacket::speed_28step(addr, 1, Direction::Forward).unwrap();
        let bytes = pkt.to_bytes().unwrap();
        assert_eq!(bytes[1] & 0b0001_1111, 0b0_0001, "speed=1 should be e-stop");

        // Verify direction bit: forward=1, reverse=0
        let fwd = DccPacket::speed_28step(addr, 10, Direction::Forward).unwrap();
        let rev = DccPacket::speed_28step(addr, 10, Direction::Reverse).unwrap();
        assert_eq!(
            fwd.to_bytes().unwrap()[1] & 0b0010_0000,
            0b0010_0000,
            "forward D=1"
        );
        assert_eq!(
            rev.to_bytes().unwrap()[1] & 0b0010_0000,
            0b0000_0000,
            "reverse D=0"
        );

        // Verify NMRA S-9.2 interleaved encoding for speed values:
        // step=speed-1, SSSS=(step+3)/2, C=(step-1)&1
        let test_cases: &[(u8, u8)] = &[
            (2, 0b0_0010),  // step=1:  SSSS=2,  C=0
            (3, 0b1_0010),  // step=2:  SSSS=2,  C=1
            (4, 0b0_0011),  // step=3:  SSSS=3,  C=0
            (5, 0b1_0011),  // step=4:  SSSS=3,  C=1
            (14, 0b0_1000), // step=13: SSSS=8,  C=0
            (28, 0b0_1111), // step=27: SSSS=15, C=0
            (29, 0b1_1111), // step=28: SSSS=15, C=1
        ];

        for &(speed, expected_cssss) in test_cases {
            let pkt = DccPacket::speed_28step(addr, speed, Direction::Forward).unwrap();
            let bytes = pkt.to_bytes().unwrap();
            assert_eq!(
                bytes[1] & 0b0001_1111,
                expected_cssss,
                "speed={speed}: instruction byte CSSSS={:#07b}, expected {expected_cssss:#07b}",
                bytes[1] & 0b0001_1111
            );
        }
    }

    #[test]
    fn test_speed128_constructor_validates_range() {
        let addr = DccAddress::new_short(3).unwrap();
        assert!(DccPacket::speed_128step(addr, 0, Direction::Forward).is_some());
        assert!(DccPacket::speed_128step(addr, 126, Direction::Forward).is_some());
        assert!(DccPacket::speed_128step(addr, 127, Direction::Forward).is_none());
    }

    #[test]
    fn test_service_mode_verify_byte_encoding() {
        // Test CV1 verify value 3 (common decoder address verification)
        let packet = DccPacket::ServiceModeVerifyByte { cv: 1, value: 3 };
        let bytes = packet.to_bytes().unwrap();

        // Expected: [0b0111_0100, 0x01, 0x03, checksum]
        // CV high byte: 0b0111_0100 (0111 01 00 - Verify instruction, CV bits 9-8 = 00)
        // CV low byte: 0x01 (CV = 1)
        // Value: 0x03
        // Checksum: 0b0111_0100 ^ 0x01 ^ 0x03
        assert_eq!(bytes.len(), 4);
        assert_eq!(bytes[0], 0b0111_0100);
        assert_eq!(bytes[1], 0x01);
        assert_eq!(bytes[2], 0x03);
        assert_eq!(bytes[3], 0b0111_0100 ^ 0x01 ^ 0x03);
    }

    #[test]
    fn test_service_mode_write_byte_encoding() {
        // Test CV29 write value 6 (common configuration CV)
        let packet = DccPacket::ServiceModeWriteByte { cv: 29, value: 6 };
        let bytes = packet.to_bytes().unwrap();

        // Expected: [0b0111_1100, 0x1D, 0x06, checksum]
        // CV high byte: 0b0111_1100 (0111 11 00 - Write instruction, CV bits 9-8 = 00)
        // CV low byte: 0x1D (CV = 29)
        // Value: 0x06
        // Checksum: 0b0111_1100 ^ 0x1D ^ 0x06
        assert_eq!(bytes.len(), 4);
        assert_eq!(bytes[0], 0b0111_1100);
        assert_eq!(bytes[1], 29);
        assert_eq!(bytes[2], 6);
        assert_eq!(bytes[3], 0b0111_1100 ^ 29 ^ 6);
    }

    #[test]
    fn test_service_mode_verify_vs_write_instruction_bits() {
        let verify = DccPacket::ServiceModeVerifyByte { cv: 1, value: 0 };
        let write = DccPacket::ServiceModeWriteByte { cv: 1, value: 0 };

        let verify_bytes = verify.to_bytes().unwrap();
        let write_bytes = write.to_bytes().unwrap();

        // Instruction bytes differ only in bits 3-2: Verify=01, Write=11
        // Verify: 0b0111_0100 (0x74)
        // Write:  0b0111_1100 (0x7C)
        assert_eq!(verify_bytes[0], 0b0111_0100);
        assert_eq!(write_bytes[0], 0b0111_1100);
        assert_eq!(verify_bytes[0] & 0b1111_0011, write_bytes[0] & 0b1111_0011); // Other bits match
    }

    #[test]
    fn test_service_mode_cv_address_range() {
        // Test boundary cases for CV 1-256
        let cv1 = DccPacket::ServiceModeVerifyByte { cv: 1, value: 0 };
        let cv256 = DccPacket::ServiceModeVerifyByte { cv: 256, value: 0 };

        let bytes1 = cv1.to_bytes().unwrap();
        let bytes256 = cv256.to_bytes().unwrap();

        // CV1: high byte = 0b0111_0100, low byte = 0x01
        assert_eq!(bytes1[0], 0b0111_0100);
        assert_eq!(bytes1[1], 1);

        // CV256: high byte = 0b0111_0101, low byte = 0x00
        assert_eq!(bytes256[0], 0b0111_0101);
        assert_eq!(bytes256[1], 0);
    }

    #[test]
    fn test_service_mode_cv_out_of_range_rejected() {
        let cv0 = DccPacket::ServiceModeVerifyByte { cv: 0, value: 0 };
        assert!(matches!(
            cv0.to_bytes(),
            Err(PacketEncodeError::InvalidCvAddress { cv: 0 })
        ));

        let cv257 = DccPacket::ServiceModeWriteByte { cv: 257, value: 0 };
        assert!(matches!(
            cv257.to_bytes(),
            Err(PacketEncodeError::InvalidCvAddress { cv: 257 })
        ));
    }

    #[test]
    fn test_service_mode_checksum_calculation() {
        // Verify checksum is XOR of all data bytes
        let packet = DccPacket::ServiceModeWriteByte { cv: 17, value: 192 };
        let bytes = packet.to_bytes().unwrap();

        let expected_checksum = bytes[0] ^ bytes[1] ^ bytes[2];
        assert_eq!(bytes[3], expected_checksum);
    }
}
