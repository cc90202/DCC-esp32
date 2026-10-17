//! Validated value newtypes carried by DCC packets: speeds, CV numbers,
//! binary-state addresses and RCN-218 logon fields.

/// Validated 128-step speed value per NMRA S-9.2.1.
///
/// Range 0-126: 0=stop, 1-126=speed steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NmraSpeed128(u8);

impl NmraSpeed128 {
    pub const fn new(speed: u8) -> Option<Self> {
        match speed {
            value @ 0..=126 => Some(Self(value)),
            _ => None,
        }
    }

    pub const fn value(self) -> u8 {
        self.0
    }
}

/// Validated service-mode CV address.
///
/// Direct service-mode packets encode CVs 1..=256.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ServiceModeCv(u16);

impl ServiceModeCv {
    #[must_use]
    pub const fn new(cv: u16) -> Option<Self> {
        match cv {
            value @ 1..=256 => Some(Self(value)),
            _ => None,
        }
    }

    #[must_use]
    pub const fn value(self) -> u16 {
        self.0
    }
}

/// Validated Programming-on-Main CV address.
///
/// Long-form POM CV access packets encode CVs 1..=1024.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PomCv(u16);

impl PomCv {
    #[must_use]
    pub const fn new(cv: u16) -> Option<Self> {
        match cv {
            value @ 1..=1024 => Some(Self(value)),
            _ => None,
        }
    }

    #[must_use]
    pub const fn value(self) -> u16 {
        self.0
    }
}

/// Validated seven-bit binary-state address used by NMRA feature expansion packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BinaryStateAddress(u8);

impl BinaryStateAddress {
    #[must_use]
    pub const fn new(address: u8) -> Option<Self> {
        if address <= 0x7F {
            Some(Self(address))
        } else {
            None
        }
    }

    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }
}

/// Validated two-bit RCN-218 automatic-logon group.
///
/// Values are 0=all, 1=loco, 2=accessory, 3=now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LogonGroup(u8);

impl LogonGroup {
    #[must_use]
    pub const fn new(group: u8) -> Option<Self> {
        if group <= 0x03 {
            Some(Self(group))
        } else {
            None
        }
    }

    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }
}

/// RCN-218 logon session identifier emitted by one command station.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct LogonSessionId(u8);

impl LogonSessionId {
    #[must_use]
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }
}
