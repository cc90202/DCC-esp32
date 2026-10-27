//! The 8 MB flash layout of the device: the one source for
//! `partitions.csv`, the boot-time partition check and every flash offset.
//! The chip and project a package names are in [`dcc_ota_format`].

use dcc_ota_format::ImageLen;

/// Total flash size the layout needs.
pub const FLASH_SIZE: usize = 0x80_0000;
/// Erase unit of the ESP32-C6 flash, and the unit of every update write.
pub const SECTOR_LEN: u32 = 0x1000;
/// [`SECTOR_LEN`] as a buffer length.
pub const SECTOR_SIZE: usize = SECTOR_LEN as usize;
/// Size of one application slot: the largest image.
pub const SLOT_SIZE: u32 = ImageLen::MAX.get();
const _: () = assert!(
    ImageLen::MIN.get() == SECTOR_LEN,
    "an image fills its first sector"
);
/// Start of partition `ota_0`, right after `phy_init`.
const SLOT_A_OFFSET: u32 = 0x1_0000;
/// Start of partition `ota_1`, right after `ota_0`.
const SLOT_B_OFFSET: u32 = SLOT_A_OFFSET + SLOT_SIZE;
/// Start of the bootloader's two-sector selection record.
pub const OTADATA_OFFSET: u32 = 0xD000;
/// Start of the two-sector update journal.
pub const JOURNAL_OFFSET: u32 = 0x61_3000;

/// One of the two application slots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    /// Partition `ota_0`.
    A,
    /// Partition `ota_1`.
    B,
}

impl Slot {
    /// Both slots, in partition order.
    pub(crate) const ALL: [Self; 2] = [Self::A, Self::B];

    /// Returns the slot numbered `index` (0 or 1) on disk.
    pub(crate) const fn from_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Self::A),
            1 => Some(Self::B),
            _ => None,
        }
    }

    /// Returns the slot number stored on disk.
    #[must_use]
    pub const fn index(self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
        }
    }

    /// Returns the slot an update writes while this one runs.
    #[must_use]
    pub const fn other(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }

    /// Returns the slot's first flash byte.
    #[must_use]
    pub const fn offset(self) -> u32 {
        match self {
            Self::A => SLOT_A_OFFSET,
            Self::B => SLOT_B_OFFSET,
        }
    }

    /// Returns the slot whose partition starts at `offset`, if any.
    #[must_use]
    pub fn at(offset: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|slot| slot.offset() == offset)
    }
}

/// One row of the required partition table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Partition {
    /// Partition label.
    pub name: &'static str,
    /// Raw ESP-IDF partition type: 0 application, 1 data.
    pub kind: u8,
    /// Raw ESP-IDF partition subtype.
    pub subtype: u8,
    /// First flash byte.
    pub offset: u32,
    /// Length in bytes.
    pub size: u32,
}

/// Raw ESP-IDF partition types and subtypes used below.
const APP: u8 = 0;
const DATA: u8 = 1;
const OTA_0: u8 = 0x10;
const OTA_1: u8 = 0x11;
const DATA_OTA: u8 = 0;
const DATA_PHY: u8 = 1;
const DATA_NVS: u8 = 2;
const DATA_UNDEFINED: u8 = 6;

/// The Wi-Fi configuration partition, also opened by the firmware's
/// configuration store.
pub const DCC_CFG: Partition = Partition {
    name: "dcc_cfg",
    kind: DATA,
    subtype: DATA_NVS,
    offset: 0x61_0000,
    size: 0x3000,
};

/// The exact partition table the firmware must find before it trusts flash.
pub const PARTITIONS: [Partition; 7] = [
    Partition {
        name: "nvs",
        kind: DATA,
        subtype: DATA_NVS,
        offset: 0x9000,
        size: 0x4000,
    },
    Partition {
        name: "otadata",
        kind: DATA,
        subtype: DATA_OTA,
        offset: OTADATA_OFFSET,
        size: 2 * SECTOR_LEN,
    },
    Partition {
        name: "phy_init",
        kind: DATA,
        subtype: DATA_PHY,
        offset: 0xF000,
        size: 0x1000,
    },
    Partition {
        name: "ota_0",
        kind: APP,
        subtype: OTA_0,
        offset: Slot::A.offset(),
        size: SLOT_SIZE,
    },
    Partition {
        name: "ota_1",
        kind: APP,
        subtype: OTA_1,
        offset: Slot::B.offset(),
        size: SLOT_SIZE,
    },
    DCC_CFG,
    Partition {
        name: "ota_journal",
        kind: DATA,
        subtype: DATA_UNDEFINED,
        offset: JOURNAL_OFFSET,
        size: 2 * SECTOR_LEN,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    /// `partitions.csv` is what espflash writes; it must say what the code checks.
    #[test]
    fn partitions_csv_matches_the_required_table() {
        let csv = include_str!("../../../partitions.csv");
        let rows: std::vec::Vec<std::vec::Vec<&str>> = csv
            .lines()
            .filter(|line| !line.starts_with('#') && !line.trim().is_empty())
            .map(|line| line.split(',').map(str::trim).collect())
            .collect();
        assert_eq!(rows.len(), PARTITIONS.len());
        for (row, expected) in rows.iter().zip(PARTITIONS) {
            let number =
                |text: &str| u32::from_str_radix(text.trim_start_matches("0x"), 16).unwrap();
            assert_eq!(row[0], expected.name);
            assert_eq!(row[1], if expected.kind == APP { "app" } else { "data" });
            let subtype = match row[2] {
                "nvs" => DATA_NVS,
                "ota" => DATA_OTA,
                "phy" => DATA_PHY,
                "ota_0" => OTA_0,
                "ota_1" => OTA_1,
                "undefined" => DATA_UNDEFINED,
                other => panic!("unknown subtype {other}"),
            };
            assert_eq!(subtype, expected.subtype, "{}", expected.name);
            assert_eq!(number(row[3]), expected.offset, "{}", expected.name);
            assert_eq!(number(row[4]), expected.size, "{}", expected.name);
        }
    }
}
