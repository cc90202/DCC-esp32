//! Persistent WiFi credential record codec and store policy.
//!
//! The store is written against `embedded-storage` NOR flash traits so the
//! production adapter can use `esp-storage` without a project-specific flash
//! abstraction.

use super::credentials::WifiCredentials;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

const MAGIC: &[u8; 7] = b"DCCWIFI";
const VERSION: u8 = 1;
const FLAG_FORCE_PROVISIONING: u8 = 0b0000_0001;
const VALID_FLAGS_MASK: u8 = FLAG_FORCE_PROVISIONING;
const SSID_MAX_LEN: usize = 32;
const PASSWORD_MAX_LEN: usize = 64;
const SECTOR_SIZE: usize = 4096;

// Record layout, shared by `encode_record` and `decode_record`.
const OFFSET_VERSION: usize = MAGIC.len();
const OFFSET_GENERATION: usize = OFFSET_VERSION + 1;
const OFFSET_FLAGS: usize = OFFSET_GENERATION + 4;
const OFFSET_SSID_LEN: usize = OFFSET_FLAGS + 1;
const OFFSET_PASSWORD_LEN: usize = OFFSET_SSID_LEN + 1;
const OFFSET_SSID: usize = OFFSET_PASSWORD_LEN + 1;
const OFFSET_PASSWORD: usize = OFFSET_SSID + SSID_MAX_LEN;
const RECORD_BODY_LEN: usize = OFFSET_PASSWORD + PASSWORD_MAX_LEN;
const RECORD_LEN: usize = (RECORD_BODY_LEN + 4).next_multiple_of(4);
const CHECKSUM_OFFSET: usize = RECORD_LEN - 4;

// esp-storage writes must be 4-byte aligned, and both slots must fit their
// own erase sector.
const _: () = assert!(RECORD_LEN.is_multiple_of(4));
const _: () = assert!(RECORD_LEN <= SECTOR_SIZE);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum StoreError {
    FlashRead,
    FlashErase,
    FlashWrite,
    Corrupt,
    MissingCredentials,
    BufferTooSmall,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConfigSlot {
    A,
    B,
}

impl ConfigSlot {
    const fn inactive_after(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }
}

pub trait WifiCredentialsStore {
    fn load(&mut self) -> Result<Option<WifiCredentials>, StoreError>;
    fn save(&mut self, credentials: &WifiCredentials) -> Result<(), StoreError>;
    fn clear(&mut self) -> Result<(), StoreError>;
}

pub trait ProvisioningFlagStore {
    fn force_on_next_boot(&mut self) -> Result<bool, StoreError>;
    fn set_force_on_next_boot(&mut self) -> Result<(), StoreError>;
    fn clear_force_on_next_boot(&mut self) -> Result<(), StoreError>;
}

pub struct WifiConfigStore<F> {
    flash: F,
}

#[derive(Debug, PartialEq, Eq)]
struct StoredRecord {
    credentials: WifiCredentials,
    generation: u32,
    force_provisioning: bool,
}

#[derive(Debug, PartialEq, Eq)]
enum SlotState {
    Missing,
    Valid(StoredRecord),
    Corrupt,
}

#[derive(Debug, PartialEq, Eq)]
enum LatestRecord {
    Missing,
    Valid {
        slot: ConfigSlot,
        record: StoredRecord,
    },
    Corrupt,
}

impl<F> WifiConfigStore<F> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }
}

impl<F> WifiCredentialsStore for WifiConfigStore<F>
where
    F: ReadNorFlash + NorFlash,
{
    fn load(&mut self) -> Result<Option<WifiCredentials>, StoreError> {
        match self.load_latest()? {
            LatestRecord::Missing => Ok(None),
            LatestRecord::Valid { record, .. } => Ok(Some(record.credentials)),
            LatestRecord::Corrupt => Err(StoreError::Corrupt),
        }
    }

    fn save(&mut self, credentials: &WifiCredentials) -> Result<(), StoreError> {
        let latest = self.load_latest()?;
        let (slot, generation) = match latest {
            LatestRecord::Missing | LatestRecord::Corrupt => (ConfigSlot::A, 1),
            LatestRecord::Valid { slot, record } => {
                (slot.inactive_after(), next_generation(record.generation))
            }
        };

        // A successful save always clears any pending force-provisioning
        // flag: rebooting after saving must reach station mode.
        self.write_slot(slot, credentials, generation, false)
    }

    fn clear(&mut self) -> Result<(), StoreError> {
        // Erase the inactive (older) slot first so a power loss between the
        // two erases cannot resurrect a stale record.
        let first = match self.load_latest() {
            Ok(LatestRecord::Valid { slot, .. }) => slot.inactive_after(),
            _ => ConfigSlot::A,
        };
        self.erase_slot(first)?;
        self.erase_slot(first.inactive_after())?;
        Ok(())
    }
}

impl<F> ProvisioningFlagStore for WifiConfigStore<F>
where
    F: ReadNorFlash + NorFlash,
{
    fn force_on_next_boot(&mut self) -> Result<bool, StoreError> {
        match self.load_latest()? {
            LatestRecord::Missing => Ok(false),
            LatestRecord::Valid { record, .. } => Ok(record.force_provisioning),
            LatestRecord::Corrupt => Err(StoreError::Corrupt),
        }
    }

    fn set_force_on_next_boot(&mut self) -> Result<(), StoreError> {
        self.update_force_flag(true)
    }

    fn clear_force_on_next_boot(&mut self) -> Result<(), StoreError> {
        match self.update_force_flag(false) {
            // Clearing with no stored record is a no-op, not an error.
            Err(StoreError::MissingCredentials) => Ok(()),
            result => result,
        }
    }
}

impl<F> WifiConfigStore<F>
where
    F: ReadNorFlash + NorFlash,
{
    fn update_force_flag(&mut self, force_provisioning: bool) -> Result<(), StoreError> {
        match self.load_latest()? {
            LatestRecord::Missing => Err(StoreError::MissingCredentials),
            LatestRecord::Valid { slot, record } => {
                let next_slot = slot.inactive_after();
                self.write_slot(
                    next_slot,
                    &record.credentials,
                    next_generation(record.generation),
                    force_provisioning,
                )
            }
            LatestRecord::Corrupt => Err(StoreError::Corrupt),
        }
    }

    fn write_slot(
        &mut self,
        slot: ConfigSlot,
        credentials: &WifiCredentials,
        generation: u32,
        force_provisioning: bool,
    ) -> Result<(), StoreError> {
        let record = encode_record(credentials, generation, force_provisioning)?;
        self.erase_slot(slot)?;
        self.flash
            .write(slot_offset(slot), &record)
            .map_err(|_| StoreError::FlashWrite)
    }

    fn load_latest(&mut self) -> Result<LatestRecord, StoreError> {
        let a = self.read_slot(ConfigSlot::A)?;
        let b = self.read_slot(ConfigSlot::B)?;
        Ok(select_latest(a, b))
    }

    fn read_slot(&mut self, slot: ConfigSlot) -> Result<SlotState, StoreError> {
        let mut record = [0xff; RECORD_LEN];
        self.flash
            .read(slot_offset(slot), &mut record)
            .map_err(|_| StoreError::FlashRead)?;
        Ok(decode_slot(&record))
    }

    fn erase_slot(&mut self, slot: ConfigSlot) -> Result<(), StoreError> {
        let from = slot_offset(slot);
        self.flash
            .erase(from, from + SECTOR_SIZE as u32)
            .map_err(|_| StoreError::FlashErase)
    }
}

const fn slot_offset(slot: ConfigSlot) -> u32 {
    match slot {
        ConfigSlot::A => 0,
        ConfigSlot::B => SECTOR_SIZE as u32,
    }
}

fn select_latest(a: SlotState, b: SlotState) -> LatestRecord {
    match (a, b) {
        (SlotState::Missing, SlotState::Missing) => LatestRecord::Missing,
        (SlotState::Valid(record), SlotState::Missing | SlotState::Corrupt) => {
            LatestRecord::Valid {
                slot: ConfigSlot::A,
                record,
            }
        }
        (SlotState::Missing | SlotState::Corrupt, SlotState::Valid(record)) => {
            LatestRecord::Valid {
                slot: ConfigSlot::B,
                record,
            }
        }
        (SlotState::Valid(a), SlotState::Valid(b)) => {
            if is_same_or_newer_generation(a.generation, b.generation) {
                LatestRecord::Valid {
                    slot: ConfigSlot::A,
                    record: a,
                }
            } else {
                LatestRecord::Valid {
                    slot: ConfigSlot::B,
                    record: b,
                }
            }
        }
        (SlotState::Corrupt, SlotState::Corrupt)
        | (SlotState::Corrupt, SlotState::Missing)
        | (SlotState::Missing, SlotState::Corrupt) => LatestRecord::Corrupt,
    }
}

fn next_generation(generation: u32) -> u32 {
    generation.wrapping_add(1)
}

fn is_same_or_newer_generation(candidate: u32, current: u32) -> bool {
    candidate == current || candidate.wrapping_sub(current) < (1 << 31)
}

fn decode_slot(record: &[u8; RECORD_LEN]) -> SlotState {
    if is_empty_record(record) {
        return SlotState::Missing;
    }

    decode_record(record).map_or(SlotState::Corrupt, SlotState::Valid)
}

fn is_empty_record(record: &[u8; RECORD_LEN]) -> bool {
    record.iter().all(|byte| *byte == 0xff)
}

fn encode_record(
    credentials: &WifiCredentials,
    generation: u32,
    force_provisioning: bool,
) -> Result<[u8; RECORD_LEN], StoreError> {
    let ssid = credentials.ssid().as_bytes();
    let password = credentials.password().as_bytes();
    if ssid.len() > SSID_MAX_LEN || password.len() > PASSWORD_MAX_LEN {
        return Err(StoreError::BufferTooSmall);
    }

    let mut record = [0u8; RECORD_LEN];
    record[..MAGIC.len()].copy_from_slice(MAGIC);
    record[OFFSET_VERSION] = VERSION;
    record[OFFSET_GENERATION..OFFSET_FLAGS].copy_from_slice(&generation.to_le_bytes());
    record[OFFSET_FLAGS] = u8::from(force_provisioning);
    record[OFFSET_SSID_LEN] = ssid.len() as u8;
    record[OFFSET_PASSWORD_LEN] = password.len() as u8;
    record[OFFSET_SSID..OFFSET_SSID + ssid.len()].copy_from_slice(ssid);
    record[OFFSET_PASSWORD..OFFSET_PASSWORD + password.len()].copy_from_slice(password);

    let checksum = checksum(&record[..CHECKSUM_OFFSET]);
    record[CHECKSUM_OFFSET..].copy_from_slice(&checksum.to_le_bytes());
    Ok(record)
}

fn decode_record(record: &[u8]) -> Option<StoredRecord> {
    if record.len() < RECORD_LEN || &record[..MAGIC.len()] != MAGIC {
        return None;
    }
    if record[OFFSET_VERSION] != VERSION {
        return None;
    }

    let expected = u32::from_le_bytes(record[CHECKSUM_OFFSET..].try_into().ok()?);
    if checksum(&record[..CHECKSUM_OFFSET]) != expected {
        return None;
    }

    let flags = record[OFFSET_FLAGS];
    if flags & !VALID_FLAGS_MASK != 0 {
        return None;
    }

    let ssid_len = record[OFFSET_SSID_LEN] as usize;
    let password_len = record[OFFSET_PASSWORD_LEN] as usize;
    if ssid_len > SSID_MAX_LEN || password_len > PASSWORD_MAX_LEN {
        return None;
    }

    let ssid = core::str::from_utf8(&record[OFFSET_SSID..OFFSET_SSID + ssid_len]).ok()?;
    let password =
        core::str::from_utf8(&record[OFFSET_PASSWORD..OFFSET_PASSWORD + password_len]).ok()?;
    let credentials = WifiCredentials::new(ssid, password).ok()?;
    let generation = u32::from_le_bytes(record[OFFSET_GENERATION..OFFSET_FLAGS].try_into().ok()?);

    Some(StoredRecord {
        credentials,
        generation,
        force_provisioning: flags & FLAG_FORCE_PROVISIONING != 0,
    })
}

/// FNV-1a 32-bit hash (offset basis 0x811c9dc5, prime 0x01000193).
///
/// Detects flash corruption and torn writes; it is not a cryptographic
/// integrity check and does not need to be.
fn checksum(bytes: &[u8]) -> u32 {
    let mut hash = 0x811c_9dc5u32;
    for byte in bytes {
        hash ^= u32::from(*byte);
        hash = hash.wrapping_mul(0x0100_0193);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockFlashError;

    impl NorFlashError for MockFlashError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    #[derive(Debug)]
    struct MockFlash {
        sectors: [[u8; SECTOR_SIZE]; 2],
        /// `Some(n)`: the erase after `n` successful ones fails once.
        erases_before_failure: Option<u8>,
        fail_next_write: bool,
    }

    impl MockFlash {
        const fn erased() -> Self {
            Self {
                sectors: [[0xff; SECTOR_SIZE]; 2],
                erases_before_failure: None,
                fail_next_write: false,
            }
        }

        fn sector_index(offset: u32) -> Result<usize, MockFlashError> {
            match offset {
                0 => Ok(0),
                x if x == SECTOR_SIZE as u32 => Ok(1),
                _ => Err(MockFlashError),
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = MockFlashError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let sector = Self::sector_index(offset)?;
            if bytes.len() > SECTOR_SIZE {
                return Err(MockFlashError);
            }
            bytes.copy_from_slice(&self.sectors[sector][..bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            SECTOR_SIZE * 2
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = SECTOR_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            match self.erases_before_failure {
                Some(0) => {
                    self.erases_before_failure = None;
                    return Err(MockFlashError);
                }
                Some(remaining) => self.erases_before_failure = Some(remaining - 1),
                None => {}
            }
            if to != from + SECTOR_SIZE as u32 {
                return Err(MockFlashError);
            }
            let sector = Self::sector_index(from)?;
            self.sectors[sector] = [0xff; SECTOR_SIZE];
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if self.fail_next_write {
                self.fail_next_write = false;
                return Err(MockFlashError);
            }
            let sector = Self::sector_index(offset)?;
            self.sectors[sector][..bytes.len()].copy_from_slice(bytes);
            Ok(())
        }
    }

    fn credentials(ssid: &str, password: &str) -> WifiCredentials {
        WifiCredentials::new(ssid, password).unwrap()
    }

    #[test]
    fn missing_config_returns_none_and_force_flag_false() {
        let mut store = WifiConfigStore::new(MockFlash::erased());

        assert_eq!(store.load(), Ok(None));
        assert_eq!(store.force_on_next_boot(), Ok(false));
    }

    #[test]
    fn valid_record_round_trips_without_password_in_debug() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let credentials = credentials("DCC-Lab", "password123");

        assert_eq!(store.save(&credentials), Ok(()));
        let loaded = store.load().unwrap().unwrap();
        let debug = format!("{loaded:?}");

        assert_eq!(loaded.ssid(), "DCC-Lab");
        assert_eq!(loaded.password(), "password123");
        assert!(!debug.contains("password123"));
    }

    #[test]
    fn corrupt_magic_is_rejected() {
        let mut flash = MockFlash::erased();
        flash.sectors[0][..MAGIC.len()].copy_from_slice(b"BADWIFI");
        let mut store = WifiConfigStore::new(flash);

        assert_eq!(store.load(), Err(StoreError::Corrupt));
    }

    #[test]
    fn corrupt_checksum_is_rejected() {
        let credentials = credentials("DCC-Lab", "password123");
        let mut record = encode_record(&credentials, 1, false).unwrap();
        record[20] ^= 0x55;
        let mut flash = MockFlash::erased();
        flash.sectors[0][..record.len()].copy_from_slice(&record);
        let mut store = WifiConfigStore::new(flash);

        assert_eq!(store.load(), Err(StoreError::Corrupt));
    }

    #[test]
    fn force_provisioning_flag_round_trips_and_clears() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let credentials = credentials("DCC-Lab", "password123");

        assert_eq!(store.save(&credentials), Ok(()));
        assert_eq!(store.set_force_on_next_boot(), Ok(()));
        assert_eq!(store.force_on_next_boot(), Ok(true));
        assert_eq!(store.clear_force_on_next_boot(), Ok(()));
        assert_eq!(store.force_on_next_boot(), Ok(false));
    }

    #[test]
    fn flag_clear_is_idempotent_when_missing() {
        let mut store = WifiConfigStore::new(MockFlash::erased());

        assert_eq!(store.clear_force_on_next_boot(), Ok(()));
    }

    #[test]
    fn setting_force_flag_requires_existing_credentials() {
        let mut store = WifiConfigStore::new(MockFlash::erased());

        assert_eq!(
            store.set_force_on_next_boot(),
            Err(StoreError::MissingCredentials)
        );
    }

    #[test]
    fn save_clears_pending_force_flag() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let old = credentials("DCC-Old", "password123");
        let new = credentials("DCC-New", "password456");

        assert_eq!(store.save(&old), Ok(()));
        assert_eq!(store.set_force_on_next_boot(), Ok(()));
        assert_eq!(store.save(&new), Ok(()));

        assert_eq!(store.force_on_next_boot(), Ok(false));
    }

    #[test]
    fn completed_save_selects_newer_generation() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let old = credentials("DCC-Old", "password123");
        let new = credentials("DCC-New", "password456");

        assert_eq!(store.save(&old), Ok(()));
        assert_eq!(store.save(&new), Ok(()));

        let loaded = store.load().unwrap().unwrap();
        assert_eq!(loaded.ssid(), "DCC-New");
        assert_eq!(loaded.password(), "password456");
    }

    #[test]
    fn interrupted_inactive_write_leaves_old_record_selected() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let old = credentials("DCC-Old", "password123");
        let new = credentials("DCC-New", "password456");

        assert_eq!(store.save(&old), Ok(()));
        store.flash.fail_next_write = true;
        assert_eq!(store.save(&new), Err(StoreError::FlashWrite));

        let loaded = store.load().unwrap().unwrap();
        assert_eq!(loaded.ssid(), "DCC-Old");
        assert_eq!(loaded.password(), "password123");
    }

    #[test]
    fn interrupted_inactive_erase_leaves_old_record_selected() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let old = credentials("DCC-Old", "password123");
        let new = credentials("DCC-New", "password456");

        assert_eq!(store.save(&old), Ok(()));
        store.flash.erases_before_failure = Some(0);
        assert_eq!(store.save(&new), Err(StoreError::FlashErase));

        let loaded = store.load().unwrap().unwrap();
        assert_eq!(loaded.ssid(), "DCC-Old");
        assert_eq!(loaded.password(), "password123");
    }

    #[test]
    fn both_slots_corrupt_returns_corrupt() {
        let mut flash = MockFlash::erased();
        flash.sectors[0][0] = 0x12;
        flash.sectors[1][0] = 0x34;
        let mut store = WifiConfigStore::new(flash);

        assert_eq!(store.load(), Err(StoreError::Corrupt));
    }

    #[test]
    fn all_zero_slot_is_corrupt_not_missing() {
        let mut flash = MockFlash::erased();
        flash.sectors[0] = [0x00; SECTOR_SIZE];
        let mut store = WifiConfigStore::new(flash);

        assert_eq!(store.load(), Err(StoreError::Corrupt));
    }

    #[test]
    fn wrapped_generation_selects_newer_slot() {
        let older = encode_record(&credentials("DCC-Old", "password123"), u32::MAX, false).unwrap();
        let newer = encode_record(&credentials("DCC-New", "password456"), 0, false).unwrap();
        let mut flash = MockFlash::erased();
        flash.sectors[0][..older.len()].copy_from_slice(&older);
        flash.sectors[1][..newer.len()].copy_from_slice(&newer);
        let mut store = WifiConfigStore::new(flash);

        let loaded = store.load().unwrap().unwrap();
        assert_eq!(loaded.ssid(), "DCC-New");
        assert_eq!(loaded.password(), "password456");
    }

    #[test]
    fn clear_erases_saved_credentials() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let credentials = credentials("DCC-Lab", "password123");

        assert_eq!(store.save(&credentials), Ok(()));
        assert_eq!(store.clear(), Ok(()));
        assert_eq!(store.load(), Ok(None));
    }

    #[test]
    fn clear_erases_the_inactive_slot_first() {
        let mut store = WifiConfigStore::new(MockFlash::erased());
        let old = credentials("DCC-Old", "password123");
        let new = credentials("DCC-New", "password456");

        // Three saves: the active record (DCC-New) lives in slot A, the
        // stale one (DCC-Old) in slot B - the case a fixed A-then-B erase
        // order gets wrong.
        assert_eq!(store.save(&old), Ok(()));
        assert_eq!(store.save(&old), Ok(()));
        assert_eq!(store.save(&new), Ok(()));

        // Fail the second erase: with the correct order (inactive B first)
        // only the stale record is gone and DCC-New survives. With the wrong
        // order the active slot A would be erased first and DCC-Old would
        // resurrect.
        store.flash.erases_before_failure = Some(1);
        assert_eq!(store.clear(), Err(StoreError::FlashErase));
        let survivor = store.load().unwrap().unwrap();
        assert_eq!(survivor.ssid(), "DCC-New");
    }
}
