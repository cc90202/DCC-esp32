//! The JSON document behind `GET /update/status`.
//!
//! Every value is an internal enum, a number, a canonical version or hex,
//! never client input, so no escaping is needed.

use core::{fmt::Write, time::Duration};

use dcc_ota_format::Version;

use crate::{
    Error,
    admission::Lifecycle,
    journal::{Ending, Record},
    layout::Slot,
    uptime::Uptime,
};

/// Capacity of the rendered document. The widest one, with every number,
/// version and code at its longest, takes about 520 bytes; the rest is room
/// for new fields. The HTTP transmit buffer is sized from it.
pub const JSON_CAPACITY: usize = 768;

/// Random identifier of one boot, so the status page notices a restart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootId(u64);

impl BootId {
    /// Wraps `id`, drawn at random at boot.
    #[must_use]
    pub const fn new(id: u64) -> Self {
        Self(id)
    }
}

impl core::fmt::Display for BootId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:016x}", self.0)
    }
}

/// Progress of the current or last operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Stage {
    /// Boot check running.
    Boot,
    /// Nothing in progress.
    Idle,
    /// Image bytes arriving.
    Receiving,
    /// Image written, being read back.
    Verifying,
    /// Image published, restart pending.
    Ready,
    /// New image on trial.
    Health,
    /// Update state unusable; USB needed.
    Recovery,
}

impl Stage {
    const fn code(self) -> &'static str {
        match self {
            Self::Boot => "boot",
            Self::Idle => "idle",
            Self::Receiving => "receiving",
            Self::Verifying => "verifying",
            Self::Ready => "ready",
            Self::Health => "health",
            Self::Recovery => "recovery",
        }
    }
}

/// How the last update ended, as the page names it; `none` while one runs.
const fn result_code(ending: Option<Ending>) -> &'static str {
    match ending {
        None => "none",
        Some(Ending::Confirmed { .. }) => "ok",
        Some(Ending::Aborted) => "aborted",
        Some(Ending::RolledBack) => "rolled_back",
        Some(Ending::NotBootable) => "not_bootable",
    }
}

/// Displays the value, or nothing when there is none.
struct Blank<T>(Option<T>);

impl<T: core::fmt::Display> core::fmt::Display for Blank<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.as_ref().map_or(Ok(()), |value| value.fmt(f))
    }
}

/// Whole seconds of a span, or `null` when there is none.
struct Seconds(Option<Duration>);

impl core::fmt::Display for Seconds {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            Some(span) => write!(f, "{}", span.as_secs()),
            None => f.write_str("null"),
        }
    }
}

/// Byte counts of the transfer in progress or last finished.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct Transfer {
    /// Bytes received from the network.
    pub received: u32,
    /// Bytes durable in flash.
    pub written: u32,
    /// Image length.
    pub total: u32,
}

/// Everything the status document shows, read at one instant.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Snapshot {
    /// Journal record, if loaded.
    pub record: Option<Record>,
    /// Random per-boot identifier, so the page notices a restart.
    pub boot_id: BootId,
    /// Time since boot.
    pub uptime: Uptime,
    /// Version of the running image.
    pub version: Version,
    /// Slot of the running image.
    pub slot: Slot,
    /// Service state.
    pub lifecycle: Lifecycle,
    /// An operation holds the update lock.
    pub busy: bool,
    /// The track is powered.
    pub track_enabled: bool,
    /// Transfer progress.
    pub transfer: Transfer,
    /// Operation progress.
    pub stage: Stage,
    /// Why the last operation failed.
    pub reason: Option<Error>,
    /// What is left of the trial's healthy minute.
    pub trial_left: Option<Duration>,
}

impl Snapshot {
    /// Renders the document.
    ///
    /// # Errors
    /// Returns [`Error::Internal`] if it does not fit [`JSON_CAPACITY`].
    pub(crate) fn json(&self) -> Result<heapless::String<JSON_CAPACITY>, Error> {
        let mut json = heapless::String::new();
        self.write(&mut json).map_err(|_| Error::Internal)?;
        Ok(json)
    }

    fn write(&self, json: &mut impl Write) -> core::fmt::Result {
        let Transfer {
            received,
            written,
            total,
        } = self.transfer;
        write!(
            json,
            "{{\"schema\":1,\"boot_id\":\"{boot_id}\",\"uptime\":{uptime},\
             \"version\":\"{version}\",\"slot\":{slot},\"pending\":{pending},\
             \"ota_available\":{available},\"uploading\":{busy},\
             \"received_bytes\":{received},\"written_bytes\":{written},\
             \"total_bytes\":{total},\"stage\":\"{stage}\",\
             \"track_enabled\":{track},\"trial_seconds_left\":{left},\
             \"last_update\":",
            boot_id = self.boot_id,
            uptime = self.uptime.since_boot().as_secs(),
            version = self.version,
            slot = self.slot.index(),
            pending = self.lifecycle == Lifecycle::Trial,
            available = self.lifecycle != Lifecycle::Recovery && self.record.is_some(),
            busy = self.busy,
            stage = self.stage.code(),
            track = self.track_enabled,
            left = Seconds(self.trial_left),
        )?;
        let Some(record) = self.record else {
            return json.write_str("null}");
        };
        let (digest, version) = record
            .candidate()
            .map(|c| (c.digest(), c.version()))
            .unzip();
        write!(
            json,
            "{{\"candidate_digest\":\"{digest}\",\"candidate_version\":\"{version}\",\
             \"result\":\"{result}\",\"current_version\":\"{current}\",\
             \"reason\":\"{reason}\"}}}}",
            digest = Blank(digest),
            version = Blank(version),
            result = result_code(record.update().ending()),
            current = record.current().version(),
            reason = self.reason.map_or("", Error::code),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Refusal, image::ImageId};
    use dcc_ota_format::{Digest, ImageLen};

    fn snapshot() -> Snapshot {
        let len = ImageLen::new(9120).unwrap();
        let current = ImageId::new(Slot::A, Version::new(5, 2, 7), len, Digest::new([0x12; 32]));
        let candidate = ImageId::new(Slot::B, Version::new(5, 3, 0), len, Digest::new([0xab; 32]));
        Snapshot {
            record: Some(Record::installed(current).begin(candidate).unwrap()),
            boot_id: BootId::new(0x1234_5678_9abc_def0),
            uptime: Uptime::new(Duration::from_secs(71)),
            version: candidate.version(),
            slot: Slot::B,
            lifecycle: Lifecycle::Trial,
            busy: false,
            track_enabled: false,
            transfer: Transfer {
                received: 9120,
                written: 4096,
                total: 9120,
            },
            stage: Stage::Health,
            reason: None,
            trial_left: Some(Duration::from_millis(42_500)),
        }
    }

    fn parse(snapshot: &Snapshot) -> serde_json::Value {
        serde_json::from_str(&snapshot.json().unwrap()).unwrap()
    }

    #[test]
    fn the_document_correlates_the_candidate_with_the_running_image() {
        let json = parse(&snapshot());
        assert_eq!(json["schema"], 1);
        assert_eq!(json["boot_id"], "123456789abcdef0");
        assert_eq!(json["uptime"], 71);
        assert_eq!(json["version"], "5.3.0");
        assert_eq!(json["slot"], 1);
        assert_eq!(json["pending"], true);
        assert_eq!(json["ota_available"], true);
        assert_eq!(json["uploading"], false);
        assert_eq!(json["received_bytes"], 9120);
        assert_eq!(json["written_bytes"], 4096);
        assert_eq!(json["total_bytes"], 9120);
        assert_eq!(json["stage"], "health");
        assert_eq!(json["track_enabled"], false);
        assert_eq!(json["trial_seconds_left"], 42);
        let last = &json["last_update"];
        assert_eq!(last["candidate_digest"], "ab".repeat(32));
        assert_eq!(last["candidate_version"], "5.3.0");
        assert_eq!(last["current_version"], "5.2.7");
        assert_eq!(last["result"], "none");
        assert_eq!(last["reason"], "");
    }

    #[test]
    fn recovery_is_never_shown_as_available() {
        let mut snapshot = snapshot();
        snapshot.lifecycle = Lifecycle::Recovery;
        snapshot.stage = Stage::Recovery;
        snapshot.reason = Some(Error::Corrupt);
        snapshot.trial_left = None;
        let json = parse(&snapshot);
        assert!(json["trial_seconds_left"].is_null());
        assert_eq!(json["pending"], false);
        assert_eq!(json["ota_available"], false);
        assert_eq!(json["stage"], "recovery");
        assert_eq!(json["last_update"]["reason"], "ota_corrupt");
        snapshot.record = None;
        assert!(parse(&snapshot)["last_update"].is_null());
    }

    #[test]
    fn the_widest_document_fits_with_room_to_spare() {
        let widest = Version::new(u16::MAX, u16::MAX, u16::MAX);
        let image = |slot| ImageId::new(slot, widest, ImageLen::MAX, Digest::new([0xff; 32]));
        let record = Record::installed(image(Slot::A))
            .begin(image(Slot::B))
            .unwrap();
        let snapshot = Snapshot {
            record: Some(record),
            boot_id: BootId::new(u64::MAX),
            uptime: Uptime::new(Duration::from_secs(u64::MAX)),
            version: widest,
            busy: true,
            track_enabled: true,
            transfer: Transfer {
                received: u32::MAX,
                written: u32::MAX,
                total: u32::MAX,
            },
            stage: Stage::Receiving,
            reason: Some(Error::Refused(Refusal::Recovery)),
            trial_left: Some(Duration::MAX),
            ..snapshot()
        };
        let len = snapshot.json().unwrap().len();
        assert!(len <= JSON_CAPACITY * 3 / 4, "{len}");
    }
}
