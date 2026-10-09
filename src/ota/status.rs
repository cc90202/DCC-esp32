//! Host-testable OTA status projection; no HAL, flash or state transitions.
use core::fmt::Write;
use core::time::Duration;

use super::{
    Error, Version,
    journal::{Outcome, Record},
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    Boot,
    Idle,
    Receiving,
    Verifying,
    Ready,
    Health,
    Recovery,
}

impl Stage {
    fn code(self) -> &'static str {
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

pub struct Snapshot {
    pub record: Option<Record>,
    pub boot_id: u64,
    pub uptime: Duration,
    pub version: Version,
    pub slot: u8,
    pub pending: bool,
    pub recovery: bool,
    pub busy: bool,
    pub track_enabled: bool,
    pub received: u32,
    pub written: u32,
    pub total: u32,
    pub stage: Stage,
    pub reason: Option<Error>,
}

impl Snapshot {
    pub fn json(&self) -> Result<heapless::String<1536>, Error> {
        let mut json = heapless::String::new();
        // Values are internal enums, canonical versions or hex, never client input.
        write!(
        json,
        "{{\"schema\":1,\"boot_id\":\"{:016x}\",\"uptime\":{},\"version\":\"{}\",\"slot\":{},\"pending\":{},\"ota_available\":{},\"uploading\":{},\"received_bytes\":{},\"written_bytes\":{},\"total_bytes\":{},\"fase\":\"{}\",\"track_enabled\":{},\"last_update\":",
        self.boot_id,
        self.uptime.as_secs(),
        self.version,
        self.slot,
        self.pending,
        !self.recovery && self.record.is_some(),
        self.busy,
        self.received,
        self.written,
        self.total,
        self.stage.code(),
        self.track_enabled
    ).map_err(|_| Error::Corrupt)?;
        if let Some(r) = self.record {
            let result = match r.outcome {
                Outcome::None => "none",
                Outcome::Ok => "ok",
                Outcome::Aborted => "aborted",
                Outcome::RolledBack => "rolled_back",
                Outcome::NotBootable => "not_bootable",
            };
            write!(json, "{{\"candidate_digest\":\"").map_err(|_| Error::Corrupt)?;
            if let Some(id) = r.candidate {
                for b in id.digest {
                    write!(json, "{b:02x}").map_err(|_| Error::Corrupt)?;
                }
            }
            write!(
            json,
            "\",\"candidate_version\":\"{}\",\"result\":\"{}\",\"current_version\":\"{}\",\"reason\":\"{}\"}}",
            r.candidate.map(|c| c.version).unwrap_or_default(),
            result,
            r.current.version,
            if self.reason.is_none() && r.outcome == Outcome::RolledBack {
                "pending_verify"
            } else {
                self.reason.map_or("", Error::code)
            }
        ).map_err(|_| Error::Corrupt)?;
        } else {
            json.push_str("null").map_err(|_| Error::Corrupt)?;
        }
        json.push('}').map_err(|_| Error::Corrupt)?;
        Ok(json)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::{ImageId, journal::Phase};

    #[test]
    fn status_correlates_candidate_and_keeps_recovery_distinct_from_success() {
        let current = ImageId {
            slot: 0,
            version: Version([5, 2, 7]),
            len: 9120,
            digest: [0x12; 32],
        };
        let candidate = ImageId {
            slot: 1,
            version: Version([5, 3, 0]),
            digest: [0xab; 32],
            ..current
        };
        let mut snapshot = Snapshot {
            record: Some(Record {
                generation: 17,
                phase: Phase::Trying,
                current,
                candidate: Some(candidate),
                previous: None,
                rejected: None,
                outcome: Outcome::None,
                hold_track_off: true,
            }),
            boot_id: 0x123456789abcdef0,
            uptime: Duration::from_secs(71),
            version: candidate.version,
            slot: 1,
            pending: true,
            recovery: false,
            busy: false,
            track_enabled: false,
            received: 9120,
            written: 4096,
            total: 9120,
            stage: Stage::Health,
            reason: None,
        };
        let json: serde_json::Value = serde_json::from_str(&snapshot.json().unwrap()).unwrap();
        assert_eq!(json["schema"], 1);
        assert_eq!(json["boot_id"], "123456789abcdef0");
        assert_eq!(json["uptime"], 71);
        assert_eq!(json["version"], "5.3.0");
        assert_eq!(json["slot"], 1);
        assert_eq!(json["pending"], true);
        assert_eq!(json["ota_available"], true);
        assert_eq!(json["track_enabled"], false);
        assert_eq!(json["uploading"], false);
        assert_eq!(json["received_bytes"], 9120);
        assert_eq!(json["written_bytes"], 4096);
        assert_eq!(json["total_bytes"], 9120);
        assert_eq!(json["fase"], "health");
        assert_eq!(json["last_update"]["candidate_digest"], "ab".repeat(32));
        assert_eq!(json["last_update"]["candidate_version"], "5.3.0");
        assert_eq!(json["last_update"]["current_version"], "5.2.7");
        assert_eq!(json["last_update"]["result"], "none");
        assert_eq!(json["last_update"]["reason"], "");
        for (outcome, result, reason) in [
            (Outcome::Aborted, "aborted", ""),
            (Outcome::NotBootable, "not_bootable", ""),
            (Outcome::RolledBack, "rolled_back", "pending_verify"),
            (Outcome::Ok, "ok", ""),
        ] {
            snapshot.record.as_mut().unwrap().outcome = outcome;
            let json: serde_json::Value = serde_json::from_str(&snapshot.json().unwrap()).unwrap();
            assert_eq!(json["last_update"]["result"], result);
            assert_eq!(json["last_update"]["reason"], reason);
        }
        snapshot.pending = false;
        snapshot.recovery = true;
        snapshot.stage = Stage::Recovery;
        snapshot.reason = Some(Error::Flash);
        let json: serde_json::Value = serde_json::from_str(&snapshot.json().unwrap()).unwrap();
        assert_eq!(json["pending"], false);
        assert_eq!(json["ota_available"], false);
        assert_eq!(json["fase"], "recovery");
        assert_eq!(json["last_update"]["reason"], "flash_error");
        snapshot.record = None;
        let json: serde_json::Value = serde_json::from_str(&snapshot.json().unwrap()).unwrap();
        assert!(json["last_update"].is_null());
        for (stage, label) in [
            (Stage::Boot, "boot"),
            (Stage::Idle, "idle"),
            (Stage::Receiving, "receiving"),
            (Stage::Verifying, "verifying"),
            (Stage::Ready, "ready"),
            (Stage::Health, "health"),
            (Stage::Recovery, "recovery"),
        ] {
            snapshot.stage = stage;
            let json: serde_json::Value = serde_json::from_str(&snapshot.json().unwrap()).unwrap();
            assert_eq!(json["fase"], label);
        }
    }
}
