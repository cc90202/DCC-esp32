//! Boot decisions use identities recorded by the signed installation path.
//! The journal CRC detects torn writes; it does not authenticate the journal.
use super::{
    Error, ImageId,
    journal::{Outcome, Phase, Record},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Run(Record),
    Trial(Record),
    Restore(Record),
}

/// `booted` is independently hashed; `current_ok` describes the recorded image
/// in flash, not the bootloader's selection or state word.
pub fn decide(mut r: Record, booted: ImageId, current_ok: bool) -> Result<Action, Error> {
    r.current.validate()?;
    if r.previous.is_some_and(|p| p.slot == r.current.slot) {
        return Err(Error::Corrupt);
    }
    match r.phase {
        Phase::Confirmed | Phase::RolledBack => {
            if r.current == booted {
                return Ok(Action::Run(r));
            }
            if r.previous == Some(booted) {
                r.candidate = Some(r.current);
                r.reject_candidate();
                r.current = booted;
                r.previous = None;
                r.phase = Phase::RolledBack;
                r.outcome = Outcome::NotBootable;
                r.hold_track_off = true;
                return Ok(Action::Restore(r));
            }
            if current_ok {
                return Ok(Action::Restore(r));
            }
            Err(Error::Corrupt)
        }
        Phase::Receiving | Phase::Ready | Phase::Trying => {
            let candidate = r.candidate.ok_or(Error::Corrupt)?;
            if candidate.slot == r.current.slot || r.previous.is_some() {
                return Err(Error::Corrupt);
            }
            r.hold_track_off = true;
            if r.phase == Phase::Ready && candidate == booted && current_ok {
                r.phase = Phase::Trying;
                return Ok(Action::Trial(r));
            }
            if !current_ok {
                return Err(Error::Corrupt);
            }
            if r.phase == Phase::Trying {
                r.reject_candidate();
                r.outcome = Outcome::RolledBack;
                r.phase = Phase::RolledBack;
            } else {
                r = r.abort(Outcome::Aborted)?;
            }
            Ok(Action::Restore(r))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::Version;
    fn id(slot: u8, v: u16) -> ImageId {
        ImageId {
            slot,
            version: Version([1, v, 0]),
            len: 8192,
            digest: [v as u8; 32],
        }
    }
    fn record(phase: Phase) -> Record {
        Record {
            generation: 1,
            phase,
            current: id(0, 0),
            candidate: Some(id(1, 1)),
            previous: None,
            rejected: None,
            outcome: Outcome::None,
            hold_track_off: true,
        }
    }
    #[test]
    fn only_ready_exact_candidate_can_start_one_trial() {
        let old = id(0, 0);
        let new = id(1, 1);
        let unknown = id(1, 2);
        for phase in [Phase::Receiving, Phase::Ready, Phase::Trying] {
            for booted in [old, new, unknown] {
                let action = decide(record(phase), booted, true).unwrap();
                if phase == Phase::Ready && booted == new {
                    let Action::Trial(r) = action else {
                        panic!("missing trial")
                    };
                    assert_eq!(r.phase, Phase::Trying);
                    assert!(matches!(
                        decide(r, new, true),
                        Ok(Action::Restore(Record {
                            phase: Phase::RolledBack,
                            ..
                        }))
                    ));
                } else {
                    let Action::Restore(r) = action else {
                        panic!("untrusted boot")
                    };
                    assert_eq!(r.current, old);
                    assert!(r.hold_track_off);
                    assert_eq!(
                        r.rejected_version(),
                        (phase == Phase::Trying).then_some(new.version)
                    );
                }
            }
            assert_eq!(decide(record(phase), new, false), Err(Error::Corrupt));
        }
    }
    #[test]
    fn fallback_must_match_full_confirmed_identity_not_slot_or_version() {
        let mut r = record(Phase::Confirmed);
        r.current = id(1, 1);
        r.candidate = Some(r.current);
        r.previous = Some(id(0, 0));
        let Action::Restore(restored) = decide(r, id(0, 0), false).unwrap() else {
            panic!("missing fallback")
        };
        assert_eq!(restored.current, id(0, 0));
        assert_eq!(restored.rejected_version(), Some(id(1, 1).version));
        let mut changed = id(0, 0);
        changed.digest[0] ^= 1;
        assert_eq!(decide(r, changed, false), Err(Error::Corrupt));
        assert_eq!(decide(r, id(1, 1), true), Ok(Action::Run(r)));
        assert!(matches!(decide(r, changed, true), Ok(Action::Restore(_))));
    }

    #[test]
    fn boot_abort_preserves_previous_failed_release() {
        let Action::Restore(rejected) = decide(record(Phase::Trying), id(0, 0), true).unwrap()
        else {
            panic!("missing rollback")
        };
        let receiving = rejected.begin(id(1, 2)).unwrap();
        let Action::Restore(aborted) = decide(receiving, id(0, 0), true).unwrap() else {
            panic!("missing abort")
        };
        assert_eq!(aborted.phase, Phase::Confirmed);
        assert_eq!(aborted.rejected_version(), Some(id(1, 1).version));
        assert_eq!(aborted.candidate, Some(id(1, 2)));
    }
}
