//! Read-only network projection of scheduler-owned locomotive state.

use crate::application::{LocoSlots, LocoState};
use crate::dcc::{
    DccAddress, Direction, LocoRequestResult, LocoSnapshot, LogicalSpeed, SpeedFormat,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum ProjectionError {
    FullWithoutAuthoritativeReplacement,
}

/// Apply only state explicitly confirmed by the scheduler.
///
/// Rejections and lookup misses intentionally leave the projection untouched.
pub(crate) fn apply_loco_result(
    slots: &mut LocoSlots,
    result: LocoRequestResult,
) -> Result<Option<LocoState>, ProjectionError> {
    match result {
        LocoRequestResult::Found(snapshot)
        | LocoRequestResult::Inserted(snapshot)
        | LocoRequestResult::Updated(snapshot) => apply_snapshot(slots, snapshot, None).map(Some),
        LocoRequestResult::Replaced { removed, inserted } => {
            apply_snapshot(slots, inserted, Some(removed)).map(Some)
        }
        LocoRequestResult::Full
        | LocoRequestResult::NotFound
        | LocoRequestResult::Rejected
        | LocoRequestResult::Expired => Ok(None),
    }
}

#[must_use]
pub(crate) fn projected_or_neutral(slots: &LocoSlots, address: DccAddress) -> LocoState {
    slots
        .iter()
        .flatten()
        .find(|state| state.address == address)
        .copied()
        .unwrap_or(LocoState {
            address,
            speed: LogicalSpeed::zero(SpeedFormat::Speed128),
            direction: Direction::Forward,
            functions: 0,
        })
}

pub(crate) fn remove_projected_loco(slots: &mut LocoSlots, address: DccAddress) {
    if let Some(index) = projected_slot_index(slots, address) {
        slots[index] = None;
    }
}

fn apply_snapshot(
    slots: &mut LocoSlots,
    snapshot: LocoSnapshot,
    removed: Option<DccAddress>,
) -> Result<LocoState, ProjectionError> {
    let state = LocoState::from(snapshot);

    if let Some(index) = projected_slot_index(slots, snapshot.address) {
        slots[index] = Some(state);
        if let Some(removed) = removed {
            clear_other_address(slots, removed, index);
        }
        return Ok(state);
    }

    let target = removed
        .and_then(|address| projected_slot_index(slots, address))
        .or_else(|| slots.iter().position(Option::is_none))
        .ok_or(ProjectionError::FullWithoutAuthoritativeReplacement)?;

    slots[target] = Some(state);
    Ok(state)
}

fn projected_slot_index(slots: &LocoSlots, address: DccAddress) -> Option<usize> {
    slots
        .iter()
        .position(|slot| slot.as_ref().is_some_and(|slot| slot.address == address))
}

fn clear_other_address(slots: &mut LocoSlots, address: DccAddress, retained_index: usize) {
    if let Some(index) = slots.iter().enumerate().position(|(index, slot)| {
        index != retained_index && slot.as_ref().is_some_and(|slot| slot.address == address)
    }) {
        slots[index] = None;
    }
}

impl From<LocoSnapshot> for LocoState {
    fn from(snapshot: LocoSnapshot) -> Self {
        Self {
            address: snapshot.address,
            speed: snapshot.speed,
            direction: snapshot.direction,
            functions: snapshot.functions,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::{Direction, LogicalSpeed, SpeedFormat};

    fn address(value: u8) -> DccAddress {
        DccAddress::new_short(value).unwrap()
    }

    fn snapshot(value: u8, speed: u8) -> LocoSnapshot {
        LocoSnapshot {
            address: address(value),
            speed: LogicalSpeed::new(speed, SpeedFormat::Speed128).unwrap(),
            direction: Direction::Forward,
            functions: u32::from(value),
        }
    }

    #[test]
    fn rejected_results_leave_projection_bit_identical() {
        let mut slots = [None; 12];
        for (index, slot) in slots.iter_mut().enumerate() {
            *slot = Some(LocoState::from(snapshot(index as u8 + 1, 5)));
        }
        let before = slots;

        for result in [
            LocoRequestResult::Full,
            LocoRequestResult::NotFound,
            LocoRequestResult::Rejected,
            LocoRequestResult::Expired,
        ] {
            assert_eq!(apply_loco_result(&mut slots, result), Ok(None));
            assert_eq!(slots, before);
        }
    }

    #[test]
    fn confirmed_replacement_overwrites_only_removed_address() {
        let mut slots = [None; 12];
        for (index, slot) in slots.iter_mut().enumerate() {
            *slot = Some(LocoState::from(snapshot(index as u8 + 1, 0)));
        }

        let inserted = snapshot(13, 7);
        assert_eq!(
            apply_loco_result(
                &mut slots,
                LocoRequestResult::Replaced {
                    removed: address(4),
                    inserted,
                },
            ),
            Ok(Some(LocoState::from(inserted)))
        );
        assert!(
            slots
                .iter()
                .flatten()
                .all(|slot| slot.address != address(4))
        );
        assert_eq!(
            slots
                .iter()
                .flatten()
                .find(|slot| slot.address == address(13)),
            Some(&LocoState::from(inserted))
        );
    }

    #[test]
    fn full_projection_without_confirmed_removed_address_is_not_guessed() {
        let mut slots = [None; 12];
        for (index, slot) in slots.iter_mut().enumerate() {
            *slot = Some(LocoState::from(snapshot(index as u8 + 1, 0)));
        }
        let before = slots;

        assert_eq!(
            apply_loco_result(&mut slots, LocoRequestResult::Inserted(snapshot(13, 0)),),
            Err(ProjectionError::FullWithoutAuthoritativeReplacement)
        );
        assert_eq!(slots, before);
    }

    #[test]
    fn timeout_fallback_prefers_last_confirmed_state() {
        let confirmed = LocoState::from(snapshot(3, 18));
        let mut slots = [None; 12];
        slots[0] = Some(confirmed);

        assert_eq!(projected_or_neutral(&slots, address(3)), confirmed);
        assert!(projected_or_neutral(&slots, address(4)).speed.is_zero());
    }

    #[test]
    fn confirmed_not_found_can_remove_stale_projection() {
        let mut slots = [None; 12];
        slots[0] = Some(LocoState::from(snapshot(3, 5)));

        remove_projected_loco(&mut slots, address(3));

        assert!(slots.iter().all(Option::is_none));
    }
}
