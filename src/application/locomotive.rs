//! Application policy for scheduler-backed locomotive operations.
//!
//! This module contains no Z21 encoding, logging, Embassy channels, or timing.
//! Adapters provide an optional scheduler result and translate the typed outcome
//! to their protocol-specific response.

use crate::application::loco_projection::{
    ProjectionError, apply_loco_result, projected_or_neutral, remove_projected_loco,
};
use crate::application::{LocoSlots, LocoState};
use crate::dcc::{
    DccAddress, Direction, FunctionChange, FunctionIndex, LocoRequest, LocoRequestResult,
    LogicalSpeed, SpeedFormat,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum LocoRequestError {
    TrackPowerOff,
    InvalidSpeed { speed: u8, format: SpeedFormat },
    InvalidFunction { function: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum LocoCommandFailure {
    SchedulerTimeout,
    SchedulerRejected(LocoRequestResult),
    Projection(ProjectionError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LocoCommandOutcome {
    Applied(LocoState),
    Rejected(LocoCommandFailure),
}

impl LocoCommandOutcome {
    #[must_use]
    pub(crate) const fn confirmed_state(self) -> Option<LocoState> {
        match self {
            Self::Applied(state) => Some(state),
            Self::Rejected(_) => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum LocoLookupIssue {
    SchedulerTimeout,
    UnexpectedResult(LocoRequestResult),
    Projection(ProjectionError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LocoLookupOutcome {
    pub(crate) state: LocoState,
    pub(crate) issue: Option<LocoLookupIssue>,
}

pub(crate) fn prepare_drive_request(
    track_power_on: bool,
    address: DccAddress,
    speed: u8,
    direction: Direction,
    format: SpeedFormat,
) -> Result<LocoRequest, LocoRequestError> {
    if !track_power_on {
        return Err(LocoRequestError::TrackPowerOff);
    }
    let logical_speed =
        LogicalSpeed::new(speed, format).ok_or(LocoRequestError::InvalidSpeed { speed, format })?;
    Ok(LocoRequest::SetSpeed {
        address,
        speed: logical_speed,
        direction,
    })
}

pub(crate) fn prepare_function_request(
    track_power_on: bool,
    address: DccAddress,
    function: u8,
    change: FunctionChange,
) -> Result<LocoRequest, LocoRequestError> {
    if !track_power_on {
        return Err(LocoRequestError::TrackPowerOff);
    }
    let function =
        FunctionIndex::new(function).ok_or(LocoRequestError::InvalidFunction { function })?;
    Ok(LocoRequest::SetFunction {
        address,
        function,
        change,
    })
}

pub(crate) fn resolve_loco_command(
    slots: &mut LocoSlots,
    address: DccAddress,
    result: Option<LocoRequestResult>,
) -> LocoCommandOutcome {
    let Some(result) = result else {
        return LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerTimeout);
    };
    if matches!(result, LocoRequestResult::NotFound) {
        remove_projected_loco(slots, address);
    }
    match apply_loco_result(slots, result) {
        Ok(Some(state)) => LocoCommandOutcome::Applied(state),
        Ok(None) => LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerRejected(result)),
        Err(error) => LocoCommandOutcome::Rejected(LocoCommandFailure::Projection(error)),
    }
}

pub(crate) fn resolve_loco_lookup(
    slots: &mut LocoSlots,
    address: DccAddress,
    result: Option<LocoRequestResult>,
) -> LocoLookupOutcome {
    match result {
        Some(LocoRequestResult::Found(snapshot)) => {
            match apply_loco_result(slots, LocoRequestResult::Found(snapshot)) {
                Ok(Some(state)) => LocoLookupOutcome { state, issue: None },
                Ok(None) => LocoLookupOutcome {
                    state: snapshot.into(),
                    issue: Some(LocoLookupIssue::UnexpectedResult(LocoRequestResult::Found(
                        snapshot,
                    ))),
                },
                Err(error) => LocoLookupOutcome {
                    state: snapshot.into(),
                    issue: Some(LocoLookupIssue::Projection(error)),
                },
            }
        }
        Some(LocoRequestResult::NotFound) => {
            remove_projected_loco(slots, address);
            LocoLookupOutcome {
                state: projected_or_neutral(slots, address),
                issue: None,
            }
        }
        Some(result) => LocoLookupOutcome {
            state: projected_or_neutral(slots, address),
            issue: Some(LocoLookupIssue::UnexpectedResult(result)),
        },
        None => LocoLookupOutcome {
            state: projected_or_neutral(slots, address),
            issue: Some(LocoLookupIssue::SchedulerTimeout),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::{LocoSnapshot, LogicalSpeed};

    fn address(value: u8) -> DccAddress {
        DccAddress::new_short(value).unwrap()
    }

    fn snapshot(value: u8, speed: u8) -> LocoSnapshot {
        LocoSnapshot {
            address: address(value),
            speed: LogicalSpeed::new(speed, SpeedFormat::Speed128).unwrap(),
            direction: Direction::Forward,
            functions: 0,
        }
    }

    #[test]
    fn track_power_off_rejects_drive_before_building_scheduler_request() {
        assert_eq!(
            prepare_drive_request(
                false,
                address(3),
                10,
                Direction::Forward,
                SpeedFormat::Speed128,
            ),
            Err(LocoRequestError::TrackPowerOff)
        );
    }

    #[test]
    fn invalid_function_is_rejected_before_building_scheduler_request() {
        assert_eq!(
            prepare_function_request(true, address(3), 29, FunctionChange::Enable,),
            Err(LocoRequestError::InvalidFunction { function: 29 })
        );
    }

    #[test]
    fn timeout_keeps_last_confirmed_projection() {
        let confirmed = LocoState::from(snapshot(3, 12));
        let mut slots = [None; 12];
        slots[0] = Some(confirmed);

        let outcome = resolve_loco_lookup(&mut slots, address(3), None);

        assert_eq!(outcome.state, confirmed);
        assert_eq!(outcome.issue, Some(LocoLookupIssue::SchedulerTimeout));
        assert_eq!(slots[0], Some(confirmed));
    }

    #[test]
    fn rejected_command_does_not_acknowledge_or_mutate_projection() {
        let confirmed = LocoState::from(snapshot(3, 12));
        let mut slots = [None; 12];
        slots[0] = Some(confirmed);

        let outcome =
            resolve_loco_command(&mut slots, address(3), Some(LocoRequestResult::Rejected));

        assert_eq!(
            outcome,
            LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerRejected(
                LocoRequestResult::Rejected
            ))
        );
        assert_eq!(outcome.confirmed_state(), None);
        assert_eq!(slots[0], Some(confirmed));
    }

    #[test]
    fn confirmed_command_updates_projection_and_can_be_acknowledged() {
        let mut slots = [None; 12];
        let updated = snapshot(3, 24);

        let outcome = resolve_loco_command(
            &mut slots,
            address(3),
            Some(LocoRequestResult::Inserted(updated)),
        );

        assert_eq!(outcome.confirmed_state(), Some(updated.into()));
        assert_eq!(slots[0], Some(updated.into()));
    }

    #[test]
    fn confirmed_not_found_removes_stale_projection() {
        let mut slots = [None; 12];
        slots[0] = Some(snapshot(3, 5).into());

        let outcome =
            resolve_loco_command(&mut slots, address(3), Some(LocoRequestResult::NotFound));

        assert!(matches!(outcome, LocoCommandOutcome::Rejected(_)));
        assert!(slots.iter().all(Option::is_none));
    }
}
