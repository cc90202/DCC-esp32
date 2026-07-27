//! Framework-independent Programming on Main application policy.

use crate::application::LocoSlots;
use crate::application::loco_projection::{ProjectionError, apply_loco_result};
use crate::dcc::{DccAddress, LocoRequest, LocoRequestResult, PomCv, SpeedFormat};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum PomAdmissionError {
    InvalidCv(u16),
    TrackPowerOff,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomReadPlan {
    pub(crate) address: DccAddress,
    pub(crate) cv: PomCv,
    pub(crate) refresh_request: LocoRequest,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomWritePlan {
    pub(crate) address: DccAddress,
    pub(crate) cv: PomCv,
    pub(crate) value: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum PomRefreshFailure {
    SchedulerTimeout,
    SchedulerRejected(LocoRequestResult),
    Projection(ProjectionError),
}

pub(crate) fn prepare_read(
    slots: &LocoSlots,
    pom_allowed: bool,
    address: DccAddress,
    raw_cv: u16,
) -> Result<PomReadPlan, PomAdmissionError> {
    let cv = PomCv::new(raw_cv).ok_or(PomAdmissionError::InvalidCv(raw_cv))?;
    if !pom_allowed {
        return Err(PomAdmissionError::TrackPowerOff);
    }
    let format = projected_format(slots, address);
    Ok(PomReadPlan {
        address,
        cv,
        refresh_request: LocoRequest::EnsureRefresh { address, format },
    })
}

pub(crate) fn prepare_write(
    pom_allowed: bool,
    address: DccAddress,
    raw_cv: u16,
    value: u8,
) -> Result<PomWritePlan, PomAdmissionError> {
    let cv = PomCv::new(raw_cv).ok_or(PomAdmissionError::InvalidCv(raw_cv))?;
    if !pom_allowed {
        return Err(PomAdmissionError::TrackPowerOff);
    }
    Ok(PomWritePlan { address, cv, value })
}

pub(crate) fn apply_refresh_result(
    slots: &mut LocoSlots,
    result: Option<LocoRequestResult>,
) -> Result<(), PomRefreshFailure> {
    let Some(result) = result else {
        return Err(PomRefreshFailure::SchedulerTimeout);
    };
    match apply_loco_result(slots, result) {
        Ok(Some(_)) => Ok(()),
        Ok(None) => Err(PomRefreshFailure::SchedulerRejected(result)),
        Err(error) => Err(PomRefreshFailure::Projection(error)),
    }
}

fn projected_format(slots: &LocoSlots, address: DccAddress) -> SpeedFormat {
    slots
        .iter()
        .flatten()
        .find(|slot| slot.address == address)
        .map_or(SpeedFormat::Speed128, |slot| slot.speed.format())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::application::LocoState;
    use crate::dcc::{Direction, FunctionState, LocoSnapshot, LogicalSpeed};

    fn address() -> DccAddress {
        DccAddress::new_short(3).unwrap()
    }

    fn state(format: SpeedFormat) -> LocoState {
        LocoState {
            address: address(),
            speed: LogicalSpeed::zero(format),
            direction: Direction::Forward,
            functions: FunctionState::empty(),
        }
    }

    fn snapshot() -> LocoSnapshot {
        LocoSnapshot {
            address: address(),
            speed: LogicalSpeed::zero(SpeedFormat::Speed128),
            direction: Direction::Forward,
            functions: FunctionState::empty(),
        }
    }

    #[test]
    fn invalid_cv_is_rejected_before_track_power_policy() {
        assert_eq!(
            prepare_write(false, address(), 0, 7),
            Err(PomAdmissionError::InvalidCv(0))
        );
    }

    #[test]
    fn read_plan_uses_confirmed_loco_format() {
        let mut slots = [None; 12];
        slots[0] = Some(state(SpeedFormat::Speed28));

        let plan = prepare_read(&slots, true, address(), 17).unwrap();

        assert_eq!(
            plan.refresh_request,
            LocoRequest::EnsureRefresh {
                address: address(),
                format: SpeedFormat::Speed28,
            }
        );
    }

    #[test]
    fn unknown_loco_defaults_read_refresh_to_speed128() {
        let plan = prepare_read(&[None; 12], true, address(), 17).unwrap();
        assert_eq!(
            plan.refresh_request,
            LocoRequest::EnsureRefresh {
                address: address(),
                format: SpeedFormat::Speed128,
            }
        );
    }

    #[test]
    fn refresh_timeout_does_not_mutate_projection() {
        let confirmed = state(SpeedFormat::Speed28);
        let mut slots = [None; 12];
        slots[0] = Some(confirmed);

        assert_eq!(
            apply_refresh_result(&mut slots, None),
            Err(PomRefreshFailure::SchedulerTimeout)
        );
        assert_eq!(slots[0], Some(confirmed));
    }

    #[test]
    fn confirmed_refresh_updates_projection() {
        let mut slots = [None; 12];
        assert_eq!(
            apply_refresh_result(&mut slots, Some(LocoRequestResult::Inserted(snapshot()))),
            Ok(())
        );
        assert_eq!(slots[0], Some(snapshot().into()));
    }
}
