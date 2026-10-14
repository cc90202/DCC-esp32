//! Framework-independent Programming on Main application policy.

use crate::application::LocoSlots;
use crate::application::loco_projection::projected_slot_index;
use crate::dcc::{DccAddress, LocoRequest, PomCv, SpeedFormat};

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

fn projected_format(slots: &LocoSlots, address: DccAddress) -> SpeedFormat {
    projected_slot_index(slots, address)
        .and_then(|index| slots[index])
        .map_or(SpeedFormat::Speed128, |slot| slot.speed.format())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::application::LocoState;
    use crate::dcc::{Direction, FunctionState, LogicalSpeed};

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

    #[test]
    fn invalid_cv_is_rejected_before_track_power_policy() {
        assert_eq!(
            prepare_write(false, address(), 0, 7),
            Err(PomAdmissionError::InvalidCv(0))
        );
    }

    #[test]
    fn write_is_admitted_only_while_pom_is_allowed() {
        let cv = PomCv::new(29).unwrap();
        assert_eq!(
            prepare_write(true, address(), 29, 7),
            Ok(PomWritePlan {
                address: address(),
                cv,
                value: 7,
            })
        );
        assert_eq!(
            prepare_write(false, address(), 29, 7),
            Err(PomAdmissionError::TrackPowerOff)
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
}
