//! Framework-independent application policy for track power and status.

use crate::application::StatusModel;
use crate::system_status::{FaultEvent, SystemStatusEvent};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrackPowerRequest {
    Enable,
    Disable,
    EmergencyStop,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrackPowerFeedback {
    None,
    Stopped,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TrackPowerDecision {
    pub(crate) disable_output_immediately: bool,
    pub(crate) fault_event: FaultEvent,
    pub(crate) feedback: TrackPowerFeedback,
}

#[must_use]
pub(crate) const fn decide_track_power(request: TrackPowerRequest) -> TrackPowerDecision {
    match request {
        TrackPowerRequest::Enable => TrackPowerDecision {
            disable_output_immediately: false,
            fault_event: FaultEvent::ResumeShortPressed,
            feedback: TrackPowerFeedback::None,
        },
        TrackPowerRequest::Disable => TrackPowerDecision {
            disable_output_immediately: true,
            fault_event: FaultEvent::StopPressed,
            feedback: TrackPowerFeedback::None,
        },
        TrackPowerRequest::EmergencyStop => TrackPowerDecision {
            disable_output_immediately: true,
            fault_event: FaultEvent::StopPressed,
            feedback: TrackPowerFeedback::Stopped,
        },
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TrackStatus {
    pub(crate) power_on: bool,
    pub(crate) estop_active: bool,
    pub(crate) short_circuit: bool,
}

impl TrackStatus {
    #[must_use]
    pub(crate) const fn from_model(model: StatusModel) -> Self {
        Self {
            power_on: model.track_power_on(),
            estop_active: model.estop_active(),
            short_circuit: model.short_circuit(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StatusBroadcast {
    TrackPower(bool),
    Stopped,
    SystemState(TrackStatus),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct StatusBroadcastPlan {
    pub(crate) messages: [Option<StatusBroadcast>; 3],
}

#[must_use]
pub(crate) const fn plan_status_broadcast(
    event: SystemStatusEvent,
    status: TrackStatus,
) -> StatusBroadcastPlan {
    let messages = match event {
        SystemStatusEvent::FaultLatched(_) => [
            Some(StatusBroadcast::TrackPower(false)),
            Some(StatusBroadcast::Stopped),
            Some(StatusBroadcast::SystemState(status)),
        ],
        SystemStatusEvent::FaultCleared => [
            Some(StatusBroadcast::TrackPower(true)),
            Some(StatusBroadcast::SystemState(status)),
            None,
        ],
        SystemStatusEvent::EstopActive => [Some(StatusBroadcast::Stopped), None, None],
        SystemStatusEvent::EstopCleared => [Some(StatusBroadcast::TrackPower(true)), None, None],
        SystemStatusEvent::BootStarted
        | SystemStatusEvent::BootCompleted
        | SystemStatusEvent::WifiConnecting
        | SystemStatusEvent::WifiConnected
        | SystemStatusEvent::WifiDisconnected => [None, None, None],
    };
    StatusBroadcastPlan { messages }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system_status::FaultCause;

    const RUNNING: TrackStatus = TrackStatus {
        power_on: true,
        estop_active: false,
        short_circuit: false,
    };

    #[test]
    fn enable_requests_resume_without_touching_output_directly() {
        assert_eq!(
            decide_track_power(TrackPowerRequest::Enable),
            TrackPowerDecision {
                disable_output_immediately: false,
                fault_event: FaultEvent::ResumeShortPressed,
                feedback: TrackPowerFeedback::None,
            }
        );
    }

    #[test]
    fn disable_cuts_output_before_latching_stop() {
        assert_eq!(
            decide_track_power(TrackPowerRequest::Disable),
            TrackPowerDecision {
                disable_output_immediately: true,
                fault_event: FaultEvent::StopPressed,
                feedback: TrackPowerFeedback::None,
            }
        );
    }

    #[test]
    fn emergency_stop_adds_stopped_feedback() {
        assert_eq!(
            decide_track_power(TrackPowerRequest::EmergencyStop).feedback,
            TrackPowerFeedback::Stopped
        );
    }

    #[test]
    fn track_status_is_projected_from_status_model() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::FaultLatched(FaultCause::TrackShort));
        assert_eq!(
            TrackStatus::from_model(model),
            TrackStatus {
                power_on: false,
                estop_active: false,
                short_circuit: true,
            }
        );
    }

    #[test]
    fn fault_latched_broadcasts_power_off_stop_and_snapshot() {
        let status = TrackStatus {
            power_on: false,
            estop_active: false,
            short_circuit: true,
        };
        assert_eq!(
            plan_status_broadcast(
                SystemStatusEvent::FaultLatched(FaultCause::TrackShort),
                status
            )
            .messages,
            [
                Some(StatusBroadcast::TrackPower(false)),
                Some(StatusBroadcast::Stopped),
                Some(StatusBroadcast::SystemState(status)),
            ]
        );
    }

    #[test]
    fn fault_clear_broadcasts_power_on_and_snapshot() {
        assert_eq!(
            plan_status_broadcast(SystemStatusEvent::FaultCleared, RUNNING).messages,
            [
                Some(StatusBroadcast::TrackPower(true)),
                Some(StatusBroadcast::SystemState(RUNNING)),
                None,
            ]
        );
    }

    #[test]
    fn estop_events_have_minimal_broadcasts() {
        assert_eq!(
            plan_status_broadcast(SystemStatusEvent::EstopActive, RUNNING).messages,
            [Some(StatusBroadcast::Stopped), None, None]
        );
        assert_eq!(
            plan_status_broadcast(SystemStatusEvent::EstopCleared, RUNNING).messages,
            [Some(StatusBroadcast::TrackPower(true)), None, None]
        );
    }

    #[test]
    fn unrelated_status_events_do_not_emit_track_messages() {
        assert_eq!(
            plan_status_broadcast(SystemStatusEvent::WifiConnected, RUNNING).messages,
            [None, None, None]
        );
    }
}
