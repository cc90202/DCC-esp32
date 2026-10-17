//! Framework-independent application policy for track power and status.

use crate::application::StatusModel;
use crate::authority::LeasePermit;
use crate::system_status::{FaultEvent, SystemStatusEvent};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrackPowerRequest {
    Enable,
    Disable,
    EmergencyStop,
}

/// What the controller lease allows a power-on request to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LeaseCheck {
    /// The sender holds no lease.
    None,
    /// The sender holds a lease the authority no longer accepts.
    Expired,
    /// The sender holds a lease the authority accepts.
    Valid(LeasePermit),
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

/// Why a power-on request is turned down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RefuseReason {
    Inhibited,
    NoLease,
    ExpiredLease,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrackPowerVerdict {
    Refuse(RefuseReason),
    Apply(TrackPowerDecision),
}

/// Answer sent to the controller for a refused power-on request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RefusalReply {
    Silent,
    TrackPowerOff,
}

/// Judges a track-power request against every precondition.
///
/// Only power-on can be refused. A missing or expired lease is reported
/// before the update inhibit, so a sender without authority learns nothing
/// about the inhibit state.
#[must_use]
pub(crate) const fn judge_track_power(
    request: TrackPowerRequest,
    lease: LeaseCheck,
    inhibited: bool,
) -> TrackPowerVerdict {
    match request {
        TrackPowerRequest::Enable => match lease {
            LeaseCheck::None => TrackPowerVerdict::Refuse(RefuseReason::NoLease),
            LeaseCheck::Expired => TrackPowerVerdict::Refuse(RefuseReason::ExpiredLease),
            LeaseCheck::Valid(_) if inhibited => TrackPowerVerdict::Refuse(RefuseReason::Inhibited),
            LeaseCheck::Valid(permit) => TrackPowerVerdict::Apply(TrackPowerDecision {
                disable_output_immediately: false,
                fault_event: FaultEvent::NetworkResume(permit),
                feedback: TrackPowerFeedback::None,
            }),
        },
        TrackPowerRequest::Disable => TrackPowerVerdict::Apply(TrackPowerDecision {
            disable_output_immediately: true,
            fault_event: FaultEvent::StopPressed,
            feedback: TrackPowerFeedback::None,
        }),
        TrackPowerRequest::EmergencyStop => TrackPowerVerdict::Apply(TrackPowerDecision {
            disable_output_immediately: true,
            fault_event: FaultEvent::StopPressed,
            feedback: TrackPowerFeedback::Stopped,
        }),
    }
}

#[must_use]
pub(crate) const fn refusal_reply(reason: RefuseReason) -> RefusalReply {
    match reason {
        RefuseReason::NoLease | RefuseReason::ExpiredLease => RefusalReply::Silent,
        RefuseReason::Inhibited => RefusalReply::TrackPowerOff,
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

    const PERMIT: LeasePermit = LeasePermit::new(crate::authority::LeaseEpoch::new(1), u64::MAX);

    fn applied(verdict: TrackPowerVerdict) -> TrackPowerDecision {
        match verdict {
            TrackPowerVerdict::Apply(decision) => decision,
            TrackPowerVerdict::Refuse(reason) => panic!("unexpected refusal {reason:?}"),
        }
    }

    #[test]
    fn valid_lease_power_on_resumes_without_touching_output_directly() {
        assert_eq!(
            judge_track_power(TrackPowerRequest::Enable, LeaseCheck::Valid(PERMIT), false),
            TrackPowerVerdict::Apply(TrackPowerDecision {
                disable_output_immediately: false,
                fault_event: FaultEvent::NetworkResume(PERMIT),
                feedback: TrackPowerFeedback::None,
            })
        );
    }

    #[test]
    fn no_lease_beats_inhibited_and_replies_nothing() {
        let verdict = judge_track_power(TrackPowerRequest::Enable, LeaseCheck::None, true);
        assert_eq!(verdict, TrackPowerVerdict::Refuse(RefuseReason::NoLease));
        assert_eq!(refusal_reply(RefuseReason::NoLease), RefusalReply::Silent);

        let verdict = judge_track_power(TrackPowerRequest::Enable, LeaseCheck::Expired, true);
        assert_eq!(
            verdict,
            TrackPowerVerdict::Refuse(RefuseReason::ExpiredLease)
        );
        assert_eq!(
            refusal_reply(RefuseReason::ExpiredLease),
            RefusalReply::Silent
        );
    }

    #[test]
    fn inhibited_power_on_is_refused_even_with_lease() {
        let verdict = judge_track_power(TrackPowerRequest::Enable, LeaseCheck::Valid(PERMIT), true);
        assert_eq!(verdict, TrackPowerVerdict::Refuse(RefuseReason::Inhibited));
        assert_eq!(
            refusal_reply(RefuseReason::Inhibited),
            RefusalReply::TrackPowerOff
        );
    }

    #[test]
    fn power_on_without_lease_is_refused_even_when_not_inhibited() {
        assert_eq!(
            judge_track_power(TrackPowerRequest::Enable, LeaseCheck::None, false),
            TrackPowerVerdict::Refuse(RefuseReason::NoLease)
        );
        assert_eq!(
            judge_track_power(TrackPowerRequest::Enable, LeaseCheck::Expired, false),
            TrackPowerVerdict::Refuse(RefuseReason::ExpiredLease)
        );
    }

    #[test]
    fn power_off_and_estop_are_never_refused() {
        for request in [TrackPowerRequest::Disable, TrackPowerRequest::EmergencyStop] {
            for lease in [LeaseCheck::None, LeaseCheck::Expired] {
                assert!(matches!(
                    judge_track_power(request, lease, true),
                    TrackPowerVerdict::Apply(_)
                ));
            }
        }
    }

    #[test]
    fn disable_and_estop_verdicts_request_immediate_output_disable() {
        for request in [TrackPowerRequest::Disable, TrackPowerRequest::EmergencyStop] {
            let decision = applied(judge_track_power(request, LeaseCheck::None, false));
            assert!(decision.disable_output_immediately);
            assert_eq!(decision.fault_event, FaultEvent::StopPressed);
        }
    }

    #[test]
    fn estop_keeps_the_stopped_feedback() {
        let estop = applied(judge_track_power(
            TrackPowerRequest::EmergencyStop,
            LeaseCheck::None,
            false,
        ));
        let off = applied(judge_track_power(
            TrackPowerRequest::Disable,
            LeaseCheck::None,
            false,
        ));
        assert_eq!(estop.feedback, TrackPowerFeedback::Stopped);
        assert_eq!(off.feedback, TrackPowerFeedback::None);
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
