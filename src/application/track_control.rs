//! Framework-independent application policy for track power and status.

use crate::application::StatusModel;
use crate::authority::LeasePermit;
use crate::station_status::{FaultCause, FaultEvent, SystemStatusEvent};

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
    /// The answer depends on the state the request finds; see
    /// [`power_on_answer`].
    CurrentState,
    /// `LAN_X_BC_TRACK_POWER_OFF` (§2.7).
    ///
    /// Also the answer to `LAN_X_SET_STOP`: this station cuts the bridge on
    /// a stop, while `LAN_X_BC_STOPPED` would tell the client the track is
    /// still powered.
    TrackPowerOff,
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
                feedback: TrackPowerFeedback::CurrentState,
            }),
        },
        TrackPowerRequest::Disable => TrackPowerVerdict::Apply(TrackPowerDecision {
            disable_output_immediately: true,
            fault_event: FaultEvent::StopPressed,
            feedback: TrackPowerFeedback::TrackPowerOff,
        }),
        TrackPowerRequest::EmergencyStop => TrackPowerVerdict::Apply(TrackPowerDecision {
            disable_output_immediately: true,
            fault_event: FaultEvent::StopPressed,
            feedback: TrackPowerFeedback::TrackPowerOff,
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
    /// `LAN_X_BC_TRACK_SHORT_CIRCUIT` (§2.10).
    ShortCircuit,
    SystemState(TrackStatus),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct StatusBroadcastPlan {
    pub(crate) messages: [Option<StatusBroadcast>; 3],
}

/// Frames sent to the lease owner for a status event.
///
/// A stop and a latched fault both cut the bridge here, so both report the
/// track off; `LAN_X_BC_STOPPED` is never sent, because the Z21 protocol and
/// JMRI read it as "locomotives stopped, track still powered". A short is
/// reported with its own message, as the protocol asks (§2.10).
#[must_use]
pub(crate) const fn plan_status_broadcast(
    event: SystemStatusEvent,
    status: TrackStatus,
) -> StatusBroadcastPlan {
    let snapshot = Some(StatusBroadcast::SystemState(status));
    let messages = match event {
        // JMRI and other X-Bus clients only read `LAN_X_BC_TRACK_POWER_OFF`
        // as power off, so the short message comes on top of it, not instead.
        SystemStatusEvent::FaultLatched(FaultCause::TrackShort) => [
            Some(StatusBroadcast::TrackPower(false)),
            Some(StatusBroadcast::ShortCircuit),
            snapshot,
        ],
        SystemStatusEvent::FaultLatched(_) | SystemStatusEvent::EstopActive => {
            [Some(StatusBroadcast::TrackPower(false)), snapshot, None]
        }
        SystemStatusEvent::FaultCleared | SystemStatusEvent::EstopCleared => {
            [Some(StatusBroadcast::TrackPower(true)), snapshot, None]
        }
        SystemStatusEvent::BootStarted
        | SystemStatusEvent::BootCompleted
        | SystemStatusEvent::WifiConnecting
        | SystemStatusEvent::WifiConnected
        | SystemStatusEvent::WifiDisconnected => [None, None, None],
    };
    StatusBroadcastPlan { messages }
}

/// Frames sent to the lease owner after the status model was rebuilt from
/// the fault manager, when the events that changed it were dropped.
///
/// Any change sends the snapshot; a change of track power and a new short
/// also send their own messages before it, as the events would have.
#[must_use]
pub(crate) const fn plan_resync_broadcast(
    before: TrackStatus,
    after: TrackStatus,
) -> StatusBroadcastPlan {
    let unchanged = before.power_on == after.power_on
        && before.estop_active == after.estop_active
        && before.short_circuit == after.short_circuit;
    if unchanged {
        return StatusBroadcastPlan {
            messages: [None, None, None],
        };
    }
    let power = if after.power_on == before.power_on {
        None
    } else {
        Some(StatusBroadcast::TrackPower(after.power_on))
    };
    let short = if after.short_circuit && !before.short_circuit {
        Some(StatusBroadcast::ShortCircuit)
    } else {
        None
    };
    StatusBroadcastPlan {
        messages: [power, short, Some(StatusBroadcast::SystemState(after))],
    }
}

/// Immediate answer to an accepted `LAN_X_SET_TRACK_POWER_ON` (Z21 LAN
/// protocol §2.6, §2.8): `Some(on)` to send `LAN_X_BC_TRACK_POWER_ON` or
/// `_OFF`, `None` to send nothing yet.
///
/// A track already powered is reported on, and a latched fault, which
/// refuses the request, off. Otherwise an emergency stop is being lifted:
/// the `LAN_X_BC_TRACK_POWER_ON` broadcast sent once the track is really
/// powered answers the lease owner who asked. Nothing waits here, so a stop
/// in the same datagram is read at once.
#[must_use]
pub(crate) const fn power_on_answer(track_enabled: bool, fault_latched: bool) -> Option<bool> {
    if track_enabled {
        Some(true)
    } else if fault_latched {
        Some(false)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
                feedback: TrackPowerFeedback::CurrentState,
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
    fn estop_and_power_off_both_answer_track_off() {
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
        assert_eq!(estop.feedback, TrackPowerFeedback::TrackPowerOff);
        assert_eq!(off.feedback, TrackPowerFeedback::TrackPowerOff);
    }

    #[test]
    fn a_power_on_is_answered_from_the_state_it_finds() {
        assert_eq!(power_on_answer(true, false), Some(true));
        assert_eq!(power_on_answer(false, true), Some(false));
        assert_eq!(
            power_on_answer(false, false),
            None,
            "lifting a stop: the broadcast answers once the track is on"
        );
    }

    /// The frames a plan sends, in order.
    fn sent(plan: StatusBroadcastPlan) -> Vec<StatusBroadcast> {
        plan.messages.into_iter().flatten().collect()
    }

    #[test]
    fn a_resync_reports_every_change() {
        let estop = TrackStatus {
            power_on: false,
            estop_active: true,
            short_circuit: false,
        };
        let short = TrackStatus {
            power_on: false,
            estop_active: false,
            short_circuit: true,
        };
        assert!(sent(plan_resync_broadcast(RUNNING, RUNNING)).is_empty());
        assert_eq!(
            sent(plan_resync_broadcast(RUNNING, estop)),
            [
                StatusBroadcast::TrackPower(false),
                StatusBroadcast::SystemState(estop)
            ]
        );
        assert_eq!(
            sent(plan_resync_broadcast(estop, short)),
            [
                StatusBroadcast::ShortCircuit,
                StatusBroadcast::SystemState(short)
            ]
        );
        // The track stays off, but the short is gone: the snapshot says so.
        assert_eq!(
            sent(plan_resync_broadcast(short, estop)),
            [StatusBroadcast::SystemState(estop)]
        );
        assert_eq!(
            sent(plan_resync_broadcast(estop, RUNNING)),
            [
                StatusBroadcast::TrackPower(true),
                StatusBroadcast::SystemState(RUNNING)
            ]
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
    fn a_short_broadcasts_power_off_the_short_message_and_snapshot() {
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
                Some(StatusBroadcast::ShortCircuit),
                Some(StatusBroadcast::SystemState(status)),
            ]
        );
    }

    #[test]
    fn an_internal_fault_broadcasts_power_off_and_snapshot() {
        let status = TrackStatus {
            power_on: false,
            estop_active: false,
            short_circuit: false,
        };
        assert_eq!(
            plan_status_broadcast(
                SystemStatusEvent::FaultLatched(FaultCause::Internal),
                status
            )
            .messages,
            [
                Some(StatusBroadcast::TrackPower(false)),
                Some(StatusBroadcast::SystemState(status)),
                None,
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
    fn estop_events_broadcast_power_and_snapshot() {
        assert_eq!(
            plan_status_broadcast(SystemStatusEvent::EstopActive, RUNNING).messages,
            [
                Some(StatusBroadcast::TrackPower(false)),
                Some(StatusBroadcast::SystemState(RUNNING)),
                None,
            ]
        );
        assert_eq!(
            plan_status_broadcast(SystemStatusEvent::EstopCleared, RUNNING).messages,
            [
                Some(StatusBroadcast::TrackPower(true)),
                Some(StatusBroadcast::SystemState(RUNNING)),
                None,
            ]
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
