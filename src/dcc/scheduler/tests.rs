use super::railcom_policy::{
    PacketClass, RAILCOM_MIN_PACKET_GAP, RailcomCutoutBudget, reset_railcom_scheduler_stats,
};
use super::slot_manager::{
    DIRTY_FUNCTION_RETRY_COUNT, DIRTY_STOP_RETRY_COUNT, MAX_FUNCTION_REFRESH_MS,
    allowed_slot_visits_without_function, scheduler_tick_ms_for_slot_count,
};
use super::*;
use crate::dcc::packet::PomCv;

fn addr(n: u8) -> DccAddress {
    DccAddress::new_short(n).unwrap()
}

fn pom_cv(cv: u16) -> PomCv {
    PomCv::new(cv).expect("test POM CV must be valid")
}

fn ls(value: u8, format: SpeedFormat) -> LogicalSpeed {
    LogicalSpeed::new(value, format).expect("test speed value must be valid for format")
}

#[test]
fn test_new_slot_defaults() {
    let mut mgr = SlotManager::new();
    assert!(mgr.is_empty());
    assert_eq!(mgr.slot_count(), 0);

    let _ = mgr.set_speed(addr(3), ls(0, SpeedFormat::Speed28), Direction::Forward);
    assert_eq!(mgr.slot_count(), 1);
}

#[test]
fn test_set_speed_creates_or_updates() {
    let mut mgr = SlotManager::new();

    assert!(mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward));
    assert_eq!(mgr.slot_count(), 1);

    assert!(mgr.set_speed(addr(3), ls(20, SpeedFormat::Speed28), Direction::Reverse));
    assert_eq!(mgr.slot_count(), 1);

    assert!(mgr.set_speed(addr(5), ls(15, SpeedFormat::Speed28), Direction::Forward));
    assert_eq!(mgr.slot_count(), 2);
}

#[test]
fn test_set_speed_rejects_out_of_range() {
    // Range validation now lives solely in `LogicalSpeed::new` — an
    // out-of-range value never becomes a `LogicalSpeed`, so it can no longer
    // reach `SlotManager` at all. This replaces the old
    // `SlotManager::set_speed_with_format` rejection path.
    assert!(LogicalSpeed::new(30, SpeedFormat::Speed28).is_none());
    assert!(LogicalSpeed::new(128, SpeedFormat::Speed128).is_none());

    let mgr = SlotManager::new();
    assert_eq!(mgr.slot_count(), 0);
}

#[test]
fn test_speed28_step_one_maps_to_nmra_step_one_not_estop() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_speed_with_format(
        addr(3),
        ls(1, SpeedFormat::Speed28),
        Direction::Forward,
        SpeedFormat::Speed28
    ));

    let packet = mgr.build_next_packet().expect("expected speed packet");
    let DccPacket::Speed28 {
        address,
        direction,
        speed,
    } = packet
    else {
        panic!("expected Speed28, got {packet:?}");
    };

    assert_eq!(address, addr(3));
    assert_eq!(direction, Direction::Forward);
    assert_eq!(
        speed.value(),
        2,
        "logical step 1 must map to NMRA packet step 1"
    );
}

#[test]
fn test_speed28_logical_max_maps_to_nmra_max() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_speed_with_format(
        addr(3),
        ls(28, SpeedFormat::Speed28),
        Direction::Forward,
        SpeedFormat::Speed28
    ));

    let packet = mgr.build_next_packet().expect("expected speed packet");
    let DccPacket::Speed28 { speed, .. } = packet else {
        panic!("expected Speed28, got {packet:?}");
    };

    assert_eq!(
        speed.value(),
        29,
        "logical step 28 must map to NMRA packet max step"
    );
}

#[test]
fn test_is_safety_critical_packet_detection() {
    assert!(is_safety_critical_packet(&DccPacket::BroadcastStop));
    assert!(is_safety_critical_packet(&DccPacket::EmergencyStop {
        address: addr(3),
        direction: Direction::Forward,
    }));
    assert!(!is_safety_critical_packet(&DccPacket::Idle));
}

#[test]
fn test_safety_send_timeout_streak_transitions() {
    let mut streak = 0;
    streak = advance_safety_send_timeout_streak(streak, true);
    assert_eq!(streak, 1);
    streak = advance_safety_send_timeout_streak(streak, true);
    assert_eq!(streak, 2);
    streak = advance_safety_send_timeout_streak(streak, false);
    assert_eq!(streak, 0);
}

#[test]
fn test_capacity_limit() {
    let mut mgr = SlotManager::new();
    for i in 1..=12 {
        assert!(mgr.set_speed(addr(i), ls(0, SpeedFormat::Speed28), Direction::Forward));
    }
    assert_eq!(mgr.slot_count(), 12);

    assert!(!mgr.set_speed(addr(13), ls(0, SpeedFormat::Speed28), Direction::Forward));
    assert_eq!(mgr.slot_count(), 12);
}

#[test]
fn test_emergency_stop_all() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(2), ls(15, SpeedFormat::Speed28), Direction::Reverse);

    mgr.build_next_packet();
    mgr.build_next_packet();

    mgr.emergency_stop_all();

    // Global e-stop must preempt queue and emit BroadcastStop first.
    let p0 = mgr.build_next_packet().unwrap();
    assert!(matches!(p0, DccPacket::BroadcastStop));

    let p1 = mgr.build_next_packet().unwrap();
    let p2 = mgr.build_next_packet().unwrap();

    let DccPacket::Speed28 { speed, .. } = p1 else {
        panic!("expected Speed28, got {p1:?}");
    };
    assert_eq!(speed.value(), 0);

    let DccPacket::Speed28 { speed, .. } = p2 else {
        panic!("expected Speed28, got {p2:?}");
    };
    assert_eq!(speed.value(), 0);
}

#[test]
fn test_emergency_stop_single() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(2), ls(15, SpeedFormat::Speed28), Direction::Forward);

    mgr.build_next_packet();
    mgr.build_next_packet();

    assert!(mgr.emergency_stop(addr(1)));

    let p = mgr.build_next_packet().unwrap();
    let DccPacket::EmergencyStop { address, .. } = p else {
        panic!("expected EmergencyStop, got {p:?}");
    };
    assert_eq!(address, addr(1));
}

#[test]
fn test_build_next_packet_empty() {
    let mut mgr = SlotManager::new();
    assert!(mgr.build_next_packet().is_none());
}

#[test]
fn test_dirty_priority() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(10, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(2), ls(20, SpeedFormat::Speed28), Direction::Forward);

    for _ in 0..2 {
        let _ = mgr.build_next_packet();
    }

    let _ = mgr.set_speed(addr(2), ls(25, SpeedFormat::Speed28), Direction::Forward);

    let p = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 { address, speed, .. } = p else {
        panic!("expected Speed28, got {p:?}");
    };
    assert_eq!(address, addr(2));
    assert_eq!(speed.value(), 26);
}

#[test]
fn test_round_robin_fairness() {
    let mut mgr = SlotManager::new();
    let n = 4;
    for i in 1..=(n as u8) {
        let _ = mgr.set_speed(addr(i), ls(i * 5, SpeedFormat::Speed28), Direction::Forward);
    }

    for _ in 0..n {
        let _ = mgr.build_next_packet();
    }

    let mut seen = [0u8; 4];
    for _ in 0..n {
        let p = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 { address, .. } = p else {
            panic!("expected Speed28, got {p:?}");
        };
        let idx = (address.value() - 1) as usize;
        seen[idx] += 1;
    }

    for count in &seen {
        assert_eq!(
            *count, 1,
            "each slot should be visited exactly once per cycle"
        );
    }
}

#[test]
fn test_remove_slot() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(10, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(2), ls(20, SpeedFormat::Speed28), Direction::Forward);

    assert!(mgr.remove_slot(addr(1)));
    assert_eq!(mgr.slot_count(), 1);
    assert!(!mgr.remove_slot(addr(1)));

    mgr.build_next_packet();
    let p = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 { address, .. } = p else {
        panic!("expected Speed28, got {p:?}");
    };
    assert_eq!(address, addr(2));
}

#[test]
fn test_speed128_format() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed_with_format(
        addr(3),
        ls(100, SpeedFormat::Speed128),
        Direction::Forward,
        SpeedFormat::Speed128,
    );

    let p = mgr.build_next_packet().unwrap();
    let DccPacket::Speed128 {
        address,
        speed,
        direction,
    } = p
    else {
        panic!("expected Speed128, got {p:?}");
    };
    assert_eq!(address, addr(3));
    assert_eq!(speed.value(), 100);
    assert_eq!(direction, Direction::Forward);
}

#[test]
fn test_dirty_cleared_after_build() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(10, SpeedFormat::Speed28), Direction::Forward);

    let p1 = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 { address, .. } = p1 else {
        panic!("expected Speed28, got {p1:?}");
    };
    assert_eq!(address, addr(1));

    let p2 = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 { address, .. } = p2 else {
        panic!("expected Speed28, got {p2:?}");
    };
    assert_eq!(address, addr(1));

    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Reverse);
    let p3 = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 {
        speed, direction, ..
    } = p3
    else {
        panic!("expected Speed28, got {p3:?}");
    };
    assert_eq!(speed.value(), 21);
    assert_eq!(direction, Direction::Reverse);
}

#[test]
fn test_set_function_generates_function_packets() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_function(addr(3), 0, true)); // FL
    assert!(mgr.set_function(addr(3), 5, true)); // F5

    // Speed is dirty first (slot default).
    let _ = mgr.build_next_packet().unwrap();

    // Then function dirty groups should be emitted.
    let p = mgr.build_next_packet().unwrap();
    assert!(matches!(
        p,
        DccPacket::FunctionGroup1 { .. }
            | DccPacket::FunctionGroup2A { .. }
            | DccPacket::FunctionGroup2B { .. }
            | DccPacket::FunctionGroup3 { .. }
            | DccPacket::FunctionGroup4 { .. }
    ));
}

#[test]
fn test_function_refresh_fairness_speed_then_function() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_function(addr(1), 1, true);

    // Drain dirty speed + dirty function groups.
    for _ in 0..(1 + usize::from(DIRTY_FUNCTION_RETRY_COUNT)) {
        let _ = mgr.build_next_packet();
    }

    // Refresh should alternate speed/function for slot with active functions.
    let p1 = mgr.build_next_packet().unwrap();
    let p2 = mgr.build_next_packet().unwrap();
    assert!(matches!(
        p1,
        DccPacket::Speed28 { .. } | DccPacket::Speed128 { .. }
    ));
    assert!(matches!(
        p2,
        DccPacket::FunctionGroup1 { .. }
            | DccPacket::FunctionGroup2A { .. }
            | DccPacket::FunctionGroup2B { .. }
            | DccPacket::FunctionGroup3 { .. }
            | DccPacket::FunctionGroup4 { .. }
    ));
}

#[test]
fn test_consist_base_direction_mapping() {
    let mut mgr = SlotManager::new();
    assert!(mgr.create_consist(1));
    assert!(mgr.add_to_consist(1, addr(3), false));
    assert!(mgr.add_to_consist(1, addr(4), true));

    let updated = mgr.set_consist_speed(1, ls(18, SpeedFormat::Speed28), Direction::Forward);
    assert_eq!(updated, 2);

    let mut seen_fwd = false;
    let mut seen_rev = false;
    for _ in 0..2 {
        let p = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 {
            address,
            direction,
            speed,
        } = p
        else {
            panic!("expected Speed28, got {p:?}");
        };
        assert_eq!(speed.value(), 19);
        if address == addr(3) && direction == Direction::Forward {
            seen_fwd = true;
        }
        if address == addr(4) && direction == Direction::Reverse {
            seen_rev = true;
        }
    }

    assert!(
        seen_fwd && seen_rev,
        "consist direction mapping should be applied"
    );
}

#[test]
fn test_emergency_stop_preempts_dirty_queue() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(2), ls(15, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_function(addr(1), 1, true);

    // Queue a global e-stop after normal dirty traffic exists.
    mgr.request_emergency_stop_all();

    // Must preempt dirty queue.
    let p = mgr.build_next_packet().unwrap();
    assert!(matches!(p, DccPacket::BroadcastStop));
}

#[test]
fn test_emergency_stop_single_does_not_target_others() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(2), ls(15, SpeedFormat::Speed28), Direction::Reverse);

    // Drain initial dirty speed packets.
    let _ = mgr.build_next_packet();
    let _ = mgr.build_next_packet();

    assert!(mgr.request_emergency_stop(addr(1)));

    let p = mgr.build_next_packet().unwrap();
    let DccPacket::EmergencyStop { address, .. } = p else {
        panic!("expected EmergencyStop, got {p:?}");
    };
    assert_eq!(address, addr(1));

    // Next packet must not be an e-stop for addr(2).
    let p2 = mgr.build_next_packet().unwrap();
    assert!(!matches!(
        p2,
        DccPacket::EmergencyStop { address, .. } if address == addr(2)
    ));
}

#[test]
fn test_request_emergency_stop_unknown_address_is_rejected() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    assert!(!mgr.request_emergency_stop(addr(2)));
}

#[test]
fn test_recovery_after_emergency_stop() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(1), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.build_next_packet(); // drain initial dirty

    assert!(mgr.request_emergency_stop(addr(1)));
    let p = mgr.build_next_packet().unwrap();
    assert!(matches!(p, DccPacket::EmergencyStop { .. }));

    // New command after e-stop should be emitted as dirty speed update.
    let _ = mgr.set_speed(addr(1), ls(12, SpeedFormat::Speed28), Direction::Reverse);
    let p2 = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 {
        address,
        speed,
        direction,
    } = p2
    else {
        panic!("expected Speed28, got {p2:?}");
    };
    assert_eq!(address, addr(1));
    assert_eq!(speed.value(), 13);
    assert_eq!(direction, Direction::Reverse);
}

#[test]
fn test_f13_f28_dirty_and_refresh() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_function(addr(3), 15, true)); // F15 in FG3
    assert!(mgr.set_function(addr(3), 25, true)); // F25 in FG4

    // No dirty speed when slot is created by function-only command.
    // First two packets should be dirty function groups FG3 and FG4.
    let p1 = mgr.build_next_packet().unwrap();
    let p2 = mgr.build_next_packet().unwrap();

    let mut seen_fg3 = false;
    let mut seen_fg4 = false;
    for p in [p1, p2] {
        match p {
            DccPacket::FunctionGroup3 { address, f15, .. } => {
                assert_eq!(address, addr(3));
                assert!(f15);
                seen_fg3 = true;
            }
            DccPacket::FunctionGroup4 { address, f25, .. } => {
                assert_eq!(address, addr(3));
                assert!(f25);
                seen_fg4 = true;
            }
            _ => {}
        }
    }
    assert!(seen_fg3 && seen_fg4, "Both FG3 and FG4 should be emitted");
}

#[test]
fn test_function_index_validation() {
    assert!(FunctionIndex::new(0).is_some());
    assert!(FunctionIndex::new(28).is_some());
    assert!(FunctionIndex::new(29).is_none());
    assert!(FunctionIndex::try_from(12).is_ok());
    assert!(FunctionIndex::try_from(40).is_err());
}

#[test]
fn test_function_refresh_bound_under_12_slots() {
    let mut mgr = SlotManager::new();
    for i in 1..=12 {
        assert!(mgr.set_speed(addr(i), ls(10, SpeedFormat::Speed28), Direction::Forward));
        assert!(mgr.set_function(addr(i), 13, true));
        assert!(mgr.set_function(addr(i), 21, true));
    }

    // Drain initial dirty speed/function traffic.
    for _ in 0..200 {
        let _ = mgr.build_next_packet();
    }

    let slot_count = 12usize;
    let tick_ms = scheduler_tick_ms_for_slot_count(slot_count);
    // Tight budget to force deadline path under heavy slot pressure.
    let max_visits = 1;
    let mut last_function_packet_idx = [None; 12];

    for packet_idx in 0..1500usize {
        let packet = mgr
            .build_next_packet_with_function_budget(max_visits)
            .unwrap_or(DccPacket::Idle);
        let address = match packet {
            DccPacket::FunctionGroup1 { address, .. }
            | DccPacket::FunctionGroup2A { address, .. }
            | DccPacket::FunctionGroup2B { address, .. }
            | DccPacket::FunctionGroup3 { address, .. }
            | DccPacket::FunctionGroup4 { address, .. } => Some(address),
            _ => None,
        };

        if let Some(address) = address {
            let idx = (address.value() - 1) as usize;
            if let Some(previous) = last_function_packet_idx[idx] {
                let delta_packets = packet_idx - previous;
                let gap_ms = (delta_packets as u64) * tick_ms;
                assert!(
                    gap_ms <= MAX_FUNCTION_REFRESH_MS,
                    "function refresh gap too high for addr {}: {}ms",
                    address.value(),
                    gap_ms
                );
            }
            last_function_packet_idx[idx] = Some(packet_idx);
        }
    }

    for (idx, seen) in last_function_packet_idx.iter().enumerate() {
        assert!(
            seen.is_some(),
            "slot {} never received function refresh",
            idx + 1
        );
    }

    let deadline_hits = mgr.deadline_enforced_refresh_count();
    assert!(
        deadline_hits > 0,
        "deadline enforcement path should be exercised under 12-slot pressure"
    );
}

#[test]
fn test_pause_stops_emission() {
    let mut mgr = SlotManager::new();
    assert!(!mgr.is_paused());

    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);

    // Normal emission works
    assert!(mgr.build_next_packet().is_some());

    // Pause stops emission
    mgr.pause();
    assert!(mgr.is_paused());
    assert!(mgr.build_next_packet().is_none());
    assert!(mgr.build_next_packet().is_none());
}

#[test]
fn test_resume_restarts_emission() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);

    mgr.pause();
    assert!(mgr.build_next_packet().is_none());

    mgr.resume();
    assert!(!mgr.is_paused());
    assert!(mgr.build_next_packet().is_some());
}

#[test]
fn test_pause_preserves_slot_state() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.build_next_packet(); // Drain dirty

    mgr.pause();

    // Update state during pause
    let _ = mgr.set_speed(addr(3), ls(20, SpeedFormat::Speed28), Direction::Reverse);
    assert_eq!(mgr.slot_count(), 1);

    // No emission during pause
    assert!(mgr.build_next_packet().is_none());

    // Resume and verify updated state is emitted
    mgr.resume();
    let packet = mgr.build_next_packet().unwrap();

    let DccPacket::Speed28 {
        address,
        speed,
        direction,
    } = packet
    else {
        panic!("expected Speed28, got {packet:?}");
    };
    assert_eq!(address, addr(3));
    assert_eq!(speed.value(), 21);
    assert_eq!(direction, Direction::Reverse);
}

#[test]
fn test_pause_resume_emits_dirty_first() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.set_speed(addr(5), ls(15, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.build_next_packet(); // Drain one dirty
    let _ = mgr.build_next_packet(); // Drain second dirty

    mgr.pause();

    // Make addr(3) dirty during pause
    let _ = mgr.set_speed(addr(3), ls(20, SpeedFormat::Speed28), Direction::Forward);

    mgr.resume();

    // Should emit dirty packet first (priority over refresh)
    let packet = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 { address, speed, .. } = packet else {
        panic!("expected Speed28, got {packet:?}");
    };
    assert_eq!(address, addr(3));
    assert_eq!(speed.value(), 21);
}

#[test]
fn test_dirty_speed_retries_non_zero_speed() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);

    let packet = mgr.build_next_packet().expect("expected speed packet");
    let DccPacket::Speed28 { address, speed, .. } = packet else {
        panic!("expected Speed28, got {packet:?}");
    };
    assert_eq!(address, addr(3));
    assert_eq!(speed.value(), 11);
}

#[test]
fn test_dirty_speed_retries_stop_longer() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(20, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.build_next_packet();

    let _ = mgr.set_speed(addr(3), ls(0, SpeedFormat::Speed28), Direction::Forward);

    for i in 0..=DIRTY_STOP_RETRY_COUNT {
        let packet = mgr.build_next_packet().expect("expected stop speed packet");
        let DccPacket::Speed28 { address, speed, .. } = packet else {
            panic!("expected Speed28 stop on retry {i}, got {packet:?}");
        };
        assert_eq!(address, addr(3));
        assert_eq!(speed.value(), 0);
    }
}

#[test]
fn test_pause_via_apply_command() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);

    assert!(mgr.apply_command(SchedulerCommand::Pause));
    assert!(mgr.is_paused());
    assert!(mgr.build_next_packet().is_none());

    assert!(mgr.apply_command(SchedulerCommand::Resume));
    assert!(!mgr.is_paused());
    assert!(mgr.build_next_packet().is_some());
}

#[test]
fn test_multiple_pause_resume_cycles() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);
    let _ = mgr.build_next_packet(); // Drain dirty

    // Cycle 1
    mgr.pause();
    assert!(mgr.build_next_packet().is_none());
    mgr.resume();
    assert!(mgr.build_next_packet().is_some());

    // Cycle 2
    mgr.pause();
    assert!(mgr.build_next_packet().is_none());
    mgr.resume();
    assert!(mgr.build_next_packet().is_some());
}

#[test]
fn test_program_on_main_emits_once_before_normal_traffic() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward);

    assert!(mgr.program_on_main(DccPacket::PomReadByte {
        address: addr(3),
        cv: pom_cv(29),
    }));

    let packet = mgr
        .build_next_packet()
        .expect("expected programming packet");
    assert!(matches!(packet, DccPacket::PomReadByte { cv, .. } if cv.value() == 29));

    let next = mgr
        .build_next_packet()
        .expect("expected normal packet after pom");
    assert!(matches!(next, DccPacket::Speed28 { .. }));
}

#[test]
fn test_program_on_main_queues_multiple_pending_packets() {
    let mut mgr = SlotManager::new();

    assert!(mgr.program_on_main(DccPacket::PomReadByte {
        address: addr(3),
        cv: pom_cv(1),
    }));
    assert!(mgr.program_on_main(DccPacket::PomWriteByte {
        address: addr(3),
        cv: pom_cv(29),
        value: 6,
    }));

    let first = mgr
        .build_next_packet()
        .expect("expected first programming packet");
    assert!(matches!(first, DccPacket::PomReadByte { cv, .. } if cv.value() == 1));

    let second = mgr
        .build_next_packet()
        .expect("expected second programming packet");
    assert!(matches!(second, DccPacket::PomWriteByte { cv, .. } if cv.value() == 29));
}

#[test]
fn test_program_on_main_rejects_when_queue_full() {
    let mut mgr = SlotManager::new();

    for cv in 1..=PENDING_POM_CAPACITY as u16 {
        assert!(mgr.program_on_main(DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(cv),
        }));
    }

    assert!(!mgr.program_on_main(DccPacket::PomReadByte {
        address: addr(3),
        cv: pom_cv(29),
    }));
}

#[test]
fn test_ensure_railcom_refresh_slot_creates_non_dirty_refresh() {
    let mut mgr = SlotManager::new();

    assert!(mgr.ensure_railcom_refresh_slot(addr(3), SpeedFormat::Speed128));
    assert_eq!(mgr.slot_count(), 1);

    let packet = mgr
        .build_next_packet()
        .expect("expected refresh packet for RailCom attribution");
    assert!(matches!(
        packet,
        DccPacket::Speed128 {
            address,
            speed,
            direction,
        } if address == addr(3)
            && speed.value() == 0
            && direction == Direction::Forward
    ));
}

#[test]
fn test_ensure_railcom_refresh_slot_does_not_modify_existing_slot() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_speed_with_format(
        addr(3),
        ls(42, SpeedFormat::Speed128),
        Direction::Reverse,
        SpeedFormat::Speed128
    ));
    let _ = mgr.build_next_packet();

    assert!(mgr.ensure_railcom_refresh_slot(addr(3), SpeedFormat::Speed28));

    let packet = mgr
        .build_next_packet()
        .expect("expected existing slot refresh packet");
    assert!(matches!(
        packet,
        DccPacket::Speed128 {
            address,
            speed,
            direction,
        } if address == addr(3)
            && speed.value() == 42
            && direction == Direction::Reverse
    ));
}

#[test]
fn test_function_group_refreshed_after_all_off() {
    let mut mgr = SlotManager::new();
    let _ = mgr.set_function_indexed(addr(3), FunctionIndex::new(0).unwrap(), true);
    // No dirty speed packet when slot is created by function-only command
    mgr.build_next_packet(); // dirty FunctionGroup1{fl:true}

    let _ = mgr.set_function_indexed(addr(3), FunctionIndex::new(0).unwrap(), false);
    mgr.build_next_packet(); // dirty FunctionGroup1{fl:false}

    // After: functions=0, known_groups=0b00001
    // The refresh loop MUST continue sending FunctionGroup1
    let mut got_fg1 = false;
    for _ in 0..20 {
        if let Some(DccPacket::FunctionGroup1 { fl, .. }) = mgr.build_next_packet() {
            assert!(!fl, "fl must be false after F0 OFF");
            got_fg1 = true;
            break;
        }
    }
    assert!(
        got_fg1,
        "FunctionGroup1 must appear in refresh even with all functions off"
    );
}

#[test]
fn test_railcom_budget_denies_safety_and_command_packets() {
    reset_railcom_scheduler_stats();
    let mut budget = RailcomCutoutBudget::new();

    assert!(!budget.allow_cutout_for(PacketClass::Safety));
    assert!(!budget.allow_cutout_for(PacketClass::Command));

    assert_eq!(
        railcom_scheduler_stats(),
        RailcomSchedulerStats {
            cutout_granted_count: 0,
            cutout_skipped_budget_count: 0,
            cutout_skipped_priority_count: 2,
            ..Default::default()
        }
    );
}

#[test]
fn test_railcom_budget_grants_idle_after_minimum_gap() {
    reset_railcom_scheduler_stats();
    let mut budget = RailcomCutoutBudget::new();

    assert!(budget.allow_cutout_for(PacketClass::Idle));

    assert_eq!(
        railcom_scheduler_stats(),
        RailcomSchedulerStats {
            cutout_granted_count: 1,
            cutout_skipped_budget_count: 0,
            cutout_skipped_priority_count: 0,
            ..Default::default()
        }
    );
}

#[test]
fn test_railcom_budget_enforces_gap_between_refresh_cutouts() {
    reset_railcom_scheduler_stats();
    let mut budget = RailcomCutoutBudget::new();

    assert!(budget.allow_cutout_for(PacketClass::Refresh));
    for _ in 0..RAILCOM_MIN_PACKET_GAP {
        assert!(!budget.allow_cutout_for(PacketClass::Refresh));
    }
    assert!(budget.allow_cutout_for(PacketClass::Refresh));

    assert_eq!(
        railcom_scheduler_stats(),
        RailcomSchedulerStats {
            cutout_granted_count: 2,
            cutout_skipped_budget_count: RAILCOM_MIN_PACKET_GAP as u32,
            cutout_skipped_priority_count: 0,
            ..Default::default()
        }
    );
}

#[test]
fn test_railcom_budget_recovers_on_packets_without_cutout_request() {
    reset_railcom_scheduler_stats();
    let mut budget = RailcomCutoutBudget::new();

    assert!(budget.allow_cutout_for(PacketClass::Idle));
    for _ in 0..RAILCOM_MIN_PACKET_GAP {
        budget.note_packet_without_cutout();
    }
    assert!(budget.allow_cutout_for(PacketClass::Idle));

    assert_eq!(
        railcom_scheduler_stats(),
        RailcomSchedulerStats {
            cutout_granted_count: 2,
            cutout_skipped_budget_count: 0,
            cutout_skipped_priority_count: 0,
            ..Default::default()
        }
    );
}

#[test]
fn test_railcom_budget_allows_programming_cutout_immediately() {
    reset_railcom_scheduler_stats();
    let mut budget = RailcomCutoutBudget::new();

    assert!(budget.allow_cutout_for(PacketClass::Programming));
    assert_eq!(
        railcom_scheduler_stats(),
        RailcomSchedulerStats {
            cutout_granted_count: 1,
            cutout_granted_pom_count: 1,
            cutout_skipped_budget_count: 0,
            cutout_skipped_priority_count: 0,
            ..Default::default()
        }
    );
}

#[test]
fn test_railcom_budget_preserves_priority_during_command_burst_then_recovers() {
    reset_railcom_scheduler_stats();
    let mut budget = RailcomCutoutBudget::new();

    assert!(budget.allow_cutout_for(PacketClass::Idle));
    for _ in 0..3 {
        assert!(!budget.allow_cutout_for(PacketClass::Command));
    }
    for _ in 0..(RAILCOM_MIN_PACKET_GAP - 3) {
        assert!(!budget.allow_cutout_for(PacketClass::Refresh));
    }
    assert!(budget.allow_cutout_for(PacketClass::Refresh));

    assert_eq!(
        railcom_scheduler_stats(),
        RailcomSchedulerStats {
            cutout_granted_count: 2,
            cutout_skipped_budget_count: (RAILCOM_MIN_PACKET_GAP - 3) as u32,
            cutout_skipped_priority_count: 3,
            ..Default::default()
        }
    );
}

#[test]
fn test_scheduler_classifies_dirty_speed_as_command() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward));

    let (packet, class) = mgr
        .build_next_packet_classified_with_function_budget(allowed_slot_visits_without_function(1))
        .expect("expected classified packet");

    assert!(matches!(packet, DccPacket::Speed28 { .. }));
    assert_eq!(class, PacketClass::Command);
}

#[test]
fn test_scheduler_classifies_following_packet_as_refresh_after_dirty_command() {
    let mut mgr = SlotManager::new();
    assert!(mgr.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward));

    let (_, first_class) = mgr
        .build_next_packet_classified_with_function_budget(allowed_slot_visits_without_function(1))
        .expect("expected dirty command packet");
    let (packet, second_class) = mgr
        .build_next_packet_classified_with_function_budget(allowed_slot_visits_without_function(1))
        .expect("expected refresh packet");

    assert_eq!(first_class, PacketClass::Command);
    assert!(matches!(packet, DccPacket::Speed28 { .. }));
    assert_eq!(second_class, PacketClass::Refresh);
}
