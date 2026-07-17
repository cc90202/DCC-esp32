use super::core::handle_loco_request_message;
use super::railcom_policy::{
    PacketClass, RAILCOM_MIN_PACKET_GAP, RailcomCutoutBudget, reset_railcom_scheduler_stats,
};
use super::slot_manager::{
    DIRTY_FUNCTION_RETRY_COUNT, DIRTY_STOP_RETRY_COUNT, MAX_FUNCTION_REFRESH_MS,
    allowed_slot_visits_without_function, scheduler_tick_ms_for_slot_count,
};
use super::*;
use crate::dcc::CutoutMode;
use crate::dcc::packet::PomCv;

fn addr(n: u8) -> DccAddress {
    DccAddress::new_short(n).unwrap()
}

fn pom_cv(cv: u16) -> PomCv {
    PomCv::new(cv).expect("test POM CV must be valid")
}

fn pom_id(value: u32) -> PomRequestId {
    PomRequestId::new(value)
}

fn ls(value: u8, format: SpeedFormat) -> LogicalSpeed {
    LogicalSpeed::new(value, format).expect("test speed value must be valid for format")
}

fn set_function(
    manager: &mut SlotManager,
    address: DccAddress,
    function: u8,
    enabled: bool,
) -> bool {
    let function = FunctionIndex::new(function).expect("test function index must be valid");
    let change = if enabled {
        FunctionChange::Enable
    } else {
        FunctionChange::Disable
    };
    matches!(
        manager.handle_loco_request(LocoRequest::SetFunction {
            address,
            function,
            change,
        }),
        LocoRequestResult::Inserted(_)
            | LocoRequestResult::Updated(_)
            | LocoRequestResult::Replaced { .. }
    )
}

fn fill_with_moving_locomotives(manager: &mut SlotManager) {
    for value in 1..=12 {
        assert!(matches!(
            manager.handle_loco_request(LocoRequest::SetSpeed {
                address: addr(value),
                speed: ls(value, SpeedFormat::Speed128),
                direction: Direction::Forward,
            }),
            LocoRequestResult::Inserted(_)
        ));
    }
}

fn drain_pending_slot_transmissions(manager: &mut SlotManager) {
    for _ in 0..256 {
        if !manager.has_pending_slot_transmissions_for_test() {
            return;
        }
        assert!(manager.build_next_packet().is_some());
    }
    panic!("slot transmissions did not quiesce within the bounded test budget");
}

#[test]
fn loco_request_id_preserves_wrapped_counter_values() {
    let id = LocoRequestId::new(u32::MAX);
    assert_eq!(id.value(), u32::MAX);
}

#[test]
fn expired_loco_request_is_rejected_without_mutating_scheduler_state() {
    let mut manager = SlotManager::new();
    let message = LocoRequestMessage {
        request_id: LocoRequestId::new(4),
        request: LocoRequest::SetSpeed {
            address: addr(3),
            speed: ls(10, SpeedFormat::Speed128),
            direction: Direction::Forward,
        },
        deadline: LocoRequestDeadline::from_ticks(100),
    };

    assert_eq!(
        handle_loco_request_message(&mut manager, message, 100),
        LocoResponse {
            request_id: LocoRequestId::new(4),
            result: LocoRequestResult::Expired,
        }
    );
    assert!(manager.is_empty());
}

#[test]
fn unexpired_loco_request_is_applied_before_its_deadline() {
    let mut manager = SlotManager::new();
    let message = LocoRequestMessage {
        request_id: LocoRequestId::new(5),
        request: LocoRequest::SetSpeed {
            address: addr(3),
            speed: ls(10, SpeedFormat::Speed128),
            direction: Direction::Forward,
        },
        deadline: LocoRequestDeadline::from_ticks(101),
    };

    assert!(matches!(
        handle_loco_request_message(&mut manager, message, 100),
        LocoResponse {
            request_id,
            result: LocoRequestResult::Inserted(_),
        } if request_id == LocoRequestId::new(5)
    ));
}

#[test]
fn loco_requests_return_authoritative_inserted_updated_and_found_snapshots() {
    let mut manager = SlotManager::new();
    let address = addr(3);

    let inserted = manager.handle_loco_request(LocoRequest::SetSpeed {
        address,
        speed: ls(12, SpeedFormat::Speed28),
        direction: Direction::Reverse,
    });
    assert!(matches!(
        inserted,
        LocoRequestResult::Inserted(LocoSnapshot {
            address: snapshot_address,
            speed,
            direction: Direction::Reverse,
            functions: 0,
        }) if snapshot_address == address
            && speed.value() == 12
            && speed.format() == SpeedFormat::Speed28
    ));

    let function = FunctionIndex::new(4).unwrap();
    let updated = manager.handle_loco_request(LocoRequest::SetFunction {
        address,
        function,
        change: FunctionChange::Enable,
    });
    assert!(matches!(
        updated,
        LocoRequestResult::Updated(LocoSnapshot { functions, .. })
            if functions == 1 << function.get()
    ));

    let found = manager.handle_loco_request(LocoRequest::GetState { address });
    assert!(matches!(
        found,
        LocoRequestResult::Found(LocoSnapshot { speed, functions, .. })
            if speed.value() == 12 && functions == 1 << function.get()
    ));
}

#[test]
fn function_toggle_is_resolved_against_scheduler_state() {
    let mut manager = SlotManager::new();
    let address = addr(9);
    let function = FunctionIndex::new(2).unwrap();

    for expected_enabled in [true, false, true] {
        let result = manager.handle_loco_request(LocoRequest::SetFunction {
            address,
            function,
            change: FunctionChange::Toggle,
        });
        assert!(matches!(
            result,
            LocoRequestResult::Inserted(LocoSnapshot { functions, .. })
                | LocoRequestResult::Updated(LocoSnapshot { functions, .. })
                if ((functions >> function.get()) & 1 != 0) == expected_enabled
        ));
    }
}

#[test]
fn ensure_refresh_is_idempotent_and_get_unknown_does_not_create_slot() {
    let mut manager = SlotManager::new();
    let address = addr(7);

    assert_eq!(
        manager.handle_loco_request(LocoRequest::GetState { address }),
        LocoRequestResult::NotFound
    );
    assert!(manager.is_empty());

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::EnsureRefresh {
            address,
            format: SpeedFormat::Speed128,
        }),
        LocoRequestResult::Inserted(LocoSnapshot {
            speed,
            ..
        }) if speed.is_zero() && speed.format() == SpeedFormat::Speed128
    ));
    assert!(matches!(
        manager.handle_loco_request(LocoRequest::EnsureRefresh {
            address,
            format: SpeedFormat::Speed28,
        }),
        LocoRequestResult::Found(LocoSnapshot {
            speed,
            ..
        }) if speed.format() == SpeedFormat::Speed128
    ));
    assert_eq!(manager.slot_count(), 1);
}

#[test]
fn full_loco_table_with_all_moving_rejects_without_any_mutation() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    let before = manager.clone();

    assert_eq!(
        manager.handle_loco_request(LocoRequest::SetSpeed {
            address: addr(13),
            speed: ls(1, SpeedFormat::Speed128),
            direction: Direction::Forward,
        }),
        LocoRequestResult::Full
    );
    assert_eq!(manager, before);
}

#[test]
fn full_loco_table_replaces_only_the_oldest_stopped_slot() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    assert!(manager.request_emergency_stop(addr(2)));
    assert!(manager.request_emergency_stop(addr(4)));
    drain_pending_slot_transmissions(&mut manager);

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::SetSpeed {
            address: addr(13),
            speed: ls(40, SpeedFormat::Speed128),
            direction: Direction::Reverse,
        }),
        LocoRequestResult::Replaced {
            removed,
            inserted: LocoSnapshot {
                address: inserted,
                speed,
                direction: Direction::Reverse,
                ..
            },
        } if removed == addr(2) && inserted == addr(13) && speed.value() == 40
    ));
    assert_eq!(manager.slot_count(), 12);
    assert!(manager.loco_snapshot(addr(2)).is_none());
    assert!(manager.loco_snapshot(addr(4)).is_some());
}

#[test]
fn simultaneous_global_stop_uses_lowest_logical_index_as_tie_breaker() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    manager.request_emergency_stop_all();
    drain_pending_slot_transmissions(&mut manager);

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::SetFunction {
            address: addr(13),
            function: FunctionIndex::new(1).unwrap(),
            change: FunctionChange::Enable,
        }),
        LocoRequestResult::Replaced {
            removed,
            inserted: LocoSnapshot {
                address: inserted,
                speed,
                functions,
                ..
            },
        } if removed == addr(1) && inserted == addr(13) && speed.is_zero() && functions == 1 << 1
    ));
}

#[test]
fn global_stop_preserves_the_age_of_slots_that_were_already_stopped() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    assert!(manager.request_emergency_stop(addr(7)));
    drain_pending_slot_transmissions(&mut manager);
    manager.request_emergency_stop_all();
    drain_pending_slot_transmissions(&mut manager);

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::SetSpeed {
            address: addr(13),
            speed: ls(1, SpeedFormat::Speed128),
            direction: Direction::Forward,
        }),
        LocoRequestResult::Replaced { removed, .. } if removed == addr(7)
    ));
}

#[test]
fn stopped_age_comparison_remains_correct_across_u64_wrap() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    manager.set_mutation_sequence_for_test(u64::MAX - 2);
    assert!(manager.request_emergency_stop(addr(3)));
    assert!(manager.request_emergency_stop(addr(4)));
    assert!(manager.request_emergency_stop(addr(5)));
    drain_pending_slot_transmissions(&mut manager);

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::SetSpeed {
            address: addr(13),
            speed: ls(1, SpeedFormat::Speed128),
            direction: Direction::Forward,
        }),
        LocoRequestResult::Replaced { removed, .. } if removed == addr(3)
    ));
}

#[test]
fn pending_targeted_estop_prevents_replacement_until_stop_is_queued() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    assert!(manager.request_emergency_stop(addr(2)));

    let request = LocoRequest::SetSpeed {
        address: addr(13),
        speed: ls(20, SpeedFormat::Speed128),
        direction: Direction::Forward,
    };
    assert_eq!(
        manager.handle_loco_request(request),
        LocoRequestResult::Full
    );
    assert!(matches!(
        manager.build_next_packet(),
        Some(DccPacket::EmergencyStop { address, .. }) if address == addr(2)
    ));
    assert_eq!(
        manager.handle_loco_request(request),
        LocoRequestResult::Full
    );

    drain_pending_slot_transmissions(&mut manager);
    assert!(matches!(
        manager.handle_loco_request(request),
        LocoRequestResult::Replaced { removed, .. } if removed == addr(2)
    ));
}

#[test]
fn dirty_normal_stop_prevents_replacement_until_all_retries_are_queued() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    assert!(manager.set_speed_with_format(
        addr(3),
        LogicalSpeed::zero(SpeedFormat::Speed128),
        Direction::Forward,
        SpeedFormat::Speed128,
    ));

    let request = LocoRequest::SetSpeed {
        address: addr(13),
        speed: ls(20, SpeedFormat::Speed128),
        direction: Direction::Forward,
    };
    assert_eq!(
        manager.handle_loco_request(request),
        LocoRequestResult::Full
    );
    drain_pending_slot_transmissions(&mut manager);
    assert!(matches!(
        manager.handle_loco_request(request),
        LocoRequestResult::Replaced { removed, .. } if removed == addr(3)
    ));
}

#[test]
fn dirty_function_prevents_replacement_and_consist_does_not_keep_removed_address() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    drain_pending_slot_transmissions(&mut manager);
    assert!(manager.create_consist(1));
    assert!(manager.add_to_consist(1, addr(4), false));
    assert!(manager.request_emergency_stop(addr(4)));
    drain_pending_slot_transmissions(&mut manager);
    assert!(set_function(&mut manager, addr(4), 2, true));

    let request = LocoRequest::SetSpeed {
        address: addr(13),
        speed: ls(20, SpeedFormat::Speed128),
        direction: Direction::Forward,
    };
    assert_eq!(
        manager.handle_loco_request(request),
        LocoRequestResult::Full
    );
    drain_pending_slot_transmissions(&mut manager);
    assert!(matches!(
        manager.handle_loco_request(request),
        LocoRequestResult::Replaced { removed, .. } if removed == addr(4)
    ));
    assert_eq!(
        manager.set_consist_speed(1, ls(5, SpeedFormat::Speed28), Direction::Forward),
        0
    );
}

#[test]
fn ensure_refresh_never_replaces_a_full_table_even_when_a_slot_is_stopped() {
    let mut manager = SlotManager::new();
    fill_with_moving_locomotives(&mut manager);
    assert!(manager.request_emergency_stop(addr(1)));
    let before = manager.clone();

    assert_eq!(
        manager.handle_loco_request(LocoRequest::EnsureRefresh {
            address: addr(13),
            format: SpeedFormat::Speed128,
        }),
        LocoRequestResult::Full
    );
    assert_eq!(manager, before);
}

#[test]
fn stopped_age_changes_only_when_motion_state_changes() {
    let mut manager = SlotManager::new();
    let address = addr(6);
    assert!(manager.set_speed(address, LogicalSpeed::ZERO, Direction::Forward));
    let original_age = manager.stopped_since_for_test(address);
    assert!(original_age.is_some());

    assert!(manager.set_speed(address, LogicalSpeed::ZERO, Direction::Reverse));
    assert!(set_function(&mut manager, address, 2, true));
    assert_eq!(manager.stopped_since_for_test(address), original_age);

    assert!(manager.set_speed(address, ls(1, SpeedFormat::Speed28), Direction::Forward));
    assert_eq!(manager.stopped_since_for_test(address), None);
    assert!(manager.request_emergency_stop(address));
    assert_ne!(manager.stopped_since_for_test(address), original_age);
}

#[test]
fn emergency_stop_request_requires_existing_slot_and_returns_stopped_state() {
    let mut manager = SlotManager::new();
    let address = addr(8);
    assert_eq!(
        manager.handle_loco_request(LocoRequest::EmergencyStop { address }),
        LocoRequestResult::NotFound
    );
    assert!(manager.set_speed(address, ls(20, SpeedFormat::Speed28), Direction::Forward));

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::EmergencyStop { address }),
        LocoRequestResult::Updated(LocoSnapshot {
            speed: LogicalSpeed::ZERO,
            ..
        })
    ));
}

#[test]
fn logical_speed_keeps_its_validated_format_when_applied() {
    let mut manager = SlotManager::new();
    let address = addr(9);
    let speed_128 = ls(100, SpeedFormat::Speed128);

    assert!(matches!(
        manager.handle_loco_request(LocoRequest::SetSpeed {
            address,
            speed: speed_128,
            direction: Direction::Forward,
        }),
        LocoRequestResult::Inserted(LocoSnapshot { speed, .. })
            if speed.format() == SpeedFormat::Speed128 && speed.value() == 100
    ));
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

    mgr.request_emergency_stop_all();

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

    assert!(mgr.request_emergency_stop(addr(1)));

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

    assert!(!mgr.remove_slot(addr(1)));
    assert!(mgr.set_speed(addr(1), LogicalSpeed::ZERO, Direction::Forward));
    assert!(!mgr.remove_slot(addr(1)));
    drain_pending_slot_transmissions(&mut mgr);
    assert!(mgr.remove_slot(addr(1)));
    assert_eq!(mgr.slot_count(), 1);
    assert!(!mgr.remove_slot(addr(1)));

    let p = mgr.build_next_packet().unwrap();
    let DccPacket::Speed28 { address, .. } = p else {
        panic!("expected Speed28, got {p:?}");
    };
    assert_eq!(address, addr(2));
}

#[test]
fn remove_slot_rejects_pending_targeted_estop() {
    let mut manager = SlotManager::new();
    assert!(manager.set_speed(addr(3), ls(10, SpeedFormat::Speed28), Direction::Forward,));
    drain_pending_slot_transmissions(&mut manager);
    assert!(manager.request_emergency_stop(addr(3)));

    assert!(!manager.remove_slot(addr(3)));
    assert!(matches!(
        manager.build_next_packet(),
        Some(DccPacket::EmergencyStop { address, .. }) if address == addr(3)
    ));
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
    assert!(set_function(&mut mgr, addr(3), 0, true)); // FL
    assert!(set_function(&mut mgr, addr(3), 5, true)); // F5

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
    let _ = set_function(&mut mgr, addr(1), 1, true);

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
    let _ = set_function(&mut mgr, addr(1), 1, true);

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
    assert!(set_function(&mut mgr, addr(3), 15, true)); // F15 in FG3
    assert!(set_function(&mut mgr, addr(3), 25, true)); // F25 in FG4

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
        assert!(set_function(&mut mgr, addr(i), 13, true));
        assert!(set_function(&mut mgr, addr(i), 21, true));
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

    assert!(mgr.program_on_main(
        pom_id(1),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(29),
        }
    ));

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

    assert!(mgr.program_on_main(
        pom_id(2),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(1),
        }
    ));
    assert!(mgr.program_on_main(
        pom_id(2),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(29),
        }
    ));

    let first = mgr
        .build_next_packet()
        .expect("expected first programming packet");
    assert!(matches!(first, DccPacket::PomReadByte { cv, .. } if cv.value() == 1));

    let second = mgr
        .build_next_packet()
        .expect("expected second programming packet");
    assert!(matches!(second, DccPacket::PomReadByte { cv, .. } if cv.value() == 29));
}

#[test]
fn test_same_pom_request_cannot_change_target_or_operation() {
    let mut mgr = SlotManager::new();
    assert!(mgr.program_on_main(
        pom_id(4),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(1),
        },
    ));

    assert!(!mgr.program_on_main(
        pom_id(4),
        DccPacket::PomReadByte {
            address: addr(4),
            cv: pom_cv(1),
        },
    ));
    assert!(!mgr.program_on_main(
        pom_id(4),
        DccPacket::PomWriteByte {
            address: addr(3),
            cv: pom_cv(1),
            value: 7,
        },
    ));
}

#[test]
fn test_program_on_main_rejects_when_queue_full() {
    let mut mgr = SlotManager::new();

    for cv in 1..=PENDING_POM_CAPACITY as u16 {
        assert!(mgr.program_on_main(
            pom_id(3),
            DccPacket::PomReadByte {
                address: addr(3),
                cv: pom_cv(cv),
            }
        ));
    }

    assert!(!mgr.program_on_main(
        pom_id(3),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(29),
        }
    ));
}

#[test]
fn test_new_pom_request_discards_queued_packets_from_previous_request() {
    let mut mgr = SlotManager::new();
    assert!(mgr.program_on_main(
        pom_id(10),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(1),
        },
    ));
    assert!(mgr.program_on_main(
        pom_id(11),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(2),
        },
    ));

    let packet = mgr
        .build_next_packet()
        .expect("new request must remain queued");
    assert!(matches!(packet, DccPacket::PomReadByte { cv, .. } if cv.value() == 2));
}

#[test]
fn test_stale_close_cannot_clear_newer_pom_request() {
    let mut mgr = SlotManager::new();
    assert!(mgr.program_on_main(
        pom_id(20),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(1),
        },
    ));
    assert!(mgr.program_on_main(
        pom_id(21),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(2),
        },
    ));

    assert!(!mgr.close_program_on_main(pom_id(20)));
    assert_eq!(
        mgr.pom_context_for_packet(DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(2),
        }),
        Some((pom_id(21), CutoutMode::PomRead)),
    );
}

#[test]
fn test_pom_request_tags_followup_packets_until_matching_close() {
    let mut mgr = SlotManager::new();
    assert!(mgr.program_on_main(
        pom_id(30),
        DccPacket::PomReadByte {
            address: addr(3),
            cv: pom_cv(8),
        },
    ));
    let followup = DccPacket::Speed128 {
        address: addr(3),
        speed: crate::dcc::NmraSpeed128::new(0).unwrap(),
        direction: Direction::Forward,
    };

    assert_eq!(
        mgr.pom_context_for_packet(followup),
        Some((pom_id(30), CutoutMode::PomRead)),
    );
    assert!(mgr.close_program_on_main(pom_id(30)));
    assert_eq!(mgr.pom_context_for_packet(followup), None);
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
    let _ = set_function(&mut mgr, addr(3), 0, true);
    // No dirty speed packet when slot is created by function-only command
    mgr.build_next_packet(); // dirty FunctionGroup1{fl:true}

    let _ = set_function(&mut mgr, addr(3), 0, false);
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
