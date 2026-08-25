//! Runtime adapter that serializes controller authority with track power.
//!
//! The lease policy and registry remain framework-independent in
//! `application`; this module owns the ESP critical section and hardware
//! side-effect at the outer runtime boundary.

use core::cell::RefCell;

use critical_section::Mutex;

use crate::application::lease_registry::LeaseRegistry;
use crate::authority::LeasePermit;

static REGISTRY: Mutex<RefCell<LeaseRegistry>> = Mutex::new(RefCell::new(LeaseRegistry::new()));

pub(crate) fn publish(permit: LeasePermit) {
    critical_section::with(|cs| REGISTRY.borrow(cs).borrow_mut().publish(permit));
}

/// Invalidate authority and cut bridge power in one serialized region.
pub(crate) fn invalidate_and_disable() {
    critical_section::with(|cs| {
        REGISTRY.borrow(cs).borrow_mut().invalidate();
        crate::track_safety::disable_track_intentionally();
    });
}

#[must_use]
pub(crate) fn accepts(permit: LeasePermit, now_ms: u64) -> bool {
    critical_section::with(|cs| {
        REGISTRY
            .borrow(cs)
            .borrow()
            .accepts_mutation(permit, now_ms)
    })
}

#[must_use]
pub(crate) fn accepts_current(permit: LeasePermit, now_ms: u64) -> bool {
    critical_section::with(|cs| REGISTRY.borrow(cs).borrow().accepts(permit, now_ms))
}

pub(crate) fn accept_epoch(permit: LeasePermit, now_ms: u64) -> bool {
    critical_section::with(|cs| {
        REGISTRY
            .borrow(cs)
            .borrow_mut()
            .accept_epoch(permit, now_ms)
    })
}

/// Run a physical enable action only while this permit is still the accepted
/// authority. The check and action share the same critical section so expiry
/// or revocation cannot open a time-of-check/time-of-use window.
pub(crate) fn apply_if_current<T>(
    permit: LeasePermit,
    now_ms: u64,
    action: impl FnOnce() -> T,
) -> Option<T> {
    critical_section::with(|cs| {
        REGISTRY
            .borrow(cs)
            .borrow()
            .accepts_mutation(permit, now_ms)
            .then(action)
    })
}
