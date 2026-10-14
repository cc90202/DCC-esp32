//! Runtime adapter that serializes controller authority with track power.
//!
//! The lease policy and registry remain framework-independent in
//! `authority`; this module owns the ESP critical section, the clock read
//! that judges lease expiry, and the hardware side-effect at the outer
//! runtime boundary.
//!
//! Every check reads the clock inside the critical section, so the time a
//! permit is judged against cannot go stale between the read and the
//! decision. The cost is one timer read (a few hundred nanoseconds) held
//! under the lock.

use core::cell::RefCell;

use critical_section::Mutex;

use crate::authority::LeasePermit;
use crate::authority::lease_registry::LeaseRegistry;
use crate::clock::now_ms;

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

/// Whether `permit` is current, unexpired and its epoch has been accepted.
#[must_use]
pub(crate) fn accepts(permit: LeasePermit) -> bool {
    critical_section::with(|cs| {
        REGISTRY
            .borrow(cs)
            .borrow()
            .accepts_mutation(permit, now_ms())
    })
}

/// Whether `permit` is current and unexpired, whatever its epoch state.
#[must_use]
pub(crate) fn accepts_current(permit: LeasePermit) -> bool {
    critical_section::with(|cs| REGISTRY.borrow(cs).borrow().accepts(permit, now_ms()))
}

/// Marks the epoch of a current, unexpired `permit` as accepted.
pub(crate) fn accept_epoch(permit: LeasePermit) -> bool {
    critical_section::with(|cs| {
        REGISTRY
            .borrow(cs)
            .borrow_mut()
            .accept_epoch(permit, now_ms())
    })
}

/// Run a physical enable action only while this permit is still the accepted
/// authority. The check and action share the same critical section so expiry
/// or revocation cannot open a time-of-check/time-of-use window.
pub(crate) fn apply_if_current<T>(permit: LeasePermit, action: impl FnOnce() -> T) -> Option<T> {
    critical_section::with(|cs| {
        REGISTRY
            .borrow(cs)
            .borrow()
            .accepts_mutation(permit, now_ms())
            .then(action)
    })
}
