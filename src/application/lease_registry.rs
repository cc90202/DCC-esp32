//! Process-wide registry for the lease accepted at mutation boundaries.
//!
//! `ClientSafetyPolicy` decides ownership.  This registry publishes only the
//! currently accepted epoch so consumers can reject stale queued work.

use super::client_safety::{LeaseEpoch, LeasePermit};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct LeaseRegistry {
    current: Option<LeasePermit>,
    accepted_epoch: Option<LeaseEpoch>,
}

impl LeaseRegistry {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            current: None,
            accepted_epoch: None,
        }
    }

    pub(crate) fn publish(&mut self, permit: LeasePermit) {
        self.current = Some(permit);
    }

    pub(crate) fn invalidate(&mut self) {
        self.current = None;
        self.accepted_epoch = None;
    }

    #[must_use]
    pub(crate) fn accepts(self, permit: LeasePermit, now_ms: u64) -> bool {
        self.current.is_some_and(|current| {
            current.epoch() == permit.epoch()
                && current.is_valid_at(now_ms)
                && permit.is_valid_at(now_ms)
        })
    }

    #[must_use]
    pub(crate) fn accepts_mutation(self, permit: LeasePermit, now_ms: u64) -> bool {
        self.accepted_epoch == Some(permit.epoch()) && self.accepts(permit, now_ms)
    }

    pub(crate) fn accept_epoch(&mut self, permit: LeasePermit, now_ms: u64) -> bool {
        if !self.accepts(permit, now_ms) {
            return false;
        }
        self.accepted_epoch = Some(permit.epoch());
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::application::client_safety::{
        ClientSafetyPolicy, LeaseActivity, LeaseDecision, LeaseRequest,
    };

    #[test]
    fn only_current_unexpired_permit_is_accepted() {
        let mut policy = ClientSafetyPolicy::new(30_000);
        let LeaseDecision::Acquired(first) = policy.handle(
            LeaseRequest {
                client: 1,
                activity: LeaseActivity::Acquire,
                observed_at_ms: 100,
            },
            100,
        ) else {
            panic!("expected first permit");
        };
        let _ = policy.handle(
            LeaseRequest {
                client: 1,
                activity: LeaseActivity::Logoff,
                observed_at_ms: 150,
            },
            150,
        );
        let LeaseDecision::Acquired(second) = policy.handle(
            LeaseRequest {
                client: 2,
                activity: LeaseActivity::Acquire,
                observed_at_ms: 200,
            },
            200,
        ) else {
            panic!("expected second permit");
        };
        let mut registry = LeaseRegistry::new();

        registry.publish(first);
        assert!(registry.accepts(first, 30_099));
        assert!(!registry.accepts_mutation(first, 30_099));
        assert!(registry.accept_epoch(first, 30_099));
        assert!(registry.accepts_mutation(first, 30_099));
        assert!(!registry.accepts(first, 30_100));
        assert!(!registry.accepts(second, 200));

        registry.invalidate();
        assert!(!registry.accepts(first, 200));
    }

    #[test]
    fn renewal_does_not_revoke_older_permit_from_same_epoch() {
        let mut policy = ClientSafetyPolicy::new(30_000);
        let LeaseDecision::Acquired(first) = policy.handle(
            LeaseRequest {
                client: 1,
                activity: LeaseActivity::Acquire,
                observed_at_ms: 100,
            },
            100,
        ) else {
            panic!("expected first permit");
        };
        let LeaseDecision::Maintained(renewed) = policy.handle(
            LeaseRequest {
                client: 1,
                activity: LeaseActivity::KeepAlive,
                observed_at_ms: 1_000,
            },
            1_000,
        ) else {
            panic!("expected renewed permit");
        };
        let mut registry = LeaseRegistry::new();
        registry.publish(renewed);
        assert!(registry.accept_epoch(renewed, 1_000));

        assert!(registry.accepts(first, 1_000));
        assert!(!registry.accepts(first, first.expires_at_ms()));
        assert!(registry.accepts(renewed, first.expires_at_ms()));
    }
}
