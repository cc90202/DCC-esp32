//! Pure controller-lease policy for safety-sensitive network control.
//!
//! The policy deliberately knows nothing about UDP, Embassy or GPIO.  It
//! decides ownership and deadlines from the observation timestamp carried by
//! the adapter.  The watchdog adapter is responsible for applying a
//! [`LeaseDecision::Trip`] before acknowledging the request.

pub use crate::authority::{LeaseEpoch, LeasePermit};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum LeaseActivity {
    /// A mutating command may acquire a free lease and needs a permit.
    Acquire,
    /// A valid non-mutating command only refreshes the current owner.
    KeepAlive,
    /// The owner deliberately closes its session.
    Logoff,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LeaseRequest<Client> {
    pub(crate) client: Client,
    pub(crate) activity: LeaseActivity,
    pub(crate) observed_at_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum LeaseRejection {
    OwnedByAnotherClient,
    NoActiveLease,
    ObservationExpired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum TripReason {
    Timeout,
    OwnerLogoff,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LeaseDecision<Client> {
    /// A free lease was acquired with a new epoch.
    Acquired(LeasePermit),
    /// A mutating command renewed the existing owner's epoch.
    Renewed(LeasePermit),
    /// A valid query refreshed the current owner's deadline.
    Maintained(LeasePermit),
    /// The activity did not change lease state.
    Rejected(LeaseRejection),
    /// The old lease must be invalidated and the track tripped.
    Trip { client: Client, reason: TripReason },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ClientSafetyPolicy<Client> {
    active_client: Option<Client>,
    permit: Option<LeasePermit>,
    timeout_ms: u64,
    next_epoch: u32,
}

impl<Client: Copy + Eq> ClientSafetyPolicy<Client> {
    #[must_use]
    pub(crate) const fn new(timeout_ms: u64) -> Self {
        Self {
            active_client: None,
            permit: None,
            timeout_ms,
            next_epoch: 1,
        }
    }

    #[must_use]
    #[cfg(test)]
    pub(crate) const fn active_client(self) -> Option<Client> {
        self.active_client
    }

    #[must_use]
    pub(crate) const fn next_deadline_ms(self) -> Option<u64> {
        match self.permit {
            Some(permit) => Some(permit.expires_at_ms()),
            None => None,
        }
    }

    /// Applies one activity using its receive timestamp, then advances the
    /// policy to `now_ms`.
    ///
    /// Equality is intentionally expired: a packet observed exactly at the
    /// prior deadline cannot revive the lease.
    pub(crate) fn handle(
        &mut self,
        request: LeaseRequest<Client>,
        now_ms: u64,
    ) -> LeaseDecision<Client> {
        if self.is_expired_at(request.observed_at_ms) {
            let client = self.disarm().expect("expired lease has an owner");
            return LeaseDecision::Trip {
                client,
                reason: TripReason::Timeout,
            };
        }

        let decision = match request.activity {
            LeaseActivity::Acquire => self.acquire(request.client, request.observed_at_ms, now_ms),
            LeaseActivity::KeepAlive => {
                self.keep_alive(request.client, request.observed_at_ms, now_ms)
            }
            LeaseActivity::Logoff => {
                if self.active_client == Some(request.client) {
                    let client = self.disarm().expect("matching owner is active");
                    LeaseDecision::Trip {
                        client,
                        reason: TripReason::OwnerLogoff,
                    }
                } else {
                    LeaseDecision::Rejected(LeaseRejection::NoActiveLease)
                }
            }
        };

        if !matches!(decision, LeaseDecision::Trip { .. }) && self.is_expired_at(now_ms) {
            let client = self.disarm().expect("expired lease has an owner");
            return LeaseDecision::Trip {
                client,
                reason: TripReason::Timeout,
            };
        }
        decision
    }

    pub(crate) fn on_timer(&mut self, now_ms: u64) -> Option<LeaseDecision<Client>> {
        if !self.is_expired_at(now_ms) {
            return None;
        }
        let client = self.disarm().expect("expired lease has an owner");
        Some(LeaseDecision::Trip {
            client,
            reason: TripReason::Timeout,
        })
    }

    fn acquire(
        &mut self,
        client: Client,
        observed_at_ms: u64,
        now_ms: u64,
    ) -> LeaseDecision<Client> {
        match self.active_client {
            Some(owner) if owner != client => {
                LeaseDecision::Rejected(LeaseRejection::OwnedByAnotherClient)
            }
            Some(_) => self.renew(observed_at_ms, now_ms, true),
            None => {
                let permit = LeasePermit::new(
                    LeaseEpoch::new(self.take_epoch()),
                    observed_at_ms.saturating_add(self.timeout_ms),
                );
                if !permit.is_valid_at(now_ms) {
                    return LeaseDecision::Rejected(LeaseRejection::ObservationExpired);
                }
                self.active_client = Some(client);
                self.permit = Some(permit);
                LeaseDecision::Acquired(permit)
            }
        }
    }

    fn keep_alive(
        &mut self,
        client: Client,
        observed_at_ms: u64,
        now_ms: u64,
    ) -> LeaseDecision<Client> {
        match self.active_client {
            Some(owner) if owner == client => self.renew(observed_at_ms, now_ms, false),
            Some(_) => LeaseDecision::Rejected(LeaseRejection::OwnedByAnotherClient),
            None => LeaseDecision::Rejected(LeaseRejection::NoActiveLease),
        }
    }

    fn renew(&mut self, observed_at_ms: u64, now_ms: u64, grant: bool) -> LeaseDecision<Client> {
        let mut permit = self.permit.expect("active owner has a permit");
        permit = permit.with_expiry(observed_at_ms.saturating_add(self.timeout_ms));
        if !permit.is_valid_at(now_ms) {
            let client = self.disarm().expect("renewed lease has an owner");
            return LeaseDecision::Trip {
                client,
                reason: TripReason::Timeout,
            };
        }
        self.permit = Some(permit);
        if grant {
            LeaseDecision::Renewed(permit)
        } else {
            LeaseDecision::Maintained(permit)
        }
    }

    fn is_expired_at(&self, at_ms: u64) -> bool {
        self.permit.is_some_and(|permit| !permit.is_valid_at(at_ms))
    }

    fn disarm(&mut self) -> Option<Client> {
        self.permit = None;
        self.active_client.take()
    }

    fn take_epoch(&mut self) -> u32 {
        let epoch = self.next_epoch;
        self.next_epoch = self.next_epoch.wrapping_add(1);
        if self.next_epoch == 0 {
            self.next_epoch = 1;
        }
        epoch
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT_MS: u64 = 30_000;

    fn request(client: u8, activity: LeaseActivity, observed_at_ms: u64) -> LeaseRequest<u8> {
        LeaseRequest {
            client,
            activity,
            observed_at_ms,
        }
    }

    #[test]
    fn mutating_command_acquires_a_free_lease() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let LeaseDecision::Acquired(permit) =
            policy.handle(request(7, LeaseActivity::Acquire, 100), 100)
        else {
            panic!("expected a permit");
        };

        assert_eq!(policy.active_client(), Some(7));
        assert_eq!(permit.epoch().get(), 1);
        assert_eq!(permit.expires_at_ms(), 30_100);
    }

    #[test]
    fn query_cannot_acquire_a_free_lease() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);

        assert_eq!(
            policy.handle(request(7, LeaseActivity::KeepAlive, 100), 100),
            LeaseDecision::Rejected(LeaseRejection::NoActiveLease)
        );
        assert_eq!(policy.active_client(), None);
    }

    #[test]
    fn other_client_cannot_refresh_or_replace_owner() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let _ = policy.handle(request(7, LeaseActivity::Acquire, 100), 100);

        assert_eq!(
            policy.handle(request(9, LeaseActivity::Acquire, 200), 200),
            LeaseDecision::Rejected(LeaseRejection::OwnedByAnotherClient)
        );
        assert_eq!(
            policy.handle(request(9, LeaseActivity::KeepAlive, 300), 300),
            LeaseDecision::Rejected(LeaseRejection::OwnedByAnotherClient)
        );
        assert_eq!(policy.active_client(), Some(7));
        assert_eq!(policy.next_deadline_ms(), Some(30_100));
    }

    #[derive(Debug, Clone, Copy)]
    struct ControllerEndpoint {
        address: u8,
        port: u16,
    }

    impl PartialEq for ControllerEndpoint {
        fn eq(&self, other: &Self) -> bool {
            self.address == other.address
        }
    }

    impl Eq for ControllerEndpoint {}

    #[test]
    fn one_controller_can_rotate_its_udp_source_port() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let first = ControllerEndpoint {
            address: 7,
            port: 30_001,
        };
        let rotated_port = ControllerEndpoint {
            address: 7,
            port: 30_002,
        };

        let LeaseDecision::Acquired(permit) = policy.handle(
            LeaseRequest {
                client: first,
                activity: LeaseActivity::Acquire,
                observed_at_ms: 100,
            },
            100,
        ) else {
            panic!("expected acquisition");
        };
        let LeaseDecision::Renewed(renewed) = policy.handle(
            LeaseRequest {
                client: rotated_port,
                activity: LeaseActivity::Acquire,
                observed_at_ms: 200,
            },
            200,
        ) else {
            panic!("expected same controller to renew after port rotation");
        };

        assert_ne!(first.port, rotated_port.port);
        assert_eq!(permit.epoch(), renewed.epoch());
        assert_eq!(renewed.expires_at_ms(), 30_200);
    }

    #[test]
    fn owner_query_renews_same_epoch() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let LeaseDecision::Acquired(first) =
            policy.handle(request(7, LeaseActivity::Acquire, 100), 100)
        else {
            panic!("expected first permit");
        };
        let LeaseDecision::Maintained(second) =
            policy.handle(request(7, LeaseActivity::KeepAlive, 5_000), 5_000)
        else {
            panic!("expected renewed permit");
        };

        assert_eq!(first.epoch(), second.epoch());
        assert_eq!(second.expires_at_ms(), 35_000);
    }

    #[test]
    fn owner_mutation_is_a_renewal_not_a_new_acquisition() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let LeaseDecision::Acquired(first) =
            policy.handle(request(7, LeaseActivity::Acquire, 100), 100)
        else {
            panic!("expected acquisition");
        };

        let LeaseDecision::Renewed(renewed) =
            policy.handle(request(7, LeaseActivity::Acquire, 500), 500)
        else {
            panic!("expected renewal");
        };

        assert_eq!(renewed.epoch(), first.epoch());
        assert_eq!(renewed.expires_at_ms(), 30_500);
    }

    #[test]
    fn observation_exactly_at_deadline_trips_before_response() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let _ = policy.handle(request(7, LeaseActivity::Acquire, 100), 100);

        assert_eq!(
            policy.handle(request(7, LeaseActivity::Acquire, 30_100), 30_100),
            LeaseDecision::Trip {
                client: 7,
                reason: TripReason::Timeout,
            }
        );
        assert_eq!(policy.active_client(), None);
    }

    #[test]
    fn pre_deadline_observation_consumed_after_renewed_deadline_trips() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let _ = policy.handle(request(7, LeaseActivity::Acquire, 100), 100);

        assert_eq!(
            policy.handle(request(7, LeaseActivity::Acquire, 20_000), 50_000),
            LeaseDecision::Trip {
                client: 7,
                reason: TripReason::Timeout,
            }
        );
        assert_eq!(policy.active_client(), None);
    }

    #[test]
    fn rejected_non_owner_observation_cannot_delay_advancing_to_current_time() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let _ = policy.handle(request(7, LeaseActivity::Acquire, 100), 100);

        assert_eq!(
            policy.handle(request(9, LeaseActivity::KeepAlive, 20_000), 40_000),
            LeaseDecision::Trip {
                client: 7,
                reason: TripReason::Timeout,
            }
        );
        assert_eq!(policy.active_client(), None);
    }

    #[test]
    fn owner_logoff_trips_and_non_owner_logoff_is_noop() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let _ = policy.handle(request(7, LeaseActivity::Acquire, 100), 100);

        assert_eq!(
            policy.handle(request(9, LeaseActivity::Logoff, 200), 200),
            LeaseDecision::Rejected(LeaseRejection::NoActiveLease)
        );
        assert_eq!(policy.active_client(), Some(7));
        assert_eq!(
            policy.handle(request(7, LeaseActivity::Logoff, 300), 300),
            LeaseDecision::Trip {
                client: 7,
                reason: TripReason::OwnerLogoff,
            }
        );
    }

    #[test]
    fn timer_fires_once_and_disarms() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        let _ = policy.handle(request(7, LeaseActivity::Acquire, 100), 100);

        assert_eq!(policy.on_timer(30_099), None);
        assert_eq!(
            policy.on_timer(30_100),
            Some(LeaseDecision::Trip {
                client: 7,
                reason: TripReason::Timeout,
            })
        );
        assert_eq!(policy.on_timer(60_000), None);
    }
}
