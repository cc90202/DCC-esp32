//! Pure client-liveness policy for safety-sensitive network control.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ClientTimedOut<Client> {
    pub(crate) client: Client,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ClientSafetyPolicy<Client> {
    active_client: Option<Client>,
    last_activity_ms: u64,
    timeout_ms: u64,
}

impl<Client: Copy> ClientSafetyPolicy<Client> {
    #[must_use]
    pub(crate) const fn new(timeout_ms: u64) -> Self {
        Self {
            active_client: None,
            last_activity_ms: 0,
            timeout_ms,
        }
    }

    #[must_use]
    pub(crate) const fn active_client(self) -> Option<Client> {
        self.active_client
    }

    pub(crate) fn observe_packet(&mut self, client: Client, now_ms: u64) {
        self.active_client = Some(client);
        self.last_activity_ms = now_ms;
    }

    #[must_use]
    pub(crate) fn next_wake_ms(self, now_ms: u64) -> u64 {
        if self.active_client.is_some() {
            self.last_activity_ms
                .saturating_add(self.timeout_ms)
                .saturating_add(1)
        } else {
            now_ms.saturating_add(self.timeout_ms)
        }
    }

    pub(crate) fn on_timer(&mut self, now_ms: u64) -> Option<ClientTimedOut<Client>> {
        let client = self.active_client?;
        let elapsed = now_ms.saturating_sub(self.last_activity_ms);
        if elapsed < self.timeout_ms {
            return None;
        }
        self.active_client = None;
        Some(ClientTimedOut { client })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT_MS: u64 = 30_000;

    #[test]
    fn first_packet_arms_client_deadline() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);

        policy.observe_packet(7u8, 100);
        assert_eq!(policy.active_client(), Some(7));
        assert_eq!(policy.next_wake_ms(100), 30_101);
    }

    #[test]
    fn packet_from_same_client_refreshes_deadline() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        policy.observe_packet(7u8, 100);

        policy.observe_packet(7u8, 5_000);
        assert_eq!(policy.on_timer(30_100), None);
        assert_eq!(policy.on_timer(35_000), Some(ClientTimedOut { client: 7 }));
    }

    #[test]
    fn packet_from_new_client_replaces_previous_owner() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        policy.observe_packet(7u8, 100);

        policy.observe_packet(9u8, 200);
        assert_eq!(policy.active_client(), Some(9));
    }

    #[test]
    fn stale_timer_does_not_stop_active_client_early() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        policy.observe_packet(7u8, 10_000);

        assert_eq!(policy.on_timer(39_999), None);
        assert_eq!(policy.active_client(), Some(7));
    }

    #[test]
    fn timeout_fires_once_and_disarms_client() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        policy.observe_packet(7u8, 10_000);

        assert_eq!(policy.on_timer(40_000), Some(ClientTimedOut { client: 7 }));
        assert_eq!(policy.on_timer(70_000), None);
        assert_eq!(policy.active_client(), None);
    }

    #[test]
    fn deadline_calculation_saturates_instead_of_wrapping() {
        let mut policy = ClientSafetyPolicy::new(TIMEOUT_MS);
        policy.observe_packet(7u8, u64::MAX - 10);

        assert_eq!(policy.next_wake_ms(u64::MAX), u64::MAX);
    }
}
