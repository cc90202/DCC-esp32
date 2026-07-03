//! Pure boot decision policy for WiFi provisioning.

/// State returned by the persistent credential store.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoredCredentialsState {
    Present,
    Missing,
    Invalid,
}

/// Boot mode decision for WiFi startup.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvisioningDecision {
    StationMode,
    ProvisioningMode(ProvisioningReason),
}

/// Reason for entering provisioning mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvisioningReason {
    MissingCredentials,
    ButtonOverride,
    InvalidStoredCredentials,
}

/// Decide whether boot should start station mode or provisioning mode.
///
/// Runtime connection failures are intentionally not an input: after valid
/// credentials are loaded, failed association remains a station-mode retry
/// problem until the operator explicitly requests setup.
#[must_use]
pub const fn decide_provisioning(
    stored_credentials: StoredCredentialsState,
    force_button_override: bool,
) -> ProvisioningDecision {
    if force_button_override {
        return ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride);
    }

    match stored_credentials {
        StoredCredentialsState::Present => ProvisioningDecision::StationMode,
        StoredCredentialsState::Missing => {
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::MissingCredentials)
        }
        StoredCredentialsState::Invalid => {
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::InvalidStoredCredentials)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_credentials_enter_provisioning() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Missing, false),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::MissingCredentials)
        );
    }

    #[test]
    fn button_override_wins_over_valid_credentials() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Present, true),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride)
        );
    }

    #[test]
    fn invalid_stored_credentials_enter_provisioning() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Invalid, false),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::InvalidStoredCredentials)
        );
    }

    #[test]
    fn valid_credentials_start_station_mode() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Present, false),
            ProvisioningDecision::StationMode
        );
    }
}
