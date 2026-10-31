//! Pure boot decision policy for WiFi provisioning.

/// State returned by the persistent credential store.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StoredCredentialsState {
    /// A valid record holds credentials.
    Present,
    /// Nothing was ever saved, or the record was cleared.
    Missing,
    /// A record is there but fails its checks.
    Invalid,
}

/// Boot mode decision for WiFi startup.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProvisioningDecision {
    /// Join the stored network.
    StationMode,
    /// Open the setup access point instead of starting the station.
    ProvisioningMode(ProvisioningReason),
}

/// Reason for entering provisioning mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProvisioningReason {
    /// No credentials are stored.
    MissingCredentials,
    /// The setup button was held at boot.
    ButtonOverride,
    /// The persistent flag in flash asked for setup on this boot.
    PersistentOverride,
    /// The stored record is corrupt.
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
    force_persistent_override: bool,
) -> ProvisioningDecision {
    if force_button_override {
        return ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride);
    }

    if force_persistent_override {
        return ProvisioningDecision::ProvisioningMode(ProvisioningReason::PersistentOverride);
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
            decide_provisioning(StoredCredentialsState::Missing, false, false),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::MissingCredentials)
        );
    }

    #[test]
    fn button_override_wins_over_valid_credentials() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Present, true, true),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride)
        );
    }

    #[test]
    fn persistent_override_enters_provisioning() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Present, false, true),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::PersistentOverride)
        );
    }

    #[test]
    fn invalid_stored_credentials_enter_provisioning() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Invalid, false, false),
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::InvalidStoredCredentials)
        );
    }

    #[test]
    fn valid_credentials_start_station_mode() {
        assert_eq!(
            decide_provisioning(StoredCredentialsState::Present, false, false),
            ProvisioningDecision::StationMode
        );
    }
}
