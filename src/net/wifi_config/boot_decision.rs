//! Store-backed boot decision glue.
//!
//! Loads stored credentials and the persistent override flag, feeds the pure
//! `decide_provisioning` policy, and clears the flag once it has been
//! observed. Generic over the store traits so it stays host-testable.

use super::credentials::WifiCredentials;
use super::provisioning_policy::{
    ProvisioningDecision, StoredCredentialsState, decide_provisioning,
};
use super::store::{ProvisioningFlagStore, StoreError, WifiCredentialsStore};

/// Load credentials and compute the boot provisioning decision.
///
/// Corrupt stored credentials become `StoredCredentialsState::Invalid`
/// (entering provisioning) instead of a boot error; a corrupt flag read is
/// treated as "not forced". The persistent flag is cleared as soon as it is
/// observed set, regardless of which provisioning reason wins.
pub fn load_wifi_credentials_and_decision<S>(
    store: &mut S,
    button_override: bool,
) -> Result<(ProvisioningDecision, Option<WifiCredentials>), StoreError>
where
    S: WifiCredentialsStore + ProvisioningFlagStore,
{
    let (stored_state, credentials) = match store.load() {
        Ok(Some(credentials)) => (StoredCredentialsState::Present, Some(credentials)),
        Ok(None) => (StoredCredentialsState::Missing, None),
        Err(StoreError::Corrupt) => (StoredCredentialsState::Invalid, None),
        Err(error) => return Err(error),
    };

    let persistent_override = match store.force_on_next_boot() {
        Ok(force) => force,
        Err(StoreError::Corrupt) => false,
        Err(error) => return Err(error),
    };

    let decision = decide_provisioning(stored_state, button_override, persistent_override);
    if persistent_override {
        store.clear_force_on_next_boot()?;
    }

    Ok((decision, credentials))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::net::wifi_config::ProvisioningReason;

    #[derive(Default)]
    struct FakeStore {
        stored: Option<(&'static str, &'static str)>,
        load_error: Option<StoreError>,
        flag: bool,
        flag_error: Option<StoreError>,
        flag_clears: u8,
    }

    impl WifiCredentialsStore for FakeStore {
        fn load(&mut self) -> Result<Option<WifiCredentials>, StoreError> {
            match self.load_error {
                Some(error) => Err(error),
                None => Ok(self
                    .stored
                    .map(|(ssid, password)| WifiCredentials::new(ssid, password).unwrap())),
            }
        }

        fn save(&mut self, _credentials: &WifiCredentials) -> Result<(), StoreError> {
            unimplemented!("not exercised by the boot decision")
        }

        fn clear(&mut self) -> Result<(), StoreError> {
            unimplemented!("not exercised by the boot decision")
        }
    }

    impl ProvisioningFlagStore for FakeStore {
        fn force_on_next_boot(&mut self) -> Result<bool, StoreError> {
            match self.flag_error {
                Some(error) => Err(error),
                None => Ok(self.flag),
            }
        }

        fn set_force_on_next_boot(&mut self) -> Result<(), StoreError> {
            unimplemented!("not exercised by the boot decision")
        }

        fn clear_force_on_next_boot(&mut self) -> Result<(), StoreError> {
            self.flag = false;
            self.flag_clears += 1;
            Ok(())
        }
    }

    const STORED: Option<(&str, &str)> = Some(("DCC-Lab", "password123"));

    #[test]
    fn valid_credentials_without_overrides_start_station_mode() {
        let mut store = FakeStore {
            stored: STORED,
            ..FakeStore::default()
        };

        let (decision, credentials) =
            load_wifi_credentials_and_decision(&mut store, false).unwrap();

        assert_eq!(decision, ProvisioningDecision::StationMode);
        assert_eq!(credentials.unwrap().ssid(), "DCC-Lab");
        assert_eq!(store.flag_clears, 0);
    }

    #[test]
    fn missing_credentials_enter_provisioning() {
        let mut store = FakeStore::default();

        let (decision, credentials) =
            load_wifi_credentials_and_decision(&mut store, false).unwrap();

        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::MissingCredentials)
        );
        assert!(credentials.is_none());
    }

    #[test]
    fn corrupt_credentials_enter_provisioning_instead_of_failing_boot() {
        let mut store = FakeStore {
            load_error: Some(StoreError::Corrupt),
            ..FakeStore::default()
        };

        let (decision, credentials) =
            load_wifi_credentials_and_decision(&mut store, false).unwrap();

        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::InvalidStoredCredentials)
        );
        assert!(credentials.is_none());
    }

    #[test]
    fn corrupt_flag_read_is_treated_as_not_forced() {
        let mut store = FakeStore {
            stored: STORED,
            flag_error: Some(StoreError::Corrupt),
            ..FakeStore::default()
        };

        let (decision, _) = load_wifi_credentials_and_decision(&mut store, false).unwrap();

        assert_eq!(decision, ProvisioningDecision::StationMode);
        assert_eq!(store.flag_clears, 0);
    }

    #[test]
    fn persistent_override_wins_and_is_cleared() {
        let mut store = FakeStore {
            stored: STORED,
            flag: true,
            ..FakeStore::default()
        };

        let (decision, _) = load_wifi_credentials_and_decision(&mut store, false).unwrap();

        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::PersistentOverride)
        );
        assert_eq!(store.flag_clears, 1);
        assert!(!store.flag);
    }

    #[test]
    fn button_override_wins_but_pending_flag_is_still_cleared() {
        let mut store = FakeStore {
            stored: STORED,
            flag: true,
            ..FakeStore::default()
        };

        let (decision, _) = load_wifi_credentials_and_decision(&mut store, true).unwrap();

        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride)
        );
        assert_eq!(store.flag_clears, 1);
    }

    #[test]
    fn non_corrupt_load_error_propagates() {
        let mut store = FakeStore {
            load_error: Some(StoreError::FlashRead),
            ..FakeStore::default()
        };

        assert_eq!(
            load_wifi_credentials_and_decision(&mut store, false),
            Err(StoreError::FlashRead)
        );
    }
}
