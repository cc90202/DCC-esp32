//! Store-backed boot decision glue.
//!
//! Loads stored credentials and the persistent override flag, feeds the pure
//! `decide_provisioning` policy, and clears the flag once it has been
//! observed. Generic over the store traits so it stays host-testable.

use crate::credentials::WifiCredentials;
use crate::provisioning_policy::{
    ProvisioningDecision, StoredCredentialsState, decide_provisioning,
};
use crate::store::{ProvisioningFlagStore, StoreError, WifiCredentialsStore};

/// What a boot does with the persistent "enter setup" flag once read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetupFlag {
    /// Clear it: this boot acts on it.
    Consume,
    /// Leave it set: a recovery boot must not use up the request.
    Keep,
    /// Leave it set, and save a button press into it: a trial boot restarts
    /// before setup could open, so the press is kept for the boot after.
    Trial,
}

/// Load credentials and compute the boot provisioning decision.
///
/// Corrupt stored credentials become `StoredCredentialsState::Invalid`
/// (entering provisioning) instead of a boot error; a corrupt flag read is
/// treated as "not forced"; any other storage error is returned. With
/// [`SetupFlag::Consume`] the persistent flag is cleared as soon as it is
/// observed set, regardless of which provisioning reason wins; a clear that
/// fails is ignored. With [`SetupFlag::Trial`] a button press is saved as the
/// persistent flag, for the boot after the restart; a save that fails is
/// ignored too. [`SetupFlag::Keep`] writes nothing.
pub fn load_wifi_credentials_and_decision<S>(
    store: &mut S,
    button_override: bool,
    flag: SetupFlag,
) -> Result<
    (ProvisioningDecision, Option<WifiCredentials>),
    StoreError<<S as WifiCredentialsStore>::Error>,
>
where
    S: WifiCredentialsStore + ProvisioningFlagStore<Error = <S as WifiCredentialsStore>::Error>,
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
    match flag {
        // A flag that cannot be cleared only means setup opens again on the
        // next boot. Failing the boot instead would restart into the same
        // failure, again and again.
        SetupFlag::Consume if persistent_override => {
            let _ = store.clear_force_on_next_boot();
        }
        // A trial boot leaves its trial and restarts before setup opens, so a
        // button press is kept in flash for the boot that follows. A failed
        // save only loses the setup request: failing the boot would restart
        // a trial image, and the bootloader would roll it back for good.
        SetupFlag::Trial if button_override && !persistent_override => {
            let _ = store.set_force_on_next_boot();
        }
        SetupFlag::Consume | SetupFlag::Keep | SetupFlag::Trial => {}
    }

    Ok((decision, credentials))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ProvisioningReason;
    use embedded_storage::nor_flash::NorFlashErrorKind;

    #[derive(Default)]
    struct FakeStore {
        stored: Option<(&'static str, &'static str)>,
        load_error: Option<StoreError<NorFlashErrorKind>>,
        flag: bool,
        flag_error: Option<StoreError<NorFlashErrorKind>>,
        flag_clears: u8,
        clear_fails: bool,
        set_fails: bool,
    }

    impl WifiCredentialsStore for FakeStore {
        type Error = NorFlashErrorKind;

        fn load(&mut self) -> Result<Option<WifiCredentials>, StoreError<Self::Error>> {
            match self.load_error {
                Some(error) => Err(error),
                None => Ok(self
                    .stored
                    .map(|(ssid, password)| WifiCredentials::new(ssid, password).unwrap())),
            }
        }

        fn save(&mut self, _credentials: &WifiCredentials) -> Result<(), StoreError<Self::Error>> {
            unreachable!("not exercised by the boot decision")
        }

        fn clear(&mut self) -> Result<(), StoreError<Self::Error>> {
            unreachable!("not exercised by the boot decision")
        }
    }

    impl ProvisioningFlagStore for FakeStore {
        type Error = NorFlashErrorKind;

        fn force_on_next_boot(&mut self) -> Result<bool, StoreError<Self::Error>> {
            match self.flag_error {
                Some(error) => Err(error),
                None => Ok(self.flag),
            }
        }

        fn set_force_on_next_boot(&mut self) -> Result<(), StoreError<Self::Error>> {
            if self.set_fails {
                return Err(StoreError::FlashWrite(NorFlashErrorKind::Other));
            }
            self.flag = true;
            Ok(())
        }

        fn clear_force_on_next_boot(&mut self) -> Result<(), StoreError<Self::Error>> {
            if self.clear_fails {
                return Err(StoreError::FlashWrite(NorFlashErrorKind::Other));
            }
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
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume).unwrap();

        assert_eq!(decision, ProvisioningDecision::StationMode);
        assert_eq!(credentials.unwrap().ssid(), "DCC-Lab");
        assert_eq!(store.flag_clears, 0);
    }

    #[test]
    fn missing_credentials_enter_provisioning() {
        let mut store = FakeStore::default();

        let (decision, credentials) =
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume).unwrap();

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
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume).unwrap();

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

        let (decision, _) =
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume).unwrap();

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

        let (decision, _) =
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume).unwrap();

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

        let (decision, _) =
            load_wifi_credentials_and_decision(&mut store, true, SetupFlag::Consume).unwrap();

        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride)
        );
        assert_eq!(store.flag_clears, 1);
    }

    #[test]
    fn non_corrupt_load_error_propagates() {
        let mut store = FakeStore {
            load_error: Some(StoreError::FlashRead(NorFlashErrorKind::Other)),
            ..FakeStore::default()
        };

        assert_eq!(
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume),
            Err(StoreError::FlashRead(NorFlashErrorKind::Other))
        );
    }

    #[test]
    fn a_button_press_on_a_trial_boot_is_kept_for_the_next_boot() {
        // A failed save only loses the setup request; the boot goes on.
        for set_fails in [false, true] {
            let mut store = FakeStore {
                stored: STORED,
                set_fails,
                ..FakeStore::default()
            };
            let (decision, _) =
                load_wifi_credentials_and_decision(&mut store, true, SetupFlag::Trial).unwrap();
            assert_eq!(
                decision,
                ProvisioningDecision::ProvisioningMode(ProvisioningReason::ButtonOverride)
            );
            assert_eq!(
                store.flag, !set_fails,
                "the confirmed image must open setup after the restart"
            );
        }
    }

    #[test]
    fn a_recovery_boot_saves_no_button_press() {
        // Recovery stays in recovery: a press saved here would open setup
        // on the next normal boot, after the USB repair.
        let mut store = FakeStore {
            stored: STORED,
            ..FakeStore::default()
        };
        load_wifi_credentials_and_decision(&mut store, true, SetupFlag::Keep).unwrap();
        assert!(!store.flag);
    }

    #[test]
    fn a_flag_that_cannot_be_cleared_does_not_fail_the_boot() {
        let mut store = FakeStore {
            stored: STORED,
            flag: true,
            clear_fails: true,
            ..FakeStore::default()
        };
        let (decision, _) =
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Consume).unwrap();
        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::PersistentOverride)
        );
    }

    #[test]
    fn a_trial_boot_keeps_the_flag_but_still_reports_flash_errors() {
        let mut store = FakeStore {
            stored: STORED,
            flag: true,
            ..FakeStore::default()
        };
        let (decision, _) =
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Keep).unwrap();
        assert_eq!(
            decision,
            ProvisioningDecision::ProvisioningMode(ProvisioningReason::PersistentOverride)
        );
        assert_eq!(store.flag_clears, 0);
        assert!(store.flag);

        let mut store = FakeStore {
            stored: STORED,
            flag_error: Some(StoreError::FlashRead(NorFlashErrorKind::Other)),
            ..FakeStore::default()
        };
        assert_eq!(
            load_wifi_credentials_and_decision(&mut store, false, SetupFlag::Keep),
            Err(StoreError::FlashRead(NorFlashErrorKind::Other))
        );
    }
}
