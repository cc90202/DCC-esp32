//! Validated WiFi station credentials.

use core::fmt;

use heapless::String;

const SSID_MAX_LEN: usize = 32;
const PASSWORD_CAPACITY: usize = 64;
const PASSWORD_MIN_LEN: usize = 8;
const PASSWORD_MAX_LEN: usize = 63;

/// Validation error for WiFi station credentials.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CredentialError {
    EmptySsid,
    SsidTooLong,
    PasswordTooShort,
    PasswordTooLong,
}

impl CredentialError {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::EmptySsid => "SSID is empty",
            Self::SsidTooLong => "SSID is too long",
            Self::PasswordTooShort => "WiFi password is too short",
            Self::PasswordTooLong => "WiFi password is too long",
        }
    }
}

/// Validated WPA/WPA2 station credentials.
#[derive(PartialEq, Eq)]
pub struct WifiCredentials {
    ssid: String<SSID_MAX_LEN>,
    password: String<PASSWORD_CAPACITY>,
}

impl WifiCredentials {
    /// Create validated WPA/WPA2 credentials.
    ///
    /// Open networks are intentionally rejected in the first provisioning
    /// version. The password storage capacity is 64 bytes, but accepted WPA
    /// passphrase length is 8..=63 bytes.
    pub fn new(ssid: &str, password: &str) -> Result<Self, CredentialError> {
        if ssid.is_empty() {
            return Err(CredentialError::EmptySsid);
        }
        let ssid = String::try_from(ssid).map_err(|_| CredentialError::SsidTooLong)?;

        let password_len = password.len();
        if password_len < PASSWORD_MIN_LEN {
            return Err(CredentialError::PasswordTooShort);
        }
        if password_len > PASSWORD_MAX_LEN {
            return Err(CredentialError::PasswordTooLong);
        }
        let password = String::try_from(password).map_err(|_| CredentialError::PasswordTooLong)?;

        Ok(Self { ssid, password })
    }

    /// Network SSID.
    #[must_use]
    pub fn ssid(&self) -> &str {
        self.ssid.as_str()
    }

    /// WPA/WPA2 passphrase.
    #[must_use]
    pub fn password(&self) -> &str {
        self.password.as_str()
    }
}

impl fmt::Debug for WifiCredentials {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WifiCredentials")
            .field("ssid", &self.ssid())
            .field("password", &"<redacted>")
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_credentials_are_accepted() {
        let credentials = WifiCredentials::new("DCC-Lab", "password123").unwrap();

        assert_eq!(credentials.ssid(), "DCC-Lab");
        assert_eq!(credentials.password(), "password123");
    }

    #[test]
    fn empty_ssid_is_rejected() {
        assert_eq!(
            WifiCredentials::new("", "password123"),
            Err(CredentialError::EmptySsid)
        );
    }

    #[test]
    fn ssid_longer_than_32_bytes_is_rejected() {
        assert_eq!(
            WifiCredentials::new("123456789012345678901234567890123", "password123"),
            Err(CredentialError::SsidTooLong)
        );
    }

    #[test]
    fn password_shorter_than_8_bytes_is_rejected() {
        assert_eq!(
            WifiCredentials::new("DCC-Lab", "1234567"),
            Err(CredentialError::PasswordTooShort)
        );
    }

    #[test]
    fn password_longer_than_63_bytes_is_rejected() {
        assert_eq!(
            WifiCredentials::new(
                "DCC-Lab",
                "1234567890123456789012345678901234567890123456789012345678901234",
            ),
            Err(CredentialError::PasswordTooLong)
        );
    }

    #[test]
    fn password_with_63_bytes_is_accepted() {
        let password = "123456789012345678901234567890123456789012345678901234567890123";
        let credentials = WifiCredentials::new("DCC-Lab", password).unwrap();

        assert_eq!(credentials.password(), password);
    }

    #[test]
    fn debug_redacts_password() {
        let credentials = WifiCredentials::new("DCC-Lab", "secret123").unwrap();
        let debug = format!("{credentials:?}");

        assert!(debug.contains("DCC-Lab"));
        assert!(debug.contains("<redacted>"));
        assert!(!debug.contains("secret123"));
    }
}
