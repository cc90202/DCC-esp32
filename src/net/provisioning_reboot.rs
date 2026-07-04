//! Host-testable post-save reboot policy for WiFi provisioning.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PostSaveAction {
    Reboot,
    StayInProvisioning,
}

#[must_use]
pub(crate) const fn post_save_action(
    credentials_saved: bool,
    success_response_sent: bool,
) -> PostSaveAction {
    if credentials_saved && success_response_sent {
        PostSaveAction::Reboot
    } else {
        PostSaveAction::StayInProvisioning
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saved_credentials_and_sent_response_reboot() {
        assert_eq!(post_save_action(true, true), PostSaveAction::Reboot);
    }

    #[test]
    fn saved_credentials_without_sent_response_stays_in_provisioning() {
        assert_eq!(
            post_save_action(true, false),
            PostSaveAction::StayInProvisioning
        );
    }

    #[test]
    fn failed_save_never_reboots_even_if_response_was_sent() {
        assert_eq!(
            post_save_action(false, true),
            PostSaveAction::StayInProvisioning
        );
        assert_eq!(
            post_save_action(false, false),
            PostSaveAction::StayInProvisioning
        );
    }
}
