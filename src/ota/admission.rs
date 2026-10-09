//! Update/setup admission policy; evaluate and acquire under the same critical section.
use super::Error;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Operation {
    Update,
    Provisioning,
}

pub struct Admission {
    pub available: bool,
    pub pending: bool,
    pub boot_ready: bool,
    pub armed: bool,
    pub busy: bool,
    pub track_on: bool,
    pub backoff_until: u64,
    pub next_check: u64,
}

impl Admission {
    pub fn check(&self, now: u64, operation: Operation) -> Result<(), Error> {
        if !self.available {
            return Err(Error::Unavailable);
        }
        if self.pending {
            return Err(Error::PendingVerify);
        }
        if !self.boot_ready || !self.armed {
            return Err(Error::BootNotReady);
        }
        if self.busy || now < self.backoff_until {
            return Err(Error::Busy);
        }
        if operation == Operation::Update {
            if now < self.next_check {
                return Err(Error::Busy);
            }
            if self.track_on {
                return Err(Error::TrackOn);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn admission_preserves_precedence_and_update_setup_boundaries() {
        let mut admission = Admission {
            available: false,
            pending: true,
            boot_ready: false,
            armed: false,
            busy: true,
            track_on: true,
            backoff_until: 1000,
            next_check: 2000,
        };
        for (available, pending, ready, armed, busy, expected) in [
            (false, true, false, false, true, Error::Unavailable),
            (true, true, false, false, true, Error::PendingVerify),
            (true, false, false, true, true, Error::BootNotReady),
            (true, false, true, false, true, Error::BootNotReady),
            (true, false, true, true, true, Error::Busy),
        ] {
            admission.available = available;
            admission.pending = pending;
            admission.boot_ready = ready;
            admission.armed = armed;
            admission.busy = busy;
            for operation in [Operation::Update, Operation::Provisioning] {
                assert_eq!(admission.check(3000, operation), Err(expected));
            }
        }
        admission.busy = false;
        for operation in [Operation::Update, Operation::Provisioning] {
            assert_eq!(admission.check(999, operation), Err(Error::Busy));
        }
        assert_eq!(admission.check(1000, Operation::Provisioning), Ok(()));
        assert_eq!(admission.check(1999, Operation::Update), Err(Error::Busy));
        assert_eq!(
            admission.check(2000, Operation::Update),
            Err(Error::TrackOn)
        );
        admission.track_on = false;
        assert_eq!(admission.check(2000, Operation::Update), Ok(()));
    }
}
