//! Pure value objects shared by controller authority and DCC consumers.

/// Registry for the lease accepted at mutation boundaries.
///
/// Gated `any(test, ...)` like every other policy module: the target-only form
/// it carried before this move meant its tests were never compiled, let alone
/// run.
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod lease_registry;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LeaseEpoch(u32);

impl LeaseEpoch {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LeasePermit {
    epoch: LeaseEpoch,
    expires_at_ms: u64,
}

impl LeasePermit {
    #[must_use]
    pub const fn new(epoch: LeaseEpoch, expires_at_ms: u64) -> Self {
        Self {
            epoch,
            expires_at_ms,
        }
    }

    #[must_use]
    pub const fn with_expiry(self, expires_at_ms: u64) -> Self {
        Self {
            epoch: self.epoch,
            expires_at_ms,
        }
    }

    #[must_use]
    pub const fn epoch(self) -> LeaseEpoch {
        self.epoch
    }

    #[must_use]
    pub const fn expires_at_ms(self) -> u64 {
        self.expires_at_ms
    }

    #[must_use]
    pub const fn is_valid_at(self, now_ms: u64) -> bool {
        now_ms < self.expires_at_ms
    }
}
