//! Locomotive speed as the command station holds it, independent of the
//! packet format that later carries it.

/// Speed format for a locomotive slot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpeedFormat {
    Speed28,
    Speed128,
}

/// Validated logical (runtime) speed value for a locomotive slot.
///
/// "Logical" speed is the scheduler's internal protocol-agnostic representation:
/// - `0` = stop
/// - Speed28  → `1..=28`  = speed steps
/// - Speed128 → `1..=126` = speed steps
///
/// The format is stored with the value so a speed validated for one format
/// cannot later be paired with another format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LogicalSpeed {
    value: u8,
    format: SpeedFormat,
}

impl LogicalSpeed {
    /// Validate `value` as a logical speed for `format`.
    #[must_use]
    pub fn new(value: u8, format: SpeedFormat) -> Option<Self> {
        let in_range = match format {
            SpeedFormat::Speed28 => value <= 28,
            SpeedFormat::Speed128 => value <= 126,
        };
        in_range.then_some(Self { value, format })
    }

    /// Build a stopped speed in the requested format.
    #[must_use]
    pub const fn zero(format: SpeedFormat) -> Self {
        Self { value: 0, format }
    }

    /// Return the raw logical speed value.
    #[must_use]
    pub const fn value(self) -> u8 {
        self.value
    }

    /// Return the format against which this speed was validated.
    #[must_use]
    pub const fn format(self) -> SpeedFormat {
        self.format
    }

    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.value == 0
    }
}
