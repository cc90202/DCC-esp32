//! Locomotive speed as the command station holds it, independent of the
//! packet format that later carries it.

/// Speed format for a locomotive slot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpeedFormat {
    /// 28 speed steps (NMRA S-9.2 basic speed and direction instruction).
    Speed28,
    /// 126 speed steps (NMRA S-9.2.1 advanced operations, 128-step mode).
    Speed128,
}

impl SpeedFormat {
    /// Highest logical speed step of this format; step 0 is stop.
    #[must_use]
    pub const fn max_step(self) -> u8 {
        match self {
            Self::Speed28 => 28,
            Self::Speed128 => 126,
        }
    }
}

/// Validated logical (runtime) speed value for a locomotive slot.
///
/// "Logical" speed is the scheduler's internal protocol-agnostic representation:
/// - `0` = stop
/// - `1..=`[`SpeedFormat::max_step`] = speed steps
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
        (value <= format.max_step()).then_some(Self { value, format })
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

    /// Whether this speed is stop.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.value == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn logical_speed_accepts_up_to_the_top_step_of_its_format() {
        for (format, top) in [(SpeedFormat::Speed28, 28), (SpeedFormat::Speed128, 126)] {
            let speed = LogicalSpeed::new(top, format).expect("top step is valid");
            assert_eq!((speed.value(), speed.format()), (top, format));
            assert_eq!(LogicalSpeed::new(top + 1, format), None, "{format:?}");
        }
    }
}
