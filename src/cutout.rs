//! Shared contract for DCC packet emission and RailCom cutout handling.

/// RailCom cutout policy attached to a DCC frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CutoutMode {
    #[default]
    None,
    Telemetry,
    PomWrite,
    PomRead,
}

/// Receive window within one RailCom cutout.
///
/// This belongs to the cutout contract rather than to the RailCom parser: the
/// track-output timer, UART capture and parser all need the same identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomChannel {
    Channel1,
    Channel2,
}

impl RailcomChannel {
    pub(crate) const fn index(self) -> usize {
        match self {
            Self::Channel1 => 0,
            Self::Channel2 => 1,
        }
    }
}

/// Wire/ISR-facing numeric id: 1 = CH1, 2 = CH2; zero is reserved for unset.
impl From<RailcomChannel> for u8 {
    #[inline(always)]
    fn from(channel: RailcomChannel) -> Self {
        match channel {
            RailcomChannel::Channel1 => 1,
            RailcomChannel::Channel2 => 2,
        }
    }
}

impl TryFrom<u8> for RailcomChannel {
    type Error = ();

    #[inline]
    fn try_from(id: u8) -> Result<Self, Self::Error> {
        match id {
            1 => Ok(Self::Channel1),
            2 => Ok(Self::Channel2),
            _ => Err(()),
        }
    }
}

/// Wraparound-safe identity of a DCC packet boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct PacketSequence(u32);

impl PacketSequence {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u32 {
        self.0
    }

    #[must_use]
    pub const fn age_since(self, earlier: Self) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }

    #[must_use]
    pub const fn is_within(self, earlier: Self, window: u32) -> bool {
        self.age_since(earlier) <= window
    }

    /// Wraparound-safe ordering for observations less than half the `u32`
    /// sequence space apart.
    #[must_use]
    pub const fn is_at_or_after(self, earlier: Self) -> bool {
        self.age_since(earlier) < (1 << 31)
    }
}

impl From<u32> for PacketSequence {
    fn from(value: u32) -> Self {
        Self::new(value)
    }
}

impl From<PacketSequence> for u32 {
    fn from(sequence: PacketSequence) -> Self {
        sequence.value()
    }
}

/// Hardware timing contract for one DCC packet followed by a RailCom cutout.
///
/// RCN-217 §2.4 references every cutout timestamp to the zero crossing of the
/// last edge of the packet end bit: the transition that closes the fully
/// emitted end bit and opens the first preamble bit of the following packet.
/// The stream must actually EMIT that reference transition: after the end bit
/// it drives the first preamble half for [`timing::PREAMBLE_STUB_US`] before
/// the brake shorts the track at `CUTOUT_CONTROL_START_US`. In this design
/// the cutout window is padding between packets — the following packet then
/// starts with its full untouched 20-bit preamble, comfortably above the 12
/// post-cutout sync bits RCN-211 requires.
pub mod timing {
    /// Driven first-preamble-half stub emitted after the end bit so the
    /// decoder sees the RCN-217 reference edge. The brake overrides the rail
    /// from `CUTOUT_CONTROL_START_US`, so the stub only needs to outlast it.
    pub const PREAMBLE_STUB_US: u32 = 32;
    /// GPIO4 falls at the cutout-start timestamp (RCN-217 Tcs, 26-32 µs
    /// after the reference edge; mid-window for decoder margin).
    pub const CUTOUT_CONTROL_START_US: u32 = 28;
    /// GPIO4 rises when RailCom channel 2 ends (454-488 µs after the
    /// reference edge).
    pub const CUTOUT_CONTROL_END_US: u32 = 454;
    /// Duration of the active-low GPIO4 control pulse.
    pub const GPIO4_LOW_DURATION_US: u32 = CUTOUT_CONTROL_END_US - CUTOUT_CONTROL_START_US;
    /// Start and end of RailCom channel 1, relative to the final DCC edge.
    pub const CHANNEL1_START_US: u32 = 80;
    pub const CHANNEL1_END_US: u32 = 177;
    /// Start and end of RailCom channel 2, relative to the final DCC edge.
    pub const CHANNEL2_START_US: u32 = 193;
    pub const CHANNEL2_END_US: u32 = 454;
    /// UART capture opens before channel 1; the guard reserves ISR settling time.
    pub const RX_CAPTURE_START_US: u32 = 40;
    pub const RX_ISR_GUARD_US: u32 = 40;
    pub const RX_CHANNEL_SPLIT_US: u32 = 185;

    const _: () = assert!(CUTOUT_CONTROL_START_US >= 26 && CUTOUT_CONTROL_START_US <= 32);
    const _: () = assert!(CUTOUT_CONTROL_END_US >= 454 && CUTOUT_CONTROL_END_US <= 488);
    const _: () = assert!(GPIO4_LOW_DURATION_US == 426);
    const _: () = assert!(PREAMBLE_STUB_US >= CUTOUT_CONTROL_START_US);
    const _: () = assert!(PREAMBLE_STUB_US < CUTOUT_CONTROL_END_US);
    const _: () = assert!(CHANNEL1_START_US >= 75);
    const _: () = assert!(CHANNEL1_START_US < CHANNEL1_END_US);
    const _: () = assert!(CHANNEL1_END_US < CHANNEL2_START_US);
    const _: () = assert!(CHANNEL2_START_US < CHANNEL2_END_US);
    const _: () = assert!(CHANNEL2_END_US == CUTOUT_CONTROL_END_US);
    const _: () = assert!(RX_CAPTURE_START_US < CHANNEL1_START_US);
    const _: () = assert!(RX_ISR_GUARD_US < CHANNEL1_START_US);
    const _: () = assert!(RX_CHANNEL_SPLIT_US > CHANNEL1_END_US);
    const _: () = assert!(RX_CHANNEL_SPLIT_US < CHANNEL2_START_US);

    /// Timeline for one DCC packet followed by a RailCom cutout.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct CutoutTimeline {
        dcc_packet_duration_us: u32,
    }

    impl CutoutTimeline {
        #[must_use]
        pub const fn new(dcc_packet_duration_us: u32) -> Self {
            Self {
                dcc_packet_duration_us,
            }
        }

        #[must_use]
        pub const fn control_start_from_packet_start_us(self) -> u32 {
            self.reference_edge_from_packet_start_us() + CUTOUT_CONTROL_START_US
        }

        /// RCN-217 reference: the zero crossing that closes the fully emitted
        /// end bit and opens the driven preamble stub.
        #[must_use]
        pub const fn reference_edge_from_packet_start_us(self) -> u32 {
            self.dcc_packet_duration_us
        }

        #[must_use]
        pub const fn cycle_duration_us(self) -> u32 {
            self.reference_edge_from_packet_start_us() + CUTOUT_CONTROL_END_US
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn rcn217_timestamps_anchor_on_the_end_bit_reference_edge() {
            let timeline = CutoutTimeline::new(5_000);
            assert_eq!(timeline.reference_edge_from_packet_start_us(), 5_000);
            assert_eq!(timeline.control_start_from_packet_start_us(), 5_028);
            assert_eq!(timeline.cycle_duration_us(), 5_454);
            assert_eq!(CHANNEL1_START_US, 80);
            assert_eq!(CHANNEL1_END_US, 177);
            assert_eq!(CHANNEL2_START_US, 193);
            assert_eq!(CHANNEL2_END_US, CUTOUT_CONTROL_END_US);
        }
    }
}
