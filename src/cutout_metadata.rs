//! Cutout metadata shared between the track-output interrupt and RailCom.
//!
//! The interrupt packs the target address, cutout mode and POM request
//! presence of each cutout into one `u32` so it can be published through
//! atomic slots; the RailCom consumers unpack it on the task side.

use crate::cutout::{CutoutMode, PacketSequence};
use crate::dcc::PackedAddressFlags;
use crate::dcc::{DccAddress, PomRequestId};

/// Cutout event published by the track-output interrupt to the RailCom tasks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CutoutRuntimeEvent {
    /// A cutout window opened after the packet with `packet_sequence`.
    Opened {
        /// Sequence number of the packet that preceded the cutout.
        packet_sequence: PacketSequence,
        /// Kind of answer expected in this cutout.
        cutout: CutoutMode,
        /// Decoder addressed by the preceding packet, if it carried one.
        target_address: Option<DccAddress>,
        /// POM request the cutout answers, if the packet was a POM.
        pom_request_id: Option<PomRequestId>,
    },
}

/// Metadata recorded for the cutout that follows one DCC packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailcomPacketMetadata {
    /// Decoder addressed by the packet, if it carried one.
    pub target_address: Option<DccAddress>,
    /// Kind of answer expected in the cutout.
    pub cutout: CutoutMode,
    /// POM request the cutout answers, if the packet was a POM.
    pub pom_request_id: Option<PomRequestId>,
}

/// Bit-packed cutout metadata: `PackedAddressFlags` carries the target,
/// POM mode, and presence of the separately stored request identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PackedRailcomPacketMetadata(PackedAddressFlags);

// The RMT handler packs this from RAM and the cutout handler unpacks and
// repacks it; the helpers are forced inline so neither handler calls into
// flash for them.
impl PackedRailcomPacketMetadata {
    const CUTOUT_MODE_FIELD: u32 = 0;
    const CUTOUT_MODE_WIDTH: u32 = 2;
    const HAS_POM_REQUEST_ID_BIT: u32 = 2;
    #[inline(always)]
    pub(crate) const fn from_raw(raw: u32) -> Self {
        Self(PackedAddressFlags::from_raw(raw))
    }
    #[inline(always)]
    pub(crate) const fn raw(self) -> u32 {
        self.0.raw()
    }
    #[inline(always)]
    pub(crate) fn new(
        target_address: Option<DccAddress>,
        cutout: CutoutMode,
        pom_request_id: Option<PomRequestId>,
    ) -> Self {
        Self(
            PackedAddressFlags::new(target_address)
                .with_field(
                    Self::CUTOUT_MODE_FIELD,
                    Self::CUTOUT_MODE_WIDTH,
                    match cutout {
                        CutoutMode::None => 0,
                        CutoutMode::Telemetry => 1,
                        CutoutMode::PomWrite => 2,
                        CutoutMode::PomRead => 3,
                    },
                )
                .with_flag(Self::HAS_POM_REQUEST_ID_BIT, pom_request_id.is_some()),
        )
    }
    #[inline(always)]
    pub(crate) fn target_address(self) -> Option<DccAddress> {
        self.0.address()
    }
    #[inline(always)]
    pub(crate) const fn cutout(self) -> CutoutMode {
        match self
            .0
            .field(Self::CUTOUT_MODE_FIELD, Self::CUTOUT_MODE_WIDTH)
        {
            0 => CutoutMode::None,
            1 => CutoutMode::Telemetry,
            2 => CutoutMode::PomWrite,
            3 => CutoutMode::PomRead,
            _ => unreachable!(),
        }
    }
    #[inline(always)]
    pub(crate) fn pom_request_id(self, raw_request_id: u32) -> Option<PomRequestId> {
        self.0
            .flag(Self::HAS_POM_REQUEST_ID_BIT)
            .then(|| PomRequestId::new(raw_request_id))
    }

    pub(crate) fn runtime_event(
        self,
        packet_sequence: PacketSequence,
        raw_request_id: u32,
    ) -> CutoutRuntimeEvent {
        CutoutRuntimeEvent::Opened {
            packet_sequence,
            cutout: self.cutout(),
            target_address: self.target_address(),
            pom_request_id: self.pom_request_id(raw_request_id),
        }
    }

    pub(crate) fn public_metadata(self, raw_request_id: u32) -> RailcomPacketMetadata {
        RailcomPacketMetadata {
            target_address: self.target_address(),
            cutout: self.cutout(),
            pom_request_id: self.pom_request_id(raw_request_id),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_metadata_round_trips_every_field() {
        let addresses = [
            None,
            DccAddress::new_short(1),
            DccAddress::new_short(127),
            DccAddress::new_long(128),
            DccAddress::new_long(10_239),
        ];
        let cutouts = [
            CutoutMode::None,
            CutoutMode::Telemetry,
            CutoutMode::PomWrite,
            CutoutMode::PomRead,
        ];
        let request_ids = [
            None,
            Some(PomRequestId::new(0)),
            Some(PomRequestId::new(u32::MAX)),
        ];
        let packet_sequence = PacketSequence::new(u32::MAX);

        for target_address in addresses {
            for cutout in cutouts {
                for pom_request_id in request_ids {
                    let raw_request_id = pom_request_id.map_or(0, PomRequestId::value);
                    let packed = PackedRailcomPacketMetadata::from_raw(
                        PackedRailcomPacketMetadata::new(target_address, cutout, pom_request_id)
                            .raw(),
                    );

                    assert_eq!(
                        packed.public_metadata(raw_request_id),
                        RailcomPacketMetadata {
                            target_address,
                            cutout,
                            pom_request_id,
                        }
                    );
                    assert_eq!(
                        packed.runtime_event(packet_sequence, raw_request_id),
                        CutoutRuntimeEvent::Opened {
                            packet_sequence,
                            cutout,
                            target_address,
                            pom_request_id,
                        }
                    );
                }
            }
        }
    }
}
