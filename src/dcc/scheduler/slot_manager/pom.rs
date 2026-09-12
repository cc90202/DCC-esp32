//! Programming-on-main queue ownership and request correlation.

use super::{ActivePomRequest, PendingPomPacket, SlotManager};
use crate::authority::LeasePermit;
use crate::cutout::CutoutMode;
use crate::dcc::{DccPacket, PomRequestId};

impl SlotManager {
    /// Queue one explicit POM packet for single transmission.
    #[must_use]
    fn queue_program_on_main(
        &mut self,
        request_id: PomRequestId,
        permit: Option<LeasePermit>,
        packet: DccPacket,
    ) -> bool {
        let (target_address, cutout) = match packet {
            DccPacket::PomReadByte { address, .. } => (address, CutoutMode::PomRead),
            DccPacket::PomWriteByte { address, .. } => (address, CutoutMode::PomWrite),
            _ => return false,
        };
        if let Some(active) = self.active_pom {
            if active.request_id == request_id {
                if active.target_address != target_address || active.cutout != cutout {
                    return false;
                }
            } else {
                self.pending_pom.clear();
            }
        }
        self.active_pom = Some(ActivePomRequest {
            request_id,
            permit,
            target_address,
            cutout,
        });
        self.pending_pom
            .push(PendingPomPacket {
                request_id,
                permit,
                packet,
            })
            .is_ok()
    }

    #[cfg(test)]
    #[must_use]
    pub fn program_on_main(&mut self, request_id: PomRequestId, packet: DccPacket) -> bool {
        self.queue_program_on_main(request_id, None, packet)
    }

    pub(super) fn program_on_main_authorized(
        &mut self,
        request_id: PomRequestId,
        permit: LeasePermit,
        packet: DccPacket,
    ) -> bool {
        self.queue_program_on_main(request_id, Some(permit), packet)
    }

    pub fn close_program_on_main(&mut self, request_id: PomRequestId) -> bool {
        if !self
            .active_pom
            .is_some_and(|active| active.request_id == request_id)
        {
            return false;
        }
        self.active_pom = None;
        self.pending_pom.clear();
        true
    }

    #[cfg(any(test, target_arch = "riscv32"))]
    pub(in crate::dcc::scheduler) fn pom_context_for_packet(
        &self,
        packet: DccPacket,
    ) -> Option<(PomRequestId, CutoutMode)> {
        let target_address = packet.railcom_target_address()?;
        self.active_pom
            .filter(|active| active.target_address == target_address)
            .map(|active| (active.request_id, active.cutout))
    }

    pub(in crate::dcc::scheduler) fn discard_pom(
        &mut self,
        request_id: PomRequestId,
        permit: LeasePermit,
    ) {
        self.pending_pom.retain(|pending| {
            pending.request_id != request_id
                || pending.permit.map(LeasePermit::epoch) != Some(permit.epoch())
        });
        if self.active_pom.is_some_and(|active| {
            active.request_id == request_id
                && active.permit.map(LeasePermit::epoch) == Some(permit.epoch())
        }) {
            self.active_pom = None;
        }
    }
}
