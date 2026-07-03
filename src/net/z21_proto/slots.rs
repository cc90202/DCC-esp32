use crate::dcc::{DccAddress, Direction, SpeedFormat};
use crate::net::{LocoSlots, LocoState};

/// Find an existing slot for `addr` or insert a new one.
///
/// Eviction policy: if all slots are full, evict the first slot with speed=0.
/// If all slots have speed>0, returns None (command dropped).
pub fn find_or_insert(slots: &mut LocoSlots, addr: DccAddress) -> Option<&mut LocoState> {
    // Pass 1: find existing slot.
    if let Some(i) = slots
        .iter()
        .position(|s| s.as_ref().is_some_and(|s| s.address == addr))
    {
        return slots[i].as_mut();
    }

    // Pass 2: find empty slot.
    if let Some(i) = slots.iter().position(|s| s.is_none()) {
        slots[i] = Some(LocoState {
            address: addr,
            speed: 0,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: 0,
        });
        return slots[i].as_mut();
    }

    // Pass 3: evict first stopped slot (speed == 0).
    if let Some(i) = slots
        .iter()
        .position(|s| s.as_ref().is_some_and(|s| s.speed == 0))
    {
        slots[i] = Some(LocoState {
            address: addr,
            speed: 0,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: 0,
        });
        return slots[i].as_mut();
    }

    None
}

/// Find an existing slot for `addr` (read-only).
pub fn find_slot(slots: &LocoSlots, addr: DccAddress) -> Option<&LocoState> {
    slots.iter().flatten().find(|s| s.address == addr)
}
