use heapless::Vec;

use crate::railcom::pipeline::RailcomChannel;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomCutoutEdges {
    pub started_count: u32,
    pub ended_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomWindowCommand {
    Open {
        packet_sequence: u32,
        channel: RailcomChannel,
    },
    Close,
}

/// Pure cutout-edge to UART-window bridge.
///
/// Translates observed physical cutout edge counters into the logical
/// `Open/Close` commands consumed by the UART-side reader. Contains no async
/// runtime code and no hardware access so the invariants can be exercised
/// end-to-end in host tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RailcomWindowBridge {
    last_started_count: u32,
    last_ended_count: u32,
    window_open: bool,
}

impl RailcomWindowBridge {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            last_started_count: 0,
            last_ended_count: 0,
            window_open: false,
        }
    }

    #[must_use]
    pub fn update(&mut self, edges: RailcomCutoutEdges) -> Vec<RailcomWindowCommand, 4> {
        let mut out = Vec::new();

        while self.last_started_count < edges.started_count {
            self.last_started_count = self.last_started_count.wrapping_add(1);

            if self.window_open {
                let _ = out.push(RailcomWindowCommand::Close);
            }

            let _ = out.push(RailcomWindowCommand::Open {
                packet_sequence: self.last_started_count,
                channel: RailcomChannel::Channel2,
            });
            self.window_open = true;
        }

        while self.last_ended_count < edges.ended_count {
            self.last_ended_count = self.last_ended_count.wrapping_add(1);

            if self.window_open {
                let _ = out.push(RailcomWindowCommand::Close);
                self.window_open = false;
            }
        }

        out
    }

    /// Realign the bridge state to externally observed counters without
    /// emitting commands. Useful after a dropped runtime event.
    pub fn resync(&mut self, edges: RailcomCutoutEdges) {
        self.last_started_count = edges.started_count;
        self.last_ended_count = edges.ended_count;
        self.window_open = edges.started_count > edges.ended_count;
    }

    #[must_use]
    pub const fn window_open(&self) -> bool {
        self.window_open
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edges(started: u32, ended: u32) -> RailcomCutoutEdges {
        RailcomCutoutEdges {
            started_count: started,
            ended_count: ended,
        }
    }

    #[test]
    fn test_bridge_emits_open_on_first_started_edge() {
        let mut bridge = RailcomWindowBridge::new();
        let out = bridge.update(edges(1, 0));

        assert_eq!(
            out.as_slice(),
            &[RailcomWindowCommand::Open {
                packet_sequence: 1,
                channel: RailcomChannel::Channel2,
            }]
        );
        assert!(bridge.window_open());
    }

    #[test]
    fn test_bridge_emits_close_when_end_edge_arrives() {
        let mut bridge = RailcomWindowBridge::new();
        let _ = bridge.update(edges(1, 0));
        let out = bridge.update(edges(1, 1));

        assert_eq!(out.as_slice(), &[RailcomWindowCommand::Close]);
        assert!(!bridge.window_open());
    }

    #[test]
    fn test_bridge_handles_multiple_completed_cutouts() {
        let mut bridge = RailcomWindowBridge::new();
        let out = bridge.update(edges(2, 2));

        assert_eq!(
            out.as_slice(),
            &[
                RailcomWindowCommand::Open {
                    packet_sequence: 1,
                    channel: RailcomChannel::Channel2,
                },
                RailcomWindowCommand::Close,
                RailcomWindowCommand::Open {
                    packet_sequence: 2,
                    channel: RailcomChannel::Channel2,
                },
                RailcomWindowCommand::Close,
            ]
        );
        assert!(!bridge.window_open());
    }

    #[test]
    fn test_bridge_closes_previous_window_if_started_counter_jumps_while_open() {
        let mut bridge = RailcomWindowBridge::new();
        let _ = bridge.update(edges(1, 0));
        let out = bridge.update(edges(2, 0));

        assert_eq!(
            out.as_slice(),
            &[
                RailcomWindowCommand::Close,
                RailcomWindowCommand::Open {
                    packet_sequence: 2,
                    channel: RailcomChannel::Channel2,
                },
            ]
        );
        assert!(bridge.window_open());
    }

    #[test]
    fn test_bridge_resync_tracks_observed_edge_state() {
        let mut bridge = RailcomWindowBridge::new();
        bridge.resync(edges(4, 3));
        assert!(bridge.window_open());

        let out = bridge.update(edges(4, 4));
        assert_eq!(out.as_slice(), &[RailcomWindowCommand::Close]);
        assert!(!bridge.window_open());
    }
}
