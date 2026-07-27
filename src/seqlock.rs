//! Fixed-word seqlock slots for single-writer ISR publication.

use core::sync::atomic::{AtomicU32, Ordering};

/// Publishes a fixed-size atomic snapshot without duplicating memory-ordering
/// decisions at each ISR/task boundary.
///
/// Generation zero means unpublished. Odd generations are writes in progress;
/// readers accept only the same non-zero even generation before and after all
/// payload words are loaded.
pub(crate) struct SeqSlot<const N: usize> {
    generation: AtomicU32,
    words: [AtomicU32; N],
}

impl<const N: usize> SeqSlot<N> {
    pub(crate) const fn new() -> Self {
        Self {
            generation: AtomicU32::new(0),
            words: [const { AtomicU32::new(0) }; N],
        }
    }

    #[inline(always)]
    pub(crate) fn publish(&self, words: [u32; N]) {
        let updating_generation = self.generation.load(Ordering::Relaxed).wrapping_add(1) | 1;
        self.generation.store(updating_generation, Ordering::SeqCst);
        for (destination, word) in self.words.iter().zip(words) {
            destination.store(word, Ordering::Relaxed);
        }
        self.generation
            .store(updating_generation.wrapping_add(1), Ordering::SeqCst);
    }

    #[inline(always)]
    pub(crate) fn snapshot(&self) -> Option<[u32; N]> {
        let generation_before = self.generation.load(Ordering::SeqCst);
        if generation_before == 0 || generation_before & 1 != 0 {
            return None;
        }

        let words = core::array::from_fn(|index| self.words[index].load(Ordering::Acquire));
        let generation_after = self.generation.load(Ordering::SeqCst);
        (generation_before == generation_after && generation_after & 1 == 0).then_some(words)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unpublished_slot_has_no_snapshot() {
        assert_eq!(SeqSlot::<3>::new().snapshot(), None);
    }

    #[test]
    fn published_words_round_trip_including_zero() {
        let slot = SeqSlot::<3>::new();
        slot.publish([0, 7, u32::MAX]);

        assert_eq!(slot.snapshot(), Some([0, 7, u32::MAX]));
    }

    #[test]
    fn later_publication_replaces_the_whole_snapshot() {
        let slot = SeqSlot::<3>::new();
        slot.publish([1, 2, 3]);
        slot.publish([4, 5, 6]);

        assert_eq!(slot.snapshot(), Some([4, 5, 6]));
    }
}
