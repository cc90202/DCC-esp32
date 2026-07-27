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
        let current_generation = self.generation.load(Ordering::Relaxed);
        let mut published_generation = current_generation.wrapping_add(2) & !1;
        if published_generation == 0 {
            // Generation zero is reserved for "never published". Skip it when
            // the counter wraps so a valid snapshot is never misclassified.
            published_generation = 2;
        }
        let updating_generation = published_generation - 1;
        self.generation.store(updating_generation, Ordering::SeqCst);
        for (destination, word) in self.words.iter().zip(words) {
            destination.store(word, Ordering::Relaxed);
        }
        self.generation
            .store(published_generation, Ordering::SeqCst);
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
    use core::sync::atomic::{AtomicBool, AtomicUsize};
    use std::sync::Barrier;
    use std::thread;

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

    #[test]
    fn generation_wrap_never_reuses_the_unpublished_sentinel() {
        let slot = SeqSlot::<1>::new();
        slot.generation.store(u32::MAX - 1, Ordering::Relaxed);

        slot.publish([9]);

        assert_eq!(slot.snapshot(), Some([9]));
        assert_eq!(slot.generation.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn concurrent_snapshots_never_mix_two_publications() {
        const ITERATIONS: usize = 100_000;

        let slot = SeqSlot::<3>::new();
        let finished = AtomicBool::new(false);
        let observed = AtomicUsize::new(0);
        let start = Barrier::new(3);

        thread::scope(|scope| {
            let writer = scope.spawn(|| {
                start.wait();
                for iteration in 0..ITERATIONS {
                    let marker = (iteration & 1) as u32 + 1;
                    slot.publish([marker; 3]);
                    if iteration.is_multiple_of(256) {
                        thread::yield_now();
                    }
                }
                finished.store(true, Ordering::Release);
            });
            let reader = scope.spawn(|| {
                start.wait();
                while !finished.load(Ordering::Acquire) {
                    if let Some(words) = slot.snapshot() {
                        assert!(words == [1; 3] || words == [2; 3]);
                        observed.fetch_add(1, Ordering::Relaxed);
                    }
                }
            });

            start.wait();
            writer.join().unwrap();
            reader.join().unwrap();
        });

        assert!(observed.load(Ordering::Relaxed) > 0);
        assert!(matches!(slot.snapshot(), Some([1, 1, 1] | [2, 2, 2])));
    }
}
