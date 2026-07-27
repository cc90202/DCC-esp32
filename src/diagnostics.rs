//! Declaration of the firmware's diagnostic counter groups.
//!
//! Counters are telemetry: they record how often something happened so a fault
//! can be explained after the fact, without logging every event from an ISR.

/// Declares one group of diagnostic counters.
///
/// A counter group is the single source of truth for the atomic storage, the
/// snapshot type, and the snapshot operation. Groups exercised by stateful
/// host tests can additionally request a test-only reset. Adding a counter to
/// the declaration then keeps storage, snapshots, and resets in lockstep.
///
/// Increments stay at the call site and remain ordinary Rust:
///
/// ```ignore
/// RAILCOM_RX.parse_ok_count.fetch_add(1, Ordering::Relaxed);
/// ```
///
/// That is deliberate. Counters incremented from an interrupt use `Relaxed`
/// because they are advisory telemetry rather than a synchronisation point,
/// and that reasoning belongs at the call site. Snapshot loads use `Acquire`;
/// requested test resets use `Release`.
///
/// A group is one `static` holding a struct of atomics, rather than one
/// `static` per counter. This keeps every increment greppable without requiring
/// identifier construction through a dependency such as `paste`.
///
/// Attributes above the `static` line are applied to every generated item, so
/// a `#[cfg(...)]` there gates the entire group.
///
/// # Example
///
/// ```ignore
/// diagnostic_counters! {
///     #[cfg(any(test, target_arch = "riscv32"))]
///     static RAILCOM_SCHEDULER: RailcomSchedulerCounters;
///
///     /// Snapshot of scheduler-side RailCom cutout decisions.
///     pub snapshot RailcomSchedulerStats;
///
///     @test reset;
///
///     cutout_granted_count,
///     cutout_skipped_budget_count,
/// }
/// ```
macro_rules! diagnostic_counters {
    (@types
        $(#[$group_meta:meta])*
        $svis:vis $counters:ident,
        $(#[$snap_meta:meta])*
        $pvis:vis $stats:ident,
        $( $(#[$field_meta:meta])* $field:ident )+
    ) => {
        $(#[$group_meta])*
        $(#[$snap_meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
        #[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
        $pvis struct $stats {
            $( $(#[$field_meta])* pub $field: u32, )+
        }

        $(#[$group_meta])*
        #[derive(Debug)]
        $svis struct $counters {
            $( $(#[$field_meta])* pub $field: ::core::sync::atomic::AtomicU32, )+
        }

        $(#[$group_meta])*
        impl $counters {
            const fn new() -> Self {
                Self {
                    $( $field: ::core::sync::atomic::AtomicU32::new(0), )+
                }
            }

            /// Loads every counter in the group into a plain snapshot.
            #[must_use]
            $pvis fn snapshot(&self) -> $stats {
                $stats {
                    $(
                        $field: self
                            .$field
                            .load(::core::sync::atomic::Ordering::Acquire),
                    )+
                }
            }

        }
    };

    (@maybe_reset
        []
        [$(#[$group_meta:meta])*]
        $counters:ident,
        $( $(#[$field_meta:meta])* $field:ident )+
    ) => {};

    (@maybe_reset
        [$reset:ident]
        [$(#[$group_meta:meta])*]
        $counters:ident,
        $( $(#[$field_meta:meta])* $field:ident )+
    ) => {
        $(#[$group_meta])*
        #[cfg(test)]
        impl $counters {
            /// Clears every counter in the group. Host tests only.
            fn $reset(&self) {
                $(
                    self.$field
                        .store(0, ::core::sync::atomic::Ordering::Release);
                )+
            }
        }
    };

    (
        $(#[$group_meta:meta])*
        $svis:vis static $storage:ident: [$counters:ident; $len:expr];
        $(#[$snap_meta:meta])*
        $pvis:vis snapshot $stats:ident;
        $(@test $reset:ident;)?
        $( $(#[$field_meta:meta])* $field:ident ),+ $(,)?
    ) => {
        diagnostic_counters!(@types
            $(#[$group_meta])* $svis $counters,
            $(#[$snap_meta])* $pvis $stats,
            $( $(#[$field_meta])* $field )+
        );

        diagnostic_counters!(@maybe_reset
            [$($reset)?]
            [$(#[$group_meta])*]
            $counters,
            $( $(#[$field_meta])* $field )+
        );

        $(#[$group_meta])*
        $svis static $storage: [$counters; $len] = [const { $counters::new() }; $len];
    };

    (
        $(#[$group_meta:meta])*
        $svis:vis static $storage:ident: $counters:ident;
        $(#[$snap_meta:meta])*
        $pvis:vis snapshot $stats:ident;
        $(@test $reset:ident;)?
        $( $(#[$field_meta:meta])* $field:ident ),+ $(,)?
    ) => {
        diagnostic_counters!(@types
            $(#[$group_meta])* $svis $counters,
            $(#[$snap_meta])* $pvis $stats,
            $( $(#[$field_meta])* $field )+
        );

        diagnostic_counters!(@maybe_reset
            [$($reset)?]
            [$(#[$group_meta])*]
            $counters,
            $( $(#[$field_meta])* $field )+
        );

        $(#[$group_meta])*
        $svis static $storage: $counters = $counters::new();
    };
}

pub(crate) use diagnostic_counters;
