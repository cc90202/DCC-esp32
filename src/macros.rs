//! Crate-wide macros.

/// Runs the enclosed statements only in bench-diagnostics builds.
///
/// Compiles to nothing unless the `bench-diag` feature is enabled, arguments
/// included. Every `defmt` line is emitted inside a critical section, so a log
/// statement on a per-packet path adds interrupt latency to the DCC waveform
/// ISR and the RailCom cutout timer, both of which run at `Priority3`. Wrap
/// diagnostics that fire per packet, per cutout window or per network command;
/// warnings and errors stay unconditional.
///
/// Two rules for the block body:
///
/// - `return`, `?`, `break` and `continue` act on the function that contains
///   the block, so they would take effect only in bench builds. Either the
///   block is the whole function body, as in `log_pom_window` in
///   `railcom::runtime_dispatch`, or it must not contain control flow that
///   leaves it.
/// - `rustfmt` does not descend into brace-delimited macro bodies, so line
///   width and layout inside the block are the author's responsibility.
///   `clippy` does descend and still applies.
///
/// A function whose whole body is one of these blocks needs
/// `#[cfg_attr(not(feature = "bench-diag"), allow(unused_variables))]`, since
/// its parameters go unread once the block is compiled out.
#[cfg_attr(not(target_arch = "riscv32"), allow(unused_macros))]
macro_rules! bench_diag {
    ($($body:tt)*) => {
        #[cfg(feature = "bench-diag")]
        {
            $($body)*
        }
    };
}

// Every call site is in a firmware-only module, so on the host target both the
// macro and this re-export are genuinely unused.
#[cfg(target_arch = "riscv32")]
pub(crate) use bench_diag;
