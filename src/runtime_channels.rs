//! Generic Embassy transport aliases.
//!
//! Only the parameterised forms live here. Every concrete alias lives beside the
//! message type it carries, so this module imports nothing from the crate and
//! cannot take part in a dependency cycle.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};

pub(crate) type RuntimeChannel<T, const N: usize> = Channel<CriticalSectionRawMutex, T, N>;
pub(crate) type RuntimeSender<T, const N: usize> = Sender<'static, CriticalSectionRawMutex, T, N>;
pub(crate) type RuntimeReceiver<T, const N: usize> =
    Receiver<'static, CriticalSectionRawMutex, T, N>;
