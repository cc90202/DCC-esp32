//! The way back to safety: the current image of the journal, restarting
//! into it unless it already runs.

use core::time::Duration;

use super::{Device, Restart, Runtime};
use crate::{Error, journal::Record, notice::Notice, sequence};

/// Time for the display to draw a line before the chip resets.
const DRAW_DELAY: Duration = Duration::from_millis(200);

impl<D: Device> Runtime<'_, D> {
    /// Goes back to the current image of `record`, after checking it is
    /// intact.
    ///
    /// # Errors
    /// Returns [`Error::Corrupt`] if the current image is damaged, or the
    /// error of the write sequence.
    pub(super) async fn restore(self, record: Record) -> Result<Option<Restart>, Error> {
        if !self.image_intact(record.current()).await? {
            return Err(Error::Corrupt);
        }
        self.restore_verified(record).await
    }

    /// As [`Self::restore`], for a current image already known to be
    /// intact.
    pub(super) async fn restore_verified(self, record: Record) -> Result<Option<Restart>, Error> {
        let saved = self
            .apply(|flash, booted| sequence::restore(flash, record, booted))
            .await?;
        if saved.current().slot() == self.device.state(|s| s.slot) {
            // The restored image already runs: this boot goes on with the
            // track off, as the boot after a restart would.
            self.device.state(|s| s.track_held = true);
            return Ok(None);
        }
        self.device.show(Notice::RestartingBack);
        self.device.pause(DRAW_DELAY).await;
        Ok(Some(Restart))
    }
}
