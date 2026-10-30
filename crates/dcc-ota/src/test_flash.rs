//! Simulated NOR flash for host tests, able to cut power inside any call.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

/// Error of a simulated power cut.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PowerCut;

impl NorFlashError for PowerCut {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

/// A window of flash starting at `base`, erased to `0xFF`.
///
/// Writes may only clear bits, as on real NOR flash. `cut = Some((call, n))`
/// fails the `call`-th operation after it has changed its first `n` bytes.
/// `mutable[call]` records how many bytes each call could change: zero for
/// reads, the length for erases and writes.
#[derive(Clone)]
pub(crate) struct SimFlash {
    pub bytes: Vec<u8>,
    pub base: u32,
    pub calls: usize,
    pub cut: Option<(usize, usize)>,
    pub erased: Vec<u32>,
    pub mutable: Vec<usize>,
}

impl SimFlash {
    pub(crate) fn new(base: u32, len: usize) -> Self {
        Self {
            bytes: std::vec![0xFF; len],
            base,
            calls: 0,
            cut: None,
            erased: Vec::new(),
            mutable: Vec::new(),
        }
    }

    /// Forgets the calls made so far, so the next ones are counted from 0.
    pub(crate) fn start_log(&mut self) {
        self.calls = 0;
        self.mutable.clear();
        self.erased.clear();
    }

    /// Like [`Self::cuts`], but only before, halfway through and after each
    /// mutating call, for sequences too long to cut at every byte.
    pub(crate) fn coarse_cuts(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.mutable.iter().enumerate().flat_map(|(call, &len)| {
            let prefixes: &[usize] = if len == 0 { &[0] } else { &[0, 1, 2] };
            prefixes.iter().map(move |&step| (call, len * step / 2))
        })
    }

    fn index(&self, offset: u32) -> usize {
        (offset - self.base) as usize
    }

    /// Every cut a sequence of calls allows: each call, with every prefix of
    /// the bytes it changes.
    pub(crate) fn cuts(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.mutable
            .iter()
            .enumerate()
            .flat_map(|(call, &len)| (0..=len).map(move |prefix| (call, prefix)))
    }

    /// Returns how many bytes this call may change and whether it then fails.
    fn step(&mut self, len: usize) -> (usize, bool) {
        let call = self.calls;
        self.calls += 1;
        self.mutable.push(len);
        match self.cut {
            Some((at, prefix)) if at == call => (prefix.min(len), true),
            _ => (len, false),
        }
    }
}

impl ErrorType for SimFlash {
    type Error = PowerCut;
}

impl ReadNorFlash for SimFlash {
    const READ_SIZE: usize = 4;

    fn capacity(&self) -> usize {
        self.base as usize + self.bytes.len()
    }

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), PowerCut> {
        assert_eq!(offset % 4, 0);
        assert_eq!(bytes.len() % 4, 0);
        if self.step(0).1 {
            return Err(PowerCut);
        }
        let i = self.index(offset);
        bytes.copy_from_slice(&self.bytes[i..i + bytes.len()]);
        Ok(())
    }
}

impl NorFlash for SimFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), PowerCut> {
        assert_eq!(from % 4096, 0);
        assert_eq!(to - from, 4096);
        self.erased.push(from);
        let (n, cut) = self.step((to - from) as usize);
        let i = self.index(from);
        self.bytes[i..i + n].fill(0xFF);
        if cut { Err(PowerCut) } else { Ok(()) }
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), PowerCut> {
        assert_eq!(offset % 4, 0);
        assert_eq!(bytes.len() % 4, 0);
        let (n, cut) = self.step(bytes.len());
        let i = self.index(offset);
        for (cell, &bits) in self.bytes[i..i + n].iter_mut().zip(bytes) {
            assert_eq!(*cell & bits, bits, "write must only clear bits");
            *cell &= bits;
        }
        if cut { Err(PowerCut) } else { Ok(()) }
    }
}
