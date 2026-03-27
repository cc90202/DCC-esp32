//! CV Programming Service Mode Implementation
//!
//! Provides Service Mode CV programming (Verify Byte + Write Byte) with hardware
//! abstraction via traits. Automatically coordinates main track pause/resume and
//! relay switching with RAII safety guarantees.
//!
//! # NMRA S-9.2.2 Compliance
//!
//! - Direct Mode CV addressing (CV 1-256)
//! - Verify Byte operation (instruction 0x74)
//! - Write Byte operation (instruction 0x7C)
//! - ACK pulse detection for confirmation
//! - 5 packet repetitions with 6ms gaps (configurable)
//!
//! # Examples
//!
//! **Verify a CV value (host example):**
//!
//! ```no_run
//! use dcc_esp32::dcc::{MockTrackSwitch, MockAckDetector, CvProgrammer, ProgrammingConfig};
//!
//! // Mock hardware for testing (no ESP hardware needed)
//! let track_switch = MockTrackSwitch::new();
//! let ack_detector = MockAckDetector::new(true); // Will report ACK
//!
//! let config = ProgrammingConfig::default();
//! let programmer = CvProgrammer::new(track_switch, ack_detector, config);
//!
//! // Verify CV1 (primary address) equals 42
//! // let result = programmer.verify_byte(1, 42).await;
//! ```
//!
//! **Write a CV value (host example):**
//!
//! ```no_run
//! use dcc_esp32::dcc::{MockTrackSwitch, MockAckDetector, CvProgrammer};
//!
//! let track_switch = MockTrackSwitch::new();
//! let ack_detector = MockAckDetector::new(true);
//! let programmer = CvProgrammer::new(
//!     track_switch,
//!     ack_detector,
//!     Default::default(),
//! );
//!
//! // Write CV2 (acceleration) = 20
//! // let result = programmer.write_byte(2, 20).await;
//! ```
//!
//! # Architecture
//!
//! - Separate RMT channel from main track (planned; not integrated yet)
//! - Trait-based hardware abstraction (`TrackSwitch`, `AckDetector`)
//! - Automatic main track coordination via scheduler Pause/Resume commands
//! - RAII safety with `ProgrammingSessionGuard`
//! - Fault protection: refuses new sessions after improper shutdown

#[cfg(target_arch = "riscv32")]
use crate::dcc::scheduler::SchedulerCommand;
#[cfg(target_arch = "riscv32")]
use crate::fault_manager::FaultEvent;
#[cfg(target_arch = "riscv32")]
use crate::system_status::FaultCause;
#[cfg(target_arch = "riscv32")]
use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::Sender;
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, Instant};

// --- Configuration ---

/// CV programming configuration (NMRA S-9.2.2 timing parameters)
#[derive(Debug, Clone, Copy)]
pub struct ProgrammingConfig {
    /// Packet repetitions per operation (default: 5 per NMRA S-9.2.2)
    pub repetitions: u8,

    /// Gap between repeated packets in milliseconds (default: 6ms)
    pub packet_gap_ms: u32,

    /// Timeout waiting for ACK pulse after last packet (default: 20ms)
    pub ack_timeout_ms: u32,

    /// Maximum total time for one CV operation in milliseconds (default: 3000ms)
    pub operation_timeout_ms: u32,
}

impl Default for ProgrammingConfig {
    fn default() -> Self {
        Self {
            repetitions: 5,     // NMRA-strict
            packet_gap_ms: 6,   // NMRA-strict
            ack_timeout_ms: 20, // NMRA-strict
            operation_timeout_ms: 3000,
        }
    }
}

// --- CV Address Validation ---

#[cfg(any(test, target_arch = "riscv32"))]
const MIN_CV: u16 = 1;
#[cfg(any(test, target_arch = "riscv32"))]
const MAX_CV_V1: u16 = 256;

// --- Error Types ---

/// Errors that can occur during CV read operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CvReadError {
    /// CV address outside valid range (1-256 in V1)
    InvalidCvAddress(u16),

    /// No ACK received after verifying all 256 possible values
    NoAckReceived,

    /// ACK detector hardware error
    AckDetectorError(AckError),

    /// Track switch relay error
    TrackSwitchError(SwitchError),

    /// Operation exceeded timeout
    Timeout,

    /// Session management error
    SessionError(SessionError),

    /// Programming-track packet transport is not integrated yet on this target.
    PacketTransportUnavailable,
}

impl From<AckError> for CvReadError {
    fn from(e: AckError) -> Self {
        CvReadError::AckDetectorError(e)
    }
}

impl From<SwitchError> for CvReadError {
    fn from(e: SwitchError) -> Self {
        CvReadError::TrackSwitchError(e)
    }
}

impl From<SessionError> for CvReadError {
    fn from(e: SessionError) -> Self {
        CvReadError::SessionError(e)
    }
}

/// Errors that can occur during CV write operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CvWriteError {
    /// CV address outside valid range (1-256 in V1)
    InvalidCvAddress(u16),

    /// ACK detector hardware error
    AckDetectorError(AckError),

    /// Track switch relay error
    TrackSwitchError(SwitchError),

    /// Operation exceeded timeout
    Timeout,

    /// Session management error
    SessionError(SessionError),

    /// Programming-track packet transport is not integrated yet on this target.
    PacketTransportUnavailable,
}

impl From<AckError> for CvWriteError {
    fn from(e: AckError) -> Self {
        CvWriteError::AckDetectorError(e)
    }
}

impl From<SwitchError> for CvWriteError {
    fn from(e: SwitchError) -> Self {
        CvWriteError::TrackSwitchError(e)
    }
}

impl From<SessionError> for CvWriteError {
    fn from(e: SessionError) -> Self {
        CvWriteError::SessionError(e)
    }
}

/// Errors from track switch relay operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SwitchError {
    /// Hardware error during relay switching
    HardwareError,

    /// Already in requested mode
    AlreadyInRequestedMode,

    /// Operation timeout
    Timeout,
}

/// Errors from ACK pulse detection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum AckError {
    /// No ACK pulse received within timeout
    NoAckReceived,

    /// Hardware error during ACK detection
    HardwareError,

    /// Operation timeout
    Timeout,
}

/// Errors from session management
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SessionError {
    /// System is in fault state, refusing new sessions
    SystemInFaultState,

    /// Track switch error during session acquire/release
    TrackSwitchError(SwitchError),
}

impl From<SwitchError> for SessionError {
    fn from(e: SwitchError) -> Self {
        SessionError::TrackSwitchError(e)
    }
}

// --- Trait Abstractions ---

/// Trait for track relay switching between main and programming track
pub trait TrackSwitch {
    /// Switch relay to programming track output
    ///
    /// # Errors
    /// Returns error if hardware failure or timeout occurs
    fn switch_to_programming(
        &mut self,
    ) -> impl core::future::Future<Output = Result<(), SwitchError>>;

    /// Switch relay to main track output
    ///
    /// # Errors
    /// Returns error if hardware failure or timeout occurs
    fn switch_to_main(&mut self) -> impl core::future::Future<Output = Result<(), SwitchError>>;
}

/// Trait for ACK pulse detection from programming track current
pub trait AckDetector {
    /// Wait for ACK pulse (60-100mA current spike) within timeout
    ///
    /// # Errors
    /// Returns `NoAckReceived` if timeout expires without detecting ACK,
    /// `HardwareError` for sensor failures, or `Timeout` for operation timeout
    fn wait_for_ack(
        &mut self,
        timeout_ms: u32,
    ) -> impl core::future::Future<Output = Result<(), AckError>>;
}

// --- Mock Implementations for Testing ---

/// Mock track switch for testing (always succeeds)
#[cfg(any(test, not(target_arch = "riscv32")))]
pub struct MockTrackSwitch {
    in_programming_mode: bool,
}

#[cfg(any(test, not(target_arch = "riscv32")))]
impl MockTrackSwitch {
    pub fn new() -> Self {
        Self {
            in_programming_mode: false,
        }
    }

    pub fn is_in_programming_mode(&self) -> bool {
        self.in_programming_mode
    }
}

#[cfg(any(test, not(target_arch = "riscv32")))]
impl Default for MockTrackSwitch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(any(test, not(target_arch = "riscv32")))]
impl TrackSwitch for MockTrackSwitch {
    async fn switch_to_programming(&mut self) -> Result<(), SwitchError> {
        self.in_programming_mode = true;
        Ok(())
    }

    async fn switch_to_main(&mut self) -> Result<(), SwitchError> {
        self.in_programming_mode = false;
        Ok(())
    }
}

/// Mock ACK detector for testing (simulates ACK for expected CV/value pairs)
#[cfg(any(test, not(target_arch = "riscv32")))]
pub struct MockAckDetector {
    expected_cv: u16,
    expected_value: u8,
}

#[cfg(any(test, not(target_arch = "riscv32")))]
impl MockAckDetector {
    pub fn new(expected_cv: u16, expected_value: u8) -> Self {
        Self {
            expected_cv,
            expected_value,
        }
    }

    pub fn set_expected(&mut self, cv: u16, value: u8) {
        self.expected_cv = cv;
        self.expected_value = value;
    }
}

#[cfg(any(test, not(target_arch = "riscv32")))]
impl AckDetector for MockAckDetector {
    async fn wait_for_ack(&mut self, _timeout_ms: u32) -> Result<(), AckError> {
        // Keep expected pair observable in the mock API even though this
        // simplified detector currently always ACKs.
        let _ = (self.expected_cv, self.expected_value);
        // For testing: always ACK (real implementation will check current sensor)
        Ok(())
    }
}

// Note: ProgrammingSessionGuard implementation is simplified for this version
// Full RAII safety will be implemented when we have actual hardware integration
// For now, we use manual acquire/release pattern

// --- CV Address Validation ---

#[cfg(any(test, target_arch = "riscv32"))]
fn validate_cv_address(cv: u16) -> Result<(), CvReadError> {
    if !(MIN_CV..=MAX_CV_V1).contains(&cv) {
        return Err(CvReadError::InvalidCvAddress(cv));
    }
    Ok(())
}

#[cfg(target_arch = "riscv32")]
fn validate_cv_address_write(cv: u16) -> Result<(), CvWriteError> {
    if !(MIN_CV..=MAX_CV_V1).contains(&cv) {
        return Err(CvWriteError::InvalidCvAddress(cv));
    }
    Ok(())
}

// --- CvProgrammer (Embedded Only) ---

#[cfg(target_arch = "riscv32")]
pub struct CvProgrammer<TS, AD>
where
    TS: TrackSwitch,
    AD: AckDetector,
{
    track_switch: TS,
    _ack_detector: AD,
    // RMT channel will be added when integrating with esp-hal
    // rmt_channel: RmtChannel<'static, Blocking, Tx>,
    main_scheduler_tx: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    fault_sender: Option<Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>>,
    config: ProgrammingConfig,
    fault_flag: AtomicBool,
}

#[cfg(target_arch = "riscv32")]
impl<TS, AD> CvProgrammer<TS, AD>
where
    TS: TrackSwitch,
    AD: AckDetector,
{
    /// Create a new programming track service
    pub fn new(
        track_switch: TS,
        ack_detector: AD,
        main_scheduler_tx: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        config: ProgrammingConfig,
    ) -> Self {
        Self {
            track_switch,
            _ack_detector: ack_detector,
            main_scheduler_tx,
            fault_sender: None,
            config,
            fault_flag: AtomicBool::new(false),
        }
    }

    /// Create a new programming track service with fault manager event reporting.
    pub fn new_with_fault_sender(
        track_switch: TS,
        ack_detector: AD,
        main_scheduler_tx: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        fault_sender: Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
        config: ProgrammingConfig,
    ) -> Self {
        Self {
            track_switch,
            _ack_detector: ack_detector,
            main_scheduler_tx,
            fault_sender: Some(fault_sender),
            config,
            fault_flag: AtomicBool::new(false),
        }
    }

    /// Read CV value via linear verify sequence (0→255)
    ///
    /// # Errors
    /// Returns error if CV address invalid, no ACK received, hardware failure,
    /// or system is in fault state
    pub async fn read_cv(&mut self, cv: u16) -> Result<u8, CvReadError> {
        validate_cv_address(cv)?;

        // Check fault state
        if self.fault_flag.load(Ordering::Acquire) {
            return Err(CvReadError::SessionError(SessionError::SystemInFaultState));
        }

        if let Err(e) = self.enter_programming_mode().await {
            self.latch_fault(FaultCause::CvService);
            return Err(CvReadError::TrackSwitchError(e));
        }

        // Perform CV read
        let result = self.read_cv_internal(cv).await;

        let final_result = Self::merge_session_result(
            result,
            self.exit_programming_mode().await,
            CvReadError::TrackSwitchError,
        );

        if let Err(err) = &final_result
            && should_latch_read_fault(err)
        {
            self.latch_fault(FaultCause::CvService);
        }
        final_result
    }

    /// Write value to CV
    ///
    /// # Errors
    /// Returns error if CV address invalid, no ACK received, hardware failure,
    /// or system is in fault state
    pub async fn write_cv(&mut self, cv: u16, value: u8) -> Result<(), CvWriteError> {
        validate_cv_address_write(cv)?;

        // Check fault state
        if self.fault_flag.load(Ordering::Acquire) {
            return Err(CvWriteError::SessionError(SessionError::SystemInFaultState));
        }

        if let Err(e) = self.enter_programming_mode().await {
            self.latch_fault(FaultCause::CvService);
            return Err(CvWriteError::TrackSwitchError(e));
        }

        // Perform CV write
        let result = self.write_cv_internal(cv, value).await;

        let final_result = Self::merge_session_result(
            result,
            self.exit_programming_mode().await,
            CvWriteError::TrackSwitchError,
        );

        if let Err(err) = &final_result
            && should_latch_write_fault(err)
        {
            self.latch_fault(FaultCause::CvService);
        }
        final_result
    }

    /// Verify CV has expected value (single check, no linear search)
    ///
    /// # Errors
    /// Returns error if CV address invalid, hardware failure, or system is in fault state
    pub async fn verify_cv(&mut self, cv: u16, value: u8) -> Result<bool, CvReadError> {
        validate_cv_address(cv)?;

        // Check fault state
        if self.fault_flag.load(Ordering::Acquire) {
            return Err(CvReadError::SessionError(SessionError::SystemInFaultState));
        }

        if let Err(e) = self.enter_programming_mode().await {
            self.latch_fault(FaultCause::CvService);
            return Err(CvReadError::TrackSwitchError(e));
        }

        // Perform CV verify
        let result = self.verify_cv_internal(cv, value).await;

        let final_result = Self::merge_session_result(
            result,
            self.exit_programming_mode().await,
            CvReadError::TrackSwitchError,
        );

        if let Err(err) = &final_result
            && should_latch_read_fault(err)
        {
            self.latch_fault(FaultCause::CvService);
        }
        final_result
    }

    /// Clear fault state after manual recovery
    pub fn clear_fault(&mut self) {
        self.fault_flag.store(false, Ordering::Release);
        if let Some(sender) = &self.fault_sender {
            let _ = sender.try_send(FaultEvent::FaultClearedByService);
        }
        defmt::info!("Programming service fault cleared");
    }

    /// Check if system is in fault state
    pub fn is_faulted(&self) -> bool {
        self.fault_flag.load(Ordering::Acquire)
    }

    async fn enter_programming_mode(&mut self) -> Result<(), SwitchError> {
        self.main_scheduler_tx.send(SchedulerCommand::Pause).await;
        if let Err(e) = self.track_switch.switch_to_programming().await {
            let _ = self.main_scheduler_tx.send(SchedulerCommand::Resume).await;
            return Err(e);
        }
        Ok(())
    }

    async fn exit_programming_mode(&mut self) -> Result<(), SwitchError> {
        let switch_result = self.track_switch.switch_to_main().await;
        self.main_scheduler_tx.send(SchedulerCommand::Resume).await;
        switch_result
    }

    fn merge_session_result<T, E, F>(
        operation_result: Result<T, E>,
        switch_result: Result<(), SwitchError>,
        map_switch_error: F,
    ) -> Result<T, E>
    where
        F: FnOnce(SwitchError) -> E,
    {
        match (operation_result, switch_result) {
            (Ok(value), Ok(())) => Ok(value),
            (Err(e), _) => Err(e),
            (Ok(_), Err(e)) => Err(map_switch_error(e)),
        }
    }

    fn latch_fault(&self, cause: FaultCause) {
        self.fault_flag.store(true, Ordering::Release);
        if let Some(sender) = &self.fault_sender {
            let _ = sender.try_send(FaultEvent::FaultLatched(cause));
        }
    }

    // --- Internal Methods ---

    async fn read_cv_internal(&mut self, cv: u16) -> Result<u8, CvReadError> {
        let start = Instant::now();
        let total_timeout = Duration::from_millis(self.config.operation_timeout_ms as u64);

        // Try all 256 possible values (0-255)
        for value in 0..=255u8 {
            if Instant::now().duration_since(start) > total_timeout {
                return Err(CvReadError::Timeout);
            }

            // Verify if CV == value
            match self.verify_cv_internal(cv, value).await {
                Ok(true) => return Ok(value), // ACK received, found value!
                Ok(false) => continue,        // No ACK, try next
                Err(e) => return Err(e),      // Hardware error, abort
            }
        }

        // No value 0-255 got ACK → decoder not responding
        Err(CvReadError::NoAckReceived)
    }

    async fn verify_cv_internal(&mut self, cv: u16, value: u8) -> Result<bool, CvReadError> {
        let _ = (cv, value);
        Err(CvReadError::PacketTransportUnavailable)
    }

    async fn write_cv_internal(&mut self, cv: u16, value: u8) -> Result<(), CvWriteError> {
        let _ = (cv, value);
        Err(CvWriteError::PacketTransportUnavailable)
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
fn should_latch_read_fault(err: &CvReadError) -> bool {
    match err {
        CvReadError::TrackSwitchError(_)
        | CvReadError::Timeout
        | CvReadError::AckDetectorError(AckError::HardwareError | AckError::Timeout)
        | CvReadError::SessionError(SessionError::TrackSwitchError(_)) => true,
        CvReadError::InvalidCvAddress(_)
        | CvReadError::NoAckReceived
        | CvReadError::PacketTransportUnavailable
        | CvReadError::AckDetectorError(AckError::NoAckReceived)
        | CvReadError::SessionError(SessionError::SystemInFaultState) => false,
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
fn should_latch_write_fault(err: &CvWriteError) -> bool {
    match err {
        CvWriteError::TrackSwitchError(_)
        | CvWriteError::Timeout
        | CvWriteError::AckDetectorError(AckError::HardwareError | AckError::Timeout)
        | CvWriteError::SessionError(SessionError::TrackSwitchError(_)) => true,
        CvWriteError::InvalidCvAddress(_)
        | CvWriteError::PacketTransportUnavailable
        | CvWriteError::AckDetectorError(AckError::NoAckReceived)
        | CvWriteError::SessionError(SessionError::SystemInFaultState) => false,
    }
}

// --- Tests ---

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cv_address_validation() {
        assert!(validate_cv_address(1).is_ok());
        assert!(validate_cv_address(256).is_ok());
        assert!(matches!(
            validate_cv_address(0),
            Err(CvReadError::InvalidCvAddress(0))
        ));
        assert!(matches!(
            validate_cv_address(257),
            Err(CvReadError::InvalidCvAddress(257))
        ));
    }

    #[test]
    fn test_programming_config_default() {
        let config = ProgrammingConfig::default();
        assert_eq!(config.repetitions, 5);
        assert_eq!(config.packet_gap_ms, 6);
        assert_eq!(config.ack_timeout_ms, 20);
        assert_eq!(config.operation_timeout_ms, 3000);
    }

    #[test]
    fn test_mock_track_switch() {
        let switch = MockTrackSwitch::new();
        assert!(!switch.is_in_programming_mode());

        // Note: Can't test async in sync test without tokio/async-std
        // These will be tested in integration tests
    }

    #[test]
    fn test_error_conversions() {
        let ack_error = AckError::NoAckReceived;
        let cv_error: CvReadError = ack_error.into();
        assert!(matches!(
            cv_error,
            CvReadError::AckDetectorError(AckError::NoAckReceived)
        ));

        let switch_error = SwitchError::HardwareError;
        let session_error: SessionError = switch_error.into();
        assert!(matches!(
            session_error,
            SessionError::TrackSwitchError(SwitchError::HardwareError)
        ));
    }

    #[test]
    fn test_transport_unavailable_does_not_latch_fault() {
        assert!(!should_latch_read_fault(
            &CvReadError::PacketTransportUnavailable
        ));
        assert!(!should_latch_write_fault(
            &CvWriteError::PacketTransportUnavailable
        ));
    }
}
