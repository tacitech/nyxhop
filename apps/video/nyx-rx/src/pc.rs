//! PC-side demodulator hooks. With the `pcphy` feature (development tree) the
//! receiver can also demodulate raw IQ and spectral-grid captures on the PC.
//! The public build consumes only frames the board has already decoded
//! (`DecFrame`), and every hook here degrades to "nothing decoded".

use num_complex::Complex32;
use nyx_proto::Mcs;

/// Control word of a frame as read from its SIG symbol.
#[derive(Clone, Copy, Debug, Default)]
pub struct SigPayload {
    pub mcs_index: u8,
    pub rv: u8,
    pub seq_lsb: u8,
    pub pair: bool,
}

/// Demodulation result of one frame.
pub struct DemodResult {
    pub synced: bool,
    pub sync_metric: f32,
    pub cfo_hz: f32,
    /// Post-equalizer SNR estimate in dB.
    pub snr_db: f32,
    /// Coded-domain LLRs, empty when the frame was decoded on the board.
    pub llrs: Vec<f32>,
    /// Equalized data symbols with their noise variance (PC path only).
    pub eq_syms: Vec<(Complex32, f32)>,
    pub constellation: Vec<(f32, f32)>,
    pub chan_mag: Vec<f32>,
}

impl DemodResult {
    /// An empty result for the no-sync / no-control-info paths.
    pub fn failed() -> Self {
        DemodResult {
            synced: false,
            sync_metric: 0.0,
            cfo_hz: 0.0,
            snr_db: 0.0,
            llrs: Vec::new(),
            eq_syms: Vec::new(),
            constellation: Vec::new(),
            chan_mag: Vec::new(),
        }
    }
}

/// FEC outcome of one frame.
pub struct DecodeOutcome {
    pub payload: Vec<u8>,
    pub cw_ok: Vec<bool>,
    /// Pre-FEC BER estimate (over successfully decoded codewords).
    pub pre_ber: f32,
}

/// Preamble search result.
pub struct SyncResult {
    /// Index of the first sample of the preamble's useful (non-CP) part.
    pub useful_start: usize,
    /// Fractional CFO estimate in units of subcarrier spacing.
    pub cfo_scs: f32,
    /// Peak timing metric (0..1), for diagnostics.
    pub metric: f32,
}

/// Codewords in one frame.
pub const CODEWORDS_PER_FRAME: usize = 2;

#[cfg(feature = "pcphy")]
#[path = "pc_phy.rs"]
mod imp;

#[cfg(not(feature = "pcphy"))]
mod imp {
    use super::*;

    pub const AVAILABLE: bool = false;

    pub struct PcRx;

    impl PcRx {
        pub fn new() -> Self {
            PcRx
        }
        pub fn set_search_limit(&mut self, _n: Option<usize>) {}
        pub fn set_bicm_id(&mut self, _on: bool) {}
        pub fn set_ldpc_iters(&mut self, _n: usize) {}
        pub fn sig_core(&mut self, _pre: &[Complex32], _freq: &[Complex32]) -> (SigPayload, bool) {
            (SigPayload::default(), false)
        }
        pub fn demodulate_sig(&mut self, _rx: &[Complex32]) -> Option<(SigPayload, bool)> {
            None
        }
        pub fn demodulate_to_llrs(&mut self, _rx: &[Complex32], _mcs: Mcs) -> DemodResult {
            DemodResult::failed()
        }
        pub fn llrs_from_grid(
            &mut self,
            _grid: &[Vec<Complex32>],
            _mcs: Mcs,
            _sync_metric: f32,
            _cfo_hz: f32,
        ) -> DemodResult {
            DemodResult::failed()
        }
        pub fn redemap_with_priors(
            &self,
            _mcs: Mcs,
            _eq_syms: &[(Complex32, f32)],
            _priors: &[f32],
        ) -> Vec<f32> {
            Vec::new()
        }
        pub fn decode_buffers(&self, _buffers: &[f32], _posteriors: Option<&mut [f32]>) -> DecodeOutcome {
            DecodeOutcome { payload: Vec::new(), cw_ok: vec![false; CODEWORDS_PER_FRAME], pre_ber: 0.0 }
        }
    }

    pub fn schmidl_cox(_rx: &[Complex32]) -> Option<SyncResult> {
        None
    }
    pub const FRAME_BUFFER: usize = 1;
    pub fn descramble_llrs(_llrs: &mut [f32]) {}
    pub fn interleave_stride(_t: usize) -> usize {
        1
    }
    pub fn scatter_frame_llrs(_mcs: Mcs, _rv: u8, _llrs: &[f32], _buffers: &mut [f32]) {}
    pub fn gather_extrinsic_priors(_mcs: Mcs, _rv: u8, _posteriors: &[f32], _llrs: &[f32]) -> Vec<f32> {
        Vec::new()
    }
    pub fn data_syms(_m: Mcs) -> usize {
        0
    }
    pub fn frame_len_samples(_m: Mcs) -> usize {
        0
    }
    pub fn coded_bits(_m: Mcs) -> usize {
        0
    }
    pub fn set_samp_rate_hz(_hz: u64) {}

    /// Log once that a raw capture was ignored.
    pub fn warn_no_pc_demod() {
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            nyx_common::logging::log(
                "rx: this build has no PC demodulator; raw IQ/LLR captures are ignored, \
                 only frames decoded on the board (rx2) are used",
            );
        });
    }
}

pub use imp::*;
