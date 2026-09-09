//! PC-side modulator for the "PC IQ (test)" transmit path. It is only present
//! when the crate is built with the `pcphy` feature (development tree); the
//! public build always sends bits to the board's modulator.

use num_complex::Complex32;
use nyx_proto::Mcs;

#[cfg(feature = "pcphy")]
#[path = "pc_phy.rs"]
mod imp;

#[cfg(not(feature = "pcphy"))]
mod imp {
    use super::*;

    pub struct PcTx;

    impl PcTx {
        pub const AVAILABLE: bool = false;

        pub fn new() -> Self {
            PcTx
        }

        pub fn modulate_frame(&mut self, _payload: &[u8], _mcs: Mcs, _rv: u8, _seq_lsb: u8) -> Vec<Complex32> {
            Vec::new()
        }
    }
}

pub use imp::PcTx;
