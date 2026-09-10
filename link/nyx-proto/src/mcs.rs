//! Modulation and coding scheme of the video link as the protocol and the
//! user interface see it: the wire index (`u8` in the messages) and a label.
//! Frame properties that follow from the modem design (symbols per frame,
//! frame length, bit rate) are computed here too, so the apps need no modem.

/// Modulation and coding scheme of a video frame (LTE/NR style, SISO).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mcs {
    /// QPSK, code rate 1/2
    Qpsk12,
    /// QPSK, code rate 2/3
    Qpsk23,
    /// 16-QAM, code rate 1/2
    Qam16R12,
    /// 16-QAM, code rate 2/3
    Qam16R23,
    /// 64-QAM, code rate 2/3
    Qam64R23,
    /// 64-QAM, code rate 4/5
    Qam64R45,
}

impl Mcs {
    pub const ALL: [Mcs; 6] = [
        Mcs::Qpsk12,
        Mcs::Qpsk23,
        Mcs::Qam16R12,
        Mcs::Qam16R23,
        Mcs::Qam64R23,
        Mcs::Qam64R45,
    ];

    /// Index into `ALL`: the wire encoding of the MCS.
    pub fn index(self) -> usize {
        Mcs::ALL.iter().position(|&m| m == self).unwrap()
    }

    pub fn from_index(i: usize) -> Option<Mcs> {
        Mcs::ALL.get(i).copied()
    }

    /// Bits per QAM symbol.
    pub fn bits_per_sym(self) -> usize {
        match self {
            Mcs::Qpsk12 | Mcs::Qpsk23 => 2,
            Mcs::Qam16R12 | Mcs::Qam16R23 => 4,
            Mcs::Qam64R23 | Mcs::Qam64R45 => 6,
        }
    }

    /// Bits carried by the data symbols of one frame at this MCS. The board
    /// reports the sum of the LLR magnitudes of every frame it decodes
    /// (`Msg::DecFrame::llr_sum`); divided by this it gives the mean LLR
    /// magnitude the apps turn into an SNR estimate.
    pub fn frame_bit_capacity(self) -> usize {
        match self {
            Mcs::Qpsk12 => 33_300,
            Mcs::Qpsk23 => 25_200,
            Mcs::Qam16R12 => 34_200,
            Mcs::Qam16R23 => 25_200,
            Mcs::Qam64R23 => 27_000,
            Mcs::Qam64R45 => 21_600,
        }
    }

    /// Label for the user interface.
    pub fn label(self) -> &'static str {
        match self {
            Mcs::Qpsk12 => "MCS0  QPSK 1/2",
            Mcs::Qpsk23 => "MCS1  QPSK 2/3",
            Mcs::Qam16R12 => "MCS2  16QAM 1/2",
            Mcs::Qam16R23 => "MCS3  16QAM 2/3",
            Mcs::Qam64R23 => "MCS4  64QAM 2/3",
            Mcs::Qam64R45 => "MCS5  64QAM 4/5",
        }
    }
}

/// Payload bytes in one video frame (equals `nyx_link::BLOCK_BYTES`).
pub const FRAME_PAYLOAD_BYTES: usize = 2016;

impl Mcs {
    /// Data OFDM symbols in one frame at this MCS.
    pub fn data_syms_per_frame(self) -> usize {
        match self {
            Mcs::Qpsk12 => 37,
            Mcs::Qpsk23 => 28,
            Mcs::Qam16R12 => 19,
            Mcs::Qam16R23 => 14,
            Mcs::Qam64R23 => 10,
            Mcs::Qam64R45 => 8,
        }
    }

    /// Frame length in samples: preamble, SIG and the data symbols.
    pub fn frame_len_samples(self) -> usize {
        (2 + self.data_syms_per_frame()) * 1096
    }

    /// Net bit rate in bit/s with frames sent back to back at `samp_hz`.
    pub fn phy_bitrate(self, samp_hz: f64) -> f64 {
        let frame_secs = self.frame_len_samples() as f64 / samp_hz;
        (FRAME_PAYLOAD_BYTES * 8) as f64 / frame_secs
    }
}

/// Redundancy version of the n-th HARQ retransmission.
pub const RV_SEQUENCE: [u8; 4] = [0, 2, 3, 1];

/// Steps of the convolutional (robust) rate table the video path uses.
pub const CONV_MCS_COUNT: usize = 7;

/// Payload bytes one conv frame carries at rate step `cm` (0..CONV_MCS_COUNT).
pub fn conv_payload_bytes(cm: usize) -> usize {
    [447, 672, 897, 1347, 1797, 2022, 222][cm.min(CONV_MCS_COUNT - 1)]
}

/// Frames a block of `block_len` bytes takes at conv step `cm` (one header byte per frame).
pub fn conv_frames(cm: usize, block_len: usize) -> usize {
    block_len.div_ceil(conv_payload_bytes(cm) - 1).max(1)
}

/// `Msg::TxBlock` flags.
/// Conv path: the board fragments the block and conv-encodes every frame.
pub const TXB_CONV: u8 = 1;
/// The board hardware encoder codes the frames (otherwise the daemon codes them on the
/// ARM): the LDPC path, and with `TXB_CONV` MCS 0-5 when the bitstream supports it (mod
/// version >= 7; older ones fall back to the ARM). The fabric conv path writes seq 0 in SIG.
pub const TXB_FABRIC: u8 = 2;
/// The payload holds two blocks sent as a pair.
pub const TXB_PAIR: u8 = 4;
/// Full-frame interleave (conv path).
pub const TXB_FILV: u8 = 8;

#[cfg(test)]
mod tests {
    use super::Mcs;

    #[test]
    fn index_round_trips() {
        for (i, m) in Mcs::ALL.iter().enumerate() {
            assert_eq!(m.index(), i);
            assert_eq!(Mcs::from_index(i), Some(*m));
        }
        assert_eq!(Mcs::from_index(6), None);
    }

    #[test]
    fn capacity_is_a_whole_number_of_symbols() {
        // 450 data subcarriers per symbol: every capacity is symbols x 450 x bits.
        for m in Mcs::ALL {
            assert_eq!(m.frame_bit_capacity() % (450 * m.bits_per_sym()), 0, "{m:?}");
        }
    }
}
