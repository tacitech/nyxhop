//! nyx-proto: TCP wire protocol between the TX, channel and RX processes.
//!
//! Framing: `[u32 le body_len][u8 msg_type][body]`.
//!
//! The baseband path carries `IqFrame` (one PHY frame of complex samples,
//! plus the PHY seq number and the MCS it was modulated with — the MCS
//! field stands in for an ideal control channel / PDCCH). The return path
//! carries `Feedback` (link quality telemetry for MCS adaptation) and
//! `Nack` (per-PHY-frame retransmission requests).

use std::io::{self, Read, Write};
use std::net::TcpStream;

use num_complex::Complex32;

pub const DEFAULT_TX_PORT: u16 = 7010;
pub const DEFAULT_RX_PORT: u16 = 7011;

const MAX_BODY: usize = 64 * 1024 * 1024;

// --- 16-bit converter (DAC/ADC) model ------------------------------------
// The wire carries 16-bit I/Q exactly like the FPGA front-end would: the
// TX quantizes its unit-power baseband at -12 dBFS (PAPR head-room), the
// channel node re-quantizes its "analog" output through a per-frame AGC at
// -15 dBFS. At 16 bits both operating points leave ~85 dB SQNR — far above
// any channel SNR simulated here — so the sole real effect is faithful
// hardware clipping behaviour (counted and logged).

/// DAC gain: unit-RMS baseband -> -12 dBFS.
pub const DAC_SCALE: f32 = 32767.0 / 3.9811; // 10^(12/20)
/// RX AGC target RMS at the ADC input (-15 dBFS).
pub const ADC_TARGET_RMS: f32 = 32767.0 / 5.6234; // 10^(15/20)

/// One 16-bit I/Q sample.
pub type Iq16 = (i16, i16);

/// Quantize float baseband to 16-bit with saturation; returns clip count.
pub fn quantize(samples: &[Complex32], scale: f32) -> (Vec<Iq16>, u64) {
    let mut clips = 0u64;
    let mut q = |x: f32| -> i16 {
        let v = (x * scale).round();
        if v > 32767.0 {
            clips += 1;
            32767
        } else if v < -32768.0 {
            clips += 1;
            -32768
        } else {
            v as i16
        }
    };
    let out = samples.iter().map(|s| (q(s.re), q(s.im))).collect();
    (out, clips)
}

/// Convert 16-bit I/Q back to float with the given gain.
pub fn dequantize(samples: &[Iq16], gain: f32) -> Vec<Complex32> {
    samples
        .iter()
        .map(|&(i, q)| Complex32::new(i as f32 * gain, q as f32 * gain))
        .collect()
}

/// v40.33 licence file format (tool, daemon and apps share it).
pub mod lic;
pub mod mcs;

pub use mcs::{
    CONV_MCS_COUNT, FRAME_PAYLOAD_BYTES, Mcs, RV_SEQUENCE, TXB_CONV, TXB_FABRIC, TXB_FILV,
    TXB_PAIR, conv_frames, conv_payload_bytes,
};

#[derive(Debug, Clone)]
pub enum Msg {
    /// One PHY frame of baseband IQ as 16-bit converter samples (the
    /// FPGA <-> CPU interface format). `rv` is the LDPC redundancy version
    /// (IR-HARQ); retransmissions carry a different rv.
    IqFrame { seq: u64, mcs: u8, rv: u8, samples: Vec<Iq16> },
    /// RX -> TX link telemetry (relayed transparently by the channel node).
    /// `need_idr` asks the video encoder for a keyframe after loss.
    /// v32 minstrel: ok_mcs = dem BLOCK giai ok RIENG TUNG MCS (wrap u16)
    /// — thong ke per-rate kieu minstrel, TX tinh prob_i = dok_i/dsent_i.
    Feedback { snr_db: f32, bler: f32, segs_ok: u64, segs_lost: u64, need_idr: bool, ok_mcs: [u16; 6], ok_base: u16 },
    /// RX -> TX: PHY frame `seq` failed to decode, please retransmit.
    Nack { seq: u64 },
    /// Keepalive.
    Ping,
    /// Capture demodulated in the fabric (v1.5): H = LS channel estimate (600 used subcarriers,
    /// from the frame 1 preamble), Y = n_syms x 600 subcarriers on the grid us + s*1096
    /// (back-to-back frames sit ON the grid, so the next frame's preamble/SIG are symbols in the
    /// sequence too). n_syms = y.len()/600. us/theta are diagnostic only (best effort, may be off
    /// by a capture).
    EqFrame { seq: u64, us: u16, theta: i32, h: Vec<Iq16>, y: Vec<Iq16> },
    /// v2: capture DEMAPPED in the fabric: 6-bit LLRs packed 5 per word (bits [29:0], first LLR in
    /// [5:0]) for the DATA symbols of frame 1. SIG is Polar-decoded by the ARM: mcs/rv/seq_lsb
    /// attached, the PC touches no DSP.
    LlrFrame {
        seq: u64,
        us: u16,
        theta: i32,
        mcs: u8,
        rv: u8,
        seq_lsb: u8,
        words: Vec<u32>,
    },
    /// v4.2: frame FULLY DECODED in the fabric (demap + LDPC): payload FRAME_PAYLOAD_BYTES (2x1008,
    /// CRC16 per codeword checked and stripped in the daemon). Sent only when both codewords are ok
    /// (on failure the daemon falls back to LlrFrame for PC HARQ). llr_sum = Σ|6-bit llr| of the
    /// frame (SNR heuristic: mean = llr_sum / t). iters = total iterations of the 2 codewords.
    DecFrame {
        seq: u64,
        mcs: u8,
        seq_lsb: u8,
        iters: u8,
        llr_sum: u32,
        payload: Vec<u8>,
    },
    /// v3.1: TX frame as BITS for the fabric modulator: words = the ready DMA packet layout (header
    /// {n_syms<<2|mbits} + 4 SIG words + t/32 data words); the daemon only adds MAGIC and pushes
    /// DMA. mcs/rv are for the daemon's bookkeeping/log only.
    TxBits { seq: u64, mcs: u8, rv: u8, words: Vec<u32> },
    /// v4.3: TX frame as PAYLOAD for the fabric encoder: words = {hdr, cfg, 4 SIG words, 504
    /// payload words}; the daemon adds MAGIC 0x4E4D_5459 and DMAs; the fabric LDPC-encodes and
    /// interleaves/scrambles itself.
    TxPayload { seq: u64, mcs: u8, rv: u8, words: Vec<u32> },
    /// v40.40: one raw payload block for the board to frame, encode and modulate
    /// (`flags`: TXB_*). The PC builds no modulator words any more.
    TxBlock { seq: u64, mcs: u8, rv: u8, flags: u8, payload: Vec<u8> },
    /// v9.3: RX -> TX over the control channel: choose the HOPPING SCHEDULE. The RX (the end with
    /// the screen, like a DJI controller) picks the mode and tells the TX so BOTH use the same
    /// channel set + seed/dwell (prediction matches). mode: 0 manual 1 band24 2 band58 3 full.
    HopCmd { mode: u8, seed: u64, dwell: u32, mask: u16 },
    /// v12: text / parameters RX -> TX over the OTA control channel (inside the ctrl frame like
    /// Feedback). Limited to 400 B to fit beside the feedback.
    UserText { text: String },
    /// v40.22: telemetry (MAVLink...) RX->TX qua kenh control hep — 1 datagram
    /// UDP = 1 Msg (<= 210 B = 15 manh x 14 B tren song).
    Tlm { bytes: Vec<u8> },
    /// v40.29 time-mode AFH: "from epoch (epoch_hi<<8 | eff_e8) the video schedule
    /// runs on this channel mask". Relayed by the control receiver (A) from the
    /// narrow control frame; also carries the high epoch byte the receive end
    /// cannot get from w9.
    HopMask { epoch_hi: u8, eff_e8: u8, mask: u128 },
    /// v40.30: the receive end's schedule descriptor, relayed from the narrow
    /// control frame: video hop mode (hop::HopMode::as_u8), flags (bit0 clock
    /// exists, bit1 control slotted), time-mode dwell and seed, control slots per
    /// epoch, the sender's clock (epoch_hi, e8) and its table sizes. The transmit
    /// end enters/leaves time mode from it instead of from a console command.
    HopDesc { mode: u8, flags: u8, dwell_ms: u16, seed: u64, ctl_k: u8, epoch_hi: u8, e8: u8, n_video: u8, n_ctl: u8, tag8: u8 },
    /// v40.32: a LINK (bind) frame heard on the link channel, relayed from the narrow
    /// control frame: the master's unit id and the pair key it chose.
    LinkBind { unit: u32, key: u64, chan_mhz: u16 },
    /// v40.31: the transmit-end daemon tells nyx-tx which channel the video is on
    /// now (Hz) so its rate control can keep per-channel memory.
    TxChan { hz: u64 },
    /// v40.33: the daemon's licence state for the app on the other end of the
    /// video link (nyx-tx forwards it in-band): device DNA, state (0 locked,
    /// 1 trial, 2 licensed), minutes used, trial length, PL gate version.
    LicInfo { dna: u64, state: u8, minutes: u32, trial_min: u32, ver: u8 },
    /// v40.33: a licence core heard on the narrow control channel (LIC frames from
    /// the receive end) - dna/aux/word, the PL decides.
    LicPush { dna: u64, aux: u64, word: u64 },
    /// v40.37: one chunk of the receive end's channel table heard on the control
    /// channel (kind 0 video, 1 control pool; channels in 0.1 MHz; eff epoch 0,0 = now).
    HopTable { kind: u8, idx: u8, n: u8, tag: u8, epoch_hi: u8, eff_e8: u8, mhz10: Vec<u16> },
}

impl Msg {
    fn type_byte(&self) -> u8 {
        match self {
            Msg::IqFrame { .. } => 1,
            Msg::Feedback { .. } => 2,
            Msg::Nack { .. } => 3,
            Msg::Ping => 4,
            Msg::EqFrame { .. } => 5,
            Msg::LlrFrame { .. } => 6,
            Msg::TxBits { .. } => 7,
            Msg::DecFrame { .. } => 8,
            Msg::TxPayload { .. } => 9,
            Msg::HopCmd { .. } => 10,
            Msg::UserText { .. } => 11,
            Msg::Tlm { .. } => 12,
            Msg::HopMask { .. } => 13,
            Msg::HopDesc { .. } => 14,
            Msg::LinkBind { .. } => 16,
            Msg::TxChan { .. } => 15,
            Msg::LicInfo { .. } => 17,
            Msg::LicPush { .. } => 18,
            Msg::HopTable { .. } => 19,
            Msg::TxBlock { .. } => 20,
        }
    }
}

/// Encode one Msg as a frame [u32 len][u8 type][body], shared by the TCP stream and the UDP
/// payload.
pub fn encode_msg(msg: &Msg) -> Vec<u8> {
    let mut body = Vec::new();
    match msg {
        Msg::IqFrame { seq, mcs, rv, samples } => {
            body.reserve(14 + samples.len() * 4);
            body.extend_from_slice(&seq.to_le_bytes());
            body.push(*mcs);
            body.push(*rv);
            body.extend_from_slice(&(samples.len() as u32).to_le_bytes());
            for &(i, q) in samples {
                body.extend_from_slice(&i.to_le_bytes());
                body.extend_from_slice(&q.to_le_bytes());
            }
        }
        Msg::Feedback { snr_db, bler, segs_ok, segs_lost, need_idr, ok_mcs, ok_base } => {
            body.extend_from_slice(&snr_db.to_le_bytes());
            body.extend_from_slice(&bler.to_le_bytes());
            body.extend_from_slice(&segs_ok.to_le_bytes());
            body.extend_from_slice(&segs_lost.to_le_bytes());
            body.push(*need_idr as u8);
            for v in ok_mcs {
                body.extend_from_slice(&v.to_le_bytes());
            }
            body.extend_from_slice(&ok_base.to_le_bytes());
        }
        Msg::Nack { seq } => body.extend_from_slice(&seq.to_le_bytes()),
        Msg::HopCmd { mode, seed, dwell, mask } => {
            body.push(*mode);
            body.extend_from_slice(&seed.to_le_bytes());
            body.extend_from_slice(&dwell.to_le_bytes());
            body.extend_from_slice(&mask.to_le_bytes());
        }
        Msg::HopMask { epoch_hi, eff_e8, mask } => {
            body.push(*epoch_hi);
            body.push(*eff_e8);
            body.extend_from_slice(&mask.to_le_bytes());
        }
        Msg::TxChan { hz } => body.extend_from_slice(&hz.to_le_bytes()),
        Msg::HopDesc { mode, flags, dwell_ms, seed, ctl_k, epoch_hi, e8, n_video, n_ctl, tag8 } => {
            body.push(*mode);
            body.push(*flags);
            body.extend_from_slice(&dwell_ms.to_le_bytes());
            body.extend_from_slice(&seed.to_le_bytes());
            body.extend_from_slice(&[*ctl_k, *epoch_hi, *e8, *n_video, *n_ctl, *tag8]);
        }
        Msg::LinkBind { unit, key, chan_mhz } => {
            body.extend_from_slice(&unit.to_le_bytes());
            body.extend_from_slice(&key.to_le_bytes());
            body.extend_from_slice(&chan_mhz.to_le_bytes());
        }
        Msg::LicInfo { dna, state, minutes, trial_min, ver } => {
            body.extend_from_slice(&dna.to_le_bytes());
            body.push(*state);
            body.extend_from_slice(&minutes.to_le_bytes());
            body.extend_from_slice(&trial_min.to_le_bytes());
            body.push(*ver);
        }
        Msg::LicPush { dna, aux, word } => {
            body.extend_from_slice(&dna.to_le_bytes());
            body.extend_from_slice(&aux.to_le_bytes());
            body.extend_from_slice(&word.to_le_bytes());
        }
        Msg::HopTable { kind, idx, n, tag, epoch_hi, eff_e8, mhz10 } => {
            body.extend_from_slice(&[*kind, *idx, *n, *tag, *epoch_hi, *eff_e8, mhz10.len().min(255) as u8]);
            for c in mhz10.iter().take(255) {
                body.extend_from_slice(&c.to_le_bytes());
            }
        }
        Msg::Ping => {}
        Msg::UserText { text } => {
            let b = text.as_bytes();
            let n = b.len().min(400);
            body.extend_from_slice(&(n as u16).to_le_bytes());
            body.extend_from_slice(&b[..n]);
        }
        Msg::Tlm { bytes } => {
            let n = bytes.len().min(512);
            body.extend_from_slice(&(n as u16).to_le_bytes());
            body.extend_from_slice(&bytes[..n]);
        }
        Msg::LlrFrame { seq, us, theta, mcs, rv, seq_lsb, words } => {
            body.reserve(21 + words.len() * 4);
            body.extend_from_slice(&seq.to_le_bytes());
            body.extend_from_slice(&us.to_le_bytes());
            body.extend_from_slice(&theta.to_le_bytes());
            body.push(*mcs);
            body.push(*rv);
            body.push(*seq_lsb);
            body.extend_from_slice(&(words.len() as u32).to_le_bytes());
            for w in words {
                body.extend_from_slice(&w.to_le_bytes());
            }
        }
        Msg::TxPayload { seq, mcs, rv, words }
        | Msg::TxBits { seq, mcs, rv, words } => {
            body.reserve(14 + words.len() * 4);
            body.extend_from_slice(&seq.to_le_bytes());
            body.push(*mcs);
            body.push(*rv);
            body.extend_from_slice(&(words.len() as u32).to_le_bytes());
            for w in words {
                body.extend_from_slice(&w.to_le_bytes());
            }
        }
        Msg::TxBlock { seq, mcs, rv, flags, payload } => {
            body.reserve(15 + payload.len());
            body.extend_from_slice(&seq.to_le_bytes());
            body.push(*mcs);
            body.push(*rv);
            body.push(*flags);
            body.extend_from_slice(&(payload.len() as u32).to_le_bytes());
            body.extend_from_slice(payload);
        }
        Msg::DecFrame { seq, mcs, seq_lsb, iters, llr_sum, payload } => {
            body.reserve(19 + payload.len());
            body.extend_from_slice(&seq.to_le_bytes());
            body.push(*mcs);
            body.push(*seq_lsb);
            body.push(*iters);
            body.extend_from_slice(&llr_sum.to_le_bytes());
            body.extend_from_slice(&(payload.len() as u32).to_le_bytes());
            body.extend_from_slice(payload);
        }
        Msg::EqFrame { seq, us, theta, h, y } => {
            body.reserve(22 + (h.len() + y.len()) * 4);
            body.extend_from_slice(&seq.to_le_bytes());
            body.extend_from_slice(&us.to_le_bytes());
            body.extend_from_slice(&theta.to_le_bytes());
            body.extend_from_slice(&(h.len() as u32).to_le_bytes());
            body.extend_from_slice(&(y.len() as u32).to_le_bytes());
            for &(i, q) in h.iter().chain(y.iter()) {
                body.extend_from_slice(&i.to_le_bytes());
                body.extend_from_slice(&q.to_le_bytes());
            }
        }
    }
    let mut head = Vec::with_capacity(5 + body.len());
    head.extend_from_slice(&(body.len() as u32 + 1).to_le_bytes());
    head.push(msg.type_byte());
    head.extend_from_slice(&body);
    head
}

/// v7: over-the-air CONTROL frame: Feedback/Nack packed into the 2016 B payload of an ordinary PHY
/// frame (the RX -> TX direction flies over the air, no LAN cable between the ends any more).
/// Layout: magic 4 B + ver 1 B + n 1 B + n times encode_msg (self-delimiting). The rest is zero
/// padding.
pub const CTRL_MAGIC: [u8; 4] = *b"NYXC";

pub fn pack_ctrl(msgs: &[Msg], payload_len: usize) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(payload_len);
    out.extend_from_slice(&CTRL_MAGIC);
    out.push(1); // ver
    out.push(0); // n, patched afterwards
    let mut n = 0u8;
    for m in msgs {
        let b = encode_msg(m);
        if out.len() + b.len() > payload_len || n == u8::MAX {
            break;
        }
        out.extend_from_slice(&b);
        n += 1;
    }
    if n == 0 {
        return None;
    }
    out[5] = n;
    out.resize(payload_len, 0);
    Some(out)
}

pub fn unpack_ctrl(payload: &[u8]) -> Option<Vec<Msg>> {
    if payload.len() < 6 || payload[0..4] != CTRL_MAGIC || payload[4] != 1 {
        return None;
    }
    let n = payload[5] as usize;
    let mut msgs = Vec::with_capacity(n);
    let mut off = 6usize;
    for _ in 0..n {
        if off + 4 > payload.len() {
            return None;
        }
        let len =
            u32::from_le_bytes(payload[off..off + 4].try_into().unwrap())
                as usize;
        if len == 0 || len > MAX_BODY || off + 4 + len > payload.len() {
            return None;
        }
        msgs.push(decode_msg(&payload[off + 4..off + 4 + len]).ok()?);
        off += 4 + len;
    }
    Some(msgs)
}

pub fn write_msg(stream: &mut TcpStream, msg: &Msg) -> io::Result<()> {
    stream.write_all(&encode_msg(msg))
}

pub fn read_msg(stream: &mut TcpStream) -> io::Result<Msg> {
    let mut len4 = [0u8; 4];
    stream.read_exact(&mut len4)?;
    let len = u32::from_le_bytes(len4) as usize;
    if len == 0 || len > MAX_BODY {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "bad frame length"));
    }
    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf)?;
    decode_msg(&buf)
}

/// Decode the [u8 type][body] part (after the length field), the variant used for UDP.
pub fn decode_msg(buf: &[u8]) -> io::Result<Msg> {
    if buf.is_empty() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "empty message"));
    }
    let ty = buf[0];
    let body = &buf[1..];
    let err = || io::Error::new(io::ErrorKind::InvalidData, "truncated message");
    match ty {
        1 => {
            if body.len() < 14 {
                return Err(err());
            }
            let seq = u64::from_le_bytes(body[0..8].try_into().unwrap());
            let mcs = body[8];
            let rv = body[9];
            let n = u32::from_le_bytes(body[10..14].try_into().unwrap()) as usize;
            if body.len() != 14 + n * 4 {
                return Err(err());
            }
            let mut samples = Vec::with_capacity(n);
            for i in 0..n {
                let o = 14 + i * 4;
                let iv = i16::from_le_bytes(body[o..o + 2].try_into().unwrap());
                let qv = i16::from_le_bytes(body[o + 2..o + 4].try_into().unwrap());
                samples.push((iv, qv));
            }
            Ok(Msg::IqFrame { seq, mcs, rv, samples })
        }
        2 => {
            if ![25usize, 37, 39].contains(&body.len()) {
                return Err(err());
            }
            let mut ok_mcs = [0u16; 6];
            if body.len() >= 37 {
                for (i, m) in ok_mcs.iter_mut().enumerate() {
                    *m = u16::from_le_bytes(
                        body[25 + i * 2..27 + i * 2].try_into().unwrap(),
                    );
                }
            }
            let ok_base = if body.len() >= 39 {
                u16::from_le_bytes(body[37..39].try_into().unwrap())
            } else {
                0xFFFF // sentinel: an old feedback does not carry this number
            };
            Ok(Msg::Feedback {
                snr_db: f32::from_le_bytes(body[0..4].try_into().unwrap()),
                bler: f32::from_le_bytes(body[4..8].try_into().unwrap()),
                segs_ok: u64::from_le_bytes(body[8..16].try_into().unwrap()),
                segs_lost: u64::from_le_bytes(body[16..24].try_into().unwrap()),
                need_idr: body[24] != 0,
                ok_mcs,
                ok_base,
            })
        }
        3 => {
            if body.len() != 8 {
                return Err(err());
            }
            Ok(Msg::Nack { seq: u64::from_le_bytes(body.try_into().unwrap()) })
        }
        13 => {
            if body.len() < 18 {
                return Err(err());
            }
            Ok(Msg::HopMask {
                epoch_hi: body[0],
                eff_e8: body[1],
                mask: u128::from_le_bytes(body[2..18].try_into().unwrap()),
            })
        }
        15 => {
            if body.len() < 8 {
                return Err(err());
            }
            Ok(Msg::TxChan { hz: u64::from_le_bytes(body[..8].try_into().unwrap()) })
        }
        14 => {
            if body.len() < 18 {
                return Err(err());
            }
            Ok(Msg::HopDesc {
                mode: body[0],
                flags: body[1],
                dwell_ms: u16::from_le_bytes(body[2..4].try_into().unwrap()),
                seed: u64::from_le_bytes(body[4..12].try_into().unwrap()),
                ctl_k: body[12],
                epoch_hi: body[13],
                e8: body[14],
                n_video: body[15],
                n_ctl: body[16],
                tag8: body[17],
            })
        }
        16 => {
            if body.len() < 14 {
                return Err(err());
            }
            Ok(Msg::LinkBind {
                unit: u32::from_le_bytes(body[0..4].try_into().unwrap()),
                key: u64::from_le_bytes(body[4..12].try_into().unwrap()),
                chan_mhz: u16::from_le_bytes(body[12..14].try_into().unwrap()),
            })
        }
        17 => {
            if body.len() < 18 {
                return Err(err());
            }
            Ok(Msg::LicInfo {
                dna: u64::from_le_bytes(body[0..8].try_into().unwrap()),
                state: body[8],
                minutes: u32::from_le_bytes(body[9..13].try_into().unwrap()),
                trial_min: u32::from_le_bytes(body[13..17].try_into().unwrap()),
                ver: body[17],
            })
        }
        18 => {
            if body.len() < 24 {
                return Err(err());
            }
            Ok(Msg::LicPush {
                dna: u64::from_le_bytes(body[0..8].try_into().unwrap()),
                aux: u64::from_le_bytes(body[8..16].try_into().unwrap()),
                word: u64::from_le_bytes(body[16..24].try_into().unwrap()),
            })
        }
        20 => {
            if body.len() < 15 {
                return Err(err());
            }
            let seq = u64::from_le_bytes(body[0..8].try_into().unwrap());
            let n = u32::from_le_bytes(body[11..15].try_into().unwrap()) as usize;
            if body.len() != 15 + n || n > 65536 {
                return Err(err());
            }
            Ok(Msg::TxBlock {
                seq,
                mcs: body[8],
                rv: body[9],
                flags: body[10],
                payload: body[15..].to_vec(),
            })
        }
        19 => {
            if body.len() < 7 {
                return Err(err());
            }
            let k = usize::from(body[6]);
            if body.len() < 7 + 2 * k {
                return Err(err());
            }
            let mhz10 = (0..k).map(|i| u16::from_le_bytes([body[7 + 2 * i], body[8 + 2 * i]])).collect();
            Ok(Msg::HopTable { kind: body[0], idx: body[1], n: body[2], tag: body[3], epoch_hi: body[4], eff_e8: body[5], mhz10 })
        }
        10 => {
            if body.len() < 15 {
                return Err(err());
            }
            Ok(Msg::HopCmd {
                mode: body[0],
                seed: u64::from_le_bytes(body[1..9].try_into().unwrap()),
                dwell: u32::from_le_bytes(body[9..13].try_into().unwrap()),
                mask: u16::from_le_bytes(body[13..15].try_into().unwrap()),
            })
        }
        4 => Ok(Msg::Ping),
        11 => {
            if body.len() < 2 {
                return Err(err());
            }
            let n = u16::from_le_bytes(body[0..2].try_into().unwrap()) as usize;
            if n > 400 || body.len() != 2 + n {
                return Err(err());
            }
            let text = String::from_utf8_lossy(&body[2..2 + n]).into_owned();
            Ok(Msg::UserText { text })
        }
        12 => {
            if body.len() < 2 {
                return Err(err());
            }
            let n = u16::from_le_bytes(body[0..2].try_into().unwrap()) as usize;
            if n > 512 || body.len() != 2 + n {
                return Err(err());
            }
            Ok(Msg::Tlm { bytes: body[2..2 + n].to_vec() })
        }
        5 => {
            if body.len() < 22 {
                return Err(err());
            }
            let seq = u64::from_le_bytes(body[0..8].try_into().unwrap());
            let us = u16::from_le_bytes(body[8..10].try_into().unwrap());
            let theta = i32::from_le_bytes(body[10..14].try_into().unwrap());
            let nh = u32::from_le_bytes(body[14..18].try_into().unwrap()) as usize;
            let ny = u32::from_le_bytes(body[18..22].try_into().unwrap()) as usize;
            if body.len() != 22 + (nh + ny) * 4 || nh > 4096 || ny > 65536 {
                return Err(err());
            }
            let pair = |o: usize| -> Iq16 {
                (
                    i16::from_le_bytes(body[o..o + 2].try_into().unwrap()),
                    i16::from_le_bytes(body[o + 2..o + 4].try_into().unwrap()),
                )
            };
            let h = (0..nh).map(|i| pair(22 + i * 4)).collect();
            let y = (0..ny).map(|i| pair(22 + (nh + i) * 4)).collect();
            Ok(Msg::EqFrame { seq, us, theta, h, y })
        }
        6 => {
            if body.len() < 21 {
                return Err(err());
            }
            let seq = u64::from_le_bytes(body[0..8].try_into().unwrap());
            let us = u16::from_le_bytes(body[8..10].try_into().unwrap());
            let theta = i32::from_le_bytes(body[10..14].try_into().unwrap());
            let mcs = body[14];
            let rv = body[15];
            let seq_lsb = body[16];
            let n = u32::from_le_bytes(body[17..21].try_into().unwrap()) as usize;
            if body.len() != 21 + n * 4 || n > 16384 {
                return Err(err());
            }
            let words = (0..n)
                .map(|i| {
                    u32::from_le_bytes(
                        body[21 + i * 4..25 + i * 4].try_into().unwrap(),
                    )
                })
                .collect();
            Ok(Msg::LlrFrame { seq, us, theta, mcs, rv, seq_lsb, words })
        }
        8 => {
            if body.len() < 19 {
                return Err(err());
            }
            let seq = u64::from_le_bytes(body[0..8].try_into().unwrap());
            let mcs = body[8];
            let seq_lsb = body[9];
            let iters = body[10];
            let llr_sum = u32::from_le_bytes(body[11..15].try_into().unwrap());
            let n = u32::from_le_bytes(body[15..19].try_into().unwrap()) as usize;
            if body.len() != 19 + n || n > 4096 {
                return Err(err());
            }
            let payload = body[19..19 + n].to_vec();
            Ok(Msg::DecFrame { seq, mcs, seq_lsb, iters, llr_sum, payload })
        }
        7 | 9 => {
            if body.len() < 14 {
                return Err(err());
            }
            let seq = u64::from_le_bytes(body[0..8].try_into().unwrap());
            let mcs = body[8];
            let rv = body[9];
            let n = u32::from_le_bytes(body[10..14].try_into().unwrap()) as usize;
            if body.len() != 14 + n * 4 || n > 4096 {
                return Err(err());
            }
            let words = (0..n)
                .map(|i| {
                    u32::from_le_bytes(
                        body[14 + i * 4..18 + i * 4].try_into().unwrap(),
                    )
                })
                .collect();
            if ty == 9 {
                Ok(Msg::TxPayload { seq, mcs, rv, words })
            } else {
                Ok(Msg::TxBits { seq, mcs, rv, words })
            }
        }
        _ => Err(io::Error::new(io::ErrorKind::InvalidData, "unknown message type")),
    }
}

// ---- UDP capture transport --------------------------------------------
//
// The board -> PC capture path over UDP: connectionless (a daemon restart leaves no ghost socket),
// loss-tolerant by design (a lost capture = a loss on air, ARQ handles it). The client registers
// with a SUB packet every second; the daemon remembers the source address and streams captures back
// as chunk sequences <= MTU (avoiding IP fragmentation). The client's Feedback/Nack go back on the
// same socket (an MSG packet wrapping a Msg frame).

/// Default UDP port of the daemon for capture + feedback.
pub const UDP_CAPTURE_PORT: u16 = 7012;
/// Client -> daemon packet: register/keepalive (1 byte).
pub const UDP_SUB: u8 = 0x01;
/// Client -> daemon packet: [0x02][Msg frame as on TCP] (Feedback/Nack).
pub const UDP_MSG: u8 = 0x02;
/// Iq16 samples per chunk: 350 x 4 B + 20 B header = 1420 B < MTU 1500.
pub const CHUNK_SAMPLES: usize = 350;
const CHUNK_MAGIC: u32 = 0x4E59_4C4B; // "NYLK"
const CHUNK_HDR: usize = 20;

/// Header of one capture chunk (daemon -> client).
pub struct ChunkHdr {
    pub cap_seq: u64,
    pub mcs: u8,
    pub rv: u8,
    pub chunk: u16,
    pub nchunks: u16,
}

/// Pack one chunk into `out` (buffer reused, no allocation).
pub fn pack_chunk(
    out: &mut Vec<u8>,
    hdr: &ChunkHdr,
    payload: &[Iq16],
) {
    out.clear();
    out.extend_from_slice(&CHUNK_MAGIC.to_le_bytes());
    out.extend_from_slice(&hdr.cap_seq.to_le_bytes());
    out.push(hdr.mcs);
    out.push(hdr.rv);
    out.extend_from_slice(&hdr.chunk.to_le_bytes());
    out.extend_from_slice(&hdr.nchunks.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    for &(i, q) in payload {
        out.extend_from_slice(&i.to_le_bytes());
        out.extend_from_slice(&q.to_le_bytes());
    }
}

/// Decode one chunk datagram; None if it is not a valid chunk.
pub fn parse_chunk(d: &[u8]) -> Option<(ChunkHdr, Vec<Iq16>)> {
    if d.len() < CHUNK_HDR
        || u32::from_le_bytes(d[0..4].try_into().ok()?) != CHUNK_MAGIC
    {
        return None;
    }
    let n = u16::from_le_bytes(d[18..20].try_into().ok()?) as usize;
    if d.len() != CHUNK_HDR + n * 4 {
        return None;
    }
    let hdr = ChunkHdr {
        cap_seq: u64::from_le_bytes(d[4..12].try_into().ok()?),
        mcs: d[12],
        rv: d[13],
        chunk: u16::from_le_bytes(d[14..16].try_into().ok()?),
        nchunks: u16::from_le_bytes(d[16..18].try_into().ok()?),
    };
    let mut samples = Vec::with_capacity(n);
    for k in 0..n {
        let o = CHUNK_HDR + k * 4;
        samples.push((
            i16::from_le_bytes(d[o..o + 2].try_into().ok()?),
            i16::from_le_bytes(d[o + 2..o + 4].try_into().ok()?),
        ));
    }
    Some((hdr, samples))
}

/// Parse `--flag value` style CLI options; returns default when absent.
pub fn cli_arg(name: &str, default: &str) -> String {
    let args: Vec<String> = std::env::args().collect();
    for i in 0..args.len().saturating_sub(1) {
        if args[i] == name {
            return args[i + 1].clone();
        }
    }
    default.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{TcpListener, TcpStream};

    #[test]
    fn txblock_roundtrip() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let payload: Vec<u8> = (0..2016u32).map(|i| (i * 7) as u8).collect();
        let want = payload.clone();
        let sender = std::thread::spawn(move || {
            let mut s = TcpStream::connect(addr).unwrap();
            write_msg(&mut s, &Msg::TxBlock { seq: 77, mcs: 5, rv: 2, flags: TXB_CONV | TXB_FILV, payload })
                .unwrap();
        });
        let (mut s, _) = listener.accept().unwrap();
        match read_msg(&mut s).unwrap() {
            Msg::TxBlock { seq, mcs, rv, flags, payload } => {
                assert_eq!((seq, mcs, rv, flags), (77, 5, 2, TXB_CONV | TXB_FILV));
                assert_eq!(payload, want);
            }
            other => panic!("wrong variant {:?}", std::mem::discriminant(&other)),
        }
        sender.join().unwrap();
    }

    #[test]
    fn roundtrip_all_messages() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let sender = std::thread::spawn(move || {
            let mut s = TcpStream::connect(addr).unwrap();
            write_msg(
                &mut s,
                &Msg::IqFrame { seq: 42, mcs: 3, rv: 2, samples: vec![(1234, -567); 1000] },
            )
            .unwrap();
            write_msg(
                &mut s,
                &Msg::Feedback {
                    snr_db: 21.5,
                    bler: 0.125,
                    segs_ok: 10,
                    segs_lost: 2,
                    need_idr: true,
                    ok_mcs: [1, 2, 3, 4, 5, 6],
                    ok_base: 7,
                },
            )
            .unwrap();
            write_msg(&mut s, &Msg::Nack { seq: 7 }).unwrap();
            write_msg(&mut s, &Msg::Ping).unwrap();
        });
        let (mut r, _) = listener.accept().unwrap();
        match read_msg(&mut r).unwrap() {
            Msg::IqFrame { seq, mcs, rv, samples } => {
                assert_eq!((seq, mcs, rv, samples.len()), (42, 3, 2, 1000));
                assert_eq!(samples[999], (1234, -567));
            }
            _ => panic!("wrong message"),
        }
        match read_msg(&mut r).unwrap() {
            Msg::Feedback { snr_db, bler, segs_ok, segs_lost, need_idr, ok_mcs, ok_base } => {
                assert_eq!(
                    (snr_db, bler, segs_ok, segs_lost, need_idr),
                    (21.5, 0.125, 10, 2, true)
                );
                assert_eq!(ok_mcs, [1, 2, 3, 4, 5, 6]);
                assert_eq!(ok_base, 7);
            }
            _ => panic!("wrong message"),
        }
        assert!(matches!(read_msg(&mut r).unwrap(), Msg::Nack { seq: 7 }));
        assert!(matches!(read_msg(&mut r).unwrap(), Msg::Ping));
        sender.join().unwrap();
    }

    #[test]
    fn converter_16bit_is_transparent() {
        // OFDM-like Gaussian baseband at unit RMS through DAC quantization
        // and back: EVM must sit far below any operating channel SNR.
        let mut state = 0x1234_5678_u64;
        let mut gauss = || {
            // crude CLT gaussian, good enough for an EVM check
            let mut acc = 0.0f32;
            for _ in 0..12 {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                acc += (state >> 40) as f32 / (1u64 << 24) as f32;
            }
            (acc - 6.0) * (1.0f32 / 1.0)
        };
        let samples: Vec<Complex32> = (0..20000)
            .map(|_| Complex32::new(gauss() * 0.7071, gauss() * 0.7071))
            .collect();
        let (q, clips) = quantize(&samples, DAC_SCALE);
        let back = dequantize(&q, 1.0 / DAC_SCALE);
        let mut sig = 0.0f64;
        let mut err = 0.0f64;
        for (a, b) in samples.iter().zip(&back) {
            sig += a.norm_sqr() as f64;
            err += (a - b).norm_sqr() as f64;
        }
        let evm_db = 10.0 * (err / sig).log10();
        assert!(evm_db < -70.0, "16-bit EVM too high: {evm_db:.1} dB");
        // At -12 dBFS back-off, 4-sigma+ peaks are rare.
        assert!(clips < 40, "unexpected clip count: {clips}");
    }
}

#[cfg(test)]
mod hop_desc_tests {
    use super::*;

    #[test]
    fn hop_desc_round_trips() {
        let m = Msg::HopDesc {
            mode: 7, flags: 3, dwell_ms: 1000, seed: 0x0123_4567_89AB_CDEF,
            ctl_k: 8, epoch_hi: 0x22, e8: 0xF1, n_video: 5, n_ctl: 4, tag8: 0x5A,
        };
        let b = encode_msg(&m);
        let back = decode_msg(&b[4..]).expect("decodes");
        // Msg carries no PartialEq (IqFrame payloads); compare the re-encoding.
        assert_eq!(encode_msg(&back), b);
        let c = encode_msg(&Msg::TxChan { hz: 5_755_000_000 });
        assert!(matches!(decode_msg(&c[4..]), Ok(Msg::TxChan { hz: 5_755_000_000 })));
        let l = encode_msg(&Msg::LinkBind { unit: 7, key: 0xABCD_0000_0000_0001, chan_mhz: 2432 });
        assert!(matches!(decode_msg(&l[4..]), Ok(Msg::LinkBind { unit: 7, key: 0xABCD_0000_0000_0001, chan_mhz: 2432 })));
        assert!(matches!(back, Msg::HopDesc { mode: 7, flags: 3, dwell_ms: 1000, ctl_k: 8, n_video: 5, n_ctl: 4, .. }));
    }
}

#[cfg(test)]
mod lic_msgs {
    use super::*;

    #[test]
    fn lic_messages_round_trip() {
        for m in [
            Msg::LicInfo { dna: 0x0123456789abcde, state: 2, minutes: 77, trial_min: 1200, ver: 1 },
            Msg::LicPush { dna: 0x0123456789abcde, aux: 0x0101000100001234, word: 0xdeadbeef01234567 },
            Msg::HopTable { kind: 1, idx: 0, n: 4, tag: 9, epoch_hi: 1, eff_e8: 2, mhz10: vec![24120, 24320, 24520, 24720] },
        ] {
            let b = encode_msg(&m);
            let back = decode_msg(&b[4..]).expect("decodes");
            assert_eq!(encode_msg(&back), b);
        }
    }
}
