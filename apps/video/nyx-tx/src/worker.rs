//! TX worker: source -> JPEG -> segmentation -> PHY modulation -> IqFrame
//! over TCP, plus ARQ retransmissions and feedback-driven MCS adaptation.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use nyx_common::codec::VideoEncoder;
use nyx_common::logging::{StatusLogger, log};
use nyx_common::source::{DataGen, PatternGen, RtspAu, RtspShared, SourceKind, bytes_to_image, resize_rgb};
use nyx_common::{RgbFrame, encode_jpeg};
use nyx_link::{BLOCK_BYTES as FRAME_PAYLOAD_BYTES, SourceType, packetize};
use nyx_proto::{Mcs, Msg, RV_SEQUENCE, TXB_FABRIC, TXB_PAIR, conv_frames};

use crate::net::Net;
use crate::pc::PcTx;
use crate::{Codec, Rolling, Shared, TxConfig};

/// Estimated-SNR thresholds per MCS, recalibrated after the NMS decoder,
/// log-MAP demap and phase-slope fix (AWGN waterfalls now 4/4/6/9/14/16 dB
/// channel SNR; the RX estimate reads ~2 dB above channel SNR).
const MCS_SNR_THRESH: [f32; 6] = [0.0, 4.0, 7.0, 10.5, 15.5, 17.5];

/// The threshold of rung `i` for the PHY the board runs (branch B: the ofdm64 PHY's own).
fn snr_thresh(i: usize) -> f32 {
    if crate::phy64() { nyx_proto::PHY64_SNR_THRESH[i.min(5)] } else { MCS_SNR_THRESH[i.min(5)] }
}
/// v40.45 interference regime: the receiver reports an SNR this far above the
/// current rung's threshold and still loses blocks. That loss is not the fading
/// channel (a lower rung would not fix it) but something else on the air - a
/// WiFi burst, another radio - hitting whole frames. Stepping down then only
/// lengthens the airtime of every block (more exposure) and costs bitrate.
/// Measured 16/9 on 2437 MHz next to a WiFi AP: SNR 24-29 dB, 15-22 % of the
/// video frames lost, minstrel walked MCS 5 -> 1, fps 30 -> 15, 1.9 Mbps ->
/// 0.18 Mbps, and the loss did not move. General rule, any band.
const INTERF_MARGIN_DB: f32 = 6.0;
/// v40.53: the repeat step below MCS0 - in when MCS0's blocks fall under this (two windows;
/// the stream itself is at MCS0 then, a hundred blocks a second, the number is a good one).
/// Not 0.75: a picture is several blocks, and MCS0 at 0.8-0.9 kept tearing them (bench 5/10,
/// A -44 dB: 16 % CRC failures, 70 pictures lost in 40 s, never under 0.75 for two windows),
/// where the step - about 3 dB more robust - carries nearly all of them...
const REP2_IN_PROB: f32 = 0.85;
/// ...out when MCS0's probes are back at this, counted over the last REP2_OUT_WIN windows and at
/// least REP2_OUT_N of them. MCS0 gets 4-8 probe blocks a window there: "three windows at 0.85"
/// was met by luck at -44 dB on the bench (5/10), out and back in every 10-20 s, the pictures
/// lost at each return to MCS0. 38 of 40 at a true 0.8 happens 1 time in 120.
const REP2_OUT_PROB: f32 = 0.95;
const REP2_OUT_N: u64 = 40;
const REP2_OUT_WIN: usize = 8;
/// ...and the current rung's block success below this counts as "losing".
const INTERF_LOSS_PROB: f32 = 0.9;
/// A report this long after the last block the far end got, still with none since: the link is
/// QUIET. Its SNR is then no measure of the air the blocks die in - the receiving app's is the last
/// good frame's (held 300 ms, then falling), the ground board's own only while our packets still
/// come in - so the loss is not taken for interference, and an SNR well under the rung is believed
/// whatever the rung's own numbers say. 14/10 and 13/10, the aircraft 20 dB down: "interference
/// regime ON" at 34.5 dB a second after the fade (MCS5 held 1 s, MCS4 6 s more), and MCS2 held 8 s
/// at a reported 0 dB on its numbers from before. Interference never silences a link this long:
/// WiFi's bursts are milliseconds, and between them the blocks get through.
const QUIET: Duration = Duration::from_millis(600);
/// v40.50: a climb lost again within this many seconds failed (see `climb_at` in the worker)...
const CLIMB_FAIL_S: u64 = 10;
/// ...and a rung held this long clears its back-off.
const CLIMB_OK_S: u64 = 20;
/// v40.53: our encoder's bitrate ceiling when `maxkbps` is 0, bits per pixel and frame: 640x480@30
/// 1.84 Mbit/s, 1280x720@30 5.5. The budget used to fill whatever the air carried: the phone
/// (XCover 5, 640x480@30) at 5-7.6 Mbit/s on MCS4-5 two rooms apart, a picture of up to 31
/// blocks, each a chance to lose it, for no sharpness the eye can tell from 2 Mbit/s at that size.
/// The resolution ladder reads the bitrate from before the cap (`uncapped_bitrate`), so a small
/// tier's cap does not keep it from climbing.
const AUTO_BPP: f32 = 0.2;
/// v40.53: a rung's block success counted as clean by the reliability floor (the same 0.95 the
/// probes call a rung certain at).
const REL_OK: f32 = 0.95;
/// How many recent PHY frames stay cached for ARQ.
const ARQ_CACHE: usize = 256;
/// Max retransmissions per PHY frame (each combine at RX is worth ~+3 dB).
const MAX_RETX: u8 = 3;
/// v40.45c interference regime: more attempts per block (the frame is still alive at the
/// receiver, the air is just being hit), and every retransmission and every parity block
/// gets a SECOND copy this much later - a WiFi burst that ate the first one has moved on.
const MAX_RETX_INTERF: u8 = 6;
/// The lowest rung ARQ is still worth using on (conv path).
///
/// A block is 2016 B whatever the rung, but a conv frame carries 2022 B at MCS5 and only
/// 447 B at MCS0 - so the same block is ONE frame up there and FIVE down here, on a link
/// whose ceiling is ~330 frames/s. Two things follow, and they multiply: a retry costs five
/// frames of airtime instead of one, and a block dies if any ONE of its five fragments does,
/// so the block loss is about five times the frame loss. The retries then take the airtime
/// the new data needed, which loses more blocks, which asks for more retries.
///
/// Measured 19/9 with the rung pinned and SNR 22 dB (so the modulation was never the limit),
/// 45 s windows at the ground:
///   MCS0, ARQ on : 641 frames, 14 lost, 20.5 fps, longest freeze 411 ms (2054 ms earlier)
///   MCS0, ARQ off: 496 frames,  0 lost, 15.0 fps, longest freeze 134 ms
///   auto (MCS5), ARQ on vs off: 140 ms vs 99 ms - no harm up there, so only the low rungs
///     are gated.
/// Fewer frames arrive without ARQ, but they arrive steadily, and steady is what a picture
/// needs. Above this rung a retry is one frame and pays for itself, so ARQ stays on.
const ARQ_MIN_MCS: u64 = 2;
const DUP_GAP: Duration = Duration::from_millis(12);

/// AIRTIME CEILING of the fabric conv path, in bit/s: how much the air can CARRY.
///
/// v30 (three diseases, one formula): (1) the constant CONV_FRAMES_PS=90 was a relic of the 100 f/s
/// ceiling; the air now carries 550+. (2) BLIND TO MCS: every block is nfrag FRAMES on air (MCS0 ~5
/// fragments) while the budget counted BLOCKS, so it offered nfrag times the capacity and cq_drop
/// fired in bulk. (3) BLIND TO PACING: capacity depends on the air_gap slot. air_fps = 1/(713 µs
/// frame + gap); divided by nfrag it gives real blocks/s.
///
/// `simulcast` = whether the base layer is running: v36.2 SUBTRACTS ~150 f/s (30 blk/s incl. parity
/// x 5 MCS0 fragments) plus the main layer's own parity factor (~0.8). Without these two, at MCS0
/// demand was 725 f/s against 550 of capacity -> cq_drop 25 % -> every frame short of a block ->
/// FROZEN although the radio delivered cleanly.
///
/// v40.8: SPLIT OUT OF THE VIDEO BRANCH. This number depends on no measurement, so it is the only
/// backstop keeping a "follow the measurement x1.35" budget from overrunning physics. The DATA
/// source used to have no such ceiling: it sent 24 KB x fps FLAT (~5.9 Mbps at 30 fps) and cq_drop
/// ate the excess. v40.25c: the real conv pacing of the transmitting daemon (`txpace`, 3 ms/frame)
/// makes the link's true ceiling a NUMBER OF CONV FRAMES/s (~330), not bits. Measured at forced
/// MCS0: the old formula (gap 0.4 ms -> 900 frames/s) gave a budget 3x too high, the transmitting
/// board sent exactly 330 frames/s saturated, the receiving board (2.1 ms capture floor) lost ~1.4
/// % of frames = 20 % of blocks (4 frames/block) -> NACK 4.6/s, torn frames, 0-3 pictures/s.
/// Turning off the base layer + parity (~50 % fewer frames) gave 18-26 pictures/s at once.
const TX_PACE_MS: f32 = 3.0;
/// v40.48: what one frame costs the board this app sends through, microseconds - the longer of
/// its transmit pace and its encoder, both read from the daemon's `stats` (`txpace_us`,
/// `txenc_us`). 0 = not heard yet, and then TX_PACE_MS stands in (the daemon's default). The
/// constant used to be the only source, so a board paced faster was never used faster.
pub static BOARD_FRAME_US: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
/// Branch B: the share of the air (per mille, the board's last second) the board's IP port took -
/// the IP bridge beside the video (`ip_air_pm` in its stats). The board sends the IP's packets
/// only when no video block waits, so the video's ceiling (conv_air_ceiling_bps) stays the whole
/// air and the rate control decides nothing on it: the air the IP takes is air the video left,
/// never a sign the link has less for the video. In `stats` (ip_air_pm) beside the budget.
pub static BOARD_IP_AIR_PM: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// Branch B: the board's channel width changed (lib.rs, from its `stats`): a count, then the
/// divisors of 20 MHz before and after, {seq, old << 8, new}.
static BW_EVENT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
/// ... and how long after it the rate control reads nothing as loss (the change's outage: ~0.3 s
/// since 13/10, then the board's queue of the old width's pace)
const BW_HOLD: Duration = Duration::from_secs(3);
/// ... the rung it jumps to has this much SNR over its threshold: held there for BW_HOLD, it must
/// not be one that loses (13/10: 5 -> 20 MHz at 22 dB, MCS5 with 1.5 dB - where the rate control
/// keeps MCS4 at that SNR - and frames lost in the hold)
const BW_JUMP_MARGIN_DB: f32 = 3.0;
/// ... when the board made it: the stats come every 2 s, the board says how long ago (chbw_ms)
static BW_AT: std::sync::Mutex<Option<Instant>> = std::sync::Mutex::new(None);

pub fn note_width_change(old: u64, new: u64, age_ms: Option<u64>) {
    // (an age past one read of `stats` is not this change's: a board from before 13/10 dated its
    // width after the change, a read in between gave the change before)
    let at = age_ms
        .filter(|ms| *ms <= 3000)
        .and_then(|ms| Instant::now().checked_sub(Duration::from_millis(ms)))
        .unwrap_or_else(Instant::now);
    *BW_AT.lock().unwrap() = Some(at);
    let seq = (BW_EVENT.load(Ordering::Relaxed) >> 16) + 1;
    BW_EVENT.store(seq << 16 | (old & 0xFF) << 8 | (new & 0xFF), Ordering::Relaxed);
}

/// v40.48: the board sends small conv blocks in compact form (`txcompact=1` in its stats, read
/// by the GUI's board poll), so a block cut to `conv_seg_one_frame` bytes is ONE frame at its
/// rung. Set per video frame into COMPACT_ON together with the app's own conv path.
pub static BOARD_COMPACT: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static COMPACT_ON: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Application bytes a block carries at conv step `i`, and the frames it takes on air.
fn block_shape(i: usize) -> (f32, f32) {
    if crate::phy64() {
        // branch B: a whole segment in one packet at every rung
        (nyx_link::SEG_PAYLOAD as f32, 1.0)
    } else if COMPACT_ON.load(Ordering::Relaxed) {
        (nyx_proto::conv_seg_one_frame(i) as f32, 1.0)
    } else {
        (FRAME_PAYLOAD_BYTES as f32, conv_frames(i, FRAME_PAYLOAD_BYTES) as f32)
    }
}

/// The time per frame on air the budget is sized from, ms.
fn board_frame_ms(air_gap_ms: f32) -> f32 {
    match BOARD_FRAME_US.load(Ordering::Relaxed) {
        // the board paces start to start, and a frame is 0.713 ms long whatever the pace
        us if us > 0 => (us as f32 / 1000.0).max(0.713),
        _ => (0.713 + air_gap_ms.max(0.4)).max(TX_PACE_MS),
    }
}

/// v40.53: the block loss at the rung in use while the rate control is in the interference
/// regime (q16; 0 otherwise = the XOR alone), for the parity a frame gets, and the
/// parity blocks past the first one a video frame has been getting lately (q8), for the budget.
static FEC_LOSS_Q16: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// Branch B: the block loss at the rung in use, every window whatever the regime (q16) - on the
/// ofdm64 PHY every picture's parity is sized from it (the air has room for it there).
static PHY64_LOSS_Q16: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// The ground's own ceiling on the video, kbps ("!cap N" from a ground that decodes on its ARM, the
/// Pluto's - its decoding falls behind long before the air does, and this end's rate control only
/// sees the air), and when it came: older than FAR_CAP_TTL it is gone.
static FAR_CAP: std::sync::Mutex<Option<(u32, Instant)>> = std::sync::Mutex::new(None);
const FAR_CAP_TTL: Duration = Duration::from_secs(15);

pub fn far_cap_set(kbps: u32) {
    let mut c = FAR_CAP.lock().unwrap();
    if c.is_none_or(|(k, _)| k != kbps) {
        log(&format!("far cap: the ground keeps up with {kbps} kbps of video"));
    }
    *c = Some((kbps, Instant::now()));
}

fn far_cap_kbps() -> Option<u32> {
    FAR_CAP.lock().unwrap().filter(|(_, t)| t.elapsed() < FAR_CAP_TTL).map(|(k, _)| k)
}
static FEC_EXTRA_Q8: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// ... the rung that loss was measured at: a picture going out at another (the rate control just
/// stepped, or the repeat step) gets the XOR alone - after a fade the loss of the rung it fled
/// was handed to the keyframe asked for at MCS0 (sim drop: 40-66 pictures lost against 0-1)
static FEC_RUNG: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(u64::MAX);
/// ... losses come in bursts (WiFi takes neighbouring frames), the independent-loss count is
/// stretched by this
const FEC_BURST: f32 = 1.5;
/// ... and the chance a picture loses more blocks than its parity can give back: a keyframe
/// (everything after it needs it), any other picture (at most two parity blocks, and above MCS0)
const FEC_TARGET_KEY: f32 = 0.005;
const FEC_TARGET: f32 = 0.02;

/// v40.53: an H.264 access unit that starts a picture sequence (SPS or an IDR slice first)
fn h264_is_key(d: &[u8]) -> bool {
    let lim = d.len().min(256);
    let mut i = 0;
    while i + 3 < lim {
        if d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 {
            match d[i + 3] & 0x1F {
                5 | 7 => return true,
                1 => return false,
                _ => i += 3,
            }
        } else {
            i += 1;
        }
    }
    false
}

/// Extra parity blocks per video frame lately (beyond the one XOR the budget always counts).
fn fec_extra() -> f32 {
    FEC_EXTRA_Q8.load(Ordering::Relaxed) as f32 / 256.0
}

fn conv_air_ceiling_bps(
    air_gap_ms: f32,
    simulcast: bool,
    rung: usize,
    payload_bits: f32,
    fps: f32,
    fec: bool,
) -> f32 {
    if crate::phy64() {
        // branch B: blocks back to back at this rung's packet time, 90 % of it; the base layer's
        // blocks (15 fps x 2 at MCS 0) take theirs at MCS 0
        let per = nyx_proto::phy64_air_us(rung.min(6), FRAME_PAYLOAD_BYTES);
        let mut blocks_ps = 1e6 / per * 0.9;
        if simulcast {
            blocks_ps = (blocks_ps - 30.0 * nyx_proto::phy64_air_us(0, FRAME_PAYLOAD_BYTES) / per).max(30.0);
        }
        if fec {
            blocks_ps = (blocks_ps - fps.max(1.0) * (1.0 + fec_extra())).max(blocks_ps * 0.5);
        }
        let _ = (air_gap_ms, payload_bits);
        return blocks_ps * nyx_link::SEG_PAYLOAD as f32 * 8.0 * 0.85;
    }
    let period_ms = board_frame_ms(air_gap_ms);
    // v40.48: with compact blocks a block is one frame and carries a rung-sized segment
    let (seg_bytes, nfrag) = block_shape(rung.min(6));
    let payload_bits = if COMPACT_ON.load(Ordering::Relaxed) { seg_bytes * 8.0 } else { payload_bits };
    // Measured: right at the ceiling (328 frames/s) the receiving board loses ~1.4 % of frames;
    // with 1 frame/block (MCS5, 308 frames/s) that is still a clean 30 fps, but with 4 frames/block
    // (MCS0) one lost frame = one lost block -> 20 % of blocks. Multi-fragment rates must stay
    // further from the ceiling.
    // v40.48: compact blocks are small - a picture is many of them and comes in a thicker burst;
    // at 0.93 the board's queue dropped ~2 % of blocks at MCS0 (measured 20/9), so keep 0.8
    let safety = if nfrag >= 3.0 || COMPACT_ON.load(Ordering::Relaxed) { 0.8 } else { 0.93 };
    let mut air_fps = 1000.0 / period_ms * safety;
    let nfrag0 = conv_frames(0, FRAME_PAYLOAD_BYTES) as f32;
    if simulcast {
        // base layer: 15 fps x (1 block + parity) x frames/block at MCS0
        air_fps = (air_fps - 15.0 * 2.0 * nfrag0).max(30.0);
    }
    let mut blocks_ps = air_fps / nfrag.max(1.0);
    if fec {
        // parity = +1 block per video frame, and the Reed-Solomon rows past it (v40.53)
        blocks_ps = (blocks_ps - fps.max(1.0) * (1.0 + fec_extra())).max(blocks_ps * 0.5);
    }
    blocks_ps * payload_bits * 0.85
}

/// camera pass-through: how far behind the camera the sending may fall before whole frames are
/// dropped (up to the newest key frame). A camera frame waits here while the frames before it
/// are still on air; past this the picture at the ground is late enough to be useless.
const PASS_MAX_LAG: Duration = Duration::from_millis(400);

/// camera bitrate over ONVIF: the camera's bitrate is set to this share of what the link carries. The rest is
/// room for the camera's key frames (several times a P frame, on air in one burst), the parity
/// block of every frame, retransmissions and the rate probes.
const CAM_HEADROOM: f32 = 0.6;
/// ...and never below this (kbit/s): a camera starved further gives nothing worth seeing.
const CAM_MIN_KBPS: u32 = 250;

/// State of the pass-through reader (`pass_next`).
struct PassState {
    /// an access unit was lost: drop until a key frame
    wait_key: bool,
    /// camera frame interval (EMA of RTP time steps), for pacing bursts out
    interval_s: f32,
    last_ts: Option<i64>,
    last_out: Option<Instant>,
    drop_late: u64,
    drop_nokey: u64,
}

impl PassState {
    fn new() -> Self {
        PassState {
            wait_key: true,
            interval_s: 1.0 / 25.0,
            last_ts: None,
            last_out: None,
            drop_late: 0,
            drop_nokey: 0,
        }
    }
}

/// The next camera access unit to send, or None after waiting a little (nothing due).
///
/// The camera's H.264 cannot be thinned: every P frame needs the frames before it. So nothing is
/// dropped one frame at a time. A frame older than `PASS_MAX_LAG` means the link is carrying
/// less than the camera makes; everything up to the newest key frame goes, and if there is none,
/// everything goes and sending resumes at the next key frame. RTSP over TCP delivers in bursts
/// (10/9: sending a burst as it came read as loss at the receiver), so frames leave no closer
/// than half the camera's frame interval unless a backlog is building.
fn pass_next(r: &RtspShared, st: &mut PassState) -> Option<RtspAu> {
    let mut q = r.aus.lock().unwrap();
    if q.is_empty() {
        // woken by a new access unit or by a NACK (net.rs); no polling: on the board the radio
        // daemon needs the CPU (17/9: a 4 ms poll took 19 % of a core and the link got worse)
        q = r.aus_cv.wait_timeout(q, Duration::from_millis(50)).unwrap().0;
    }
    if r.aus_broken.swap(false, Ordering::Relaxed) {
        st.wait_key = true;
    }
    if q.front().is_some_and(|a| a.at.elapsed() > PASS_MAX_LAG) {
        let n0 = q.len();
        match q.iter().rposition(|a| a.key && a.at.elapsed() <= PASS_MAX_LAG) {
            Some(k) => {
                q.drain(..k);
                st.wait_key = false;
            }
            None => {
                q.clear();
                st.wait_key = true;
            }
        }
        let n = (n0 - q.len()) as u64;
        if st.drop_late / 50 != (st.drop_late + n) / 50 || st.drop_late == 0 {
            log(&format!(
                "pass: camera frames {} ms old, dropped {n} to the next key frame (total {})",
                PASS_MAX_LAG.as_millis(), st.drop_late + n
            ));
        }
        st.drop_late += n;
    }
    if st.wait_key {
        while q.front().is_some_and(|a| !a.key) {
            q.pop_front();
            st.drop_nokey += 1;
        }
        if q.is_empty() {
            return None;
        }
        st.wait_key = false;
    }
    if q.is_empty() {
        return None;
    }
    if q.len() < 3
        && let Some(t) = st.last_out
    {
        let gap = Duration::from_secs_f32((st.interval_s * 0.5).clamp(0.004, 0.1));
        let el = t.elapsed();
        if el < gap {
            let _ = r.aus_cv.wait_timeout(q, gap - el).unwrap();
            return None;
        }
    }
    let au = q.pop_front()?;
    drop(q);
    if let Some(prev) = st.last_ts {
        let d = (au.ts90k - prev) as f32 / 90_000.0;
        if d > 0.0 && d < 1.0 {
            st.interval_s = 0.9 * st.interval_s + 0.1 * d;
        }
    }
    st.last_ts = Some(au.ts90k);
    st.last_out = Some(Instant::now());
    Some(au)
}

pub fn spawn(shared: Arc<Shared>, net: Arc<Net>) {
    std::thread::Builder::new()
        .name("tx-worker".into())
        .spawn(move || run(shared, net))
        .expect("spawn tx-worker");
}

fn run(shared: Arc<Shared>, net: Arc<Net>) {
    let mut phy = PcTx::new();
    let mut pattern = PatternGen::new();
    let mut datagen = DataGen::new();
    // v23: the conv transmit path is SPLIT OFF (crates/nyx-tx/src/conv_tx.rs). The capture loop
    // only queues blocks and moves on; nothing is sent INSIDE the loop any more, so a block no
    // longer costs a whole frame period.
    let convtx = crate::conv_tx::spawn(shared.clone(), net.clone());
    // v6.14: rate of DELIVERED blocks (delta segs_ok/s from feedback), the true measure of link
    // capacity; the fabric budget follows this rather than the number of blocks the TX sent itself
    // (flooding inflates the budget -> ARQ storm, measured on air).
    let mut deliv: Option<(u64, Instant)> = None;
    let mut deliv_rate = 0.0f32;
    // ... and when the far end last reported one more delivered (see QUIET)
    let mut segs_seen: Option<u64> = None;
    let mut segs_adv_at = Instant::now();
    // v7: quality ladder: (w, h, fps, minimum kbps to CLIMB to that rung). width/height/fps in the
    // config are a CEILING: the ladder only steps down.
    const LADDER: &[(usize, usize, f32, f32)] = &[
        (320, 240, 10.0, 0.0),
        (320, 240, 15.0, 80.0),
        (480, 360, 20.0, 180.0),
        (640, 480, 30.0, 380.0),
        (854, 480, 30.0, 750.0),
        (1280, 720, 30.0, 1400.0),
        (1280, 720, 60.0, 2600.0),
        // branch B: 1080p30 between the two (the ofdm64 PHY carries it at every rung from 16QAM)
        (1920, 1080, 30.0, 4000.0),
        (1920, 1080, 60.0, 6000.0),
        (2560, 1440, 60.0, 12000.0),
    ];
    /// How long a rung is out of bounds after falling off it, and the ceiling on that wait.
    const TIER_BACKOFF: Duration = Duration::from_secs(20);
    const TIER_BACKOFF_MAX: Duration = Duration::from_secs(160);
    /// A rung that holds this long is considered good again: its back-off resets.
    const TIER_SETTLED: Duration = Duration::from_secs(45);
    /// v40.62 SMOOTHNESS (`smoothres`): SMOOTH_IDR_N lost pictures (keyframe requests and LTR
    /// repairs) within SMOOTH_WIN is a picture breaking up whatever the capacity reads: a rung
    /// down, to SMOOTH_FLOOR_TIER (640x480@30) at the lowest, the rung's back-off as for any
    /// fall; the rung above only with at most SMOOTH_UP_MAX in the window. Bench 11/10, 2.4 GHz
    /// WiFi, Auto on one channel: 720p stalled >= 300 ms 20.8 times a minute (25 lost pictures a
    /// minute) against 2.8 at 640x480 (6.4), on 3.3 Mbps of a link good for 6.6 - the capacity
    /// ladder kept 720p.
    const SMOOTH_WIN: Duration = Duration::from_secs(60);
    const SMOOTH_IDR_N: usize = 10;
    const SMOOTH_UP_MAX: usize = 3;
    const SMOOTH_FLOOR_TIER: usize = 3;
    /// A link that lost pictures within SMOOTH_WIN climbs only from a rung held SMOOTH_UP_HOLD,
    /// its last loss SMOOTH_UP_QUIET ago (a clean one climbs at the usual pace). WiFi comes in
    /// bursts 20-40 s apart: bench 12/10, 854x480 -> 720p30 -> 720p60 -> 1080p in 12 s, each
    /// rung judged for its 6 s dwell, 44 pictures lost at 1080p in the next 20 s; per rung over
    /// 8 min, 640x480 lost 2.2 a minute and 1080p 31.
    const SMOOTH_UP_HOLD: Duration = Duration::from_secs(30);
    const SMOOTH_UP_QUIET: Duration = Duration::from_secs(20);
    let mut lost_pics: std::collections::VecDeque<Instant> = std::collections::VecDeque::new();
    let mut cur_tier = LADDER.len() - 1; // seed high, step down with the channel
    let mut tier_since = Instant::now();
    let mut tier_up_since: Option<Instant> = None;
    let mut tier_down_since: Option<Instant> = None;
    // v40.47: per rung, when it may be tried again after falling off it, and how long that wait
    // was last time (it doubles from TIER_BACKOFF up to TIER_BACKOFF_MAX, and is forgotten once
    // the rung holds for a while).
    let mut tier_retry_at: Vec<Option<Instant>> = vec![None; LADDER.len()];
    let mut tier_backoff: Vec<Duration> = vec![TIER_BACKOFF; LADDER.len()];
    let mut link_cap_kbps = 0.0f32; // capacity measured in the PREVIOUS loop
    // OLLA v32 minstrel-lite: the per-rate statistics
    let mut mn_prob: [f32; 6] = [-1.0; 6]; // <0 = no sample yet
    // v40.34: smoothed per-rate counts (ok, sent) - the probability is their ratio
    let mut mn_acc: [(f32, f32); 6] = [(0.0, 0.0); 6];
    let mut mn_age: [Instant; 6] = [Instant::now(); 6];
    let mut mn_sent_last: [u64; 6] = [0; 6];
    let mut mn_ok_last: [u16; 6] = [0; 6];
    let mut last_step = Instant::now();
    let mut probe_rr: u64 = 0; // xoay vong bac probe
    let mut up_want: Option<usize> = None; // v32.2: climbing needs 2 windows in agreement
    let mut up_confirm = 0u32;
    // v40.50: under interference, windows in a row that found a measured lower rung better
    let mut interf_down = 0u32;
    let mut interf_on = false; // v40.45: interference regime (logged on change)
    // v40.49: the last rung that held (block ok >= 0.9, no interference) with the SNR it had,
    // and the lowest SNR reported since: a deep dip and a return to that SNR is an outage or a
    // fade that is over, and the rung comes back in one step (see "back from")
    let mut stable: Option<(usize, f32, Instant)> = None;
    // v40.50: a rung climbed into and lost again within CLIMB_FAIL_S with the SNR well above what
    // it needs (loss that is not SNR: bursts of interference, a band the receiver is weak in) is tried
    // again only after a back-off that doubles per failure, 6 -> 12 -> 24 -> 30 s; holding it
    // CLIMB_OK_S clears it, so does another channel. Probe blocks are too few to see a 10-15 %
    // loss: on 2500 MHz on the bench (MCS5 CRC-fails 13 % at 30 dB) the climb was tried every
    // ~20 s, and the 15 % of the time spent there lost as many frames as the other 85 %.
    let mut climb_at: Option<(usize, Instant)> = None;
    let mut retry_at: [Option<Instant>; 6] = [None; 6];
    let mut retry_gap_s = [0u64; 6];
    // v40.53: the last "back from an outage" return (rung, when). One lost again within
    // CLIMB_FAIL_S backs that rung off like a failed climb, whatever the SNR then: a marginal link
    // starves the SNR to 0 every second (no frames for 300 ms), the next frame brings the RSSI air
    // SNR back (25-47 dB with WiFi on the channel), and it was a return to MCS4 every 2-5 s, each
    // one dead within a second (two rooms apart, EDUP, 6/10: 18 fps -> 0 -> 18, 355 lost in 40 s)
    let mut restore_at: Option<(usize, Instant)> = None;
    let mut rel_low = 0u32; // v40.53: windows in a row the current rung read under REL_OK
    let mut retry_hz = 0u64;
    let mut snr_dip = f32::MAX;
    // ...and the delivered/needed ceiling (air_scale) the video had then
    let mut stable_air = 1.0f32;
    // ...and the resolution tier; `back from an outage` hands it to the ladder
    let mut stable_tier = LADDER.len() - 1;
    let mut ladder_restore: Option<usize> = None;
    // v40.53: the repeat step (conv rung 6, QPSK 1/2 x2, 222 B a frame) below MCS0, for the edge
    // of coverage. minstrel's table stays 0..5 (the receiver reports no per-rung count for it):
    // the stream goes out at rung 6 while MCS0 is the chosen rung and its own blocks fail, and
    // MCS0 keeps being measured by the probe blocks to decide when to come back.
    let mut rep2_on = false;
    let mut rep2_in = 0u32;
    // MCS0's probe blocks (sent, delivered) of the last windows while on the step
    let mut rep2_probes: VecDeque<(u64, u64)> = VecDeque::new();
    // windows to leave out after going on the step: the receiver's counts still hold the
    // stream's own MCS0 blocks (sim: "21/22 ok" two seconds after going on at 0.64)
    let mut rep2_settle = 0u32;
    let mut surv_cnt = 0u32;   // v40.45: consecutive dead windows while in it
    let mut clean_cnt = 0u32;  // v40.45: consecutive clean windows before leaving it
    let mut interf_win = 0u32; // v40.53: windows in a row in it (the parity waits for 3)...
    let mut interf_win_rung = usize::MAX; // ... at one rung
    // v40.45c: delayed second copies (parity blocks, retransmissions) while under interference
    let mut dup_q: std::collections::VecDeque<(Instant, u64, [u8; nyx_link::BLOCK_BYTES])> =
        std::collections::VecDeque::new();
    let mut dups_sent: u64 = 0;
    let mut sent_last: u64 = 0; // v30.1: CQ_IN last time (for sent_rate)
    let mut base_in_last: u64 = 0; // v37: base stream kept separate
    let mut ok_base_last: u16 = 0;
    let mut mn_dbg = 0u32;
    let (mut mn_dbg_ds, mut mn_dbg_dok) = ([0u64; 6], [0u16; 6]);
    let mut sent_win_t = Instant::now(); // v30.5: sent measurement window
    let mut sent_ps = 0.0f32;
    let mut segs_last: u64 = 0; // v31: segs_ok on the same window as CQ_IN
    let mut r_ema = -1.0f32;    // <0 = no sample yet
    struct CacheEntry {
        seq: u64,
        block: [u8; nyx_link::BLOCK_BYTES],
        mcs: Mcs,
        retx: u8,
    }
    let mut seq: u64 = 0;
    let mut frame_id: u32 = 0;
    let mut lic_info_at = Instant::now() - Duration::from_secs(10); // v40.33
    let mut next_frame = Instant::now();
    let mut cache: VecDeque<CacheEntry> = VecDeque::new();
    // v40.25: start at a MIDDLE rung (MCS3), not MCS0. Measured: after a nyx-tx restart at MCS0
    // every block = 4 conv frames, 30 fps x (block + parity) + base layer = ~82 blocks/s = 328
    // frames/s = EXACTLY the daemon's 3 ms/frame pacing ceiling -> queue full, retx 300-900 ms
    // late, ~20 % of blocks "evaporate", torn frames, a 13-block IDR torn again -> minstrel cannot
    // climb (stuck at MCS0 for minutes). Starting at MCS3 (1-2 frames/block) does not saturate; on
    // a bad channel minstrel steps down on its own (it has statistics).
    let mut auto_mcs = Mcs::ALL[3];
    // branch B: the last width change seen (BW_EVENT), and the end of its hold
    let mut bw_seen = 0u64;
    let mut bw_hold_until = Instant::now();
    let mut snr_hist: std::collections::VecDeque<(Instant, f32)> = std::collections::VecDeque::new();
    let mut bw_prev_at: Option<Instant> = None;
    // v40.31: per-channel memory for minstrel. The daemon reports every hop
    // (Msg::TxChan); on a channel we have seen in the last two minutes the MCS and
    // the per-rate statistics it had are restored at once instead of being
    // rediscovered over the next 1-3 s on every hop.
    let mut last_chan_rung: Option<u8> = None;
    let mut mem_hz: u64 = 0;
    let mut chan_mem: std::collections::HashMap<u64, (usize, [f32; 6], Instant)> =
        std::collections::HashMap::new();

    // v12.1: how long feedback has been blind; cold start / deep fade steps the MCS down gradually.
    let mut blind_since: Option<Instant> = None;
    // Outer-loop link adaptation (LTE-style OLLA): a learned SNR back-off
    // that grows quickly on errors and decays slowly when clean, absorbing
    // the gap between average SNR and what the fading channel really allows.
    let mut olla_margin_db = 0.0f32;
    let mut fps_roll = Rolling::new(2.0);
    let mut iq_roll = Rolling::new(2.0);
    // PHY frames (blocks) actually pushed to the link per second — the real,
    // capture-pipeline-limited throughput that video rate control budgets on.
    let mut block_roll = Rolling::new(2.0);
    // The receiver-side FPGA capture window is ~3 ms of air per trigger and
    // cannot re-fire inside it: ANY frame launched in the previous frame's
    // capture shadow is structurally lost (never captured). Every IqFrame
    // send therefore enforces a minimum air gap — this is what made the
    // 2nd block of every IDR and almost every ARQ retransmission vanish,
    // which in turn self-sustained an IDR->loss->IDR flywheel (measured:
    // 54 of 55 loss-seconds had an IDR within +-2 s). The full shadow is
    // capture (2.9 ms) + refractory (0.53 ms) + re-arm + daemon-side
    // chunking jitter (~1 ms), so 4 ms sat right on the edge — 8 ms clears
    // it with margin and is still invisible at a 41.7 ms frame period.
    const MIN_AIR_GAP: Duration = Duration::from_millis(8);
    /// v6.14: the sustained capture cadence of the RX fabric (~24-35 ms period measured on air);
    /// the fabric path's budget ceiling counts CAPTURES, not video frames (one video frame of
    /// several blocks = SEVERAL captures).
    const FAB_CAPTURES_PS: f32 = 60.0;
    /// v6.14: the minimum gap between SENDS on the fabric path = the RX's processing period for one
    /// capture. Sending faster than that, the extra blocks NEVER get a capture slot -> a permanent
    /// NACK/ARQ storm (measured: MJPEG q92 at gap 8 ms collapsed). v15 (fabric-rx-multiframe step
    /// 1): LDPC on FCLK0 100 MHz -> the demod period measured on the board fell from 24 ms to ~14
    /// ms (sig 0.3 + demap 10.7 + ldpc 2.9); gap lowered 25 -> 16 ms (14 + ~2 ms margin) -> ~60/s
    /// -> ~1 Mbps. v15 clk55 (fabric-rx-demap-clk60): the demod engine (demap+LDPC+AXI) on clk_wiz
    /// 55.556 MHz -> predicted sig 0.5 + demap 2.9 + ldpc 5.2 ≈ 8.7 ms; floor 11 ms (8.7 + ~2 ms
    /// margin). This is a FLOOR: if the board measures slower, raise it at runtime with `set gap
    /// 12..20` (air_gap_ms.max(floor)), no rebuild.
    const FAB_GAP_SINGLE_MS: f32 = 12.0; // v19cx: 77-78 captures/s measured sustained (sample-domain pacing)
    // v20.11 (0x1C): the receiving board consumes in ~13 ms and the strict gate absorbs the excess
    // (clean drop, no corruption) -> floor lowered 15 -> 8; real spacing = PC encode ~7 ms + gap +
    // send, the receiving board protects itself. (Old v20.6: 15 ms, sustained 1.47-1.57 M.)
    const FAB_GAP_PAIR_MS: f32 = 8.0;
    // Samples of one capture window usable for a back-to-back block burst
    // (46080 minus pre-trigger and margin) — see the send loop.
    const BURST_BUDGET: usize = 42_000;
    let mut last_air = Instant::now();
    let mut nacks_handled: u64 = 0;
    // NACKs answered with silence because the rung is too low to retry on (see ARQ_MIN_MCS)
    let mut nacks_low: u64 = 0;
    // v40.47 long-term reference: references the receiver confirmed, repairs coded against
    // one, and how many repairs have been tried since the last confirmation (a repair that
    // does not fix the receiver must give way to a keyframe, or the picture stays broken).
    let (mut ltr_marks, mut ltr_repairs): (u64, u64) = (0, 0);
    let mut ltr_tries: u32 = 0;
    let mut ltr_last = Instant::now() - Duration::from_secs(10);
    /// Repairs to try from one broken state before falling back to a keyframe.
    const LTR_MAX_TRIES: u32 = 2;
    let mut retransmits: u64 = 0;
    // the rung the conv path is sending at right now
    // v40.48: the gate exists because a retry at MCS0/1 cost five/three frames of a fragmented
    // block; a compact block is one frame at any rung, so there is nothing left to gate
    let low_rung = || {
        let r = crate::conv_tx::CUR_MCS.load(Ordering::Relaxed);
        // (branch B: never - a block is one packet at every rung, a retry costs one block of air)
        (r < ARQ_MIN_MCS || r == 6) && !COMPACT_ON.load(Ordering::Relaxed) && !crate::phy64()
    };
    // The conv path's NACKs, answered from the cache AHEAD of the queue (`push_retx`). Between
    // camera frames as well as after one: see the frame sleep below.
    macro_rules! answer_conv_nacks {
        ($cfg:expr, $pass:expr) => {{
            let nack_rx = net.nacks.lock().unwrap();
            let max_retx = if interf_on || $pass { MAX_RETX_INTERF } else { MAX_RETX };
            while let Ok(nseq) = nack_rx.try_recv() {
                nacks_handled += 1;
                if !$cfg.arq || low_rung() {
                    nacks_low += u64::from(low_rung());
                    continue;
                }
                if let Some(entry) = cache
                    .iter_mut()
                    .rev()
                    .find(|e| e.seq & 0xFF == nseq & 0xFF)
                    // v40.53: the newest block with that sequence, and no Reed-Solomon row: lost
                    // as the link fades, its retry goes at the rung the rate control fled to (five
                    // frames a block at MCS0) for a row the data blocks' own retries make useless
                    .filter(|e| e.retx < max_retx && !nyx_link::is_rs_parity(&e.block))
                {
                    entry.retx += 1;
                    convtx.push_retx(entry.seq, &entry.block);
                    retransmits += 1;
                    if $cfg.dup && interf_on {
                        dup_q.push_back((Instant::now() + DUP_GAP, entry.seq, entry.block));
                    }
                }
            }
        }};
    }
    // v20.11 rate diagnostic: time slept in the gap vs net.send per status window
    let mut tp_gap: u64 = 0;
    let mut tp_send: u64 = 0;
    let mut tp_enc: u64 = 0;
    let mut tp_msgs: u64 = 0;
    let mut encoder: Option<VideoEncoder> = None;
    // v36 simulcast
    let mut enc_base: Option<VideoEncoder> = None;
    // v40.62: the size the base encoder was made for (made again when the main layer's changes)
    let mut enc_base_dims = (0usize, 0usize);
    let mut base_flip = false;
    let mut base_id: u32 = 0;

    let mut idr_sent: u64 = 0;
    // Time of the most recent IDR (Option: do NOT use Instant::now() - Duration, a Windows Instant
    // counts from boot; lesson learned).
    let mut last_idr: Option<Instant> = None;
    let mut blind_idrs: u64 = 0; // v40.53: keyframes sent because the feedback was stale
    // v40.24 SELF-RECOVERY AFTER LINK LOSS: an IDR that does not resolve the request (the RX keeps
    // asking) must not be served every 700 ms for ever: at low MCS each IDR spans dozens of PHY
    // frames, 700 ms apart is ~85 % of the airtime in IDRs, P-frames starve, holes keep coming ->
    // another IDR (measured: disp 0 fps, goodput 30 kbps although the PHY was clean at 0.6 %
    // errors; the app had to be restarted). Back off 700 -> 1400 -> 2800 ms with the run of
    // consecutive IDRs; from the third one halve the bitrate each step (floor 0.25, 60 kbps) so the
    // IDR gets short enough to get through. Request cleared (the RX decoded a picture) and > 1.5 s
    // quiet -> back to normal.
    let mut idr_streak: u32 = 0;
    let mut idr_backoff_ms: u64 = 700;
    let mut survival: f32 = 1.0;
    // segs_ok (frames the RX delivered) at the previous IDR: an advance of >= 5 means that IDR DID
    // resolve things (the RX has pictures again) and the new request comes from a stray block loss
    // (~every 5 s at 3 NACK/s), NOT a loop. The old "asked again within 2x backoff" test misfired
    // even at 29 fps video (streak 5, survival 0.25 for no reason).
    let mut idr_segs_at: u64 = 0;
    let mut video_bitrate: u32 = 0;
    // v40.53: the encoder's bitrate before the picture cap - what the ladder reads as capacity
    let mut uncapped_bitrate: u32 = 0;
    // v40.53: air the picture cap leaves unused, as a fraction of the video bitrate: the parity
    // a picture that is not a keyframe gets past the XOR has to fit in it
    let mut fec_room: f32 = 0.0;
    // v-lat CONTROL BY DELIVERED/SENT RATIO (the missing piece: the encoder throttling itself to
    // the link). Signal: blocks SENT vs blocks the RX REPORTS DECODED (segs_ok in feedback) over a
    // 1 s window.
    //   deliv/sent < 0.95  = producing MORE than the link can carry -> lower the ceiling 10 %
    //   deliv/sent > 0.98  = comfortable -> raise 5 % (slowly, against oscillation)
    // NO frame dropping (last time dropping frames + lowering the bitrate was a death spiral: drop
    // -> lower -> still dropping -> hit the 197 k floor, video dead). Only the BITRATE is
    // regulated, so the loop closes by itself: lower bitrate -> smaller frames -> fewer blocks sent
    // -> the ratio recovers.
    let mut sent_blocks: u64 = 0;
    let mut rc_win = Instant::now();
    let mut rc_sent0: u64 = 0;
    let mut rc_ok0: Option<u64> = None;
    let mut air_scale: f32 = 1.0;
    let mut tp_win = Instant::now();
    // v27: the raw r oscillated around 1 through PHASE MISMATCH: sent is counted at send time,
    // deliv arrives with the feedback 150-300 ms later, so a 1 s window sometimes piles up
    // (measured 1.04-1.10) and sometimes falls short (< 0.95); the x0.90 steps overpower the x1.05
    // ones and the noise GROUND air_scale down to ~0.26 at bler = 0 (measured: vbr 430 k =
    // one_burst x 0.26, the resolution ladder stuck at 640x480). The same mechanism punished the
    // budget's +35 % climb (sent rises one feedback tick before deliv -> r dips falsely -> the
    // climb collapses). Fix: compare deliv with sent IN PHASE (blend 30 % of the previous window ~
    // the feedback delay) + EMA, and decide on the average. A REAL loss (r low for long) still
    // pulls the EMA down within ~2 s.
    let mut rc_sent_prev: f32 = 0.0;
    // v40.35: ARQ-corrected delivery - retransmissions / queue drops per window
    let mut rc_retx0: u64 = 0;
    let mut rc_drop0: u64 = 0;
    let mut rc_log_at = Instant::now() - Duration::from_secs(10);
    let mut r_ema: f32 = 1.0;
    let mut dac_clips: u64 = 0;
    // v5: MJPEG adaptive quality, following the measured goodput budget (as H264 does); the old
    // fixed quality let JPEGs swell 3-10x on moving pictures -> bursts -> structural frame loss on
    // the fabric path.
    let mut mj_quality: f32 = 60.0;
    let mut status = StatusLogger::new(1.0);
    // Breadcrumbs for the very first frame — pinpoints startup hangs.
    let mut first_frame = true;
    let mut said_acquiring = false;
    // v40.44z: how long the camera has given nothing; past 3 s with an open error the
    // source drops to the pattern so a box without a camera still sends a picture.
    let mut webcam_wait = 0u32;
    // camera pass-through: the camera's access units sent as they are
    let mut pst = PassState::new();
    let mut pass_fps_roll = Rolling::new(2.0);
    let mut pass_byte_roll = Rolling::new(2.0);
    let mut pass_idr_req: u64 = 0;
    let mut pass_was = false;
    // camera bitrate over ONVIF: the bitrate the camera is asked for, and when it last looked worth raising
    let mut cam_want: u32 = 0;
    let mut cam_tick = Instant::now();
    let mut cam_up_since: Option<Instant> = None;
    let mut cam_late0: u64 = 0;

    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        let cfg: TxConfig = shared.config.lock().unwrap().clone();
        // on-board camera app: the board is the receiving end - no camera, nothing to send
        if shared.standby.load(Ordering::Relaxed) {
            shared.webcam.wanted.store(false, Ordering::Relaxed);
            shared.rtsp.wanted.store(false, Ordering::Relaxed);
            shared.vfile.wanted.store(false, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(200));
            next_frame = Instant::now();
            continue;
        }
        shared
            .webcam
            .wanted
            .store(cfg.source == SourceKind::Webcam, Ordering::Relaxed);
        shared.rtsp.wanted.store(cfg.source == SourceKind::Rtsp, Ordering::Relaxed);
        shared.vfile.wanted.store(cfg.source == SourceKind::File, Ordering::Relaxed);
        if cfg.source == SourceKind::File {
            let mut p = shared.vfile.path.lock().unwrap();
            if *p != cfg.video_file {
                *p = cfg.video_file.clone();
            }
        }
        let pass = cfg.source == SourceKind::Rtsp && cfg.rtsp_pass;
        shared.rtsp.pass.store(pass, Ordering::Relaxed);
        if pass != pass_was {
            pass_was = pass;
            shared.rtsp.clear_aus();
            pst = PassState::new();
            log(if pass {
                "pass: sending the camera's H.264 as it is (no re-encode)"
            } else {
                "pass: off"
            });
        }
        if cfg.source == SourceKind::Rtsp {
            let mut u = shared.rtsp.url.lock().unwrap();
            if *u != cfg.rtsp_url {
                *u = cfg.rtsp_url.clone();
            }
            drop(u);
            let mut o = shared.rtsp.onvif_url.lock().unwrap();
            if *o != cfg.onvif_url {
                *o = cfg.onvif_url.clone();
            }
        }
        shared.rtsp.onvif_adapt.store(cfg.cam_adapt, Ordering::Relaxed);

        if cfg.paused || !shared.connected.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(60));
            next_frame = Instant::now();
            continue;
        }

        // ---- v7: adaptive quality ladder (resolution + fps + bitrate) ----
        // Ceiling = the user's configuration; only rungs <= the ceiling are chosen. Climbing needs
        // 15 % headroom + a 2.5 s dwell (H.264 pays a keyframe for a resolution change); step down
        // fast when the channel collapses.
        let max_tier = LADDER
            .iter()
            .rposition(|&(w, h, f, _)| {
                w <= cfg.width && h <= cfg.height && f <= cfg.fps + 0.5
            })
            .unwrap_or(0);
        if cfg.auto_res && cfg.txbits && link_cap_kbps > 1.0 {
            cur_tier = cur_tier.min(max_tier);
            // v40.49: back from an outage (the rate control saw the SNR come back to where it
            // was): the tier that held then, at once. The ladder's persistence, dwell and
            // back-off are there against capacity hovering at a boundary; after a fade that is
            // over they kept the picture at 480x360@20 for 20 s and more (simulator).
            if let Some(t) = ladder_restore.take() {
                let t = t.min(max_tier);
                if t > cur_tier {
                    log(&format!(
                        "ladder: back from an outage -> {}x{}@{} as before",
                        LADDER[t].0, LADDER[t].1, LADDER[t].2
                    ));
                    for i in (cur_tier + 1)..=t {
                        tier_retry_at[i] = None;
                        tier_backoff[i] = TIER_BACKOFF;
                    }
                    cur_tier = t;
                    tier_since = Instant::now();
                    tier_up_since = None;
                    tier_down_since = None;
                }
            }
            // WIDE hysteresis: entering the rung above needs 30 % headroom (otherwise capacity
            // sitting right at the boundary flaps the resolution -> ugly H264 keyframes); leave on
            // a 15 % drop. Borderline capacity settles on the LOWER rung, which is more honest
            // (854p at 800 k looks as bad as 640p and cuts out). v14 SMOOTH: a stickier ladder,
            // since every resolution change = one big, easily lost H264 keyframe -> a stutter. Up
            // needs 45 % headroom + 6 s dwell; down on a 20 % drop; only a STEEP drop (crash)
            // changes at once. Resolution changes become rare -> fewer stutters (the jitter buffer
            // hides the rest). v40.35: the condition must PERSIST (up 3 s, down 2 s): a single
            // frame's capacity estimate flipped 320x240 <-> 480x360 every few seconds, each flip an
            // encoder restart and a new IDR.
            // the rung above is out of bounds while its back-off runs
            let up_free = tier_retry_at
                .get(cur_tier + 1)
                .and_then(|o| *o)
                .is_none_or(|t| Instant::now() >= t);
            // v40.62: pictures lost lately (see SMOOTH_IDR_N)
            while lost_pics.front().is_some_and(|t| t.elapsed() > SMOOTH_WIN) {
                lost_pics.pop_front();
            }
            let recent_loss = cfg.smooth_res && lost_pics.len() > SMOOTH_UP_MAX;
            // a rung is judged on its own pictures: the ones lost on the rung above made the step
            // down, and counted again they made the next ones too - bench 12/10, one burst of
            // 10-11 lost at 1080p took the ladder 1080p -> 720p60 -> 720p30 -> 854x480 -> 640x480,
            // a rung every 6 s, on a link carrying 3x what 720p needs. (The climb still waits for
            // the whole window to be quiet: recent_loss.)
            let lost_here = lost_pics.iter().filter(|t| **t >= tier_since).count();
            let rough = cfg.smooth_res && lost_here >= SMOOTH_IDR_N && cur_tier > SMOOTH_FLOOR_TIER;
            let paced = !cfg.smooth_res
                || lost_pics.back().is_none_or(|t| {
                    t.elapsed() >= SMOOTH_UP_QUIET && tier_since.elapsed() >= SMOOTH_UP_HOLD
                });
            let up_now = cur_tier < max_tier
                && up_free
                && !recent_loss
                && paced
                && link_cap_kbps >= LADDER[cur_tier + 1].3 * 1.45;
            let need = LADDER[cur_tier].3;
            let down_now = cur_tier > 0 && link_cap_kbps < need * 0.80;
            if !up_now {
                tier_up_since = None;
            } else if tier_up_since.is_none() {
                tier_up_since = Some(Instant::now());
            }
            if !down_now {
                tier_down_since = None;
            } else if tier_down_since.is_none() {
                tier_down_since = Some(Instant::now());
            }
            let up_ok = tier_up_since.is_some_and(|t| t.elapsed() >= Duration::from_secs(3));
            let down = tier_down_since.is_some_and(|t| t.elapsed() >= Duration::from_secs(2));
            // v40.47: even a crash has to LAST. This branch skips both the 2 s persistence and
            // the 6 s dwell, so ONE low capacity estimate dropped the rung at once - and every
            // rung change is an encoder restart, a fresh IDR and a burst of broken frames.
            // Measured 19/9 with the rung pinned at MCS0 (where the estimate swings hardest):
            // 480x360 -> 320x240 -> 480x360 -> 640x480 inside 40 s, and the picture stalled ~400
            // ms at each flip while everything else was clean (p50/p95 gaps 67/84 ms). Half a
            // second of persistence is nothing against a real collapse and kills the flapping.
            let crash = cur_tier > 0
                && link_cap_kbps < need * 0.50
                && tier_down_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500));
            let dwell = tier_since.elapsed() >= Duration::from_millis(6000);
            if crash || (down && dwell) || (rough && dwell) {
                if rough && !crash && !down {
                    log(&format!(
                        "ladder: {} pictures lost in {} s on this rung -> {}x{}@{} (the capacity reads {link_cap_kbps:.0} kbps)",
                        lost_here,
                        tier_since.elapsed().as_secs().min(SMOOTH_WIN.as_secs()),
                        LADDER[cur_tier - 1].0,
                        LADDER[cur_tier - 1].1,
                        LADDER[cur_tier - 1].2
                    ));
                }
                // the rung we are leaving did not hold: keep out of it, longer each time
                let held = tier_since.elapsed();
                let b = &mut tier_backoff[cur_tier];
                if held >= TIER_SETTLED {
                    *b = TIER_BACKOFF; // it was fine for a long time - this was a real change
                } else {
                    *b = (*b * 2).min(TIER_BACKOFF_MAX);
                }
                tier_retry_at[cur_tier] = Some(Instant::now() + *b);
                cur_tier -= 1;
                tier_since = Instant::now();
            } else if up_ok && dwell {
                cur_tier += 1;
                tier_since = Instant::now();
            } else if tier_since.elapsed() >= TIER_SETTLED {
                // this rung is holding: let the one above become reachable again at the normal
                // wait rather than the doubled one
                if let Some(b) = tier_backoff.get_mut(cur_tier + 1) {
                    *b = TIER_BACKOFF;
                }
            }
        } else {
            cur_tier = max_tier; // auto off: use the configuration as it is
        }
        let (aw, ah, afps) = {
            let (w, h, f, _) = LADDER[cur_tier];
            (w, h, f)
        };
        // The `data` source is a LOAD TEST and is not subject to the video ladder. The ladder
        // climbs on link_cap, and link_cap is ONLY updated from the H264 vbr, so the data source
        // stayed on the lowest rung for ever (measured: 480x360@20 -> 14 blk x 20 = a flat 280/s,
        // while the webcam had just pushed the same path to 425/s on rung 30). Video is unchanged.
        let afps = if cfg.source == SourceKind::RandomData {
            cfg.fps
        } else {
            afps
        };

        if pass {
            next_frame = Instant::now();
        } else {
            // A NACK wakes this sleep (net.rs notifies the condvar the pass-through waits on):
            // answered here, a retransmission no longer waits for the next camera frame AND its
            // encode - up to 33 ms + the encode at 30 fps, of the 300 ms the receiver gives a hole
            // (retx round trip p50 127 ms, p90 231 ms measured at MCS0, 4/10).
            loop {
                if cfg.txconv {
                    answer_conv_nacks!(cfg, pass);
                }
                let now = Instant::now();
                if now >= next_frame {
                    break;
                }
                let g = shared.rtsp.aus.lock().unwrap();
                let _ = shared.rtsp.aus_cv.wait_timeout(g, next_frame - now).unwrap();
            }
            let period = Duration::from_secs_f32(1.0 / afps.clamp(1.0, 60.0));
            next_frame = Instant::now().max(next_frame + period);
        }

        // ------------------------------------------------- MCS adaptation
        let fb = shared.feedback.lock().unwrap().clone();
        let fb_fresh = fb.updated.is_some_and(|t| t.elapsed().as_secs_f32() < 3.0);
        // the SNR reports of the last seconds, for a width change (the SNR at the old width)
        if let Some(t) = fb.updated.filter(|t| snr_hist.back().is_none_or(|(b, _)| t > b)) {
            snr_hist.push_back((t, fb.snr_db));
            while snr_hist.front().is_some_and(|(b, _)| b.elapsed() > Duration::from_secs(10)) {
                snr_hist.pop_front();
            }
        }
        // branch B: the board's channel width moved (its autobw, or the user's `chbw`), and the
        // SNR with it, 3 dB a step (up when narrower). The rung goes straight to what the new SNR
        // carries, what was learnt at the old width goes, and the change's outage is nobody's loss
        // for BW_HOLD from the change: no step down, no back-off, no interference regime, no
        // parity sized from it, no cut of the video's ceiling, no picture counted lost by the
        // resolution ladder. The SNR it starts from is the old width's: the reports from before
        // the change (read up to 2 s after it, the reports since are the new width's already).
        // Until 13/10 the step had its sign the wrong way round: 20 -> 5 MHz at 30 dB read
        // "24 dB" (34 measured), 5 -> 20 MHz at 34 dB "40 dB" (30) - a rung 12 dB too high when
        // the width widened, held there for BW_HOLD.
        let bw_ev = BW_EVENT.load(Ordering::Relaxed);
        if bw_ev != bw_seen {
            bw_seen = bw_ev;
            let (old, new) = ((bw_ev >> 8) & 0xFF, bw_ev & 0xFF);
            if old > 0 && new > 0 {
                let d_db = 10.0 * (new as f32 / old as f32).log10();
                let at = BW_AT.lock().unwrap().unwrap_or_else(Instant::now);
                bw_hold_until = at + BW_HOLD;
                lost_pics.retain(|t| *t < at);
                // (the old width's last reports - since the change before this one had settled. The
                // last few, not the window's: the automatic width narrows BECAUSE the SNR just fell -
                // 14/10, 40 dB less power: the median of 10 s read 24 dB at 20 MHz with 7 left, MCS5
                // held at 10 MHz for BW_HOLD, the video at 4 fps)
                let since = bw_prev_at.map(|p: Instant| p + Duration::from_secs(1));
                bw_prev_at = Some(at);
                let mut before: Vec<f32> = snr_hist
                    .iter()
                    .filter(|(t, _)| *t < at && since.is_none_or(|p| *t > p))
                    .map(|(_, s)| *s)
                    .collect();
                before.drain(..before.len().saturating_sub(3));
                before.sort_unstable_by(f32::total_cmp);
                // ... the lowest of them when narrower: a fade narrows the width ~1.2 s after it
                // (autobw), and the middle one of three could still be from before (14/10: 5 MHz
                // entered at MCS5 on "30.8 dB", reports 24.8, 24.8, 4.5 - 10 dB there). A rung too
                // low costs a climb; one too high, the picture until the new width's own reports.
                let pick = if new > old { 0 } else { before.len() / 2 };
                let snr = match before.get(pick) {
                    Some(s) => Some(s + d_db),
                    None if fb_fresh && fb.updated.is_some_and(|t| t >= at) => Some(fb.snr_db),
                    None => stable.map(|s| s.1 + d_db),
                };
                mn_prob = [-1.0; 6];
                mn_acc = [(0.0, 0.0); 6];
                retry_at = [None; 6];
                retry_gap_s = [0; 6];
                climb_at = None;
                restore_at = None;
                interf_on = false;
                crate::conv_tx::SPREAD_ON.store(false, Ordering::Relaxed);
                interf_down = 0;
                interf_win = 0;
                clean_cnt = 0;
                surv_cnt = 0;
                rel_low = 0;
                snr_dip = f32::MAX;
                chan_mem.clear();
                FEC_LOSS_Q16.store(0, Ordering::Relaxed);
                PHY64_LOSS_Q16.store(0, Ordering::Relaxed);
                stable = None;
                match (cfg.auto_mcs, snr) {
                    (true, Some(snr)) => {
                        let target = (0..6).rev().find(|&r| snr_thresh(r) + BW_JUMP_MARGIN_DB <= snr).unwrap_or(0);
                        log(&format!(
                            "olla: channel {:.2} -> {:.2} MHz, SNR now ~{snr:.1} dB -> {} (was {})",
                            20.0 / old as f32, 20.0 / new as f32, Mcs::ALL[target].label(), auto_mcs.label()
                        ));
                        auto_mcs = Mcs::ALL[target];
                        last_step = Instant::now();
                        up_want = None;
                        up_confirm = 0;
                        if target > 0 && rep2_on {
                            rep2_on = false;
                            rep2_probes.clear();
                        }
                        mn_prob[target] = 1.0;
                        mn_acc[target] = (2.0, 2.0);
                        mn_age[target] = Instant::now();
                    }
                    _ => log(&format!("olla: channel {:.2} -> {:.2} MHz", 20.0 / old as f32, 20.0 / new as f32)),
                }
            }
        }
        let bw_hold = Instant::now() < bw_hold_until;
        // ... and once the far end reports from the new width (a second on: its SNR is a median of
        // packets), a rung its SNR does not carry steps down to one it does - the hold is for the
        // outage's loss, not for a guess that turned out wrong
        if bw_hold
            && cfg.auto_mcs
            && fb_fresh
            && fb.updated.is_some_and(|t| bw_prev_at.is_some_and(|a| t > a + Duration::from_secs(1)))
        {
            let cur = auto_mcs.index();
            if cur > 0 && fb.snr_db < snr_thresh(cur) + 1.5 {
                let target = (0..cur).rev().find(|&r| snr_thresh(r) + BW_JUMP_MARGIN_DB <= fb.snr_db).unwrap_or(0);
                log(&format!(
                    "olla: the new width's SNR is {:.1} dB - {} -> {}",
                    fb.snr_db, Mcs::ALL[cur].label(), Mcs::ALL[target].label()
                ));
                auto_mcs = Mcs::ALL[target];
                last_step = Instant::now();
                mn_prob[target] = 1.0;
                mn_acc[target] = (2.0, 2.0);
                mn_age[target] = Instant::now();
            }
        }
        if bw_hold {
            // the windows of the outage are not counted: the counters move on without them
            for i in 0..6 {
                mn_sent_last[i] = crate::conv_tx::SENT_MCS[i].load(Ordering::Relaxed);
                if fb.ok_mcs != [0xFFFF; 6] {
                    mn_ok_last[i] = fb.ok_mcs[i];
                }
            }
            base_in_last = crate::conv_tx::BASE_IN.load(Ordering::Relaxed);
            if fb.ok_base != 0xFFFF {
                ok_base_last = fb.ok_base;
            }
            sent_win_t = Instant::now();
        }
        let hz_now = shared.chan_hz.load(std::sync::atomic::Ordering::Relaxed);
        if cfg.chan_mem && cfg.auto_mcs && hz_now != 0 && hz_now != mem_hz {
            if mem_hz != 0 {
                chan_mem.insert(mem_hz, (auto_mcs.index(), mn_prob, Instant::now()));
            }
            if let Some(&(m, probs, at)) = chan_mem.get(&hz_now) {
                if at.elapsed() < Duration::from_secs(120) {
                    if m != auto_mcs.index() {
                        log(&format!(
                            "olla: channel {:.0} MHz -> restore {} (was {})",
                            hz_now as f64 / 1e6, Mcs::ALL[m].label(), auto_mcs.label()
                        ));
                        auto_mcs = Mcs::ALL[m];
                        last_step = Instant::now();
                    }
                    mn_prob = probs;
                    for a in mn_age.iter_mut() {
                        *a = Instant::now();
                    }
                    up_want = None;
                    up_confirm = 0;
                }
            }
            mem_hz = hz_now;
        }
        if let Some(tu) = fb.updated {
            match deliv {
                Some((p_ok, p_t)) if tu > p_t => {
                    let dt = tu.duration_since(p_t).as_secs_f32();
                    if fb.segs_ok < p_ok {
                        deliv_rate = 0.0; // nyx-rx restarted: count again
                        deliv = Some((fb.segs_ok, tu));
                    } else if dt > 0.05 {
                        let d = (fb.segs_ok - p_ok) as f32;
                        deliv_rate = 0.7 * deliv_rate + 0.3 * (d / dt);
                        deliv = Some((fb.segs_ok, tu));
                    }
                }
                None => deliv = Some((fb.segs_ok, tu)),
                _ => {}
            }
        }
        // (see QUIET; measured between reports, so a lost report is not a quiet link)
        let mut quiet_ms = 0u64;
        if let Some(tu) = fb.updated {
            if segs_seen != Some(fb.segs_ok) {
                segs_seen = Some(fb.segs_ok);
                segs_adv_at = tu;
            }
            quiet_ms = tu.saturating_duration_since(segs_adv_at).as_millis() as u64;
        }
        let quiet = quiet_ms >= QUIET.as_millis() as u64;
        // PL mode (txpl/txbits): the frame must fit the fabric modulator: ceiling cfg.pl_syms (16
        // with the old bitstream -> floor MCS3; 37 with the v7 streaming modulator -> floor MCS0,
        // the PL covers everything). MCS below the floor = the PC IQ path, for test measurements
        // (chosen by hand, txpl/txbits off).
        let pl = cfg.pl_syms.clamp(1, 37);
        let mcs_floor = if cfg.txpl || cfg.txbits {
            Mcs::ALL
                .iter()
                .position(|m| m.data_syms_per_frame() <= pl)
                .unwrap_or(3)
        } else {
            0
        };
        if cfg.auto_mcs {
            if fb_fresh {
                blind_since = None;
                // ========== OLLA v32 "minstrel-lite" ======================
                // After mac80211/minstrel: SUCCESS PROBABILITY PER RATE + PROBE blocks interleaved
                // at the neighbouring rung + rate = argmax(prob x payload). Unlike v31 (one signal
                // r for the current rate) it knows whether the rung above works BEFORE moving
                // there: no more "climb, try, die", no thresholds or hysteresis to tune.
                //
                // prob_i = d(ok_mcs[i]) / d(SENT_MCS[i]) on the SAME 1 s window (the phase-mismatch
                // lesson was paid for twice), EMA 0.6/0.4, aged 10 s.
                if !bw_hold
                    && sent_win_t.elapsed().as_secs_f32() >= 1.0
                    && fb.ok_mcs != [0xFFFF; 6]
                {
                    // v37 separate streams: rung 0 minus the BASE layer's share (pinned at MCS0,
                    // always delivered), otherwise prob[0] is falsely high.
                    let bi_now = crate::conv_tx::BASE_IN
                        .load(std::sync::atomic::Ordering::Relaxed);
                    let dbase_sent = bi_now.saturating_sub(base_in_last);
                    base_in_last = bi_now;
                    let dbase_ok = if fb.ok_base != 0xFFFF {
                        let d = fb.ok_base.wrapping_sub(ok_base_last);
                        ok_base_last = fb.ok_base;
                        d
                    } else {
                        0
                    };
                    for i in 0..6 {
                        let sent_now = crate::conv_tx::SENT_MCS[i]
                            .load(std::sync::atomic::Ordering::Relaxed);
                        let mut ds = sent_now.wrapping_sub(mn_sent_last[i]);
                        let mut dok =
                            fb.ok_mcs[i].wrapping_sub(mn_ok_last[i]);
                        mn_sent_last[i] = sent_now;
                        mn_ok_last[i] = fb.ok_mcs[i];
                        if i == 0 {
                            ds = ds.saturating_sub(dbase_sent);
                            dok = dok.saturating_sub(dbase_ok);
                        }
                        mn_dbg_ds[i] = ds;
                        mn_dbg_dok[i] = dok;
                        // v40.34: smooth the COUNTS, then divide. Clamping each window's
                        // ratio at 1.0 before averaging biased a 97 % link down to ~0.87
                        // (feedback counters run ahead in one window and lag in the next).
                        if ds > 0 || dok > 0 {
                            let (mut ok_acc, mut sent_acc) = mn_acc[i];
                            // v40.49: the history weighs at most three windows like this one. A
                            // rung that carried the stream (40 blocks/s) and then only its probes
                            // (2/s) kept its fade-time counts for ~10 s: the simulator showed MCS2
                            // at 0.24..0.48 at 24 dB and the climb back took 12 s.
                            let cap = (3.0 * (ds as f32).max(dok as f32)).max(6.0);
                            if sent_acc > cap {
                                ok_acc *= cap / sent_acc;
                                sent_acc = cap;
                            }
                            mn_acc[i] = (0.6 * ok_acc + 0.4 * dok as f32, 0.6 * sent_acc + 0.4 * ds as f32);
                        }
                        if mn_acc[i].1 >= 2.0 && ds >= 1 {
                            mn_prob[i] = (mn_acc[i].0 / mn_acc[i].1).min(1.0);
                            mn_age[i] = Instant::now();
                        }
                    }
                    if std::env::var_os("NYX_MN_DBG").is_some() {
                        let c = auto_mcs.index();
                        log(&format!(
                            "mndbg: cur={c} ds={} dok={} acc={:.1}/{:.1} prob={:.2} ok_age={} fb_age={}",
                            mn_dbg_ds[c], mn_dbg_dok[c], mn_acc[c].0, mn_acc[c].1, mn_prob[c],
                            fb.ok_changed.map_or(-1, |t| t.elapsed().as_millis() as i64),
                            fb.updated.map_or(-1, |t| t.elapsed().as_millis() as i64),
                        ));
                    }
                    {
                        // v40.53: the loss the parity is sized for - only loss that is not the
                        // SNR's (the interference regime: frames hit at an SNR the rung is fine
                        // at). A rung dying in a fade is the rate control's to leave, and parity
                        // there only burns the air it is short of (sim fade: 22 % more frames on
                        // air, 121 pictures lost against 66 with the XOR alone). Three windows in
                        // a row: a sudden drop puts the regime on for a window (the SNR reported
                        // is still the old one) and its 44-96 % "loss" bought a keyframe 4 parity
                        // blocks just as the link went (sim drop: 62-66 pictures lost against 0).
                        // At most 30 %.
                        let c = auto_mcs.index();
                        interf_win = if interf_on && c == interf_win_rung { interf_win + 1 } else { u32::from(interf_on) };
                        interf_win_rung = c;
                        FEC_RUNG.store(c as u64, Ordering::Relaxed);
                        let loss = if interf_win >= 3 && mn_prob[c] >= 0.0 { (1.0 - mn_prob[c]).clamp(0.0, 0.3) } else { 0.0 };
                        FEC_LOSS_Q16.store((loss * 65535.0) as u32, Ordering::Relaxed);
                        if mn_prob[c] >= 0.0 {
                            PHY64_LOSS_Q16.store(((1.0 - mn_prob[c]).clamp(0.0, 0.3) * 65535.0) as u32, Ordering::Relaxed);
                        }
                    }
                    mn_dbg = mn_dbg.wrapping_add(1);
                    if mn_dbg % 5 == 0 {
                        log(&format!(
                            "mn: cur={} prob=[{:.2},{:.2},{:.2},{:.2},{:.2},{:.2}] base_s/ok={dbase_sent}/{dbase_ok} snr={:.1} interf={}",
                            auto_mcs.index(),
                            mn_prob[0], mn_prob[1], mn_prob[2],
                            mn_prob[3], mn_prob[4], mn_prob[5],
                            fb.snr_db, u8::from(interf_on),
                        ));
                    }
                    let cur = auto_mcs.index();
                    let fresh =
                        |i: usize| mn_age[i].elapsed().as_secs_f32() < 10.0;
                    // v40.45: interference regime - SNR says this rung is fine, the
                    // counters say blocks die anyway. Hold the rung: no prior-driven
                    // step down, survival only after several dead windows in a row.
                    let snr_margin = fb.snr_db - snr_thresh(cur);
                    // Hysteresis: WiFi is bursty second to second (block ok swung 0.7..1.0
                    // every window on 2437 MHz, 42 on/off flips in 3 min without it). Enter on
                    // one losing window with the margin; leave only after 3 clean windows in a
                    // row, or when the margin is gone (then it IS an SNR problem).
                    // (nor while the link is QUIET: that SNR is no measure of the air)
                    let enter = mn_prob[cur] >= 0.0
                        && mn_prob[cur] < INTERF_LOSS_PROB
                        && snr_margin >= INTERF_MARGIN_DB
                        && !quiet;
                    let clean = mn_prob[cur] >= 0.97;
                    // (gone when every report of the last 0.5 s says so: a WiFi burst that keeps
                    // every packet from decoding for 250 ms is reported at its packets' own SNR -
                    // the ground board's, see nyx-radio-node FB_SNR_STALE_MS - one report under
                    // the others: 2437 MHz, 14/10, 13-17 dB at 25, the regime left, MCS5 -> 4. A
                    // fade's fall is in all of them, and QUIET is sooner still.)
                    let snr_hold = snr_hist
                        .iter()
                        .rev()
                        .take_while(|(t, _)| t.elapsed() < Duration::from_millis(500))
                        .fold(fb.snr_db, |m, (_, s)| m.max(*s));
                    let margin_gone = snr_hold - snr_thresh(cur) < INTERF_MARGIN_DB - 3.0;
                    let was = interf_on;
                    if !interf_on {
                        if enter {
                            interf_on = true;
                            clean_cnt = 0;
                        }
                    } else if margin_gone || quiet {
                        interf_on = false;
                    } else if clean {
                        clean_cnt += 1;
                        if clean_cnt >= 3 {
                            interf_on = false;
                        }
                    } else {
                        clean_cnt = 0;
                    }
                    let interf = interf_on;
                    crate::conv_tx::SPREAD_ON.store(interf, Ordering::Relaxed);
                    if interf != was {
                        surv_cnt = 0;
                        log(&format!(
                            "olla: interference regime {} at {} (snr {:.1} dB, rung needs {:.1}, block ok {:.2}{})",
                            if interf { "ON - holding the rung, loss is not SNR" } else { "off" },
                            Mcs::ALL[cur].label(), fb.snr_db, snr_thresh(cur), mn_prob[cur],
                            if quiet { format!(", nothing delivered for {quiet_ms} ms") } else { String::new() }
                        ));
                    }
                    // expected throughput: frame airtime is FIXED, so thr ~ prob x
                    // payload_block/nfrag (useful payload per FRAME; low MCS spends more frames per
                    // block). v40.34: a lost block tears the WHOLE video frame (and costs an IDR),
                    // so score a rate by the chance a frame of K blocks survives, pb^K, not by pb
                    // alone: MCS5 at 0.85 with 4-block frames is worth 0.52 x 6, MCS3 at 0.95 is
                    // worth 0.81 x 4; the safer rate wins, which is also what the video rate
                    // controller below wants (the two disagreed and the bitrate sat at 25 % for
                    // hours).
                    let kbpf = {
                        let st = shared.stats.lock().unwrap();
                        (st.blocks_per_frame as i32).clamp(4, 8)
                    };
                    let thr = |i: usize, pb: f32| -> f32 {
                        // v40.48: useful bytes per frame of air - a rung-sized segment in one
                        // frame with compact blocks, 2016/fragments without
                        let (bytes, nf) = block_shape(i);
                        if crate::phy64() {
                            // branch B: every rung is one packet a block - what differs is its air
                            return pb.max(0.0).powi(kbpf) * bytes / nyx_proto::phy64_air_us(i, FRAME_PAYLOAD_BYTES);
                        }
                        pb.max(0.0).powi(kbpf) * bytes / nf
                    };
                    // candidates: ALL 6 rungs (minstrel considers them all, not just the
                    // neighbours: MCS3 -> 4 both use 2 fragments/block so their thr is equal, and
                    // ±1 would stay at 3 for ever while the real gain sits at 1-fragment MCS5;
                    // measured: an MCS3 plateau). A rung ABOVE with no fresh data is NOT eligible
                    // (the rotating probe will measure it in time); a rung BELOW with no fresh data
                    // gets a monotone prior (at least as good as the current rung).
                    let mut best = cur;
                    let mut best_thr = if fresh(cur) && mn_prob[cur] >= 0.0 {
                        thr(cur, mn_prob[cur])
                    } else {
                        0.0
                    };
                    for i in 0..6 {
                        if i == cur {
                            continue;
                        }
                        let pb = if fresh(i) && mn_prob[i] >= 0.0 {
                            mn_prob[i]
                        } else if i < cur {
                            if mn_prob[cur] >= 0.0 {
                                // v40.45: under interference an unmeasured lower rung gets
                                // NO optimism - its longer frames are hit just the same
                                if interf { mn_prob[cur] } else { (mn_prob[cur] + 0.35).min(1.0) }
                            } else {
                                1.0
                            }
                        } else {
                            continue; // the rung above unknown -> wait for a probe
                        };
                        // v40.49: no climb into a rung the SNR says cannot hold. The rungs a
                        // fast step down jumps over keep their numbers from before the fade and
                        // were climbed back into within 3 s (sim: MCS3 -> 4 at 16 dB, half its
                        // frames lost). Clearing those numbers instead cost the climb after an
                        // OUTAGE, where they are right again the moment the signal is back
                        // (bench, 5 s outages: MCS3-4 instead of 5 twelve seconds after).
                        if i > cur && snr_thresh(i) > fb.snr_db {
                            continue;
                        }
                        // v40.50: a rung that just failed waits out its back-off
                        if i > cur && retry_at[i].is_some_and(|t| Instant::now() < t) {
                            continue;
                        }
                        // (a floor of prob >= 0.7 for "reliable enough for whole frames" was tried
                        // and was WORSE: in transitions every rung is < 0.7 -> only MCS0 qualifies
                        // -> sinks to the bottom and sticks. Low frame probability at a high rung
                        // is what the BASE layer is for; that is the simulcast design.)
                        let t = thr(i, pb);
                        // v40.45b: under interference a measured lower rung must beat the
                        // current one by 20 % (not just edge it): WiFi comes in seconds of
                        // clean and seconds of loss, and the rung we sat on a moment ago
                        // always looks a little better than the one being hit now
                        // (measured: 5 <-> 4 every ~5 s at 0.86 vs 0.69).
                        let need = if i > cur {
                            best_thr * 1.1
                        } else if interf {
                            best_thr * 1.2
                        } else {
                            best_thr
                        };
                        if t > need {
                            best = i;
                            best_thr = t;
                        }
                    }
                    let _ = best_thr;
                    // v32.3: EVERY DECISION MOVES AT MOST ONE RUNG. Measured with the static ruler:
                    // 49 of 57 changes were jumps of >= 2 rungs, because a rung WITHOUT DATA got an
                    // optimistic prior (prob_cur + 0.35), so argmax jumped straight 5 -> 0 and
                    // back. Minstrel would not: an unmeasured rate is PROBED, never used as grounds
                    // for a jump. Clamping to ±1 keeps the destination (any rung is reachable
                    // within seconds) and removes the leaps; the video bitrate stops jerking.
                    if best > cur + 1 {
                        best = cur + 1;
                    } else if best + 1 < cur {
                        best = cur - 1;
                    }
                    // v40.53: ...and the clamp must not land on a rung that is backing off. The
                    // loop skips it, a rung above it wins, and the clamp walked into it anyway:
                    // bench 6/10, MCS0 -> 1 every 4 s, each lost within a second, "not tried again
                    // for 30 s" logged each time.
                    if best > cur && retry_at[best].is_some_and(|t| Instant::now() < t) {
                        best = cur;
                    }
                    // v40.50: under interference the rung below is measured by a few probe blocks
                    // a window (see `below`): one window's luck is not a loss that depends on the
                    // rung. Three windows in a row must agree (sim, WiFi-like bursts that kill
                    // every rung alike: one window was enough to walk MCS5 -> 4 for 46 of 110 s,
                    // 43 frames lost against 27). The survival and fading paths below still act
                    // at once. The same whenever the SNR is well above what the rung needs (the
                    // regime flag goes off after three clean windows and the probed rung below
                    // then won the plain comparison: MCS5 -> 4 on a single window again).
                    if (interf || snr_margin >= INTERF_MARGIN_DB) && best < cur {
                        interf_down += 1;
                        if interf_down < 3 {
                            best = cur;
                        }
                    } else {
                        interf_down = 0;
                    }
                    // survival: the current rate is nearly dead -> go down at once, do not wait for
                    // argmax (argmax needs samples and a collapsing channel gives poor ones)
                    if mn_prob[cur] >= 0.0 && mn_prob[cur] < 0.2 && cur > 0 {
                        // v40.45: in the interference regime a dead rung is not evidence
                        // for the rung below; only 3 dead windows in a row (or a measured
                        // better rung below) move us
                        surv_cnt = if interf { surv_cnt + 1 } else { 0 };
                        let below_better = cur > 0 && fresh(cur - 1) && mn_prob[cur - 1] > mn_prob[cur] + 0.2;
                        if !interf || surv_cnt >= 3 || below_better {
                            best = cur - 1;
                            surv_cnt = 0;
                        }
                    } else {
                        surv_cnt = 0;
                    }
                    // v40.49: a sudden fade. Going down one rung per window left the video on a
                    // rung that loses half its frames for ~2 s at every sudden drop (simulator,
                    // 24 -> 16 dB: MCS5 -> 4 -> 3). When the rung is failing anyway and the SNR
                    // (honest now: it falls when nothing decodes) says a lower rung, go there at
                    // once, at most three rungs; interference keeps its own rules.
                    // A window that straddles the drop still shows the rung half alive (0.52 in
                    // the simulator, with the SNR already at 1.4 dB): an SNR well under what the
                    // rung needs counts with a milder loss, and far under it (> 6 dB) lifts the
                    // three-rung limit (24 -> 5 dB took 5 -> 4 -> 1 -> 0 over 2 s, 3.6 s frozen).
                    let snr_rung = (0..6)
                        .rev()
                        .find(|&r| snr_thresh(r) + 1.5 <= fb.snr_db)
                        .unwrap_or(0);
                    let under = snr_thresh(cur) - fb.snr_db;
                    // (a QUIET link delivers nothing at this rung whatever its numbers say: they
                    // can be from before, the window's blocks still in flight)
                    let failing = (mn_prob[cur] >= 0.0
                        && (mn_prob[cur] < 0.5 || (under > 3.0 && mn_prob[cur] < 0.8)))
                        || (quiet && under > 3.0);
                    if !interf && failing && snr_rung < cur {
                        let floor = if under > 6.0 { 0 } else { cur.saturating_sub(3) };
                        let target = snr_rung.max(floor);
                        if target < best {
                            best = target;
                        }
                    }
                    // v40.53 RELIABILITY FIRST. Where some rung measures clean (>= REL_OK), a
                    // picture is worth more than the bitrate a losing rung adds: each lost block is
                    // a frozen frame. A rung under REL_OK for two windows (none of the first 1.5 s
                    // on it: that window counts blocks sent at the new rung against acknowledgements
                    // still 100-300 ms behind) goes straight to the highest clean rung below - past
                    // the one-rung clamp and the interference regime's three windows and 5 s dwell,
                    // which are for loss that hits every rung alike. And while a clean rung exists
                    // at or below this one, no climb into a rung not measured clean. Two rooms
                    // apart, EDUP, 2474.5 MHz: 64QAM dies under the WiFi bursts, 16QAM lives; the
                    // stream sat on MCS4-5 at 0.3-1.0, 63-78 frames lost a minute, 26 with MCS3 held
                    // by hand; the simulator's edge (22 dB, 30 % of 20 ms bursts 8 dB down, control
                    // 35 % lost): 113-314 lost against 1. Loss that hits every rung (WiFi kills
                    // every frame) leaves no rung clean, and the rules above keep deciding.
                    let clean = |i: usize| fresh(i) && mn_prob[i] >= REL_OK;
                    let mut rel_jump = false;
                    let low_now = fresh(cur)
                        && mn_prob[cur] >= 0.0
                        && mn_prob[cur] < REL_OK
                        && last_step.elapsed() >= Duration::from_millis(1500);
                    rel_low = if low_now { rel_low + 1 } else { 0 };
                    let clean_below = (0..cur).rev().find(|&i| clean(i));
                    if let (true, Some(r)) = (rel_low >= 2, clean_below) {
                        if r < best || best >= cur {
                            log(&format!(
                                "olla: {} loses blocks ({:.2}), {} measured clean ({:.2}) -> straight there",
                                Mcs::ALL[cur].label(), mn_prob[cur], Mcs::ALL[r].label(), mn_prob[r]
                            ));
                            best = r;
                            rel_jump = true;
                            rel_low = 0;
                        }
                    } else if best > cur && !clean(best) && (clean(cur) || clean_below.is_some()) {
                        best = cur;
                    }
                    // v40.53: on the repeat step the way up is MCS0 first (its probes, below): the
                    // step is a rung under MCS0, and one rung at a time holds for it too. MCS1's
                    // numbers from before the fade outweighed a dying MCS0 and took the stream
                    // from the step straight to MCS1 (sim, 2 dB: on/off every 2-4 s).
                    if rep2_on && best > cur {
                        best = cur;
                    }
                    // climbing needs 2 CONSECUTIVE windows in agreement (probes at the fade edge
                    // are noisy; a single good window often lies; stepping down stays immediate).
                    // Soak: 18 flips -> target < 10.
                    if best > cur {
                        up_confirm = if up_want == Some(best) {
                            up_confirm + 1
                        } else {
                            up_want = Some(best);
                            1
                        };
                    } else {
                        up_want = None;
                        up_confirm = 0;
                    }
                    let allow = best < cur || up_confirm >= 2;
                    if best != cur
                        && allow
                        && last_step.elapsed() > Duration::from_millis(
                            // v40.45b: a step down under interference dwells 5 s (not one onto a
                            // rung measured clean - v40.53)
                            if best < cur { if interf && !rel_jump { 5000 } else { 1000 } } else { 3000 },
                        )
                    {
                        auto_mcs = Mcs::ALL[best];
                        last_step = Instant::now();
                        interf_down = 0;
                        log(&format!(
                            "olla: {} -> {} (prob {:.2}/{:.2}/{:.2})",
                            Mcs::ALL[cur].label(),
                            auto_mcs.label(),
                            if cur > 0 { mn_prob[cur - 1] } else { -1.0 },
                            mn_prob[cur],
                            if cur + 1 < 6 { mn_prob[cur + 1] } else { -1.0 },
                        ));
                    }
                    // v40.49: back from an outage. The rung that held before, again at once when
                    // the SNR that fell far below what it needs is back near where it was - the
                    // climb (one rung per 3 s, two windows to agree) took ~14 s from MCS3 to 5
                    // after every 5 s outage in the simulator, the video at 2/3 of its rate.
                    // Interference keeps its own logic: its SNR does not dip.
                    {
                        let now_cur = auto_mcs.index();
                        let snr = fb.snr_db;
                        snr_dip = snr_dip.min(snr);
                        if let Some((r, s0, at)) = stable {
                            if now_cur < r
                                && at.elapsed() < Duration::from_secs(60)
                                && !retry_at[r].is_some_and(|t| Instant::now() < t)
                                && snr_dip < snr_thresh(r) - 6.0
                                && snr >= s0 - 3.0
                                && snr >= snr_thresh(r) + 3.0
                            {
                                log(&format!(
                                    "olla: back from an outage (snr fell to {snr_dip:.1}, now {snr:.1} dB) -> {} as before",
                                    Mcs::ALL[r].label()
                                ));
                                auto_mcs = Mcs::ALL[r];
                                last_step = Instant::now();
                                restore_at = Some((r, Instant::now()));
                                up_want = None;
                                up_confirm = 0;
                                snr_dip = snr;
                                // its numbers are from the windows the outage began in (dead):
                                // kept, they took it straight back down, and with the rungs
                                // below cleared by the fast step down the climb started again
                                // from nothing (bench: MCS3 / 2 12 s after a 5 s outage). A
                                // clean slate the next real window outweighs.
                                mn_prob[r] = 1.0;
                                mn_acc[r] = (2.0, 2.0);
                                // the video's ceiling too: it collapsed with delivery during the
                                // outage and climbs x1.05 a window (bench: 1.67 -> 2.1 Mbps took
                                // 9 s at MCS5 after the rung was back)
                                air_scale = air_scale.max(stable_air);
                                r_ema = r_ema.max(0.96);
                                ladder_restore = Some(stable_tier);
                            }
                        }
                        let c = auto_mcs.index();
                        // not moved down while a deep dip is under way: the first window after
                        // the signal returns, the SNR still climbing, would overwrite it
                        let dipping = stable.is_some_and(|(r, _, at)| {
                            c < r && snr_dip < snr_thresh(r) - 6.0 && at.elapsed() < Duration::from_secs(60)
                        });
                        // v40.50: kept fresh in the interference regime too - a rung at
                        // >= 0.9 carried the stream whatever the regime says. On the bench the
                        // regime is on most of the time, the record went stale (> 60 s) and a
                        // 12 s outage was climbed back from MCS0 over 17 s.
                        if c == now_cur && !dipping && fresh(c) && mn_prob[c] >= 0.9 && snr >= snr_thresh(c) {
                            stable = Some((c, snr, Instant::now()));
                            snr_dip = snr;
                            stable_air = air_scale;
                            stable_tier = cur_tier;
                        }
                    }
                    // v40.50: climbs that failed (see `climb_at`)
                    {
                        let now_idx = auto_mcs.index();
                        if hz_now != retry_hz {
                            retry_hz = hz_now;
                            retry_at = [None; 6];
                            retry_gap_s = [0; 6];
                            climb_at = None;
                            restore_at = None;
                        }
                        // v40.53: an outage return lost again (see `restore_at`); before the climb
                        // check below, which sees the same return as a climb and needs the SNR
                        // high at the loss - it is 0 then, starved
                        if let Some((r, t)) = restore_at {
                            if now_idx < r {
                                if t.elapsed() < Duration::from_secs(CLIMB_FAIL_S) {
                                    retry_gap_s[r] = (retry_gap_s[r] * 2).clamp(6, 30);
                                    retry_at[r] = Some(Instant::now() + Duration::from_secs(retry_gap_s[r]));
                                    climb_at = None;
                                    log(&format!(
                                        "olla: {} lost again {:.1} s after the outage return -> no return for {} s",
                                        Mcs::ALL[r].label(), t.elapsed().as_secs_f32(), retry_gap_s[r]
                                    ));
                                }
                                restore_at = None;
                            } else if t.elapsed() >= Duration::from_secs(CLIMB_OK_S) {
                                restore_at = None;
                            }
                        }
                        if now_idx > cur {
                            climb_at = Some((now_idx, Instant::now()));
                        } else if now_idx < cur {
                            if let Some((r, t)) = climb_at {
                                // (the interference margin: at a rung's edge the loss is the SNR
                                // and the next climb is the fade ending - sim fade profile, MCS2
                                // at 10 dB locked out 6-12 s with a 1.5 dB margin)
                                if r == cur
                                    && t.elapsed() < Duration::from_secs(CLIMB_FAIL_S)
                                    && fb.snr_db >= snr_thresh(r) + INTERF_MARGIN_DB
                                {
                                    retry_gap_s[r] = (retry_gap_s[r] * 2).clamp(6, 30);
                                    retry_at[r] = Some(Instant::now() + Duration::from_secs(retry_gap_s[r]));
                                    log(&format!(
                                        "olla: {} lost again {:.1} s after the climb at snr {:.1} dB (loss, not SNR) -> not tried again for {} s",
                                        Mcs::ALL[r].label(), t.elapsed().as_secs_f32(), fb.snr_db, retry_gap_s[r]
                                    ));
                                }
                            }
                            climb_at = None;
                        } else if let Some((r, t)) = climb_at {
                            if r == now_idx && t.elapsed() >= Duration::from_secs(CLIMB_OK_S) {
                                retry_gap_s[r] = 0;
                                retry_at[r] = None;
                                climb_at = None;
                            }
                        }
                    }
                    // set the probe: the rung above the CURRENT one, only when sending enough and
                    // that rung is not yet known to be good (minstrel: no probes wasted on a rate
                    // already known)
                    let cur = auto_mcs.index();
                    // ROTATING probe over every uncertain rung above (minstrel keeps every rate's
                    // statistics fresh; a rung known >= 0.95 and still fresh costs no probe).
                    // v40.49: the rung just above is the only one a step can reach, so while it is
                    // uncertain it gets half of the probe blocks in EVERY window and the rotation
                    // shares the other half. One rung per window left it without data four
                    // windows in five: 15-25 s from MCS0 back up after a fade, in the simulator.
                    let uncertain = |i: usize| i < 6 && (!fresh(i) || mn_prob[i] < 0.95);
                    let near = if uncertain(cur + 1) { Some(cur + 1) } else { None };
                    let mut far = None;
                    for k in 1..6 {
                        let i = cur + ((probe_rr as usize + k) % (6 - cur).max(1));
                        if i > cur + 1 && uncertain(i) {
                            far = Some(i);
                            probe_rr = (i - cur) as u64;
                            break;
                        }
                    }
                    // v40.50: and the rung BELOW while this one is losing. Under the interference
                    // regime a lower rung counts only when measured (it must beat this one by 20 %,
                    // an unmeasured one gets no optimism) - right for WiFi, whose bursts kill every
                    // rung alike, but nothing measured it: on 2500 MHz on the bench (MCS5 13 %
                    // CRC-failed at 30 dB, MCS4 4 %) the regime held MCS5 for 150 s, 139 frames lost.
                    let losing = interf_on || (fresh(cur) && mn_prob[cur] < 0.95);
                    // v40.53: past the rungs below already measured losing, to the first one not
                    // known to lose - MCS4 losing like MCS5 left MCS3 unmeasured, and the
                    // reliability floor above had nothing to go to
                    let below = (cur > 0 && losing).then(|| {
                        (0..cur).rev().find(|&i| !(fresh(i) && mn_prob[i] < REL_OK)).unwrap_or(cur - 1)
                    });
                    // v40.53: the repeat step. In while MCS0 loses REP2_IN_PROB of its blocks two
                    // windows in a row, out when the probes find MCS0 at REP2_OUT_PROB three in a
                    // row. Where the two cross: per picture of K blocks the step carries half as
                    // much at a higher block rate, p6^K x 222 against p0^K x 447 - with the bench's
                    // edge (A -48 dB: MCS0 0.51 of the synced frames, the step 0.73) the step
                    // wins below p0 ~0.8. The band between keeps it from flapping.
                    // v40.59: not in the interference regime (see Config::rep2_interf)
                    if cur != 0 || !cfg.rep2 || (interf_on && !cfg.rep2_interf) {
                        if rep2_on {
                            log(&format!(
                                "olla: repeat step off (now {}{})",
                                Mcs::ALL[cur].label(),
                                if interf_on { ", interference: a frame hit is lost at any rung" } else { "" }
                            ));
                        }
                        rep2_on = false;
                        rep2_in = 0;
                        rep2_probes.clear();
                    } else if !rep2_on {
                        rep2_in = if fresh(0) && mn_prob[0] >= 0.0 && mn_prob[0] < REP2_IN_PROB {
                            rep2_in + 1
                        } else {
                            0
                        };
                        if rep2_in >= 2 {
                            rep2_on = true;
                            rep2_in = 0;
                            rep2_probes.clear();
                            rep2_settle = 2;
                            log(&format!(
                                "olla: MCS0 block ok {:.2} -> repeat step on (QPSK 1/2 x2, snr {:.1} dB)",
                                mn_prob[0], fb.snr_db
                            ));
                        }
                    } else if rep2_settle > 0 {
                        rep2_settle -= 1;
                    } else {
                        rep2_probes.push_back((mn_dbg_ds[0], u64::from(mn_dbg_dok[0])));
                        if rep2_probes.len() > REP2_OUT_WIN {
                            rep2_probes.pop_front();
                        }
                        // (summed, then capped: the receiver's counts run a window ahead or behind)
                        let (n, ok) = rep2_probes.iter().fold((0u64, 0u64), |a, &(s, o)| (a.0 + s, a.1 + o));
                        let ok = ok.min(n);
                        if n >= REP2_OUT_N && ok as f32 >= REP2_OUT_PROB * n as f32 {
                            rep2_on = false;
                            rep2_probes.clear();
                            log(&format!(
                                "olla: MCS0 probes {ok}/{n} ok -> repeat step off (snr {:.1} dB)",
                                fb.snr_db
                            ));
                        }
                    }
                    // ...and while on, only MCS0 is probed (the stream itself is not at a rung of the
                    // table, and the way up is MCS0 first: a probe at MCS1 there is a block lost)
                    let slots: Vec<usize> = if rep2_on {
                        vec![0]
                    } else {
                        [near, below, far].into_iter().flatten().take(2).collect()
                    };
                    let pm = match slots.as_slice() {
                        [a, b] => *a as u64 | (*b as u64) << 8,
                        [a] => *a as u64 | 0xFF00,
                        _ => 0xFFFF,
                    };
                    crate::conv_tx::PROBE_MCS.store(
                        pm,
                        std::sync::atomic::Ordering::Relaxed,
                    );
                    sent_win_t = Instant::now();
                }
            } else {
                // v12.1: NO feedback. A REAL cold start (never any feedback): probe at the
                // fabric-decode level (MCS3 for the PL) then STEP DOWN one rung every 4 s to the
                // bottom while still blind (a very bad channel needs low MCS; the ground PC demod
                // at MCS0/1 still produces feedback -> the loop closes, fb_fresh comes on and OLLA
                // takes over). A fade after running: KEEP the current MCS, then also step down. The
                // IQ path keeps its floor.
                let start = if cfg.txbits || cfg.txpl {
                    mcs_floor.max(3)
                } else {
                    mcs_floor
                };
                // v40.54: blind, the floor is `blindfloor`: the repeat step only when it says 6 (it
                // doubles a frame's air - under WiFi more of it is hit - and nothing can tell here
                // whether the loss it was taken on for was the SNR's). With `blindhold` (a fade
                // that took the way back, not a cold start) the rung stays where the feedback
                // left it, repeat step and all.
                let hold = (cfg.blind_hold && fb.updated.is_some()) || bw_hold;
                if rep2_on && cfg.blind_floor < 6 && !hold {
                    rep2_on = false;
                    rep2_probes.clear();
                    log(&format!("auto: feedback blind -> repeat step off (blind floor MCS{})", cfg.blind_floor));
                }
                match blind_since {
                    None => {
                        blind_since = Some(Instant::now());
                        // real cold start -> jump to the probe level; fade -> hold
                        if fb.updated.is_none() {
                            auto_mcs = Mcs::ALL[start];
                        }
                    }
                    Some(t) => {
                        if !hold && t.elapsed() >= Duration::from_secs(4) {
                            let cur = auto_mcs.index();
                            // v30.3: through the floor: at the fade edge feedback often goes stale,
                            // so this blind branch takes over, and if it stops at floor 3 then MCS1
                            // (the only thing that survives 13 dB SNR) is never reached.
                            // v40.54: down to the blind floor (`blindfloor`; 6 = on through MCS0
                            // to the repeat step, as before). A rung below it is kept.
                            let floor = usize::from(cfg.blind_floor).min(6);
                            let stop = if floor >= 6 { 0 } else { floor };
                            if cur > stop {
                                auto_mcs = Mcs::ALL[cur - 1];
                                log(&format!(
                                    "auto: feedback blind for {}s -> stepping down to {}",
                                    4, auto_mcs.label()
                                ));
                            } else if floor >= 6 && !rep2_on && cfg.rep2 && fb.updated.is_some() {
                                // v40.53: a fade (the feedback was there before), not a cold start
                                rep2_on = true;
                                rep2_probes.clear();
                                rep2_settle = 2;
                                log("auto: feedback blind at MCS0 -> repeat step on");
                            }
                            blind_since = Some(Instant::now());
                        }
                    }
                }
            }
        }
        let mcs = if cfg.auto_mcs { auto_mcs } else { cfg.mcs };
        // v40.53: the rung the conv path sends the stream at - 6 = the repeat step below MCS0
        let rep2 = cfg.auto_mcs && cfg.rep2 && rep2_on && mcs.index() == 0;
        let rung = if rep2 { 6 } else { mcs.index() };
        // v40.56: while the link hops, the rung of the channel this end is on (the receiving end
        // measures each one, Msg::ChanRates; fresh = under 5 s); the rung above still sets the
        // budget. Not with the rung set by hand.
        let chan_rung = if cfg.auto_mcs && cfg.chan_rate && fb_fresh {
            shared.chan_rates.lock().unwrap().as_ref().and_then(|(at, v)| {
                (at.elapsed() < Duration::from_secs(5))
                    .then(|| v.iter().find(|(hz, _)| *hz == hz_now).map(|&(_, r)| r))
                    .flatten()
                    .filter(|&r| r <= 6)
            })
        } else {
            None
        };
        crate::conv_tx::CHAN_RUNG_ON.store(chan_rung.is_some(), Ordering::Relaxed);
        // v40.57: with per-channel rungs the budget comes from the lowest rung of the channels in
        // use (excluded ones carry none): the rate control's own rung saw the losses of every
        // channel mixed and sat on the repeat step (bench 8/10: 0.23 Mbps while hopping). The
        // lowest, not the average: a 1 s dwell on a slow channel would otherwise queue more than
        // the 300 ms the queue holds.
        let budget_rung = if chan_rung.is_some() {
            shared.chan_rates.lock().unwrap().as_ref().and_then(|(_, v)| {
                v.iter().map(|&(_, r)| r).filter(|&r| r <= 6).reduce(|a, b| {
                    let k = |r: u8| if r == 6 { -1i8 } else { r as i8 };
                    if k(a) <= k(b) { a } else { b }
                })
            })
        } else {
            None
        }
        .map_or(rung, usize::from);
        if chan_rung != last_chan_rung {
            if let Some(r) = chan_rung {
                log(&format!("olla: per-channel rung {:.1} MHz -> {}", hz_now as f64 / 1e6,
                             if r == 6 { "the repeat step".to_string() } else { Mcs::ALL[usize::from(r)].label().to_string() }));
            } else if last_chan_rung.is_some() {
                log("olla: per-channel rungs off - one rung again");
            }
            last_chan_rung = chan_rung;
        }
        // v40.49: no probes with the rung set by hand, nor while blind (nobody counts them); the
        // last probe rung used to stay set and every 12th block kept going out at it. v40.56: nor
        // with per-channel rungs (the channel's own rung is the measurement)
        if !cfg.auto_mcs || !fb_fresh || chan_rung.is_some() {
            crate::conv_tx::PROBE_MCS.store(0xFFFF, Ordering::Relaxed);
        }
        // Publish for the conv transmit path (it runs on its own thread and cannot see this local).
        // This lets OLLA drive conv too; before, conv was pinned at MCS5 because only that rate
        // carried a block.
        crate::conv_tx::CUR_MCS.store(chan_rung.map_or(rung, usize::from) as u64, Ordering::Relaxed);

        // ------------------------------------------------------- source
        if first_frame && !said_acquiring {
            // once: with no camera yet the loop comes back here every 100 ms
            said_acquiring = true;
            log("worker: first iteration — acquiring source");
        }
        let pass_au: Option<RtspAu> = if pass {
            match pass_next(&shared.rtsp, &mut pst) {
                Some(au) => Some(au),
                None => {
                    // No camera frame due yet: answer NACKs now rather than after the next
                    // frame (17/9 on air: retransmissions came back 54-223 ms after the loss,
                    // the receiver gives a hole 300 ms, and in pass-through every hole it
                    // gives up on costs the picture until the camera's next key frame).
                    // Only the conv path exists on the board; the full handling is below.
                    if cfg.txconv {
                        answer_conv_nacks!(cfg, pass);
                    }
                    continue;
                }
            }
        } else {
            None
        };
        let video_frame: Option<RgbFrame> = if pass { None } else { match cfg.source {
            SourceKind::Pattern => Some(pattern.render(aw, ah)),
            SourceKind::Webcam => {
                let raw = shared.webcam.frame.lock().unwrap().clone();
                match raw {
                    Some(raw) => {
                        webcam_wait = 0;
                        Some(resize_rgb(&raw, aw, ah))
                    }
                    None => {
                        webcam_wait += 1;
                        if webcam_wait >= 30 && shared.webcam.status().starts_with("open error") {
                            log("webcam: no camera opens - sending the test pattern instead (Source in the settings switches back)");
                            shared.config.lock().unwrap().source = SourceKind::Pattern;
                            webcam_wait = 0;
                        }
                        std::thread::sleep(Duration::from_millis(100));
                        continue;
                    }
                }
            }
            SourceKind::Rtsp => {
                // Paced by this loop's tick like the webcam, not by the camera: taking
                // pictures as they arrived sent them in the bursts RTSP delivers them
                // in, the receiver's hold timer read the gaps as loss and asked for
                // IDRs, and the rate controller walked down to MCS0 (10/9). A tick
                // faster than the camera re-encodes the same picture, which is cheap.
                let raw = shared.rtsp.frame.lock().unwrap().clone();
                match raw {
                    Some(raw) => Some(resize_rgb(&raw, aw, ah)),
                    None => {
                        std::thread::sleep(Duration::from_millis(100));
                        continue;
                    }
                }
            }
            SourceKind::File => {
                // the decoder runs at the file's pace (ffmpeg -re); this tick takes the latest
                let raw = shared.vfile.frame.lock().unwrap().clone();
                match raw {
                    Some(raw) => Some(resize_rgb(&raw, aw, ah)),
                    None => {
                        std::thread::sleep(Duration::from_millis(100));
                        continue;
                    }
                }
            }
            SourceKind::RandomData => None,
        } };
        // v40.46 latency test: the millisecond counter into the picture before encoding (the
        // ground app shows the same counter; freeze both and subtract)
        let video_frame = video_frame.map(|mut f| {
            if cfg.stamp {
                nyx_common::stamp::burn(&mut f, nyx_common::stamp::counter_ms());
            }
            f
        });

        // v12.2: GLASS-IN mark for the glass-to-glass measurement: the moment the frame is produced
        // (pattern), BEFORE encoding. Pairs with "G2G out" in nyx-rx (after decode). frame_id % 8
        // matches the sampling cadence of LAT.
        if video_frame.is_some() && frame_id % 8 == 0 {
            log(&format!("G2G in id={frame_id}"));
        }
        let (src_type, app_data, preview) = if let Some(au) = pass_au {
            pass_fps_roll.push(1);
            pass_byte_roll.push(au.data.len());
            // The receiver asks for a key frame after a loss; the camera sends one when it
            // wants to, so the request can only be counted.
            if std::mem::take(&mut shared.feedback.lock().unwrap().need_idr) {
                pass_idr_req += 1;
            }
            let payload_bits = (FRAME_PAYLOAD_BYTES * 8) as f32;
            let cam_fps = pass_fps_roll.count_per_sec().max(1.0);
            let cap = conv_air_ceiling_bps(cfg.air_gap_ms, false, budget_rung, payload_bits, cam_fps, cfg.fec);
            video_bitrate = (pass_byte_roll.per_sec() * 8.0) as u32;
            // ONVIF: once a second, the bitrate the camera should be at. What the air carries at
            // this rung, cut to the spacing the daemon really keeps up with (the board app's
            // pacer) and to the share of blocks that arrive; the camera gets CAM_HEADROOM of it.
            // Down at once, up only after 4 s of room and by at most 30 % a step (the rung flaps
            // 5 <-> 4 while the air changes, and a camera may restart its stream on a change).
            // ...and only once the receiving end is answering: while blind the rung starts at
            // MCS3 and the delivered/needed ratio is stale, which drove the camera to its floor
            // right after every restart and took half a minute to climb back (18/9).
            if cfg.cam_adapt && shared.rtsp.onvif_ok.load(Ordering::Relaxed) && fb_fresh {
                if cam_tick.elapsed() >= Duration::from_secs(1) {
                    cam_tick = Instant::now();
                    let ceiling = crate::cam_ceiling(&shared);
                    let mut air = cap;
                    let floor_us = shared.pace_floor_us.load(Ordering::Relaxed) as f32;
                    if floor_us > 3000.0 {
                        let nfrag = conv_frames(rung.min(6), FRAME_PAYLOAD_BYTES) as f32;
                        let blocks = (1e6 / floor_us / nfrag - if cfg.fec { cam_fps * (1.0 + fec_extra()) } else { 0.0 }).max(1.0);
                        air = air.min(blocks * payload_bits * 0.85);
                    }
                    let target = ((air * air_scale * CAM_HEADROOM / 1000.0) as u32).max(CAM_MIN_KBPS);
                    let target = if ceiling > 0 { target.min(ceiling) } else { target };
                    let late = pst.drop_late.saturating_sub(cam_late0);
                    cam_late0 = pst.drop_late;
                    if cam_want == 0 {
                        cam_want = target;
                    } else if late > 0 {
                        // frames went old waiting: the camera is above what really goes out
                        cam_want = target.min((video_bitrate as f32 / 1000.0 * 0.7) as u32).max(CAM_MIN_KBPS);
                        cam_up_since = None;
                    } else if target < cam_want {
                        cam_want = target;
                        cam_up_since = None;
                    } else if target as f32 > cam_want as f32 * 1.15 {
                        let since = *cam_up_since.get_or_insert_with(Instant::now);
                        if since.elapsed() >= Duration::from_secs(4) {
                            cam_want = target.min((cam_want as f32 * 1.3) as u32);
                            cam_up_since = None;
                        }
                    } else {
                        cam_up_since = None;
                    }
                    shared.rtsp.want_kbps.store(cam_want, Ordering::Relaxed);
                }
            } else if !cfg.cam_adapt {
                cam_want = 0;
                shared.rtsp.want_kbps.store(0, Ordering::Relaxed);
            }
            {
                let mut st = shared.stats.lock().unwrap();
                st.pass_fps = cam_fps;
                st.pass_kbps = video_bitrate as f32 / 1000.0;
                st.pass_drop_late = pst.drop_late;
                st.pass_drop_nokey = pst.drop_nokey;
                st.pass_idr_req = pass_idr_req;
                st.link_cap_bps = cap;
                st.rc_one_burst = cap;
            }
            // the PC build still decodes the camera: that picture is the preview
            let preview = shared.rtsp.frame.lock().unwrap().take();
            (SourceType::H264, au.data, preview)
        } else { match video_frame {
            Some(f) => match cfg.codec {
                Codec::H264 => {
                    // Rate control follows the ACHIEVED burst cadence, not
                    // the theoretical back-to-back PHY rate. The FPGA capture
                    // pipeline caps real throughput near a dozen frames/s, so
                    // 0.55*phy_bitrate (which assumes ~360 frames/s) over-
                    // drives the link ~10x and floods it with video that can
                    // never be delivered — the root of the stutter. Budget on
                    // the MEASURED PHY-frame throughput and reported BLER,
                    // and keep each video frame inside ONE CAPTURE: blocks
                    // of a frame ride a single trigger window back-to-back,
                    // so the per-frame byte budget scales with how many PHY
                    // frames fit the window at the current MCS (3 at MCS5,
                    // 2 at MCS4, 1 below) — 2-3x the video quality at high
                    // MCS for the same capture rate.
                    let phy_fps = block_roll.per_sec().max(1.0);
                    let usable = (1.0 - fb.bler).clamp(0.3, 1.0);
                    let payload_bits = (FRAME_PAYLOAD_BYTES * 8) as f32;
                    // v5: the fabric RX path only demodulates the FIRST frame of each capture
                    // (llrmode/DEC): a burst of several blocks per capture loses blocks 2-3
                    // structurally. txbits (MCS3+) => 1 block per capture.
                    let per_capture = if cfg.txbits
                        && mcs.data_syms_per_frame() <= pl
                    {
                        // v6: pair 2 frames per capture when pair is on and the pair fits the
                        // 24-symbol grid (2*(2+dsy)+1 <= 24 -> dsy <= 9)
                        if cfg.pair && mcs.data_syms_per_frame() <= 9 {
                            2.0
                        } else {
                            1.0
                        }
                    } else {
                        (BURST_BUDGET / mcs.frame_len_samples()).clamp(1, 3)
                            as f32
                    };
                    let fabric_path =
                        cfg.txbits && mcs.data_syms_per_frame() <= pl;
                    let one_burst = if cfg.txconv {
                        conv_air_ceiling_bps(
                            cfg.air_gap_ms, (cfg.simulcast || (cfg.blind_base && !fb_fresh)) && mcs.index() >= 2,
                            budget_rung, payload_bits, afps, cfg.fec,
                        )
                    } else if fabric_path {
                        FAB_CAPTURES_PS * per_capture * payload_bits * 0.85
                    } else {
                        cfg.fps.max(1.0) * payload_bits * 0.85 * per_capture
                    };
                    // v6.14: fabric: measured in DELIVERED blocks (feedback), ceiling in
                    // captures/s. The IQ path keeps the old formula.
                    let budget = if cfg.txconv {
                        // The conv path budgets from the air this board really has: one_burst
                        // comes from the frame time the board reports (its pace, encoder and
                        // tx-air). It replaces CONV_FRAMES_PS (90 frames/s, sized when the board
                        // took 8.5 ms a frame), which held MCS0-2 at 1666 kbps whatever the board
                        // could send, and the delivered-rate branch, which only crept 5 % above
                        // what the encoder had just made and left MCS5 at its 50 % floor (3.4 of
                        // 6.8 Mbps). Losses still pull it down: usable (BLER) here, air_scale
                        // (delivered/needed) below.
                        one_burst * usable / 1.35
                    } else if fabric_path && fb_fresh && deliv_rate > 1.0
                    {
                        // 0.78 x climb factor 1.35 = probe ~5 % above delivered (at 15 % MJPEG
                        // oscillates: one lost segment kills the frame)
                        deliv_rate * payload_bits * 0.78
                    } else {
                        phy_fps * usable * payload_bits * 0.85
                    };
                    // branch B: the air ceiling above is the ofdm64 PHY's own, nothing to add
                    let ceiling = if crate::phy64() {
                        one_burst
                    } else {
                        0.55 * mcs.phy_bitrate(crate::samp_hz()) as f32
                    };
                    // A budget that follows the measured rate locks itself: the encoder can only
                    // produce what the budget allows, so the measurement never exceeds the current
                    // level (stuck at 616 k on a channel good for 1.2 M, seen at 720p30/20 MHz).
                    // The "capture pipeline only takes ~12 frames/s" constraint went away once the
                    // engine covered 100 %; the real ceiling is one_burst (airtime by fps). Let the
                    // budget CLIMB +35 % per measurement window: geometric convergence to one_burst
                    // in seconds, and rising bler pulls usable down as before.
                    {
                        let mut st = shared.stats.lock().unwrap();
                        st.rc_air_scale = air_scale;
                        st.rc_one_burst = one_burst;
                        st.rc_ceiling = ceiling;
                        st.rc_budget = budget * 1.35;
                        st.rc_blocks_ps = phy_fps;
                    }
                    video_bitrate = (budget * 1.35)
                        .max(one_burst.min(ceiling) * 0.5)
                        .min(one_burst)
                        .min(ceiling)
                        .min(one_burst * air_scale) // ceiling by delivered/sent ratio
                        .clamp(60_000.0, if crate::phy64() { 30_000_000.0 } else { 8_000_000.0 })
                        as u32;
                    uncapped_bitrate = video_bitrate;
                    // v40.53: no more than the picture needs (`set maxkbps`, 0 = AUTO_BPP)
                    let pic_cap = if cfg.max_kbps > 0 {
                        cfg.max_kbps as f32 * 1000.0
                    } else {
                        aw as f32 * ah as f32 * (afps as f32).max(1.0) * AUTO_BPP
                    };
                    video_bitrate = video_bitrate.min(pic_cap.max(60_000.0) as u32);
                    // ... and no more than the ground decodes ("!cap" from it)
                    if let Some(k) = far_cap_kbps() {
                        video_bitrate = video_bitrate.min((k.max(60) * 1000) as u32);
                    }
                    fec_room = (uncapped_bitrate as f32 / video_bitrate.max(1) as f32 - 1.0).max(0.0);
                    if survival < 1.0 {
                        // v40.24 survival: the IDR must be short to get through at low MCS
                        video_bitrate =
                            ((video_bitrate as f32 * survival) as u32).max(60_000);
                    }
                    if first_frame {
                        log("worker: initializing h264 encoder");
                    }
                    let enc = match &mut encoder {
                        Some(e) => {
                            e.ensure(aw, ah, video_bitrate, afps);
                            e
                        }
                        None => match VideoEncoder::new(
                            aw,
                            ah,
                            video_bitrate,
                            afps,
                        ) {
                            Ok(e) => encoder.insert(e),
                            Err(e) => {
                                log(&format!("h264 encoder init failed: {e}"));
                                std::thread::sleep(Duration::from_secs(1));
                                continue;
                            }
                        },
                    };
                    // v40.47: the cheap repair first. Every long-term reference the receiver
                    // confirms has to go into the encoder - openh264 will only code a repair
                    // against a confirmed one - and a repair request is then served by coding
                    // the next picture against it instead of a whole keyframe. At MCS0 that is
                    // the difference between one frame of airtime and five.
                    let mut ltr_repair = false;
                    if cfg.ltr {
                        let pending: Vec<(u8, u16, u16, u16)> =
                            std::mem::take(&mut shared.feedback.lock().unwrap().ltr);
                        for (kind, id, marked, cur) in pending {
                            if kind == 0 {
                                if enc.ltr_marked_ok(u32::from(id), i32::from(marked)) {
                                    ltr_marks += 1;
                                    ltr_tries = 0; // the receiver is healthy again
                                    if ltr_marks <= 3 || ltr_marks % 20 == 0 {
                                        log(&format!(
                                            "LTR: the RX confirms reference {marked} (#{ltr_marks}, idr {id}) - a repair can use it"
                                        ));
                                    }
                                }
                            } else if ltr_tries < LTR_MAX_TRIES
                                && enc.request_ltr_recovery(u32::from(id), i32::from(marked), i32::from(cur))
                            {
                                ltr_repair = true;
                                if !bw_hold {
                                    lost_pics.push_back(Instant::now());
                                }
                                ltr_tries += 1;
                                ltr_repairs += 1;
                                ltr_last = Instant::now();
                                log(&format!(
                                    "LTR repair #{ltr_repairs} (try {ltr_tries}/{LTR_MAX_TRIES}): coding against the reference the RX holds (frame {marked}, it is at {cur}) instead of a keyframe"
                                ));
                            }
                        }
                        // a quiet spell means the last repair worked, or the receiver went away
                        // - either way the next break starts its own count
                        if ltr_tries > 0 && ltr_last.elapsed() >= Duration::from_secs(3) {
                            ltr_tries = 0;
                        }
                    } else {
                        shared.feedback.lock().unwrap().ltr.clear();
                    }
                    // RX asked for a recovery keyframe? Serve AT MOST one IDR per 700 ms. A webcam
                    // IDR spans many blocks and at low MCS is the easiest thing to lose on air;
                    // serving every request turned one lost frame into a self-feeding IDR flood
                    // (every frame an IDR, P-frames starving: the webcam "1 frame" incident at 13
                    // dB SNR). The request stays sticky in the feedback until it is really served.
                    let idr_ok = last_idr.map_or(true, |t| {
                        t.elapsed() >= Duration::from_millis(idr_backoff_ms)
                    });
                    let (force_idr, segs_now) = {
                        let mut fb = shared.feedback.lock().unwrap();
                        let resolved = fb.segs_ok >= idr_segs_at.saturating_add(5);
                        if idr_streak > 0 && !fb.need_idr && resolved {
                            idr_streak = 0;
                            idr_backoff_ms = 700;
                            if survival < 1.0 {
                                survival = 1.0;
                                log("idr: RX is decoding pictures - leaving survival mode, bitrate back to normal");
                            }
                        }
                        if ltr_repair {
                            // the repair IS the answer to the request: do not also spend a
                            // keyframe on it
                            fb.need_idr = false;
                            (false, fb.segs_ok)
                        } else {
                            (idr_ok && std::mem::take(&mut fb.need_idr), fb.segs_ok)
                        }
                    };
                    // v40.53 NO WAY BACK: with the feedback stale nothing the receiver asks for
                    // arrives - no NACK, no keyframe, no LTR repair - and the encoder's own GOP is
                    // 300 frames: one lost block froze the picture for up to 10 s. Bench 5/10,
                    // B's power 40 dB down (control dead, the video way A->B clean): 0.7 fps,
                    // "not recovered within 300 ms, asking for a keyframe" every few frames.
                    // A keyframe every `blindidr` ms while blind; not counted as a failed
                    // repair (nothing can confirm it), so no survival mode from it.
                    let blind_idr = !force_idr
                        && !fb_fresh
                        && last_idr.map_or(true, |t| t.elapsed() >= Duration::from_millis(u64::from(cfg.blind_idr_ms)));
                    if blind_idr {
                        idr_sent += 1;
                        blind_idrs += 1;
                        last_idr = Some(Instant::now());
                        if blind_idrs == 1 || blind_idrs % 30 == 0 {
                            log(&format!(
                                "idr: no feedback for 3 s - a keyframe every {} ms until it is back (#{blind_idrs})",
                                cfg.blind_idr_ms
                            ));
                        }
                    }
                    let force_idr = force_idr || blind_idr;
                    if force_idr && !blind_idr {
                        idr_sent += 1;
                        if !bw_hold {
                            lost_pics.push_back(Instant::now());
                        }
                        // no frame delivered since the previous IDR = that IDR did not arrive
                        let unresolved = last_idr.is_some()
                            && segs_now < idr_segs_at.saturating_add(5);
                        idr_streak = if unresolved { idr_streak + 1 } else { 1 };
                        idr_segs_at = segs_now;
                        idr_backoff_ms =
                            (700u64 << idr_streak.saturating_sub(1).min(2)).min(3000);
                        if idr_streak >= 3 {
                            survival = (survival * 0.5).max(0.25);
                        }
                        last_idr = Some(Instant::now());
                        log(&format!(
                            "IDR requested by RX (video loss recovery) streak={idr_streak}                              backoff={idr_backoff_ms}ms survival={survival:.2}"
                        ));
                    }
                    let t_enc = Instant::now();
                    let bytes = enc.encode(&f, force_idr);
                    let enc_ms = t_enc.elapsed().as_millis();
                    // v20.11 waterfall: an encode spike is the stall suspect
                    if enc_ms > 30 || frame_id % 8 == 0 {
                        log(&format!(
                            "wf enc id={frame_id} ms={enc_ms} idr={} len={}",
                            u8::from(force_idr), bytes.len()
                        ));
                    }
                    if bytes.is_empty() {
                        continue;
                    }
                    // v36 SIMULCAST BASE layer: 320x240@15 ~200 kbps PINNED at MCS0 (covered by the
                    // v34 1-block parity). Frame airtime is fixed at 713 µs, so it costs only ~4-6
                    // % of airtime; in return, when a fade hits, the RX still has a picture (falls
                    // back to base at the receiver, zero delay) while minstrel/AGC catch up. Only
                    // runs with txconv (the conv path is the one that can mix MCS per frame).
                    // v40.25c: base layer only while the main layer is at MCS >= 2: at MCS0/1 it is
                    // a copy of equal robustness costing 120 frames/s (36 % of the ceiling) and
                    // tearing the main layer itself.
                    let main_mcs_now =
                        crate::conv_tx::CUR_MCS.load(Ordering::Relaxed) as usize;
                    // v40.54: and while blind (`blindbase`)
                    let base_on = cfg.simulcast || (cfg.blind_base && !fb_fresh);
                    if cfg.txconv && base_on && (2..6).contains(&main_mcs_now) {
                        base_flip = !base_flip;
                        if base_flip {
                            // downsample nearest 2x2 (cheap; enough for the inset). v40.62: to even sizes -
                            // H.264 needs them: 854x480 halved to 427 panicked the encoder thread
                            // (openh264 "width needs to be a multiple of 2", 11/10 - the link went
                            // blind, `blindbase` turned the base layer on at the 854x480 rung, and
                            // the video stopped for good)
                            let bw = (f.width / 2) & !1;
                            let bh = (f.height / 2) & !1;
                            if enc_base_dims != (bw, bh) {
                                enc_base = None;
                                enc_base_dims = (bw, bh);
                            }
                            let mut rgb =
                                Vec::with_capacity(bw * bh * 3);
                            for y in 0..bh {
                                let row = y * 2 * f.width;
                                for x in 0..bw {
                                    let i = (row + x * 2) * 3;
                                    rgb.extend_from_slice(&f.rgb[i..i + 3]);
                                }
                            }
                            let bf = nyx_common::RgbFrame {
                                width: bw, height: bh, rgb,
                            };
                            let be = enc_base.get_or_insert_with(|| {
                                nyx_common::codec::VideoEncoder::new_with_intra(
                                    bw, bh, 200_000, 15.0, 30, // IDR 2s
                                )
                                .expect("enc base")
                            });
                            let bb = be.encode(&bf, false);
                            if !bb.is_empty() {
                                base_id = base_id.wrapping_add(1);
                                for blk in crate::packetize_fec_base(
                                    base_id, &bb,
                                ) {
                                    // seq SHARED with the main layer: seq numbers the LINK PATH;
                                    // two separate ranges blew up the RX's seq-gap counter (14393
                                    // "lost" in 30 s -> NACK/IDR storm wrecking the main layer;
                                    // soak test). v40: the base layer's pinned rung: 0 (QPSK 1/2)
                                    // or 6 (the repeat step, +3 dB reach).
                                    seq = seq.wrapping_add(1);
                                    convtx.push_mcs(seq, &blk, Some(cfg.base_mcs));
                                }
                            }
                        }
                    }
                    (SourceType::H264, bytes, Some(f))
                }
                Codec::Mjpeg => {
                    // v5: rate control for MJPEG: byte budget per picture from the MEASURED goodput
                    // (block_roll) x (1 - bler), adjusting quality gradually (down fast, up slow);
                    // cfg.jpeg_quality becomes a CEILING instead of a fixed value.
                    let phy_fps = block_roll.per_sec().max(1.0);
                    let usable = (1.0 - fb.bler).clamp(0.3, 1.0);
                    let budget_bytes = (phy_fps * usable
                        * FRAME_PAYLOAD_BYTES as f32 * 0.85
                        / afps.max(1.0))
                        .max(FRAME_PAYLOAD_BYTES as f32 * 0.8);
                    let jpeg = encode_jpeg(&f, mj_quality as u8);
                    let sz = jpeg.len() as f32;
                    if sz > budget_bytes {
                        mj_quality -= (4.0 * sz / budget_bytes).min(12.0);
                    } else if sz < 0.7 * budget_bytes {
                        mj_quality += 1.5;
                    }
                    mj_quality =
                        mj_quality.clamp(20.0, f32::from(cfg.jpeg_quality));
                    video_bitrate =
                        (sz * 8.0 * afps.max(1.0)) as u32;
                    (SourceType::Jpeg, jpeg, Some(f))
                }
            },
            None => {
                // v40.8: the DATA source is regulated like video. block_len used to be a FIXED 24
                // KB x fps = 5.9 Mbps of demand at 30 fps, ~6x the real capacity (measured with the
                // channel carrying ~0.5-1 Mbps): 13 blocks per frame, one lost fragment loses the
                // whole frame -> 6 % of frames produced a picture, 2 pictures in 40 s. It looked
                // like "the data path is broken" when the source was drowning itself. Follow the
                // REAL block rate (block_roll = blocks/s actually pushed on air; the send loop
                // sleeps by air_gap, so it is real airtime) + 35 % probing to climb back when the
                // channel improves, then CLAMP TO THE AIRTIME CEILING. Without the ceiling the 1.35
                // probe runs for ever: the first measurement had cq_drop climbing steadily at
                // ~100/s because it always offered 135 % of capacity. The ceiling depends on no
                // measurement, so it stays put at the physical limit. simulcast=false: the base
                // layer is made from video_frame; data mode has no base layer, so its 150 f/s share
                // is not subtracted.
                let payload_bits = (FRAME_PAYLOAD_BYTES * 8) as f32;
                let deliv_bps = block_roll.per_sec().max(1.0)
                    * payload_bits
                    * (1.0 - fb.bler).clamp(0.3, 1.0);
                let cap = if cfg.txconv {
                    conv_air_ceiling_bps(
                            cfg.air_gap_ms, false,
                            budget_rung, payload_bits, afps, cfg.fec,
                        )
                } else {
                    f32::MAX
                };
                datagen.block_len = (((deliv_bps * 1.35).min(cap)
                    / afps.max(1.0)
                    / 8.0) as usize)
                    .clamp(512, 24 * 1024);
                video_bitrate =
                    (datagen.block_len as f32 * 8.0 * afps.max(1.0)) as u32;
                {
                    // The gauge must LIVE in both modes: rc_blocks_ps used to be written only in
                    // the video branch, so in data mode it kept the old number -> the link looked
                    // like 85 blk/s when it was really doing 260. A frozen number is worse than
                    // none.
                    let mut st = shared.stats.lock().unwrap();
                    st.rc_blocks_ps = block_roll.per_sec();
                    st.rc_budget = deliv_bps * 1.35;
                    st.rc_one_burst = cap;
                }
                let t_d = Instant::now();
                let data = datagen.next_block();
                let d_us = t_d.elapsed().as_micros() as u64;
                let t_p = Instant::now();
                let preview = bytes_to_image(&data, 192);
                if frame_id % 8 == 0 {
                    log(&format!(
                        "datapace: gen={}us preview={}us",
                        d_us, t_p.elapsed().as_micros()
                    ));
                }
                (SourceType::RawData, data, Some(preview))
            }
        } };

        // v7: capacity for the ladder (read by the NEXT loop) = the video_bitrate the system just
        // converged to, already clamped by one_burst/ceiling/budget, self-consistent (resolution
        // does NOT affect video_bitrate, so no feedback loop). EMA so the rung does not shake; only
        // H264 has a video_bitrate that is a "true ceiling".
        if src_type == SourceType::H264 {
            // (the capacity before the picture cap, or the ladder would never climb past it)
            let vb = video_bitrate.max(std::mem::take(&mut uncapped_bitrate)) as f32 / 1000.0;
            link_cap_kbps = if link_cap_kbps < 1.0 {
                vb
            } else {
                0.8 * link_cap_kbps + 0.2 * vb
            };
        }

        // ---------------------------------------------- modulate + send
        if first_frame {
            log("worker: source ready, modulating first frame");
            // v40.22: lower the flag here: the conv path (default since v40) does not pass through
            // the "first PHY frame modulated" branch below, so the flag stayed up for ever: 3 log
            // lines EVERY FRAME (30/s, tx.log 244 MB/day).
            first_frame = false;
        }
        let t_pk = Instant::now();
        let (n_data, fec_k): (usize, usize);
        let mut blocks =
            {
                // v40.48: cut the picture to one frame per block at the rung it will go out on,
                // when the board compacts blocks (see BOARD_COMPACT)
                // (branch B: never - a block is one packet at any length there)
                let on = cfg.txconv && BOARD_COMPACT.load(Ordering::Relaxed) && !crate::phy64();
                COMPACT_ON.store(on, Ordering::Relaxed);
                let seg = if on {
                    nyx_proto::conv_seg_one_frame(crate::conv_tx::CUR_MCS.load(Ordering::Relaxed) as usize)
                } else {
                    nyx_link::SEG_PAYLOAD
                };
                // v40.53: as many parity blocks as the loss at this rung calls for (a keyframe
                // more), the first the XOR as ever
                let seg_eff = if seg >= nyx_link::SEG_PAYLOAD - 2 { nyx_link::SEG_PAYLOAD } else { seg.max(1) };
                n_data = app_data.len().div_ceil(seg_eff).max(1);
                fec_k = if !cfg.fec {
                    0
                } else if crate::phy64() && cfg.fec_rs {
                    // branch B: the ofdm64 PHY carries 3-10x what the picture needs, so every
                    // picture gets the parity its rung's measured loss calls for, never sized for
                    // under 3 % (WiFi takes 1.4-4.4 % of the packets on 2.4 GHz, measured 11/10 -
                    // with the XOR alone ~7 % of the pictures broke and the ladder never left
                    // 854x480): a keyframe up to half its blocks again, a picture up to a third
                    let key = matches!(src_type, SourceType::H264) && h264_is_key(&app_data);
                    let p = (PHY64_LOSS_Q16.load(Ordering::Relaxed) as f32 / 65535.0).max(0.03) * FEC_BURST;
                    let kmax = if key { n_data.div_ceil(2).max(2) } else { n_data.div_ceil(3).max(2) };
                    let k = nyx_link::parity_for(n_data, p.clamp(0.01, 0.6), 0.002, kmax);
                    if key || frame_id % 30 == 0 {
                        log(&format!(
                            "fec: phy64 {} {n_data} blocks + {k} parity (design loss {:.1} %)",
                            if key { "keyframe" } else { "picture" }, p / FEC_BURST * 100.0
                        ));
                    }
                    k
                } else if cfg.fec_rs && !fb_fresh && cfg.fec_blind > 0 {
                    // v40.54 ONE WAY: no retransmission, no keyframe on request, no loss measured -
                    // the parity is all the repair there is, sized for the design loss (`fecblind`)
                    // at whatever rung the picture goes out on. The keyframe is what ends a tear,
                    // so it gets up to half its blocks again; a picture the XOR and one more for
                    // every four (the receiver conceals a lost one: torn, not frozen, until the
                    // keyframe). Bench 8/10: up to as many parity blocks as data on every picture
                    // ("1 block + 2 parity") took the air of a third of the pictures (20 fps).
                    let key = matches!(src_type, SourceType::H264) && h264_is_key(&app_data);
                    let p = f32::from(cfg.fec_blind) / 100.0 * FEC_BURST;
                    let (target, kmax) = if key {
                        (FEC_TARGET_KEY, (n_data / 2).max(2))
                    } else {
                        (FEC_TARGET, (n_data / 4).max(1))
                    };
                    let k = nyx_link::parity_for(n_data, p.clamp(0.01, 0.6), target, kmax);
                    if key || frame_id % 30 == 0 {
                        log(&format!(
                            "fec: blind {} {n_data} blocks + {k} parity (design loss {} %)",
                            if key { "keyframe" } else { "picture" }, cfg.fec_blind
                        ));
                    }
                    k
                } else if !cfg.fec_rs
                    || FEC_LOSS_Q16.load(Ordering::Relaxed) == 0
                    || crate::conv_tx::CUR_MCS.load(Ordering::Relaxed) != FEC_RUNG.load(Ordering::Relaxed)
                {
                    // the XOR alone outside the interference regime (see FEC_LOSS_Q16)
                    1
                } else {
                    let key = matches!(src_type, SourceType::H264) && h264_is_key(&app_data);
                    // a picture that is not a keyframe gets more than the XOR only above MCS0 and
                    // only into the air the picture cap leaves (fec_room): where the air is what
                    // the video is short of, parity takes it from the pictures - on the bench under
                    // WiFi a second and third parity block on every picture left the video on the
                    // repeat step at 207 kbps, 166 pictures lost a minute where the XOR alone lost 28
                    let rung = crate::conv_tx::CUR_MCS.load(Ordering::Relaxed);
                    // v40.59 (`fecsmall`): down to MCS0 and up to half the picture again (two at
                    // least) - see Config::fec_small
                    let (kmax_p, rung_ok) = if cfg.fec_small {
                        (n_data.div_ceil(2).max(2), rung <= 5)
                    } else {
                        ((1 + (n_data as f32 * fec_room) as usize).min(2), (1..=5).contains(&rung))
                    };
                    if !key && (!rung_ok || kmax_p <= 1) {
                        1
                    } else {
                        let p = FEC_LOSS_Q16.load(Ordering::Relaxed) as f32 / 65535.0 * FEC_BURST;
                        let (target, kmax) = if key { (FEC_TARGET_KEY, (n_data / 4).max(2)) } else { (FEC_TARGET, kmax_p) };
                        let k = nyx_link::parity_for(n_data, p.clamp(0.01, 0.6), target, kmax);
                        if key || frame_id % 30 == 0 {
                            log(&format!(
                                "fec: {} {n_data} blocks + {k} parity (block loss {:.0} %)",
                                if key { "keyframe" } else { "picture" }, p / FEC_BURST * 100.0
                            ));
                        }
                        k
                    }
                };
                let extra = fec_k.saturating_sub(1) as f32;
                let e = FEC_EXTRA_Q8.load(Ordering::Relaxed) as f32 / 256.0;
                FEC_EXTRA_Q8.store(((0.9 * e + 0.1 * extra) * 256.0) as u32, Ordering::Relaxed);
                nyx_link::packetize_rs(frame_id, src_type, &app_data, seg, fec_k)
            };
        // v40.45c: the video parity block - the XOR, right after the data blocks (before the
        // text/data/info frames are appended); under interference it goes out twice.
        let parity_idx = if fec_k >= 1 && blocks.len() > n_data { Some(n_data) } else { None };
        if frame_id % 8 == 0 {
            log(&format!("pkpace: packetize={}us", t_pk.elapsed().as_micros()));
        }
        // v12: text messages ride along: each message is one small frame (a block describes its own
        // frame_id + src, so it takes the identical pipeline/ARQ).
        {
            let mut q = shared.text_out.lock().unwrap();
            for t in q.drain(..) {
                frame_id = frame_id.wrapping_add(1);
                blocks.extend(packetize(frame_id, SourceType::Text, t.as_bytes()));
            }
        }
        // v40.22: user datagrams (UDP --tlm-in): each datagram one Data frame of its own, same
        // trip/ARQ as text.
        {
            let mut q = shared.tlm_tx_q.lock().unwrap();
            for d in q.drain(..) {
                frame_id = frame_id.wrapping_add(1);
                blocks.extend(packetize(frame_id, SourceType::Data, &d));
            }
        }
        // v40.33: this board's licence state for the far app, every 3 s (one block)
        if lic_info_at.elapsed() >= Duration::from_secs(3) {
            lic_info_at = Instant::now();
            let mut t = shared.lic_info.lock().unwrap().clone();
            // camera pass-through 18/9: tell the receiving end that this is a camera's own H.264. It cannot ask
            // for a key frame here (the camera decides), so a hole given up on costs the picture
            // until the camera's next one - the receiver waits longer for a retransmission when it
            // knows that, and knows how long the wait is worth (`cam_gop_ms`).
            if pass {
                let fps = shared.rtsp.fps_x10.load(Ordering::Relaxed) as f32 / 10.0;
                let gop = shared.rtsp.gop.load(Ordering::Relaxed);
                let gop_ms = if fps > 1.0 && gop > 0 { (gop as f32 / fps * 1000.0) as u32 } else { 0 };
                t.push_str(&format!("video=pass\ncam_gop_ms={gop_ms}\n"));
            }
            if !t.is_empty() {
                frame_id = frame_id.wrapping_add(1);
                blocks.extend(packetize(frame_id, SourceType::Info, t.as_bytes()));
            }
        }
        let n_blocks = blocks.len();
        {
            // v22: blocks PER FRAME + source bytes, to tell whether low throughput is a source with
            // few bytes or the send loop dropping blocks.
            let mut st = shared.stats.lock().unwrap();
            st.rc_blocks_frame = n_blocks as f32;
            st.rc_src_bytes = app_data.len() as f32;
        }
        // Blocks of ONE video frame ride a single FPGA capture when they
        // fit: the receiver demodulates every frame it finds in a window,
        // so a back-to-back burst arrives atomically (no capture-shadow
        // loss between an IDR's blocks). Only when the burst would overrun
        // the capture does the next block wait out the shadow.
        let mut burst_samples = 0usize;
        let pair_ok = cfg.pair
            && cfg.txbits
            && mcs.data_syms_per_frame() <= 9;
        let mut bi = 0usize;
        while bi < blocks.len() {
            let block = &blocks[bi];
            if cfg.txconv {
                seq = seq.wrapping_add(1);
                if pass {
                    // a camera frame cannot lose a block; the reader drops whole frames by age
                    convtx.push_wait(seq, block);
                } else {
                    convtx.push(seq, block);
                }
                if cfg.dup && interf_on && Some(bi) == parity_idx {
                    dup_q.push_back((Instant::now() + DUP_GAP, seq, *block));
                }
                // v37.1: load the ARQ cache on the conv path TOO. Before, only the legacy path
                // (below) loaded it, so conv mode answered every NACK from an empty cache: nack > 0
                // / retx == 0 FOR EVER, a block lost on air was never repaired. On the cable (loss
                // ~0) the disease was hidden; on antennas ~8 blocks/s lost meant the main GOP broke
                // continuously.
                if cfg.arq {
                    cache.push_back(CacheEntry {
                        seq, block: *block, mcs, retx: 0,
                    });
                    while cache.len() > ARQ_CACHE {
                        cache.pop_front();
                    }
                }
                bi += 1;
                continue;
            }
            // v6: a pair = ONE message: block 2's words (with its own MAGIC) follow block 1's
            // IMMEDIATELY. Sent as two messages, the daemon pads each packet to the 8192-sample DMA
            // block boundary, the parser swallows zeros one sample per tick -> ~7 dead symbols
            // between the two frames, frame 2 falls off the capture grid (measured on air: SIG2
            // scan 0 hits).
            let pair_lead = pair_ok && bi + 1 < n_blocks;
            seq += 1;
            if first_frame {
                log("worker: first PHY frame modulated, sending");
                first_frame = false;
            }
            // v3.1: the BIT path for the fabric modulator: ~2-3 KB of words instead of ~184 KB of
            // IQ; the burst/gap bookkeeping stays in terms of LENGTH ON AIR (the fabric transmits
            // the same number of samples although we only send bits).
            let w_len = mcs.frame_len_samples()
                * if pair_ok && bi + 1 < n_blocks { 2 } else { 1 };
            // txbits only for frames within the fabric modulator's 16-symbol limit (5-bit header +
            // 16-symbol uram, like the RX grid): MCS3+. Low MCS (long frames) falls back to IQ by
            // itself; without this guard MCS0's nsym 36 wrapped in 5 bits to 4, the frame went out
            // truncated, bler 100 %, OLLA stuck at the bottom for good (measured on air). v6: a
            // PAIR always takes the TxBits path (PC encode): the fabric encoder codes packets one
            // at a time (single bank), the two packets end up dozens of symbols apart and frame 2
            // falls off the grid (measured: SIG2 0 hits with txpl). Single blocks use txpl as
            // usual.
            let t_e = Instant::now();
            let msg = if false {
                unreachable!()
            } else if cfg.txpl
                && cfg.txbits
                && !pair_lead
                && mcs.data_syms_per_frame() <= 16
            {
                // v4.3: PAYLOAD for the fabric encoder (~2 KB, the PC does not encode). v40.40: the
                // raw block goes to the board, its encoder does the FEC
                iq_roll.push(block.len());
                Msg::TxBlock {
                    seq,
                    mcs: mcs.index() as u8,
                    rv: 0,
                    flags: TXB_FABRIC,
                    payload: block.to_vec(),
                }
            } else if cfg.txbits && mcs.data_syms_per_frame() <= pl || !PcTx::AVAILABLE {
                // v40.40: raw block(s); the daemon encodes and joins the pair
                let mut payload = block.to_vec();
                let mut flags = 0u8;
                if pair_lead {
                    seq += 1;
                    payload.extend_from_slice(&blocks[bi + 1]);
                    flags |= TXB_PAIR;
                }
                iq_roll.push(payload.len());
                Msg::TxBlock { seq, mcs: mcs.index() as u8, rv: 0, flags, payload }
            } else {
                let wave = phy.modulate_frame(block, mcs, 0, seq as u8);
                // DAC model: 16-bit at -12 dBFS back-off, hard saturation.
                let (wave16, clips) =
                    nyx_proto::quantize(&wave, nyx_proto::DAC_SCALE);
                dac_clips += clips;
                iq_roll.push(wave16.len() * 4);
                Msg::IqFrame { seq, mcs: mcs.index() as u8, rv: 0, samples: wave16 }
            };
            tp_enc += t_e.elapsed().as_micros() as u64;
            // v5: fabric path: NO bursts (frame 2+ in a capture is invisible to the fabric demod);
            // every block waits the full air gap.
            let fabric_tx =
                cfg.txbits && mcs.data_syms_per_frame() <= pl;
            if fabric_tx
                || burst_samples == 0
                || burst_samples + w_len > BURST_BUDGET
            {
                // fabric: gap = the RX capture cadence (back-pressure throttles the video fps
                // instead of flooding and leaving ARQ to clean up)
                let gap_ms = if fabric_tx {
                    cfg.air_gap_ms.max(if pair_lead {
                        FAB_GAP_PAIR_MS
                    } else {
                        FAB_GAP_SINGLE_MS
                    })
                } else {
                    cfg.air_gap_ms
                };
                let min_gap =
                    Duration::from_micros((gap_ms * 1000.0) as u64);
                let gap = min_gap.saturating_sub(last_air.elapsed());
                if !gap.is_zero() {
                    let t_g = Instant::now();
                    std::thread::sleep(gap);
                    tp_gap += t_g.elapsed().as_micros() as u64;
                }
                burst_samples = 0;
            }
            // v20.11: last_air is set BEFORE the send: spacing = send-to-send. Set after, the 15 ms
            // gap + 8-50 ms send (TCP/back-pressure) counted TWICE as 23-65 ms/msg -> the rate
            // halved (measured txpace).
            last_air = Instant::now();
            let ok = net.send(&shared, &msg);
            sent_blocks += if pair_lead { 2 } else { 1 };
            tp_send += last_air.elapsed().as_micros() as u64;
            tp_msgs += 1;
            burst_samples += w_len;
            if !ok {
                break;
            }
            // PROACTIVE REPETITION: resend the message (SAME seq) rep-1 times, one air-gap apart ->
            // time diversity. The RX dedups by seq (delivered_seqs) -> takes the FIRST copy that
            // decodes, drops the repeats; if noise or a fade burst kills one copy the other
            // survives. NO NACK round trip needed (unlike reactive ARQ). rep=1 (default) = empty
            // range = off. The price: N x airtime per frame.
            for _ in 1..cfg.rep_count.clamp(1, 8) {
                let g = Duration::from_micros((cfg.air_gap_ms * 1000.0) as u64)
                    .saturating_sub(last_air.elapsed());
                if !g.is_zero() {
                    std::thread::sleep(g);
                }
                last_air = Instant::now();
                if !net.send(&shared, &msg) {
                    break;
                }
                tp_msgs += 1;
                burst_samples += w_len;
            }
            if cfg.arq {
                if pair_lead {
                    // pair: cache BOTH blocks (seq-1 = the leading block)
                    cache.push_back(CacheEntry {
                        seq: seq - 1, block: *block, mcs, retx: 0,
                    });
                    cache.push_back(CacheEntry {
                        seq, block: blocks[bi + 1], mcs, retx: 0,
                    });
                } else {
                    cache.push_back(CacheEntry {
                        seq, block: *block, mcs, retx: 0,
                    });
                }
                while cache.len() > ARQ_CACHE {
                    cache.pop_front();
                }
            }
            bi += if pair_lead { 2 } else { 1 };
        }
        {
            // blocks of this frame ACTUALLY sent (bi = the cursor after the while loop); a
            // difference from n_blocks means the send loop dropped some.
            let mut st = shared.stats.lock().unwrap();
            st.rc_blocks_sent = bi as f32;
        }
        // ---- 1 s window: delivered/sent ratio -> bitrate ceiling regulation ----
        if rc_win.elapsed() >= Duration::from_secs(1) {
            let fbs = shared.feedback.lock().unwrap().clone();
            let fresh =
                fbs.updated.is_some_and(|t| t.elapsed().as_secs_f32() < 1.5);
            if fresh {
                // The conv path NO LONGER goes through sent_blocks (blocks go straight into the
                // conv_tx queue). Count with CQ_IN, the same BLOCK UNIT as the feedback's segs_ok.
                // Before the fix sent_cur stayed 0, the deliv/sent ratio was meaningless, air_scale
                // fell straight to the 0.25 floor and pinned vbr at 308 kbps (one_burst 1234 x
                // 0.25): throughput throttled 4x by one division with the wrong unit.
                let sent_cur = if cfg.txconv {
                    crate::conv_tx::CQ_IN.load(Ordering::Relaxed)
                        .saturating_sub(rc_sent0) as f32
                } else {
                    sent_blocks.saturating_sub(rc_sent0) as f32
                };
                // v40.35: blocks the ARQ sent AGAIN are not new demand, blocks the
                // conv queue dropped are demand that never flew - both belong in
                // the denominator of "how much of what the video needed arrived".
                let retx_cur = retransmits.saturating_sub(rc_retx0) as f32;
                let drop_cur = crate::conv_tx::CQ_DROP.load(Ordering::Relaxed)
                    .saturating_sub(rc_drop0) as f32;
                let orig_cur = (sent_cur - retx_cur).max(0.0) + drop_cur;
                if let Some(ok0) = rc_ok0 {
                    let deliv = fbs.segs_ok.saturating_sub(ok0) as f32;
                    // v27: in phase: blocks delivered in this window include ~30 % sent in the
                    // PREVIOUS one (feedback delay); the EMA removes the rest of the shake.
                    let sent_eff = 0.7 * orig_cur + 0.3 * rc_sent_prev;
                    if sent_eff > 4.0 {
                        // v40.34: throttle on the delivery probability of the rate the
                        // video rides on (probe blocks at higher rates and the base
                        // layer excluded) - the all-blocks ratio counted every failed
                        // probe against the video and pinned the bitrate at 25 %.
                        // v40.35: with ARQ a lost block is retransmitted, so the
                        // yardstick is ARQ-CORRECTED delivery: blocks that arrived
                        // (first try or retransmitted) over original blocks + queue
                        // drops. A 4 % air loss reads 0.998, not 0.96, and the
                        // bitrate no longer sits at the 25 % floor because of it;
                        // the ratio only falls when retransmissions fail too
                        // (fade / overload), which is when throttling helps.
                        let cur = auto_mcs.index();
                        let r = if cfg.arq {
                            (deliv / sent_eff).clamp(0.0, 1.5)
                        } else if cfg.auto_mcs
                            && mn_prob[cur] >= 0.0
                            && mn_age[cur].elapsed().as_secs_f32() < 3.0
                        {
                            mn_prob[cur]
                        } else {
                            (deliv / sent_eff).clamp(0.0, 1.5)
                        };
                        // v40.53: a window that delivered NOTHING is an outage (the far end out of
                        // reach, a blockage), not a rate the air cannot carry: no cut. Cutting
                        // x0.9 a second took the ceiling to its 25 % floor in a 13 s outage, and
                        // the climb back at x1.05 a second held the video at 108 kbps on a clean
                        // MCS1 for half a minute after it (bench 5/10, A -36 -> -40 dB).
                        let outage = deliv < 1.0 || bw_hold;
                        if !outage {
                            r_ema = 0.5 * r_ema + 0.5 * r;
                        }
                        // (A faster climb after clean windows, x1.15, was tried with it: in the
                        // simulator's fade it overshot the MCS0 / repeat-step ceiling, 1091 blocks
                        // dropped from the queue, 10 % fewer pictures.)
                        if outage {
                        } else if r_ema < 0.92 {
                            air_scale = (air_scale * 0.90).max(0.25);
                        } else if r_ema > 0.96 {
                            air_scale = (air_scale * 1.05).min(1.0);
                        }
                        if rc_log_at.elapsed() >= Duration::from_secs(5) {
                            rc_log_at = Instant::now();
                            log(&format!(
                                "rc: delivered/needed={r:.2} ema={r_ema:.2} -> ceiling {:.0}% (vbr {} kbps) deliv={deliv} needed={sent_eff:.1} retx={retx_cur} drop={drop_cur} mn={:?} fb_age={:.2}",
                                air_scale * 100.0,
                                video_bitrate / 1000,
                                mn_prob.iter().map(|p| (p * 100.0).round() as i32).collect::<Vec<_>>(),
                                fbs.updated.map_or(-1.0, |t| t.elapsed().as_secs_f32())
                            ));
                        }
                    }
                }
                rc_sent_prev = orig_cur;
                rc_retx0 = retransmits;
                rc_drop0 = crate::conv_tx::CQ_DROP.load(Ordering::Relaxed);
                rc_ok0 = Some(fbs.segs_ok);
                rc_sent0 = if cfg.txconv {
                    crate::conv_tx::CQ_IN.load(Ordering::Relaxed)
                } else {
                    sent_blocks
                };
                rc_win = Instant::now();
            } else {
                rc_ok0 = None;
                rc_win = Instant::now();
            }
        }
        // End-to-end latency mark: TX and RX run on the same machine, so the timestamps of the two
        // logs compare directly; match by frame_id (LAT tx/rx).
        if frame_id % 8 == 0 {
            log(&format!("LAT tx id={frame_id}"));
        }

        // v40.45c: second copies whose time has come (same seq: the RX dedups)
        if cfg.txconv {
            while let Some((due, _, _)) = dup_q.front() {
                if Instant::now() < *due {
                    break;
                }
                let (_, dseq, dblk) = dup_q.pop_front().unwrap();
                convtx.push(dseq, &dblk);
                dups_sent += 1;
            }
            if !interf_on {
                dup_q.clear();
            }
        }
        // --------------------------------------------- ARQ retransmits
        {
            let nack_rx = net.nacks.lock().unwrap();
            // camera pass-through 18/9: a camera passed through pays more for a lost frame than an encoder does
            // (no key frame on request, so the smear lasts until the camera's own), and a retry is
            // one block: spend the larger budget on it as under interference.
            let max_retx = if interf_on || pass { MAX_RETX_INTERF } else { MAX_RETX };
            while let Ok(nseq) = nack_rx.try_recv() {
                nacks_handled += 1;
                if !cfg.arq || (cfg.txconv && low_rung()) {
                    nacks_low += u64::from(cfg.txconv && low_rung());
                    continue;
                }
                // Match by the sequence LSB, newest first. The RX can only
                // see 8 seq bits over the air (SIG field) and reconstructs
                // the rest by counting — if TX and RX didn't start together
                // its absolute numbers sit at a different multiple of 256
                // and an exact-match lookup never hits (nack>0, retx==0
                // forever). The LSB is unambiguous within the 256-deep
                // cache, so match on it and prefer the most recent entry.
                if let Some(entry) = cache
                    .iter_mut()
                    .rev()
                    .find(|e| e.seq & 0xFF == nseq & 0xFF)
                    // v40.53: the newest block with that sequence, and no Reed-Solomon row: lost
                    // as the link fades, its retry goes at the rung the rate control fled to (five
                    // frames a block at MCS0) for a row the data blocks' own retries make useless
                    .filter(|e| e.retx < max_retx && !nyx_link::is_rs_parity(&e.block))
                {
                    entry.retx += 1;
                    if cfg.txconv {
                        // v37.1: the conv path retransmits through the SAME conv_tx queue, OLD seq
                        // (the RX dedups by seq), at minstrel's CURRENT MCS; no need to remember
                        // the old rung.
                        convtx.push_retx(entry.seq, &entry.block);
                        retransmits += 1;
                        if cfg.dup && interf_on {
                            dup_q.push_back((Instant::now() + DUP_GAP, entry.seq, entry.block));
                        }
                        continue;
                    }
                    // 5G-style IR: same MCS, next redundancy version — the
                    // retransmission carries fresh parity from the rate-1/3
                    // mother code's circular buffer, and the receiver
                    // accumulates everything in one soft buffer.
                    // Chase mode: repeat rv0 (pure energy combining).
                    let rv = if cfg.harq_ir {
                        RV_SEQUENCE
                            [(entry.retx as usize).min(3)]
                    } else {
                        0
                    };
                    let rmsg = if cfg.txpl
                        && cfg.txbits
                        && entry.mcs.data_syms_per_frame() <= 16
                    {
                        iq_roll.push(entry.block.len());
                        Msg::TxBlock {
                            seq: nseq,
                            mcs: entry.mcs.index() as u8,
                            rv,
                            flags: TXB_FABRIC,
                            payload: entry.block.to_vec(),
                        }
                    } else if cfg.txbits && entry.mcs.data_syms_per_frame() <= pl
                        || !PcTx::AVAILABLE
                    {
                        iq_roll.push(entry.block.len());
                        Msg::TxBlock {
                            seq: nseq,
                            mcs: entry.mcs.index() as u8,
                            rv,
                            flags: 0,
                            payload: entry.block.to_vec(),
                        }
                    } else {
                        let wave = phy.modulate_frame(
                            &entry.block, entry.mcs, rv, nseq as u8,
                        );
                        let (wave16, clips) =
                            nyx_proto::quantize(&wave, nyx_proto::DAC_SCALE);
                        dac_clips += clips;
                        iq_roll.push(wave16.len() * 4);
                        Msg::IqFrame {
                            seq: nseq,
                            mcs: entry.mcs.index() as u8,
                            rv,
                            samples: wave16,
                        }
                    };
                    retransmits += 1;
                    let gap = Duration::from_micros(
                        (cfg.air_gap_ms.max(FAB_GAP_SINGLE_MS) * 1000.0)
                            as u64,
                    )
                    .saturating_sub(last_air.elapsed());
                    if !gap.is_zero() {
                        std::thread::sleep(gap);
                    }
                    net.send(&shared, &rmsg);
                    last_air = Instant::now();
                }
            }
        }

        // -------------------------------------------------- UI + logs
        fps_roll.push(1);
        block_roll.push(n_blocks);
        if let Some(p) = preview {
            publish_preview(&shared, p);
        }
        let tx_fps = fps_roll.count_per_sec();
        let iq_mbps = iq_roll.per_sec() * 8.0 / 1e6;
        {
            let mut st = shared.stats.lock().unwrap();
            st.active_mcs = Some(mcs);
            st.rep2 = rep2;
            st.seq = seq;
            st.blocks_per_frame = n_blocks;
            st.app_bytes_per_frame = app_data.len();
            st.tx_fps = tx_fps;
            st.iq_mbps = iq_mbps;
            st.nacks_handled = nacks_handled;
            st.retransmits = retransmits;
            // v40.8: data mode now has a real target cadence too (it had no regulation before,
            // hence no number to report) -> report it, otherwise console/`stats` show vbr=0 and the
            // TX looks mute.
            st.video_bitrate_bps =
                if matches!(src_type, SourceType::H264 | SourceType::RawData) {
                    video_bitrate
                } else {
                    0
                };
            st.idr_sent = idr_sent;
        }
        status.tick(&format!(
            "status | src={:?} codec={:?} {}x{}@{:.0} vbr={}kbps idr={} mcs={} seq={} \
             blocks/frame={} tx_fps={:.1} iq={:.1}Mbps nack={} retx={} nack_low={} dup={}              ltr={}/{} \
             fb_snr={:.1}dB fb_bler={:.1}% olla={:.1}dB dac_clip={}",
            cfg.source,
            cfg.codec,
            aw,
            ah,
            afps,
            video_bitrate / 1000,
            idr_sent,
            if rep2 { "QPSK 1/2 x2 (rung 6)" } else { mcs.label() },
            seq,
            n_blocks,
            tx_fps,
            iq_mbps,
            nacks_handled,
            retransmits,
            nacks_low,
            dups_sent,
            ltr_marks,
            ltr_repairs,
            if fb_fresh { fb.snr_db } else { f32::NAN },
            if fb_fresh { fb.bler * 100.0 } else { f32::NAN },
            olla_margin_db,
            dac_clips,
        ));
        // v20.11 chan doan rate TX: gap-sleep vs net.send (us/msg)
        if tp_msgs > 0 && frame_id % 4 == 0 {
            // v22: expose msgs/s in stats, to separate "the app is not sending enough" from
            // "sending enough but losing on the way down to the daemon".
            {
                let dt = tp_win.elapsed().as_secs_f32().max(0.001);
                let mut st = shared.stats.lock().unwrap();
                st.rc_msgs_ps = tp_msgs as f32 / dt;
                st.rc_enc_us = (tp_enc / tp_msgs) as f32;
                st.rc_gap_us = (tp_gap / tp_msgs) as f32;
                st.rc_send_us = (tp_send / tp_msgs) as f32;
                st.rc_loop_us = dt * 1e6 / tp_msgs as f32;
                drop(st);
                tp_win = Instant::now();
            }
            log(&format!(
                "txpace: msgs={} enc_avg={}us gap_avg={}us send_avg={}us",
                tp_msgs, tp_enc / tp_msgs, tp_gap / tp_msgs, tp_send / tp_msgs
            ));
            tp_gap = 0;
            tp_send = 0;
            tp_enc = 0;
            tp_msgs = 0;
        }

        frame_id = frame_id.wrapping_add(1);
    }
}

fn publish_preview(shared: &Shared, frame: RgbFrame) {
    *shared.preview.lock().unwrap() = Some(frame);
    shared.preview_version.fetch_add(1, Ordering::Relaxed);
}
