//! The conv transmit path, SPLIT OFF from the capture/encode loop.
//!
//! WHY A REWRITE: the old send loop carried the LDPC/IQ-era legacy (air_scale, LADDER, per_capture,
//! BURST_BUDGET, pair, ARQ, rep) and sent INSIDE the capture loop, so every block cost a whole
//! frame period. Measured on the cable: the webcam produced 15.9 KB/frame (9 blocks) yet the
//! modulator sent EXACTLY 40/s, flat at every resolution: a hard 25 ms cadence, not the content.
//! The old counters even contradicted each other (blocks_sent=14 while blocks_frame=9), so they
//! could no longer be used to trace anything.
//!
//! PRINCIPLE: exactly ONE clock, and counters at exactly TWO points: block into the queue (`CQ_IN`)
//! and message written to the socket (`CQ_OUT`). Compare directly with the daemon's `tx_frames` and
//! the modulator's `frames`: three numbers, one truth. Nowhere else may a block be dropped.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use nyx_common::logging::log;
use nyx_proto::{Msg, TXB_CONV, TXB_FILV, TXB_RUNG_FREE};

use crate::net::Net;
use crate::Shared;

/// The MCS rung in use, published by the worker (OLLA chooses it when `auto` is on). The transmit
/// path runs on its own thread and cannot see the frame loop's locals.
pub static CUR_MCS: AtomicU64 = AtomicU64::new(5);
// v32 minstrel: the worker sets a probe rung (0xFF = none); every PROBE_EVERY-th block goes out at
// that rung so the NEIGHBOUR's prob statistics stay fresh without betting the whole stream (the
// minstrel/mac80211 principle). v40.49: two rungs, low byte and the byte above it; the probe
// blocks alternate between them (0xFF in the second = the first one only).
pub static PROBE_MCS: AtomicU64 = AtomicU64::new(0xFFFF);
/// v40.57: CUR_MCS is the per-channel rung (worker.rs): blocks that follow it go out marked
/// TXB_RUNG_FREE, so the board may send them at the rung of the channel they really go out on
pub static CHAN_RUNG_ON: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub static PROBE_EVERY: AtomicU64 = AtomicU64::new(12);
/// BLOCKS sent per MCS actually used (probes included); the worker divides ok_mcs (from feedback)
/// by this for a per-rate prob, on the SAME window. v40: 7 entries: rung 6 (the repeat step) is
/// counted separately for inspection.
pub static SENT_MCS: [AtomicU64; 7] = [
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0),
];

/// Branch B: spread a picture's blocks (SLOW_MS apart, over SPREAD_MS) only while the rate control
/// is in the interference regime (the worker sets it every frame). On a clean channel spreading
/// buys nothing and costs its whole width in latency: 14 blocks of a 720p picture 2 ms apart were
/// 28 of the 28 ms from the encoder to the ground app (17/10, 2300 MHz, ARQ on or off alike).
pub static SPREAD_ON: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Blocks placed in the transmit queue.
pub static CQ_IN: AtomicU64 = AtomicU64::new(0);
/// Messages written to the socket.
pub static CQ_OUT: AtomicU64 = AtomicU64::new(0);
/// Blocks dropped because the queue was full (overload): the ONLY place a loss is allowed.
pub static CQ_DROP: AtomicU64 = AtomicU64::new(0);
/// The queue is full at this much air, not at a number of blocks. 128 blocks was 192 ms at
/// MCS0 (compact 424-byte blocks, 1.5 ms a frame) - less than one keyframe once the budget
/// grew with a faster board: 40-55 KB is 97-130 blocks, its head was dropped, and the pictures
/// broke more the faster the board was paced (txpace 1500: 243-273 blocks dropped in 25 s, 30-50
/// bad pictures, against 0 and 3-5 at 2000; 4/10). The daemon bounds its own queue the same way.
const CQ_MAX_AIR_US: u64 = 300_000;
/// ...and this many blocks, a backstop.
const CQ_MAX_BLOCKS: usize = 1024;

/// What `block` takes on air at the rung it will go out on, microseconds.
fn air_us(block: &[u8], pin: Option<usize>) -> u64 {
    let mcs = pin.unwrap_or(CUR_MCS.load(Ordering::Relaxed) as usize).min(6);
    if crate::phy64() {
        // branch B: one packet, its own time
        return nyx_proto::phy64_air_us(mcs, block.len()) as u64;
    }
    let nfrag = nyx_proto::conv_frames_block(mcs, block, crate::worker::BOARD_COMPACT.load(Ordering::Relaxed));
    let frame_us = match crate::worker::BOARD_FRAME_US.load(Ordering::Relaxed) {
        0 => 3_000,
        us => us,
    };
    nfrag as u64 * frame_us
}
/// v37: BASE-layer blocks queued, so the worker can separate r/prob per stream (base always gets
/// through, so mixing them masks the main layer's death; measured: MCS held at 5 at -9 dB).
pub static BASE_IN: AtomicU64 = AtomicU64::new(0);

/// First byte of every frame: (frag_idx << 4) | frag_cnt. Thanks to it EVERY MCS carries a 2016 B
/// block; the low rates just need more frames.
///
/// WHY: payload per frame by MCS is 447/672/897/1347/1797/2022 B while the application block is
/// fixed at 2016 B, so BEFORE this only MCS5 could carry a block, i.e. the link had no rung to fall
/// back to when the air got bad, although MCS5 is the most marginal rate (measured on the cable:
/// 1.6 % errors at MCS5 against 0.2-0.5 % at the rates below).
///
/// The PN scrambling is applied AFTER the header is attached, so the header is protected too; the
/// RX descrambles the whole frame before reading the header. The CRC16 is computed last by
/// tx_frame_words_conv, so the fabric knows nothing about fragmentation.
const FRAG_HDR: usize = 1;
// v40: 12 (possible, the header nibble goes to 15): the MCS6 repeat step carries 221 B per
// fragment, so a 2016 B block needs 10 fragments (MCS0-5 need at most 5). That is the price of the
// rung: one block = 10 frames ~7 ms, accepted because MCS6 only serves the long-reach base layer.
pub const MAX_FRAGS: usize = 12;

/// (seq, block, pinned mcs, air time): None = follow minstrel (CUR_MCS/probe), Some(m) = the
/// simulcast base layer pinned hard (no OLLA, no probe).
type Item = (u64, Vec<u8>, Option<usize>, u64);

struct Queue {
    items: VecDeque<Item>,
    /// retransmissions at the head, in the order they were asked for
    retx_head: usize,
    /// air time of everything in `items`, microseconds (`air_us`)
    air_us: u64,
    /// the worker that fills the queue is gone: the sender ends
    closed: bool,
}

struct Lane {
    q: Mutex<Queue>,
    ready: Condvar,
    room: Condvar,
}

pub struct ConvTx {
    lane: Arc<Lane>,
}

impl Drop for ConvTx {
    fn drop(&mut self) {
        self.lane.q.lock().unwrap().closed = true;
        self.lane.ready.notify_all();
        self.lane.room.notify_all();
    }
}

impl ConvTx {
    /// Queue one block. NEVER blocks the capture loop.
    pub fn push(&self, seq: u64, block: &[u8]) {
        self.push_mcs(seq, block, None)
    }

    /// An ARQ retransmission goes AHEAD of the blocks waiting (behind the retransmissions asked
    /// for before it). At the back of the queue it came 150-680 ms after the NACK (MCS0, txpace
    /// 1500, 4/10) while the receiver waits 300 ms for a hole, then skips the frame and the
    /// picture breaks until a repair. The blocks it overtakes are not late yet; the one the
    /// receiver is waiting on is. Never dropped: it is one block and the air bound counts it.
    pub fn push_retx(&self, seq: u64, block: &[u8]) {
        let air = air_us(block, None);
        let mut q = self.lane.q.lock().unwrap();
        let at = q.retx_head.min(q.items.len());
        q.items.insert(at, (seq, block.to_vec(), None, air));
        q.retx_head = at + 1;
        q.air_us += air;
        drop(q);
        CQ_IN.fetch_add(1, Ordering::Relaxed);
        self.lane.ready.notify_one();
    }

    /// camera pass-through: queue one block, waiting for room. A camera frame loses nothing
    /// block by block; when the link cannot keep up the worker drops whole frames by age.
    pub fn push_wait(&self, seq: u64, block: &[u8]) {
        let air = air_us(block, None);
        // 2 s at most: a sender that stopped taking cannot hold the camera for ever
        let until = Instant::now() + Duration::from_secs(2);
        let mut q = self.lane.q.lock().unwrap();
        while !q.closed
            && q.air_us > 0
            && (q.air_us + air > CQ_MAX_AIR_US || q.items.len() >= CQ_MAX_BLOCKS)
        {
            let left = until.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            q = self.lane.room.wait_timeout(q, left).unwrap().0;
        }
        q.items.push_back((seq, block.to_vec(), None, air));
        q.air_us += air;
        drop(q);
        CQ_IN.fetch_add(1, Ordering::Relaxed);
        self.lane.ready.notify_one();
    }

    /// v36 simulcast: as push, the MCS PINNED (the base layer goes at MCS0 too).
    pub fn push_mcs(&self, seq: u64, block: &[u8], pin: Option<usize>) {
        if pin.is_some() {
            BASE_IN.fetch_add(1, Ordering::Relaxed);
        }
        let air = air_us(block, pin);
        let mut q = self.lane.q.lock().unwrap();
        if q.air_us > 0 && (q.air_us + air > CQ_MAX_AIR_US || q.items.len() >= CQ_MAX_BLOCKS) {
            drop(q);
            let k = CQ_DROP.fetch_add(1, Ordering::Relaxed);
            if k % 200 == 0 {
                log(&format!("conv-tx: queue full, dropping block (total {})", k + 1));
            }
            return;
        }
        q.items.push_back((seq, block.to_vec(), pin, air));
        q.air_us += air;
        drop(q);
        CQ_IN.fetch_add(1, Ordering::Relaxed);
        self.lane.ready.notify_one();
    }
}

/// v40.18: full-frame interleaving flag, read per fragment (`set filv 0|1` takes effect at once).
fn cfg_filv(shared: &Shared) -> bool {
    shared.config.lock().unwrap().filv
}

/// Build the transmit path: one thread, one clock.
///
/// `gap_ms` is re-read from the config every loop, so `set gap` works at runtime.
pub fn spawn(shared: Arc<Shared>, net: Arc<Net>) -> ConvTx {
    // A queue shorter than a keyframe cuts its head off, which costs a repair and a longer
    // freeze than letting it wait for the air it needs anyway: bounded by air time
    // (CQ_MAX_AIR_US), see push_mcs.
    let lane = Arc::new(Lane {
        q: Mutex::new(Queue { items: VecDeque::new(), retx_head: 0, air_us: 0, closed: false }),
        ready: Condvar::new(),
        room: Condvar::new(),
    });
    let out = lane.clone();
    std::thread::Builder::new()
        .name("conv-tx".into())
        .spawn(move || {
            let mut last = Instant::now();
            let mut blk_ctr: u64 = 0; // v32: nhip xen block tham do
            loop {
                let ((seq, block, pin, _), queued_us) = {
                    let mut q = out.q.lock().unwrap();
                    loop {
                        if let Some(it) = q.items.pop_front() {
                            q.retx_head = q.retx_head.saturating_sub(1);
                            q.air_us = q.air_us.saturating_sub(it.3);
                            break (it, q.air_us);
                        }
                        if q.closed {
                            return;
                        }
                        q = out.ready.wait(q).unwrap();
                    }
                };
                out.room.notify_all();
                if shared.stop.load(Ordering::Relaxed) {
                    return;
                }
                // The rung chosen by the OLLA loop (published by worker.rs every frame), or cfg.mcs
                // when auto is off. Re-read per fragment, so a rung change takes effect at once; a
                // block already being cut finishes at the old rung because `pb` was fixed before
                // the fragment loop. v40: ceiling 6 (adds the MCS6 repeat step). minstrel still
                // chooses 0..5; MCS6 is reached only by PINNING (simulcast base / console); auto
                // does not have it in its table (a rung below the floor, only for deliberately
                // crawling far).
                let mut mcs = (CUR_MCS.load(Ordering::Relaxed) as usize).min(6);
                let mut free = CHAN_RUNG_ON.load(Ordering::Relaxed);
                // xen block tham do (minstrel lookaround)
                blk_ctr = blk_ctr.wrapping_add(1);
                let pw = PROBE_MCS.load(Ordering::Relaxed);
                let pe = PROBE_EVERY.load(Ordering::Relaxed).max(2);
                if blk_ctr % pe == 0 {
                    let (p1, p2) = ((pw & 0xFF) as usize, ((pw >> 8) & 0xFF) as usize);
                    let pm = if p2 < 6 && (blk_ctr / pe) % 2 == 1 { p2 } else { p1 };
                    if pm < 6 {
                        mcs = pm;
                        free = false;
                    }
                }
                // v36: the simulcast base layer PINNED hard (no minstrel/probe)
                if let Some(p) = pin {
                    mcs = p.min(6);
                    free = false;
                }
                SENT_MCS[mcs].fetch_add(1, Ordering::Relaxed);
                // v40.48: the frames this block REALLY takes on air (compact when the board does)
                // (branch B: one packet whatever the rung - counting the conv fragments here paced
                // the blocks 2-5x apart and the app's own queue dropped a third of 1080p, 11/10)
                let nfrag = if crate::phy64() {
                    1
                } else {
                    nyx_proto::conv_frames_block(
                        mcs,
                        &block,
                        crate::worker::BOARD_COMPACT.load(Ordering::Relaxed),
                    )
                };
                if nfrag > MAX_FRAGS {
                    log(&format!("conv-tx: MCS{mcs} needs {nfrag} fragments > {MAX_FRAGS}"));
                    continue;
                }
                // v40.40: one raw block per message; the board fragments, encodes
                // and modulates it. Paced here for the air time of all its frames
                // so the queue keeps its drop-to-live behaviour.
                let flags = TXB_CONV
                    | if cfg_filv(&shared) { TXB_FILV } else { 0 }
                    | if free { TXB_RUNG_FREE } else { 0 };
                let msg = Msg::TxBlock {
                    seq, mcs: mcs as u8, rv: 0, flags, payload: block.to_vec(),
                };
                send_paced(&shared, &net, &mut last, nfrag, queued_us, msg);
            }
        })
        .expect("spawn conv-tx");
    ConvTx { lane }
}

/// ONE clock for the whole transmit path: the minimum gap between two WRITES. Floor 2 ms; the
/// daemon separates socket reading from streamer pushing, so the real cadence is set by the `txgap`
/// sample gap on the board, and this only stops pathological cases.
/// Spacing between frames while the queue has slack, ms: what the link ran at until 4/10 (txpace
/// 2000). A burst of interference hits fewer blocks of one picture when its blocks are spread:
/// with a fixed 1.2 ms (2500 MHz, A/B 2 x 60 s, both rate controls) pictures broke 1.5-3x as
/// often and the display hitched twice as often as at 2.0, with every frame still caught by the
/// receiver - the losses were CRC errors, i.e. the air, not the trigger.
const SLOW_MS: f32 = 2.0;
/// ...as long as what waits drains within this: a picture's blocks keep SLOW_MS apart while they
/// fit in about one camera frame interval, and only a keyframe or a picture bigger than that
/// goes faster, down to the board's frame time.
const SPREAD_MS: f32 = 30.0;

fn send_paced(
    shared: &Arc<Shared>,
    net: &Arc<Net>,
    last: &mut Instant,
    nfrag: usize,
    queued_us: u64,
    msg: Msg,
) {
    let gap_ms = {
        let c = shared.config.lock().unwrap();
        // The 2.0 floor was once a harmless "pathology stop"; now IT is the ceiling: 2 ms = 500
        // f/s, exactly the modulator rate measured at EVERY txgap (1600 down to 900), with cq_drop
        // 35 %: the queue fills because the transmit thread is pinned. Lowered to 0.4 ms (2500
        // f/s): still a stop against runaway, while the real cadence is handed back to the board's
        // txgap as originally designed.
        let floor_ms = shared.pace_floor_us.load(Ordering::Relaxed) as f32 / 1000.0;
        // v40.48: never faster than the board puts frames on air (its txframe_us). At 1.8 ms a
        // frame against the board's 2.013 the app ran 10 % ahead, and a keyframe of small
        // compact blocks filled the board's queue and lost its head (12 blocks in 20 ms seen
        // at the drop, 20/9).
        let board_ms = crate::worker::BOARD_FRAME_US.load(Ordering::Relaxed) as f32 / 1000.0;
        // ...and no slower either: a board that reports its frame time is paced by it, the
        // budget is sized from it (worker::board_frame_ms), and the two must be the same number.
        // With the air gap (1.8 ms) on top the app sent 555 frames/s at most, against a budget
        // for 1000 (txpace 1000 and a 4096-sample block, 4/10): its own queue dropped 110-330
        // blocks in 25 s and pictures broke. The air gap is for a board that does not report.
        let own_ms = if board_ms > 0.0 { board_ms } else { c.air_gap_ms };
        // branch B: a block's packet time at the rung in use (2016 B: 0.39 ms at 64QAM 3/4 ..
        // 1.4 ms at QPSK 1/2), not the conv frame's
        let own_ms = if crate::phy64() {
            nyx_proto::phy64_air_us(CUR_MCS.load(Ordering::Relaxed) as usize, nyx_proto::FRAME_PAYLOAD_BYTES) / 1000.0
        } else {
            own_ms
        };
        let fast = own_ms.max(floor_ms).max(if crate::phy64() { 0.2 } else { 0.4 });
        if crate::phy64() && !SPREAD_ON.load(Ordering::Relaxed) {
            // branch B on a clean channel: back to back at the air's own pace
            fast
        } else {
            // frames waiting, this block's included (queued_us is air at the board's frame time)
            let waiting = queued_us as f32 / 1000.0 / fast + nfrag.max(1) as f32;
            (SPREAD_MS / waiting).clamp(fast, SLOW_MS.max(fast))
        }
    };
    let want = Duration::from_micros((gap_ms * 1000.0 * nfrag.max(1) as f32) as u64);
    // Windows thread::sleep OVERSLEEPS by ~1.5 ms (timer resolution); measured on the cable: a 2 ms
    // floor gave a real period of 3.57 ms, flat at every txpace (the pacer's period). Remedy: sleep
    // coarsely (want - 2 ms), then SPIN for the last part for precision. One thread spinning at
    // most 2 ms per frame is cheap on a PC.
    // Linux (the on-board camera app) sleeps to within a fraction of a
    // millisecond, and a spinning thread there takes CPU from the radio daemon on the same
    // two cores: only Windows spins.
    let spin = if cfg!(windows) { Duration::from_millis(2) } else { Duration::ZERO };
    loop {
        let el = last.elapsed();
        if el >= want {
            break;
        }
        let left = want - el;
        if left > spin {
            std::thread::sleep(left - spin);
        } else {
            std::hint::spin_loop();
        }
    }
    *last = Instant::now();
    if net.send(shared, &msg) {
        CQ_OUT.fetch_add(nfrag as u64, Ordering::Relaxed);
    }
}
