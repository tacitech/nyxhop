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

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::Arc;
use std::time::{Duration, Instant};

use nyx_common::logging::log;
use nyx_proto::{Msg, TXB_CONV, TXB_FILV, conv_frames};

use crate::net::Net;
use crate::Shared;

/// The MCS rung in use, published by the worker (OLLA chooses it when `auto` is on). The transmit
/// path runs on its own thread and cannot see the frame loop's locals.
pub static CUR_MCS: AtomicU64 = AtomicU64::new(5);
// v32 minstrel: the worker sets a probe rung (0xFF = none); every PROBE_EVERY-th block goes out at
// that rung so the NEIGHBOUR's prob statistics stay fresh without betting the whole stream (the
// minstrel/mac80211 principle).
pub static PROBE_MCS: AtomicU64 = AtomicU64::new(0xFF);
pub static PROBE_EVERY: AtomicU64 = AtomicU64::new(12);
/// BLOCKS sent per MCS actually used (probes included); the worker divides ok_mcs (from feedback)
/// by this for a per-rate prob, on the SAME window. v40: 7 entries: rung 6 (the repeat step) is
/// counted separately for inspection.
pub static SENT_MCS: [AtomicU64; 7] = [
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0),
];

/// Blocks placed in the transmit queue.
pub static CQ_IN: AtomicU64 = AtomicU64::new(0);
/// Messages written to the socket.
pub static CQ_OUT: AtomicU64 = AtomicU64::new(0);
/// Blocks dropped because the queue was full (overload): the ONLY place a loss is allowed.
pub static CQ_DROP: AtomicU64 = AtomicU64::new(0);
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

pub struct ConvTx {
    // v36: (seq, block, pinned mcs): None = follow minstrel (CUR_MCS/probe), Some(m) = the
    // simulcast base layer pinned hard (no OLLA, no probe).
    tx: SyncSender<(u64, Vec<u8>, Option<usize>)>,
}

impl ConvTx {
    /// Queue one block. NEVER blocks the capture loop.
    pub fn push(&self, seq: u64, block: &[u8]) {
        self.push_mcs(seq, block, None)
    }

    /// v36 simulcast: nhu push nhung GHIM MCS (lop nen di MCS0 cung).
    pub fn push_mcs(&self, seq: u64, block: &[u8], pin: Option<usize>) {
        if pin.is_some() {
            BASE_IN.fetch_add(1, Ordering::Relaxed);
        }
        match self.tx.try_send((seq, block.to_vec(), pin)) {
            Ok(()) => {
                CQ_IN.fetch_add(1, Ordering::Relaxed);
            }
            Err(TrySendError::Full(_)) => {
                let k = CQ_DROP.fetch_add(1, Ordering::Relaxed);
                if k % 200 == 0 {
                    log(&format!("conv-tx: queue full, dropping block (total {})", k + 1));
                }
            }
            Err(TrySendError::Disconnected(_)) => {}
        }
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
    let (tx, rx) = sync_channel::<(u64, Vec<u8>, Option<usize>)>(64);
    std::thread::Builder::new()
        .name("conv-tx".into())
        .spawn(move || {
            let mut last = Instant::now();
            let mut blk_ctr: u64 = 0; // v32: nhip xen block tham do
            for (seq, block, pin) in rx {
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
                // xen block tham do (minstrel lookaround)
                blk_ctr = blk_ctr.wrapping_add(1);
                let pm = PROBE_MCS.load(Ordering::Relaxed) as usize;
                let pe = PROBE_EVERY.load(Ordering::Relaxed).max(2);
                if pm < 6 && blk_ctr % pe == 0 {
                    mcs = pm;
                }
                // v36: the simulcast base layer PINNED hard (no minstrel/probe)
                if let Some(p) = pin {
                    mcs = p.min(6);
                }
                SENT_MCS[mcs].fetch_add(1, Ordering::Relaxed);
                let nfrag = conv_frames(mcs, block.len());
                if nfrag > MAX_FRAGS {
                    log(&format!("conv-tx: MCS{mcs} needs {nfrag} fragments > {MAX_FRAGS}"));
                    continue;
                }
                // v40.40: one raw block per message; the board fragments, encodes
                // and modulates it. Paced here for the air time of all its frames
                // so the queue keeps its drop-to-live behaviour.
                let flags = TXB_CONV | if cfg_filv(&shared) { TXB_FILV } else { 0 };
                let msg = Msg::TxBlock {
                    seq, mcs: mcs as u8, rv: 0, flags, payload: block.to_vec(),
                };
                send_paced(&shared, &net, &mut last, nfrag, msg);
            }
        })
        .expect("spawn conv-tx");
    ConvTx { tx }
}

/// ONE clock for the whole transmit path: the minimum gap between two WRITES. Floor 2 ms; the
/// daemon separates socket reading from streamer pushing, so the real cadence is set by the `txgap`
/// sample gap on the board, and this only stops pathological cases.
fn send_paced(
    shared: &Arc<Shared>,
    net: &Arc<Net>,
    last: &mut Instant,
    nfrag: usize,
    msg: Msg,
) {
    let gap_ms = {
        let c = shared.config.lock().unwrap();
        // The 2.0 floor was once a harmless "pathology stop"; now IT is the ceiling: 2 ms = 500
        // f/s, exactly the modulator rate measured at EVERY txgap (1600 down to 900), with cq_drop
        // 35 %: the queue fills because the transmit thread is pinned. Lowered to 0.4 ms (2500
        // f/s): still a stop against runaway, while the real cadence is handed back to the board's
        // txgap as originally designed.
        c.air_gap_ms.max(0.4)
    };
    let want = Duration::from_micros((gap_ms * 1000.0 * nfrag.max(1) as f32) as u64);
    // Windows thread::sleep OVERSLEEPS by ~1.5 ms (timer resolution); measured on the cable: a 2 ms
    // floor gave a real period of 3.57 ms, flat at every txpace (the pacer's period). Remedy: sleep
    // coarsely (want - 2 ms), then SPIN for the last part for precision. One thread spinning at
    // most 2 ms per frame is cheap on a PC.
    loop {
        let el = last.elapsed();
        if el >= want {
            break;
        }
        let left = want - el;
        if left > Duration::from_millis(2) {
            std::thread::sleep(left - Duration::from_millis(2));
        } else {
            std::hint::spin_loop();
        }
    }
    *last = Instant::now();
    if net.send(shared, &msg) {
        CQ_OUT.fetch_add(nfrag as u64, Ordering::Relaxed);
    }
}
