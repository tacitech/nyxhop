//! nyx-link: segmentation of application data units (video frames or raw
//! blobs) into fixed-size PHY payload blocks, with CRC32 protection and
//! reassembly at the receiver.

use std::collections::HashMap;

/// Must equal the board modem's frame payload size (nyx_proto::FRAME_PAYLOAD_BYTES).
/// (16 codewords x 126 payload bytes — 2 bytes per codeword go to the
/// code-block CRC16.)
pub const BLOCK_BYTES: usize = 2016;

const MAGIC: u16 = 0x4E58; // "NX"
const HEADER_BYTES: usize = 14;
const CRC_BYTES: usize = 4;
/// Max application payload bytes per block.
pub const SEG_PAYLOAD: usize = BLOCK_BYTES - HEADER_BYTES - CRC_BYTES;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceType {
    Jpeg = 1,
    RawData = 2,
    H264 = 3,
    /// v12: text message / parameters as a string, interleaved between video frames on the same
    /// link (blocks carry their own label, ARQ as usual).
    Text = 4,
    /// v36 simulcast: the BASE layer: low-resolution video PINNED at MCS0, sent alongside the main
    /// layer. The RX shows main while fresh main frames arrive and falls back to base when main is
    /// late (layer switch at the receiver, zero delay, no feedback needed).
    H264Base = 5,
    /// v40.22: arbitrary USER data (UDP into nyx-tx -> UDP out of nyx-rx): 1 datagram = 1 frame of
    /// its own, ARQ/NACK like video. MAVLink telemetry is just one use of it.
    Data = 6,
    /// v40.33: `key=value` lines the transmit-end daemon wants the far app to see
    /// (licence state, DNA); one frame every few seconds, shown, never forwarded.
    Info = 7,
}

impl SourceType {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(SourceType::Jpeg),
            2 => Some(SourceType::RawData),
            3 => Some(SourceType::H264),
            4 => Some(SourceType::Text),
            5 => Some(SourceType::H264Base),
            6 => Some(SourceType::Data),
            7 => Some(SourceType::Info),
            _ => None,
        }
    }
}

/// Split one application data unit into `BLOCK_BYTES` PHY payload blocks.
pub fn packetize(frame_id: u32, src: SourceType, data: &[u8]) -> Vec<[u8; BLOCK_BYTES]> {
    packetize_seg(frame_id, src, data, SEG_PAYLOAD)
}

/// v40.48: like `packetize`, at most `seg` bytes of data per block. A board that sends blocks
/// in compact form (only the bytes in use) puts a small block on air as ONE frame at a slow
/// rung instead of five; `nyx_proto::conv_seg_one_frame` says how small is small enough.
/// Blocks keep their 2016 B and their own `len`, so the receiver needs nothing new for the
/// data blocks.
pub fn packetize_seg(frame_id: u32, src: SourceType, data: &[u8], seg: usize) -> Vec<[u8; BLOCK_BYTES]> {
    let seg = seg.clamp(1, SEG_PAYLOAD);
    let seg_cnt = data.len().div_ceil(seg).max(1) as u16;
    let mut out = Vec::with_capacity(seg_cnt as usize);
    for idx in 0..seg_cnt {
        let lo = idx as usize * seg;
        let hi = (lo + seg).min(data.len());
        let chunk = &data[lo..hi];
        let mut blk = [0u8; BLOCK_BYTES];
        blk[0..2].copy_from_slice(&MAGIC.to_be_bytes());
        blk[2] = 1; // version
        blk[3] = src as u8;
        blk[4..8].copy_from_slice(&frame_id.to_be_bytes());
        blk[8..10].copy_from_slice(&idx.to_be_bytes());
        blk[10..12].copy_from_slice(&seg_cnt.to_be_bytes());
        blk[12..14].copy_from_slice(&(chunk.len() as u16).to_be_bytes());
        blk[HEADER_BYTES..HEADER_BYTES + chunk.len()].copy_from_slice(chunk);
        let crc = crc32fast::hash(&blk[..BLOCK_BYTES - CRC_BYTES]);
        blk[BLOCK_BYTES - CRC_BYTES..].copy_from_slice(&crc.to_be_bytes());
        out.push(blk);
    }
    out
}

/// v-fec: like `packetize` but ADDS one PARITY block (XOR of all data blocks) when the frame has >=
/// 2 blocks. Lose EXACTLY one block -> the RX rebuilds it -> the frame is still complete -> NO IDR
/// request (each IDR is a burst that hogs the air, the main source of stutter). Overhead = 1/N (N=4
/// -> +25 %); with N=1 it is SKIPPED (parity would cost 100 % for nothing).
///
/// Parity is recognised by `seg_idx == seg_cnt` (outside the data range 0..seg_cnt-1). An OLD RX
/// treats that as a broken header and ignores it -> BACKWARD COMPATIBLE. The parity block's `len` =
/// the length of the LAST data block (needed to cut correctly when rebuilding that last block).
pub fn packetize_fec(
    frame_id: u32,
    src: SourceType,
    data: &[u8],
    fec: bool,
) -> Vec<[u8; BLOCK_BYTES]> {
    packetize_fec_seg(frame_id, src, data, fec, SEG_PAYLOAD)
}

/// Where a version-2 parity block keeps the segment size (the last two payload bytes, past any
/// segment it can describe - see `packetize_fec_seg`).
const PAR_SEG_AT: usize = HEADER_BYTES + SEG_PAYLOAD - 2;

/// v40.48: `packetize_fec` with `seg` bytes per block. With segments shorter than the full
/// block, a rebuilt MIDDLE block is `seg` long, which the receiver cannot know from a two-block
/// frame whose first block was lost - so the parity block says it: version 2, the segment size
/// in its last two payload bytes (outside the XOR, which covers only `seg`). A receiver that
/// predates this refuses version 2 and simply goes without the parity: never a wrong frame.
pub fn packetize_fec_seg(
    frame_id: u32,
    src: SourceType,
    data: &[u8],
    fec: bool,
    seg: usize,
) -> Vec<[u8; BLOCK_BYTES]> {
    let seg = if seg >= SEG_PAYLOAD - 2 { SEG_PAYLOAD } else { seg.max(1) };
    let mut out = packetize_seg(frame_id, src, data, seg);
    let n = out.len();
    // v34: the n < 2 guard is GONE. Measured on the board: at low bitrate (vbr 309 kbps @ 30 fps ->
    // ~1.3 KB/frame < 1 block) EVERY frame is a single block, so FEC never ran exactly when it was
    // needed most: a lost block was a lost frame. With n=1 the parity is a copy of the block (XOR
    // of one element): twice the cost, but small frames are cheap, and in return a single loss NO
    // LONGER kills the frame. The receiver needs no change: the "exactly 1 block missing + parity
    // present" branch is already general (n=1 is the special case where the XOR gives the block
    // itself).
    if !fec || n == 0 || n >= u16::MAX as usize {
        return out;
    }
    let last_len = {
        let rem = data.len() % seg;
        if rem == 0 && !data.is_empty() { seg } else { rem }
    };
    let mut par = [0u8; BLOCK_BYTES];
    par[0..2].copy_from_slice(&MAGIC.to_be_bytes());
    par[2] = if seg < SEG_PAYLOAD { 2 } else { 1 };
    par[3] = src as u8;
    par[4..8].copy_from_slice(&frame_id.to_be_bytes());
    par[8..10].copy_from_slice(&(n as u16).to_be_bytes()); // idx == cnt
    par[10..12].copy_from_slice(&(n as u16).to_be_bytes());
    par[12..14].copy_from_slice(&(last_len as u16).to_be_bytes());
    for b in &out {
        for i in 0..SEG_PAYLOAD {
            par[HEADER_BYTES + i] ^= b[HEADER_BYTES + i];
        }
    }
    if seg < SEG_PAYLOAD {
        par[PAR_SEG_AT..PAR_SEG_AT + 2].copy_from_slice(&(seg as u16).to_be_bytes());
    }
    let crc = crc32fast::hash(&par[..BLOCK_BYTES - CRC_BYTES]);
    par[BLOCK_BYTES - CRC_BYTES..].copy_from_slice(&crc.to_be_bytes());
    out.push(par);
    out
}

/// Parsed, CRC-verified segment.
pub struct Segment {
    pub frame_id: u32,
    pub seg_idx: u16,
    pub seg_cnt: u16,
    pub src: SourceType,
    pub payload: Vec<u8>,
    /// Some(len) if this is the PARITY block: length of the LAST data block.
    pub par_last_len: Option<usize>,
    /// v40.48: a version-2 parity block's segment size (the length of every block but the last).
    pub par_seg: Option<usize>,
}

/// Validate CRC and parse one received block. None => block corrupt.
pub fn parse_block(blk: &[u8]) -> Option<Segment> {
    if blk.len() != BLOCK_BYTES {
        return None;
    }
    let crc_rx = u32::from_be_bytes(blk[BLOCK_BYTES - CRC_BYTES..].try_into().ok()?);
    if crc32fast::hash(&blk[..BLOCK_BYTES - CRC_BYTES]) != crc_rx {
        return None;
    }
    if u16::from_be_bytes([blk[0], blk[1]]) != MAGIC || !(blk[2] == 1 || blk[2] == 2) {
        return None;
    }
    let src = SourceType::from_u8(blk[3])?;
    let frame_id = u32::from_be_bytes(blk[4..8].try_into().ok()?);
    let seg_idx = u16::from_be_bytes([blk[8], blk[9]]);
    let seg_cnt = u16::from_be_bytes([blk[10], blk[11]]);
    let len = u16::from_be_bytes([blk[12], blk[13]]) as usize;
    // seg_idx == seg_cnt = the PARITY block (v-fec); the payload takes the FULL SEG_PAYLOAD because
    // the XOR is computed over the zero-padded area.
    let is_par = seg_idx == seg_cnt;
    if seg_cnt == 0 || seg_idx > seg_cnt || len > SEG_PAYLOAD {
        return None;
    }
    let take = if is_par { SEG_PAYLOAD } else { len };
    // version 2 is only ever a parity block carrying its segment size
    let par_seg = if blk[2] == 2 {
        let s = u16::from_be_bytes([blk[PAR_SEG_AT], blk[PAR_SEG_AT + 1]]) as usize;
        if !is_par || s == 0 || s >= SEG_PAYLOAD - 1 {
            return None;
        }
        Some(s)
    } else {
        None
    };
    Some(Segment {
        frame_id,
        seg_idx,
        seg_cnt,
        src,
        payload: blk[HEADER_BYTES..HEADER_BYTES + take].to_vec(),
        par_last_len: if is_par { Some(len) } else { None },
        par_seg,
    })
}

/// Reassembles segments into complete application data units.
struct Pending {
    src: SourceType,
    parts: Vec<Option<Vec<u8>>>,
    /// payload of the PARITY block (XOR, zero-padded to SEG_PAYLOAD) if received.
    parity: Option<Vec<u8>>,
    /// length of the LAST data block (from the parity header).
    last_len: usize,
    /// v40.48: length of every other block (a version-2 parity says it; full otherwise).
    seg: usize,
}

#[derive(Default)]
pub struct Reassembler {
    pending: HashMap<u32, Pending>,
    /// frames rebuilt thanks to parity (exactly 1 block lost).
    pub fec_recovered: u64,
    /// v40.35: recently completed frame ids (late blocks are ignored)
    done: std::collections::VecDeque<u32>,
    /// INCOMPLETE frames delivered (slicing: one band broken, not the whole frame lost).
    pub partial_delivered: u64,
}

impl Reassembler {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one verified segment; returns a completed data unit if this
    /// segment finished it.
    pub fn push(&mut self, seg: Segment) -> Option<(u32, SourceType, Vec<u8>)> {
        // v40.35: a frame completed once is done. Its parity block arriving
        // after the data used to open a fresh entry and, for a 1-block frame,
        // "recover" the block from parity alone -> the frame was delivered
        // twice (883 duplicates / 100 s, each dropped behind the cursor).
        if self.done.contains(&seg.frame_id) {
            return None;
        }
        let cnt = seg.seg_cnt as usize;
        let entry = self.pending.entry(seg.frame_id).or_insert_with(|| Pending {
            src: seg.src,
            parts: vec![None; cnt],
            parity: None,
            last_len: 0,
            seg: SEG_PAYLOAD,
        });
        if entry.parts.len() != cnt {
            return None; // inconsistent header, drop
        }
        match seg.par_last_len {
            // block PARITY (v-fec)
            Some(ll) => {
                entry.parity = Some(seg.payload);
                entry.last_len = ll;
                entry.seg = seg.par_seg.unwrap_or(SEG_PAYLOAD);
            }
            None => entry.parts[seg.seg_idx as usize] = Some(seg.payload),
        }
        // v-fec: EXACTLY 1 block missing + parity present -> rebuild (XOR back). This is where "1
        // lost block = broken frame -> IDR request -> burst -> congestion" turns into "the frame is
        // still complete, nobody has to ask for anything".
        let missing: Vec<usize> = entry
            .parts
            .iter()
            .enumerate()
            .filter(|(_, p)| p.is_none())
            .map(|(i, _)| i)
            .collect();
        if missing.len() == 1 && entry.parity.is_some() {
            let idx = missing[0];
            let mut rec = entry.parity.clone().unwrap();
            rec.resize(SEG_PAYLOAD, 0);
            for (i, p) in entry.parts.iter().enumerate() {
                if i == idx {
                    continue;
                }
                if let Some(p) = p {
                    for (r, b) in rec.iter_mut().zip(p.iter()) {
                        *r ^= *b;
                    }
                }
            }
            let want = if idx + 1 == cnt { entry.last_len } else { entry.seg };
            rec.truncate(want.min(SEG_PAYLOAD));
            entry.parts[idx] = Some(rec);
            self.fec_recovered += 1;
        }
        let entry = self.pending.get(&seg.frame_id)?;
        if entry.parts.iter().all(|s| s.is_some()) {
            let e = self.pending.remove(&seg.frame_id).unwrap();
            self.done.push_back(seg.frame_id);
            if self.done.len() > 128 {
                self.done.pop_front();
            }
            let (src, parts) = (e.src, e.parts);
            let mut data = Vec::new();
            for p in parts {
                data.extend_from_slice(&p.unwrap());
            }
            // Keep RECENT incomplete frames instead of dropping everything older: the twin FD/IQ
            // paths deliver segments tens of ms out of phase, and the many-block frame of vframe N
            // often lands AFTER vframe N+1 (few blocks, fast path) has assembled; dropping older
            // frames threw N's remains away and its retransmission was then useless (the other half
            // was gone) = a vframe lost for good at bler 0 %. A 16-frame window ≈ 0.5 s at 30 fps,
            // matching the decoder's 250 ms reorder stage; memory stays bounded.
            const KEEP: u32 = 16;
            self.pending.retain(|&id, _| id + KEEP > seg.frame_id);
            return Some((seg.frame_id, src, data));
        }
        None
    }

    /// v-slice: TAKE the incomplete frames that are "past their time" (a newer frame has arrived,
    /// they can no longer complete) instead of DROPPING them. Join the blocks that ARE there, skip
    /// the holes: because the encoder cuts independent SLICES, the intact slices still decode (the
    /// decoder resyncs at the next start code) => a picture with one "blurred/torn band" instead of
    /// a FROZEN picture waiting for an IDR.
    ///
    /// Only deliver while >= `min_frac` of the blocks remain (past half lost there is more garbage
    /// than signal; better keep the old picture). Returns (id, src, data) per frame.
    pub fn take_stale(
        &mut self,
        newest_id: u32,
        lag: u32,
        min_frac: f32,
    ) -> Vec<(u32, SourceType, Vec<u8>)> {
        let ids: Vec<u32> = self
            .pending
            .keys()
            .copied()
            .filter(|&id| id + lag <= newest_id)
            .collect();
        let mut out = Vec::new();
        for id in ids {
            let Some(e) = self.pending.remove(&id) else { continue };
            let have = e.parts.iter().filter(|p| p.is_some()).count();
            let total = e.parts.len().max(1);
            if have == 0 || (have as f32) < min_frac * total as f32 {
                continue; // too little -> drop, keep the old picture
            }
            let mut data = Vec::new();
            for p in e.parts.iter().flatten() {
                data.extend_from_slice(p);
            }
            self.partial_delivered += 1;
            out.push((id, e.src, data));
        }
        out
    }

    /// Drop stale partial frames (call occasionally).
    pub fn prune(&mut self, newest_id: u32, keep: u32) {
        self.pending.retain(|&id, _| id + keep >= newest_id);
    }

    /// v37.1: whether frame `id` has a partial fragment in the funnel. The RX uses it to tell a
    /// "never sent" hole (the TX advances the id on a tick with no frame / a frame dropped by
    /// budget) from a "lost on the way" hole (a fragment trace exists): only the second kind is
    /// worth waiting on ARQ for + an IDR request.
    pub fn has_partial(&self, id: u32) -> bool {
        self.pending.contains_key(&id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// v-fec: lose EXACTLY 1 block, any block -> parity rebuilds it -> the frame is STILL COMPLETE.
    #[test]
    fn fec_recovers_any_single_loss() {
        let data: Vec<u8> = (0..5000u32).map(|i| (i * 13 + 7) as u8).collect();
        let blocks = packetize_fec(7, SourceType::H264, &data, true);
        let n_data = 5000usize.div_ceil(SEG_PAYLOAD);
        assert_eq!(blocks.len(), n_data + 1, "phai co 1 block parity");
        // try DROPPING each data block in turn
        for drop_i in 0..n_data {
            let mut re = Reassembler::new();
            let mut out = None;
            for (i, b) in blocks.iter().enumerate() {
                if i == drop_i {
                    continue; // lose this block
                }
                if let Some(seg) = parse_block(b) {
                    if let Some(d) = re.push(seg) {
                        out = Some(d);
                    }
                }
            }
            let (_, _, got) = out.unwrap_or_else(|| panic!("lost block {drop_i} -> could NOT be recovered"));
            assert_eq!(got, data, "du lieu dung lai SAI khi mat block {drop_i}");
        }
    }

    /// Losing 2 blocks is beyond parity: must stay silent, never panic.
    #[test]
    fn fec_two_losses_no_panic() {
        let data: Vec<u8> = (0..5000u32).map(|i| i as u8).collect();
        let blocks = packetize_fec(9, SourceType::Jpeg, &data, true);
        let mut re = Reassembler::new();
        let mut out = None;
        for (i, b) in blocks.iter().enumerate() {
            if i < 2 {
                continue;
            }
            if let Some(seg) = parse_block(b) {
                if let Some(d) = re.push(seg) {
                    out = Some(d);
                }
            }
        }
        assert!(out.is_none());
    }

    /// v34: a 1-block frame WITH parity (a copy). Previously skipped as "100 % overhead for
    /// nothing", but board measurements showed that at low bitrate EVERY frame is 1 block, so
    /// skipping = FEC off exactly when needed most. Small frames are cheap; in return one lost
    /// block does not kill the frame.
    #[test]
    fn fec_covers_single_block_frame() {
        let data = vec![1u8; 100];
        let out = packetize_fec(1, SourceType::Jpeg, &data, true);
        assert_eq!(out.len(), 2, "1 block du lieu + 1 parity");
        // normal path: the data block arrives first -> the frame comes out at once
        let mut r = Reassembler::default();
        let dat = parse_block(&out[0]).expect("data parse");
        let done = r.push(dat).expect("du lieu du -> ra frame");
        assert_eq!(&done.2[..100], &data[..]);
    }

    /// Lose EXACTLY the only data block: parity rebuilds it in full.
    #[test]
    fn fec_recovers_lost_single_block() {
        let data = vec![7u8; 300];
        let out = packetize_fec(9, SourceType::Jpeg, &data, true);
        let mut r = Reassembler::default();
        let par = parse_block(&out[1]).expect("parity parse");
        let done = r.push(par);
        assert!(done.is_some(), "n=1: chi parity la du de dung lai");
        assert_eq!(&done.unwrap().2[..300], &data[..]);
    }

    #[test]
    fn small_segments_recover_every_single_loss() {
        // v40.48: blocks cut small (one frame at a slow rung once the board compacts them):
        // lose any ONE block of a frame and the parity puts it back at its right length -
        // including a middle block, and the first of a two-block frame, which only the
        // version-2 parity's segment size can size.
        for (len, seg) in [(3000usize, 424usize), (500, 424), (424, 424), (848, 424), (1300, 649), (10, 199)] {
            let data: Vec<u8> = (0..len).map(|i| (i * 31 % 251) as u8 + 1).collect();
            let blocks = packetize_fec_seg(9, SourceType::H264, &data, true, seg);
            let n_data = len.div_ceil(seg).max(1);
            assert_eq!(blocks.len(), n_data + 1);
            for lost in 0..n_data {
                let mut r = Reassembler::new();
                let mut got = None;
                for (k, b) in blocks.iter().enumerate() {
                    if k == lost {
                        continue;
                    }
                    let s = parse_block(b).expect("block parses");
                    if let Some(f) = r.push(s) {
                        got = Some(f);
                    }
                }
                let (id, src, out) = got.unwrap_or_else(|| panic!("len {len} seg {seg}: lost {lost} not recovered"));
                assert_eq!((id, src), (9, SourceType::H264));
                assert_eq!(out, data, "len {len} seg {seg} lost {lost}");
            }
        }
        // the old call is byte for byte what it was: version 1, full segments
        let data: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
        let a = packetize_fec(3, SourceType::H264, &data, true);
        let b = packetize_fec_seg(3, SourceType::H264, &data, true, SEG_PAYLOAD);
        assert_eq!(a, b);
        assert!(a.iter().all(|blk| blk[2] == 1));
    }

    #[test]
    fn packetize_roundtrip() {
        let data: Vec<u8> = (0..5000u32).map(|i| (i * 7) as u8).collect();
        let blocks = packetize(42, SourceType::Jpeg, &data);
        assert_eq!(blocks.len(), 5000usize.div_ceil(SEG_PAYLOAD));
        let mut re = Reassembler::new();
        let mut out = None;
        for b in &blocks {
            let seg = parse_block(b).expect("crc should pass");
            if let Some(done) = re.push(seg) {
                out = Some(done);
            }
        }
        let (id, src, rx) = out.expect("frame should complete");
        assert_eq!(id, 42);
        assert_eq!(src, SourceType::Jpeg);
        assert_eq!(rx, data);
    }

    #[test]
    fn corrupt_block_rejected() {
        let data = vec![1u8; 100];
        let mut blocks = packetize(1, SourceType::RawData, &data);
        blocks[0][20] ^= 0x40;
        assert!(parse_block(&blocks[0]).is_none());
    }
}
