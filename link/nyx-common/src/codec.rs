//! H.264 video codec (openh264) with link-aware controls: target bitrate
//! follows the PHY rate and the receiver can request an IDR after loss.

use openh264::OpenH264API;
use openh264::decoder::Decoder;
use openh264::encoder::{
    BitRate, Encoder, EncoderConfig, FrameRate, IntraFramePeriod, RateControlMode,
};
use openh264::formats::{RgbSliceU8, YUVBuffer, YUVSource};

use crate::RgbFrame;
use crate::logging::log;

/// How often the encoder marks a long-term reference. It has to be often enough that the
/// receiver always holds a recent one (an older reference means a bigger repair frame) and rare
/// enough not to cost coding efficiency: 30 pictures is 1-2 s at the rates this link runs.
const LTR_MARK_PERIOD: u32 = 30;

/// Switch long-term references on. The high-level crate does not carry these in its config, but
/// they are plain SetOptions - with one catch: the crate initialises openh264 LAZILY, on the
/// first frame (that is where it learns the picture size), and SetOption before that returns
/// cmInitExpected (4). Hence the black priming frame in `new_with_intra`.
fn enable_ltr(enc: &mut Encoder, period: u32) -> bool {
    use openh264_sys2::{ENCODER_LTR_MARKING_PERIOD, ENCODER_OPTION_LTR, SLTRConfig};
    let mut cfg = SLTRConfig { bEnableLongTermReference: true, iLTRRefNum: 1 };
    let mut per = period;
    // SAFETY: the encoder is initialised and both structs are the C types these options expect;
    // SetOption only reads them.
    let (a, b) = unsafe {
        let api = enc.raw_api();
        (
            api.set_option(ENCODER_OPTION_LTR, &mut cfg as *mut SLTRConfig as *mut std::os::raw::c_void),
            api.set_option(ENCODER_LTR_MARKING_PERIOD, &mut per as *mut u32 as *mut std::os::raw::c_void),
        )
    };
    if a != 0 {
        log(&format!("h264 encoder: long-term reference unavailable ({a}) - losses will cost a keyframe"));
        return false;
    }
    if b != 0 {
        log(&format!("h264 encoder: LTR marking period refused ({b}) - keeping the default"));
    }
    true
}

pub struct VideoEncoder {
    enc: Encoder,
    width: usize,
    height: usize,
    bitrate_bps: u32,
    fps: f32,
    frames_since_idr: u32,
    /// v33: last encoder restart (stops continuous resets while vbr jumps)
    last_reinit: std::time::Instant,
    /// v40.25: SPS+PPS of the current encoder: OpenH264 only emits them at the FIRST IDR
    /// (CONSTANT_ID); a decoder recreated at the RX (after the TX renumbers / gets stuck) cannot
    /// decode an IDR without SPS/PPS -> prepend them to EVERY IDR.
    sps_pps: Vec<u8>,
    /// v40.47: the encoder accepted long-term references (see enable_ltr)
    ltr: bool,
}

impl VideoEncoder {
    pub fn new(
        width: usize,
        height: usize,
        bitrate_bps: u32,
        fps: f32,
    ) -> Result<Self, String> {
        Self::new_with_intra(width, height, bitrate_bps, fps, 300)
    }

    pub fn new_with_intra(
        width: usize,
        height: usize,
        bitrate_bps: u32,
        fps: f32,
        intra: u32,
    ) -> Result<Self, String> {
        // STRICT bitrate mode. The crate default is RateControlMode::Quality with max_frame_rate 0,
        // under which openh264 treats the bitrate as a loose quality hint: synthetic test patterns
        // happened to fit, but real (noisy) camera content came out ~17x over target; each video
        // frame spanned ~25 PHY frames and flooded the link so thoroughly that no video frame ever
        // arrived complete. Bitrate mode + the real frame rate (the RC budget is bitrate/fps per
        // frame) + skip-frames keeps the encoder inside the link budget at any content.
        // v-lat: AGAINST I-FRAME SPIKES (the main source of stutter with a webcam).
        // scene_change_detect is ON by default: moving scenes (a real camera) keep being taken for
        // "scene changes" -> the encoder fires an I-frame -> a frame 5-10x bigger -> 6-9 blocks x
        // 20 ms of airtime -> the queue piles up -> a STALL, then a lost frame -> the RX asks for
        // an IDR -> big again (a vicious circle). A static pattern never triggers it, which is why
        // the pattern was smooth and the webcam stuttered. Turn scene-change off and set a SPARSE
        // fixed I-frame period (the network still has ARQ + on-demand IDR when the RX loses a
        // frame).
        let config = EncoderConfig::new()
            .bitrate(BitRate::from_bps(bitrate_bps))
            .rate_control_mode(RateControlMode::Bitrate)
            .max_frame_rate(FrameRate::from_hz(fps.max(1.0)))
            .scene_change_detect(false)
            // v37: the IDR period is a PARAMETER: the simulcast base layer needs a short one (a
            // lifebuoy must recover its picture fast; 300 = 20 s is useless), the main layer keeps
            // a sparse one (it has ARQ + IDR on request).
            .intra_frame_period(IntraFramePeriod::from_num_frames(intra))
            // v-slice: split the frame into SEVERAL independent SLICES, each <= 1800 B (under the
            // 1998 B payload of one block). Lose 1 block -> only 1-2 bands of the picture break,
            // the remaining slices STILL DECODE -> "gradual blur" instead of "whole frame broken".
            // Before, 1 frame = 1 big NAL: one lost block lost everything.
            .max_slice_len(1800)
            .skip_frames(true);
        let mut enc = Encoder::with_api_config(OpenH264API::from_source(), config)
            .map_err(|e| e.to_string())?;
        // Prime the encoder with one black frame so openh264 is really initialised, then switch
        // long-term references on and throw that bitstream away. Turning LTR on raises the number
        // of reference frames, which resets the encoder and changes the SPS - which is exactly why
        // it happens here, before the caller's first picture: that one then comes out as a fresh
        // IDR carrying the new SPS, and `with_sps_pps` caches it as usual.
        let _ = enc.encode(&YUVBuffer::new(width.max(16), height.max(16)));
        let ltr = enable_ltr(&mut enc, LTR_MARK_PERIOD);
        enc.force_intra_frame();
        Ok(VideoEncoder {
            enc, width, height, bitrate_bps, fps, frames_since_idr: 0,
            last_reinit: std::time::Instant::now(),
            sps_pps: Vec::new(),
            ltr,
        })
    }

    /// Recreate the encoder when resolution/fps changes or the bitrate
    /// target moves by more than 25 % (rate follows the PHY link adaptation).
    pub fn ensure(&mut self, width: usize, height: usize, bitrate_bps: u32, fps: f32) {
        let rate_moved = (bitrate_bps as f32 - self.bitrate_bps as f32).abs()
            > 0.25 * self.bitrate_bps as f32;
        // v33: restarting the encoder = RESET + IDR (a few dozen broken frames). OLLA changing
        // rungs made vbr jump > 25 % continuously -> measured on the board: the radio delivered
        // every block, cq_drop = 0, yet rx_fps 0.5 because the encoder restarted every few seconds.
        // Wi-Fi/OcuSync do not suffer this because they change the RATE without restarting the
        // encoder. Guard: a bitrate change may restart the encoder at most once per 3 s (a
        // resolution/fps change still does it at once; that genuinely needs a new encoder).
        let shape_changed = width != self.width || height != self.height;
        // v40.34: a frame-rate change is a SetOption, not a restart (10/15/20 fps
        // survival flips used to restart the encoder each time).
        if (fps - self.fps).abs() > 0.5 && !shape_changed {
            self.set_fps(fps);
        }
        // v40.25: CHANGE THE RATE WITHOUT RESTARTING THE ENCODER (SetOption BITRATE): a restart = a
        // new IDR + "a few dozen broken frames" at the decoder; the RC jumps > 25 % continuously,
        // so it restarted 3-12 times a MINUTE (measured: the main-layer decoder broke ~90 % of
        // frames, the base layer carried the picture). Restart only on a resolution/fps change.
        if !shape_changed && rate_moved {
            // v40.34: a failed SetOption keeps the old rate - it is retried on the
            // next frame; a restart for a rate change is never worth the IDR.
            self.set_bitrate(bitrate_bps);
            return;
        }
        // v40.34: resolution changes are the only restart, at most one per 3 s.
        let rate_ok = false;
        if shape_changed && self.last_reinit.elapsed() < std::time::Duration::from_secs(3) {
            return;
        }
        if shape_changed || rate_ok {
            self.last_reinit = std::time::Instant::now();
            match VideoEncoder::new(width, height, bitrate_bps, fps) {
                Ok(new) => {
                    log(&format!(
                        "h264 encoder: {}x{} @ {} kbit/s, {} fps",
                        width,
                        height,
                        bitrate_bps / 1000,
                        fps
                    ));
                    *self = new;
                }
                Err(e) => log(&format!("h264 encoder recreate failed: {e}")),
            }
        }
    }

    /// v40.25: change the target bitrate + max load (SetOption) without restarting the encoder.
    /// false = the API refused (the ensure() caller falls back to a restart).
    pub fn set_fps(&mut self, fps: f32) {
        use openh264_sys2::ENCODER_OPTION_FRAME_RATE;
        let mut f: f32 = fps.max(1.0);
        // SAFETY: encoder initialised; ENCODER_OPTION_FRAME_RATE takes a float.
        let rc = unsafe {
            self.enc.raw_api().set_option(
                ENCODER_OPTION_FRAME_RATE,
                &mut f as *mut f32 as *mut std::os::raw::c_void,
            )
        };
        if rc == 0 {
            self.fps = fps;
        } else if self.last_reinit.elapsed() >= std::time::Duration::from_secs(5) {
            self.last_reinit = std::time::Instant::now();
            log(&format!("h264 encoder: SetOption frame rate {fps} failed ({rc})"));
        }
    }

    pub fn set_bitrate(&mut self, bitrate_bps: u32) -> bool {
        use openh264_sys2::{
            TagBitrateInfo, ENCODER_OPTION_BITRATE, ENCODER_OPTION_MAX_BITRATE,
            SPATIAL_LAYER_0, SPATIAL_LAYER_ALL,
        };
        let mut info = TagBitrateInfo {
            iLayer: SPATIAL_LAYER_ALL,
            iBitrate: bitrate_bps.min(i32::MAX as u32) as i32,
        };
        // SAFETY: the encoder is initialised; TagBitrateInfo is a C struct with the right layout;
        // SetOption only copies the value. v40.34: openh264 refuses BITRATE > MAX and MAX <
        // BITRATE, so raise MAX first when going up and lower BITRATE first when going down (the
        // fixed order failed one direction every time). v40.35: the crate pins
        // sSpatialLayers[0].iMaxSpatialBitrate to the INIT rate and SPATIAL_LAYER_ALL only touches
        // the global fields, while every SetOption re-verifies the layer (bitrate <= layer max), so
        // a change in either direction failed with (1, 1) and the encoder kept its first bitrate
        // for ever. Set the LAYER max/target as well, max first on the way up, target first on the
        // way down.
        let up = bitrate_bps > self.bitrate_bps;
        let mut l0 = TagBitrateInfo { iLayer: SPATIAL_LAYER_0, iBitrate: info.iBitrate };
        let rc = unsafe {
            let api = self.enc.raw_api();
            let pa = &mut info as *mut TagBitrateInfo as *mut std::os::raw::c_void;
            let p0 = &mut l0 as *mut TagBitrateInfo as *mut std::os::raw::c_void;
            if up {
                let m0 = api.set_option(ENCODER_OPTION_MAX_BITRATE, p0);
                let m1 = api.set_option(ENCODER_OPTION_MAX_BITRATE, pa);
                let b0 = api.set_option(ENCODER_OPTION_BITRATE, p0);
                let b1 = api.set_option(ENCODER_OPTION_BITRATE, pa);
                (m0 | m1, b0 | b1)
            } else {
                let b0 = api.set_option(ENCODER_OPTION_BITRATE, p0);
                let b1 = api.set_option(ENCODER_OPTION_BITRATE, pa);
                let m0 = api.set_option(ENCODER_OPTION_MAX_BITRATE, p0);
                let m1 = api.set_option(ENCODER_OPTION_MAX_BITRATE, pa);
                (m0 | m1, b0 | b1)
            }
        };
        if rc == (0, 0) {
            log(&format!(
                "h264 encoder: bitrate {} -> {} kbit/s (no restart needed)",
                self.bitrate_bps / 1000,
                bitrate_bps / 1000
            ));
            self.bitrate_bps = bitrate_bps;
            true
        } else {
            if self.last_reinit.elapsed() >= std::time::Duration::from_secs(5) {
                self.last_reinit = std::time::Instant::now();
                log(&format!(
                    "h264 encoder: SetOption bitrate {} -> {} kbit/s failed {rc:?} - keeping the old rate",
                    self.bitrate_bps / 1000,
                    bitrate_bps / 1000
                ));
            }
            false
        }
    }

    /// Is a long-term reference being kept? (false = the repair falls back to a keyframe)
    pub fn ltr_on(&self) -> bool {
        self.ltr
    }

    /// The receiver decoded the picture that marks a long-term reference. This is the other
    /// half of the loop and it is NOT optional: openh264 will only code a repair against a
    /// reference the receiver has CONFIRMED (`uiRecieveConfirmed`), so without this feedback a
    /// recovery request finds nothing usable and falls back to a keyframe - which is the very
    /// thing we are trying to avoid. `ltr_frame_num` is the receiver's
    /// DECODER_OPTION_LTR_MARKED_FRAME_NUM, `idr_pic_id` its DECODER_OPTION_IDR_PIC_ID (feedback
    /// from an older IDR period is ignored by the encoder).
    pub fn ltr_marked_ok(&mut self, idr_pic_id: u32, ltr_frame_num: i32) -> bool {
        if !self.ltr {
            return false;
        }
        use openh264_sys2::{ENCODER_LTR_MARKING_FEEDBACK, LTR_MARKING_SUCCESS, SLTRMarkingFeedback};
        let mut fb = SLTRMarkingFeedback {
            uiFeedbackType: LTR_MARKING_SUCCESS as u32,
            uiIDRPicId: idr_pic_id,
            iLTRFrameNum: ltr_frame_num,
            iLayerId: 0,
        };
        // SAFETY: the encoder is initialised and SLTRMarkingFeedback is the C struct this option
        // expects; SetOption only reads it.
        let rc = unsafe {
            self.enc.raw_api().set_option(
                ENCODER_LTR_MARKING_FEEDBACK,
                &mut fb as *mut SLTRMarkingFeedback as *mut std::os::raw::c_void,
            )
        };
        rc == 0
    }

    /// Repair the receiver without a keyframe: code the next picture against the long-term
    /// reference it still holds. `last_correct` is the frame number it last decoded (its
    /// DECODER_OPTION_LTR_MARKED_FRAME_NUM), `current` where it is now. Returns false when the
    /// encoder refuses, and then the caller should fall back to a keyframe.
    pub fn request_ltr_recovery(&mut self, idr_pic_id: u32, last_correct: i32, current: i32) -> bool {
        if !self.ltr {
            return false;
        }
        use openh264_sys2::{ENCODER_LTR_RECOVERY_REQUEST, LTR_RECOVERY_REQUEST, SLTRRecoverRequest};
        let mut req = SLTRRecoverRequest {
            uiFeedbackType: LTR_RECOVERY_REQUEST as u32,
            uiIDRPicId: idr_pic_id,
            iLastCorrectFrameNum: last_correct,
            iCurrentFrameNum: current,
            iLayerId: 0,
        };
        // SAFETY: the encoder is initialised and SLTRRecoverRequest is the C struct this option
        // expects; SetOption only reads it.
        let rc = unsafe {
            self.enc.raw_api().set_option(
                ENCODER_LTR_RECOVERY_REQUEST,
                &mut req as *mut SLTRRecoverRequest as *mut std::os::raw::c_void,
            )
        };
        if rc != 0 {
            log(&format!("h264 encoder: LTR recovery request refused ({rc})"));
        }
        rc == 0
    }

    /// Encode one frame; returns an Annex-B byte stream (may be empty on
    /// encoder error). `force_idr` inserts a keyframe (recovery request).
    pub fn encode(&mut self, frame: &RgbFrame, force_idr: bool) -> Vec<u8> {
        // Periodic keyframe as a safety net on top of on-demand IDRs.
        if force_idr || self.frames_since_idr >= 120 {
            self.enc.force_intra_frame();
            self.frames_since_idr = 0;
        }
        self.frames_since_idr += 1;
        let rgb = RgbSliceU8::new(&frame.rgb, (frame.width, frame.height));
        let yuv = YUVBuffer::from_rgb8_source(rgb);
        let out = match self.enc.encode(&yuv) {
            Ok(bs) => bs.to_vec(),
            Err(e) => {
                log(&format!("h264 encode error: {e}"));
                Vec::new()
            }
        };
        if out.is_empty() {
            return out;
        }
        self.with_sps_pps(out)
    }
}

impl VideoEncoder {
    /// Scan the NALs (start code 00 00 01 / 00 00 00 01): if an SPS (7) is present, cache the head
    /// (SPS+PPS) up to the first slice; an IDR (5) with no SPS gets the cache prepended.
    fn with_sps_pps(&mut self, out: Vec<u8>) -> Vec<u8> {
        let (mut has_sps, mut has_idr, mut first_slice) = (false, false, None);
        let mut i = 0usize;
        while i + 3 < out.len() {
            if out[i] == 0 && out[i + 1] == 0 && out[i + 2] == 1 {
                let t = out[i + 3] & 0x1F;
                let pos = if i > 0 && out[i - 1] == 0 { i - 1 } else { i };
                match t {
                    7 => has_sps = true,
                    5 => {
                        has_idr = true;
                        first_slice.get_or_insert(pos);
                    }
                    1 => {
                        first_slice.get_or_insert(pos);
                    }
                    _ => {}
                }
                i += 4;
            } else {
                i += 1;
            }
        }
        if has_sps {
            if let Some(off) = first_slice {
                if off > 0 {
                    self.sps_pps = out[..off].to_vec();
                }
            }
            out
        } else if has_idr && !self.sps_pps.is_empty() {
            let mut v = Vec::with_capacity(self.sps_pps.len() + out.len());
            v.extend_from_slice(&self.sps_pps);
            v.extend_from_slice(&out);
            v
        } else {
            out
        }
    }
}

/// What the receiver knows about long-term references, read out of the decoder after a picture.
/// These are the numbers the transmitter needs: see `VideoEncoder::ltr_marked_ok` and
/// `VideoEncoder::request_ltr_recovery`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LtrReport {
    /// which IDR period this is - the encoder ignores feedback carrying an older one
    pub idr_pic_id: u32,
    /// the frame number of the picture just decoded
    pub frame_num: i32,
    /// the newest long-term reference this decoder holds, if it holds one
    pub marked: Option<i32>,
    /// true only on the picture that marked it - that is when to send the marking feedback
    pub fresh_mark: bool,
    /// The reference chain is broken: this picture's frame number does not follow the last
    /// one, so pictures were lost. This is the ONLY honest loss signal when the decoder
    /// conceals (which is the default at the ground): a concealed picture "decodes" happily,
    /// so "the decoder refused it" never fires and the repair would never be asked for.
    /// It also re-fires by itself if the repair is lost on the way.
    pub gap: bool,
}

pub struct VideoDecoder {
    dec: Option<Decoder>,
    /// 18/9: the concealing decoder instead of the crate's strict one (see `Conceal`).
    conceal: Option<Conceal>,
    /// the long-term reference this decoder holds (v40.47)
    last_ltr: Option<i32>,
    /// the frame number of the last picture, and which IDR period it was in, to notice
    /// pictures that never arrived (see `LtrReport::gap`)
    last_fn: Option<i32>,
    last_idr: Option<u32>,
}

impl VideoDecoder {
    /// The strict decoder: a picture whose reference is missing is refused, and nothing is shown
    /// until the next key frame.
    pub fn new() -> Self {
        VideoDecoder { dec: Decoder::new().ok(), conceal: None, last_ltr: None, last_fn: None, last_idr: None }
    }

    /// The concealing decoder: a picture whose reference is missing is still built, from what the
    /// decoder has. The picture smears where the lost frame was and cleans up at the next key
    /// frame, instead of freezing until then.
    ///
    /// camera pass-through 18/9: what this is for. A camera passed through cannot be asked for a key frame, so
    /// one lost frame froze the picture for up to a whole key-frame interval (measured: every P
    /// frame refused for 1.5-2 s after one hole, 2 s of still picture at the ground).
    pub fn new_concealing() -> Self {
        VideoDecoder { dec: None, conceal: Conceal::new().ok(), last_ltr: None, last_fn: None, last_idr: None }
    }

    /// Decode one access unit; None until a decodable picture (e.g. while
    /// waiting for an IDR after loss).
    pub fn decode(&mut self, data: &[u8]) -> Option<RgbFrame> {
        self.decode_checked(data).ok().flatten()
    }

    /// `decode` that says why there was no picture: Ok(None) = the decoder wants more
    /// data, Err = the bitstream was rejected (it then waits for the next IDR).
    pub fn decode_checked(&mut self, data: &[u8]) -> Result<Option<RgbFrame>, String> {
        if let Some(c) = &mut self.conceal {
            return c.decode(data);
        }
        if self.dec.is_none() {
            self.dec = Decoder::new().ok();
        }
        let dec = self.dec.as_mut().ok_or_else(|| "no decoder".to_string())?;
        match dec.decode(data) {
            Ok(Some(yuv)) => {
                let (w, h) = yuv.dimensions();
                let mut rgb = vec![0u8; w * h * 3];
                yuv.write_rgb8(&mut rgb);
                Ok(Some(RgbFrame { width: w, height: h, rgb }))
            }
            Ok(None) => Ok(None),
            Err(e) => Err(e.to_string()),
        }
    }

    /// One GetOption on whichever decoder this is (the crate's or the concealing one).
    fn opt_i32(&mut self, id: openh264_sys2::DECODER_OPTION) -> Option<i32> {
        let mut v: i32 = 0;
        let p = (&mut v as *mut i32).cast::<std::os::raw::c_void>();
        // SAFETY: both decoders are initialised here and every one of these options writes a
        // single int through the pointer.
        let rc = unsafe {
            if let Some(c) = &mut self.conceal {
                let f = (**c.dec).GetOption?;
                f(c.dec, id, p)
            } else if let Some(d) = &mut self.dec {
                d.raw_api().get_option(id, p)
            } else {
                return None;
            }
        };
        (rc == 0).then_some(v)
    }

    /// Where this decoder is, in the terms the encoder's long-term reference machinery speaks.
    /// Call it right after a picture comes out; `fresh_mark` says this picture marked a new
    /// long-term reference, which is when the transmitter wants to hear about it.
    pub fn ltr(&mut self) -> Option<LtrReport> {
        use openh264_sys2::{
            DECODER_OPTION_FRAME_NUM, DECODER_OPTION_IDR_PIC_ID, DECODER_OPTION_LTR_MARKED_FRAME_NUM,
            DECODER_OPTION_LTR_MARKING_FLAG,
        };
        let frame_num = self.opt_i32(DECODER_OPTION_FRAME_NUM)?;
        let idr_pic_id = self.opt_i32(DECODER_OPTION_IDR_PIC_ID)? as u32;
        let mut fresh_mark = false;
        if self.opt_i32(DECODER_OPTION_LTR_MARKING_FLAG).unwrap_or(0) != 0 {
            if let Some(n) = self.opt_i32(DECODER_OPTION_LTR_MARKED_FRAME_NUM) {
                fresh_mark = self.last_ltr != Some(n);
                self.last_ltr = Some(n);
            }
        }
        // frame_num counts coded pictures and wraps at 1 << 15 (openh264 writes
        // log2_max_frame_num = 15); every picture here is a reference, so the step is
        // exactly one. A different step means pictures never arrived. A new IDR period
        // restarts the count, and the reference held before it is gone with it.
        let new_idr = self.last_idr != Some(idr_pic_id);
        let gap = match self.last_fn {
            Some(prev) if !new_idr => !matches!((frame_num - prev) & 0x7FFF, 0 | 1),
            _ => false,
        };
        if new_idr {
            self.last_idr = Some(idr_pic_id);
            self.last_ltr = None;
        }
        self.last_fn = Some(frame_num);
        Some(LtrReport { idr_pic_id, frame_num, marked: self.last_ltr, fresh_mark, gap })
    }

    /// A new IDR period starts from nothing: the reference held before it is gone.
    pub fn forget_ltr(&mut self) {
        self.last_ltr = None;
    }
}

/// OpenH264 with error concealment on. The openh264 crate builds its decoder with concealment
/// DISABLED and exposes neither the setting nor the decoder pointer, so this drives the C API
/// itself (openh264-sys2): create, initialise with `eEcActiveIdc`, decode, convert.
struct Conceal {
    dec: *mut openh264_sys2::ISVCDecoder,
}

// The decoder is used from one thread at a time (each consumer owns its VideoDecoder); the raw
// pointer is what OpenH264 hands out and carries no thread affinity of its own.
unsafe impl Send for Conceal {}

impl Conceal {
    fn new() -> Result<Conceal, String> {
        use openh264_sys2::{
            ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE, SDecodingParam, SVideoProperty, VIDEO_BITSTREAM_DEFAULT,
            source::APILoader,
        };
        let mut dec: *mut openh264_sys2::ISVCDecoder = std::ptr::null_mut();
        // SAFETY: the C API fills `dec` on success; every call below goes through its vtable,
        // which is non-null once the decoder exists.
        unsafe {
            if APILoader::WelsCreateDecoder(&mut dec) != 0 || dec.is_null() {
                return Err("WelsCreateDecoder failed".into());
            }
            let param = SDecodingParam {
                pFileNameRestructed: std::ptr::null_mut(),
                uiCpuLoad: 0,
                uiTargetDqLayer: 0,
                eEcActiveIdc: ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE,
                bParseOnly: false,
                sVideoProperty: SVideoProperty {
                    size: std::mem::size_of::<SVideoProperty>() as u32,
                    eVideoBsType: VIDEO_BITSTREAM_DEFAULT,
                },
            };
            let init = (**dec).Initialize.ok_or("decoder without Initialize")?;
            if init(dec, &param) != 0 {
                APILoader::WelsDestroyDecoder(dec);
                return Err("decoder Initialize failed".into());
            }
        }
        log("h264 decoder: error concealment on (a lost reference smears instead of freezing)");
        Ok(Conceal { dec })
    }

    fn decode(&mut self, data: &[u8]) -> Result<Option<RgbFrame>, String> {
        use openh264_sys2::SBufferInfo;
        let mut info: SBufferInfo = unsafe { std::mem::zeroed() };
        let mut dst: [*mut u8; 3] = [std::ptr::null_mut(); 3];
        // SAFETY: `data` is a valid slice; the decoder writes pointers into its own picture buffer
        // (valid until the next decode call) and the sizes into `info`.
        let rc = unsafe {
            let f = (**self.dec).DecodeFrameNoDelay.ok_or("decoder without DecodeFrameNoDelay")?;
            f(self.dec, data.as_ptr(), data.len() as i32, dst.as_mut_ptr(), &mut info)
        };
        if info.iBufferStatus != 1 {
            // no picture: either it wants more data (rc == 0) or it could not use this one
            return if rc == 0 { Ok(None) } else { Err(format!("decode state {rc}")) };
        }
        // SAFETY: iBufferStatus == 1 means the three planes and the sizes below are filled in.
        let buf = unsafe { info.UsrData.sSystemBuffer };
        let (w, h) = (buf.iWidth.max(0) as usize, buf.iHeight.max(0) as usize);
        let (ys, uvs) = (buf.iStride[0].max(0) as usize, buf.iStride[1].max(0) as usize);
        if w == 0 || h == 0 || ys < w || uvs * 2 < w || dst[0].is_null() || dst[1].is_null() || dst[2].is_null() {
            return Err("decoded picture of odd shape".into());
        }
        // SAFETY: the planes hold at least stride x height (chroma: stride x height/2) bytes.
        let (y, u, v) = unsafe {
            (
                std::slice::from_raw_parts(dst[0], ys * h),
                std::slice::from_raw_parts(dst[1], uvs * h.div_ceil(2)),
                std::slice::from_raw_parts(dst[2], uvs * h.div_ceil(2)),
            )
        };
        let p = crate::source::Yuv420 { y, y_stride: ys, u, v, uv_stride: uvs, uv_pixel_stride: 1 };
        Ok(Some(crate::source::yuv420_to_rgb(&p, w, h, false)))
    }
}

impl Drop for Conceal {
    fn drop(&mut self) {
        // SAFETY: the decoder was created by the same API and is not used after this.
        unsafe {
            if let Some(uninit) = (**self.dec).Uninitialize {
                uninit(self.dec);
            }
            openh264_sys2::source::APILoader::WelsDestroyDecoder(self.dec);
        }
    }
}

impl Default for VideoDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::PatternGen;

    /// One frame lost: the strict decoder shows nothing until the next key frame, the concealing
    /// one keeps producing pictures. This is the whole point of `new_concealing` (camera pass-through, 18/9: a
    /// camera passed through cannot be asked for a key frame, so a hole froze the picture for up
    /// to a key-frame interval).
    /// The point of a long-term reference: after a loss, repair the picture with a frame that
    /// points at one the receiver still holds, instead of a whole keyframe. At MCS0 the link
    /// carries 131 kbit/s and a picture is a single block, so the difference is a freeze of a
    /// few hundred ms or none (measured 19/9, see the rate-control notes).
    ///
    /// This runs the whole loop, because half of it is not optional: openh264 only repairs
    /// against a reference the RECEIVER has confirmed, so the test decodes what it encodes and
    /// feeds the confirmations back, exactly as the link will.
    #[test]
    fn ltr_repairs_without_a_keyframe() {
        let (w, h) = (320, 240);
        let mut enc = VideoEncoder::new_with_intra(w, h, 300_000, 30.0, 600).expect("encoder");
        assert!(enc.ltr_on(), "openh264 would not take a long-term reference");
        let mut dec = VideoDecoder::new();

        // a still scene with one small square moving across it: an aircraft holding station,
        // which is the case a long-term reference is good for (a picture from a second ago is
        // still most of the answer).
        let frame = |n: usize| {
            let mut rgb = vec![0u8; w * h * 3];
            for y in 0..h {
                for x in 0..w {
                    let i = (y * w + x) * 3;
                    let v = (((x / 16) + (y / 16)) % 2) as u8 * 40 + 60;
                    rgb[i] = v;
                    rgb[i + 1] = v;
                    rgb[i + 2] = v;
                }
            }
            let (bx, by) = (8 + (n * 3) % (w - 40), h / 2);
            for y in by..by + 24 {
                for x in bx..bx + 24 {
                    let i = (y * w + x) * 3;
                    rgb[i] = 240;
                    rgb[i + 1] = 40;
                    rgb[i + 2] = 40;
                }
            }
            RgbFrame { width: w, height: h, rgb }
        };

        // the link running normally: everything arrives, and the receiver confirms every
        // long-term reference it is given
        let idr = enc.encode(&frame(0), true);
        dec.decode(&idr).expect("the first picture");
        let mut here = dec.ltr().expect("the decoder reports where it is");
        let mut sizes = Vec::new();
        for n in 1..=45 {
            let bs = enc.encode(&frame(n), false);
            sizes.push(bs.len());
            dec.decode(&bs);
            here = dec.ltr().expect("a report per picture");
            if here.fresh_mark {
                assert!(
                    enc.ltr_marked_ok(here.idr_pic_id, here.marked.unwrap()),
                    "the encoder refused the marking feedback"
                );
            }
        }
        let marked = here.marked.expect("the receiver should hold a long-term reference by now");
        let steady = sizes[sizes.len() - 8..].iter().sum::<usize>() / 8;

        // now lose everything for half a second: the transmitter keeps encoding, the receiver
        // gets none of it, and it is left with the reference it confirmed
        for n in 46..=60 {
            let _ = enc.encode(&frame(n), false);
        }

        // the receiver asks for a repair from where it actually is
        assert!(
            enc.request_ltr_recovery(here.idr_pic_id, marked, here.frame_num),
            "the encoder refused the recovery request"
        );
        let repair = enc.encode(&frame(61), false);
        assert!(!repair.is_empty(), "the repair frame was empty");

        // it must be a repair, not a keyframe: no IDR slice and no parameter sets
        let mut i = 0;
        let (mut has_idr, mut has_sps) = (false, false);
        while i + 3 < repair.len() {
            if repair[i] == 0 && repair[i + 1] == 0 && repair[i + 2] == 1 {
                match repair[i + 3] & 0x1F {
                    5 => has_idr = true,
                    7 => has_sps = true,
                    _ => {}
                }
                i += 4;
            } else {
                i += 1;
            }
        }
        assert!(!has_idr && !has_sps, "the repair was a keyframe ({} B)", repair.len());
        assert!(
            repair.len() < idr.len(),
            "the repair cost {} B against a keyframe's {} B",
            repair.len(),
            idr.len()
        );

        println!(
            "keyframe {} B, steady frame {steady} B, LTR repair {} B",
            idr.len(),
            repair.len()
        );

        // and it really repairs: the receiver, which missed 15 frames, decodes a picture again
        let got = dec.decode(&repair).expect("the repair did not decode after the loss");
        assert_eq!((got.width, got.height), (w, h));
        // the loss itself is visible in the frame numbers - the signal the receiver uses,
        // because a concealing decoder never refuses a picture
        assert!(dec.ltr().expect("a report").gap, "15 lost pictures left no gap in the frame numbers");
        assert!(repair.len() > steady / 4, "the repair {} B looks empty (steady {steady} B)", repair.len());
    }

    #[test]
    fn concealing_decoder_keeps_pictures_after_a_lost_frame() {
        let mut pattern = PatternGen::new();
        let mut enc = VideoEncoder::new_with_intra(320, 240, 600_000, 30.0, 300).expect("encoder");
        let mut frames = Vec::new();
        for _ in 0..24 {
            let f = pattern.render(320, 240);
            let au = enc.encode(&f, false);
            if !au.is_empty() {
                frames.push(au);
            }
        }
        assert!(frames.len() > 12, "encoder produced {} frames", frames.len());
        let feed = |dec: &mut VideoDecoder| {
            let mut pics = 0;
            for (i, au) in frames.iter().enumerate() {
                if i == 8 {
                    continue; // the frame lost on air
                }
                if dec.decode_checked(au).ok().flatten().is_some() && i > 8 {
                    pics += 1;
                }
            }
            pics
        };
        let strict = feed(&mut VideoDecoder::new());
        let concealing = feed(&mut VideoDecoder::new_concealing());
        let after = frames.len() - 9;
        assert!(
            concealing > strict && concealing * 2 >= after,
            "after the hole: concealing {concealing}, strict {strict}, of {after} frames"
        );
    }

    #[test]
    fn h264_roundtrip_and_compression() {
        let mut pattern = PatternGen::new();
        let mut enc = VideoEncoder::new(480, 360, 1_000_000, 30.0).expect("encoder");
        let mut dec = VideoDecoder::new();
        let mut total = 0usize;
        let mut decoded_frames = 0;
        for i in 0..30 {
            let frame = pattern.render(480, 360);
            let bytes = enc.encode(&frame, i == 0);
            assert!(!bytes.is_empty(), "frame {i} produced no bitstream");
            total += bytes.len();
            if let Some(out) = dec.decode(&bytes) {
                assert_eq!((out.width, out.height), (480, 360));
                decoded_frames += 1;
            }
        }
        assert!(decoded_frames >= 25, "decoded only {decoded_frames}/30");
        // ~1 Mbit/s at 30 frames -> ~4 KB/frame average; JPEG q60 of this
        // pattern is ~12 KB. Loose sanity bound:
        let avg = total / 30;
        assert!(avg < 12_000, "h264 avg frame {avg} B not smaller than JPEG");
    }
}
