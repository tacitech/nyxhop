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
        let enc = Encoder::with_api_config(OpenH264API::from_source(), config)
            .map_err(|e| e.to_string())?;
        Ok(VideoEncoder {
            enc, width, height, bitrate_bps, fps, frames_since_idr: 0,
            last_reinit: std::time::Instant::now(),
            sps_pps: Vec::new(),
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

pub struct VideoDecoder {
    dec: Option<Decoder>,
}

impl VideoDecoder {
    pub fn new() -> Self {
        VideoDecoder { dec: Decoder::new().ok() }
    }

    /// Decode one access unit; None until a decodable picture (e.g. while
    /// waiting for an IDR after loss).
    pub fn decode(&mut self, data: &[u8]) -> Option<RgbFrame> {
        if self.dec.is_none() {
            self.dec = Decoder::new().ok();
        }
        let dec = self.dec.as_mut()?;
        match dec.decode(data) {
            Ok(Some(yuv)) => {
                let (w, h) = yuv.dimensions();
                let mut rgb = vec![0u8; w * h * 3];
                yuv.write_rgb8(&mut rgb);
                Some(RgbFrame { width: w, height: h, rgb })
            }
            Ok(None) => None,
            Err(_) => {
                // Bitstream corrupted mid-GOP; wait for the next IDR.
                None
            }
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
