//! Stand-in for `codec` in a build without openh264 (the on-board camera app):
//! the same types, so the transmit worker compiles unchanged, but nothing encodes or decodes.
//! That build only sends the camera's own H.264 (pass-through) or MJPEG.

use crate::RgbFrame;

pub struct VideoEncoder;

impl VideoEncoder {
    pub fn new(_width: usize, _height: usize, _bitrate_bps: u32, _fps: f32) -> Result<Self, String> {
        Err("built without H.264 (openh264)".into())
    }

    pub fn new_with_intra(
        _width: usize,
        _height: usize,
        _bitrate_bps: u32,
        _fps: f32,
        _intra: u32,
    ) -> Result<Self, String> {
        Err("built without H.264 (openh264)".into())
    }

    pub fn ensure(&mut self, _width: usize, _height: usize, _bitrate_bps: u32, _fps: f32) {}

    pub fn set_fps(&mut self, _fps: f32) {}

    pub fn set_bitrate(&mut self, _bitrate_bps: u32) -> bool {
        false
    }

    pub fn encode(&mut self, _frame: &RgbFrame, _force_idr: bool) -> Vec<u8> {
        Vec::new()
    }
}

pub struct VideoDecoder;

impl VideoDecoder {
    pub fn new() -> Self {
        VideoDecoder
    }

    pub fn decode(&mut self, _data: &[u8]) -> Option<RgbFrame> {
        None
    }

    pub fn decode_checked(&mut self, _data: &[u8]) -> Result<Option<RgbFrame>, String> {
        Ok(None)
    }
}

impl Default for VideoDecoder {
    fn default() -> Self {
        Self::new()
    }
}
