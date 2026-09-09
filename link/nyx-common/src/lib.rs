//! nyx-common: shared pieces for the three NyxHop processes — logging,
//! video sources, JPEG codec helpers and egui plot widgets.

pub mod boardctl;
#[cfg(feature = "h264")]
pub mod codec;
pub mod control;
/// v40.33 licence rows shared by the apps.
pub mod licpanel;
/// v40.34 shared touch-first user interface (HUD, drawer, sections).
pub mod ui;
pub mod logging;
pub mod plots;
pub mod rng;
pub mod source;
pub mod theme;

/// An RGB8 frame passed between threads and to the GUI.
#[derive(Clone)]
pub struct RgbFrame {
    pub width: usize,
    pub height: usize,
    pub rgb: Vec<u8>,
}

pub fn encode_jpeg(frame: &RgbFrame, quality: u8) -> Vec<u8> {
    let mut buf = std::io::Cursor::new(Vec::new());
    let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut buf, quality);
    enc.encode(
        &frame.rgb,
        frame.width as u32,
        frame.height as u32,
        image::ExtendedColorType::Rgb8,
    )
    .expect("jpeg encode");
    buf.into_inner()
}

pub fn decode_jpeg(data: &[u8]) -> Option<RgbFrame> {
    let img = image::load_from_memory_with_format(data, image::ImageFormat::Jpeg).ok()?;
    let rgb = img.to_rgb8();
    let (w, h) = rgb.dimensions();
    Some(RgbFrame { width: w as usize, height: h as usize, rgb: rgb.into_raw() })
}

pub fn to_color_image(f: &RgbFrame) -> egui::ColorImage {
    egui::ColorImage::from_rgb([f.width, f.height], &f.rgb)
}
