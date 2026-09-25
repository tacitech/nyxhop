//! nyx-common: shared pieces for the three NyxHop processes — logging,
//! video sources, JPEG codec helpers and egui plot widgets.

pub mod boardctl;
#[cfg(feature = "h264")]
pub mod codec;
#[cfg(not(feature = "h264"))]
#[path = "codec_none.rs"]
pub mod codec;
pub mod control;
/// v40.33 licence rows shared by the apps.
pub mod licpanel;
/// v40.34 shared touch-first user interface (HUD, drawer, sections).
pub mod ui;
pub mod logging;
#[cfg(feature = "onvif")]
pub mod onvif;
pub mod plots;
pub mod rng;
pub mod source;
/// v40.46 latency test: the millisecond counter (burned in at the aircraft, shown at the ground).
pub mod stamp;
/// The start screen of the combined apps (which end, which board, put it into the role).
pub mod start;
pub mod theme;

/// v40.49: the SNR a receiver reports back is measured on frames that arrive (on a board: blocks
/// that decode), so in a fade where none does it froze at the last good value. The simulator's 5 dB
/// fade reported 10 dB for 8 s, and the sender took the loss for interference and held a rung
/// that delivered nothing. After 300 ms without a frame the reading falls 30 dB per second, to 0
/// (10 dB/s at first: a sudden -24 dB fade on the bench took 3 s to reach the rate control).
pub struct SnrStarve {
    last_frame: std::time::Instant,
    tick: std::time::Instant,
}

impl Default for SnrStarve {
    fn default() -> Self {
        let now = std::time::Instant::now();
        SnrStarve { last_frame: now, tick: now }
    }
}

impl SnrStarve {
    /// A frame arrived (its SNR is folded in by the caller).
    pub fn frame(&mut self) {
        self.last_frame = std::time::Instant::now();
    }

    /// Call often (every frame and every idle wake-up); lowers `snr` while starving.
    pub fn apply(&mut self, snr: &mut f32) {
        let now = std::time::Instant::now();
        let dt = now.duration_since(self.tick).as_secs_f32();
        self.tick = now;
        if now.duration_since(self.last_frame).as_millis() > 300 && *snr > 0.0 {
            *snr = (*snr - 30.0 * dt).max(0.0);
        }
    }
}

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

/// Command-line style options an app runs with. The standalone binaries take them from the
/// process (`from_env`); the combined app makes them up after the operator's choice
/// (`from_list`), so the same `setup()` serves both.
#[derive(Clone, Debug, Default)]
pub struct Opts {
    args: Vec<String>,
}

impl Opts {
    pub fn from_env() -> Self {
        Opts { args: std::env::args().skip(1).collect() }
    }

    pub fn from_list<S: Into<String>>(args: impl IntoIterator<Item = S>) -> Self {
        Opts { args: args.into_iter().map(Into::into).collect() }
    }

    /// `--name value`, or the default.
    pub fn arg(&self, name: &str, default: &str) -> String {
        self.args
            .iter()
            .position(|a| a == name)
            .and_then(|i| self.args.get(i + 1))
            .cloned()
            .unwrap_or_else(|| default.to_string())
    }

    /// `--name` present at all.
    pub fn flag(&self, name: &str) -> bool {
        self.args.iter().any(|a| a == name)
    }
}
