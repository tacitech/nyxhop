//! Video/data sources: animated test pattern, webcam capture, random data.

use crate::RgbFrame;
use crate::rng::Rng64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceKind {
    Pattern,
    Webcam,
    RandomData,
}

impl SourceKind {
    pub fn label(self) -> &'static str {
        match self {
            SourceKind::Pattern => "Test pattern",
            SourceKind::Webcam => "Webcam",
            SourceKind::RandomData => "Random data",
        }
    }
}

/// Animated test pattern: SMPTE-ish color bars, moving gradient, bouncing
/// ball and a binary frame counter strip.
pub struct PatternGen {
    frame_no: u64,
}

impl PatternGen {
    pub fn new() -> Self {
        PatternGen { frame_no: 0 }
    }

    pub fn render(&mut self, w: usize, h: usize) -> RgbFrame {
        const BARS: [[u8; 3]; 7] = [
            [192, 192, 192],
            [192, 192, 0],
            [0, 192, 192],
            [0, 192, 0],
            [192, 0, 192],
            [192, 0, 0],
            [0, 0, 192],
        ];
        let t = self.frame_no as f32;
        let mut rgb = vec![0u8; w * h * 3];
        let bars_h = h * 55 / 100;
        let grad_h = h * 25 / 100;

        for y in 0..h {
            for x in 0..w {
                let i = (y * w + x) * 3;
                if y < bars_h {
                    let b = (x * 7 / w).min(6);
                    rgb[i] = BARS[b][0];
                    rgb[i + 1] = BARS[b][1];
                    rgb[i + 2] = BARS[b][2];
                } else if y < bars_h + grad_h {
                    let v = ((x as f32 / w as f32 * 255.0) + t * 3.0) % 255.0;
                    rgb[i] = v as u8;
                    rgb[i + 1] = ((v * 0.5) as u8).wrapping_add(60);
                    rgb[i + 2] = 255 - v as u8;
                } else {
                    let cell = x * 32 / w;
                    let bit = (self.frame_no >> (31 - cell)) & 1;
                    let v = if bit == 1 { 230 } else { 25 };
                    rgb[i] = v;
                    rgb[i + 1] = v;
                    rgb[i + 2] = v;
                }
            }
        }

        let r = (h as f32 * 0.07).max(6.0);
        let cx = (w as f32 - 2.0 * r) * (0.5 + 0.5 * (t * 0.13).sin()) + r;
        let cy = (h as f32 - 2.0 * r) * (0.5 + 0.5 * (t * 0.19).cos()) + r;
        let (x0, x1) = ((cx - r).max(0.0) as usize, ((cx + r) as usize).min(w - 1));
        let (y0, y1) = ((cy - r).max(0.0) as usize, ((cy + r) as usize).min(h - 1));
        for y in y0..=y1 {
            for x in x0..=x1 {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                if dx * dx + dy * dy <= r * r {
                    let i = (y * w + x) * 3;
                    rgb[i] = 255;
                    rgb[i + 1] = 90;
                    rgb[i + 2] = 0;
                }
            }
        }

        self.frame_no += 1;
        RgbFrame { width: w, height: h, rgb }
    }
}

impl Default for PatternGen {
    fn default() -> Self {
        Self::new()
    }
}

/// Nearest-neighbour resize (avoids coupling to nokhwa's image version).
pub fn resize_rgb(src: &RgbFrame, dw: usize, dh: usize) -> RgbFrame {
    if src.width == dw && src.height == dh {
        return src.clone();
    }
    let mut rgb = vec![0u8; dw * dh * 3];
    for y in 0..dh {
        let sy = y * src.height / dh;
        for x in 0..dw {
            let sx = x * src.width / dw;
            let si = (sy * src.width + sx) * 3;
            let di = (y * dw + x) * 3;
            rgb[di..di + 3].copy_from_slice(&src.rgb[si..si + 3]);
        }
    }
    RgbFrame { width: dw, height: dh, rgb }
}

/// Random payload generator for "arbitrary data" mode.
pub struct DataGen {
    rng: Rng64,
    pub block_len: usize,
}

impl DataGen {
    pub fn new() -> Self {
        DataGen { rng: Rng64::new(0xDA7A_0001), block_len: 24 * 1024 }
    }

    pub fn next_block(&mut self) -> Vec<u8> {
        (0..self.block_len).map(|_| self.rng.next_u64() as u8).collect()
    }
}

impl Default for DataGen {
    fn default() -> Self {
        Self::new()
    }
}

/// Visualize an arbitrary byte blob as an RGB image.
pub fn bytes_to_image(data: &[u8], w: usize) -> RgbFrame {
    let px = data.len() / 3;
    let h = (px / w).max(1);
    let mut rgb = vec![0u8; w * h * 3];
    let n = rgb.len().min(data.len());
    rgb[..n].copy_from_slice(&data[..n]);
    RgbFrame { width: w, height: h, rgb }
}

// ---------------------------------------------------------------- webcam --

/// Shared state between the webcam capture thread and its consumer.
pub struct WebcamShared {
    pub wanted: std::sync::atomic::AtomicBool,
    pub stop: std::sync::atomic::AtomicBool,
    pub frame: std::sync::Mutex<Option<RgbFrame>>,
    pub status: std::sync::Mutex<String>,
}

impl WebcamShared {
    pub fn new() -> std::sync::Arc<Self> {
        std::sync::Arc::new(WebcamShared {
            wanted: std::sync::atomic::AtomicBool::new(false),
            stop: std::sync::atomic::AtomicBool::new(false),
            frame: std::sync::Mutex::new(None),
            status: std::sync::Mutex::new(String::from("idle")),
        })
    }

    pub fn status(&self) -> String {
        self.status.lock().unwrap().clone()
    }
}

#[cfg(feature = "webcam")]
pub mod webcam {
    use std::sync::Arc;
    use std::sync::atomic::Ordering;
    use std::time::Duration;

    use nokhwa::Camera;
    use nokhwa::pixel_format::RgbFormat;
    use nokhwa::utils::{
        CameraFormat, CameraIndex, FrameFormat, RequestedFormat,
        RequestedFormatType, Resolution,
    };

    use super::WebcamShared;
    use crate::RgbFrame;
    use crate::logging::log;

    /// Background thread: opens the camera only while `wanted` is set.
    pub fn spawn(shared: Arc<WebcamShared>) {
        std::thread::Builder::new()
            .name("webcam".into())
            .spawn(move || run(shared))
            .expect("spawn webcam thread");
    }

    fn set_status(shared: &WebcamShared, s: String) {
        log(&format!("webcam: {s}"));
        *shared.status.lock().unwrap() = s;
    }

    fn run(shared: Arc<WebcamShared>) {
        let mut camera: Option<Camera> = None;
        // v-lat: measure capture + decode rate to know whether the loop KEEPS UP with the camera
        // (too slow -> the driver buffer grows -> old frames -> delay).
        let mut cap_cnt = 0u64;
        let mut dec_us = 0u64;
        let mut grab_sum = 0u64;
        let mut report = std::time::Instant::now();
        loop {
            if shared.stop.load(Ordering::Relaxed) {
                return;
            }
            let wanted = shared.wanted.load(Ordering::Relaxed);
            if !wanted {
                if camera.is_some() {
                    camera = None;
                    set_status(&shared, "idle".into());
                }
                std::thread::sleep(Duration::from_millis(150));
                continue;
            }
            if camera.is_none() {
                // v-lat: choose a LOW-LATENCY webcam mode. AbsoluteHighestFrameRate (old) picked
                // the HIGHEST resolution mode; on this camera that is 2560x1440, RGB decode ~50
                // ms/frame -> the loop could NOT keep up -> the driver buffer grew -> OLD frames ->
                // delayed video (G2G in->out was only ~26 ms; the delay sat in capture). A specific
                // mode (Closest 640x480@30) was refused by MediaFoundation. -> PROBE: open
                // temporarily, LIST the modes the camera really supports, choose a LIGHT one (res
                // <= 1280 for fast decode, fps >= 24 for smoothness, the smallest such res), reopen
                // with Exact. The consumer downscales to 480x360. This camera is NV12 (Closest
                // MJPEG/YUYV REFUSED by MediaFoundation). Ask for 848x480 NV12 30 fps (near the
                // 480x360 output; decode measured at 1 ms). Closest includes fps in its distance,
                // so take the 30 fps variant (Exact tends to match the 1 fps variant). FALLBACK
                // AbsoluteHighestFrameRate (always opens: never lose the webcam).
                let want = RequestedFormat::new::<RgbFormat>(
                    RequestedFormatType::Closest(CameraFormat::new(
                        Resolution::new(848, 480), FrameFormat::NV12, 30)));
                let cam = Camera::new(CameraIndex::Index(0), want).or_else(|_| {
                    let fb = RequestedFormat::new::<RgbFormat>(
                        RequestedFormatType::AbsoluteHighestFrameRate);
                    Camera::new(CameraIndex::Index(0), fb)
                });
                match cam {
                    Ok(mut cam) => {
                        if cam.open_stream().is_ok() {
                            set_status(&shared, format!(
                                "open {}x{}@{}fps: {}",
                                cam.resolution().width_x,
                                cam.resolution().height_y,
                                cam.frame_rate(),
                                cam.info().human_name()));
                            camera = Some(cam);
                        } else {
                            set_status(&shared, "stream error".into());
                            std::thread::sleep(Duration::from_secs(2));
                        }
                    }
                    Err(e) => {
                        set_status(&shared, format!("open error: {e}"));
                        std::thread::sleep(Duration::from_secs(2));
                    }
                }
                continue;
            }
            let cam = camera.as_mut().unwrap();
            let t0 = std::time::Instant::now();
            let raw = cam.frame();
            let grab_us = t0.elapsed().as_micros() as u64;
            let t1 = std::time::Instant::now();
            match raw.and_then(|f| f.decode_image::<RgbFormat>()) {
                Ok(img) => {
                    dec_us += t1.elapsed().as_micros() as u64;
                    grab_sum += grab_us;
                    cap_cnt += 1;
                    let (w, h) = img.dimensions();
                    let frame = RgbFrame {
                        width: w as usize,
                        height: h as usize,
                        rgb: img.into_raw(),
                    };
                    *shared.frame.lock().unwrap() = Some(frame);
                    let el = report.elapsed().as_secs_f32();
                    if el >= 2.0 {
                        let n = cap_cnt.max(1);
                        set_status(&shared, format!(
                            "cap {:.0}fps {}x{} grab {}ms decode {}ms",
                            cap_cnt as f32 / el, w, h,
                            grab_sum / 1000 / n, dec_us / 1000 / n));
                        cap_cnt = 0; dec_us = 0; grab_sum = 0;
                        report = std::time::Instant::now();
                    }
                }
                Err(e) => {
                    set_status(&shared, format!("frame error: {e}"));
                    camera = None;
                    std::thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }
}
