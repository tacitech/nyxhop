//! Video/data sources: animated test pattern, webcam capture, random data.

use crate::RgbFrame;
use crate::rng::Rng64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceKind {
    Pattern,
    Webcam,
    /// An IP camera: H.264 over RTSP, decoded here and fed on like the webcam.
    Rtsp,
    RandomData,
}

impl SourceKind {
    pub fn label(self) -> &'static str {
        match self {
            SourceKind::Pattern => "Test pattern",
            // the same slot: on the phone its own camera feeds it (nyx-mobile), on the PC nokhwa
            SourceKind::Webcam => if cfg!(target_os = "android") { "Phone camera" } else { "Webcam" },
            SourceKind::Rtsp => "IP camera (RTSP)",
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
    /// The capture size the host would like, `(w << 32) | h`; 0 = the source's own choice.
    /// The phone camera opens the smallest mode that covers it.
    pub want_wh: std::sync::atomic::AtomicU64,
}

impl WebcamShared {
    pub fn new() -> std::sync::Arc<Self> {
        std::sync::Arc::new(WebcamShared {
            wanted: std::sync::atomic::AtomicBool::new(false),
            stop: std::sync::atomic::AtomicBool::new(false),
            frame: std::sync::Mutex::new(None),
            status: std::sync::Mutex::new(String::from("idle")),
            want_wh: std::sync::atomic::AtomicU64::new(0),
        })
    }

    pub fn status(&self) -> String {
        self.status.lock().unwrap().clone()
    }

    pub fn set_want(&self, w: usize, h: usize) {
        self.want_wh.store(((w as u64) << 32) | (h as u64 & 0xffff_ffff), std::sync::atomic::Ordering::Relaxed);
    }

    pub fn want(&self) -> Option<(usize, usize)> {
        let v = self.want_wh.load(std::sync::atomic::Ordering::Relaxed);
        (v != 0).then(|| ((v >> 32) as usize, (v & 0xffff_ffff) as usize))
    }
}

// ------------------------------------------------------------------- yuv --

/// The three planes of a YUV 4:2:0 picture as a camera hands them over (Android's
/// YUV_420_888: each plane with its own row stride, chroma possibly interleaved with a
/// pixel stride of 2).
pub struct Yuv420<'a> {
    pub y: &'a [u8],
    pub y_stride: usize,
    pub u: &'a [u8],
    pub v: &'a [u8],
    pub uv_stride: usize,
    pub uv_pixel_stride: usize,
}

/// YUV 4:2:0 (BT.601, limited range) -> RGB8, optionally turned by 180 degrees (a phone
/// held the other way round: the sensor does not turn with the screen).
pub fn yuv420_to_rgb(p: &Yuv420, w: usize, h: usize, rotate180: bool) -> RgbFrame {
    let mut rgb = vec![0u8; w * h * 3];
    let clip = |v: i32| v.clamp(0, 255) as u8;
    for y in 0..h {
        let yrow = &p.y[y * p.y_stride..y * p.y_stride + w];
        let uvrow = (y / 2) * p.uv_stride;
        // the output row: the same row, or the mirrored one when turning
        let orow = if rotate180 { h - 1 - y } else { y };
        for x in 0..w {
            let c = i32::from(yrow[x]) - 16;
            let ci = uvrow + (x / 2) * p.uv_pixel_stride;
            let d = i32::from(p.u[ci]) - 128;
            let e = i32::from(p.v[ci]) - 128;
            let r = clip((298 * c + 409 * e + 128) >> 8);
            let g = clip((298 * c - 100 * d - 208 * e + 128) >> 8);
            let b = clip((298 * c + 516 * d + 128) >> 8);
            let ox = if rotate180 { w - 1 - x } else { x };
            let o = (orow * w + ox) * 3;
            rgb[o] = r;
            rgb[o + 1] = g;
            rgb[o + 2] = b;
        }
    }
    RgbFrame { width: w, height: h, rgb }
}

#[cfg(test)]
mod yuv_tests {
    use super::*;

    #[test]
    fn grey_and_red_pixels_come_out_right() {
        // 4x2: left half mid grey (Y 128, U/V 128), right half red (Y 81, U 90, V 240)
        let y = [128u8, 128, 81, 81, 128, 128, 81, 81];
        let u = [128u8, 90];
        let v = [128u8, 240];
        let p = Yuv420 { y: &y, y_stride: 4, u: &u, v: &v, uv_stride: 2, uv_pixel_stride: 1 };
        let f = yuv420_to_rgb(&p, 4, 2, false);
        assert_eq!(&f.rgb[0..3], &[130, 130, 130]);
        let red = &f.rgb[6..9];
        assert!(red[0] > 230 && red[1] < 30 && red[2] < 30, "{red:?}");
        // turned by 180 degrees the red half comes first
        let t = yuv420_to_rgb(&p, 4, 2, true);
        assert!(t.rgb[0] > 230 && t.rgb[1] < 30, "{:?}", &t.rgb[0..3]);
        assert_eq!(&t.rgb[6..9], &[130, 130, 130]);
    }

    #[test]
    fn interleaved_chroma_with_pixel_stride_two() {
        // NV12-like: U and V interleaved, pixel stride 2, the V slice starts one byte later
        let y = [128u8, 128, 128, 128];
        let uv = [128u8, 128, 90, 240]; // (u,v) for the left pair, (u,v) for the right pair
        let p = Yuv420 { y: &y, y_stride: 4, u: &uv[0..], v: &uv[1..], uv_stride: 4, uv_pixel_stride: 2 };
        let f = yuv420_to_rgb(&p, 4, 1, false);
        assert_eq!(&f.rgb[0..3], &[130, 130, 130]);
        assert!(f.rgb[6] > f.rgb[0], "right pair should lean red: {:?}", &f.rgb[6..9]);
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

// ------------------------------------------------------------------ rtsp --

/// Shared state between the RTSP client thread and its consumer: the URL to pull,
/// whether anyone wants it, the latest decoded picture, and a status line.
pub struct RtspShared {
    pub url: std::sync::Mutex<String>,
    pub wanted: std::sync::atomic::AtomicBool,
    pub stop: std::sync::atomic::AtomicBool,
    /// The latest decoded picture; the consumer takes it (one encode per camera frame).
    pub frame: std::sync::Mutex<Option<RgbFrame>>,
    pub status: std::sync::Mutex<String>,
}

impl RtspShared {
    pub fn new() -> std::sync::Arc<Self> {
        std::sync::Arc::new(RtspShared {
            url: std::sync::Mutex::new(String::new()),
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

#[cfg(feature = "rtsp")]
pub mod rtsp {
    //! RTSP client (retina) -> H.264 access units -> openh264 -> `RtspShared::frame`.
    //! Runs only while `wanted`; reconnects on its own after an error or a URL change.
    use std::sync::Arc;
    use std::sync::atomic::Ordering;
    use std::time::{Duration, Instant};

    use futures::StreamExt;
    use retina::client::{Credentials, PlayOptions, SessionGroup, SessionOptions, SetupOptions, Transport};
    use retina::codec::{CodecItem, ParametersRef};

    use super::RtspShared;
    use crate::codec::VideoDecoder;
    use crate::logging::log;

    pub fn spawn(shared: Arc<RtspShared>) {
        std::thread::Builder::new()
            .name("rtsp".into())
            .spawn(move || run(shared))
            .expect("spawn rtsp thread");
    }

    fn set_status(shared: &RtspShared, s: String) {
        if *shared.status.lock().unwrap() != s {
            log(&format!("rtsp: {s}"));
            *shared.status.lock().unwrap() = s;
        }
    }

    fn run(shared: Arc<RtspShared>) {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("tokio runtime");
        loop {
            if shared.stop.load(Ordering::Relaxed) {
                return;
            }
            if !shared.wanted.load(Ordering::Relaxed) {
                set_status(&shared, "idle".into());
                std::thread::sleep(Duration::from_millis(150));
                continue;
            }
            let url = shared.url.lock().unwrap().clone();
            if url.trim().is_empty() {
                set_status(&shared, "no camera URL".into());
                std::thread::sleep(Duration::from_millis(500));
                continue;
            }
            set_status(&shared, format!("connecting {}", redact(&url)));
            match rt.block_on(stream_one(&shared, &url)) {
                Ok(()) => {}
                Err(e) => {
                    set_status(&shared, format!("error: {e}"));
                    std::thread::sleep(Duration::from_secs(2));
                }
            }
        }
    }

    /// The URL without its password, for the log and the screen.
    fn redact(url: &str) -> String {
        match url::Url::parse(url) {
            Ok(mut u) if u.password().is_some() => {
                let _ = u.set_password(Some("***"));
                u.to_string()
            }
            _ => url.to_string(),
        }
    }

    /// SPS and PPS out of an avcC record, as Annex B.
    fn avcc_to_annex_b(extra: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        if extra.len() < 7 || extra[0] != 1 {
            return out;
        }
        let mut i = 5;
        let n_sps = (extra[i] & 0x1F) as usize;
        i += 1;
        for _ in 0..n_sps {
            if i + 2 > extra.len() {
                return out;
            }
            let l = u16::from_be_bytes([extra[i], extra[i + 1]]) as usize;
            i += 2;
            if i + l > extra.len() {
                return out;
            }
            out.extend_from_slice(&[0, 0, 0, 1]);
            out.extend_from_slice(&extra[i..i + l]);
            i += l;
        }
        if i >= extra.len() {
            return out;
        }
        let n_pps = extra[i] as usize;
        i += 1;
        for _ in 0..n_pps {
            if i + 2 > extra.len() {
                return out;
            }
            let l = u16::from_be_bytes([extra[i], extra[i + 1]]) as usize;
            i += 2;
            if i + l > extra.len() {
                return out;
            }
            out.extend_from_slice(&[0, 0, 0, 1]);
            out.extend_from_slice(&extra[i..i + l]);
            i += l;
        }
        out
    }

    /// The NAL units of one access unit as retina hands it out: 4-byte length prefixes.
    /// Some when the prefixes walk the buffer exactly; a frame of 256..511 bytes starts
    /// `00 00 01 xx`, which LOOKS like an Annex B start code, so the shape is never
    /// sniffed from the first bytes (that was done at first, and every such frame went
    /// to the decoder raw and broke the GOP until the next IDR).
    fn length_prefixed(data: &[u8]) -> Option<Vec<&[u8]>> {
        let mut nals = Vec::new();
        let mut i = 0usize;
        while i + 4 <= data.len() {
            let l = u32::from_be_bytes([data[i], data[i + 1], data[i + 2], data[i + 3]]) as usize;
            i += 4;
            if l == 0 || i + l > data.len() {
                return None;
            }
            nals.push(&data[i..i + l]);
            i += l;
        }
        (i == data.len() && !nals.is_empty()).then_some(nals)
    }

    /// One access unit as the decoder wants it: Annex B start codes, and SPS/PPS in
    /// front of a key frame that carries none.
    fn to_annex_b(data: &[u8], key: bool, sps_pps: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(data.len() + sps_pps.len() + 16);
        match length_prefixed(data) {
            Some(nals) => {
                let has_sps = nals.iter().any(|n| n[0] & 0x1F == 7);
                if key && !has_sps {
                    out.extend_from_slice(sps_pps);
                }
                for nal in nals {
                    out.extend_from_slice(&[0, 0, 0, 1]);
                    out.extend_from_slice(nal);
                }
            }
            None => {
                // already Annex B (not what retina does today; kept for another client)
                let has_sps = data.windows(4).any(|w| w[0] == 0 && w[1] == 0 && w[2] == 1 && w[3] & 0x1F == 7);
                if key && !has_sps {
                    out.extend_from_slice(sps_pps);
                }
                out.extend_from_slice(data);
            }
        }
        out
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        /// A 309-byte NAL: its length prefix reads 00 00 01 35, the shape of a start code.
        #[test]
        fn short_nal_is_not_a_start_code() {
            let mut au = vec![0, 0, 1, 0x35];
            au.extend(std::iter::repeat(0x41u8).take(0x135));
            let out = to_annex_b(&au, false, &[]);
            assert_eq!(&out[..5], &[0, 0, 0, 1, 0x41]);
            assert_eq!(out.len(), 4 + 0x135);
        }
    }

    async fn stream_one(shared: &RtspShared, url_s: &str) -> Result<(), String> {
        let mut url = url::Url::parse(url_s.trim()).map_err(|e| format!("bad URL: {e}"))?;
        let creds = if url.username().is_empty() {
            None
        } else {
            let c = Credentials {
                username: url.username().to_string(),
                password: url.password().unwrap_or("").to_string(),
            };
            let _ = url.set_username("");
            let _ = url.set_password(None);
            Some(c)
        };
        let opts = SessionOptions::default()
            .creds(creds)
            .user_agent("nyxhop".to_owned())
            .session_group(Arc::new(SessionGroup::default()));
        let mut session = retina::client::Session::describe(url, opts).await.map_err(|e| e.to_string())?;
        let vi = session
            .streams()
            .iter()
            .position(|s| s.media() == "video" && s.encoding_name().eq_ignore_ascii_case("h264"))
            .ok_or_else(|| "no H.264 video track".to_string())?;
        let mut sps_pps = match session.streams()[vi].parameters() {
            Some(ParametersRef::Video(v)) => avcc_to_annex_b(v.extra_data()),
            _ => Vec::new(),
        };
        session
            .setup(vi, SetupOptions::default().transport(Transport::Tcp(Default::default())))
            .await
            .map_err(|e| e.to_string())?;
        let session = session
            .play(PlayOptions::default())
            .await
            .map_err(|e| e.to_string())?;
        let mut demuxed = session.demuxed().map_err(|e| e.to_string())?;
        let mut dec = VideoDecoder::new();
        let (mut n_frames, mut n_pics, mut bytes) = (0u64, 0u64, 0u64);
        let (mut n_none, mut n_err) = (0u64, 0u64);
        // NYX_RTSP_DEBUG=<n>: log the first n access units (NAL types, decode result)
        let mut debug: u32 = std::env::var("NYX_RTSP_DEBUG").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
        let mut t_report = Instant::now();
        let mut dims = (0usize, 0usize);
        let (mut waiting_key, mut key_logged) = (true, false);
        loop {
            if shared.stop.load(Ordering::Relaxed) || !shared.wanted.load(Ordering::Relaxed) {
                return Ok(());
            }
            if *shared.url.lock().unwrap() != url_s {
                return Ok(());
            }
            let item = match tokio::time::timeout(Duration::from_secs(10), demuxed.next()).await {
                Err(_) => return Err("no data for 10 s".into()),
                Ok(None) => return Err("stream ended".into()),
                Ok(Some(Err(e))) => return Err(e.to_string()),
                Ok(Some(Ok(item))) => item,
            };
            let CodecItem::VideoFrame(f) = item else { continue };
            if f.has_new_parameters() {
                if let Some(ParametersRef::Video(v)) = demuxed.streams()[f.stream_id()].parameters() {
                    sps_pps = avcc_to_annex_b(v.extra_data());
                }
            }
            let key = f.is_random_access_point();
            if waiting_key {
                if !key {
                    if !key_logged {
                        key_logged = true;
                        set_status(shared, "waiting for a key frame".into());
                    }
                    continue;
                }
                waiting_key = false;
            }
            n_frames += 1;
            bytes += f.data().len() as u64;
            let au = to_annex_b(f.data(), key, &sps_pps);
            let r = dec.decode_checked(&au);
            if debug > 0 {
                debug -= 1;
                let nals: Vec<u8> = au.windows(4).enumerate()
                    .filter(|(_, w)| w[0] == 0 && w[1] == 0 && w[2] == 1)
                    .map(|(_, w)| w[3] & 0x1F).collect();
                log(&format!(
                    "rtsp dbg: {} B key={} nals={:?} raw_head={:02x?} -> {}",
                    f.data().len(), u8::from(key), nals, &f.data()[..f.data().len().min(6)],
                    match &r { Ok(Some(p)) => format!("picture {}x{}", p.width, p.height), Ok(None) => "no picture".into(), Err(e) => format!("ERR {e}") }
                ));
            }
            match r {
                Ok(Some(rgb)) => {
                    n_pics += 1;
                    dims = (rgb.width, rgb.height);
                    *shared.frame.lock().unwrap() = Some(rgb);
                }
                Ok(None) => n_none += 1,
                Err(_) => n_err += 1,
            }
            let dt = t_report.elapsed();
            if dt >= Duration::from_secs(2) {
                set_status(shared, format!(
                    "streaming {}x{} {:.0} fps {:.0} kbps ({} frames: {} pictures, {} no-picture, {} rejected)",
                    dims.0, dims.1,
                    n_pics as f64 / dt.as_secs_f64(),
                    bytes as f64 * 8.0 / dt.as_secs_f64() / 1000.0,
                    n_frames, n_pics, n_none, n_err,
                ));
                n_frames = 0;
                n_pics = 0;
                n_none = 0;
                n_err = 0;
                bytes = 0;
                t_report = Instant::now();
            }
        }
    }
}

