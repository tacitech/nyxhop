//! nyx-mobile: NyxHop RX/TX on ANDROID, replacing the PC. The phone connects to a board (Wi-Fi or
//! USB-C -> Ethernet), receives the video itself and shows the HUD.
//!
//! Stage 1 (this version): the RX role: TCP to the receiving board's daemon (:7011), receive
//! DecFrames (already decoded in the fabric), reassemble blocks, decode video, show a DJI-style
//! HUD; send Feedback back so OLLA/AGC on the TX side keep adapting. Video: H264 (openh264,
//! VERIFIED to cross-compile with the NDK, no MediaCodec/JNI) AND MJPEG (image crate, pure Rust):
//! both are accepted, whichever codec the TX sets. A camera (for the TX role) is later work: nokhwa
//! is Windows-only, Android needs CameraX/ndk-camera through JNI.

use std::collections::{BTreeMap, VecDeque};
use std::io::Write;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nyx_common::codec::VideoDecoder;
use nyx_common::{RgbFrame, decode_jpeg, theme as th, to_color_image};
use nyx_link::{Reassembler, SourceType, parse_block};
use nyx_proto::{Msg, read_msg, write_msg};

pub const DEFAULT_BOARD: &str = "192.168.0.10:7011";

/// Android hands every app socket to the DEFAULT network, which is WiFi whenever
/// WiFi has internet and the wired link does not. The board hangs off USB-C
/// Ethernet, so on a phone with both up the app tried to reach 192.168.0.10 over
/// WiFi and failed, even though eth0 pinged the board in 0.8 ms. Routing cannot
/// fix it from our side: the per-uid rules key off the socket fwmark, not the
/// source address, and SO_BINDTODEVICE needs CAP_NET_RAW. The supported way is
/// ConnectivityManager.bindProcessToNetwork() on the Ethernet network, which is
/// what prefer_wired() does over JNI. Called before every connect so plugging the
/// cable in later also works.
#[cfg(target_os = "android")]
mod wired {
    use std::sync::atomic::{AtomicPtr, Ordering};

    static VM: AtomicPtr<std::ffi::c_void> = AtomicPtr::new(std::ptr::null_mut());
    static ACTIVITY: AtomicPtr<std::ffi::c_void> = AtomicPtr::new(std::ptr::null_mut());

    pub fn remember(vm: *mut std::ffi::c_void, activity: *mut std::ffi::c_void) {
        VM.store(vm, Ordering::Relaxed);
        ACTIVITY.store(activity, Ordering::Relaxed);
    }

    /// Bind this process to the Ethernet network. Ok(true) = bound.
    pub fn bind() -> Result<bool, jni::errors::Error> {
        use jni::objects::{JObject, JObjectArray, JValue};
        let vmp = VM.load(Ordering::Relaxed);
        let actp = ACTIVITY.load(Ordering::Relaxed);
        if vmp.is_null() || actp.is_null() {
            return Ok(false);
        }
        let vm = unsafe { jni::JavaVM::from_raw(vmp.cast()) }?;
        let mut env = vm.attach_current_thread()?;
        let activity = unsafe { JObject::from_raw(actp.cast()) };
        let name = env.new_string("connectivity")?;
        let cm = env
            .call_method(
                &activity,
                "getSystemService",
                "(Ljava/lang/String;)Ljava/lang/Object;",
                &[JValue::Object(&name)],
            )?
            .l()?;
        let nets = env
            .call_method(&cm, "getAllNetworks", "()[Landroid/net/Network;", &[])?
            .l()?;
        let nets = JObjectArray::from(nets);
        let n = env.get_array_length(&nets)?;
        for i in 0..n {
            let net = env.get_object_array_element(&nets, i)?;
            let caps = env
                .call_method(
                    &cm,
                    "getNetworkCapabilities",
                    "(Landroid/net/Network;)Landroid/net/NetworkCapabilities;",
                    &[JValue::Object(&net)],
                )?
                .l()?;
            if caps.is_null() {
                continue;
            }
            // NetworkCapabilities.TRANSPORT_ETHERNET = 3
            if env.call_method(&caps, "hasTransport", "(I)Z", &[JValue::Int(3)])?.z()? {
                return env
                    .call_method(
                        &cm,
                        "bindProcessToNetwork",
                        "(Landroid/net/Network;)Z",
                        &[JValue::Object(&net)],
                    )?
                    .z();
            }
        }
        Ok(false)
    }
}

/// Prefer the wired link when one is present (Android only; no-op elsewhere).
fn prefer_wired(sh: &Arc<Shared>) {
    #[cfg(target_os = "android")]
    {
        static SAID: std::sync::atomic::AtomicBool =
            std::sync::atomic::AtomicBool::new(false);
        match wired::bind() {
            Ok(true) => {
                if !SAID.swap(true, std::sync::atomic::Ordering::Relaxed) {
                    sh.say("network: bound to wired (Ethernet)");
                }
            }
            Ok(false) => {}
            Err(e) => sh.say(format!("network: wired bind failed: {e}")),
        }
    }
    #[cfg(not(target_os = "android"))]
    let _ = sh;
}

/// Board address override, read at startup from a plain text file so the IP can
/// be set without typing it on a phone screen:
///   adb shell "echo 192.168.1.101:7011 > ///     /sdcard/Android/data/com.nyxhop.mobile/files/board.txt"
/// That directory belongs to the app, so no storage permission is needed. Falls
/// back to DEFAULT_BOARD when the file is missing or empty.
pub fn board_addr() -> String {
    #[cfg(target_os = "android")]
    const PATHS: [&str; 1] = ["/sdcard/Android/data/com.nyxhop.mobile/files/board.txt"];
    #[cfg(not(target_os = "android"))]
    const PATHS: [&str; 1] = ["board.txt"];
    for p in PATHS {
        if let Ok(t) = std::fs::read_to_string(p) {
            let t = t.trim();
            if !t.is_empty() {
                return t.to_string();
            }
        }
    }
    DEFAULT_BOARD.to_string()
}

#[derive(Default, Clone)]
pub struct Metrics {
    pub snr_db: f32,
    pub bler: f32,
    pub rx_fps: f32,
    pub kbps: f32,
    pub frames: u64,
    pub lost: u64,
    pub mcs: Option<u8>,
}

pub struct Shared {
    pub frame: Mutex<Option<RgbFrame>>,
    pub frame_ver: AtomicU64,
    pub metrics: Mutex<Metrics>,
    pub connected: AtomicBool,
    pub stop: AtomicBool,
    pub addr: Mutex<String>,
    /// short on-screen log (connection/errors): Android has no visible stdout.
    pub log: Mutex<VecDeque<String>>,
    /// Deliver frames that are missing blocks (>=60%) instead of waiting: a torn
    /// stripe rather than a freeze. Same knob as `set partial` on the PC receiver.
    pub partial: AtomicBool,
    /// v40.33: the aircraft's licence lines (SourceType::Info) for the UI.
    pub far_lic: Mutex<String>,
}

impl Shared {
    pub fn new(addr: String) -> Arc<Self> {
        Arc::new(Shared {
            frame: Mutex::new(None),
            frame_ver: AtomicU64::new(0),
            far_lic: Mutex::new(String::new()),
            metrics: Mutex::new(Metrics::default()),
            connected: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            addr: Mutex::new(addr),
            log: Mutex::new(VecDeque::new()),
            partial: AtomicBool::new(true),
        })
    }
    pub fn say(&self, s: impl Into<String>) {
        let s = s.into();
        #[cfg(not(target_os = "android"))]
        eprintln!("[mobile] {s}");
        let mut l = self.log.lock().unwrap();
        l.push_back(s);
        while l.len() > 6 {
            l.pop_front();
        }
    }
}

/// Network thread: connect to the board, receive DecFrames -> reassemble -> decode -> publish.
/// Reconnects by itself (a phone changing Wi-Fi or waking up is routine).
pub fn spawn_net(sh: Arc<Shared>) {
    std::thread::Builder::new()
        .name("net".into())
        .spawn(move || {
            loop {
                if sh.stop.load(Ordering::Relaxed) {
                    return;
                }
                let addr = sh.addr.lock().unwrap().clone();
                sh.say(format!("connecting {addr}…"));
                prefer_wired(&sh);
                let Ok(mut s) = TcpStream::connect(&addr) else {
                    sh.connected.store(false, Ordering::Relaxed);
                    std::thread::sleep(Duration::from_secs(2));
                    continue;
                };
                let _ = s.set_nodelay(true);
                sh.connected.store(true, Ordering::Relaxed);
                sh.say(format!("connected {addr}"));
                session(&sh, &mut s);
                sh.connected.store(false, Ordering::Relaxed);
                sh.say("disconnected — retry 2s");
                std::thread::sleep(Duration::from_secs(2));
            }
        })
        .expect("spawn net");
}

fn session(sh: &Arc<Shared>, s: &mut TcpStream) {
    let mut reasm = Reassembler::new();
    // The base layer numbers its frames on its OWN counter, so it needs its own
    // reassembler: mixing both streams into one made every base block sweep the
    // main frames out as "stale" partials at the wrong reference id. Measured
    // before the split: 52 pictures decoded vs 2263 failed, 1-3 fps.
    let mut reasm_base = Reassembler::new();
    let mut h264 = VideoDecoder::new();
    // v36 simulcast: the base layer is a SEPARATE H.264 stream (own SPS/IDR), so
    // it needs its own decoder. Feeding both into one decoder corrupts state and
    // most pictures fail. Ignoring H264Base blocks entirely (as this app used to)
    // throws away every base-layer frame, which is most of them at low MCS.
    let mut h264_base = VideoDecoder::new();
    let mut fps_win: VecDeque<Instant> = VecDeque::new();
    let mut byte_win: VecDeque<(Instant, usize)> = VecDeque::new();
    let mut last_fb = Instant::now();
    let mut segs_ok = 0u64;
    let mut segs_lost = 0u64;
    // v32 minstrel: per-MCS OK block counts (wrapping u16) + base-layer count,
    // same bookkeeping as nyx-rx so the TX rate controller keeps working when
    // the phone is the receiver.
    let mut ok_mcs = [0u16; 6];
    let mut ok_base = 0u16;
    let mut snr = 0.0f32;
    let mut last_stat = Instant::now();
    let mut pics_ok = 0u64;
    let mut pics_fail = 0u64;
    // Ask the TX for a keyframe whenever the main layer fails to decode. Without
    // this the app never recovers: one failure leaves the H.264 decoder without a
    // reference and every later frame fails too (measured: 49 pictures decoded vs
    // 2033 failed, 1-2 fps, while the PC receiver on the same stream did 30 fps).
    let mut need_idr = false;
    let mut dbg_dec = 0u32;
    // H.264 must be fed in DECODE ORDER. The reassembler completes frames in
    // whatever order their last block happens to arrive (parity, retransmissions
    // and the base layer all interleave), so feeding it straight through made the
    // decoder fail on ~89% of otherwise COMPLETE frames: measured 32 complete
    // main frames/s delivered but only 3-6 pictures/s. This is the same reorder
    // buffer nyx-rx uses: hold frames by id, release consecutively, and skip a
    // hole once something older than HOLD is waiting behind it.
    let mut pend_pics: BTreeMap<u32, (SourceType, Vec<u8>, Instant)> = BTreeMap::new();
    let mut next_id: Option<u32> = None;
    const HOLD: Duration = Duration::from_millis(250);
    let mut n_push = 0u64;   // frames completed by the reassembler
    let mut n_stale = 0u64;  // partial frames flushed by take_stale
    let mut segcnt_sum = 0u64;
    let mut segcnt_n = 0u64;
    let mut last_main = Instant::now();
    let mut vdec_reset_at = Instant::now();
    loop {
        if sh.stop.load(Ordering::Relaxed) {
            return;
        }
        let Ok(msg) = read_msg(s) else { return };
        match msg {
            // the fabric already decoded; only reassembly + video decode remain (light,
            // phone-friendly)
            Msg::DecFrame { payload, mcs, llr_sum, .. } => {
                segs_ok += 1;
                // SNR heuristic straight from the frame, same anchor as nyx-rx:
                // mean|llr| = llr_sum / frame_bit_capacity, 8 ~ the decode
                // threshold of each MCS, +-6 dB per doubling. The board never
                // sends us a Feedback message (that is OUR message to the TX), so
                // without this the HUD had no SNR at all.
                if let Some(m) = nyx_proto::Mcs::from_index(usize::from(mcs)) {
                    let mean_abs = llr_sum as f32 / m.frame_bit_capacity() as f32;
                    let base = match m.bits_per_sym() { 2 => 8.0f32, 4 => 14.0, _ => 20.0 };
                    snr = base + 6.0 * (mean_abs.max(0.5) / 8.0).log2();
                }
                if let Some(seg) = parse_block(&payload) {
                    let sid = seg.frame_id;
                    let mi = usize::from(mcs);
                    if mi < 6 {
                        ok_mcs[mi] = ok_mcs[mi].wrapping_add(1);
                    }
                    let is_base = seg.src == SourceType::H264Base;
                    if is_base {
                        ok_base = ok_base.wrapping_add(1);
                    }
                    let r = if is_base { &mut reasm_base } else { &mut reasm };
                    segcnt_sum += u64::from(seg.seg_cnt);
                    segcnt_n += 1;
                    let mut done: Vec<(u32, SourceType, Vec<u8>)> =
                        r.push(seg).into_iter().collect();
                    n_push += done.len() as u64;
                    let before = done.len();
                    // v-slice: a frame more than 2 ids old and still incomplete -> deliver WHAT IS
                    // THERE (>= 60 % of its blocks): one torn band, but the video MOVES instead of
                    // freezing until an IDR (the encoder cuts independent slices).
                    if sh.partial.load(Ordering::Relaxed) {
                        done.extend(r.take_stale(sid, 2, 0.6));
                    }
                    n_stale += (done.len() - before) as u64;
                    let mut ready: Vec<(u32, SourceType, Vec<u8>)> = Vec::new();
                    for (id, src, data) in done {
                        if src == SourceType::H264Base {
                            // Base layer has its own id space and is only a
                            // fallback: decode it as it arrives.
                            ready.push((id, src, data));
                            continue;
                        }
                        // Ids wrap, so compare with wrapping distance. Anything not
                        // AHEAD of the next id we owe has already been shown: a
                        // retransmitted block can re-complete a frame we delivered
                        // seconds ago, and feeding that to H.264 a second time
                        // breaks the decoder (measured: id 216015/216017/216018 each
                        // decoded twice, second time always BAD).
                        let ahead = |a: u32, b: u32| a.wrapping_sub(b) < 0x8000_0000;
                        match next_id {
                            Some(n) if !ahead(id, n) => {} // late duplicate, drop
                            Some(_) => { pend_pics.insert(id, (src, data, Instant::now())); }
                            None => {
                                pend_pics.insert(id, (src, data, Instant::now()));
                                next_id = Some(id);
                            }
                        }
                    }
                    // Guard: never walk a huge id gap one step at a time. Ids are
                    // u32 and a resync (or a TX restart) can jump them by millions;
                    // stepping through that froze the network thread long enough for
                    // the board to drop the connection (measured: 8 blocks then
                    // "disconnected"). Anything further than a small window means we
                    // lost sync, so jump straight to the oldest frame we hold.
                    if let (Some(n), Some(&oldest)) = (next_id, pend_pics.keys().next()) {
                        let fwd = oldest.wrapping_sub(n);
                        if fwd > 64 && fwd < 0x8000_0000 {
                            need_idr = true;
                            next_id = Some(oldest);
                        }
                    }
                    while let Some(n) = next_id {
                        if let Some((src, data, _)) = pend_pics.remove(&n) {
                            ready.push((n, src, data));
                            next_id = Some(n.wrapping_add(1));
                        } else if !pend_pics.is_empty() && !r.has_partial(n) {
                            // A hole the reassembler never saw a block for is a
                            // frame the TX never sent (it bumps the id every tick,
                            // empty ticks included). Skip it at once: waiting HOLD
                            // and asking for an IDR on every such hole starves the
                            // link instead of healing it.
                            next_id = Some(n.wrapping_add(1));
                        } else if pend_pics.values().any(|(_, _, t)| t.elapsed() > HOLD) {
                            let skip_to = *pend_pics.keys().next().unwrap();
                            need_idr = true;
                            next_id = Some(skip_to);
                        } else {
                            break;
                        }
                    }
                    for (_id, src, data) in ready {
                        let n = data.len();
                        let pic = match src {
                            SourceType::Jpeg => decode_jpeg(&data),
                            SourceType::H264 => {
                                let out = h264.decode(&data);
                                if dbg_dec < 24 {
                                    dbg_dec += 1;
                                    let mut nals = Vec::new();
                                    let mut i = 0usize;
                                    while i + 3 < data.len() {
                                        if data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 {
                                            nals.push(data[i + 3] & 0x1F);
                                            i += 4;
                                        } else {
                                            i += 1;
                                        }
                                    }
                                    sh.say(format!("dec-in #{dbg_dec}: id={_id} len={} nals={nals:?} -> {}",
                                        data.len(), if out.is_some() { "OK" } else { "BAD" }));
                                }
                                if out.is_some() {
                                    need_idr = false;
                                    pics_ok += 1;
                                    last_main = Instant::now();
                                } else {
                                    need_idr = true; // waiting for a keyframe
                                    pics_fail += 1;
                                    // Frames keep arriving but nothing decodes for
                                    // 6 s: the decoder is wedged and even an IDR
                                    // will not clear it, so build a new one.
                                    if last_main.elapsed() > Duration::from_secs(6)
                                        && vdec_reset_at.elapsed() > Duration::from_secs(10)
                                    {
                                        h264 = VideoDecoder::new();
                                        vdec_reset_at = Instant::now();
                                        last_main = Instant::now();
                                        sh.say("vdec: no picture for 6s -> new decoder");
                                    }
                                }
                                out
                            }
                            // Always decode the base layer to keep its reference
                            // chain alive, but only SHOW it when the main layer is
                            // late, otherwise the picture flips between layers.
                            SourceType::H264Base => {
                                let out = h264_base.decode(&data);
                                if last_main.elapsed().as_millis() > 250 { out } else { None }
                            }
                            SourceType::Info => {
                                // v40.33: the aircraft's lic_* lines for the licence row
                                *sh.far_lic.lock().unwrap() = String::from_utf8_lossy(&data).into_owned();
                                None
                            }
                            _ => None,
                        };
                        if let Some(f) = pic {
                            *sh.frame.lock().unwrap() = Some(f);
                            sh.frame_ver.fetch_add(1, Ordering::Relaxed);
                            let now = Instant::now();
                            fps_win.push_back(now);
                            byte_win.push_back((now, n));
                        }
                    }
                } else {
                    segs_lost += 1;
                }
                let cut = Instant::now() - Duration::from_secs(2);
                while fps_win.front().is_some_and(|t| *t < cut) {
                    fps_win.pop_front();
                }
                while byte_win.front().is_some_and(|(t, _)| *t < cut) {
                    byte_win.pop_front();
                }
                if last_stat.elapsed() >= Duration::from_secs(2) {
                    last_stat = Instant::now();
                    let (f, k) = {
                        let m = sh.metrics.lock().unwrap();
                        (m.rx_fps, m.kbps)
                    };
                    sh.say(format!(
                        "rx {f:.0} fps {k:.0} kbps snr {snr:.1} dB | blocks {segs_ok} bad {segs_lost} | pics {pics_ok}/{pics_fail} | push {n_push} stale {n_stale} segcnt~{:.1}{}",
                        segcnt_sum as f32 / segcnt_n.max(1) as f32,
                        if need_idr { " | want IDR" } else { "" }
                    ));
                }
                let mut m = sh.metrics.lock().unwrap();
                m.rx_fps = fps_win.len() as f32 / 2.0;
                m.kbps = byte_win.iter().map(|(_, n)| *n).sum::<usize>() as f32
                    * 8.0
                    / 2.0
                    / 1000.0;
                m.frames = fps_win.len() as u64 + m.frames.max(0);
                m.mcs = Some(mcs);
                m.snr_db = snr;
            }
            // the daemon also pushes SNR telemetry through EqFrame/Feedback depending on mode
            Msg::Feedback { snr_db, bler, .. } => {
                snr = snr_db;
                let mut m = sh.metrics.lock().unwrap();
                m.snr_db = snr_db;
                m.bler = bler;
            }
            _ => {}
        }
        // Feedback every 150 ms (as nyx-rx on the PC): the TX needs it for OLLA and the bitrate
        // regulation loop (delivered/sent ratio) to track; too sparse and the TX guesses blind. 150
        // ms normally, 100 ms while a keyframe is wanted (same as nyx-rx).
        let fb_due = if need_idr { 100 } else { 150 };
        if last_fb.elapsed() >= Duration::from_millis(fb_due) {
            last_fb = Instant::now();
            let m = sh.metrics.lock().unwrap().clone();
            let fb = Msg::Feedback {
                snr_db: m.snr_db,
                bler: m.bler,
                segs_ok,
                segs_lost,
                need_idr,
                ok_mcs,
                ok_base,
            };
            if write_msg(s, &fb).is_err() {
                return;
            }
            let _ = s.flush();
        }
    }
}

// ------------------------------------------------------------------ UI --

pub struct App {
    sh: Arc<Shared>,
    tex: Option<egui::TextureHandle>,
    tex_ver: u64,
    /// v40.34: the settings drawer (gear).
    drawer_open: bool,
    /// v40.42: the plates over the video (drawer header switch).
    hud_plates: bool,
    addr_buf: String,
    board: nyx_common::boardctl::BoardCtl,
    /// v40.33: pasted licence text + last result
    lic_buf: String,
    lic_status: String,
    /// v40.34: the board tuning fields (shared form with the PC apps).
    form: nyx_common::ui::RadioForm,
    chan_form: nyx_common::ui::ChannelForm,
    show_log: bool,
}

impl App {
    pub fn new(sh: Arc<Shared>) -> Self {
        let a = sh.clone();
        let board = nyx_common::boardctl::BoardCtl::spawn(
            Arc::new(move || {
                let addr = a.addr.lock().unwrap().clone();
                let host = addr.split(':').next().unwrap_or("192.168.0.10");
                format!("{host}:7202")
            }),
            &["get", "trig", "softagc", "hop status", "license"],
        );
        let addr_buf = sh.addr.lock().unwrap().clone();
        App {
            sh,
            tex: None,
            tex_ver: u64::MAX,
            drawer_open: false,
            hud_plates: true,
            addr_buf,
            board,
            lic_buf: String::new(),
            lic_status: String::new(),
            form: nyx_common::ui::RadioForm::default(),
            chan_form: nyx_common::ui::ChannelForm::default(),
            show_log: false,
        }
    }

    /// Everything behind the gear (same sections as the ground station on the PC).
    fn drawer(&mut self, ui: &mut egui::Ui) {
        use nyx_common::ui as nu;
        let st = self.board.snapshot();
        let connected = self.sh.connected.load(Ordering::Relaxed);
        nu::link_section(ui, &st, nu::Role::Ground, &self.board);
        nu::channel_section(ui, &st, &mut self.chan_form, &self.board);
        {
            let far = self.sh.far_lic.lock().unwrap().clone();
            nu::licence_section(
                ui, &st, if far.is_empty() { None } else { Some(far.as_str()) },
                &mut self.lic_buf, &mut self.lic_status, &self.board,
            );
        }
        nu::radio_section(ui, &st, nu::Role::Ground, &mut self.form, &self.board, &mut |_hz| {});
        if nu::connection_section(ui, &mut self.addr_buf, "192.168.0.12:7011", connected, None) {
            *self.sh.addr.lock().unwrap() = self.addr_buf.trim().to_string();
            self.sh.say("reconnecting…");
        }
        th::section(ui, "Advanced", false, |ui| {
            // App-side options (the board knows nothing about these).
            let mut part = self.sh.partial.load(Ordering::Relaxed);
            if ui
                .checkbox(&mut part, "Show partial frames")
                .on_hover_text("Deliver frames that are missing blocks instead of waiting: a torn stripe rather than a freeze.")
                .changed()
            {
                self.sh.partial.store(part, Ordering::Relaxed);
            }
            ui.checkbox(&mut self.show_log, "Connection log");
            if self.show_log {
                egui::ScrollArea::vertical().id_salt("log").max_height(160.0).stick_to_bottom(true).show(ui, |ui| {
                    for l in self.sh.log.lock().unwrap().iter() {
                        ui.label(egui::RichText::new(l).monospace().size(11.0));
                    }
                });
            }
        });
    }
}

impl eframe::App for App {
    fn ui(&mut self, root: &mut egui::Ui, _f: &mut eframe::Frame) {
        let ctx = root.ctx().clone();
        let ver = self.sh.frame_ver.load(Ordering::Relaxed);
        if ver != self.tex_ver {
            self.tex_ver = ver;
            let f = self.sh.frame.lock().unwrap().clone();
            if let Some(f) = f {
                if f.width > 0 && f.height > 0 && f.rgb.len() == f.width * f.height * 3 {
                    let img = to_color_image(&f);
                    let same = self.tex.as_ref().map_or(false, |t| t.size() == [f.width, f.height]);
                    match &mut self.tex {
                        Some(t) if same => t.set(img, egui::TextureOptions::LINEAR),
                        _ => self.tex = Some(ctx.load_texture("v", img, egui::TextureOptions::LINEAR)),
                    }
                }
            }
        }
        use nyx_common::ui as nu;
        let st = self.board.snapshot();
        let m = self.sh.metrics.lock().unwrap().clone();
        let connected = self.sh.connected.load(Ordering::Relaxed);
        let data = nu::HudData {
            connected,
            live: connected && m.rx_fps >= 5.0,
            snr_db: m.snr_db,
            fps: m.rx_fps,
            kbps: m.kbps,
            mcs: m.mcs.map(|x| x.to_string()).unwrap_or_else(|| "-".into()),
            pills: vec![nu::link_pill(&st), nu::licence_pill(&st), nu::mode_pill(&st)],
            banner: None,
            title: "NYXHOP".into(),
            empty_text: "waiting for video…".into(),
            plates: self.hud_plates,
        };
        let tex = self.tex.clone();
        let mut open = self.drawer_open;
        let mut plates = self.hud_plates;
        nu::screen(root, &mut open, &mut plates, "Settings", |ui| self.drawer(ui), |ui| nu::hud(ui, tex.as_ref(), &data));
        self.drawer_open = open;
        self.hud_plates = plates;
        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

/// Android entry point (NativeActivity calls this directly).
#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
pub fn android_main(app: winit::platform::android::activity::AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );
    wired::remember(app.vm_as_ptr().cast(), app.activity_as_ptr().cast());
    let sh = Shared::new(board_addr());
    spawn_net(sh.clone());
    let opts = eframe::NativeOptions {
        android_app: Some(app),
        ..Default::default()
    };
    let _ = eframe::run_native(
        "NyxHop",
        opts,
        Box::new(move |cc| {
            nyx_common::ui::touch_style(&cc.egui_ctx);
            Ok(Box::new(App::new(sh)))
        }),
    );
}
