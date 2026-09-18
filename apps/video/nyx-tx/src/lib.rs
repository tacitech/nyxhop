//! nyx-tx: video/data transmitter. Modulates PHY frames and streams the
//! baseband IQ to the channel node over TCP; receives telemetry feedback
//! (for MCS adaptation) and NACKs (for ARQ) on the same connection.
//!
//! Usage: nyx-tx [--channel 127.0.0.1:7010]

mod conv_tx;
mod net;
mod pc;
mod worker;

/// v36 simulcast: packetize the BASE layer: like packetize_fec but labelled H264Base so the RX
/// splits the funnel and chooses the layer to show.
pub fn packetize_fec_base(frame_id: u32, data: &[u8]) -> Vec<[u8; nyx_link::BLOCK_BYTES]> {
    nyx_link::packetize_fec(frame_id, nyx_link::SourceType::H264Base, data, true)
}

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

#[cfg(feature = "gui")]
use nyx_common::logging;
use nyx_common::logging::log;
use nyx_common::source::{SourceKind, WebcamShared};
use nyx_common::RgbFrame;
#[cfg(feature = "gui")]
use nyx_common::to_color_image;
use nyx_common::Opts;
use nyx_proto::{DEFAULT_TX_PORT, Mcs};

/// v40.40: radio sample rate the app was told about (--samp / Radio drawer). Only the
/// time-related MCS numbers (bit rate) depend on it; the modem itself runs on the board.
static SAMP_HZ: AtomicU64 = AtomicU64::new(15_360_000);

pub fn samp_hz() -> f64 {
    SAMP_HZ.load(Ordering::Relaxed) as f64
}

pub fn set_samp_hz(hz: u64) {
    if hz > 0 {
        SAMP_HZ.store(hz, Ordering::Relaxed);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codec {
    H264,
    Mjpeg,
}

impl Codec {
    pub fn label(self) -> &'static str {
        match self {
            Codec::H264 => "H.264 (openh264)",
            Codec::Mjpeg => "MJPEG",
        }
    }
}

#[derive(Clone, PartialEq)]
pub struct TxConfig {
    pub source: SourceKind,
    /// v40.45: the IP camera, as `rtsp://user:pass@host/path` (SourceKind::Rtsp).
    pub rtsp_url: String,
    /// camera pass-through 17/9: send the camera's own H.264 as it is (no decode, no re-encode): the
    /// camera's bitrate and key-frame interval rule, the receiver's key-frame requests
    /// cannot be served, and a frame that does not fit the link is dropped whole up to the
    /// next key frame. `set pass 0|1`. The only H.264 there is in a build without openh264.
    pub rtsp_pass: bool,
    /// camera pass-through 17/9: in pass-through, set the camera's bitrate limit over ONVIF so the stream
    /// fits the link (`set camadapt 0|1`). The camera cannot be thinned on the way; without this
    /// a camera above what the air carries loses whole runs of frames.
    pub cam_adapt: bool,
    /// The camera's ONVIF device service; empty = port 80 of the RTSP host (`set onvif <url>|auto`).
    pub onvif_url: String,
    /// The highest bitrate the camera is taken to (kbit/s); 0 = what the camera had when first
    /// reached (`set cammax <kbps>`).
    pub cam_max_kbps: u32,
    pub codec: Codec,
    pub width: usize,
    pub height: usize,
    pub fps: f32,
    pub jpeg_quality: u8,
    pub mcs: Mcs,
    pub auto_mcs: bool,
    pub arq: bool,
    /// Incremental-redundancy style retransmissions: resend at rate-1/2
    /// QPSK so the receiver gains real coding gain on top of the combined
    /// systematic energy (vs. plain Chase repetition).
    pub harq_ir: bool,
    /// v40.31: restore the MCS and the per-rate statistics a channel last had when the
    /// daemon reports a hop onto it. OFF by default - measured 7/9 with a 1 s dwell:
    /// every channel froze at whatever MCS it had when first left (5755 at MCS0, 5775
    /// at MCS2), the restore ran every second so minstrel never got the two agreeing
    /// windows it needs to climb, IDR backoff took the bitrate to 131 kbps and the
    /// receive end's AGC read the starved stream as "weak" and drove itself into
    /// compression. With AFH dropping channels 6 dB under the median the channels
    /// that remain are alike, and one rate state serves them all. `set chanmem 1`
    /// re-enables it for experiments.
    pub chan_mem: bool,
    /// v3.1: send frames as BITS (Msg::TxBits) for the fabric modulator instead of IQ; needs a
    /// bitstream with the fabric modulator and the daemon's `txmod 1`.
    pub txbits: bool,
    /// v4.3: send the PAYLOAD to the fabric encoder (needs modulator bitstream ver >= 3).
    pub txpl: bool,
    /// v21: conv/Viterbi frames (RX2_ONLY bitstream, demod ver >= 0x1E). MCS5 ONLY at first: conv
    /// MCS5 carries 2022 B >= BLOCK_BYTES 2016; the lower rates carry 447..1797 B, less than a
    /// block. `set txconv 0|1`.
    pub txconv: bool,
    /// v40.18: FULL-FRAME INTERLEAVING (flagged in SIG, the RX follows per frame). ON by default
    /// (both board images carry the B bitstream); an old PL ignores the flag and decodes wrongly,
    /// then `set filv 0`.
    pub filv: bool,
    /// v5: minimum silence between blocks on air (ms): the RX's shadow capture + hold chain; sweep
    /// with `set gap <ms>`.
    pub air_gap_ms: f32,
    /// v6: PAIR 2 frames per capture (needs the 24-symbol RX grid bitstream + modulator v6.6): the
    /// even block carries the pair flag in SIG, the odd block fires right behind it without waiting
    /// the gap; the fabric RX demodulates both. `set pair 0|1`.
    pub pair: bool,
    /// v7: adaptive quality ladder: on a weak channel step down resolution AND fps (not just
    /// bitrate) so the few bits are spent well. width/height/fps in the config become a CEILING;
    /// the ladder only steps down. `set autores 0|1`.
    pub auto_res: bool,
    pub paused: bool,
    /// v11: symbols-per-frame ceiling for the fabric TxBits path. Modulator v7 (0x4E4D_0005,
    /// streaming ring) accepts up to 37 (= MCS0); the old bitstream only 16. Set 37 after loading
    /// the new bitstream: `set plsyms 37`.
    pub pl_syms: usize,
    /// PROACTIVE REPETITION: send every block N times (same seq) one air-gap apart -> time
    /// diversity against noise/fade bursts with NO NACK round trip (the RX dedups by seq, takes the
    /// first copy that decodes). 1 = off (reactive ARQ only). `set rep N` (1..8). The price: N x
    /// airtime per frame.
    pub rep_count: u32,
    /// v-fec: add one XOR PARITY block per frame (>= 2 blocks): lose exactly one block and the RX
    /// rebuilds it, NO IDR request (an IDR = a bloated frame = congestion = stutter). Overhead 1/N.
    /// `set fec 0|1`.
    pub fec: bool,
    /// v40.45c: under interference send a second copy of every parity block and every
    /// retransmission 12 ms later (`set dup 0|1`). OFF by default: A/B on 2437 MHz next to
    /// WiFi (alternating minutes, MCS5 held) showed video loss tracking the PL block error
    /// rate the same way with or without it (1.9 % vs 1.4 %, 26 % vs 8 % in a heavy minute),
    /// while it added 35-60 frames/s of air. ARQ already repairs what a second copy would.
    pub dup: bool,
    /// v36 simulcast base layer, ON BY DEFAULT. Measured on the SAME ruler, disp_fps (pictures
    /// actually shown, not the flattering rx_fps): ON 72 % of samples with a picture vs OFF 59 %;
    /// the base layer carries 22 % of the samples at the fade edge. (The "100 %" of v35.1 the day
    /// before was the old rx_fps ruler, which counts delivered frames that never decoded.) Off:
    /// `set simulcast 0`.
    pub simulcast: bool,
    /// v40: PINNED MCS for the base layer (the lifebuoy). 0 = QPSK 1/2 (default); 6 = the QPSK 1/2
    /// x2 REPEAT STEP (+3 dB reach: the rescue layer goes 3 dB further, at twice the airtime, ~150
    /// f/s at 15 fps). `set basemcs 0|6`.
    pub base_mcs: usize,
    /// v40.46 latency test: burn the millisecond counter into the picture (nyx_common::stamp).
    /// `set stamp 0|1`.
    pub stamp: bool,
}

impl Default for TxConfig {
    fn default() -> Self {
        TxConfig {
            // v40.44z: the camera when the build has one (the public build has no nyx-tx.cfg to say
            // so, and a customer following the README got the test pattern); the worker falls back
            // to the pattern when no camera opens.
            source: if cfg!(feature = "webcam") { SourceKind::Webcam } else { SourceKind::Pattern },
            rtsp_url: String::new(),
            rtsp_pass: !cfg!(feature = "h264"),
            cam_adapt: true,
            onvif_url: String::new(),
            cam_max_kbps: 0,
            codec: Codec::H264,
            // v40.44z: the bench setting; at 480x360 the same camera and link gave 20 fps and a
            // display that dipped to 10, at 640x480 a steady 30 (10/9, public build, no cfg).
            width: 640,
            height: 480,
            // The hardware loopback sustains ~29 fps end-to-end (measured);
            // 24 keeps headroom for ARQ retransmissions.
            fps: 30.0, // v40.44z: what the README promises; the camera mode picked is >= 24 fps
            jpeg_quality: 60,
            mcs: Mcs::Qpsk12,
            auto_mcs: true,
            arq: true,
            // Default is the FABRIC path (txpl: the PL LDPC-encodes + modulates, the LAN only
            // carries ~2 KB of payload per frame; txbits for paired frames). Turned off
            // (GUI/console) only to measure the PC IQ path.
            txbits: true,
            // The receive path in both board images is the convolutional one, so this is
            // what carries video. It used to be off by default, which meant a fresh
            // install with no nyx-tx.cfg beside it sent frames the far end could not
            // decode: the link looked dead with every counter on the transmit side
            // still climbing.
            txconv: true,
            filv: true,
            txpl: true,
            // v20.9: 20 ms: LDPC maxit 8 stretched the SB chain, and a gap < 20 collided with the
            // demapping of the next capture (p2 fell 70 %, fallback took over; knee measured on air
            // at 16/17/18/20). Lower it again once the RTL gates demap on ldpc-idle. v40.34: 3 ms =
            // the daemon's txpace on the rx2 path (330 frames/s ceiling). 20 ms was the old LDPC
            // fabric window and caps the link at 50 frames/s: the conv queue then drops a third of
            // the blocks, the rate controller sees delivered/sent 0.6 and pins the bitrate at 60
            // kbps (two hours lost).
            air_gap_ms: 1.8,
            pair: true, // v20.6: pair by default: 1.48 M at bler 0 on air
            auto_res: true,
            harq_ir: true,
            chan_mem: false,
            paused: false,
            pl_syms: 16,
            rep_count: 1, // default off (reactive ARQ only)
            fec: true,
            dup: false,
            // v40.44z: off by default (user, 10/9): the base layer costs ~30 ms of glass-to-glass
            // latency (61 vs 28 ms measured); turn it on for reach through fades.
            simulcast: false,
            base_mcs: 0, // default MCS0; `set basemcs 6` for the +3 dB lifebuoy
            stamp: false,
        }
    }
}

#[derive(Clone, Default)]
pub struct FeedbackState {
    pub snr_db: f32,
    pub bler: f32,
    pub segs_ok: u64,
    pub segs_lost: u64,
    pub need_idr: bool,
    /// v32 minstrel: block giao ok RIENG TUNG MCS (wrap u16 tu nyx-rx).
    pub ok_mcs: [u16; 6],
    pub ok_base: u16,
    pub updated: Option<std::time::Instant>,
}

#[derive(Clone, Default)]
pub struct TxStats {
    pub active_mcs: Option<Mcs>,
    pub seq: u64,
    pub blocks_per_frame: usize,
    pub app_bytes_per_frame: usize,
    pub tx_fps: f32,
    pub iq_mbps: f32,
    pub nacks_handled: u64,
    pub retransmits: u64,
    pub video_bitrate_bps: u32,
    pub idr_sent: u64,
    /// v22: expose which constraint is cutting the bitrate ceiling (no guessing).
    pub rc_air_scale: f32,
    pub rc_one_burst: f32,
    pub rc_ceiling: f32,
    pub rc_budget: f32,
    pub rc_blocks_ps: f32,
    pub rc_msgs_ps: f32,
    pub rc_enc_us: f32,
    pub rc_gap_us: f32,
    pub rc_send_us: f32,
    pub rc_loop_us: f32,
    pub rc_blocks_frame: f32,
    pub rc_blocks_sent: f32,
    pub rc_src_bytes: f32,
    /// camera pass-through: the camera stream as it is sent, and what did not go
    pub pass_fps: f32,
    pub pass_kbps: f32,
    /// access units dropped because they waited too long (the camera outran the link)
    pub pass_drop_late: u64,
    /// access units dropped because an earlier one was lost (they cannot be decoded until the
    /// next key frame)
    pub pass_drop_nokey: u64,
    /// key-frame requests from the receiver that the camera stream could not answer
    pub pass_idr_req: u64,
    /// what the air carries at the current rung (bits of video per second)
    pub link_cap_bps: f32,
}

pub struct Shared {
    pub config: Mutex<TxConfig>,
    pub stats: Mutex<TxStats>,
    pub feedback: Mutex<FeedbackState>,
    pub preview: Mutex<Option<RgbFrame>>,
    pub preview_version: AtomicU64,
    pub connected: AtomicBool,
    pub stop: AtomicBool,
    pub webcam: Arc<WebcamShared>,
    pub rtsp: Arc<nyx_common::source::RtspShared>,
    /// Radio/channel address, editable at runtime from the GUI.
    pub channel_addr: Mutex<String>,
    /// Live socket, exposed so the GUI can drop it to force a reconnect.
    pub ctl_stream: Mutex<Option<std::net::TcpStream>>,
    /// v40.22: telemetry (MAVLink) from the air -> UDP out (--tlm-out, default 127.0.0.1:14556);
    /// tlm_rx = datagrams pushed out.
    pub tlm_out: Mutex<Option<(std::net::UdpSocket, std::net::SocketAddr)>>,
    pub tlm_rx: AtomicU64,
    /// v40.31: the video channel the daemon says it is transmitting on (Hz, 0 =
    /// unknown) and how many times it changed - minstrel keeps memory per channel.
    pub chan_hz: AtomicU64,
    pub chan_changes: AtomicU64,
    /// IqFrame transport: false = TCP (back-pressure), true = UDP (chunks, connectionless, survives
    /// a daemon restart). Switchable from the GUI.
    pub udp_mode: AtomicBool,
    /// v12: two-way message log ("→ ..." sent, "← ..." received) for the GUI.
    pub msgs: Mutex<Vec<String>>,
    /// v12: text waiting to go A -> B; the worker wraps it as a SourceType::Text frame between
    /// video frames (pushed by GUI/console).
    pub text_out: Mutex<Vec<String>>,
    /// v40.22: user datagrams (UDP --tlm-in) to send A -> B inside the video stream
    /// (SourceType::Data, 1 datagram = 1 frame). At most 64 waiting.
    pub tlm_tx_q: Mutex<Vec<Vec<u8>>>,
    /// v40.33: the daemon's licence state (lic_* lines) - sent in-band every few
    /// seconds so the app on the receive end sees this board's DNA and state.
    pub lic_info: Mutex<String>,
    /// The config file read at start; `save` writes the running settings back to it.
    pub cfg_path: Mutex<String>,
    /// Shell command run after `save` (`{path}` = the file), for a board that runs from RAM
    /// and keeps its files on the SD card (the E200). Empty = none.
    pub save_hook: Mutex<String>,
    /// camera pass-through 17/9, set by the host process: `standby` = send nothing and pull no camera (the
    /// board is the receiving end); `port_off` = leave the daemon's transmit port to someone
    /// else (the camera is silent, a PC app may send instead). See `set_port_off`.
    pub standby: AtomicBool,
    pub port_off: AtomicBool,
    /// One line on what the host process is doing, shown as `mode=` in `stats`.
    pub mode_note: Mutex<String>,
    /// Slowest spacing the conv transmit path may use between air frames (µs, 0 = only the
    /// config's gap). camera pass-through 17/9: the board app raises it while the daemon's raw queue fills, so
    /// a camera stream that outruns the air backs up here, where whole frames are dropped,
    /// instead of in the daemon, which drops single blocks and breaks many frames.
    pub pace_floor_us: AtomicU64,
}

/// Let go of the daemon's transmit port (or take it again). The daemon serves one sender at a
/// time, so a sender with nothing to send must close its connection, not just go quiet.
pub fn set_port_off(shared: &Shared, off: bool) {
    shared.port_off.store(off, Ordering::Relaxed);
    if off {
        if let Some(s) = shared.ctl_stream.lock().unwrap().take() {
            let _ = s.shutdown(std::net::Shutdown::Both);
        }
    }
}

/// No window: the threads `setup` started do all the work, this only logs a line every 2 s
/// until `shutdown`. Control through the `--ctl` console.
pub fn run_headless(shared: &Shared) {
    log("headless: no window opened; control through --ctl");
    while !shared.stop.load(Ordering::Relaxed) {
        std::thread::sleep(std::time::Duration::from_secs(2));
        let s = shared.stats.lock().unwrap().clone();
        let fb = shared.feedback.lock().unwrap().clone();
        let c = shared.config.lock().unwrap().source;
        log(&format!(
            "hl src={:?} mcs={:?} tx_fps={:.1} vbr={}kbps fb_snr={:.1} fb_bler={:.3} \
             blk_ps={:.1} cq={}/{}/{} conn={} pass_drop={}/{} idr_req={}",
            c,
            s.active_mcs.map(|m| m.index()),
            s.tx_fps,
            s.video_bitrate_bps / 1000,
            fb.snr_db,
            fb.bler,
            s.rc_blocks_ps,
            crate::conv_tx::CQ_IN.load(Ordering::Relaxed),
            crate::conv_tx::CQ_OUT.load(Ordering::Relaxed),
            crate::conv_tx::CQ_DROP.load(Ordering::Relaxed),
            shared.connected.load(Ordering::Relaxed),
            s.pass_drop_late,
            s.pass_drop_nokey,
            s.pass_idr_req,
        ));
    }
}

/// The standalone app: options from the command line, a window of its own (or
/// `--headless`).
#[cfg(feature = "gui")]
pub fn run(opts: Opts) -> eframe::Result {
    logging::init("tx");
    let shared = setup(&opts);
    if opts.flag("--headless") {
        run_headless(&shared);
        return Ok(());
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([520.0, 760.0])
            .with_title("NyxHop TX"),
        ..Default::default()
    };
    let s = shared.clone();
    let result = eframe::run_native(
        "NyxHop TX",
        options,
        Box::new(move |cc| {
            nyx_common::ui::touch_style(&cc.egui_ctx);
            Ok(Box::new(TxApp::new(s)))
        }),
    );
    shutdown(&shared);
    result
}

/// Everything but the window: threads, sockets, the source. The combined app calls
/// this and then hosts the `App` in its own window.
pub fn setup(opts: &Opts) -> Arc<Shared> {
    // v19cx: the 15.6 ms Windows sleep quantum turned a 12 ms pacing gap into ~22 ms (measured with
    // hwdt on air). timeBeginPeriod(1) pulls the timer resolution to 1 ms for the WHOLE process, so
    // the air-gap pacing matches the design.
    #[cfg(windows)]
    {
        #[link(name = "winmm")]
        unsafe extern "system" { fn timeBeginPeriod(u: u32) -> u32; }
        unsafe { timeBeginPeriod(1); }
    }

    // --samp <Hz>: the hardware sample rate (30720000 for the 20 MHz mode). The frame structure is
    // in samples, so only the time/Hz conversions change.

    if let Ok(hz) = opts.arg("--samp", "15360000").parse::<u64>() {
        crate::set_samp_hz(hz);
        if hz != 15_360_000 {
            log(&format!("samp rate override: {hz} Hz"));
        }
    }
    let channel_addr = opts.arg("--channel", &format!("127.0.0.1:{DEFAULT_TX_PORT}"));
    log(&format!("connecting to channel node at {channel_addr}"));

    // v40.22: telemetry out by UDP (a MAVLink/GCS app plugs in here). --tlm-out 0 = off.
    let tlm_out = {
        let a = opts.arg("--tlm-out", "127.0.0.1:14556");
        match (a.parse::<std::net::SocketAddr>(), std::net::UdpSocket::bind("0.0.0.0:0")) {
            (Ok(addr), Ok(sock)) => {
                nyx_common::logging::log(&format!("telemetry out: UDP -> {addr}"));
                Some((sock, addr))
            }
            _ => None,
        }
    };
    let shared = Arc::new(Shared {
        config: Mutex::new(TxConfig::default()),
        stats: Mutex::new(TxStats::default()),
        feedback: Mutex::new(FeedbackState::default()),
        preview: Mutex::new(None),
        preview_version: AtomicU64::new(0),
        connected: AtomicBool::new(false),
        stop: AtomicBool::new(false),
        webcam: WebcamShared::new(),
        rtsp: nyx_common::source::RtspShared::new(),
        channel_addr: Mutex::new(channel_addr),
        ctl_stream: Mutex::new(None),
        tlm_out: Mutex::new(tlm_out),
        tlm_rx: AtomicU64::new(0),
        chan_hz: AtomicU64::new(0),
        chan_changes: AtomicU64::new(0),
        udp_mode: AtomicBool::new(opts.flag("--udp")),
        msgs: Mutex::new(Vec::new()),
        text_out: Mutex::new(Vec::new()),
        tlm_tx_q: Mutex::new(Vec::new()),
        lic_info: Mutex::new(String::new()),
        cfg_path: Mutex::new(String::new()),
        save_hook: Mutex::new(String::new()),
        // `--standby`: start with nothing sent until the host process says otherwise
        standby: AtomicBool::new(opts.flag("--standby")),
        port_off: AtomicBool::new(opts.flag("--standby")),
        mode_note: Mutex::new(String::new()),
        pace_floor_us: AtomicU64::new(0),
    });

    // v37.3: load the config file BEFORE spawning the worker, so the worker uses the right config
    // from the first frame (no window of a few seconds on defaults). v37.5: look for the cfg in cwd
    // -> beside the exe -> exe/../.. (the repo root when the exe is in target/release): a user
    // double-clicking the exe has a cwd that is NOT the repo root, and the app used to fall back to
    // Pattern/txconv-off -> "no video" (txpl went into the removed LDPC encoder; the transmitter
    // was mute while apparently transmitting).
    let cfg_path = {
        let arg = opts.arg("--config", "");
        if arg.is_empty() {
            let mut cands: Vec<std::path::PathBuf> = vec!["nyx-tx.cfg".into()];
            if let Ok(exe) = std::env::current_exe() {
                if let Some(d) = exe.parent() {
                    cands.push(d.join("nyx-tx.cfg"));
                    cands.push(d.join("..").join("..").join("nyx-tx.cfg"));
                }
            }
            cands
                .into_iter()
                .find(|p| p.exists())
                .map_or("nyx-tx.cfg".into(), |p| p.display().to_string())
        } else {
            arg
        }
    };
    load_config_file(&shared, &cfg_path);
    *shared.cfg_path.lock().unwrap() = cfg_path.clone();

    #[cfg(feature = "webcam")]
    nyx_common::source::webcam::spawn(shared.webcam.clone());
    #[cfg(feature = "rtsp-client")]
    nyx_common::source::rtsp::spawn(shared.rtsp.clone());
    #[cfg(feature = "onvif")]
    nyx_common::onvif::spawn(shared.rtsp.clone());

    let net = net::spawn(shared.clone());
    worker::spawn(shared.clone(), net);
    spawn_control(shared.clone(), opts.arg("--ctl", "127.0.0.1:7201"));
    // v40.22: user data TX -> RX: UDP in here -> Data frames in the video stream (ARQ like video)
    // -> nyx-rx --tlm-out. --tlm-in 0 = off.
    {
        let addr = opts.arg("--tlm-in", "0.0.0.0:14557");
        if addr != "0" {
            match std::net::UdpSocket::bind(&addr) {
                Ok(sock) => {
                    nyx_common::logging::log(&format!(
                        "telemetry in: UDP {addr} -> Data frames in the video stream"));
                    let sh = shared.clone();
                    std::thread::Builder::new()
                        .name("tlm-in".into())
                        .spawn(move || {
                            let mut buf = [0u8; 4096];
                            let mut n_ok = 0u64;
                            let mut n_drop = 0u64;
                            loop {
                                let Ok((n, _)) = sock.recv_from(&mut buf) else { continue };
                                if n == 0 || n > 1400 {
                                    continue;
                                }
                                let mut q = sh.tlm_tx_q.lock().unwrap();
                                if q.len() >= 64 {
                                    q.remove(0);
                                    n_drop += 1;
                                    if n_drop % 100 == 1 {
                                        nyx_common::logging::log(&format!(
                                            "telemetry: queue full, dropping the oldest packet ({n_drop})"));
                                    }
                                }
                                q.push(buf[..n].to_vec());
                                n_ok += 1;
                                if n_ok == 1 {
                                    nyx_common::logging::log(
                                        "telemetry: first UDP packet -> video stream");
                                }
                            }
                        })
                        .expect("spawn tlm-in");
                }
                Err(e) => nyx_common::logging::log(&format!(
                    "telemetry: bind {addr} failed: {e}")),
            }
        }
    }

    // v40.8: `--headless` = run WITHOUT a window. The whole real path (source -> encoder -> FEC ->
    // conv_tx -> board) lives in the threads spawned above; the GUI is only a viewport. Meant for a
    // screenless machine next to the board (an SBC on the bench LAN, driven remotely) to measure
    // real end-to-end video rather than inferring it from tx2test counters. Control and observation
    // through the --ctl port, exactly as with the GUI.
    shared
}

/// Stop the threads `setup` started (the window is gone or the mode changes).
pub fn shutdown(shared: &Shared) {
    shared.stop.store(true, Ordering::Relaxed);
    shared.webcam.stop.store(true, Ordering::Relaxed);
    shared.rtsp.stop.store(true, Ordering::Relaxed);
}

/// v37.3: ONE parser for both the console `set` and the config file; the two share it, so keys and
/// values can never diverge.
fn apply_set(c: &mut TxConfig, key: &str, val: &str) -> Result<(), String> {
    use nyx_common::control::parse_bool;
    let ok = match key {
        "fps" => val.parse().map(|v: f32| c.fps = v.clamp(1.0, 60.0)).is_ok(),
        "quality" => val.parse().map(|v: u8| c.jpeg_quality = v.clamp(20, 95)).is_ok(),
        "auto" => parse_bool(val).map(|v| c.auto_mcs = v).is_some(),
        "arq" => parse_bool(val).map(|v| c.arq = v).is_some(),
        "plsyms" => val.parse::<usize>().map(|v| c.pl_syms = v.clamp(1, 37)).is_ok(),
        "ir" => parse_bool(val).map(|v| c.harq_ir = v).is_some(),
        "txbits" => parse_bool(val).map(|v| c.txbits = v).is_some(),
        "txconv" => parse_bool(val).map(|v| c.txconv = v).is_some(),
        "filv" => parse_bool(val).map(|v| c.filv = v).is_some(),
        "txpl" => parse_bool(val).map(|v| c.txpl = v).is_some(),
        "pair" => parse_bool(val).map(|v| c.pair = v).is_some(),
        "autores" => parse_bool(val).map(|v| c.auto_res = v).is_some(),
        "gap" => val
            .parse::<f32>()
            .map(|v| c.air_gap_ms = v.clamp(0.4, 20.0))
            .is_ok(),
        "rep" => val
            .parse::<u32>()
            .map(|v| c.rep_count = v.clamp(1, 8))
            .is_ok(),
        "fec" => parse_bool(val).map(|v| c.fec = v).is_some(),
        "dup" => parse_bool(val).map(|v| c.dup = v).is_some(),
        "simulcast" => parse_bool(val).map(|v| c.simulcast = v).is_some(),
        "stamp" => parse_bool(val).map(|v| c.stamp = v).is_some(),
        "basemcs" => val
            .parse::<usize>()
            .ok()
            .filter(|&m| m == 0 || m == 6)
            .map(|m| c.base_mcs = m)
            .is_some(),
        // "set res 480x360": previously only the GUI could change it; the auto-res ladder
        // overwrites width/height, so scripts/demos need a way to set it back (the ladder steps
        // down under a heavy source and does NOT climb back: link_cap is measured from the rate
        // being sent -> it never sees 45 % headroom).
        "res" => val
            .split_once(['x', 'X'])
            .and_then(|(w, h)| {
                Some((w.trim().parse::<usize>().ok()?,
                      h.trim().parse::<usize>().ok()?))
            })
            .map(|(w, h)| {
                c.width = w.clamp(160, 2560);
                c.height = h.clamp(120, 1440);
            })
            .is_some(),
        "pause" => parse_bool(val).map(|v| c.paused = v).is_some(),
        "chanmem" => parse_bool(val).map(|v| c.chan_mem = v).is_some(),
        "mcs" => val
            .parse::<usize>()
            .ok()
            .and_then(Mcs::from_index)
            .map(|m| c.mcs = m)
            .is_some(),
        "rtsp" => { c.rtsp_url = val.trim().to_string(); true }
        "pass" => parse_bool(val).map(|v| c.rtsp_pass = v).is_some(),
        "camadapt" => parse_bool(val).map(|v| c.cam_adapt = v).is_some(),
        "onvif" => {
            let v = val.trim();
            c.onvif_url = if v.eq_ignore_ascii_case("auto") { String::new() } else { v.to_string() };
            true
        }
        "cammax" => val.parse::<u32>().map(|v| c.cam_max_kbps = v.min(50_000)).is_ok(),
        "source" => match val.to_ascii_lowercase().as_str() {
            "pattern" => { c.source = SourceKind::Pattern; true }
            "webcam" => { c.source = SourceKind::Webcam; true }
            "rtsp" | "ipcam" => { c.source = SourceKind::Rtsp; true }
            "data" => { c.source = SourceKind::RandomData; true }
            _ => false,
        },
        "codec" => match val.to_ascii_lowercase().as_str() {
            "h264" => { c.codec = Codec::H264; true }
            "mjpeg" | "jpeg" => { c.codec = Codec::Mjpeg; true }
            _ => false,
        },
        _ => return Err(format!("err unknown key {key}")),
    };
    if ok { Ok(()) } else { Err(format!("err bad value for {key}")) }
}

/// The running settings as config-file lines (`key value`), every key `apply_set` reads back.
pub fn config_lines(c: &TxConfig) -> String {
    let b = |v: bool| if v { "1" } else { "0" };
    let source = match c.source {
        SourceKind::Pattern => "pattern",
        SourceKind::Webcam => "webcam",
        SourceKind::Rtsp => "rtsp",
        SourceKind::RandomData => "data",
    };
    let mut out = String::new();
    out.push_str(&format!("source {source}\n"));
    if !c.rtsp_url.is_empty() {
        out.push_str(&format!("rtsp {}\n", c.rtsp_url));
    }
    out.push_str(&format!("pass {}\n", b(c.rtsp_pass)));
    out.push_str(&format!("camadapt {}\nonvif {}\ncammax {}\n",
        b(c.cam_adapt), if c.onvif_url.is_empty() { "auto" } else { &c.onvif_url }, c.cam_max_kbps));
    out.push_str(&format!("codec {}\n", if c.codec == Codec::H264 { "h264" } else { "mjpeg" }));
    out.push_str(&format!("res {}x{}\nfps {}\nquality {}\n", c.width, c.height, c.fps, c.jpeg_quality));
    out.push_str(&format!("auto {}\nmcs {}\narq {}\nir {}\nfec {}\ndup {}\n",
        b(c.auto_mcs), c.mcs.index(), b(c.arq), b(c.harq_ir), b(c.fec), b(c.dup)));
    out.push_str(&format!("txconv {}\nfilv {}\ntxbits {}\ntxpl {}\npair {}\ngap {}\nrep {}\n",
        b(c.txconv), b(c.filv), b(c.txbits), b(c.txpl), b(c.pair), c.air_gap_ms, c.rep_count));
    out.push_str(&format!("autores {}\nplsyms {}\nsimulcast {}\nbasemcs {}\nchanmem {}\nstamp {}\n",
        b(c.auto_res), c.pl_syms, b(c.simulcast), c.base_mcs, b(c.chan_mem), b(c.stamp)));
    out
}

/// The highest bitrate the camera may be taken to (kbit/s): the configured one, else what the
/// camera had when ONVIF first reached it; 0 = not known yet.
pub fn cam_ceiling(shared: &Shared) -> u32 {
    let c = shared.config.lock().unwrap().cam_max_kbps;
    if c > 0 { c } else { shared.rtsp.onvif_base_kbps.load(Ordering::Relaxed) }
}

/// Write the running settings to the config file (and run the save hook): the console's `save`,
/// callable by a host process too. Returns the console answer.
pub fn save_config(shared: &Shared) -> String {
    let path = shared.cfg_path.lock().unwrap().clone();
    let text = format!(
        "# written by `save` on the console; one `key value` per line, read at start\n{}",
        config_lines(&shared.config.lock().unwrap())
    );
    match std::fs::write(&path, text) {
        Ok(()) => {
            log(&format!("control: settings saved to {path}"));
            let hook = shared.save_hook.lock().unwrap().replace("{path}", &path);
            if hook.is_empty() {
                format!("saved={path}\nok")
            } else {
                let ok = std::process::Command::new("sh").arg("-c").arg(&hook).status().is_ok_and(|s| s.success());
                log(&format!("control: save hook {}", if ok { "ok" } else { "FAILED" }));
                format!("saved={path}\npersist={}\nok", if ok { "ok" } else { "failed" })
            }
        }
        Err(e) => format!("err cannot write {path}: {e}"),
    }
}

/// v37.3: load the config file at start-up: an app restart used to LOSE every setting (restart ->
/// Pattern/txconv-off/gap 20 ms, the link "dead" while the radio was fine; a whole session lost
/// each time). Each line is `key value` as for the console `set` (the `set key value` form is
/// accepted too); `#` is a comment; no file means defaults, not an error.
fn load_config_file(shared: &Shared, path: &str) {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(_) => {
            log(&format!("config: {path} not found - using defaults"));
            return;
        }
    };
    let mut c = shared.config.lock().unwrap();
    let (mut n_ok, mut n_err) = (0u32, 0u32);
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut it = line.split_whitespace();
        let mut key = it.next().unwrap_or("");
        if key == "set" {
            key = it.next().unwrap_or("");
        }
        let Some(val) = it.next() else {
            log(&format!("config: skipping line '{line}' (missing value)"));
            n_err += 1;
            continue;
        };
        match apply_set(&mut c, key, val) {
            Ok(()) => n_ok += 1,
            Err(e) => {
                log(&format!("config: dong '{line}' -> {e}"));
                n_err += 1;
            }
        }
    }
    log(&format!("config: {path} applied {n_ok} keys, {n_err} errors"));
}

fn spawn_control(shared: Arc<Shared>, addr: String) {
    use nyx_common::control::{self, parse_bool};
    let h: control::Handler = Arc::new(move |cmd: &str| {
        let mut it = cmd.split_whitespace();
        match it.next() {
            Some("say") => {
                let t: String = cmd.splitn(2, ' ').nth(1).unwrap_or("").trim().to_string();
                if t.is_empty() {
                    "err say <text>".into()
                } else {
                    shared.text_out.lock().unwrap().push(t.clone());
                    let mut m = shared.msgs.lock().unwrap();
                    m.push(format!("→ {t}"));
                    if m.len() > 200 { m.remove(0); }
                    "ok".into()
                }
            }
            Some("msgs") => {
                let m = shared.msgs.lock().unwrap();
                let mut out: String = m.iter().rev().take(20).rev()
                    .cloned().collect::<Vec<_>>().join("
");
                out.push_str("
ok");
                out
            }
            Some("get") => {
                let c = shared.config.lock().unwrap().clone();
                format!(
                    "source={:?}\nrtsp={}\npass={}\ncamadapt={}\nonvif={}\ncammax={}\ncodec={:?}\nres={}x{}\nfps={}\nquality={}\nmcs={}\nauto={}\narq={}\nir={}\nrep={}\nsimulcast={}\nbasemcs={}\npause={}\nchanmem={}\nstamp={}\nok",
                    c.source, c.rtsp_url, u8::from(c.rtsp_pass), u8::from(c.cam_adapt),
                    if c.onvif_url.is_empty() { "auto" } else { &c.onvif_url }, c.cam_max_kbps, c.codec, c.width, c.height, c.fps, c.jpeg_quality,
                    c.mcs.index(), c.auto_mcs, c.arq, c.harq_ir, c.rep_count,
                    c.simulcast, c.base_mcs, c.paused, c.chan_mem, c.stamp
                )
            }
            Some("stats") => {
                let s = shared.stats.lock().unwrap().clone();
                let fb = shared.feedback.lock().unwrap().clone();
                let out = format!(
                    "active_mcs={:?}\nseq={}\ntx_fps={:.1}\niq_mbps={:.1}\nnacks={}\nretx={}\nfb_snr={:.1}\nfb_bler={:.3}\nconnected={}\nvbr_kbps={}\nrc_air_scale={:.2}\nrc_one_burst_kbps={:.0}\nrc_ceiling_kbps={:.0}\nrc_budget_kbps={:.0}\nrc_blocks_ps={:.1}\nrc_msgs_ps={:.1}\nrc_enc_us={:.0}\nrc_gap_us={:.0}\nrc_send_us={:.0}\nrc_loop_us={:.0}\nrc_blocks_frame={:.0}\nrc_blocks_sent={:.0}\nrc_src_bytes={:.0}\ncq_in={}\ncq_out={}\ncq_drop={}\nchan_hz={}\nchan_changes={}\nok",
                    s.active_mcs.map(|m| m.index()),
                    s.seq, s.tx_fps, s.iq_mbps, s.nacks_handled, s.retransmits,
                    fb.snr_db, fb.bler,
                    shared.connected.load(Ordering::Relaxed),
                    s.video_bitrate_bps / 1000, s.rc_air_scale,
                    s.rc_one_burst / 1000.0, s.rc_ceiling / 1000.0,
                    s.rc_budget / 1000.0, s.rc_blocks_ps, s.rc_msgs_ps,
                    s.rc_enc_us, s.rc_gap_us, s.rc_send_us, s.rc_loop_us,
                    s.rc_blocks_frame, s.rc_blocks_sent, s.rc_src_bytes,
                    crate::conv_tx::CQ_IN.load(Ordering::Relaxed),
                    crate::conv_tx::CQ_OUT.load(Ordering::Relaxed),
                    crate::conv_tx::CQ_DROP.load(Ordering::Relaxed),
                    shared.chan_hz.load(Ordering::Relaxed),
                    shared.chan_changes.load(Ordering::Relaxed)
                );
                // the camera and what pass-through did with it (the camera control screen)
                let r = &shared.rtsp;
                let (cw, ch) = *r.dims.lock().unwrap();
                let extra = format!(
                    "mode={}\npace_floor_us={}\ncam_status={}\ncam_res={cw}x{ch}\ncam_fps={:.1}\ncam_kbps={}\ncam_gop={}\npass_fps={:.1}\npass_kbps={:.0}\npass_drop_late={}\npass_drop_nokey={}\npass_idr_req={}\nlink_cap_kbps={:.0}\nonvif_status={}\ncam_want_kbps={}\ncam_set_kbps={}\ncam_ceiling_kbps={}\nok",
                    shared.mode_note.lock().unwrap().replace('=', "%3D"),
                    shared.pace_floor_us.load(Ordering::Relaxed),
                    // one line per key: an `=` inside (a camera URL) would split it in the
                    // console clients
                    r.status().replace('=', "%3D"),
                    r.fps_x10.load(Ordering::Relaxed) as f32 / 10.0,
                    r.kbps.load(Ordering::Relaxed),
                    r.gop.load(Ordering::Relaxed),
                    s.pass_fps, s.pass_kbps, s.pass_drop_late, s.pass_drop_nokey, s.pass_idr_req,
                    s.link_cap_bps / 1000.0,
                    r.onvif_status.lock().unwrap().replace('=', "%3D"),
                    r.want_kbps.load(Ordering::Relaxed),
                    r.onvif_kbps.load(Ordering::Relaxed),
                    cam_ceiling(&shared),
                );
                format!("{}{}", out.strip_suffix("ok").unwrap_or(&out), extra)
            }
            Some("save") => save_config(&shared),
            Some("set") => {
                let (Some(key), Some(val)) = (it.next(), it.next()) else {
                    return "err usage: set <key> <value>".into();
                };
                let mut c = shared.config.lock().unwrap();
                match apply_set(&mut c, key, val) {
                    Ok(()) => {
                        log(&format!("control: set {key} {val}"));
                        "ok".into()
                    }
                    Err(e) => e,
                }
            }
            _ => "err unknown command (get / stats / set / save / say / msgs / quit)".into(),
        }
    });
    control::spawn(addr, h);
}

#[cfg(feature = "gui")]
pub struct TxApp {
    shared: Arc<Shared>,
    tex: Option<egui::TextureHandle>,
    tex_ver: u64,
    addr_buf: String,
    board: nyx_common::boardctl::BoardCtl,
    /// v40.34: the board tuning fields (shared form with the other apps).
    form: nyx_common::ui::RadioForm,
    /// v40.33: pasted licence text + last result of applying it
    lic_buf: String,
    lic_status: String,
    /// v12: message input box.
    msg_buf: String,
    /// v40.34: the settings drawer (gear / H key).
    drawer_open: bool,
    /// Draw the SNR and frame-rate plates over the video (drawer header).
    hud_plates: bool,
}

#[cfg(feature = "gui")]
impl TxApp {
    pub fn new(shared: Arc<Shared>) -> Self {
        let addr_buf = shared.channel_addr.lock().unwrap().clone();
        let sh = shared.clone();
        let board = nyx_common::boardctl::BoardCtl::spawn(
            Arc::new(move || {
                let a = sh.channel_addr.lock().unwrap().clone();
                let host = a.split(':').next().unwrap_or("192.168.0.10");
                format!("{host}:7202")
            }),
            &["get", "stats", "hop status", "license", "role"],
        );
        TxApp {
            shared,
            msg_buf: String::new(),
            tex: None,
            tex_ver: u64::MAX,
            addr_buf,
            board,
            form: nyx_common::ui::RadioForm::default(),
            lic_buf: String::new(),
            lic_status: String::new(),
            drawer_open: true,
            hud_plates: true,
        }
    }

    /// The settings drawer open or closed (the phone starts with the picture).
    pub fn set_drawer_open(&mut self, open: bool) {
        self.drawer_open = open;
    }

    /// Everything behind the gear.
    fn drawer(&mut self, ui: &mut egui::Ui) {
        use nyx_common::theme as th;
        use nyx_common::ui as nu;
        let st = self.board.snapshot();
        let connected = self.shared.connected.load(Ordering::Relaxed);

        // ---- video source ----
        th::section(ui, "Video", true, |ui| {
            let mut cfg = self.shared.config.lock().unwrap();
            th::row(ui, "Source", |ui| {
                egui::ComboBox::from_id_salt("src").selected_text(cfg.source.label()).show_ui(ui, |ui| {
                    for s in [SourceKind::Pattern, SourceKind::Webcam, SourceKind::Rtsp, SourceKind::RandomData] {
                        ui.selectable_value(&mut cfg.source, s, s.label());
                    }
                });
                egui::ComboBox::from_id_salt("codec").selected_text(cfg.codec.label()).show_ui(ui, |ui| {
                    for c in [Codec::H264, Codec::Mjpeg] {
                        ui.selectable_value(&mut cfg.codec, c, c.label());
                    }
                });
            });
            if cfg.source == SourceKind::Rtsp {
                // v40.45: the camera is a string; the client thread reconnects when it changes
                th::row(ui, "Camera URL", |ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut cfg.rtsp_url)
                            .hint_text("rtsp://user:pass@192.168.1.64:554/stream1")
                            .desired_width(f32::INFINITY),
                    );
                });
                ui.checkbox(&mut cfg.rtsp_pass, "Send the camera's H.264 as it is")
                    .on_hover_text("No decoding and re-encoding: the camera's bitrate and key-frame interval go on air unchanged. Lower latency; the link cannot slow the camera down.");
                ui.label(egui::RichText::new(format!("camera: {}", self.shared.rtsp.status())).small().color(th::DIM));
                // ONVIF (onvif.rs): with pass-through nothing on this side re-encodes, so the
                // only way to fit the picture into the link is to ask the CAMERA for less. The
                // thread moves the camera's own bitrate limit and nothing else.
                ui.checkbox(&mut cfg.cam_adapt, "Camera follows the link (ONVIF)")
                    .on_hover_text(
                        "Ask the camera itself for a lower or higher bitrate as the link changes, over ONVIF (the user and password of the RTSP URL).",
                    );
                if cfg.cam_adapt {
                    th::row(ui, "ONVIF address", |ui| {
                        ui.add(
                            egui::TextEdit::singleline(&mut cfg.onvif_url)
                                .hint_text("auto: port 80 of the camera")
                                .desired_width(f32::INFINITY),
                        )
                        .on_hover_text("Leave empty unless the camera answers ONVIF on another address or port.");
                    });
                    th::row(ui, "Ceiling", |ui| {
                        ui.add(
                            egui::DragValue::new(&mut cfg.cam_max_kbps)
                                .speed(50.0)
                                .range(0..=50_000)
                                .suffix(" kbps"),
                        )
                        .on_hover_text("Never ask the camera for more than this. 0 = the limit the camera already had.");
                    });
                    let onvif = self.shared.rtsp.onvif_status.lock().unwrap().clone();
                    ui.label(egui::RichText::new(format!("onvif: {onvif}")).small().color(th::DIM));
                    let want = self.shared.rtsp.want_kbps.load(Ordering::Relaxed);
                    let set = self.shared.rtsp.onvif_kbps.load(Ordering::Relaxed);
                    if want > 0 || set > 0 {
                        ui.label(
                            egui::RichText::new(format!("link wants {want} kbps, camera set to {set} kbps"))
                                .small()
                                .color(th::DIM),
                        );
                    }
                }
            }
            const RES: [(usize, usize, &str); 7] = [
                (320, 240, "320 x 240"),
                (480, 360, "480 x 360"),
                (640, 480, "640 x 480"),
                (960, 540, "960 x 540"),
                (1280, 720, "1280 x 720 (HD)"),
                (1920, 1080, "1920 x 1080 (FHD)"),
                (2560, 1440, "2560 x 1440 (2K)"),
            ];
            let mut res_idx = RES.iter().position(|&(w, h, _)| (w, h) == (cfg.width, cfg.height)).unwrap_or(2);
            th::row(ui, "Resolution", |ui| {
                egui::ComboBox::from_id_salt("res").selected_text(RES[res_idx].2).show_ui(ui, |ui| {
                    for (i, &(_, _, label)) in RES.iter().enumerate() {
                        ui.selectable_value(&mut res_idx, i, label);
                    }
                });
            });
            (cfg.width, cfg.height) = (RES[res_idx].0, RES[res_idx].1);
            th::row(ui, "Frame rate", |ui| {
                ui.add(egui::Slider::new(&mut cfg.fps, 1.0..=60.0).suffix(" fps"));
            });
            th::row(ui, "Quality", |ui| {
                ui.add(egui::Slider::new(&mut cfg.jpeg_quality, 20..=95));
            });
            // v12.3: ONE dropdown, Auto vs Pin: choosing "Auto" or "Pin MCSx" is unambiguous.
            th::row(ui, "Rate / MCS", |ui| {
                let rate_label = if cfg.auto_mcs { "Auto".to_string() } else { format!("Pin {}", cfg.mcs.label()) };
                egui::ComboBox::from_id_salt("mcs").selected_text(rate_label).show_ui(ui, |ui| {
                    ui.selectable_value(&mut cfg.auto_mcs, true, "Auto")
                        .on_hover_text("Pick the rate from receiver feedback each block.");
                    ui.separator();
                    for m in Mcs::ALL {
                        let sel = !cfg.auto_mcs && cfg.mcs == m;
                        if ui.selectable_label(sel, format!("Pin {}", m.label())).clicked() {
                            cfg.auto_mcs = false;
                            cfg.mcs = m;
                        }
                    }
                });
                let pause_label = if cfg.paused { "Resume" } else { "Pause" };
                if nu::action(ui, pause_label) {
                    cfg.paused = !cfg.paused;
                    log(if cfg.paused { "paused" } else { "resumed" });
                }
            });
            // v40.44z: the base layer itself, so the trade can be made from the cockpit: with it
            // the picture survives a fade (soak 72 % vs 59 % of the time with a picture), without
            // it the glass-to-glass latency drops 61 -> 28 ms (10/9, the base blocks queue ahead
            // of the main ones). Console `set simulcast 0|1` sets the same flag.
            if ui
                .checkbox(&mut cfg.simulcast, "Simulcast base layer")
                .on_hover_text("Send a small MCS0 stream beside the main one; the receiver shows it when the main layer is late. Keeps a picture through fades, costs ~30 ms of latency.")
                .changed()
            {
                log(if cfg.simulcast { "simulcast on" } else { "simulcast off" });
            }
            // v40: the long-reach lifebuoy: pin the simulcast base layer to the QPSK x2 repeat step
            ui.add_enabled_ui(cfg.simulcast, |ui| {
                let mut far = cfg.base_mcs == 6;
                if ui
                    .checkbox(&mut far, "Long-range base layer (QPSK×2)")
                    .on_hover_text("Pin the base layer to the repeat rung: about +3 dB of reach, at twice the base-layer airtime.")
                    .changed()
                {
                    cfg.base_mcs = if far { 6 } else { 0 };
                }
            });
            // v40.46 latency test: pairs with "Counter on screen" in the ground app
            ui.checkbox(&mut cfg.stamp, "Counter in the picture (latency test)")
                .on_hover_text("Burns a millisecond counter into the video before encoding. The ground app shows the same counter (Latency test): freeze both and subtract.");
            ui.label(egui::RichText::new(format!("camera: {}", self.shared.webcam.status())).small().color(th::DIM));
        });

        nu::link_section(ui, &st, nu::Role::Aircraft, &self.board);
        nu::role_section(ui, &st, &self.board);
        nu::licence_section(ui, &st, None, &mut self.lic_buf, &mut self.lic_status, &self.board);
        nu::radio_section(ui, &st, nu::Role::Aircraft, &mut self.form, &self.board, &mut |hz| {
            crate::set_samp_hz(hz);
        });
        {
            let msgs = self.shared.msgs.lock().unwrap().clone();
            if let Some(t) = nu::messages_section(ui, &msgs, &mut self.msg_buf) {
                self.shared.text_out.lock().unwrap().push(t.clone());
                let mut m = self.shared.msgs.lock().unwrap();
                m.push(format!("→ {t}"));
                if m.len() > 200 {
                    m.remove(0);
                }
            }
        }
        {
            let mut udp = self.shared.udp_mode.load(Ordering::Relaxed);
            let udp0 = udp;
            let connect = nu::connection_section(ui, &mut self.addr_buf, "ip:7010", connected, Some(&mut udp));
            if udp != udp0 {
                self.shared.udp_mode.store(udp, Ordering::Relaxed);
                if let Some(s) = self.shared.ctl_stream.lock().unwrap().take() {
                    let _ = s.shutdown(std::net::Shutdown::Both);
                }
                log(&format!("transport -> {}", if udp { "UDP" } else { "TCP" }));
            }
            if connect {
                *self.shared.channel_addr.lock().unwrap() = self.addr_buf.trim().to_string();
                if let Some(s) = self.shared.ctl_stream.lock().unwrap().take() {
                    let _ = s.shutdown(std::net::Shutdown::Both);
                }
                log(&format!("connect requested -> {}", self.addr_buf.trim()));
            }
        }
        th::section(ui, "Advanced", false, |ui| {
            let mut cfg = self.shared.config.lock().unwrap();
            // v11.2: transmit path: PL fabric (default) vs PC IQ (test measurement / fallback)
            th::row(ui, "Air gap", |ui| {
                ui.add(egui::Slider::new(&mut cfg.air_gap_ms, 0.4..=20.0).suffix(" ms"))
                    .on_hover_text("Pause between PHY frames. 3 ms matches the board's pacing; 20 ms caps the link at 50 frames/s.");
            });
            if crate::pc::PcTx::AVAILABLE {
                th::row(ui, "TX path", |ui| {
                    let cur = if cfg.txbits || cfg.txpl { "pl" } else { "pc" };
                    if let Some(v) = nu::segmented(ui, &[("PL fabric", "pl"), ("PC IQ (test)", "pc")], cur) {
                        let pl = v == "pl";
                        cfg.txbits = pl;
                        cfg.txpl = pl;
                    }
                });
            }
            th::row(ui, "Repeat", |ui| {
                let mut rep = cfg.rep_count as u32;
                if ui.add(egui::Slider::new(&mut rep, 1..=4).text("×")).on_hover_text("Send every block this many times up front.").changed() {
                    cfg.rep_count = rep;
                }
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut cfg.arq, "ARQ").on_hover_text("Retransmit a block when the receiver NACKs it.");
                ui.add_enabled_ui(cfg.arq, |ui| {
                    ui.checkbox(&mut cfg.harq_ir, "IR retransmissions").on_hover_text(
                        "Send a different redundancy version each retry, so the receiver combines them instead of repeating.",
                    );
                });
            });
            drop(cfg);
            ui.add_space(6.0);
            th::card_title(ui, "tx details");
            let stats = self.shared.stats.lock().unwrap().clone();
            let fb = self.shared.feedback.lock().unwrap().clone();
            let fb_txt = match fb.updated {
                Some(t) if t.elapsed().as_secs_f32() < 3.0 => format!("{:.1} dB / {:.1} %", fb.snr_db, fb.bler * 100.0),
                _ => "no feedback".to_string(),
            };
            egui::Grid::new("tx_stats").num_columns(2).striped(true).spacing([16.0, 6.0]).show(ui, |ui| {
                let kvs: [(&str, String); 9] = [
                    ("Active MCS", stats.active_mcs.map(|m| m.label().to_string()).unwrap_or_else(|| "-".into())),
                    ("PHY seq sent", format!("{}", stats.seq)),
                    ("Blocks / video frame", format!("{}", stats.blocks_per_frame)),
                    ("App bytes / frame", format!("{} B", stats.app_bytes_per_frame)),
                    ("TX video rate", format!("{:.1} fps", stats.tx_fps)),
                    ("IQ rate to the board", format!("{:.1} Mbit/s", stats.iq_mbps)),
                    ("NACKs / retransmits", format!("{} / {}", stats.nacks_handled, stats.retransmits)),
                    ("Video bitrate / IDRs", format!("{} kbit/s / {}", stats.video_bitrate_bps / 1000, stats.idr_sent)),
                    ("Feedback SNR / BLER", fb_txt),
                ];
                for (k, v) in kvs {
                    ui.label(egui::RichText::new(k).color(th::DIM));
                    ui.label(egui::RichText::new(v).monospace());
                    ui.end_row();
                }
            });
        });
    }
}

#[cfg(feature = "gui")]
impl eframe::App for TxApp {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = root.ctx().clone();
        let ver = self.shared.preview_version.load(Ordering::Relaxed);
        if ver != self.tex_ver {
            // v40.25: same guard as nyx-rx: a 0x0 frame -> drop; a size change -> a NEW handle.
            let snap = self.shared.preview.lock().unwrap().clone();
            if let Some(f) = snap {
                if f.width == 0 || f.height == 0 || f.rgb.len() != f.width * f.height * 3 {
                    log(&format!("gui: dropping preview frame of odd size {}x{}", f.width, f.height));
                } else {
                    let img = to_color_image(&f);
                    let same = self.tex.as_ref().map_or(false, |t| t.size() == [f.width, f.height]);
                    match &mut self.tex {
                        Some(t) if same => t.set(img, egui::TextureOptions::LINEAR),
                        _ => {
                            self.tex = Some(ctx.load_texture("tx", img, egui::TextureOptions::LINEAR));
                        }
                    }
                }
            }
            self.tex_ver = ver;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::H)) {
            self.drawer_open = !self.drawer_open;
        }
        // O: the readouts over the video, the same switch as in the drawer header.
        if ctx.input(|i| i.key_pressed(egui::Key::O)) {
            self.hud_plates = !self.hud_plates;
        }

        use nyx_common::ui as nu;
        let st = self.board.snapshot();
        let connected = self.shared.connected.load(Ordering::Relaxed);
        let st_s = self.shared.stats.lock().unwrap().clone();
        let fb_s = self.shared.feedback.lock().unwrap().clone();
        let fb_fresh = fb_s.updated.is_some_and(|t| t.elapsed().as_secs_f32() < 3.0);
        let paused = self.shared.config.lock().unwrap().paused;
        let mut pills = vec![nu::link_pill(&st), nu::licence_pill(&st)];
        if paused {
            pills.push(("PAUSED".into(), nyx_common::theme::WARN));
        }
        let data = nu::HudData {
            connected,
            live: connected && st_s.tx_fps >= 5.0 && !paused,
            snr_db: if fb_fresh { fb_s.snr_db } else { 0.0 },
            fps: st_s.tx_fps,
            kbps: (st_s.video_bitrate_bps / 1000) as f32,
            mcs: st_s.active_mcs.map(|m| m.label().to_string()).unwrap_or_else(|| "-".into()),
            pills,
            banner: None,
            title: "NYXHOP · AIRCRAFT".into(),
            empty_text: "waiting for the camera…".into(),
            plates: self.hud_plates,
        };
        let tex = self.tex.clone();
        let mut open = self.drawer_open;
        let mut plates = self.hud_plates;
        nu::screen(root, &mut open, &mut plates, "Aircraft setup", |ui| self.drawer(ui), |ui| nu::hud(ui, tex.as_ref(), &data));
        self.drawer_open = open;
        self.hud_plates = plates;
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}

/// Rolling event-rate/byte-rate counter.
pub struct Rolling {
    events: std::collections::VecDeque<(Instant, usize)>,
    window: std::time::Duration,
}

impl Rolling {
    pub fn new(secs: f32) -> Self {
        Rolling {
            events: Default::default(),
            window: std::time::Duration::from_secs_f32(secs),
        }
    }
    pub fn push(&mut self, amount: usize) {
        self.events.push_back((Instant::now(), amount));
        self.trim();
    }
    fn trim(&mut self) {
        // checked_sub: Instant counts from boot on Windows — plain
        // subtraction panics right after a reboot.
        let Some(cut) = Instant::now().checked_sub(self.window) else { return };
        while self.events.front().is_some_and(|&(t, _)| t < cut) {
            self.events.pop_front();
        }
    }
    pub fn per_sec(&mut self) -> f32 {
        self.trim();
        self.events.iter().map(|&(_, a)| a).sum::<usize>() as f32
            / self.window.as_secs_f32()
    }
    pub fn count_per_sec(&mut self) -> f32 {
        self.trim();
        self.events.len() as f32 / self.window.as_secs_f32()
    }
}
