//! nyx-rx: receiver. Pulls impaired baseband IQ from the channel node over
//! TCP, runs the full PHY receive chain, reassembles and displays the video,
//! and sends telemetry feedback + NACKs back for MCS adaptation and ARQ.
//!
//! Usage: nyx-rx [--channel 127.0.0.1:7011]

mod pc;
mod web;

use std::collections::VecDeque;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use num_complex::Complex32 as C32;
use nyx_common::codec::VideoDecoder;
use nyx_common::logging::{self, StatusLogger, log};
use nyx_common::source::bytes_to_image;
use nyx_common::{RgbFrame, decode_jpeg, to_color_image};
use nyx_link::{Reassembler, SourceType, parse_block};
use nyx_common::Opts;
use nyx_proto::{DEFAULT_RX_PORT, Mcs, Msg, cli_arg, read_msg, write_msg};

use crate::pc::{DecodeOutcome, DemodResult, PcRx, SigPayload};

#[derive(Clone, Default)]
struct RxMetrics {
    snr_db: f32,
    cfo_hz: f32,
    pre_ber: f32,
    sync_metric: f32,
    sync_failures: u64,
    segs_ok: u64,
    segs_lost: u64,
    bler_recent: f32,
    frames_rx: u64,
    frames_lost: u64,
    rx_fps: f32,
    /// v36.1: frames actually DISPLAYED per second (rx_fps counts both simulcast layers).
    disp_fps: f32,
    layer_main: bool,
    // v37.1: main-layer decode ok/fail, needed to diagnose a main layer that never comes up (rx_fps
    // counts frames that ARRIVE and says nothing about decoding).
    pics_ok: u64,
    pics_fail: u64,
    goodput_kbps: f32,
    active_mcs: Option<Mcs>,
    backlog_dropped: u64,
    harq_combines: u64,
    harq_recovered: u64,
    bicm_runs: u64,
    bicm_rescues: u64,
    sig_failures: u64,
}

#[derive(Default)]
struct UiState {
    rx_frame: Option<RgbFrame>,
    rx_version: u64,
    /// frame_id of the frame being shown, for the G2G measurement at the DISPLAY (as opposed to
    /// "G2G out", which is taken right after decode).
    rx_id: u32,
    metrics: RxMetrics,
    constellation: Vec<(f32, f32)>,
    chan_mag: Vec<f32>,
    snr_history: VecDeque<f32>,
    bler_history: VecDeque<f32>,
}

/// One entry in the demod queue: (cap_seq, mcs, rv, time queued, data). v21.1: the Instant lets the
/// worker DROP stale video captures (drop-to-live); after a stall the backlog once drained the FIFO
/// with a steady ~10 s delay for a whole minute.
type CapItem = (u64, u8, u8, Instant, RxCapture);

pub struct Shared {
    ui: Mutex<UiState>,
    connected: AtomicBool,
    stop: AtomicBool,
    /// Radio/channel address, editable at runtime from the GUI.
    channel_addr: Mutex<String>,
    writer: Mutex<Option<TcpStream>>,
    /// v21.1: outgoing Feedback/Nack/Text queue. demod_loop and the GUI only try_send; the
    /// net-writer thread is the ONLY place that touches the socket. A blocking TCP write from
    /// demod_loop once stalled the whole pipeline when the buffer filled.
    net_tx: SyncSender<Msg>,
    /// UDP mode: socket + daemon address for Feedback/Nack (replaces the writer).
    udp_out: Mutex<Option<(std::net::UdpSocket, std::net::SocketAddr)>>,
    /// Capture transport: false = TCP, true = UDP. Switchable from the GUI.
    udp_mode: AtomicBool,
    dropped: AtomicU64,
    /// Runtime-tunable receiver options (control console).
    bicm_enabled: AtomicBool,
    ldpc_iters: std::sync::atomic::AtomicUsize,
    /// I/Q format probe (0=asis 1=conj 2=swap 3=swap+conj 4=negI).
    iq_mode: std::sync::atomic::AtomicUsize,
    /// v12.3: MCS of the most recent delivered video frame (0xFF = none yet). Written by both the
    /// fabric DecFrame path and the PC demod path; the UI reads it, so it no longer jumps on leaks
    /// or junk.
    disp_mcs: std::sync::atomic::AtomicU8,
    /// v-slice: DELIVER an incomplete frame (>= 60 % of its blocks) instead of dropping it: one
    /// torn band in the picture, but the video keeps MOVING rather than freezing until the next
    /// IDR. `set partial 0|1`.
    partial_ok: AtomicBool,
    /// v14 jitter buffer: a frame queue plus a thread that plays out at the source's steady cadence
    /// (absorbs the jolt of a rate change or HOL blocking). jitter_ms = target buffer depth (0 =
    /// off, show at once). frame_period_us = EMA of the source period.
    playout: Mutex<VecDeque<(u32, RgbFrame)>>,
    jitter_ms: AtomicU64,
    frame_period_us: AtomicU64,
    /// Most recent EqFrame; while one is current, duplicate IQ is dropped (the FD demod covers it).
    /// Option because an Instant on Windows cannot go backwards.
    fd_last: Mutex<Option<Instant>>,
    /// IQ of the last few captures while FD is covering: a frame that does NOT fit the 13-symbol
    /// grid falls back to the IQ copy with the same seq, and a capture the fabric was too BUSY to
    /// produce an EqFrame for (busy_drops ~30 % with a lot of junk) is demodulated by the sweeper
    /// after 150 ms. Without this nearly half the video frames vanished without trace.
    /// v12: two-way message log for the GUI ("→" sent over the OTA control channel, "←" received as
    /// a SourceType::Text frame on the video link).
    msgs: Mutex<Vec<String>>,
    /// v40.22: user data from the video stream (SourceType::Data) -> UDP out (--tlm-out, default
    /// 127.0.0.1:14558); tlm_rx = datagrams pushed out.
    pub tlm_out: Mutex<Option<(std::net::UdpSocket, std::net::SocketAddr)>>,
    pub tlm_rx: AtomicU64,
    /// v40.33: the transmit end's licence lines (SourceType::Info) for the UI.
    far_lic: Mutex<String>,
    iq_stash: Mutex<std::collections::VecDeque<(u64, Instant, Vec<C32>)>>,
}

/// The standalone app: options from the command line, a window of its own (or
/// `--headless`).
pub fn run(opts: Opts) -> eframe::Result {
    logging::init("rx");
    let shared = setup(&opts);
    if opts.flag("--headless") {
        log("headless: no window opened; control through --ctl");
        while !shared.stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_secs(2));
            let m = shared.ui.lock().unwrap().metrics.clone();
            log(&format!(
                "hl mcs={:?} snr={:.1} bler={:.3} rx_fps={:.1} disp_fps={:.1} \
                 lop={} pics={}/{} segs={}/{} goodput={:.0}kbps conn={}",
                m.active_mcs.map(|x| x.index()),
                m.snr_db,
                m.bler_recent,
                m.rx_fps,
                m.disp_fps,
                if m.layer_main { "main" } else { "nen" },
                m.pics_ok,
                m.pics_fail,
                m.segs_ok,
                m.segs_lost,
                m.goodput_kbps,
                shared.connected.load(Ordering::Relaxed),
            ));
        }
        return Ok(());
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([560.0, 900.0])
            .with_title("NyxHop RX"),
        ..Default::default()
    };
    let s = shared.clone();
    let result = eframe::run_native(
        "NyxHop RX",
        options,
        Box::new(move |cc| {
            nyx_common::ui::touch_style(&cc.egui_ctx);
            Ok(Box::new(RxApp::new(s)))
        }),
    );
    shutdown(&shared);
    result
}

/// Everything but the window: threads, sockets, the source. The combined app calls
/// this and then hosts the `App` in its own window.
pub fn setup(opts: &Opts) -> Arc<Shared> {
    // --samp <Hz>: the hardware sample rate (30720000 for the 20 MHz mode). The frame structure is
    // in samples, so only the time/Hz conversions change.

    if let Ok(hz) = opts.arg("--samp", "15360000").parse::<u64>() {
        pc::set_samp_rate_hz(hz);
        if hz != 15_360_000 {
            log(&format!("samp rate override: {hz} Hz"));
        }
    }
    let channel_addr = opts.arg("--channel", &format!("127.0.0.1:{DEFAULT_RX_PORT}"));
    log(&format!("connecting to channel node at {channel_addr}"));

    // v21.1: outgoing network queue, 64 messages (feedback ~10/s, short NACK bursts); when full,
    // try_send drops and the next tick makes up for it.
    let (net_tx, net_rx) = sync_channel::<Msg>(64);
    let shared = Arc::new(Shared {
        ui: Mutex::new(UiState::default()),
        connected: AtomicBool::new(false),
        stop: AtomicBool::new(false),
        channel_addr: Mutex::new(channel_addr),
        writer: Mutex::new(None),
        net_tx,
        udp_out: Mutex::new(None),
        udp_mode: AtomicBool::new(!opts.arg("--udp", "").is_empty()),
        dropped: AtomicU64::new(0),
        bicm_enabled: AtomicBool::new(true),
        ldpc_iters: std::sync::atomic::AtomicUsize::new(30),
        iq_mode: std::sync::atomic::AtomicUsize::new(0),
        disp_mcs: std::sync::atomic::AtomicU8::new(0xFF),
        playout: Mutex::new(VecDeque::new()),
        // v-lat: DEFAULT 0 = drop-to-live (show the newest frame at once). Measured (G2G at the
        // DISPLAY, not at decode): jitter 0 -> 39 ms; 40 -> 145 ms; 80 -> 67 ms (the pacing adds
        // one or two frame periods). At bler 0 on the bench a buffer only adds DELAY. For
        // smoothness on a choppy link: the "Smooth" slider or `set jitter N`.
        partial_ok: AtomicBool::new(true),
        jitter_ms: AtomicU64::new(0),
        frame_period_us: AtomicU64::new(40_000),
            fd_last: Mutex::new(None),
        msgs: Mutex::new(Vec::new()),
        tlm_out: Mutex::new({
            let a = opts.arg("--tlm-out", "127.0.0.1:14558");
            match (a.parse::<std::net::SocketAddr>(), std::net::UdpSocket::bind("0.0.0.0:0")) {
                (Ok(addr), Ok(sock)) => Some((sock, addr)),
                _ => None,
            }
        }),
        tlm_rx: AtomicU64::new(0),
        far_lic: Mutex::new(String::new()),
        iq_stash: Mutex::new(std::collections::VecDeque::new()),
    });
    spawn_control(shared.clone(), opts.arg("--ctl", "127.0.0.1:7203"));
    // v40.22: telemetry (MAVLink/GCS) in by UDP -> ground board -> narrow control channel ->
    // aircraft board -> nyx-tx (UDP out). One datagram = one packet (<= 210 B). --tlm-in 0 = off.
    {
        let addr = opts.arg("--tlm-in", "0.0.0.0:14555");
        if addr != "0" {
            match std::net::UdpSocket::bind(&addr) {
                Ok(sock) => {
                    nyx_common::logging::log(&format!(
                        "telemetry in: UDP {addr} -> control channel"));
                    let sh = shared.clone();
                    std::thread::Builder::new()
                        .name("tlm-in".into())
                        .spawn(move || {
                            let mut buf = [0u8; 2048];
                            let mut n_ok = 0u64;
                            let mut n_big = 0u64;
                            loop {
                                let Ok((n, _)) = sock.recv_from(&mut buf) else { continue };
                                if n == 0 {
                                    continue;
                                }
                                if n > 210 {
                                    n_big += 1;
                                    if n_big % 100 == 1 {
                                        nyx_common::logging::log(&format!(
                                            "telemetry: packet {n} B > 210 B, dropped ({n_big})"));
                                    }
                                    continue;
                                }
                                n_ok += 1;
                                if n_ok == 1 {
                                    nyx_common::logging::log(
                                        "telemetry: first UDP packet -> control channel");
                                }
                                send_msg(&sh, &Msg::Tlm { bytes: buf[..n].to_vec() });
                            }
                        })
                        .expect("spawn tlm-in");
                }
                Err(e) => nyx_common::logging::log(&format!(
                    "telemetry: bind {addr} failed: {e}")),
            }
        }
    }
    // Web ground station (dashboard + MJPEG + command proxy). --web 0 to disable.
    if let Ok(p) = opts.arg("--web", "7280").parse::<u16>() {
        if p != 0 {
            web::spawn(shared.clone(), p);
        }
    }

    // Bounded queue between the socket reader and the demod worker: if
    // demodulation cannot keep up, drop frames rather than grow forever.
    let (iq_tx, iq_rx) = sync_channel::<CapItem>(16);
    let sweep_tx = iq_tx.clone(); // the demod_loop sweeper pushes stale stash entries into the pool
    spawn_net(shared.clone(), iq_tx);
    spawn_net_writer(shared.clone(), net_rx);
    spawn_demod(shared.clone(), iq_rx, sweep_tx);
    spawn_playout(shared.clone());

    // v40.8: `--headless`, see the note of the same name in nyx-tx. Important: `disp_fps` (frames
    // ACTUALLY shown) is counted in the decode thread, NOT in the GUI paint loop, so a run without
    // a window still measures the real thing.
    shared
}

/// Stop the threads `setup` started (the window is gone or the mode changes).
pub fn shutdown(shared: &Shared) {
    shared.stop.store(true, Ordering::Relaxed);
}

/// v14: thread that plays frames out at the source's steady period (jitter buffer). Releases one
/// frame per period from the head of the queue; drops some when the buffer runs too deep (less
/// delay), waits when empty (keeps the old frame). jitter_ms = 0 -> show a new frame at once.
fn spawn_playout(shared: Arc<Shared>) {
    fn publish(shared: &Shared, f: (u32, RgbFrame)) {
        // v21.1: NEVER wait for the ui lock. If the GUI holds it (drag/resize/modal on Windows) the
        // frame goes back to the head of the queue and the 4 ms tick tries again. The video
        // pipeline must never stall behind the GUI.
        match shared.ui.try_lock() {
            Ok(mut ui) => {
                ui.rx_id = f.0;
                ui.rx_frame = Some(f.1);
                ui.rx_version += 1;
            }
            Err(_) => shared.playout.lock().unwrap().push_front(f),
        }
    }
    std::thread::Builder::new()
        .name("playout".into())
        .spawn(move || {
            let mut next = Instant::now();
            loop {
                if shared.stop.load(Ordering::Relaxed) {
                    return;
                }
                std::thread::sleep(std::time::Duration::from_millis(4));
                let jms = shared.jitter_ms.load(Ordering::Relaxed);
                if jms == 0 {
                    let f = {
                        let mut q = shared.playout.lock().unwrap();
                        let last = q.pop_back();
                        q.clear();
                        last
                    };
                    if let Some(f) = f {
                        publish(&shared, f);
                    }
                    next = Instant::now();
                    continue;
                }
                let period = std::time::Duration::from_micros(
                    shared.frame_period_us.load(Ordering::Relaxed)
                        .clamp(15_000, 70_000),
                );
                // target depth (frames) = jitter_ms / period; when it runs deeper, drop old frames
                // so the delay does not pile up.
                let target = ((jms as u128 * 1000)
                    / period.as_micros().max(1)) as usize;
                {
                    let mut q = shared.playout.lock().unwrap();
                    while q.len() > target + 3 {
                        q.pop_front();
                    }
                }
                let now = Instant::now();
                if now >= next {
                    let f = shared.playout.lock().unwrap().pop_front();
                    match f {
                        Some(f) => {
                            publish(&shared, f);
                            next += period;
                            // far too late (long hang) -> reset instead of catching up in a burst
                            if now > next
                                && now.duration_since(next) > period * 2
                            {
                                next = now + period;
                            }
                        }
                        None => next = now + period, // underrun: wait
                    }
                }
            }
        })
        .expect("spawn playout");
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
                    send_msg(&shared, &Msg::UserText { text: t.clone() });
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
            Some("get") => format!(
                "bicm={}\niters={}\niqmode={}\nconnected={}\nok",
                shared.bicm_enabled.load(Ordering::Relaxed),
                shared.ldpc_iters.load(Ordering::Relaxed),
                shared.iq_mode.load(Ordering::Relaxed),
                shared.connected.load(Ordering::Relaxed)
            ),
            Some("stats") => {
                let m = shared.ui.lock().unwrap().metrics.clone();
                format!(
                    "mcs={:?}\nsnr={:.1}\ncfo={:.0}\nbler={:.3}\nsegs_ok={}\nsegs_lost={}\nframes_rx={}\nframes_lost={}\nrx_fps={:.1}\ndisp_fps={:.1}\nlayer={}\npics={}/{}\ngoodput_kbps={:.0}\nharq={}/{}\nbicm={}/{}\nsig_fail={}\nok",
                    m.active_mcs.map(|x| x.index()),
                    m.snr_db, m.cfo_hz, m.bler_recent, m.segs_ok, m.segs_lost,
                    m.frames_rx, m.frames_lost, m.rx_fps, m.disp_fps,
                    if m.layer_main { "main" } else { "base" },
                    m.pics_ok, m.pics_fail, m.goodput_kbps,
                    m.harq_combines, m.harq_recovered, m.bicm_runs, m.bicm_rescues,
                    m.sig_failures
                )
            }
            Some("set") => {
                let (Some(key), Some(val)) = (it.next(), it.next()) else {
                    return "err usage: set <key> <value>".into();
                };
                let ok = match key {
                    "bicm" => parse_bool(val)
                        .map(|v| shared.bicm_enabled.store(v, Ordering::Relaxed))
                        .is_some(),
                    "iters" => val
                        .parse::<usize>()
                        .map(|v| shared.ldpc_iters.store(v, Ordering::Relaxed))
                        .is_ok(),
                    "iqmode" => val
                        .parse::<usize>()
                        .map(|v| shared.iq_mode.store(v.min(4), Ordering::Relaxed))
                        .is_ok(),
                    // v14: jitter buffer depth in ms (0 = off, show at once).
                    "partial" => parse_bool(val)
                        .map(|v| shared.partial_ok.store(v, Ordering::Relaxed))
                        .is_some(),
                    "jitter" => val
                        .parse::<u64>()
                        .map(|v| shared.jitter_ms.store(v.min(400), Ordering::Relaxed))
                        .is_ok(),
                    _ => return format!("err unknown key {key}"),
                };
                if ok {
                    log(&format!("control: set {key} {val}"));
                    "ok".into()
                } else {
                    format!("err bad value for {key}")
                }
            }
            _ => "err unknown command (get / stats / set / quit)".into(),
        }
    });
    control::spawn(addr, h);
}

// ---------------------------------------------------------------- net --

fn spawn_net(shared: Arc<Shared>, iq_tx: SyncSender<CapItem>) {
    std::thread::Builder::new()
        .name("rx-net".into())
        .spawn(move || {
            let mut logged_fail = false;
            loop {
                if shared.stop.load(Ordering::Relaxed) {
                    return;
                }
                // ---- UDP mode (GUI toggle / --udp <ip:port>) ---------
                if shared.udp_mode.load(Ordering::Relaxed) {
                    // Destination: an explicit --udp, otherwise the host of the channel address
                    // plus the standard UDP port.
                    let explicit = cli_arg("--udp", "");
                    let target = if explicit.is_empty() {
                        let a = shared.channel_addr.lock().unwrap().clone();
                        a.split(':').next().map(|h| {
                            format!("{h}:{}", nyx_proto::UDP_CAPTURE_PORT)
                        })
                    } else {
                        Some(explicit)
                    };
                    let Some(daemon) =
                        target.and_then(|t| t.parse::<std::net::SocketAddr>().ok())
                    else {
                        log("udp: invalid daemon address");
                        std::thread::sleep(Duration::from_secs(1));
                        continue;
                    };
                    let sock =
                        std::net::UdpSocket::bind("0.0.0.0:0").expect("udp bind");
                    let _ = sock.set_read_timeout(Some(Duration::from_millis(300)));
                    *shared.udp_out.lock().unwrap() =
                        Some((sock.try_clone().expect("udp clone"), daemon));
                    log(&format!("UDP capture mode -> {daemon}"));
                    udp_loop(&shared, &sock, daemon, &iq_tx);
                    *shared.udp_out.lock().unwrap() = None;
                    shared.connected.store(false, Ordering::Relaxed);
                    continue; // flag changed -> the outer loop picks the transport again
                }
                // ---- TCP mode ----------------------------------------
                let addr = shared.channel_addr.lock().unwrap().clone();
                match TcpStream::connect(&addr) {
                    Ok(stream) => {
                        let _ = stream.set_nodelay(true);
                        // v21.1: half-open watchdog. The daemon once hit "write failed" (buffer
                        // full while the RX was stalled) and dropped its WRITE clone, but its READ
                        // clone kept the fd, so no FIN ever came and read() on this side waited on
                        // a dead connection for ever. The video link always carries ~20 msg/s: 10 s
                        // of silence is a real death -> read times out -> the outer loop
                        // reconnects.
                        let _ = stream.set_read_timeout(
                            Some(Duration::from_secs(10)));
                        log(&format!("connected to channel node {addr}"));
                        logged_fail = false;
                        shared.connected.store(true, Ordering::Relaxed);
                        *shared.writer.lock().unwrap() =
                            Some(stream.try_clone().expect("clone"));
                        read_loop(&shared, stream, &iq_tx);
                        shared.connected.store(false, Ordering::Relaxed);
                        *shared.writer.lock().unwrap() = None;
                        log("connection to channel node lost");
                    }
                    Err(e) => {
                        if !logged_fail {
                            log(&format!(
                                "cannot reach channel node at {addr} ({e}); retrying every 1s"
                            ));
                            logged_fail = true;
                        }
                        std::thread::sleep(Duration::from_secs(1));
                    }
                }
            }
        })
        .expect("spawn rx-net");
}

fn read_loop(
    shared: &Shared,
    mut stream: TcpStream,
    iq_tx: &SyncSender<CapItem>,
) {
    // EqFrame bookkeeping (fabric H+Y): monitoring stage, continuous quality measurement logged at
    // 1 Hz; the demod path still runs on IQ as before.
    let mut eqf_cnt: u64 = 0;
    let mut eqf_last: Option<std::time::Instant> = None;
    let mut llrf_cnt = 0u64;
    let mut decf_cnt = 0u64;
    let mut decf_nz = 0u64; // v40.25 diagnostic: DecFrames with mcs != 0 (probe frames)
    let mut decf_last: Option<std::time::Instant> = None;
    let mut llrf_last: Option<std::time::Instant> = None;
    let mut msg_cnt: u64 = 0;
    log("read_loop start");
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        let m = read_msg(&mut stream);
        msg_cnt += 1;
        if msg_cnt <= 5 || msg_cnt % 500 == 0 {
            log(&format!("read_loop msg #{msg_cnt}: {}",
                match &m {
                    Ok(Msg::IqFrame { samples, .. }) =>
                        format!("IqFrame {} samples", samples.len()),
                    Ok(Msg::EqFrame { y, .. }) =>
                        format!("EqFrame {} y", y.len()),
                    Ok(Msg::LlrFrame { words, .. }) =>
                        format!("LlrFrame {} words", words.len()),
                    Ok(_) => "other".into(),
                    Err(e) => format!("ERR {e}"),
                }));
        }
        match m {
            Ok(Msg::IqFrame { seq, mcs, rv, samples }) => {
                push_capture(shared, iq_tx, seq, mcs, rv, &samples);
            }
            Ok(Msg::EqFrame { seq, us, theta, h, y }) => {
                eqf_cnt += 1;
                // Spectral grid -> FD demod. Mark the moment so push_capture drops the duplicate IQ
                // (the fabric is covering); if the daemon turns eqframe off, IQ flows again by
                // itself after 1 s, a fallback that needs no configuration.
                *shared.fd_last.lock().unwrap() = Some(std::time::Instant::now());
                let grid: Vec<Vec<C32>> = y
                    .chunks(600)
                    .filter(|c| c.len() == 600)
                    .map(|c| nyx_proto::dequantize(c, 1.0 / 32768.0))
                    .collect();
                if !grid.is_empty() {
                    let _ = iq_tx.try_send((seq, 0xFF, 0, Instant::now(), RxCapture::Grid(grid)));
                }
                let due = eqf_last.is_none_or(|t| {
                    t.elapsed() >= std::time::Duration::from_secs(1)
                });
                if due {
                    eqf_last = Some(std::time::Instant::now());
                    let (eo_db, evm, ns) = eqf_quality(&h, &y);
                    let snr = -20.0 * evm.max(1e-6).log10();
                    log(&format!(
                        "EQF seq={seq} cnt={eqf_cnt} n_syms={ns}                          pre_eo={eo_db:+.1}dB sig_evm={:.1}% (~{snr:.1}dB)                          us={us} theta={theta}",
                        evm * 100.0
                    ));
                }
            }
            Ok(Msg::LlrFrame { seq, us, theta, mcs, rv, seq_lsb, words }) => {
                llrf_cnt += 1;
                // LLRs demapped in the fabric, through the same duplicate-IQ gate as EqFrame (the
                // daemon still falls back to EqFrame when SIG fails).
                *shared.fd_last.lock().unwrap() = Some(std::time::Instant::now());
                let nw = words.len();
                let _ = iq_tx.try_send((
                    seq, 0xFF, rv, Instant::now(),
                    RxCapture::Llr { mcs, rv, seq_lsb, words },
                ));
                let due = llrf_last.is_none_or(|t| {
                    t.elapsed() >= std::time::Duration::from_secs(1)
                });
                if due {
                    llrf_last = Some(std::time::Instant::now());
                    log(&format!(
                        "LLRF seq={seq} cnt={llrf_cnt} mcs={mcs} rv={rv} \
                         lsb={seq_lsb:02x} words={nw} us={us} theta={theta}"
                    ));
                }
            }
            Ok(Msg::DecFrame { seq, mcs, seq_lsb, iters, llr_sum, payload }) => {
                decf_cnt += 1;
                if mcs != 0 {
                    decf_nz += 1;
                    if decf_nz <= 20 || decf_nz % 200 == 0 {
                        log(&format!(
                            "DECF-nz #{decf_nz}: seq={seq} mcs={mcs} lsb={seq_lsb:02x} iters={iters} len={}",
                            payload.len()));
                    }
                }
                shared.disp_mcs.store(mcs, Ordering::Relaxed);
                *shared.fd_last.lock().unwrap() =
                    Some(std::time::Instant::now());
                let _ = iq_tx.try_send((
                    seq, 0xFF, 0, Instant::now(),
                    RxCapture::Dec { mcs, seq_lsb, iters, llr_sum, payload },
                ));
                let due = decf_last.is_none_or(|t| {
                    t.elapsed() >= std::time::Duration::from_secs(1)
                });
                if due {
                    decf_last = Some(std::time::Instant::now());
                    log(&format!(
                        "DECF seq={seq} cnt={decf_cnt} mcs={mcs} \
                         lsb={seq_lsb:02x} iters={iters} sum={llr_sum}"
                    ));
                }
            }
            Ok(_) => {}
            Err(e) => {
                log(&format!("read_loop exit: {e}"));
                return;
            }
        }
    }
}

/// v40: MCS labels for display. The core Mcs enum only covers 0..5; index 6 is the repeat step,
/// QPSK 1/2 x2 (below MCS0). Short for the header, long for the detail panel.
fn mcs_disp_short(idx: usize) -> String {
    match idx {
        6 => "QPSK×2".into(),
        i => Mcs::from_index(i)
            .map(|m| m.index().to_string())
            .unwrap_or_else(|| "-".into()),
    }
}
fn mcs_disp_label(idx: usize) -> String {
    match idx {
        6 => "QPSK×2 repeat".into(),
        i => Mcs::from_index(i)
            .map(|m| m.label().to_string())
            .unwrap_or_else(|| "—".into()),
    }
}

/// SNR too low for the fabric grid (MCS <= 2, 20-38 symbol frames): tell the daemon to turn eqframe
/// off so full IQ flows again; the operator turns it back on from the console once the link
/// improves.
fn escape_eqframe(shared: Arc<Shared>) {
    std::thread::spawn(move || {
        let host = shared
            .channel_addr
            .lock()
            .unwrap()
            .split(':')
            .next()
            .unwrap_or_default()
            .to_string();
        let addr = format!("{host}:7202");
        log(&format!(
            "FD: frame does not fit the continuous grid (low MCS) -> {addr} eqframe 0"
        ));
        if let Ok(mut s) = std::net::TcpStream::connect_timeout(
            &addr.parse().unwrap(),
            Duration::from_secs(2),
        ) {
            use std::io::Write as _;
            let _ = s.write_all(b"eqframe 0
");
        }
    });
}

/// Quality of one EqFrame: (even/odd dB of the Y preamble, EVM of SIG after the PC divides Y/H,
/// symbol count). SIG mixes QPSK data slots with BPSK pilots, so the EVM is measured against the
/// 8-point set {(±1±j)/√2} ∪ {±1, ±j}.
fn eqf_quality(h: &[nyx_proto::Iq16], y: &[nyx_proto::Iq16]) -> (f32, f32, usize) {
    let ns = y.len() / 600;
    let even_k = |u: usize| if u < 300 { u % 2 == 0 } else { u % 2 == 1 };
    let mut eo = (0.0f64, 0.0f64);
    for u in 0..600.min(y.len()) {
        let (i, q) = y[u];
        let e = f64::from(i) * f64::from(i) + f64::from(q) * f64::from(q);
        if even_k(u) {
            eo.0 += e;
        } else {
            eo.1 += e;
        }
    }
    let eo_db = (10.0 * (eo.0.max(1e-9) / eo.1.max(1e-9)).log10()) as f32;

    // SIG = symbol 1 on the grid
    let mut evm = 0.0f32;
    if ns >= 2 && h.len() == 600 {
        let mut zs: Vec<C32> = Vec::with_capacity(600);
        for u in 0..600 {
            let hh = C32::new(f32::from(h[u].0), f32::from(h[u].1));
            let p2 = hh.norm_sqr();
            if p2 < 1.0 {
                continue;
            }
            let yy = C32::new(f32::from(y[600 + u].0), f32::from(y[600 + u].1));
            zs.push(yy * hh.conj() / p2);
        }
        if !zs.is_empty() {
            let a = zs.iter().map(|z| z.norm()).sum::<f32>() / zs.len() as f32;
            let s2 = std::f32::consts::FRAC_1_SQRT_2;
            let refs = [
                C32::new(s2, s2),
                C32::new(s2, -s2),
                C32::new(-s2, s2),
                C32::new(-s2, -s2),
                C32::new(1.0, 0.0),
                C32::new(-1.0, 0.0),
                C32::new(0.0, 1.0),
                C32::new(0.0, -1.0),
            ];
            let mut err2 = 0.0f32;
            for z in &zs {
                let zn = z / a.max(1e-9);
                err2 += refs
                    .iter()
                    .map(|r| (zn - r).norm_sqr())
                    .fold(f32::INFINITY, f32::min);
            }
            evm = (err2 / zs.len() as f32).sqrt();
        }
    }
    (eo_db, evm, ns)
}

/// ADC samples -> float -> demod queue (shared by the TCP and UDP paths). The absolute scale is
/// arbitrary (the AGC gain is unknown to the CPU, as in real hardware); the whole receive chain is
/// scale-invariant.
fn push_capture(
    shared: &Shared,
    iq_tx: &SyncSender<CapItem>,
    seq: u64,
    mcs: u8,
    rv: u8,
    raw: &[nyx_proto::Iq16],
) {
    // mcs=0xFE: the daemon says "the fabric skipped this capture, there is no EqFrame": demodulate
    // from IQ AT ONCE, bypassing the stash (no 150 ms sweeper delay).
    let immediate = mcs == 0xFE;
    let mcs = if immediate { 0xFF } else { mcs };
    // EqFrames are flowing (the fabric demod is covering) -> drop the duplicate IQ of the same
    // capture; if the EqFrame stream dies for over 1 s, IQ takes over by itself.
    if !immediate
        && shared
        .fd_last
        .lock()
        .unwrap()
        .is_some_and(|t| t.elapsed() < Duration::from_secs(1))
    {
        // FD is covering: keep the IQ of a few captures as the fallback for frames that do not fit
        // the grid (instead of demodulating every capture twice).
        let samples = nyx_proto::dequantize(raw, 1.0 / 32768.0);
        let mut stash = shared.iq_stash.lock().unwrap();
        stash.push_back((seq, Instant::now(), samples));
        while stash.len() > 8 {
            stash.pop_front();
        }
        return;
    }
    let mut samples = nyx_proto::dequantize(raw, 1.0 / 32768.0);
    // I/Q format probe: real front-ends often swap I/Q or flip a sign at
    // the axi_ad9361<->fabric boundary. S&C sync is blind to this
    // (autocorrelation), but SIG/data demod needs the right spectral
    // orientation. Cycle modes via `set iqmode N`.
    match shared.iq_mode.load(Ordering::Relaxed) {
        0 => {}                                          // as-is
        1 => samples.iter_mut().for_each(|c| c.im = -c.im), // conjugate
        2 => samples.iter_mut().for_each(|c| *c = C32::new(c.im, c.re)), // swap I/Q
        3 => samples.iter_mut().for_each(|c| *c = C32::new(c.im, -c.re)), // swap + conj
        _ => samples.iter_mut().for_each(|c| c.re = -c.re), // negate I
    }
    if iq_tx.try_send((seq, mcs, rv, Instant::now(), RxCapture::Iq(samples))).is_err() {
        // Demod backlog full: drop and count.
        shared.dropped.fetch_add(1, Ordering::Relaxed);
    }
}

/// UDP mode: register with the daemon (a SUB packet every second; connectionless, so however often
/// the daemon restarts we reattach with no ghost socket), receive captures as chunk sequences and
/// reassemble. A missing chunk drops that capture (like a loss on air; ARQ/IDR take care of it).
/// Feedback/Nack go back on the same socket.
fn udp_loop(
    shared: &Shared,
    sock: &std::net::UdpSocket,
    daemon: std::net::SocketAddr,
    iq_tx: &SyncSender<CapItem>,
) {
    let mut buf = [0u8; 2048];
    let mut last_sub = Instant::now();
    let _ = sock.send_to(&[nyx_proto::UDP_SUB], daemon);
    // Capture being assembled: (cap_seq, mcs, rv, sample buffer, chunks received, total)
    let mut cur: Option<(u64, u8, u8, Vec<nyx_proto::Iq16>, usize, usize)> = None;
    let mut last_rx = Instant::now();
    loop {
        if shared.stop.load(Ordering::Relaxed)
            || !shared.udp_mode.load(Ordering::Relaxed)
        {
            return; // stop, or the GUI switched back to TCP
        }
        if last_sub.elapsed() > Duration::from_secs(1) {
            let _ = sock.send_to(&[nyx_proto::UDP_SUB], daemon);
            last_sub = Instant::now();
            shared
                .connected
                .store(last_rx.elapsed() < Duration::from_secs(3), Ordering::Relaxed);
        }
        let n = match sock.recv_from(&mut buf) {
            Ok((n, _)) => n,
            Err(_) => continue, // timeout: back round the loop to keep the SUB cadence
        };
        let Some((hdr, samples)) = nyx_proto::parse_chunk(&buf[..n]) else {
            continue;
        };
        last_rx = Instant::now();
        shared.connected.store(true, Ordering::Relaxed);
        // Chunk 0 opens a new capture; a half-built capture being replaced means a chunk was lost.
        if hdr.chunk == 0 {
            cur = Some((
                hdr.cap_seq,
                hdr.mcs,
                hdr.rv,
                Vec::with_capacity(hdr.nchunks as usize * nyx_proto::CHUNK_SAMPLES),
                0,
                hdr.nchunks as usize,
            ));
        }
        let Some((seq, mcs, rv, ref mut acc, ref mut got, total)) = cur else {
            continue;
        };
        if hdr.cap_seq != seq || hdr.chunk as usize != *got {
            // Out of order / missing chunk: this capture is broken, wait for the next chunk 0.
            cur = None;
            continue;
        }
        acc.extend_from_slice(&samples);
        *got += 1;
        if *got == total {
            push_capture(shared, iq_tx, seq, mcs, rv, acc);
            cur = None;
        }
    }
}

fn send_msg(shared: &Shared, msg: &Msg) {
    // v21.1: do NOT touch the socket here. demod_loop once blocked in a TCP write_msg (buffer full
    // / GUI holding the writer lock) and stalled the WHOLE pipeline for ~20 s (24/7 bench). Queue
    // only; the net-writer is the only place that writes. When the queue is full, drop: Feedback
    // and Nack both repeat, missing one tick is harmless.
    let _ = shared.net_tx.try_send(msg.clone());
}

/// v21.1: the single network writer thread. Everything that can block (TCP full, dead connection)
/// is contained here and never spreads to demod/GUI.
fn spawn_net_writer(shared: Arc<Shared>, rx: Receiver<Msg>) {
    std::thread::Builder::new()
        .name("net-writer".into())
        .spawn(move || loop {
            if shared.stop.load(Ordering::Relaxed) {
                return;
            }
            let Ok(msg) = rx.recv_timeout(Duration::from_millis(200)) else {
                continue; // timeout: back round to check stop
            };
            // UDP mode: [UDP_MSG][Msg frame] fired straight out, stateless.
            if let Some((sock, daemon)) = &*shared.udp_out.lock().unwrap() {
                let enc = nyx_proto::encode_msg(&msg);
                let mut pkt = Vec::with_capacity(1 + enc.len() - 4);
                pkt.push(nyx_proto::UDP_MSG);
                pkt.extend_from_slice(&enc[4..]); // drop the length (the datagram size carries it)
                let _ = sock.send_to(&pkt, *daemon);
                continue;
            }
            let mut guard = shared.writer.lock().unwrap();
            if let Some(stream) = guard.as_mut()
                && write_msg(stream, &msg).is_err()
            {
                *guard = None;
            }
        })
        .expect("spawn net-writer");
}

// -------------------------------------------------------------- demod --

/// Data source of one capture: raw IQ (the old path) or the 600-subcarrier spectral grid from the
/// fabric (EqFrame: each element is one symbol on the grid at us + s*1096).
enum RxCapture {
    Iq(Vec<C32>),
    Grid(Vec<Vec<C32>>),
    /// 6-bit LLRs demapped in the fabric (v2): 5 LLRs per word, TX order. SIG was decoded by the
    /// ARM; mcs/rv/seq_lsb are the CRC-checked result.
    Llr { mcs: u8, rv: u8, seq_lsb: u8, words: Vec<u32> },
    /// v4.2: payload fully DECODED in the fabric (2x1008 B, CRC16 checked in the daemon). llr_sum =
    /// Σ|llr| of the frame, for the SNR heuristic.
    Dec { mcs: u8, seq_lsb: u8, iters: u8, llr_sum: u32, payload: Vec<u8> },
}

fn spawn_demod(
    shared: Arc<Shared>,
    iq_rx: Receiver<CapItem>,
    sweep_tx: SyncSender<CapItem>,
) {
    // Pre-demod pool: N workers carry the pure DSP part (sync + SIG + LLR) per capture, the most
    // expensive step, which once pinned one core at 100 % on a 1 Mbps webcam (backlog_drop ~1/s).
    // demod_loop keeps ALL the bookkeeping (link seq, HARQ/LDPC, NACK, reorder) and consumes the
    // results IN idx ORDER; on a cache miss it computes inline. The pool is purely an accelerator
    // and changes no semantics.
    let (pre_tx, pre_rx) = sync_channel::<(u64, PreDemod)>(32);
    let work_rx = Arc::new(Mutex::new(iq_rx));
    let idx_gen = Arc::new(AtomicU64::new(0));
    let n_workers = std::thread::available_parallelism()
        .map(|n| n.get().saturating_sub(2).clamp(2, 4))
        .unwrap_or(2);
    for w in 0..n_workers {
        let shared = shared.clone();
        let work_rx = work_rx.clone();
        let idx_gen = idx_gen.clone();
        let pre_tx = pre_tx.clone();
        std::thread::Builder::new()
            .name(format!("rx-predemod-{w}"))
            .spawn(move || {
                let mut phy = PcRx::new();
                let mut stale_n = 0u64;
                let mut stale_t = Instant::now();
                loop {
                    if shared.stop.load(Ordering::Relaxed) {
                        return;
                    }
                    // idx is assigned INSIDE the lock -> idx order == the order jobs were taken
                    let got = {
                        let rx = work_rx.lock().unwrap();
                        loop {
                            match rx.recv_timeout(Duration::from_millis(100)) {
                                Ok((s, m, r, t0, c)) => {
                                    // v21.1 drop-to-live: a HEAVY video capture that has gone stale
                                    // (backlog after a GUI/CPU stall) is dropped at once instead of
                                    // spending 100-300 ms demodulating it (24/7 bench: p50 delay
                                    // stuck at 9.6 s for a whole minute). Dec frames are kept:
                                    // cheap (~µs) and they carry the segs/ARQ bookkeeping; reorder
                                    // drops old frames by itself.
                                    let stale = !matches!(
                                        c, RxCapture::Dec { .. })
                                        && t0.elapsed()
                                            > Duration::from_millis(700);
                                    if stale {
                                        shared.dropped
                                            .fetch_add(1, Ordering::Relaxed);
                                        stale_n += 1;
                                        if stale_t.elapsed().as_secs() >= 1 {
                                            log(&format!(
                                                "predemod-{w}: drop {stale_n} \
                                                 stale captures (drop-to-live)"
                                            ));
                                            stale_n = 0;
                                            stale_t = Instant::now();
                                        }
                                        continue; // idx NOT granted yet, order is kept
                                    }
                                    break Ok((
                                        idx_gen.fetch_add(1, Ordering::Relaxed),
                                        s, m, r, c,
                                    ));
                                }
                                Err(e) => break Err(e),
                            }
                        }
                    };
                    let (idx, seq, mcs_byte, rv_hdr, cap) = match got {
                        Ok(v) => v,
                        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => continue,
                        Err(_) => {
                            log(&format!("predemod-{w}: iq channel closed - exiting"));
                            return;
                        }
                    };
                    let pre = predemod_capture(&shared, &mut phy, seq, mcs_byte,
                                               rv_hdr, cap);
                    if pre_tx.send((idx, pre)).is_err() {
                        return;
                    }
                }
            })
            .expect("spawn rx-predemod");
    }
    std::thread::Builder::new()
        .name("rx-demod".into())
        .spawn(move || demod_loop(shared, pre_rx, sweep_tx))
        .expect("spawn rx-demod");
}

/// Pre-demod result of ONE frame on the multi-frame scan grid: exactly what demod_loop will ask for
/// at that win_start.
struct PreFrame {
    win_start: usize,
    sig_res: Option<(SigPayload, bool)>,
    demod: Option<DemodResult>,
    /// v4.2: payload decoded in the fabric; demod_loop skips scatter/LDPC/BICM and goes straight to
    /// parse_block.
    decoded: Option<Vec<u8>>,
    adv: Option<usize>,
    /// (us, metric, cfo_scs) from the very schmidl_cox call used for adv, reused for the DBG frm
    /// line (which used to re-run the full S&C every time).
    dbg_sc: Option<(i64, f32, f32)>,
}

/// A pre-demodulated capture: source resolved (grid vs IQ fallback, twin claimed in the stash) plus
/// the frame sequence in scan order.
struct PreDemod {
    seq: u64,
    mcs_byte: u8,
    rv_hdr: u8,
    samples: Vec<C32>,
    grid: Vec<Vec<C32>>,
    is_grid: bool,
    /// Some(true) = fit failure with no IQ (count an escape), Some(false) = reset the fit-fail
    /// streak, None = pure IQ (leave the counter alone).
    fit_state: Option<bool>,
    frames: Vec<PreFrame>,
}

/// The pure DSP part of demod_loop, run in a worker. The win_start walk depends ONLY on the capture
/// content (SIG -> mcs -> adv), so it can be reproduced exactly; every bookkeeping decision
/// (dup-skip, HARQ, NACK) leaves the walk unchanged and stays in demod_loop.
fn predemod_capture(
    shared: &Arc<Shared>,
    phy: &mut PcRx,
    seq: u64,
    mcs_byte: u8,
    rv_hdr: u8,
    cap: RxCapture,
) -> PreDemod {
    let (samples, grid, is_grid) = match cap {
        RxCapture::Iq(s) => (s, Vec::new(), false),
        RxCapture::Grid(g) => (Vec::new(), g, true),
        RxCapture::Llr { mcs, rv, seq_lsb, words } => {
            return predemod_llr(shared, seq, rv, mcs, seq_lsb, &words);
        }
        RxCapture::Dec { mcs, seq_lsb, iters, llr_sum, payload } => {
            return predemod_dec(shared, seq, mcs, seq_lsb, iters, llr_sum,
                                payload);
        }
    };
    if !pc::AVAILABLE {
        pc::warn_no_pc_demod();
        return PreDemod {
            seq, mcs_byte, rv_hdr, samples: Vec::new(), grid: Vec::new(),
            is_grid, fit_state: None, frames: Vec::new(),
        };
    }
    // Source resolution (the old demod_loop block verbatim, minus the escape: demod_loop counts it
    // through fit_state).
    let (samples, grid, is_grid, fit_state) = if is_grid && grid.len() >= 2 {
        let (sig, sig_ok) = phy.sig_core(&grid[0], &grid[1]);
        let fits = sig_ok
            && Mcs::from_index(sig.mcs_index as usize)
                .is_some_and(|m| 2 + pc::data_syms(m) <= grid.len());
        let iq = {
            let mut stash = shared.iq_stash.lock().unwrap();
            let pos = stash.iter().position(|(s2, _, _)| *s2 == seq);
            pos.map(|i| stash.remove(i).unwrap().2)
        };
        if fits {
            (samples, grid, true, Some(false))
        } else {
            match iq {
                Some(v) => (v, Vec::new(), false, Some(false)),
                None => (samples, grid, true, Some(true)),
            }
        }
    } else if is_grid {
        (samples, grid, true, Some(false))
    } else {
        (samples, grid, false, None)
    };

    let mut frames: Vec<PreFrame> = Vec::new();
    let mut win_start = 0usize;
    for _sub in 0..4 {
        let window: &[C32] = if is_grid { &[] } else { &samples[win_start..] };
        if is_grid {
            if win_start + 2 > grid.len() {
                break;
            }
        } else if window.len() < 5 * 1096 {
            break;
        }
        let search_limit = if win_start == 0 { 6000 } else { 24000 };
        phy.set_search_limit(Some(search_limit));
        let sig_res = if is_grid {
            Some(phy.sig_core(&grid[win_start], &grid[win_start + 1]))
        } else {
            phy.demodulate_sig(window)
        };
        let control = match sig_res {
            Some((sig, true)) => Mcs::from_index(sig.mcs_index as usize)
                .map(|m| (m, sig.rv, sig.seq_lsb)),
            _ => None,
        };
        if win_start > 0 && control.is_none() {
            frames.push(PreFrame {
                win_start, sig_res, demod: None, adv: None, dbg_sc: None,
                decoded: None,
            });
            break;
        }
        let mcs = match control {
            Some((m, _, _)) => m,
            None => Mcs::Qpsk12,
        };
        let (adv, dbg_sc) = if is_grid {
            (
                control.map(|(m, _, _)| win_start + 2 + pc::data_syms(m)),
                None,
            )
        } else {
            let sc = pc::schmidl_cox(
                &window[..window.len().min(search_limit)],
            );
            let dbg = sc
                .as_ref()
                .map(|r| (r.useful_start as i64, r.metric, r.cfo_scs));
            let adv = match (control, sc) {
                (Some((m, _, _)), Some(r)) => Some(
                    win_start + r.useful_start
                        + pc::frame_len_samples(m).saturating_sub(600),
                ),
                _ => None,
            };
            (adv, dbg)
        };
        // Tail frame cut off by the capture edge: demod_loop breaks BEFORE demodulating.
        if win_start > 0 {
            let whole = match adv {
                Some(a) if is_grid => a <= grid.len(),
                Some(a) => a + 600 <= samples.len(),
                None => false,
            };
            if !whole {
                frames.push(PreFrame {
                    win_start, sig_res, demod: None, adv, dbg_sc,
                    decoded: None,
                });
                break;
            }
        }
        let demod = if control.is_none() {
            None
        } else if is_grid {
            let need = 2 + pc::data_syms(mcs);
            if win_start + need <= grid.len() {
                Some(phy.llrs_from_grid(
                    &grid[win_start..win_start + need], mcs, 1.0, 0.0,
                ))
            } else {
                None
            }
        } else {
            Some(phy.demodulate_to_llrs(window, mcs))
        };
        frames.push(PreFrame {
            win_start, sig_res, demod, adv, dbg_sc, decoded: None,
        });
        let fits = match adv {
            Some(a) if is_grid => a + 2 <= grid.len(),
            Some(a) => a + 5 * 1096 <= samples.len(),
            None => false,
        };
        match adv {
            Some(a) if fits => win_start = a,
            _ => break,
        }
    }
    PreDemod { seq, mcs_byte, rv_hdr, samples, grid, is_grid, fit_state, frames }
}

/// Capture with LLRs demapped in the fabric (v2): unpack 6-bit -> descramble -> deinterleave back
/// to coded order, packed as a one-frame PreDemod for demod_loop to consume with its bookkeeping
/// INTACT (link seq/HARQ/NACK/BLER). eq_syms is empty: BICM-ID is skipped for these frames
/// (separate guard).
fn predemod_llr(
    shared: &Arc<Shared>,
    seq: u64,
    rv: u8,
    mcs_idx: u8,
    seq_lsb: u8,
    words: &[u32],
) -> PreDemod {
    if !pc::AVAILABLE {
        pc::warn_no_pc_demod();
        return PreDemod {
            seq, mcs_byte: 0xFF, rv_hdr: rv,
            samples: Vec::new(), grid: Vec::new(), is_grid: true,
            fit_state: None, frames: Vec::new(),
        };
    }
    // Claim the IQ twin in the stash as the grid path does: the fabric handled this capture.
    {
        let mut stash = shared.iq_stash.lock().unwrap();
        if let Some(i) = stash.iter().position(|(s2, _, _)| *s2 == seq) {
            stash.remove(i);
        }
    }
    let fail = |frames: Vec<PreFrame>| PreDemod {
        seq, mcs_byte: 0xFF, rv_hdr: rv,
        samples: Vec::new(), grid: Vec::new(), is_grid: true,
        fit_state: Some(false), frames,
    };
    let Some(mcs) = Mcs::from_index(mcs_idx as usize) else {
        return fail(Vec::new());
    };
    let t = mcs.frame_bit_capacity();
    if words.len() * 5 != t {
        log(&format!(
            "LLRF seq={seq}: {} words != t={t}/5 (mcs={mcs_idx}) - dropped",
            words.len()
        ));
        return fail(Vec::new());
    }
    // 5 6-bit LLRs per word, the first in bits[5:0]; sign-extend 6 bits.
    let mut llrs_tx: Vec<f32> = Vec::with_capacity(t);
    for &w in words {
        for j in 0..5 {
            let v = ((w >> (6 * j)) & 0x3F) as u8;
            let v = ((v << 2) as i8) >> 2;
            llrs_tx.push(f32::from(v));
        }
    }
    // SNR heuristic from LLR magnitude (telemetry/feedback trend only; BLER is the real control
    // signal): anchor mean|llr| = 8 ≈ the decode threshold of each MCS, ±6 dB per doubling/halving;
    // LLRs saturate at ±31, so it compresses at high SNR, which is acceptable.
    let mean_abs = llrs_tx.iter().map(|l| l.abs()).sum::<f32>() / t as f32;
    let base = match mcs.bits_per_sym() {
        2 => 8.0f32,
        4 => 14.0,
        _ => 20.0,
    };
    let snr_db = base + 6.0 * (mean_abs.max(0.5) / 8.0).log2();

    pc::descramble_llrs(&mut llrs_tx);
    let coded_bits = pc::coded_bits(mcs);
    let stride = pc::interleave_stride(t);
    let mut llrs_coded = vec![0.0f32; coded_bits];
    let mut src = 0usize;
    for &l in llrs_tx.iter() {
        if src < coded_bits {
            llrs_coded[src] = l;
        }
        src = (src + stride) % t;
    }
    let demod = DemodResult {
        synced: true,
        sync_metric: 1.0,
        cfo_hz: 0.0,
        snr_db,
        llrs: llrs_coded,
        eq_syms: Vec::new(),
        constellation: Vec::new(),
        chan_mag: Vec::new(),
    };
    let sig = SigPayload { mcs_index: mcs_idx, rv, seq_lsb, pair: false };
    fail(vec![PreFrame {
        win_start: 0,
        sig_res: Some((sig, true)),
        demod: Some(demod),
        adv: None,
        dbg_sc: None,
        decoded: None,
    }])
}

/// v4.2: capture fully DECODED in the fabric, no DSP left for the PC. PreFrame carries the payload;
/// demod_loop skips scatter/LDPC/BICM. SNR heuristic from llr_sum (mean|llr| as in predemod_llr,
/// same anchor).
fn predemod_dec(
    shared: &Arc<Shared>,
    seq: u64,
    mcs_idx: u8,
    seq_lsb: u8,
    iters: u8,
    llr_sum: u32,
    payload: Vec<u8>,
) -> PreDemod {
    {
        let mut stash = shared.iq_stash.lock().unwrap();
        if let Some(i) = stash.iter().position(|(s2, _, _)| *s2 == seq) {
            stash.remove(i);
        }
    }
    let _ = iters;
    let fail = |frames: Vec<PreFrame>| PreDemod {
        seq, mcs_byte: 0xFF, rv_hdr: 0,
        samples: Vec::new(), grid: Vec::new(), is_grid: true,
        fit_state: Some(false), frames,
    };
    let Some(mcs) = Mcs::from_index(mcs_idx as usize) else {
        return fail(Vec::new());
    };
    if payload.len() != nyx_link::BLOCK_BYTES {
        log(&format!(
            "DECF seq={seq}: payload {} != {} - dropped",
            payload.len(),
            nyx_link::BLOCK_BYTES
        ));
        return fail(Vec::new());
    }
    let mean_abs = llr_sum as f32 / mcs.frame_bit_capacity() as f32;
    let base = match mcs.bits_per_sym() {
        2 => 8.0f32,
        4 => 14.0,
        _ => 20.0,
    };
    let snr_db = base + 6.0 * (mean_abs.max(0.5) / 8.0).log2();
    let demod = DemodResult {
        synced: true,
        sync_metric: 1.0,
        cfo_hz: 0.0,
        snr_db,
        llrs: Vec::new(),
        eq_syms: Vec::new(),
        constellation: Vec::new(),
        chan_mag: Vec::new(),
    };
    let sig = SigPayload { mcs_index: mcs_idx, rv: 0, seq_lsb, pair: false };
    fail(vec![PreFrame {
        win_start: 0,
        sig_res: Some((sig, true)),
        demod: Some(demod),
        adv: None,
        dbg_sc: None,
        decoded: Some(payload),
    }])
}

/// Per-seq accumulated circular-buffer LLRs (5G-style IR-HARQ soft
/// buffer), bounded LRU. Stores raw channel information only — BICM-ID
/// extrinsics are never persisted, so combining stays uncorrelated.
struct HarqCache {
    map: std::collections::HashMap<u64, Vec<f32>>,
    order: VecDeque<u64>,
}

impl HarqCache {
    const CAP: usize = 64;

    fn new() -> Self {
        HarqCache { map: Default::default(), order: Default::default() }
    }

    fn insert(&mut self, seq: u64, buffers: Vec<f32>) {
        if self.map.insert(seq, buffers).is_none() {
            self.order.push_back(seq);
        }
        while self.order.len() > Self::CAP {
            if let Some(old) = self.order.pop_front() {
                self.map.remove(&old);
            }
        }
    }

    fn remove(&mut self, seq: u64) {
        if self.map.remove(&seq).is_some() {
            self.order.retain(|&s| s != seq);
        }
    }
}

fn demod_loop(
    shared: Arc<Shared>,
    pre_rx: Receiver<(u64, PreDemod)>,
    sweep_tx: SyncSender<CapItem>,
) {
    let mut phy = PcRx::new();
    // Restore capture order from the pool (workers finish out of order).
    let mut pending_pre: std::collections::BTreeMap<u64, PreDemod> =
        std::collections::BTreeMap::new();
    let mut next_pre_idx = 0u64;
    let mut dumped_bad = false;
    // v21.1 diagnostic: pre_rx starved for long = predemod workers stalled or dead
    let mut pre_starve = Instant::now();
    // Captures from the FPGA are trigger-aligned: the intended frame's
    // preamble sits near the start (~PRE_LEN), and the window can also hold
    // a later frame's (truncated) preamble. Only search the head so sync
    // locks the intended, complete frame.
    phy.set_search_limit(Some(6000));
    let mut reasm = Reassembler::new();
    // v36 simulcast: a SEPARATE reassembly funnel for the base layer. The two streams' frame_ids
    // are independent sequences; one funnel would pair blocks across streams.
    let mut reasm_base = Reassembler::new();
    let mut harq = HarqCache::new();
    let mut harq_combines = 0u64;
    let mut harq_recovered = 0u64;
    let mut bicm_runs = 0u64;
    let mut bicm_rescues = 0u64;
    let mut sig_failures = 0u64;
    let mut bler_window: VecDeque<bool> = VecDeque::new();
    let mut snr_smooth = 10.0f32;
    let mut segs_ok = 0u64;
    // v32 minstrel: count delivered blocks PER MCS (wrapping u16); the TX divides by its own
    // sent-per-MCS for a per-rate probability (minstrel-style statistics).
    let mut ok_mcs = [0u16; 6];
    let mut ok_base = 0u16; // v37: BASE-layer blocks delivered ok (separate stream)
    let mut segs_lost = 0u64;
    let mut sync_failures = 0u64;
    let mut frames_rx = 0u64;
    let mut frames_lost = 0u64;
    // H.264 decoder outcomes: pictures decoded vs frames the decoder had
    // to skip (mid-GOP corruption, waiting for the next IDR).
    let mut pics_ok = 0u64;
    let mut pics_fail = 0u64;
    // Reorder stage between the reassembler and the video decoder: completed
    // frames wait here until they can be delivered in id order.
    let mut pending: std::collections::BTreeMap<u32, (SourceType, Vec<u8>, Instant)> =
        std::collections::BTreeMap::new();
    let mut next_deliver_id: Option<u32> = None;
    // v37.1: most recent delivered base id. The base layer does NOT go through the reorder funnel
    // (base ids are their own sequence; mixing them into the main cursor produced "wild jumps" ->
    // pending.clear() every frame -> reorder/HOLD paralysed since v36).
    let mut last_base_id: Option<u32> = None;
    let mut fps_events: VecDeque<Instant> = VecDeque::new();
    let mut disp_events: VecDeque<Instant> = VecDeque::new(); // v36.1
    let mut layer_main = true;
    let mut last_disp: Option<Instant> = None; // v14: for the EMA of the playout period
    let mut goodput: VecDeque<(Instant, usize)> = VecDeque::new();
    let mut last_feedback = Instant::now();
    let mut need_idr = false;
    let mut vdec = VideoDecoder::new();
    // v36 simulcast: a separate decoder for the BASE layer (H264Base). Always decode to keep its
    // state fresh; only SHOW it when the main layer is late (layer choice at the receiver).
    let mut vdec_base = VideoDecoder::new();
    let mut last_main = std::time::Instant::now();
    // v40.24: after a TX restart the H264 decoder (OpenH264) can reject EVERY frame, IDRs included
    // (measured: rx_fps 25, pics_ok flat, pics_fail +13/s, TX sent 313 IDRs, disp 0 fps for 150 s;
    // only an app restart cleared it). Recreate the decoder when nothing decodes for > 2 s although
    // frames keep arriving, and as soon as the TX is seen renumbering (wild jump).
    let mut vdec_reset_at = std::time::Instant::now();
    let mut vdec_resets = 0u32;
    // pics_fail at the previous reset: reset again only after >= 40 more BROKEN frames (frames
    // really arriving and the decoder really refusing them), not merely 2 s of blank (at MCS0 a
    // hole -> waiting 700 ms for an IDR is normal). The first version (2 s / 2 s cooldown) fired
    // every 3-6 s (#22), each time killing the decoder mid-GOP -> request an IDR -> blank again ->
    // self-feeding; minstrel stuck at MCS0.
    let mut vdec_fail_at_reset = 0u64;
    // v40.25 diagnostic: the first 40 frames into the main decoder after each reset
    let mut dbg_dec_n: u32 = 0;
    let mut status = StatusLogger::new(1.0);
    let mut dbg_count = 0u32;
    // Highest link-level (TX block) sequence seen, reconstructed from the
    // SIG's 8-bit LSB in hardware mode — see below.
    let mut last_link_seq: Option<u64> = None;
    // NACKed sequences still unseen: (first-nack time, attempts). If the
    // retransmission itself is lost on air nobody would ever re-ask — this
    // re-NACKs up to twice more before giving up.
    let mut outstanding_nacks: std::collections::HashMap<u64, (Instant, u8)> =
        std::collections::HashMap::new();
    // Sequences already delivered: a duplicate retransmission (our re-NACK
    // racing the in-flight one) must be dropped SILENTLY. Its rv1+ payload
    // is parity-only and not self-decodable, so decoding it "fresh" fails
    // by design — and counting that as a block error re-NACKs it, spawning
    // an endless fail->nack->retx cycle that melts the link (observed:
    // 10% bler at 33 dB SNR with the channel provably clean).
    let mut delivered_seqs: std::collections::HashSet<u64> =
        std::collections::HashSet::new();
    let mut delivered_order: VecDeque<u64> = VecDeque::new();

    // Automatic escape: a frame that does not fit the 16-symbol grid and has no IQ (selective mode
    // + MCS <= 2), repeatedly -> turn eqframe off at the daemon so full IQ flows; without it OLLA
    // spirals down to MCS0 for good at low SNR.
    let mut fitfail_run = 0u32;
    let mut last_escape: Option<Instant> = None;
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return;
        }
        phy.set_bicm_id(shared.bicm_enabled.load(Ordering::Relaxed));
        phy.set_ldpc_iters(shared.ldpc_iters.load(Ordering::Relaxed).clamp(5, 100));
        // Sweeper: IQ sitting in the stash for over 150 ms means the fabric SKIPPED that capture
        // (engine busy) and no EqFrame will ever come: push it into the work channel for the demod
        // pool (that item's worker will claim the stash by seq, but the seq is already popped, so
        // it is harmless).
        {
            let mut stash = shared.iq_stash.lock().unwrap();
            while stash
                .front()
                .is_some_and(|(_, t, _)| t.elapsed() > Duration::from_millis(150))
            {
                let (s2, t2, v) = stash.pop_front().unwrap();
                match sweep_tx.try_send((s2, 0xFF, 0, Instant::now(), RxCapture::Iq(v))) {
                    Ok(()) => {}
                    Err(std::sync::mpsc::TrySendError::Full((s3, _, _, _, c3))) => {
                        if let RxCapture::Iq(v3) = c3 {
                            stash.push_front((s3, t2, v3)); // channel full, next time
                        }
                        break;
                    }
                    Err(_) => break,
                }
            }
        }
        // Pre-demod results, consumed IN idx ORDER. Safety valve: if the next idx goes missing
        // (worker died?) while the queue swells, jump to the smallest idx present instead of
        // standing still for ever.
        let pre: PreDemod = if let Some(p) = pending_pre.remove(&next_pre_idx) {
            next_pre_idx += 1;
            pre_starve = Instant::now();
            p
        } else {
            match pre_rx.recv_timeout(Duration::from_millis(50)) {
                Ok((idx, p)) if idx == next_pre_idx => {
                    next_pre_idx += 1;
                    pre_starve = Instant::now();
                    p
                }
                Ok((idx, p)) => {
                    pending_pre.insert(idx, p);
                    if pending_pre.len() >= 8 {
                        let lowest = *pending_pre.keys().next().unwrap();
                        log(&format!(
                            "demod: pre idx {next_pre_idx} vanished, jumping to {lowest}"
                        ));
                        next_pre_idx = lowest;
                    }
                    continue;
                }
                Err(_) => {
                    if pre_starve.elapsed().as_secs() >= 5 {
                        pre_starve = Instant::now();
                        log(&format!(
                            "demod: pre_rx starved for 5s (next_idx={next_pre_idx}, \
                             pending={}) - predemod worker stalled?",
                            pending_pre.len()
                        ));
                    }
                    maybe_feedback(&shared, &mut last_feedback, snr_smooth,
                                   &bler_window, segs_ok, segs_lost, need_idr,
                                   ok_mcs, ok_base);
                    continue;
                }
            }
        };
        let PreDemod {
            seq,
            mcs_byte,
            rv_hdr,
            samples,
            grid,
            is_grid,
            fit_state,
            frames: mut pre_frames,
        } = pre;
        // The source was resolved by the worker (grid vs IQ fallback, twin claimed in the stash);
        // all that is left here is counting the fit-fail streak for the auto-escape.
        match fit_state {
            Some(true) => {
                fitfail_run += 1;
                let due = last_escape
                    .is_none_or(|t| t.elapsed() > Duration::from_secs(30));
                if fitfail_run >= 10 && due {
                    last_escape = Some(Instant::now());
                    fitfail_run = 0;
                    escape_eqframe(shared.clone());
                }
            }
            Some(false) => fitfail_run = 0,
            None => {}
        }

        // ---- multi-frame demodulation ---------------------------------
        // One 46080-sample capture window holds up to ~4 short (high-MCS)
        // frames, but the FPGA can only trigger once per window: any frame
        // transmitted inside another frame's capture shadow used to be
        // structurally lost. Instead of forcing every block into its own
        // capture with air gaps, demodulate the head frame and then keep
        // scanning the REST of the window for further preambles — a
        // multi-block video frame sent back-to-back then arrives whole in
        // a single capture.
        let mut win_start = 0usize;
        let mut completed_frames: Vec<(u32, SourceType, Vec<u8>)> = Vec::new();
        let mut last_demod = DemodResult::failed();
        let mut last_mcs = Mcs::Qpsk12;
        let mut pre_ber = 0.0f32;
        for _sub in 0..4 {
        let window: &[C32] = if is_grid { &[] } else { &samples[win_start..] };
        if is_grid {
            // LLR capture: the grid is empty but the frame is already demapped in pre_frames; let
            // it through so the full bookkeeping runs.
            let has_pre = pre_frames
                .iter()
                .any(|f| f.win_start == win_start && f.demod.is_some());
            if win_start + 2 > grid.len() && !has_pre {
                break;
            }
        } else if window.len() < 5 * 1096 {
            break;
        }
        // Head frame: trigger-aligned near the start. Follow-up frames:
        // nominally right after the previous frame, but the daemon's zero
        // streamer may interleave up to one ~16k-sample silence chunk
        // between burst blocks — search wider there.
        let search_limit = if win_start == 0 { 6000 } else { 24000 };
        phy.set_search_limit(Some(search_limit));
        // Radio parameters come from the SIG field (Polar + CRC-aided SCL), not from the transport
        // header; the header is only cross-checked. Look the worker result up by win_start; on a
        // miss (the walk diverged, not expected) compute inline as before.
        let pre_i = pre_frames.iter().position(|f| f.win_start == win_start);
        let sig_res = match pre_i {
            Some(i) => pre_frames[i].sig_res,
            None => {
                if is_grid {
                    Some(phy.sig_core(&grid[win_start], &grid[win_start + 1]))
                } else {
                    phy.demodulate_sig(window)
                }
            }
        };
        let control = match sig_res {
            Some((sig, true)) => match Mcs::from_index(sig.mcs_index as usize) {
                Some(m) => {
                    // Header mcs 0xFF = sentinel from the radio node ("only
                    // the SIG knows") — skip the cross-check then. It only
                    // describes the HEAD frame of a capture.
                    if win_start == 0
                        && mcs_byte != 0xFF
                        && (sig.mcs_index != mcs_byte
                            || sig.rv != rv_hdr
                            || sig.seq_lsb != (seq & 0xFF) as u8)
                    {
                        log(&format!(
                            "SIG/header mismatch: sig mcs={} rv={} seq_lsb={:02x} \
                             vs header mcs={mcs_byte} rv={rv_hdr} seq={seq}",
                            sig.mcs_index, sig.rv, sig.seq_lsb
                        ));
                    }
                    Some((m, sig.rv, sig.seq_lsb))
                }
                None => None,
            },
            Some((_, false)) => {
                // Follow-up windows usually hold nothing: only the head
                // frame's SIG verdict is link telemetry.
                if win_start == 0 {
                    sig_failures += 1;
                }
                None
            }
            None => None,
        };
        // A follow-up window with no decodable SIG is simply the end of
        // this capture's frames — not an error. Counting it as sync/BLER
        // failure poisoned the feedback (20% phantom BLER downshifted the
        // MCS all the way to MCS0).
        if win_start > 0 && control.is_none() {
            break;
        }
        let (mcs, rv) = match control {
            Some((m, rv, _)) => (m, rv),
            None => (Mcs::Qpsk12, 0),
        };

        // Canonical link-level (TX block) sequence for ARQ and HARQ. Over
        // the hardware daemon the transport header carries the CAPTURE
        // counter (mcs=0xFF sentinel), which shares no numbering with the
        // TX blocks: NACKs keyed on it never matched the TX's retransmit
        // cache (nack counted, retx forever 0) and HARQ soft buffers never
        // combined across retransmissions. Reconstruct the full sequence
        // from the SIG's 8-bit LSB (signed wrap-around delta, so a
        // retransmission of an older block resolves BACKWARD instead of
        // aliasing 250 frames ahead), and NACK the holes — blocks that
        // were never even captured, which the old per-received-frame NACK
        // could not see.
        let link_seq = if mcs_byte != 0xFF {
            Some(seq)
        } else if let Some((_, _, lsb)) = control {
            Some(match last_link_seq {
                None => lsb as u64,
                Some(p) => {
                    let mut d = (lsb as i32 - (p & 0xFF) as i32).rem_euclid(256);
                    if d > 128 {
                        d -= 256;
                    }
                    (p as i64 + d as i64).max(0) as u64
                }
            })
        } else {
            None
        };
        if let (Some(full), Some(p)) = (link_seq, last_link_seq) {
            // Forward gap = frames lost on air (missed captures): request
            // them while the TX cache still holds them. A large jump is a
            // resync (TX restart), not a loss burst — don't NACK-storm it.
            // Cap the outstanding requests: over the air, junk triggers
            // displace real frames constantly, and an unbounded NACK
            // stream turns into a retransmission storm that saturates the
            // transport (measured 73 Mbps of IQ at MCS0) and causes MORE
            // losses. Beyond the cap, let the frames go — the reorder
            // stage skips them and the fast-IDR path repairs the video.
            if full > p + 1 && full - p <= 32 && outstanding_nacks.len() < 8 {
                for missing in p + 1..full {
                    send_msg(&shared, &Msg::Nack { seq: missing });
                    outstanding_nacks.entry(missing).or_insert((Instant::now(), 1));
                }
            }
        }
        if let Some(full) = link_seq {
            // v40.16 measurement: NACK RTT -> the retransmitted fragment arrives. This number sets
            // the "hold frames for retx" limit in take_stale; guessing it was wrong.
            if let Some((t0, _)) = outstanding_nacks.remove(&full) {
                log(&format!("retx-rtt seq={full} ms={}", t0.elapsed().as_millis()));
            }
            if last_link_seq.map_or(true, |p| full > p) {
                last_link_seq = Some(full);
            }
        }
        // NACK each missing sequence exactly ONCE. Re-asking was tried and
        // it congestion-collapsed the link: the capture pipeline tops out
        // near ~27 captures/s, so every extra retransmission displaced a
        // live frame, which created the next gap, which asked for more
        // retransmissions (nack rate x5, frame loss 2%->6%). If the single
        // retransmission is lost too, the reorder stage skips the frame and
        // the fast-IDR path repairs the stream — cheaper than fighting for
        // the same airtime twice.
        outstanding_nacks.retain(|_, (t, _)| t.elapsed() < Duration::from_secs(2));

        // Where the NEXT frame in this capture would start: known once the
        // SIG decodes (frame length follows from the MCS). Back off ~0.5
        // symbol so the next search window still contains the preamble.
        let adv = if let Some(i) = pre_i {
            pre_frames[i].adv
        } else if is_grid {
            // the next frame starts at the symbol right after this one on the grid
            control.map(|(m, _, _)| win_start + 2 + pc::data_syms(m))
        } else {
            match (
                control,
                pc::schmidl_cox(&window[..window.len().min(search_limit)]),
            ) {
                (Some((m, _, _)), Some(r)) => Some(
                    win_start + r.useful_start
                        + pc::frame_len_samples(m).saturating_sub(600),
                ),
                _ => None,
            }
        };
        // "adv still has room for the next frame", in each source's own units
        let adv_fits = |a: usize| {
            if is_grid {
                a + 2 <= grid.len()
            } else {
                a + 5 * 1096 <= samples.len()
            }
        };
        // Duplicate of an already-delivered block (a raced retransmission):
        // acknowledge silently, never decode (see delivered_seqs above).
        let mut skip_dup = false;
        if let Some(full) = link_seq {
            if delivered_seqs.contains(&full) {
                bler_window.push_back(false);
                if bler_window.len() > 30 {
                    bler_window.pop_front();
                }
                skip_dup = true;
            }
        }
        if skip_dup {
            match adv {
                Some(a) if adv_fits(a) => {
                    win_start = a;
                    continue;
                }
                _ => break,
            }
        }
        // A tail frame TRUNCATED by the capture edge can never decode: its
        // full copy arrives as the head of the next capture anyway. Its
        // guaranteed demod failure was being counted as a sync failure AND
        // fired a spurious NACK -> a pointless retransmission per capture.
        // adv + 600 = absolute end of this frame (see adv above).
        if win_start > 0 {
            match adv {
                Some(a) if is_grid && a <= grid.len() => {}
                Some(a) if !is_grid && a + 600 <= samples.len() => {}
                _ => break,
            }
        }
        // From here on, ARQ/HARQ bookkeeping uses the link sequence.
        let seq = link_seq.unwrap_or(seq);

        // Bring-up diagnostic: for the first handful of captures, report
        // what S&C actually finds and how the SIG decode goes.
        if !is_grid && win_start == 0 && dbg_count == 0 {
            // Dump the very first capture (raw i16 I,Q) for offline analysis.
            let mut s = String::with_capacity(samples.len() * 12);
            for c in &samples {
                s.push_str(&format!(
                    "{},{}\n",
                    (c.re * 32768.0).round() as i32,
                    (c.im * 32768.0).round() as i32
                ));
            }
            let _ = std::fs::write("cap0.csv", s);
            log(&format!("DBG wrote cap0.csv ({} samples)", samples.len()));
        }
        // Also dump ONE "bad" capture (full-search sync lands far from the
        // trigger-aligned head ~1310) so its structure can be inspected: is
        // there really a second preamble mid-capture (double-trigger / DAC
        // replay) or is the head frame corrupt?
        if !is_grid && win_start == 0 && !dumped_bad {
            if let Some(r) = pc::schmidl_cox(&samples) {
                if r.useful_start > 8000 {
                    let mut s = String::with_capacity(samples.len() * 12);
                    for c in &samples {
                        s.push_str(&format!(
                            "{},{}\n",
                            (c.re * 32768.0).round() as i32,
                            (c.im * 32768.0).round() as i32
                        ));
                    }
                    let _ = std::fs::write("cap_bad.csv", s);
                    log(&format!(
                        "DBG wrote cap_bad.csv (full-search us={})",
                        r.useful_start
                    ));
                    dumped_bad = true;
                }
            }
        }
        // Diagnostic: sample the first 8 captures, then every 64th, and log
        // the signal energy just after useful_start (a spurious trigger on
        // an inter-frame gap has little energy there).
        if win_start == 0 {
            dbg_count += 1;
        }
        if !is_grid && win_start == 0 && (dbg_count <= 8 || dbg_count % 64 == 0) {
            let sc = pc::schmidl_cox(&samples);
            let sig = phy.demodulate_sig(&samples);
            let (m, us, cfo) = match sc {
                Some(r) => (r.metric, r.useful_start as i64, r.cfo_scs),
                None => (-1.0, -1, 0.0),
            };
            // Energy in the SIG symbol region vs a "gap" reference near the end.
            let frame_e = if us >= 0 {
                let a = us as usize + 1096;
                let b = (a + 1024).min(samples.len());
                if b > a {
                    samples[a..b].iter().map(|c| c.norm_sqr()).sum::<f32>() / (b - a) as f32
                } else { 0.0 }
            } else { 0.0 };
            let sigtxt = match sig {
                Some((s, ok)) => format!("sig_crc={ok} mcs={} rv={}", s.mcs_index, s.rv),
                None => "sig=nosync".into(),
            };
            log(&format!(
                "DBG cap: metric={:.3} us={} cfo={:+.3} frame_energy={:.4} {}",
                m, us, cfo, frame_e, sigtxt
            ));
        }

        let pre_decoded =
            pre_i.and_then(|i| pre_frames[i].decoded.take());
        let demod = if control.is_none() {
            DemodResult::failed()
        } else if let Some(d) = pre_i.and_then(|i| pre_frames[i].demod.take()) {
            d // pre-demod from the pool: same mcs because same SIG
        } else if is_grid {
            let need = 2 + pc::data_syms(mcs);
            if win_start + need <= grid.len() {
                phy.llrs_from_grid(&grid[win_start..win_start + need], mcs, 1.0, 0.0)
            } else {
                // frame overflowing the 13-symbol grid (a burst of several blocks): the tail has no
                // spectrum, leave it to ARQ to request again.
                DemodResult::failed()
            }
        } else {
            phy.demodulate_to_llrs(window, mcs)
        };
        let mut delivered = false;
        let mut seg_src_base = false; // v37
        if demod.synced {
            snr_smooth = 0.85 * snr_smooth + 0.15 * demod.snr_db;
            // IR-HARQ: deposit this transmission's LLRs (at its rv) into
            // the accumulated circular buffer for this seq.
            let combined =
                pre_decoded.is_none() && harq.map.contains_key(&seq);
            if combined {
                harq_combines += 1;
            }
            // v4.2: frame decoded in the fabric, straight to parse_block
            let (dec, buffers) = if let Some(pl) = pre_decoded {
                let dec = DecodeOutcome {
                    payload: pl,
                    cw_ok: vec![true; pc::CODEWORDS_PER_FRAME],
                    pre_ber: 0.0,
                };
                (dec, None)
            } else {
                let prev_buffers = harq
                    .map
                    .get(&seq)
                    .cloned()
                    .unwrap_or_else(|| vec![0.0f32; pc::FRAME_BUFFER]);
                let mut buffers = prev_buffers.clone();
                pc::scatter_frame_llrs(mcs, rv, &demod.llrs, &mut buffers);

                let mut posteriors = vec![0.0f32; pc::FRAME_BUFFER];
                let mut dec =
                    phy.decode_buffers(&buffers, Some(&mut posteriors));

                // BICM-ID outer iteration on residual failures: re-demap
                // this transmission's symbols with extrinsic as priors.
                if dec.cw_ok.iter().any(|&x| !x) && !demod.eq_syms.is_empty()
                {
                    bicm_runs += 1;
                    let priors = pc::gather_extrinsic_priors(
                        mcs, rv, &posteriors, &demod.llrs,
                    );
                    let ext =
                        phy.redemap_with_priors(mcs, &demod.eq_syms, &priors);
                    let mut buffers2 = prev_buffers.clone();
                    pc::scatter_frame_llrs(mcs, rv, &ext, &mut buffers2);
                    let dec2 = phy.decode_buffers(&buffers2, None);
                    let ok1 = dec.cw_ok.iter().filter(|&&x| x).count();
                    let ok2 = dec2.cw_ok.iter().filter(|&&x| x).count();
                    if ok2 > ok1 {
                        dec = dec2;
                        if ok2 == pc::CODEWORDS_PER_FRAME {
                            bicm_rescues += 1;
                        }
                    }
                }
                (dec, Some(buffers))
            };

            pre_ber = dec.pre_ber;
            if let Some(seg) = parse_block(&dec.payload) {
                seg_src_base = seg.src == SourceType::H264Base;
                if combined {
                    harq_recovered += 1;
                }
                harq.remove(seq);
                delivered_seqs.insert(seq);
                delivered_order.push_back(seq);
                if delivered_order.len() > 512 {
                    if let Some(old) = delivered_order.pop_front() {
                        delivered_seqs.remove(&old);
                    }
                }
                let sid = seg.frame_id;
                let is_base = seg.src == SourceType::H264Base;
                if let Some(done) = if is_base {
                    reasm_base.push(seg)
                } else {
                    reasm.push(seg)
                } {
                    completed_frames.push(done);
                }
                if is_base {
                    // base layer: small frames (usually 1 block + parity); staleness is simply by
                    // ITS OWN newest id
                    for p in reasm_base.take_stale(sid, 2, 0.6) {
                        completed_frames.push(p);
                    }
                }
                // v-slice: a frame more than 2 ids OLD and still incomplete has no chance of
                // completing -> DELIVER WHAT IS THERE (>= 60 % of its blocks) instead of dropping
                // it. The encoder cuts independent slices, so the intact slices still decode: "one
                // torn band" instead of a FROZEN picture waiting for an IDR.
                if !is_base && shared.partial_ok.load(Ordering::Relaxed) {
                    // (gate !is_base: a base block's sid is its OWN sequence; counting it into the
                    // main funnel makes stale eviction use the wrong mark)
                    for p in reasm.take_stale(sid, 2, 0.6) {
                        completed_frames.push(p);
                    }
                }
                delivered = true;
            } else if let Some(b) = buffers {
                // Persist the raw channel accumulation for the next rv.
                harq.insert(seq, b);
            }
        } else {
            sync_failures += 1;
        }
        // Short window (~2.5 s of attempts): the TX adaptation loop reacts
        // per feedback poll, so a long memory here would keep re-triggering
        // downshifts on stale errors and make the MCS hunt.
        //
        // Only REAL frame attempts count: over the air the FPGA trigger
        // also fires on WiFi bursts and noise (junk = no decodable SIG).
        // Counting junk as block errors reported 50%+ phantom BLER and
        // pinned the link adaptation at MCS0 even though actual frames
        // decoded cleanly. Genuine on-air losses still reach the TX via
        // the seq-gap NACK path.
        if control.is_some() {
            bler_window.push_back(!delivered);
            if bler_window.len() > 30 {
                bler_window.pop_front();
            }
        }
        if delivered {
            segs_ok += 1;
            // v32.1: use the mcs from SIG (the `mcs` variable, always real, on both the fabric and
            // the PC demod path), NOT the header's mcs_byte (the fabric DecFrame leaves the 0xFF
            // sentinel there -> ok_mcs stayed empty, the minstrel table starved for statistics ->
            // the controller sank to MCS0 and stuck).
            let mi = mcs.index();
            if mi < 6 {
                ok_mcs[mi] = ok_mcs[mi].wrapping_add(1);
            }
            if seg_src_base {
                ok_base = ok_base.wrapping_add(1);
            }
        } else {
            segs_lost += 1;
            // Only NACK with a valid link sequence: a junk capture (SIG
            // unreadable, hardware mode) has nothing the TX could look up.
            if link_seq.is_some() || mcs_byte != 0xFF {
                send_msg(&shared, &Msg::Nack { seq });
                outstanding_nacks.entry(seq).or_insert((Instant::now(), 1));
            }
        }

        // Quality telemetry per demodulated frame: sub-frames (win_start>0)
        // and any degraded frame — hunting the periodic constellation flare.
        // Also log a random sample of CLEAN head frames for comparison.
        if !is_grid && (win_start > 0 || pre_ber > 1.0e-3 || seq % 32 == 0) {
            let sc_pre = pre_i.and_then(|i| pre_frames[i].dbg_sc);
            let (us, met, cfo) = match sc_pre {
                Some(v) => v,
                None => match pc::schmidl_cox(
                    &window[..window.len().min(search_limit)],
                ) {
                    Some(r) => (r.useful_start as i64, r.metric, r.cfo_scs),
                    None => (-1, -1.0, 0.0),
                },
            };
            // Energy right BEFORE the preamble (should be silence for the
            // head frame) and in the first data symbol.
            let pre_e = if us > 1100 {
                let a = us as usize - 1100;
                window[a..a + 1024].iter().map(|c| c.norm_sqr()).sum::<f32>() / 1024.0
            } else {
                -1.0
            };
            log(&format!(
                "DBG frm: win={} seq={} snr={:.1} preBER={:.2e} us={} met={:.3} \
                 cfo={:+.3} pre_e={:.6} ok={}",
                win_start, seq, demod.snr_db, pre_ber, us, met, cfo, pre_e, delivered
            ));
        }
        // v12.3: the displayed MCS = the frame that REALLY delivered video. Gating on
        // control.is_some() alone let OTA control leaks (SIG=MCS3) and Wi-Fi false positives (MCS0)
        // make the UI jump while the TX was pinned (measured: TX pinned at MCS5, RX showed 3/0/5).
        // The fabric path (DecFrame) sets disp_mcs where it receives; the PC demod path sets it
        // here on delivery.
        if delivered {
            last_mcs = mcs;
            shared.disp_mcs.store(mcs.index() as u8, Ordering::Relaxed);
        }
        last_demod = demod;
        // Move to the next frame inside this capture, if the window can
        // still hold one.
        match adv {
            Some(a) if adv_fits(a) => win_start = a,
            _ => break,
        }
        } // ---- end multi-frame loop ------------------------------------

        // Completed application frame -> reorder -> decode & publish.
        //
        // ARQ-recovered segments complete their video frame LATE: without a
        // reorder stage the decoder would see frame N after N+1, corrupt its
        // prediction chain and skip everything until the next IDR — every
        // on-air loss became a multi-frame freeze even though the data was
        // recovered. Hold completed frames briefly and release them in id
        // order; a hole that outlives HOLD (two retransmit round-trips) is
        // abandoned (skip ahead + request an IDR).
        // v40.35: a retransmission needs NACK -> control burst -> TX loop ->
        // conv queue -> air: measured RTT 60-200 ms; 300 ms covers it.
        const HOLD: Duration = Duration::from_millis(300);
        // The base layer is split off BEFORE the funnel: delivered straight in its own id order
        // (usually 1-2 blocks per frame, no reorder needed for ARQ); only old/duplicate frames are
        // blocked so the base decoder's reference chain stays intact.
        let mut base_ready: Vec<(u32, SourceType, Vec<u8>)> = Vec::new();
        let mut main_completed: Vec<(u32, SourceType, Vec<u8>)> = Vec::new();
        for (id, src, data) in completed_frames {
            if src == SourceType::H264Base {
                if let Some(n) = last_base_id {
                    // accept only ids moving FORWARD (wrapping): id <= n is a repeat or old.
                    // v40.25: an id FAR BACK (> 4096) = the TX renumbered (app restart) -> accept
                    // again, otherwise the base layer stays DEAD for good after every nyx-tx
                    // restart.
                    let behind = id.wrapping_sub(n) > u32::MAX / 2;
                    if id == n || (behind && n.wrapping_sub(id) <= 4096) {
                        continue;
                    }
                    if behind {
                        log(&format!("base: TX renumbered (id {id}, was {n}) -> resyncing"));
                        vdec_base = VideoDecoder::new();
                    }
                }
                last_base_id = Some(id);
                base_ready.push((id, src, data));
            } else {
                main_completed.push((id, src, data));
            }
        }
        for (id, src, data) in main_completed {
            let ahead = next_deliver_id.map_or(0, |n| id.wrapping_sub(n));
            if ahead <= 1000 {
                // At or ahead of the delivery cursor: queue in order.
                pending.entry(id).or_insert_with(|| (src, data, Instant::now()));
                if next_deliver_id.is_none() {
                    next_deliver_id = Some(id);
                }
            } else if ahead > u32::MAX - 256 {
                // Slightly BEHIND the cursor: a stale ARQ recovery whose
                // frame was already skipped. Feeding it to the decoder now
                // would corrupt the GOP — drop it.
            } else {
                // Wild jump: the TX restarted its numbering. Resync.
                pending.clear();
                pending.insert(id, (src, data, Instant::now()));
                next_deliver_id = Some(id);
                // v40.24: new encoder -> the old decoder rejects even IDRs; recreate it right away
                vdec = VideoDecoder::new();
                vdec_base = VideoDecoder::new();
                vdec_reset_at = Instant::now();
                vdec_fail_at_reset = pics_fail;
                vdec_resets += 1;
                dbg_dec_n = 0;
                // v40.25: all bookkeeping derived from the TX's NUMBERING has to restart too: the
                // link seq restarts from the new LSB, and pending NACKs / delivered / HARQ /
                // partial fragments of the old numbering are meaningless (measured: after a nyx-tx
                // restart the RX took 4+ minutes to settle by itself).
                last_link_seq = None;
                outstanding_nacks.clear();
                delivered_seqs.clear();
                delivered_order.clear();
                harq = HarqCache::new();
                reasm = Reassembler::new();
                reasm_base = Reassembler::new();
                last_base_id = None;
                log(&format!("vdec: TX renumbered (id {id}) -> recreating the decoder (#{vdec_resets}) and resyncing seq/NACK/HARQ/reassembly"));
            }
        }
        let mut ready: Vec<(u32, SourceType, Vec<u8>)> = Vec::new();
        while let Some(n) = next_deliver_id {
            if let Some((src, data, _)) = pending.remove(&n) {
                ready.push((n, src, data));
                next_deliver_id = Some(n.wrapping_add(1));
            } else if !pending.is_empty()
                && !reasm.has_partial(n)
                && outstanding_nacks.is_empty()
            {
                // v40.35: ... but only while NO link seq is missing. A hole with a NACK outstanding
                // is a frame lost on air (1-2 blocks per P-frame: the whole frame vanished, no
                // partial to see) and its retransmission is on its way - hold for it below.
                // Skipping it at once broke the GOP and made the retransmission arrive behind the
                // cursor (measured: 139 broken GOPs / 4.5 min).
                // v37.1: a hole the funnel has NEVER seen a fragment of = a frame that does not
                // exist (the TX increments the id unconditionally every tick: an empty tick or a
                // frame dropped by budget both leave a hole) -> skip it AT ONCE, no HOLD, not
                // counted as lost, no IDR request. Treating every hole as a real loss cost 250 ms
                // plus an IDR per hole -> an IDR storm ate the budget -> the TX dropped more -> a
                // death spiral (measured: frames_lost 1193/90 s, all phantom holes, the main layer
                // never came up).
                next_deliver_id = Some(n.wrapping_add(1));
            } else if pending
                .values()
                .any(|(_, _, t)| t.elapsed() > HOLD)
            {
                // The hole at `n` is not coming back in time: skip to the
                // oldest frame we do have and mark the loss.
                let (&skip_to, _) = pending.iter().next().unwrap();
                log(&format!("reorder: frame {n} not recovered within {} ms -> skipping to {skip_to}, asking for a keyframe", HOLD.as_millis()));
                frames_lost += skip_to.wrapping_sub(n) as u64;
                need_idr = true;
                next_deliver_id = Some(skip_to);
            } else {
                break;
            }
        }

        let mut rx_bytes = 0usize;
        let mut display: Option<RgbFrame> = None;
        let mut disp_id = 0u32; // frame_id of the frame about to be shown (G2G display mark)
        // base first, main second: a Some main takes the display (the main layer has priority)
        for (_id, src, data) in base_ready.into_iter().chain(ready) {
            // End-to-end latency mark (pairs with "LAT tx" in nyx-tx). Main layer only:
            // the base layer numbers its frames separately, and pairing a base id with
            // the main id of the same value once produced seconds of phantom latency.
            if _id % 8 == 0 && src != SourceType::H264Base {
                log(&format!("LAT rx id={_id}"));
            }
            rx_bytes += data.len();
            frames_rx += 1;
            fps_events.push_back(Instant::now());
            let d = match src {
                SourceType::Jpeg => decode_jpeg(&data),
                SourceType::H264 => {
                    let out = vdec.decode(&data);
                    if dbg_dec_n < 40 || _id % 8 == 0 {
                        dbg_dec_n += 1;
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
                        log(&format!(
                            "dec-in #{dbg_dec_n}: id={_id} len={} nals={nals:?} -> {}",
                            data.len(), if out.is_some() { "OK" } else { "BAD" }));
                    }
                    match &out {
                        Some(_) => {
                            need_idr = false;
                            pics_ok += 1;
                            last_main = std::time::Instant::now();
                        }
                        None => {
                            need_idr = true; // waiting for a keyframe
                            pics_fail += 1;
                            // v40.24: frames keep arriving but no picture for > 2 s -> the decoder
                            // is stuck (even an IDR does not save it) -> recreate
                            if last_main.elapsed() > Duration::from_secs(6)
                                && vdec_reset_at.elapsed() > Duration::from_secs(10)
                                && pics_fail >= vdec_fail_at_reset + 40
                            {
                                vdec = VideoDecoder::new();
                                vdec_reset_at = Instant::now();
                                vdec_fail_at_reset = pics_fail;
                                vdec_resets += 1;
                                dbg_dec_n = 0;
                                log(&format!(
                                    "vdec: {} s with no decoded picture (pics_fail={pics_fail}) -> recreating the decoder (#{vdec_resets})",
                                    last_main.elapsed().as_secs()));
                            }
                        }
                    }
                    out
                }
                SourceType::H264Base => {
                    // ALWAYS decode (keeps the reference chain continuous; stopping mid-way breaks
                    // it until the next IDR); only show when main is late by > 250 ms. Do NOT touch
                    // need_idr/pics (those are the main layer's numbers).
                    let out = vdec_base.decode(&data);
                    if last_main.elapsed().as_millis() > 250 {
                        out
                    } else {
                        None
                    }
                }
                SourceType::RawData => Some(bytes_to_image(&data, 192)),
                SourceType::Text => {
                    let t = String::from_utf8_lossy(&data).into_owned();
                    log(&format!("MSG received: {t}"));
                    let mut m = shared.msgs.lock().unwrap();
                    m.push(format!("← {t}"));
                    if m.len() > 200 { m.remove(0); }
                    None
                }
                SourceType::Info => {
                    // v40.33: lic_* lines of the transmit end - shown, never forwarded
                    *shared.far_lic.lock().unwrap() = String::from_utf8_lossy(&data).into_owned();
                    None
                }
                SourceType::Data => {
                    // v40.22: user datagram from nyx-tx -> UDP out, unchanged
                    let n = shared.tlm_rx.fetch_add(1, Ordering::Relaxed) + 1;
                    if n == 1 {
                        log(&format!("telemetry: first Data packet from the link ({} B) -> UDP out", data.len()));
                    }
                    if let Some((s, addr)) = shared.tlm_out.lock().unwrap().as_ref() {
                        let _ = s.send_to(&data, addr);
                    }
                    None
                }
            };
            if d.is_some() {
                disp_events.push_back(Instant::now());
                layer_main = !matches!(src, SourceType::H264Base);
                display = d;
                disp_id = _id;
                // v12.2: GLASS-OUT mark: the frame is decoded and ready to display.
                if _id % 8 == 0 {
                    log(&format!("G2G out id={_id}"));
                }
            }
        }
        if rx_bytes > 0 {
            goodput.push_back((Instant::now(), rx_bytes));
        }

        // Rolling windows.
        if let Some(cut) = Instant::now().checked_sub(Duration::from_secs(2)) {
            while fps_events.front().is_some_and(|&t| t < cut) {
                fps_events.pop_front();
            }
            while disp_events.front().is_some_and(|&t| t < cut) {
                disp_events.pop_front();
            }
            while goodput.front().is_some_and(|&(t, _)| t < cut) {
                goodput.pop_front();
            }
        }
        let bler = if bler_window.is_empty() {
            0.0
        } else {
            bler_window.iter().filter(|&&f| f).count() as f32 / bler_window.len() as f32
        };

        maybe_feedback(&shared, &mut last_feedback, snr_smooth, &bler_window,
                       segs_ok, segs_lost, need_idr,
                       ok_mcs, ok_base);

        // ---- v14: publish the frame THROUGH the jitter buffer (no ui lock: avoids the deadlock
        // where the playout thread holds playout and then locks ui) ----
        if let Some(f) = display {
            let now = Instant::now();
            if let Some(prev) = last_disp {
                let dt = now.duration_since(prev).as_micros() as u64;
                if (10_000..200_000).contains(&dt) {
                    let old = shared.frame_period_us.load(Ordering::Relaxed);
                    shared.frame_period_us
                        .store((old * 3 + dt) / 4, Ordering::Relaxed);
                }
            }
            last_disp = Some(now);
            // v21.1: always go through playout; with jitter=0 the playout thread emits within its 4
            // ms tick (latest wins). demod_loop NEVER locks ui (the GUI holding the lock during a
            // drag stalls the whole chain).
            {
                let mut q = shared.playout.lock().unwrap();
                q.push_back((disp_id, f));
                while q.len() > 16 {
                    q.pop_front();
                }
            }
        }
        // ------------------------------------------------- publish UI (stats)
        {
            let rx_fps = fps_events.len() as f32 / 2.0;
            let goodput_kbps =
                goodput.iter().map(|&(_, b)| b).sum::<usize>() as f32 * 8.0 / 2.0 / 1000.0;
            let backlog_dropped = shared.dropped.load(Ordering::Relaxed);
            // v21.1: try_lock; if the GUI holds it (drag/modal), skip this stats tick (a new one
            // comes 50 ms later); demod_loop waits for nobody.
            if let Ok(mut ui) = shared.ui.try_lock() {
                {
                    let m = &mut ui.metrics;
                    m.snr_db = snr_smooth;
                    m.cfo_hz = last_demod.cfo_hz;
                    m.pre_ber = pre_ber;
                    m.sync_metric = last_demod.sync_metric;
                    m.sync_failures = sync_failures;
                    m.segs_ok = segs_ok;
                    m.segs_lost = segs_lost;
                    m.bler_recent = bler;
                    m.frames_rx = frames_rx;
                    m.frames_lost = frames_lost;
                    m.rx_fps = rx_fps;
                    m.disp_fps = disp_events.len() as f32 / 2.0;
                    m.layer_main = layer_main;
                    m.pics_ok = pics_ok;
                    m.pics_fail = pics_fail;
                    m.goodput_kbps = goodput_kbps;
                    // v12.3: read disp_mcs (written by both the fabric DecFrame and the PC demod)
                    // -> the console stats match the UI, no jumping on leaks or junk.
                    m.active_mcs = Mcs::from_index(
                        shared.disp_mcs.load(Ordering::Relaxed) as usize);
                    m.backlog_dropped = backlog_dropped;
                    m.harq_combines = harq_combines;
                    m.harq_recovered = harq_recovered;
                    m.bicm_runs = bicm_runs;
                    m.bicm_rescues = bicm_rescues;
                    m.sig_failures = sig_failures;
                }
                // take(): when a tick is skipped (GUI holds the lock) the data stays in last_demod
                // and the next tick publishes it.
                if !last_demod.constellation.is_empty() {
                    ui.constellation = std::mem::take(&mut last_demod.constellation);
                }
                if !last_demod.chan_mag.is_empty() {
                    ui.chan_mag = std::mem::take(&mut last_demod.chan_mag);
                }
                ui.snr_history.push_back(snr_smooth);
                if ui.snr_history.len() > 180 {
                    ui.snr_history.pop_front();
                }
                ui.bler_history.push_back(bler);
                if ui.bler_history.len() > 180 {
                    ui.bler_history.pop_front();
                }
            }

            status.tick(&format!(
                "status | mcs={} synced={} snr={:.1}dB cfo={:+.0}Hz preBER={:.2e} \
                 bler={:.1}% segs={}/{} vframes={}/{} rx_fps={:.1} goodput={:.0}kbps \
                 sync_fail={} sig_fail={} backlog_drop={} harq={}/{} bicm={}/{} \
                 pics={}/{} okm={:?}",
                last_mcs.label(),
                last_demod.synced as u8,
                snr_smooth,
                last_demod.cfo_hz,
                pre_ber,
                bler * 100.0,
                segs_ok,
                segs_lost,
                frames_rx,
                frames_lost,
                rx_fps,
                goodput_kbps,
                sync_failures,
                sig_failures,
                backlog_dropped,
                harq_combines,
                harq_recovered,
                bicm_runs,
                bicm_rescues,
                pics_ok,
                pics_fail,
                ok_mcs,
            ));
        }
    }
}

fn maybe_feedback(
    shared: &Shared,
    last: &mut Instant,
    snr: f32,
    bler_window: &VecDeque<bool>,
    segs_ok: u64,
    segs_lost: u64,
    need_idr: bool,
    ok_mcs: [u16; 6],
    ok_base: u16,
) {
    // An IDR request must not wait for the reporting cadence: every frame of delay is a frozen
    // frame at the display (the decoder is dead until the keyframe arrives). Everything else can
    // keep the 400 ms pace. v-lat: 400 -> 150 ms. The TX flow control runs on segs_ok in the
    // feedback; sparse feedback means the TX must keep a WIDE window (more delay) or throttle
    // itself (measured: 400 ms + a 10-block window -> rx_fps collapsed to 4).
    if !need_idr && last.elapsed() < Duration::from_millis(150) {
        return;
    }
    if need_idr && last.elapsed() < Duration::from_millis(100) {
        return; // still rate-limit the recovery path
    }
    *last = Instant::now();
    let bler = if bler_window.is_empty() {
        0.0
    } else {
        bler_window.iter().filter(|&&f| f).count() as f32 / bler_window.len() as f32
    };
    send_msg(
        shared,
        &Msg::Feedback {
            snr_db: snr, bler, segs_ok, segs_lost, need_idr, ok_mcs, ok_base,
        },
    );
}

// ---------------------------------------------------------------- GUI --

pub struct RxApp {
    shared: Arc<Shared>,
    tex: Option<egui::TextureHandle>,
    tex_ver: u64,
    addr_buf: String,
    /// Control of the embedded receive board (the daemon's console on :7202).
    board: nyx_common::boardctl::BoardCtl,
    /// v40.34: the board tuning fields (shared form with the other apps).
    form: nyx_common::ui::RadioForm,
    chan_form: nyx_common::ui::ChannelForm,
    lic_buf: String,
    lic_status: String,
    msg_buf: String,
    /// v40.34: the settings drawer (gear / H key).
    drawer_open: bool,
    /// Draw the SNR and frame-rate plates over the video (drawer header).
    hud_plates: bool,
    // HUD event banner: MCS change, signal lost / recovered
    ev_mcs: Option<usize>,
    ev_live: Option<bool>,
    ev_lost_at: Option<Instant>,
    ev_banner: Option<(String, egui::Color32, Instant, bool)>,
}

impl RxApp {
    pub fn new(shared: Arc<Shared>) -> Self {
        let addr_buf = shared.channel_addr.lock().unwrap().clone();
        let sh = shared.clone();
        let board = nyx_common::boardctl::BoardCtl::spawn(
            Arc::new(move || {
                let a = sh.channel_addr.lock().unwrap().clone();
                let host = a.split(':').next().unwrap_or("192.168.0.11");
                format!("{host}:7202")
            }),
            &["get", "capstat", "rssi", "trig", "softagc", "hop status", "license", "role"],
        );
        RxApp {
            shared,
            msg_buf: String::new(),
            tex: None,
            tex_ver: u64::MAX,
            addr_buf,
            board,
            form: nyx_common::ui::RadioForm::default(),
            chan_form: nyx_common::ui::ChannelForm::default(),
            lic_buf: String::new(),
            lic_status: String::new(),
            drawer_open: false,
            hud_plates: true,
            ev_mcs: None,
            ev_live: None,
            ev_lost_at: None,
            ev_banner: None,
        }
    }

    /// Event banner for the HUD: MCS moves, signal lost / recovered.
    fn banner(&mut self, mcs: Option<usize>, live: bool) -> Option<(String, egui::Color32)> {
        use nyx_common::theme as th;
        if let (Some(prev), Some(cur)) = (self.ev_mcs, mcs) {
            if cur != prev {
                let (txt, c) = if cur < prev {
                    (format!("LINK ADAPTING · MCS {prev} → {cur}"), th::WARN)
                } else {
                    (format!("LINK IMPROVED · MCS {prev} → {cur}"), th::GOOD)
                };
                self.ev_banner = Some((txt, c, Instant::now(), false));
            }
        }
        if mcs.is_some() {
            self.ev_mcs = mcs;
        }
        if self.ev_live == Some(true) && !live {
            self.ev_lost_at = Some(Instant::now());
            self.ev_banner = Some(("SIGNAL LOST - AUTO-RECOVERING".into(), th::BAD, Instant::now(), true));
        }
        if self.ev_live == Some(false) && live {
            if let Some(t0) = self.ev_lost_at.take() {
                self.ev_banner = Some((
                    format!("LINK RECOVERED · {:.1}s", t0.elapsed().as_secs_f32()),
                    th::GOOD,
                    Instant::now(),
                    false,
                ));
            }
        }
        self.ev_live = Some(live);
        match &self.ev_banner {
            Some((txt, c, t0, sticky)) if *sticky && !live => Some((txt.clone(), *c)),
            Some((txt, c, t0, _)) if t0.elapsed().as_secs_f32() < 4.0 => Some((txt.clone(), *c)),
            Some(_) => {
                self.ev_banner = None;
                None
            }
            None => None,
        }
    }

    /// Everything behind the gear.
    fn drawer(&mut self, ui: &mut egui::Ui) {
        use nyx_common::theme as th;
        use nyx_common::ui as nu;
        let st = self.board.snapshot();
        let connected = self.shared.connected.load(Ordering::Relaxed);

        nu::link_section(ui, &st, nu::Role::Ground, &self.board);
        nu::role_section(ui, &st, &self.board);
        nu::channel_section(ui, &st, &mut self.chan_form, &self.board);
        {
            let far = self.shared.far_lic.lock().unwrap().clone();
            nu::licence_section(
                ui, &st, if far.is_empty() { None } else { Some(far.as_str()) },
                &mut self.lic_buf, &mut self.lic_status, &self.board,
            );
        }
        nu::radio_section(ui, &st, nu::Role::Ground, &mut self.form, &self.board, &mut |hz| {
            pc::set_samp_rate_hz(hz);
        });
        {
            let msgs = self.shared.msgs.lock().unwrap().clone();
            if let Some(t) = nu::messages_section(ui, &msgs, &mut self.msg_buf) {
                send_msg(&self.shared, &Msg::UserText { text: t.clone() });
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
            let connect = nu::connection_section(ui, &mut self.addr_buf, "ip:7011", connected, Some(&mut udp));
            if udp != udp0 {
                self.shared.udp_mode.store(udp, Ordering::Relaxed);
                if let Some(s) = self.shared.writer.lock().unwrap().take() {
                    let _ = s.shutdown(std::net::Shutdown::Both);
                }
                log(&format!("transport -> {}", if udp { "UDP" } else { "TCP" }));
            }
            if connect {
                *self.shared.channel_addr.lock().unwrap() = self.addr_buf.trim().to_string();
                if let Some(s) = self.shared.writer.lock().unwrap().take() {
                    let _ = s.shutdown(std::net::Shutdown::Both);
                }
                log(&format!("connect requested -> {}", self.addr_buf.trim()));
            }
        }
        th::section(ui, "Advanced", false, |ui| {
            th::row(ui, "Smoothing", |ui| {
                let mut j = self.shared.jitter_ms.load(Ordering::Relaxed);
                if ui
                    .add(egui::Slider::new(&mut j, 0..=200).suffix(" ms"))
                    .on_hover_text("Playout buffer: higher = smoother but more delay. ~80 ms is OcuSync-like; 0 = lowest latency.")
                    .changed()
                {
                    self.shared.jitter_ms.store(j, Ordering::Relaxed);
                }
            });
            th::row(ui, "I/Q format", |ui| {
                let cur = self.shared.iq_mode.load(Ordering::Relaxed).to_string();
                if let Some(v) = nu::segmented(
                    ui,
                    &[("as-is", "0"), ("conj", "1"), ("swap", "2"), ("swap+conj", "3"), ("neg-I", "4")],
                    &cur,
                ) {
                    let m: usize = v.parse().unwrap_or(0);
                    self.shared.iq_mode.store(m, Ordering::Relaxed);
                    log(&format!("iqmode -> {m}"));
                }
            });
            ui.add_space(6.0);
            th::card_title(ui, "decode details");
            let metrics = self.shared.ui.lock().unwrap().metrics.clone();
            let dm = self.shared.disp_mcs.load(Ordering::Relaxed);
            egui::Grid::new("rx_stats").num_columns(2).striped(true).spacing([16.0, 6.0]).show(ui, |ui| {
                let kvs: [(&str, String); 16] = [
                    ("MCS from air", mcs_disp_label(dm as usize)),
                    ("SNR est", format!("{:.1} dB", metrics.snr_db)),
                    ("CFO est", format!("{:+.0} Hz", metrics.cfo_hz)),
                    ("pre-FEC BER", format!("{:.2e}", metrics.pre_ber)),
                    ("BLER recent", format!("{:.1} %", metrics.bler_recent * 100.0)),
                    ("Sync metric", format!("{:.2}", metrics.sync_metric)),
                    ("Segments ok / lost", format!("{} / {}", metrics.segs_ok, metrics.segs_lost)),
                    ("Sync failures", format!("{}", metrics.sync_failures)),
                    ("Video frames / lost", format!("{} / {}", metrics.frames_rx, metrics.frames_lost)),
                    ("Backlog dropped", format!("{}", metrics.backlog_dropped)),
                    ("Displayed rate", format!("{:.1} fps", metrics.disp_fps)),
                    ("Frames on air", format!("{:.1} fps", metrics.rx_fps)),
                    ("Goodput", format!("{:.0} kbit/s", metrics.goodput_kbps)),
                    ("Layer shown", if metrics.layer_main { "main".into() } else { "base".into() }),
                    ("HARQ combines / recovered", format!("{} / {}", metrics.harq_combines, metrics.harq_recovered)),
                    ("BICM-ID runs / rescues · SIG fails", format!("{} / {} · {}", metrics.bicm_runs, metrics.bicm_rescues, metrics.sig_failures)),
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

impl eframe::App for RxApp {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = root.ctx().clone();
        // v21.1: clone the frame UNDER the lock (~100 µs) and release AT ONCE; convert + texture
        // upload (~ms) happen outside. The GUI holding the ui lock for long once helped stall
        // demod_loop (24/7 bench).
        let pending: Option<(u64, RgbFrame)> = {
            let ui_state = self.shared.ui.lock().unwrap();
            if ui_state.rx_version != self.tex_ver {
                self.tex_ver = ui_state.rx_version;
                let id = ui_state.rx_id;
                if id % 8 == 0 {
                    log(&format!("G2G disp id={id}"));
                }
                ui_state.rx_frame.clone().map(|f| (ui_state.rx_version, f))
            } else {
                None
            }
        };
        if let Some((_ver, f)) = pending {
            // v40.25: nyx-rx died with a wgpu panic "Texture invalid" when the picture kept
            // switching 480x360 <-> 320x240. A 0x0 frame or a byte-count mismatch -> drop; a size
            // change -> a NEW handle instead of set() on the same id.
            if f.width == 0 || f.height == 0 || f.rgb.len() != f.width * f.height * 3 {
                log(&format!("gui: dropping frame of odd size {}x{} ({} B)", f.width, f.height, f.rgb.len()));
            } else {
                let img = to_color_image(&f);
                let same = self.tex.as_ref().map_or(false, |t| t.size() == [f.width, f.height]);
                match &mut self.tex {
                    Some(t) if same => t.set(img, egui::TextureOptions::LINEAR),
                    _ => {
                        self.tex = Some(ctx.load_texture("rx", img, egui::TextureOptions::LINEAR));
                    }
                }
            }
        }

        // H = settings drawer (same as the gear)
        if ctx.input(|i| i.key_pressed(egui::Key::H)) {
            self.drawer_open = !self.drawer_open;
        }
        // O: the readouts over the video, the same switch as in the drawer header.
        if ctx.input(|i| i.key_pressed(egui::Key::O)) {
            self.hud_plates = !self.hud_plates;
        }

        use nyx_common::ui as nu;
        let st = self.board.snapshot();
        let m = self.shared.ui.lock().unwrap().metrics.clone();
        let connected = self.shared.connected.load(Ordering::Relaxed);
        let dm = self.shared.disp_mcs.load(Ordering::Relaxed) as usize;
        let live = connected && m.rx_fps >= 5.0;
        let mcs = Mcs::from_index(dm).map(|x| x.index());
        let banner = self.banner(mcs, live);
        let data = nu::HudData {
            connected,
            live,
            snr_db: m.snr_db,
            fps: m.disp_fps,
            kbps: m.goodput_kbps,
            mcs: mcs_disp_short(dm),
            pills: vec![nu::link_pill(&st), nu::licence_pill(&st), nu::mode_pill(&st)],
            banner,
            title: "NYXHOP · GROUND".into(),
            empty_text: "waiting for video…".into(),
            plates: self.hud_plates,
        };
        let tex = self.tex.clone();
        let mut open = self.drawer_open;
        let mut plates = self.hud_plates;
        nu::screen(root, &mut open, &mut plates, "Settings", |ui| self.drawer(ui), |ui| nu::hud(ui, tex.as_ref(), &data));
        self.drawer_open = open;
        self.hud_plates = plates;
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}
