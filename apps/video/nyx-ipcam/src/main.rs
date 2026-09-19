//! nyx-ipcam: the transmitting end with the IP camera plugged straight into the board.
//!
//! The camera and the board share an Ethernet segment. This process pulls the camera's RTSP
//! stream and hands its H.264 access units, untouched, to nyx-tx's transmit path (framing,
//! parity, ARQ, rate choice), which talks to the radio daemon on 127.0.0.1:7010. Nothing is
//! decoded or encoded here: the board's two A9 cores stay with the radio.
//!
//! It runs on every board whatever its role and follows the daemon (`role` on :7202):
//!   * receiving end (`rx`), or no camera configured: standby, no camera pulled, port 7010 free;
//!   * transmitting end with a camera that sends pictures: it takes port 7010 and sends;
//!   * transmitting end, camera silent for 5 s: port 7010 is left to a PC app (nyxhop Aircraft).
//!
//! Control: the console on :7201 (`get`, `stats`, `set rtsp <url>`, `set source rtsp|pattern`,
//! `save`), used by the PC window nyx-ipcam-ctl.
//!
//! Options (all have board defaults): --rtsp <url> (first start, before anything is saved),
//! --config, --save-hook <shell command, {path} = the file>, --role-console, and nyx_tx::setup's
//! --channel, --ctl, --tlm-in, --tlm-out.

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use nyx_common::Opts;
use nyx_common::logging::{self, log};
use nyx_common::source::SourceKind;
use nyx_tx::Shared;

const DEFAULTS: &[(&str, &str)] = &[
    ("--channel", "127.0.0.1:7010"),
    ("--ctl", "0.0.0.0:7201"),
    ("--config", "/home/root/nyx-ipcam.cfg"),
    // a flight controller on the same Ethernet sends MAVLink here; it goes up the link
    ("--tlm-in", "0.0.0.0:14557"),
    // nothing on the aircraft listens for what comes down
    ("--tlm-out", "0"),
];

/// A camera that sent nothing for this long gives the transmit port back.
const CAMERA_SILENT: Duration = Duration::from_secs(5);

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    for (k, v) in DEFAULTS {
        if !args.iter().any(|a| a == k) {
            args.push(k.to_string());
            args.push(v.to_string());
        }
    }
    // nothing goes out until the watcher has seen the board's role
    args.push("--standby".into());
    logging::init("ipcam");
    let opts = Opts::from_list(args);
    let cfg = opts.arg("--config", "");
    let fresh = !std::path::Path::new(&cfg).exists();

    let shared = nyx_tx::setup(&opts);
    *shared.save_hook.lock().unwrap() = opts.arg("--save-hook", "");
    {
        let mut c = shared.config.lock().unwrap();
        // Nothing saved yet: this box exists to send the camera.
        if fresh {
            c.source = SourceKind::Rtsp;
            c.rtsp_pass = true;
        }
        let url = opts.arg("--rtsp", "");
        if !url.is_empty() {
            c.source = SourceKind::Rtsp;
            c.rtsp_url = url;
        }
        // There is no encoder in this build: H.264 can only be the camera's own.
        if c.source == SourceKind::Rtsp && !c.rtsp_pass {
            log("ipcam: no H.264 encoder on the board - pass-through forced on");
            c.rtsp_pass = true;
        }
        log(&format!(
            "ipcam: source {:?}, camera {}, config {} ({})",
            c.source,
            if c.rtsp_url.is_empty() { "not set (set rtsp <url> on :7201)" } else { "set" },
            cfg,
            if fresh { "none yet" } else { "loaded" },
        ));
    }
    let console = opts.arg("--role-console", "127.0.0.1:7202");
    let sh = shared.clone();
    let con = console.clone();
    std::thread::Builder::new()
        .name("ipcam-role".into())
        .spawn(move || watch(&sh, &con))
        .expect("spawn ipcam-role");
    let sh = shared.clone();
    std::thread::Builder::new()
        .name("ipcam-pace".into())
        .spawn(move || pace(&sh, &console))
        .expect("spawn ipcam-pace");
    nyx_tx::run_headless(&shared);
    shared.stop.store(true, Ordering::Relaxed);
}

/// One command on the daemon's console, its `key=value` lines. None while the daemon is not
/// answering (restarting after a role change).
fn console_kv(console: &str, cmd: &str) -> Option<HashMap<String, String>> {
    let addr = console.parse().ok()?;
    let mut s = TcpStream::connect_timeout(&addr, Duration::from_millis(800)).ok()?;
    s.set_read_timeout(Some(Duration::from_millis(1500))).ok()?;
    s.write_all(format!("{cmd}\n").as_bytes()).ok()?;
    let mut rd = BufReader::new(s);
    let mut kv = HashMap::new();
    for _ in 0..400 {
        let mut line = String::new();
        if rd.read_line(&mut line).ok()? == 0 {
            break;
        }
        let t = line.trim();
        if t == "ok" || t.starts_with("err") {
            break;
        }
        if let Some((k, v)) = t.split_once('=') {
            kv.insert(k.to_string(), v.to_string());
        }
    }
    Some(kv)
}

/// The daemon's `role` answer ("tx", "rx").
fn board_role(console: &str) -> Option<String> {
    console_kv(console, "role")?.remove("role")
}

/// Keep the camera stream's backlog in this process, not in the daemon.
///
/// The daemon holds 24 air frames in front of its encoder and drops the oldest block when an app
/// offers more than the air takes (TXQ_MAX_FRAMES). A PC app never gets there: its encoder follows
/// the link. The camera does not, and on 17/9 a 3 Mbit/s camera at MCS4 lost 17 blocks a second
/// in that queue, each one a hole in some frame, while this process dropped nothing. So the
/// spacing of what goes to the daemon follows its queue: widened quickly while the queue fills or
/// drops, narrowed slowly while it stays nearly empty. What cannot go then waits here, and frames
/// that get too old are dropped whole, up to the next key frame (worker.rs, pass_next).
fn pace(shared: &Shared, console: &str) {
    // about the daemon's own spacing of conv frames (3 ms): costs nothing while the air keeps up
    const FLOOR_MIN_US: u64 = 2_800;
    const FLOOR_MAX_US: u64 = 12_000;
    let mut floor = FLOOR_MIN_US;
    let mut last_drop: Option<u64> = None;
    let mut logged = (FLOOR_MIN_US, Instant::now());
    while !shared.stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(200));
        if shared.port_off.load(Ordering::Relaxed) {
            last_drop = None;
            continue;
        }
        let Some(st) = console_kv(console, "stats") else { continue };
        let num = |k: &str| st.get(k).and_then(|v| v.parse::<u64>().ok());
        let (Some(depth), Some(drop)) = (num("txq_frames"), num("txq_drop")) else { continue };
        let dropped = last_drop.is_some_and(|d| drop > d);
        last_drop = Some(drop);
        if dropped || depth >= 16 {
            floor = (floor * 5 / 4).min(FLOOR_MAX_US);
        } else if depth <= 4 {
            floor = (floor * 98 / 100).max(FLOOR_MIN_US);
        }
        shared.pace_floor_us.store(floor, Ordering::Relaxed);
        let (lf, lt) = logged;
        if (floor > lf * 3 / 2 || floor < lf * 2 / 3) && lt.elapsed() >= Duration::from_secs(2) {
            log(&format!(
                "ipcam: air frame spacing {:.1} ms (daemon queue {depth} frames{})",
                floor as f32 / 1000.0,
                if dropped { ", it dropped blocks" } else { "" }
            ));
            logged = (floor, Instant::now());
        }
    }
}

/// Follow the board's role and the camera: standby, sending, or port left free.
fn watch(shared: &Shared, console: &str) {
    let mut role = String::from("unknown");
    let mut last_frames = shared.rtsp.frames_total.load(Ordering::Relaxed);
    let mut last_frame_at: Option<Instant> = None;
    let mut note = String::new();
    let mut last_poll = Instant::now() - Duration::from_secs(10);
    while !shared.stop.load(Ordering::Relaxed) {
        if last_poll.elapsed() >= Duration::from_secs(2) {
            last_poll = Instant::now();
            // a daemon restarting (a role change) does not answer for a few seconds: keep the
            // last answer until it does
            if let Some(r) = board_role(console) {
                role = r;
            }
        }
        let n = shared.rtsp.frames_total.load(Ordering::Relaxed);
        if n != last_frames {
            last_frames = n;
            last_frame_at = Some(Instant::now());
        }
        let (source, url_set) = {
            let c = shared.config.lock().unwrap();
            (c.source, !c.rtsp_url.is_empty())
        };
        // The first time ONVIF reaches the camera, its bitrate then is the ceiling the link may
        // raise it back to: remember it in the saved settings, or after a restart the camera's
        // lowered bitrate would pass for its own.
        let base = shared.rtsp.onvif_base_kbps.load(Ordering::Relaxed);
        if base > 0 {
            let mut c = shared.config.lock().unwrap();
            if c.cam_max_kbps == 0 {
                c.cam_max_kbps = base;
                drop(c);
                log(&format!("ipcam: camera ceiling {base} kbps (its bitrate when first reached; `set cammax` changes it)"));
                let _ = nyx_tx::save_config(shared);
            }
        }
        let standby = role != "tx" || source != SourceKind::Rtsp || !url_set;
        let live = last_frame_at.is_some_and(|t| t.elapsed() < CAMERA_SILENT);
        if shared.standby.load(Ordering::Relaxed) != standby {
            shared.standby.store(standby, Ordering::Relaxed);
            if standby {
                last_frame_at = None;
            }
        }
        let port_off = standby || !live;
        if shared.port_off.load(Ordering::Relaxed) != port_off {
            nyx_tx::set_port_off(shared, port_off);
        }
        let now = if role != "tx" {
            format!("standby: the board is the {} end", if role == "rx" { "receiving" } else { "unknown" })
        } else if source != SourceKind::Rtsp {
            "standby: the camera is switched off (source is not rtsp)".to_string()
        } else if !url_set {
            "standby: no camera URL".to_string()
        } else if !live {
            "waiting for the camera; the transmit port is free for a PC app".to_string()
        } else {
            "sending the camera".to_string()
        };
        if now != note {
            log(&format!("ipcam: {now}"));
            *shared.mode_note.lock().unwrap() = now.clone();
            note = now;
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}
