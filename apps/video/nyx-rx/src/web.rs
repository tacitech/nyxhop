//! Web ground station: a small HTTP server (plain std, no dependencies).
//!   GET /        -> dashboard HTML (one file, include_str)
//!   GET /video   -> MJPEG stream (multipart/x-mixed-replace: the browser plays it
//!                   natively in an <img>, no JS decoding)
//!   GET /stats   -> JSON metrics (UiState, the same source as the egui GUI)
//!   GET /cmd/<t>/<line> -> console TCP proxy: t = rx(7203) | tx(7204) |
//!                   a(board A 7202) | b(board B 7202). Returns the text reply.
//! Listens on 0.0.0.0 (bench LAN; the console port is open on the LAN anyway). The egui GUI stays
//! as it is; the web page is an extra front, the engine is unchanged.
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crate::Shared;

const HTML: &str = include_str!("dashboard.html");
// DJI-style HUD: full-screen video with an overlaid read-only OSD, the DEMO screen for visitors.
// Operators use "/" (full controls + event ticker + simulate-jam button).
const HUD: &str = include_str!("hud.html");

pub fn spawn(shared: Arc<Shared>, port: u16) {
    std::thread::Builder::new()
        .name("web".into())
        .spawn(move || {
            let l = match TcpListener::bind(("0.0.0.0", port)) {
                Ok(l) => l,
                Err(e) => {
                    eprintln!("web: bind {port}: {e}");
                    return;
                }
            };
            println!("web: ground-station http://127.0.0.1:{port}");
            for c in l.incoming() {
                if shared.stop.load(Ordering::Relaxed) {
                    return;
                }
                if let Ok(s) = c {
                    let sh = shared.clone();
                    std::thread::spawn(move || {
                        let _ = handle(sh, s);
                    });
                }
            }
        })
        .expect("spawn web");
}

fn handle(shared: Arc<Shared>, s: TcpStream) -> std::io::Result<()> {
    s.set_read_timeout(Some(Duration::from_secs(5)))?;
    let mut r = BufReader::new(s.try_clone()?);
    let mut line = String::new();
    r.read_line(&mut line)?;
    let path = line.split_whitespace().nth(1).unwrap_or("/").to_string();
    loop {
        let mut h = String::new();
        if r.read_line(&mut h)? == 0 || h == "\r\n" || h == "\n" {
            break;
        }
    }
    let mut s = s;
    if path == "/" || path == "/index.html" {
        respond(&mut s, "200 OK", "text/html; charset=utf-8", HTML.as_bytes())
    } else if path == "/hud" || path.starts_with("/hud?") {
        respond(&mut s, "200 OK", "text/html; charset=utf-8", HUD.as_bytes())
    } else if path == "/video" {
        mjpeg(&shared, &mut s)
    } else if path == "/stats" {
        let j = stats_json(&shared);
        respond(&mut s, "200 OK", "application/json", j.as_bytes())
    } else if let Some(rest) = path.strip_prefix("/cmd/") {
        let out = cmd_proxy(rest);
        respond(&mut s, "200 OK", "text/plain; charset=utf-8", out.as_bytes())
    } else {
        respond(&mut s, "404 Not Found", "text/plain", b"not found")
    }
}

fn respond(
    s: &mut TcpStream,
    status: &str,
    ctype: &str,
    body: &[u8],
) -> std::io::Result<()> {
    write!(
        s,
        "HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\n\
         Cache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\n\
         Connection: close\r\n\r\n",
        body.len()
    )?;
    s.write_all(body)
}

/// MJPEG: push a NEW frame (rx_version changed) at up to ~30 fps. JPEG encode at q75 (~15-25 KB at
/// 480x360) outside the ui lock.
fn mjpeg(shared: &Shared, s: &mut TcpStream) -> std::io::Result<()> {
    write!(
        s,
        "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; \
         boundary=nyxframe\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
    )?;
    let mut last_ver = 0u64;
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        let frame = {
            let ui = shared.ui.lock().unwrap();
            if ui.rx_version != last_ver {
                last_ver = ui.rx_version;
                ui.rx_frame.clone()
            } else {
                None
            }
        };
        if let Some(f) = frame {
            let jpg = nyx_common::encode_jpeg(&f, 75);
            write!(
                s,
                "--nyxframe\r\nContent-Type: image/jpeg\r\n\
                 Content-Length: {}\r\n\r\n",
                jpg.len()
            )?;
            s.write_all(&jpg)?;
            s.write_all(b"\r\n")?;
        }
        std::thread::sleep(Duration::from_millis(33));
    }
}

fn stats_json(shared: &Shared) -> String {
    let ui = shared.ui.lock().unwrap();
    let m = &ui.metrics;
    let hist = |v: &std::collections::VecDeque<f32>| -> String {
        let n = v.len();
        v.iter()
            .skip(n.saturating_sub(120))
            .map(|x| format!("{x:.2}"))
            .collect::<Vec<_>>()
            .join(",")
    };
    format!(
        "{{\"snr\":{:.1},\"bler\":{:.3},\"rx_fps\":{:.1},\"goodput\":{:.0},\
         \"mcs\":{},\"frames_rx\":{},\"frames_lost\":{},\"sig_fail\":{},\
         \"segs_ok\":{},\"segs_lost\":{},\"connected\":{},\
         \"snr_hist\":[{}],\"bler_hist\":[{}]}}",
        m.snr_db,
        m.bler_recent,
        m.rx_fps,
        m.goodput_kbps,
        m.active_mcs.map(|x| x.index() as i32).unwrap_or(-1),
        m.frames_rx,
        m.frames_lost,
        m.sig_failures,
        m.segs_ok,
        m.segs_lost,
        shared.connected.load(Ordering::Relaxed),
        hist(&ui.snr_history),
        hist(&ui.bler_history),
    )
}

/// Proxy one console command line: "<target>/<cmd pct-encoded>". Read up to ok/err (console
/// protocol) or a 2.5 s timeout.
fn cmd_proxy(rest: &str) -> String {
    let (target, cmd) = match rest.split_once('/') {
        Some(t) => t,
        None => return "err usage /cmd/<rx|tx|a|b>/<cmd>".into(),
    };
    let addr = match target {
        "rx" => "127.0.0.1:7203",
        "tx" => "127.0.0.1:7204",
        "a" => "192.168.0.10:7202",
        "b" => "192.168.0.11:7202",
        _ => return "err target rx|tx|a|b".into(),
    };
    let cmd = pct_decode(cmd);
    let mut s = match TcpStream::connect_timeout(
        &addr.parse().unwrap(),
        Duration::from_secs(2),
    ) {
        Ok(s) => s,
        Err(e) => return format!("err connect {addr}: {e}"),
    };
    let _ = s.set_read_timeout(Some(Duration::from_millis(2500)));
    if writeln!(s, "{cmd}").is_err() {
        return "err write".into();
    }
    let mut buf = Vec::new();
    let mut tmp = [0u8; 4096];
    loop {
        match s.read(&mut tmp) {
            Ok(0) => break,
            Ok(n) => {
                buf.extend_from_slice(&tmp[..n]);
                let t = String::from_utf8_lossy(&buf);
                if t.lines().any(|l| l.trim() == "ok" || l.starts_with("err")) {
                    break;
                }
            }
            Err(_) => break,
        }
    }
    String::from_utf8_lossy(&buf).into_owned()
}

fn pct_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'%' => {
                let h = |c: u8| -> Option<u8> {
                    match c {
                        b'0'..=b'9' => Some(c - b'0'),
                        b'a'..=b'f' => Some(c - b'a' + 10),
                        b'A'..=b'F' => Some(c - b'A' + 10),
                        _ => None,
                    }
                };
                if i + 2 < b.len() {
                    if let (Some(x), Some(y)) = (h(b[i + 1]), h(b[i + 2])) {
                        out.push(x * 16 + y);
                        i += 3;
                        continue;
                    }
                }
                out.push(b'%');
                i += 1;
            }
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            c => {
                out.push(c);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}
