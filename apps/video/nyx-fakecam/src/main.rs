//! nyx-fakecam: a stand-in IP camera on the PC.
//!
//! Behaves like the cameras nyx-ipcam is meant for: H.264 at a fixed bitrate with a key frame
//! every `--gop` frames, served over RTSP with RTP interleaved on the RTSP TCP connection
//! (RFC 2326 section 10.12; UDP is refused with 461 and clients fall back). It does not answer
//! a receiver's wish for a key frame; only a client starting to play gets one, as most cameras
//! do.
//!
//!   nyx-fakecam [--port 8554] [--source webcam|pattern] [--res 640x480] [--fps 25]
//!               [--kbps 1000] [--gop 50] [--stamp] [--noise 0..100] [--board 192.168.0.10]
//!               [--onvif-port 80] [--user admin] [--pass <password>] [--http-digest]
//!
//! The URL is rtsp://<this PC>:<port>/cam (any path works). `--stamp` burns the millisecond
//! counter into the picture (the ground app's latency test). `--noise` adds random grain so the
//! encoder really spends the bitrate asked for (a still webcam or the pattern stays far below
//! it). `--board` is only used to print the address the board should use.
//!
//! ONVIF (onvif_srv.rs) on `--onvif-port` (0 = none): what the board's client needs to find the
//! stream's profile and change its bitrate limit, which the encoder follows at once. With `--pass`
//! the calls must be signed (WS-Security, or HTTP Digest with `--http-digest`); the RTSP stream
//! itself asks for nothing.

mod onvif_srv;

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, TrySendError, sync_channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nyx_common::Opts;
use nyx_common::codec::VideoEncoder;
use nyx_common::logging::{self, log};
use nyx_common::source::{PatternGen, WebcamShared, resize_rgb};

/// RTP payload per packet; larger NAL units are cut into FU-A fragments.
const RTP_MTU: usize = 1400;
/// Access units a client may fall behind before it starts losing them.
const CLIENT_QUEUE: usize = 60;

struct Au {
    /// Annex B, as the encoder wrote it
    data: Vec<u8>,
    key: bool,
    ts90k: u32,
}

struct Client {
    tx: SyncSender<Arc<Au>>,
    /// an access unit could not be queued: skip to the next key frame
    broken: Arc<AtomicBool>,
}

struct Cam {
    clients: Mutex<Vec<Client>>,
    sps: Mutex<Vec<u8>>,
    pps: Mutex<Vec<u8>>,
    want_idr: AtomicBool,
    t0: Instant,
    ts_base: u32,
    fps: f32,
    ssrc: u32,
    session_ctr: AtomicU32,
}

impl Cam {
    fn ts_now(&self) -> u32 {
        self.ts_base.wrapping_add((self.t0.elapsed().as_secs_f64() * 90_000.0) as u32)
    }
}

fn main() {
    logging::init("fakecam");
    let opts = Opts::from_env();
    let port: u16 = opts.arg("--port", "8554").parse().unwrap_or(8554);
    let (w, h) = opts
        .arg("--res", "640x480")
        .split_once(['x', 'X'])
        .and_then(|(a, b)| Some((a.trim().parse::<usize>().ok()?, b.trim().parse::<usize>().ok()?)))
        .map(|(a, b)| (a & !1, b & !1))
        .unwrap_or((640, 480));
    let fps: f32 = opts.arg("--fps", "25").parse().unwrap_or(25.0f32).clamp(1.0, 60.0);
    let kbps: u32 = opts.arg("--kbps", "1000").parse().unwrap_or(1000).clamp(50, 20_000);
    let gop: u32 = opts.arg("--gop", "50").parse().unwrap_or(50).max(1);
    let webcam = opts.arg("--source", "webcam") != "pattern";
    let stamp = opts.flag("--stamp");
    let noise: u8 = opts.arg("--noise", "0").parse().unwrap_or(0u8).min(100);
    let onvif_port: u16 = opts.arg("--onvif-port", "80").parse().unwrap_or(80);
    let user = opts.arg("--user", "admin");
    let pass = opts.arg("--pass", "");

    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .subsec_nanos();
    let cam = Arc::new(Cam {
        clients: Mutex::new(Vec::new()),
        sps: Mutex::new(Vec::new()),
        pps: Mutex::new(Vec::new()),
        want_idr: AtomicBool::new(false),
        t0: Instant::now(),
        ts_base: seed.wrapping_mul(2_654_435_761),
        fps,
        ssrc: seed ^ 0x4E59_4843,
        session_ctr: AtomicU32::new(seed & 0xFFFF),
    });

    let listener = match TcpListener::bind(("0.0.0.0", port)) {
        Ok(l) => l,
        Err(e) => {
            log(&format!("fakecam: cannot listen on port {port}: {e}"));
            std::process::exit(1);
        }
    };
    let board = opts.arg("--board", "192.168.0.10");
    let here = UdpSocket::bind("0.0.0.0:0")
        .and_then(|s| s.connect((board.as_str(), 9)).map(|_| s))
        .and_then(|s| s.local_addr())
        .map(|a| a.ip().to_string())
        .unwrap_or_else(|_| "<this PC>".into());
    log(&format!(
        "fakecam: {w}x{h} {fps} fps {kbps} kbps, key frame every {gop}, source {}{}",
        if webcam { "webcam (test pattern until it opens)" } else { "test pattern" },
        if stamp { ", millisecond counter in the picture" } else { "" },
    ));
    let cred = if pass.is_empty() { String::new() } else { format!("{user}:{pass}@") };
    log(&format!("fakecam: rtsp://{cred}{here}:{port}/cam  (the address {board} reaches this PC on)"));
    let ostate = onvif_srv::State::new(kbps, w as u32, h as u32, fps, gop, port, user.clone(), pass.clone(), opts.flag("--http-digest"));
    if onvif_port != 0 {
        match onvif_srv::spawn(ostate.clone(), onvif_port) {
            Some(p) => log(&format!(
                "fakecam: onvif http://{here}{}/onvif/device_service{}",
                if p == 80 { String::new() } else { format!(":{p}") },
                if pass.is_empty() { " (no authentication)".to_string() } else if opts.flag("--http-digest") { format!(" (user {user}, HTTP Digest)") } else { format!(" (user {user}, WS-Security)") }
            )),
            None => log("fakecam: onvif off (no port)"),
        }
    }

    {
        let cam = cam.clone();
        std::thread::Builder::new()
            .name("encoder".into())
            .spawn(move || encoder_loop(cam, w, h, fps, ostate, gop, webcam, stamp, noise))
            .expect("spawn encoder");
    }
    for conn in listener.incoming() {
        let Ok(stream) = conn else { continue };
        let cam = cam.clone();
        std::thread::spawn(move || {
            let peer = stream.peer_addr().map(|a| a.to_string()).unwrap_or_default();
            log(&format!("fakecam: {peer} connected"));
            let why = serve(stream, &cam, port).err().map(|e| e.to_string()).unwrap_or_else(|| "closed".into());
            log(&format!("fakecam: {peer} gone ({why})"));
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn encoder_loop(cam: Arc<Cam>, w: usize, h: usize, fps: f32, ostate: Arc<onvif_srv::State>, gop: u32, webcam: bool, stamp: bool, noise: u8) {
    let mut kbps = ostate.kbps.load(Ordering::Relaxed);
    let mut rng: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut enc = match VideoEncoder::new_with_intra(w, h, kbps * 1000, fps, gop) {
        Ok(e) => e,
        Err(e) => {
            log(&format!("fakecam: encoder: {e}"));
            std::process::exit(1);
        }
    };
    let cams = WebcamShared::new();
    if webcam {
        cams.wanted.store(true, Ordering::Relaxed);
        nyx_common::source::webcam::spawn(cams.clone());
    }
    let mut pattern = PatternGen::new();
    let period = Duration::from_secs_f32(1.0 / fps);
    let mut next = Instant::now();
    let mut on_pattern = true;
    let (mut n, mut bytes, mut keys) = (0u32, 0u64, 0u32);
    let mut t_rep = Instant::now();
    loop {
        let now = Instant::now();
        if now < next {
            std::thread::sleep(next - now);
        }
        next += period;
        if Instant::now() > next + period {
            next = Instant::now();
        }
        let shot = if webcam { cams.frame.lock().unwrap().clone() } else { None };
        let mut f = match shot {
            Some(f) => {
                if on_pattern {
                    on_pattern = false;
                    log(&format!("fakecam: webcam picture {}x{} -> {w}x{h}", f.width, f.height));
                }
                resize_rgb(&f, w, h)
            }
            None => pattern.render(w, h),
        };
        if noise > 0 {
            let amp = i32::from(noise);
            for px in f.rgb.iter_mut() {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                let d = (rng % (2 * amp as u64 + 1)) as i32 - amp;
                *px = (i32::from(*px) + d).clamp(0, 255) as u8;
            }
        }
        if stamp {
            nyx_common::stamp::burn(&mut f, nyx_common::stamp::counter_ms());
        }
        let ts = cam.ts_now();
        let idr = cam.want_idr.swap(false, Ordering::Relaxed);
        // an ONVIF client changed the bitrate limit
        let want = ostate.kbps.load(Ordering::Relaxed);
        if want != kbps {
            enc.set_bitrate(want * 1000);
            kbps = want;
        }
        let data = enc.encode(&f, idr);
        if data.is_empty() {
            continue;
        }
        let mut key = false;
        for nal in nal_units(&data) {
            match nal[0] & 0x1F {
                5 => key = true,
                7 => *cam.sps.lock().unwrap() = nal.to_vec(),
                8 => *cam.pps.lock().unwrap() = nal.to_vec(),
                _ => {}
            }
        }
        n += 1;
        bytes += data.len() as u64;
        keys += u32::from(key);
        let au = Arc::new(Au { data, key, ts90k: ts });
        let clients = {
            let mut cl = cam.clients.lock().unwrap();
            cl.retain(|c| match c.tx.try_send(au.clone()) {
                Ok(()) => true,
                Err(TrySendError::Full(_)) => {
                    c.broken.store(true, Ordering::Relaxed);
                    true
                }
                Err(TrySendError::Disconnected(_)) => false,
            });
            cl.len()
        };
        let dt = t_rep.elapsed();
        if dt >= Duration::from_secs(5) {
            log(&format!(
                "fakecam: {:.1} fps {:.0} kbps, {keys} key frames, {clients} client(s), source {}",
                n as f64 / dt.as_secs_f64(),
                bytes as f64 * 8.0 / dt.as_secs_f64() / 1000.0,
                if on_pattern { "pattern" } else { "webcam" },
            ));
            (n, bytes, keys) = (0, 0, 0);
            t_rep = Instant::now();
        }
    }
}

/// NAL units of an Annex B buffer (3- or 4-byte start codes), without the start codes.
fn nal_units(data: &[u8]) -> Vec<&[u8]> {
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 3 <= data.len() {
        if data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 {
            starts.push(i + 3);
            i += 3;
        } else {
            i += 1;
        }
    }
    let mut out = Vec::with_capacity(starts.len());
    for (k, &s) in starts.iter().enumerate() {
        let mut e = if k + 1 < starts.len() { starts[k + 1] - 3 } else { data.len() };
        // a 4-byte start code leaves its leading zero on the unit before
        while e > s && data[e - 1] == 0 && k + 1 < starts.len() {
            e -= 1;
        }
        if e > s {
            out.push(&data[s..e]);
        }
    }
    out
}

fn base64(data: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let b = [c[0], *c.get(1).unwrap_or(&0), *c.get(2).unwrap_or(&0)];
        let v = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        for k in 0..4 {
            if k <= c.len() {
                out.push(T[((v >> (18 - 6 * k)) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

struct Request {
    method: String,
    url: String,
    headers: Vec<(String, String)>,
}

impl Request {
    fn header(&self, name: &str) -> Option<&str> {
        self.headers.iter().find(|(k, _)| k.eq_ignore_ascii_case(name)).map(|(_, v)| v.as_str())
    }
}

/// The next RTSP request; binary interleaved frames from the client (its RTCP reports) are
/// read and thrown away on the way.
fn read_request(rd: &mut BufReader<TcpStream>) -> std::io::Result<Option<Request>> {
    loop {
        let buf = rd.fill_buf()?;
        if buf.is_empty() {
            return Ok(None);
        }
        if buf[0] == b'$' {
            let mut hdr = [0u8; 4];
            rd.read_exact(&mut hdr)?;
            let len = usize::from(u16::from_be_bytes([hdr[2], hdr[3]]));
            let mut skip = vec![0u8; len];
            rd.read_exact(&mut skip)?;
            continue;
        }
        let mut line = String::new();
        if rd.read_line(&mut line)? == 0 {
            return Ok(None);
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_whitespace();
        let method = parts.next().unwrap_or("").to_string();
        let url = parts.next().unwrap_or("").to_string();
        let mut headers = Vec::new();
        loop {
            let mut h = String::new();
            if rd.read_line(&mut h)? == 0 {
                return Ok(None);
            }
            let h = h.trim_end();
            if h.is_empty() {
                break;
            }
            if let Some((k, v)) = h.split_once(':') {
                headers.push((k.trim().to_string(), v.trim().to_string()));
            }
        }
        let body = headers
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case("Content-Length"))
            .and_then(|(_, v)| v.parse::<usize>().ok())
            .unwrap_or(0);
        if body > 0 {
            let mut skip = vec![0u8; body.min(65_536)];
            rd.read_exact(&mut skip)?;
        }
        return Ok(Some(Request { method, url, headers }));
    }
}

fn reply(w: &Mutex<TcpStream>, status: &str, cseq: &str, extra: &[String], body: &str) -> std::io::Result<()> {
    let mut msg = format!("RTSP/1.0 {status}\r\nCSeq: {cseq}\r\nServer: nyx-fakecam\r\n");
    for e in extra {
        msg.push_str(e);
        msg.push_str("\r\n");
    }
    if !body.is_empty() {
        msg.push_str(&format!("Content-Length: {}\r\n", body.len()));
    }
    msg.push_str("\r\n");
    msg.push_str(body);
    w.lock().unwrap().write_all(msg.as_bytes())
}

fn serve(stream: TcpStream, cam: &Arc<Cam>, port: u16) -> std::io::Result<()> {
    let _ = stream.set_nodelay(true);
    let w = Arc::new(Mutex::new(stream.try_clone()?));
    let mut rd = BufReader::new(stream);
    let session = format!("{:08X}", cam.session_ctr.fetch_add(1, Ordering::Relaxed).wrapping_mul(2_246_822_519));
    let mut channel = 0u8;
    let mut track_url = String::new();
    let stop = Arc::new(AtomicBool::new(false));
    let mut sender: Option<std::thread::JoinHandle<()>> = None;
    let result = loop {
        let req = match read_request(&mut rd) {
            Ok(Some(r)) => r,
            Ok(None) => break Ok(()),
            Err(e) => break Err(e),
        };
        let cseq = req.header("CSeq").unwrap_or("0").to_string();
        let sess = format!("Session: {session};timeout=60");
        let r = match req.method.as_str() {
            "OPTIONS" => reply(&w, "200 OK", &cseq, &["Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER, SET_PARAMETER".into()], ""),
            "DESCRIBE" => {
                // the parameter sets come with the first key frame
                let t = Instant::now();
                while cam.sps.lock().unwrap().is_empty() && t.elapsed() < Duration::from_secs(5) {
                    cam.want_idr.store(true, Ordering::Relaxed);
                    std::thread::sleep(Duration::from_millis(50));
                }
                let sps = cam.sps.lock().unwrap().clone();
                let pps = cam.pps.lock().unwrap().clone();
                if sps.len() < 4 || pps.is_empty() {
                    reply(&w, "503 Service Unavailable", &cseq, &[], "")
                } else {
                    let base = if req.url.ends_with('/') { req.url.clone() } else { format!("{}/", req.url) };
                    let host = w.lock().unwrap().local_addr().map(|a| a.ip().to_string()).unwrap_or_else(|_| "0.0.0.0".into());
                    let sdp = format!(
                        "v=0\r\no=- {sid} 1 IN IP4 {host}\r\ns=NyxHop fake camera\r\nc=IN IP4 0.0.0.0\r\nt=0 0\r\n\
                         a=control:*\r\na=range:npt=0-\r\nm=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\n\
                         a=fmtp:96 packetization-mode=1;profile-level-id={:02X}{:02X}{:02X};sprop-parameter-sets={},{}\r\n\
                         a=framerate:{}\r\na=control:trackID=0\r\n",
                        sps[1], sps[2], sps[3], base64(&sps), base64(&pps), cam.fps,
                        sid = cam.ssrc,
                    );
                    reply(&w, "200 OK", &cseq, &["Content-Type: application/sdp".into(), format!("Content-Base: {base}")], &sdp)
                }
            }
            "SETUP" => {
                let tr = req.header("Transport").unwrap_or("").to_string();
                if !tr.contains("TCP") && !tr.contains("interleaved") {
                    reply(&w, "461 Unsupported Transport", &cseq, &[], "")
                } else {
                    channel = tr
                        .split(';')
                        .find_map(|p| p.trim().strip_prefix("interleaved="))
                        .and_then(|v| v.split('-').next())
                        .and_then(|v| v.parse::<u8>().ok())
                        .unwrap_or(0);
                    track_url = req.url.clone();
                    reply(&w, "200 OK", &cseq, &[
                        format!("Transport: RTP/AVP/TCP;unicast;interleaved={}-{};ssrc={:08X};mode=\"PLAY\"", channel, channel.wrapping_add(1), cam.ssrc),
                        sess,
                    ], "")
                }
            }
            "PLAY" => {
                let seq0 = (cam.ssrc & 0x7FFF) as u16;
                let ts0 = cam.ts_now();
                let rr = reply(&w, "200 OK", &cseq, &[
                    sess,
                    "Range: npt=0.000-".into(),
                    format!("RTP-Info: url={};seq={seq0};rtptime={ts0}", if track_url.is_empty() { &req.url } else { &track_url }),
                ], "");
                if rr.is_ok() && sender.is_none() {
                    let (tx, rx) = sync_channel::<Arc<Au>>(CLIENT_QUEUE);
                    let broken = Arc::new(AtomicBool::new(false));
                    cam.clients.lock().unwrap().push(Client { tx, broken: broken.clone() });
                    cam.want_idr.store(true, Ordering::Relaxed);
                    let (w2, stop2, cam2) = (w.clone(), stop.clone(), cam.clone());
                    let ch = channel;
                    sender = Some(std::thread::spawn(move || send_rtp(&w2, rx, &broken, &stop2, &cam2, ch, seq0)));
                    log(&format!("fakecam: playing (port {port}, channel {channel})"));
                }
                rr
            }
            "TEARDOWN" => {
                let _ = reply(&w, "200 OK", &cseq, &[sess], "");
                break Ok(());
            }
            "GET_PARAMETER" | "SET_PARAMETER" => reply(&w, "200 OK", &cseq, &[sess], ""),
            _ => reply(&w, "501 Not Implemented", &cseq, &[], ""),
        };
        if let Err(e) = r {
            break Err(e);
        }
    };
    stop.store(true, Ordering::Relaxed);
    if let Some(s) = sender {
        let _ = s.join();
    }
    result
}

/// RTP (RFC 6184: single NAL unit packets, FU-A for the big ones) in interleaved frames.
fn send_rtp(w: &Mutex<TcpStream>, rx: Receiver<Arc<Au>>, broken: &AtomicBool, stop: &AtomicBool, cam: &Cam, ch: u8, seq0: u16) {
    let mut seq = seq0;
    let mut wait_key = true;
    let mut out = Vec::with_capacity(64 * 1024);
    while !stop.load(Ordering::Relaxed) {
        let au = match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(a) => a,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        if broken.swap(false, Ordering::Relaxed) {
            wait_key = true;
        }
        if wait_key && !au.key {
            continue;
        }
        wait_key = false;
        out.clear();
        let nals = nal_units(&au.data);
        let last = nals.len().saturating_sub(1);
        for (k, nal) in nals.iter().enumerate() {
            if nal.len() <= RTP_MTU {
                put_rtp(&mut out, ch, cam, &mut seq, au.ts90k, k == last, &[nal]);
            } else {
                let ind = (nal[0] & 0xE0) | 28;
                let typ = nal[0] & 0x1F;
                let body = &nal[1..];
                let chunks: Vec<&[u8]> = body.chunks(RTP_MTU - 2).collect();
                for (j, c) in chunks.iter().enumerate() {
                    let mut fu = typ;
                    if j == 0 {
                        fu |= 0x80;
                    }
                    if j + 1 == chunks.len() {
                        fu |= 0x40;
                    }
                    put_rtp(&mut out, ch, cam, &mut seq, au.ts90k, k == last && j + 1 == chunks.len(), &[&[ind, fu], c]);
                }
            }
        }
        if w.lock().unwrap().write_all(&out).is_err() {
            return;
        }
    }
}

fn put_rtp(out: &mut Vec<u8>, ch: u8, cam: &Cam, seq: &mut u16, ts: u32, marker: bool, parts: &[&[u8]]) {
    let len = 12 + parts.iter().map(|p| p.len()).sum::<usize>();
    out.extend_from_slice(&[b'$', ch, (len >> 8) as u8, len as u8]);
    out.push(0x80);
    out.push(96 | if marker { 0x80 } else { 0 });
    out.extend_from_slice(&seq.to_be_bytes());
    out.extend_from_slice(&ts.to_be_bytes());
    out.extend_from_slice(&cam.ssrc.to_be_bytes());
    for p in parts {
        out.extend_from_slice(p);
    }
    *seq = seq.wrapping_add(1);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_matches_rfc4648() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn nal_units_split_both_start_codes() {
        let d = [0, 0, 0, 1, 0x67, 1, 2, 0, 0, 1, 0x68, 3, 0, 0, 0, 1, 0x65, 4, 5];
        let n = nal_units(&d);
        assert_eq!(n, vec![&[0x67u8, 1, 2][..], &[0x68, 3][..], &[0x65, 4, 5][..]]);
    }
}
