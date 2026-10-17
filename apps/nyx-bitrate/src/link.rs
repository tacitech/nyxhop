//! The two boards through their consoles (TCP 7202), and the measurement: test packets the
//! aircraft board sends (`o64 blast`), counted by the ground board (`o64 meter`).
//!
//! A meter thread reads the ground's counts four times a second, always (the received rate, the
//! video's, the SNR); a run thread sends the test packets and changes the link for it (a fixed
//! channel and width while it measures: Auto channel and Auto width would read the test packets'
//! silence of video as a lost link) and puts everything back after.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nyx_common::i18n::t;
use nyx_common::tf;

pub const PORT: u16 = 7202;

/// The PHY's rates at 20 MHz, Mbit/s, MCS 0..7 (802.11a's).
pub const PHY_MBPS: [f64; 8] = [6.0, 9.0, 12.0, 18.0, 24.0, 36.0, 48.0, 54.0];
pub const MCS_NAME: [&str; 8] = ["BPSK 1/2", "BPSK 3/4", "QPSK 1/2", "QPSK 3/4", "16QAM 1/2", "16QAM 3/4", "64QAM 2/3", "64QAM 3/4"];
/// Channel widths as divisors of 20 MHz.
pub const DIVS: [u64; 4] = [1, 2, 4, 8];

pub fn width_mhz(div: u64) -> &'static str {
    match div {
        2 => "10",
        4 => "5",
        8 => "2.5",
        _ => "20",
    }
}

/// One console command: the reply up to its closing "ok" (Err: the "err ..." line, or no answer).
pub fn cmd(host: &str, line: &str, timeout: Duration) -> Result<String, String> {
    let addr = (host, PORT).to_socket_addrs().map_err(|e| e.to_string())?.next().ok_or("no address")?;
    let mut s = TcpStream::connect_timeout(&addr, Duration::from_millis(1500)).map_err(|e| e.to_string())?;
    let _ = s.set_read_timeout(Some(timeout));
    let _ = s.set_nodelay(true);
    s.write_all(format!("{line}\n").as_bytes()).map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    let mut buf = [0u8; 8192];
    loop {
        match s.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                out.extend_from_slice(&buf[..n]);
                let t = String::from_utf8_lossy(&out);
                let last = t.trim_end().lines().last().unwrap_or("").trim();
                if last == "ok" || last.starts_with("err") {
                    break;
                }
            }
            Err(e) => {
                if out.is_empty() {
                    return Err(e.to_string());
                }
                break;
            }
        }
    }
    let t = String::from_utf8_lossy(&out).to_string();
    if let Some(e) = t.lines().find(|l| l.trim_start().starts_with("err")) {
        return Err(e.trim().trim_start_matches("err").trim().to_string());
    }
    Ok(t)
}

/// `key=value` out of a reply (space or line separated).
pub fn kv<'a>(t: &'a str, key: &str) -> Option<&'a str> {
    t.split_whitespace().find_map(|w| w.strip_prefix(key)?.strip_prefix('='))
}

fn kv_u64(t: &str, key: &str) -> Option<u64> {
    kv(t, key)?.parse().ok()
}

fn kv_f32(t: &str, key: &str) -> Option<f32> {
    kv(t, key)?.parse().ok()
}

/// The ground's counts at one moment (`o64 meter`).
#[derive(Clone, Copy, Default)]
pub struct Meter {
    pub test_ok: u64,
    pub test_bytes: u64,
    pub video_ok: u64,
    pub video_bytes: u64,
    pub bad: u64,
    pub snr_test: Option<f32>,
    pub snr_video: Option<f32>,
    /// every packet's SNR whose SIG decoded, failed ones too (the test packets' own only counts
    /// those that passed: at an MCS the link cannot carry it stood still)
    pub snr_sig: Option<f32>,
    /// the receiver's own count of packets decoded (the FPGA's, before the DMA to the board's
    /// processor); it restarts with a width change
    pub pl_ok: u32,
    /// the video the receiver keeps up with, kbit/s, when it decodes on its processor (0: all
    /// the air brings): the test packets start there
    pub cap_kbps: u64,
    /// the packets such a receiver had no time for (None: a daemon from before the count)
    pub rx_busy: Option<u64>,
    pub at: Option<Instant>,
}

pub fn meter(host: &str) -> Result<Meter, String> {
    let t = cmd(host, "o64 meter", Duration::from_secs(2))?;
    if !t.contains("meter ") {
        return Err("no meter".into());
    }
    Ok(Meter {
        test_ok: kv_u64(&t, "test_ok").unwrap_or(0),
        test_bytes: kv_u64(&t, "test_bytes").unwrap_or(0),
        video_ok: kv_u64(&t, "video_ok").unwrap_or(0),
        video_bytes: kv_u64(&t, "video_bytes").unwrap_or(0),
        bad: kv_u64(&t, "bad").unwrap_or(0),
        snr_test: kv_f32(&t, "snr_test"),
        snr_video: kv_f32(&t, "snr_video"),
        snr_sig: kv_f32(&t, "snr_sig"),
        pl_ok: kv_u64(&t, "pl_ok").unwrap_or(0) as u32,
        cap_kbps: kv_u64(&t, "cap_kbps").unwrap_or(0),
        rx_busy: kv_u64(&t, "rx_busy"),
        at: Some(Instant::now()),
    })
}

/// A quarter second of the link, as the plots draw it.
#[derive(Clone, Copy, Default)]
pub struct Sample {
    /// seconds since the program started
    pub t: f64,
    /// test packets received by the board's software, Mbit/s; decoded by its receiver (the
    /// FPGA, before the DMA); the video's; what the aircraft sent of test packets (over 2 s)
    pub rx: f64,
    pub phy: f64,
    pub video: f64,
    pub tx: f64,
    /// test packets lost over the last two seconds: all of it (sent against received), on the
    /// air (sent against decoded) and in the receiving board (decoded against received), 0..1
    pub loss: Option<f64>,
    pub air_loss: Option<f64>,
    pub board_loss: Option<f64>,
    /// the video's blocks failed over the last two seconds, 0..1
    pub vloss: Option<f64>,
    pub snr: Option<f32>,
}

/// One cell of a sweep: a width and an MCS.
#[derive(Clone, Copy)]
pub struct Cell {
    pub div: u64,
    pub mcs: usize,
    pub rx_mbps: f64,
    /// decoded by the receiver (before the DMA to the processor)
    pub phy_mbps: f64,
    pub loss: f64,
    pub air_loss: f64,
    pub snr: Option<f32>,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Job {
    /// test packets at one MCS, packet size and width until stopped; on `chan` MHz (0: the
    /// channel the link is on)
    Test { mcs: usize, bytes: usize, div: u64, chan: f64 },
    /// every MCS at every width asked for, `secs` each
    Sweep { divs: [bool; 4], bytes: usize, secs: f64, chan: f64 },
}

#[derive(Default)]
pub struct Shared {
    pub tx_host: String,
    pub rx_host: String,
    pub samples: VecDeque<Sample>,
    pub tx_ok: bool,
    pub rx_ok: bool,
    /// the ground has `o64 meter` (a daemon from before it does not)
    pub meter_ok: bool,
    pub width_div: u64,
    pub tx_div: u64,
    pub chan_mhz: f64,
    /// the ground's channel table, MHz
    pub table: Vec<f64>,
    pub hop_mode: String,
    pub auto_bw: bool,
    pub running: Option<Job>,
    pub phase: String,
    pub cells: Vec<Cell>,
    pub msg: String,
}

pub struct Link {
    pub sh: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    /// test packets the run thread had sent, and their bytes (the meter thread's offered rate and
    /// loss)
    sent: Arc<(AtomicU64, AtomicU64)>,
    t0: Instant,
}

impl Link {
    pub fn new(tx_host: String, rx_host: String) -> Link {
        let sh = Arc::new(Mutex::new(Shared { tx_host, rx_host, width_div: 1, tx_div: 1, ..Default::default() }));
        let l = Link { sh, stop: Arc::new(AtomicBool::new(false)), sent: Arc::new((AtomicU64::new(0), AtomicU64::new(0))), t0: Instant::now() };
        l.spawn_meter();
        l
    }

    pub fn now(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }

    fn spawn_meter(&self) {
        let (sh, sent, t0) = (self.sh.clone(), self.sent.clone(), self.t0);
        std::thread::Builder::new()
            .name("meter".into())
            .spawn(move || {
                let mut last: Option<Meter> = None;
                let mut last_sent = (0u64, 0u64);
                // (recv, sent) of the last two seconds, for the loss
                // (received, sent, decoded, sent bytes, seconds) of the last two seconds
                let mut win: VecDeque<(u64, u64, u64, u64, f64)> = VecDeque::new();
                let mut vwin: VecDeque<(u64, u64)> = VecDeque::new();
                let mut pkt_bytes = 2016.0f64;
                let mut status_at = Instant::now() - Duration::from_secs(10);
                loop {
                    std::thread::sleep(Duration::from_millis(250));
                    let (tx, rx) = {
                        let s = sh.lock().unwrap();
                        (s.tx_host.clone(), s.rx_host.clone())
                    };
                    // the link's settings every two seconds (both ends' width, the channel)
                    if status_at.elapsed() >= Duration::from_secs(2) {
                        status_at = Instant::now();
                        let g = cmd(&rx, "chbw", Duration::from_secs(2));
                        let h = cmd(&rx, "hop status", Duration::from_secs(2));
                        let a = cmd(&tx, "chbw", Duration::from_secs(2));
                        let mut s = sh.lock().unwrap();
                        s.rx_ok = g.is_ok() || h.is_ok();
                        s.tx_ok = a.is_ok();
                        if let Ok(g) = &g {
                            s.width_div = kv_u64(g, "chbw_div").unwrap_or(1);
                            s.auto_bw = kv(g, "chbw_auto") == Some("1");
                        }
                        if let Ok(a) = &a {
                            s.tx_div = kv_u64(a, "chbw_div").unwrap_or(1);
                        }
                        if let Ok(h) = &h {
                            s.hop_mode = kv(h, "hop_mode").unwrap_or("").to_string();
                            s.chan_mhz = kv_u64(h, "hop_cur_hz").map_or(0.0, |hz| hz as f64 / 1e6);
                            if let Some(t) = kv(h, "hop_table") {
                                s.table = t.split(',').filter_map(|v| v.trim().parse::<f64>().ok()).filter(|v| *v > 0.0).collect();
                            }
                        }
                    }
                    let m = match meter(&rx) {
                        Ok(m) => m,
                        Err(e) => {
                            let mut s = sh.lock().unwrap();
                            s.meter_ok = false;
                            if e != "no meter" {
                                s.rx_ok = false;
                            }
                            last = None;
                            continue;
                        }
                    };
                    let snt = (sent.0.load(Ordering::Relaxed), sent.1.load(Ordering::Relaxed));
                    if let (Some(p), Some(at), Some(pat)) = (last, m.at, last.and_then(|p| p.at)) {
                        let dt = at.duration_since(pat).as_secs_f64().max(0.05);
                        let d_ok = m.test_ok.saturating_sub(p.test_ok);
                        let d_sent = snt.0.saturating_sub(last_sent.0);
                        // (the receiver's counters restart with a width change: no step back)
                        let d_pl = if m.pl_ok >= p.pl_ok { u64::from(m.pl_ok - p.pl_ok) } else { 0 };
                        if d_ok > 0 {
                            pkt_bytes = m.test_bytes.saturating_sub(p.test_bytes) as f64 / d_ok as f64;
                        }
                        win.push_back((d_ok, d_sent, d_pl, snt.1.saturating_sub(last_sent.1), dt));
                        while win.len() > 8 {
                            win.pop_front();
                        }
                        let (r, sn, dec, sb, secs) =
                            win.iter().fold((0u64, 0u64, 0u64, 0u64, 0.0f64), |a, w| (a.0 + w.0, a.1 + w.1, a.2 + w.2, a.3 + w.3, a.4 + w.4));
                        vwin.push_back((m.video_ok.saturating_sub(p.video_ok), m.bad.saturating_sub(p.bad)));
                        while vwin.len() > 8 {
                            vwin.pop_front();
                        }
                        let (vo, vb) = vwin.iter().fold((0u64, 0u64), |a, w| (a.0 + w.0, a.1 + w.1));
                        let smp = Sample {
                            t: t0.elapsed().as_secs_f64(),
                            rx: m.test_bytes.saturating_sub(p.test_bytes) as f64 * 8.0 / dt / 1e6,
                            phy: if sn > 0 { d_pl as f64 * pkt_bytes * 8.0 / dt / 1e6 } else { 0.0 },
                            video: m.video_bytes.saturating_sub(p.video_bytes) as f64 * 8.0 / dt / 1e6,
                            tx: sb as f64 * 8.0 / secs.max(0.05) / 1e6,
                            loss: (sn >= 4).then(|| (1.0 - r as f64 / sn as f64).clamp(0.0, 1.0)),
                            air_loss: (sn >= 4).then(|| (1.0 - dec as f64 / sn as f64).clamp(0.0, 1.0)),
                            board_loss: (dec >= 4).then(|| (1.0 - r as f64 / dec as f64).clamp(0.0, 1.0)),
                            vloss: (vo + vb >= 4).then(|| vb as f64 / (vo + vb) as f64),
                            snr: m.snr_sig.or(m.snr_test).or(m.snr_video),
                        };
                        let mut s = sh.lock().unwrap();
                        s.meter_ok = true;
                        s.rx_ok = true;
                        s.samples.push_back(smp);
                        while s.samples.len() > 4 * 300 {
                            s.samples.pop_front();
                        }
                    }
                    last = Some(m);
                    last_sent = snt;
                }
            })
            .expect("spawn meter");
    }

    pub fn running(&self) -> bool {
        self.sh.lock().unwrap().running.is_some()
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    pub fn start(&self, job: Job) {
        if self.running() {
            return;
        }
        self.stop.store(false, Ordering::Relaxed);
        {
            let mut s = self.sh.lock().unwrap();
            s.running = Some(job);
            s.msg.clear();
            if matches!(job, Job::Sweep { .. }) {
                s.cells.clear();
            }
        }
        let (sh, stop, sent) = (self.sh.clone(), self.stop.clone(), self.sent.clone());
        std::thread::Builder::new()
            .name("run".into())
            .spawn(move || {
                let r = run(&sh, &stop, &sent, job);
                let mut s = sh.lock().unwrap();
                if let Err(e) = r {
                    s.msg = e;
                }
                s.running = None;
                s.phase.clear();
            })
            .expect("spawn run");
    }
}

fn phase(sh: &Mutex<Shared>, p: String) {
    sh.lock().unwrap().phase = p;
}

/// What the ground had before a run, to put back.
struct Saved {
    hop_auto: bool,
    chan_mhz: f64,
    bw_auto: bool,
    div: u64,
}

fn prepare(sh: &Mutex<Shared>, rx: &str, chan: f64) -> Result<Saved, String> {
    let h = cmd(rx, "hop status", Duration::from_secs(3)).map_err(|e| tf!("receiver: {}", "máy thu: {}", e))?;
    let g = cmd(rx, "chbw", Duration::from_secs(3)).map_err(|e| tf!("receiver: {}", "máy thu: {}", e))?;
    let saved = Saved {
        hop_auto: kv(&h, "hop_mode").is_some_and(|m| m != "manual"),
        chan_mhz: kv_u64(&h, "hop_cur_hz").map_or(0.0, |hz| hz as f64 / 1e6),
        bw_auto: kv(&g, "chbw_auto") == Some("1"),
        div: kv_u64(&g, "chbw_div").unwrap_or(1),
    };
    // one channel and one width while measuring (Auto channel / Auto width would read the test
    // packets' lack of video as a lost link): the one asked for, or the one the link is on (Auto
    // may be on a channel WiFi fills: 64QAM did not decode there, 14/10)
    let hold = if chan > 0.0 { chan } else { saved.chan_mhz };
    if hold > 0.0 && (saved.hop_auto || (hold - saved.chan_mhz).abs() > 0.01) {
        phase(sh, tf!("holding the channel {:.1} MHz", "giữ kênh {:.1} MHz", hold));
        cmd(rx, &format!("hop manual {hold}"), Duration::from_secs(3))?;
    }
    if saved.bw_auto {
        cmd(rx, &format!("chbw {}", width_mhz(saved.div)), Duration::from_secs(3))?;
    }
    Ok(saved)
}

fn restore(sh: &Mutex<Shared>, rx: &str, saved: &Saved) {
    phase(sh, t("putting the link back", "trả link về như cũ").into());
    if saved.bw_auto {
        let _ = cmd(rx, "chbw auto", Duration::from_secs(3));
    } else {
        let _ = cmd(rx, &format!("chbw {}", width_mhz(saved.div)), Duration::from_secs(3));
    }
    if saved.hop_auto {
        let _ = cmd(rx, "hop auto rx 1000", Duration::from_secs(3));
    } else if saved.chan_mhz > 0.0 {
        let _ = cmd(rx, &format!("hop manual {}", saved.chan_mhz), Duration::from_secs(3));
    }
}

/// The ground asks for the width; both ends on it (the aircraft follows over the air).
fn set_width(sh: &Mutex<Shared>, stop: &AtomicBool, tx: &str, rx: &str, div: u64) -> Result<(), String> {
    let now = kv_u64(&cmd(rx, "chbw", Duration::from_secs(3))?, "chbw_div").unwrap_or(1);
    let far = cmd(tx, "chbw", Duration::from_secs(3)).ok().and_then(|t| kv_u64(&t, "chbw_div"));
    if now == div && far == Some(div) {
        return Ok(());
    }
    phase(sh, tf!("width {} MHz", "băng thông {} MHz", width_mhz(div)));
    cmd(rx, &format!("chbw {}", width_mhz(div)), Duration::from_secs(3))?;
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(20) {
        if stop.load(Ordering::Relaxed) {
            return Err(String::new());
        }
        std::thread::sleep(Duration::from_millis(500));
        let g = cmd(rx, "chbw", Duration::from_secs(2)).ok().and_then(|t| kv_u64(&t, "chbw_div"));
        let a = cmd(tx, "chbw", Duration::from_secs(2)).ok().and_then(|t| kv_u64(&t, "chbw_div"));
        if g == Some(div) && a == Some(div) {
            // the receiver's AGC and the PLLs settle
            std::thread::sleep(Duration::from_millis(1500));
            return Ok(());
        }
    }
    Err(tf!("the transmitter did not reach {} MHz", "máy phát không chuyển sang {} MHz", width_mhz(div)))
}

/// Test packets for about `secs`, in batches of about 0.4 s; (packets, bytes, seconds) sent.
/// With a receiver that decodes on its processor (`cap_kbps` > 0) no faster than it keeps up:
/// batches of 0.1 s at that rate, a pause after each.
fn blast(stop: &AtomicBool, sent: &(AtomicU64, AtomicU64), tx: &str, mcs: usize, bytes: usize, div: u64, secs: f64, cap_kbps: u64) -> Result<(u64, u64, f64), String> {
    let air = PHY_MBPS[mcs] / div as f64 * 1e6 * 0.8;
    let paced = cap_kbps > 0 && (cap_kbps as f64) * 1e3 < air;
    let (rate, batch) = if paced { (cap_kbps as f64 * 1e3, 0.1) } else { (air, 0.4) };
    let mut n = ((batch * rate / (bytes as f64 * 8.0)).ceil() as u64).clamp(1, 4000);
    let t0 = Instant::now();
    let (mut pk, mut by) = (0u64, 0u64);
    while t0.elapsed().as_secs_f64() < secs {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        let tb = Instant::now();
        cmd(tx, &format!("o64 blast {mcs} {bytes} {n}"), Duration::from_secs(30)).map_err(|e| tf!("transmitter: {}", "máy phát: {}", e))?;
        let took = tb.elapsed().as_secs_f64();
        pk += n;
        by += n * bytes as u64;
        sent.0.fetch_add(n, Ordering::Relaxed);
        sent.1.fetch_add(n * bytes as u64, Ordering::Relaxed);
        if paced {
            // the rest of the batch's time: the receiver takes them as they come
            if took < batch {
                std::thread::sleep(Duration::from_secs_f64(batch - took));
            }
        } else if took > 0.01 {
            // the next batch sized for 0.4 s at what this one took
            n = ((n as f64 * 0.4 / took).round() as u64).clamp(1, 4000);
        }
    }
    Ok((pk, by, t0.elapsed().as_secs_f64()))
}

/// The receiver decodes on its processor and the air at this MCS brings more than its cap.
fn paced(cap_kbps: u64, mcs: usize, div: u64) -> bool {
    cap_kbps > 0 && (cap_kbps as f64) < PHY_MBPS[mcs] / div as f64 * 1e3 * 0.8
}

/// One second of test packets as the receiver counted them.
#[derive(Clone, Copy, Default)]
struct Second {
    /// sent, received by the software, decoded
    tx_kbps: f64,
    rx_kbps: f64,
    dec_kbps: f64,
    loss: f64,
    air_loss: f64,
    /// let go by the receiver with no time to decode them (None: it does not say)
    busy: Option<f64>,
    snr: Option<f32>,
}

impl Second {
    /// two seconds as one: a second that also takes the packets the one before left in the
    /// receiver's queues reads high (16/10, a WiFi channel: one second of 6.2 Mbit/s in a run of
    /// 3.5)
    fn mean(&self, o: &Second) -> Second {
        let m = |a: f64, b: f64| (a + b) / 2.0;
        Second {
            tx_kbps: m(self.tx_kbps, o.tx_kbps),
            rx_kbps: m(self.rx_kbps, o.rx_kbps),
            dec_kbps: m(self.dec_kbps, o.dec_kbps),
            loss: m(self.loss, o.loss),
            air_loss: m(self.air_loss, o.air_loss),
            busy: self.busy.zip(o.busy).map(|(a, b)| m(a, b)),
            snr: self.snr.or(o.snr),
        }
    }
}

/// A step up a second while the receiver keeps every test packet; then a small one now and then
const RAMP_UP: f64 = 1.12;
const RAMP_PROBE: f64 = 1.04;
const RAMP_PROBE_EVERY: u32 = 6;
/// let go by the receiver with no time for them: it no longer keeps up. The loss tells less: a
/// channel WiFi shares lost 0-30 % a second at any rate (the workers 35 % busy, none let go), and
/// a board's own count of losses holds the control packets (2 a second, 5 % at 0.6 Mbit/s) and
/// the PPDUs WiFi cut short (16/10)
const RAMP_BUSY: f64 = 0.02;
/// or two seconds in a row lost more than this (whatever the reason, nothing to climb on)
const RAMP_GROSS: f64 = 0.5;
/// a receiver that does not say: lost over the least a second lost
const RAMP_OVER: f64 = 0.03;
/// back under the best clean two seconds' rate after an overloaded second
const RAMP_BACK: f64 = 0.97;
/// a sweep's cell: seconds at most
const RAMP_CELL_MAX: u32 = 15;

/// The most a receiver that decodes on its processor keeps up with. Its own cap (`cap_kbps`) is
/// the video's, with room left for the encoder's bursts (about two thirds of its time, 15/10), so
/// the test packets start there and climb a step each second until the receiver lets some go;
/// then they stay under the best clean two seconds, a small step up every few seconds (16/10: at
/// the cap a Pluto ground showed 1.4 Mbit/s, the most it keeps up with is 3.8).
struct Ramp {
    kbps: f64,
    air_kbps: f64,
    floor: f64,
    /// the least loss of a second so far (the air's)
    base: Option<f64>,
    /// the best two clean seconds in a row (one alone until there are two)
    best: Option<Second>,
    last_second: Second,
    prev_clean: Option<Second>,
    found: bool,
    clean: u32,
    /// seconds since the most was found, or at the air's own rate
    settled: u32,
    last: Meter,
}

impl Ramp {
    fn new(kbps: u64, mcs: usize, div: u64, last: Meter) -> Ramp {
        Ramp {
            kbps: kbps as f64,
            air_kbps: PHY_MBPS[mcs] / div as f64 * 1e3 * 0.8,
            floor: kbps as f64 * 0.5,
            base: None,
            best: None,
            last_second: Second::default(),
            prev_clean: None,
            found: false,
            clean: 0,
            settled: 0,
            last,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn step(&mut self, stop: &AtomicBool, sent: &(AtomicU64, AtomicU64), tx: &str, rx: &str, mcs: usize, bytes: usize, div: u64) -> Result<(), String> {
        let (pk, _, _) = blast(stop, sent, tx, mcs, bytes, div, 1.0, self.kbps.round() as u64)?;
        let m = meter(rx)?;
        let dt = m.at.zip(self.last.at).map_or(1.0, |(a, b)| a.duration_since(b).as_secs_f64().max(0.1));
        let got = m.test_ok.saturating_sub(self.last.test_ok);
        // (the receiver's counters restart with a width change: no step back)
        let dec = if m.pl_ok >= self.last.pl_ok { u64::from(m.pl_ok - self.last.pl_ok) } else { 0 };
        let busy = m.rx_busy.zip(self.last.rx_busy).map(|(a, b)| a.saturating_sub(b));
        let s = Second {
            tx_kbps: pk as f64 * bytes as f64 * 8.0 / 1e3 / dt,
            rx_kbps: m.test_bytes.saturating_sub(self.last.test_bytes) as f64 * 8.0 / 1e3 / dt,
            dec_kbps: dec as f64 * bytes as f64 * 8.0 / 1e3 / dt,
            loss: if pk > 0 { (1.0 - got as f64 / pk as f64).clamp(0.0, 1.0) } else { 1.0 },
            air_loss: if pk > 0 { (1.0 - dec as f64 / pk as f64).clamp(0.0, 1.0) } else { 1.0 },
            busy: busy.map(|b| if pk > 0 { b as f64 / pk as f64 } else { 0.0 }),
            snr: m.snr_sig.or(m.snr_test),
        };
        self.last = m;
        let prev_loss = self.last_second.loss;
        self.last_second = s;
        if pk == 0 || stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        let base = self.base.map_or(s.loss, |b| b.min(s.loss));
        self.base = Some(base);
        if self.found || self.kbps >= self.air_kbps {
            self.settled += 1;
        }
        let over = match s.busy {
            Some(b) => b > RAMP_BUSY || s.loss.min(prev_loss) > RAMP_GROSS,
            None => s.loss > base + RAMP_OVER,
        };
        if !over {
            let two = self.prev_clean.map(|p| p.mean(&s));
            if let Some(t) = two.filter(|t| self.best.is_none_or(|b| t.rx_kbps > b.rx_kbps)) {
                self.best = Some(t);
            } else if self.best.is_none() {
                self.best = Some(s);
            }
            self.prev_clean = Some(s);
            self.clean += 1;
            if !self.found {
                self.kbps *= RAMP_UP;
            } else if self.clean >= RAMP_PROBE_EVERY {
                self.kbps *= RAMP_PROBE;
                self.clean = 0;
            }
        } else {
            self.found = true;
            self.clean = 0;
            self.prev_clean = None;
            self.kbps = self.best.map_or(self.kbps * 0.8, |b| b.tx_kbps * RAMP_BACK);
        }
        self.kbps = self.kbps.clamp(self.floor, self.air_kbps);
        Ok(())
    }

    /// the phase line's note
    fn note(&self) -> String {
        match (self.found, self.best) {
            (true, Some(b)) => tf!(" · the receiver's most: {:.2} Mbit/s", " · trần máy thu: {:.2} Mbit/s", b.rx_kbps / 1e3),
            _ => tf!(" · finding the receiver's most: {:.2} Mbit/s", " · đang dò trần máy thu: {:.2} Mbit/s", self.kbps / 1e3),
        }
    }
}

fn run(sh: &Mutex<Shared>, stop: &AtomicBool, sent: &(AtomicU64, AtomicU64), job: Job) -> Result<(), String> {
    let (tx, rx) = {
        let s = sh.lock().unwrap();
        (s.tx_host.clone(), s.rx_host.clone())
    };
    meter(&rx).map_err(|e| if e == "no meter" { t("the receiver's software has no meter: install the new version", "phần mềm máy thu chưa có bộ đếm: cài bản mới").to_string() } else { tf!("receiver: {}", "máy thu: {}", e) })?;
    let chan = match job {
        Job::Test { chan, .. } | Job::Sweep { chan, .. } => chan,
    };
    let saved = prepare(sh, &rx, chan)?;
    let r = (|| -> Result<(), String> {
        match job {
            Job::Test { mcs, bytes, div, .. } => {
                set_width(sh, stop, &tx, &rx, div)?;
                let head = format!("MCS{mcs} {} · {bytes} B · {} MHz", MCS_NAME[mcs], width_mhz(div));
                let cap = meter(&rx)?.cap_kbps;
                if !paced(cap, mcs, div) {
                    phase(sh, head);
                    while !stop.load(Ordering::Relaxed) {
                        blast(stop, sent, &tx, mcs, bytes, div, 1.0, 0)?;
                    }
                    return Ok(());
                }
                let mut r = Ramp::new(cap, mcs, div, meter(&rx)?);
                while !stop.load(Ordering::Relaxed) {
                    phase(sh, format!("{head}{}", r.note()));
                    r.step(stop, sent, &tx, &rx, mcs, bytes, div)?;
                }
                Ok(())
            }
            Job::Sweep { divs, bytes, secs, .. } => {
                for (i, &div) in DIVS.iter().enumerate() {
                    if !divs[i] {
                        continue;
                    }
                    set_width(sh, stop, &tx, &rx, div)?;
                    // the most the last MCS kept up with, a receiver decoding on its processor
                    let mut prev_best = 0.0f64;
                    for mcs in 0..8 {
                        if stop.load(Ordering::Relaxed) {
                            return Ok(());
                        }
                        let head = format!("{} MHz · MCS{mcs} {}", width_mhz(div), MCS_NAME[mcs]);
                        let cap = meter(&rx)?.cap_kbps;
                        if paced(cap, mcs, div) {
                            let start = cap.max((prev_best * 0.85) as u64);
                            let mut r = Ramp::new(start, mcs, div, Meter::default());
                            phase(sh, format!("{head}{}", r.note()));
                            blast(stop, sent, &tx, mcs, bytes, div, 0.5, start)?;
                            r.last = meter(&rx)?;
                            let mut n = 0u32;
                            while !stop.load(Ordering::Relaxed) && n < RAMP_CELL_MAX && (f64::from(n) < secs || r.settled < 2) {
                                phase(sh, format!("{head}{}", r.note()));
                                r.step(stop, sent, &tx, &rx, mcs, bytes, div)?;
                                n += 1;
                            }
                            let b = r.best.unwrap_or(r.last_second);
                            prev_best = b.rx_kbps;
                            let cell = Cell { div, mcs, rx_mbps: b.rx_kbps / 1e3, phy_mbps: b.dec_kbps / 1e3, loss: b.loss, air_loss: b.air_loss, snr: b.snr };
                            sh.lock().unwrap().cells.push(cell);
                            continue;
                        }
                        phase(sh, head);
                        // half a second first: the receiver's AGC on the new level
                        blast(stop, sent, &tx, mcs, bytes, div, 0.5, cap)?;
                        let m0 = meter(&rx)?;
                        let (pk, _, dur) = blast(stop, sent, &tx, mcs, bytes, div, secs, cap)?;
                        std::thread::sleep(Duration::from_millis(400));
                        let m1 = meter(&rx)?;
                        let got = m1.test_ok.saturating_sub(m0.test_ok);
                        let dec = u64::from(m1.pl_ok.wrapping_sub(m0.pl_ok));
                        let cell = Cell {
                            div,
                            mcs,
                            rx_mbps: m1.test_bytes.saturating_sub(m0.test_bytes) as f64 * 8.0 / dur.max(0.1) / 1e6,
                            phy_mbps: dec as f64 * bytes as f64 * 8.0 / dur.max(0.1) / 1e6,
                            loss: if pk > 0 { (1.0 - got as f64 / pk as f64).clamp(0.0, 1.0) } else { 1.0 },
                            air_loss: if pk > 0 { (1.0 - dec as f64 / pk as f64).clamp(0.0, 1.0) } else { 1.0 },
                            snr: m1.snr_sig.or(m1.snr_test),
                        };
                        sh.lock().unwrap().cells.push(cell);
                    }
                }
                Ok(())
            }
        }
    })();
    restore(sh, &rx, &saved);
    match r {
        Err(e) if e.is_empty() => Ok(()),
        r => r,
    }
}
