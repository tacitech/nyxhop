//! nyx-board: control panel for a board (the nyx-radio-node daemon, console :7202).
//!
//! Connects by TCP to the daemon's text console and shows/edits every radio parameter at runtime:
//! RX/TX LO, gains, AGC, bandwidth, loopback, plus the capstat/stats diagnostics refreshed every
//! second. Every reply is logged to the console and to logs/board.log (debug from the log, no need
//! to watch the GUI).
//!
//! Usage: nyx-board [--board 192.168.0.10:7202]

use std::collections::{HashMap, VecDeque};
use std::io::{BufRead, BufReader, Write as IoWrite};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nyx_common::logging::{self, log};
use nyx_proto::cli_arg;

/// State shared between the TCP worker and the GUI.
#[derive(Default)]
struct State {
    /// latest key=value from get/stats/capstat/rssi.
    kv: HashMap<String, String>,
    /// The capstat verdict line (the line without '=').
    capstat_verdict: String,
    /// Log of commands + replies (shown in the console pane).
    console: VecDeque<String>,
    connected: bool,
    /// Rates computed from the stats counter deltas.
    captures_per_s: f32,
    tx_frames_per_s: f32,
}

struct Shared {
    state: Mutex<State>,
    addr: Mutex<String>,
    auto_poll: AtomicBool,
    stop: AtomicBool,
}

fn main() -> eframe::Result {
    logging::init("board");
    let addr = cli_arg("--board", "192.168.0.10:7202");
    log(&format!("board console target: {addr}"));

    let shared = Arc::new(Shared {
        state: Mutex::new(State::default()),
        addr: Mutex::new(addr),
        auto_poll: AtomicBool::new(true),
        stop: AtomicBool::new(false),
    });

    let (cmd_tx, cmd_rx) = channel::<String>();
    spawn_worker(shared.clone(), cmd_rx);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([470.0, 760.0])
            .with_title("NyxHop Board Control"),
        ..Default::default()
    };
    let s = shared.clone();
    let result = eframe::run_native(
        "NyxHop Board Control",
        options,
        Box::new(move |_cc| Ok(Box::new(BoardApp::new(s, cmd_tx)))),
    );
    shared.stop.store(true, Ordering::Relaxed);
    result
}

// ------------------------------------------------------------- worker --

/// Send one command and read the reply up to the closing "ok"/"err…" line. Returns the lines.
fn transact(stream: &mut TcpStream, cmd: &str) -> std::io::Result<Vec<String>> {
    stream.write_all(format!("{cmd}\n").as_bytes())?;
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut out = Vec::new();
    loop {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "console closed",
            ));
        }
        let t = line.trim().to_string();
        let done = t == "ok" || t.starts_with("err");
        out.push(t);
        if done {
            return Ok(out);
        }
    }
}

fn spawn_worker(shared: Arc<Shared>, cmd_rx: Receiver<String>) {
    std::thread::Builder::new()
        .name("board-io".into())
        .spawn(move || {
            let mut conn: Option<TcpStream> = None;
            let mut last_poll = Instant::now();
            let mut prev_counters: Option<(Instant, u64, u64)> = None;
            loop {
                if shared.stop.load(Ordering::Relaxed) {
                    return;
                }
                // A user command waits at most 200 ms, then it is the poll's turn.
                let user_cmd = cmd_rx.recv_timeout(Duration::from_millis(200)).ok();
                let mut batch: Vec<(String, bool)> = Vec::new(); // (cmd, is_poll)
                if let Some(c) = user_cmd {
                    batch.push((c, false));
                }
                if shared.auto_poll.load(Ordering::Relaxed)
                    && last_poll.elapsed() > Duration::from_millis(1000)
                {
                    last_poll = Instant::now();
                    for c in ["get", "stats", "capstat", "rssi"] {
                        batch.push((c.to_string(), true));
                    }
                }
                if batch.is_empty() {
                    continue;
                }

                // Make sure there is a connection.
                if conn.is_none() {
                    let addr = shared.addr.lock().unwrap().clone();
                    match std::net::TcpStream::connect_timeout(
                        &match addr.parse() {
                            Ok(a) => a,
                            Err(_) => {
                                push_console(&shared, format!("! bad address: {addr}"));
                                continue;
                            }
                        },
                        Duration::from_millis(1500),
                    ) {
                        Ok(s) => {
                            let _ = s.set_read_timeout(Some(Duration::from_millis(2000)));
                            let _ = s.set_nodelay(true);
                            log(&format!("connected to board console {addr}"));
                            conn = Some(s);
                        }
                        Err(e) => {
                            let mut st = shared.state.lock().unwrap();
                            if st.connected {
                                log(&format!("board console unreachable: {e}"));
                            }
                            st.connected = false;
                            continue;
                        }
                    }
                }
                shared.state.lock().unwrap().connected = true;

                for (cmd, is_poll) in batch {
                    let Some(s) = conn.as_mut() else { break };
                    match transact(s, &cmd) {
                        Ok(lines) => {
                            if !is_poll {
                                push_console(&shared, format!("> {cmd}"));
                                for l in &lines {
                                    push_console(&shared, format!("  {l}"));
                                }
                                log(&format!("cmd `{cmd}` -> {}", lines.join(" | ")));
                            }
                            absorb(&shared, &cmd, &lines, &mut prev_counters);
                        }
                        Err(e) => {
                            push_console(&shared, format!("! `{cmd}` failed: {e} (reconnect)"));
                            log(&format!("console io error on `{cmd}`: {e}"));
                            conn = None;
                            shared.state.lock().unwrap().connected = false;
                            break;
                        }
                    }
                }
            }
        })
        .expect("spawn board-io");
}

fn push_console(shared: &Shared, line: String) {
    let mut st = shared.state.lock().unwrap();
    st.console.push_back(line);
    while st.console.len() > 200 {
        st.console.pop_front();
    }
}

/// Load key=value lines into the state table; compute rates from stats.
fn absorb(
    shared: &Shared,
    cmd: &str,
    lines: &[String],
    prev: &mut Option<(Instant, u64, u64)>,
) {
    let mut st = shared.state.lock().unwrap();
    for l in lines {
        if l == "ok" || l.starts_with("err") {
            continue;
        }
        if let Some((k, v)) = l.split_once('=') {
            // capstat packs several fields on one line: split on whitespace.
            if cmd == "capstat" && l.contains(' ') && l.matches('=').count() > 1 {
                for part in l.split_whitespace() {
                    if let Some((pk, pv)) = part.split_once('=') {
                        st.kv.insert(pk.to_string(), pv.to_string());
                    }
                }
            } else {
                st.kv.insert(k.trim().to_string(), v.trim().to_string());
            }
        } else if cmd == "capstat" {
            st.capstat_verdict = l.clone();
        }
    }
    if cmd == "stats" {
        let cf = st.kv.get("captures_fwd").and_then(|v| v.parse::<u64>().ok());
        let tf = st.kv.get("tx_frames").and_then(|v| v.parse::<u64>().ok());
        if let (Some(cf), Some(tf)) = (cf, tf) {
            if let Some((t0, cf0, tf0)) = *prev {
                let dt = t0.elapsed().as_secs_f32();
                if dt > 0.5 {
                    st.captures_per_s = (cf.saturating_sub(cf0)) as f32 / dt;
                    st.tx_frames_per_s = (tf.saturating_sub(tf0)) as f32 / dt;
                    *prev = Some((Instant::now(), cf, tf));
                }
            } else {
                *prev = Some((Instant::now(), cf, tf));
            }
        }
    }
}

// ---------------------------------------------------------------- GUI --

struct BoardApp {
    shared: Arc<Shared>,
    cmd_tx: Sender<String>,
    addr_buf: String,
    rxfreq_mhz: String,
    txfreq_mhz: String,
    bw_mhz: String,
    rxgain: f32,
    txgain: f32,
    cmd_buf: String,
    /// Whether the initial values from the board have been loaded into the input fields.
    seeded: bool,
}

impl BoardApp {
    fn new(shared: Arc<Shared>, cmd_tx: Sender<String>) -> Self {
        let addr_buf = shared.addr.lock().unwrap().clone();
        BoardApp {
            shared,
            cmd_tx,
            addr_buf,
            rxfreq_mhz: "2450".into(),
            txfreq_mhz: "2450".into(),
            bw_mhz: "14".into(),
            rxgain: 55.0,
            txgain: -30.0,
            cmd_buf: String::new(),
            seeded: false,
        }
    }

    fn send(&self, cmd: impl Into<String>) {
        let _ = self.cmd_tx.send(cmd.into());
    }
}

fn mhz_to_hz(s: &str) -> Option<u64> {
    s.trim().parse::<f64>().ok().map(|m| (m * 1e6).round() as u64)
}

impl eframe::App for BoardApp {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        root.ctx().request_repaint_after(Duration::from_millis(300));
        let (kv, verdict, console, connected, cps, tps) = {
            let st = self.shared.state.lock().unwrap();
            (
                st.kv.clone(),
                st.capstat_verdict.clone(),
                st.console.iter().cloned().collect::<Vec<_>>(),
                st.connected,
                st.captures_per_s,
                st.tx_frames_per_s,
            )
        };
        // First `get` received: load the real values into the edit fields.
        if !self.seeded {
            if let Some(v) = kv.get("rxfreq").and_then(|v| v.parse::<f64>().ok()) {
                self.rxfreq_mhz = format!("{}", v / 1e6);
                self.seeded = true;
            }
            if let Some(v) = kv.get("txfreq").and_then(|v| v.parse::<f64>().ok()) {
                self.txfreq_mhz = format!("{}", v / 1e6);
            }
            if let Some(v) = kv.get("bw").and_then(|v| v.parse::<f64>().ok()) {
                self.bw_mhz = format!("{}", v / 1e6);
            }
            let num = |s: &String| {
                s.split_whitespace().next().and_then(|x| x.parse::<f32>().ok())
            };
            if let Some(g) = kv.get("rxgain").and_then(num) {
                self.rxgain = g;
            }
            if let Some(g) = kv.get("txgain").and_then(num) {
                self.txgain = g;
            }
        }

        egui::CentralPanel::default_margins().show(root, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("ADRV9364 Board Control");
                ui.horizontal(|ui| {
                    ui.label("Console:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.addr_buf)
                            .desired_width(150.0)
                            .hint_text("ip:7202"),
                    );
                    if ui.button("Connect").clicked() {
                        *self.shared.addr.lock().unwrap() =
                            self.addr_buf.trim().to_string();
                        self.seeded = false;
                        log(&format!("target -> {}", self.addr_buf.trim()));
                    }
                    if connected {
                        ui.colored_label(egui::Color32::LIGHT_GREEN, "● online");
                    } else {
                        ui.colored_label(egui::Color32::YELLOW, "○ offline");
                    }
                    let mut ap = self.shared.auto_poll.load(Ordering::Relaxed);
                    if ui.checkbox(&mut ap, "auto 1s").changed() {
                        self.shared.auto_poll.store(ap, Ordering::Relaxed);
                    }
                });
                ui.separator();

                // ------------------------------------------ radio state
                ui.heading("Radio");
                egui::Grid::new("radio").num_columns(3).show(ui, |ui| {
                    ui.label("RX LO (MHz):");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.rxfreq_mhz)
                            .desired_width(90.0),
                    );
                    if ui.button("Set").clicked() {
                        if let Some(hz) = mhz_to_hz(&self.rxfreq_mhz) {
                            self.send(format!("rxfreq {hz}"));
                        }
                    }
                    ui.end_row();

                    ui.label("TX LO (MHz):");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.txfreq_mhz)
                            .desired_width(90.0),
                    );
                    if ui.button("Set").clicked() {
                        if let Some(hz) = mhz_to_hz(&self.txfreq_mhz) {
                            self.send(format!("txfreq {hz}"));
                        }
                    }
                    ui.end_row();

                    ui.label("BW analog (MHz):");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.bw_mhz).desired_width(90.0),
                    );
                    if ui.button("Set").clicked() {
                        if let Some(hz) = mhz_to_hz(&self.bw_mhz) {
                            self.send(format!("bw {hz}"));
                        }
                    }
                    ui.end_row();
                });

                ui.horizontal(|ui| {
                    ui.label("RX gain:");
                    ui.add(egui::Slider::new(&mut self.rxgain, 0.0..=77.0).suffix(" dB"));
                    if ui.button("Set").clicked() {
                        self.send(format!("rxgain {}", self.rxgain.round() as i32));
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("TX gain:");
                    ui.add(egui::Slider::new(&mut self.txgain, -89.0..=0.0).suffix(" dB"));
                    if ui.button("Set").clicked() {
                        self.send(format!("txgain {}", self.txgain.round() as i32));
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("AGC:");
                    let cur = kv.get("agc").cloned().unwrap_or_default();
                    for (m, lbl) in
                        [("manual", "manual"), ("slow", "slow"), ("fast", "fast")]
                    {
                        if ui.selectable_label(cur.starts_with(m), lbl).clicked() {
                            self.send(format!("agc {m}"));
                        }
                    }
                    ui.separator();
                    ui.label("RSSI:");
                    ui.monospace(kv.get("rssi").cloned().unwrap_or_else(|| "—".into()));
                });
                ui.horizontal(|ui| {
                    ui.label("Loopback:");
                    let cur = kv.get("loopback").cloned().unwrap_or_default();
                    for (v, lbl) in [("0", "off (RF)"), ("1", "digital BIST"), ("2", "RF")]
                    {
                        if ui.selectable_label(cur == v, lbl).clicked() {
                            self.send(format!("loopback {v}"));
                        }
                    }
                });
                ui.horizontal(|ui| {
                    if ui.button("⚡ Setup digital loopback").clicked() {
                        self.send("loopback 1");
                        self.send("bw 14000000");
                        log("quick setup: digital loopback");
                    }
                    if ui.button("📡 Setup cable/RF").clicked() {
                        self.send("loopback 0");
                        self.send("bw 14000000");
                        log("quick setup: RF/cable");
                    }
                });
                ui.separator();

                // ------------------------------------------- diagnostics
                ui.heading("ADC input signal (capstat)");
                egui::Grid::new("capstat").num_columns(4).show(ui, |ui| {
                    let g = |k: &str| kv.get(k).cloned().unwrap_or_else(|| "—".into());
                    ui.label("rms:");
                    ui.monospace(g("rms"));
                    ui.label("peak:");
                    ui.monospace(g("peak"));
                    ui.end_row();
                    ui.label("DC I/Q:");
                    ui.monospace(format!("{} / {}", g("dc_i"), g("dc_q")));
                    ui.label("S&C metric:");
                    ui.monospace(g("sc_metric"));
                    ui.end_row();
                });
                if !verdict.is_empty() {
                    let color = if verdict.contains("PREAMBLE PRESENT") {
                        egui::Color32::LIGHT_GREEN
                    } else {
                        egui::Color32::YELLOW
                    };
                    ui.colored_label(color, &verdict);
                }
                ui.separator();

                ui.heading("Pipeline (daemon)");
                egui::Grid::new("stats").num_columns(4).show(ui, |ui| {
                    let g = |k: &str| kv.get(k).cloned().unwrap_or_else(|| "—".into());
                    ui.label("TX app:");
                    ui.monospace(g("tx_up"));
                    ui.label("RX app:");
                    ui.monospace(g("rx_up"));
                    ui.end_row();
                    ui.label("captures/s:");
                    ui.monospace(format!("{cps:.1}"));
                    ui.label("tx frames/s:");
                    ui.monospace(format!("{tps:.1}"));
                    ui.end_row();
                    ui.label("rx_drops:");
                    ui.monospace(g("rx_drops"));
                    ui.label("fs:");
                    ui.monospace(g("fs"));
                    ui.end_row();
                });
                ui.separator();

                // ----------------------------------------------- console
                ui.heading("Console");
                ui.horizontal(|ui| {
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut self.cmd_buf)
                            .desired_width(300.0)
                            .hint_text("any command, e.g. sysfs in_voltage0_rssi"),
                    );
                    let go = ui.button("Send").clicked()
                        || (resp.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter)));
                    if go && !self.cmd_buf.trim().is_empty() {
                        self.send(self.cmd_buf.trim().to_string());
                        self.cmd_buf.clear();
                    }
                });
                egui::ScrollArea::vertical()
                    .max_height(180.0)
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for l in &console {
                            ui.monospace(l);
                        }
                    });
            });
        });
    }
}
