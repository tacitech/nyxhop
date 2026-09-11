//! nyxhop: one app for either end of the link.
//!
//! The start screen asks which end this computer is and which board it talks to. The
//! board is put into the matching role (console `role tx|rx`, the daemon restarts on
//! its own), then the ground app (nyx-rx) or the aircraft app (nyx-tx) runs inside this
//! window. Both remain available as their own binaries; this is the same code hosted.
//!
//!   nyxhop                       the start screen (remembers the last choice in nyxhop.cfg)
//!   nyxhop --mode rx|tx --board 192.168.0.12 [--no-role]   straight in

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nyx_common::logging::{self, log};
use nyx_common::theme as th;
use nyx_common::Opts;
use nyx_proto::{DEFAULT_RX_PORT, DEFAULT_TX_PORT};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Role {
    /// this computer shows the video: the board next to it is the receiving end
    Ground,
    /// this computer has the camera: the board next to it is the transmitting end
    Aircraft,
}

impl Role {
    fn board_role(self) -> &'static str {
        match self {
            Role::Ground => "rx",
            Role::Aircraft => "tx",
        }
    }
    fn port(self) -> u16 {
        match self {
            Role::Ground => DEFAULT_RX_PORT,
            Role::Aircraft => DEFAULT_TX_PORT,
        }
    }
    fn label(self) -> &'static str {
        match self {
            Role::Ground => "Ground station",
            Role::Aircraft => "Aircraft",
        }
    }
}

enum Mode {
    Choose,
    Rx(nyx_rx::RxApp, Arc<nyx_rx::Shared>),
    Tx(nyx_tx::TxApp, Arc<nyx_tx::Shared>),
}

/// What the preparation thread reports back while the board is being switched.
#[derive(Default)]
struct Prep {
    status: String,
    done: Option<Role>,
    failed: bool,
}

struct App {
    mode: Mode,
    board: String,
    set_role: bool,
    /// the board's own answer to `role`, refreshed while the start screen is up
    board_role: Arc<Mutex<Option<String>>>,
    prep: Arc<Mutex<Prep>>,
    busy: Arc<AtomicBool>,
    cfg_path: std::path::PathBuf,
}

/// One console exchange: the reply lines up to the closing ok/err line.
fn console(addr: &str, cmd: &str, timeout: Duration) -> Result<Vec<String>, String> {
    let sa = addr.parse::<std::net::SocketAddr>().map_err(|e| format!("{addr}: {e}"))?;
    let mut s = TcpStream::connect_timeout(&sa, timeout).map_err(|e| e.to_string())?;
    s.set_read_timeout(Some(timeout)).ok();
    s.write_all(format!("{cmd}\n").as_bytes()).map_err(|e| e.to_string())?;
    let mut r = BufReader::new(s);
    let mut out = Vec::new();
    loop {
        let mut line = String::new();
        if r.read_line(&mut line).map_err(|e| e.to_string())? == 0 {
            break;
        }
        let l = line.trim_end().to_string();
        let end = l == "ok" || l.starts_with("err");
        out.push(l);
        if end {
            break;
        }
    }
    Ok(out)
}

fn console_addr(board: &str) -> String {
    format!("{}:7202", board.trim())
}

/// Ask the board which end it is: "tx", "rx" or nothing.
fn board_role_of(board: &str) -> Option<String> {
    console(&console_addr(board), "role", Duration::from_secs(2))
        .ok()?
        .iter()
        .find_map(|l| l.strip_prefix("role=").map(str::to_string))
}

impl App {
    fn new(cfg_path: std::path::PathBuf, opts: &Opts) -> Self {
        let mut board = opts.arg("--board", "");
        let mut set_role = !opts.flag("--no-role");
        if let Ok(text) = std::fs::read_to_string(&cfg_path) {
            for line in text.lines() {
                let mut it = line.split_whitespace();
                match (it.next(), it.next()) {
                    (Some("board"), Some(v)) if board.is_empty() => board = v.to_string(),
                    (Some("set_role"), Some(v)) => set_role = v != "0",
                    _ => {}
                }
            }
        }
        if board.is_empty() {
            board = "192.168.0.12".into();
        }
        let app = App {
            mode: Mode::Choose,
            board,
            set_role,
            board_role: Arc::new(Mutex::new(None)),
            prep: Arc::new(Mutex::new(Prep::default())),
            busy: Arc::new(AtomicBool::new(false)),
            cfg_path,
        };
        app.spawn_role_poll();
        match opts.arg("--mode", "").as_str() {
            "rx" | "ground" => app.start(Role::Ground),
            "tx" | "aircraft" => app.start(Role::Aircraft),
            _ => {}
        }
        app
    }

    /// While the start screen is up, keep asking the board which end it is.
    fn spawn_role_poll(&self) {
        let board = self.board.clone();
        let slot = self.board_role.clone();
        let busy = self.busy.clone();
        std::thread::Builder::new()
            .name("role-poll".into())
            .spawn(move || {
                let mut last = String::new();
                loop {
                    if !busy.load(Ordering::Relaxed) {
                        let r = board_role_of(&board);
                        if let Some(ref v) = r {
                            if *v != last {
                                log(&format!("board {board}: role {v}"));
                                last = v.clone();
                            }
                        }
                        *slot.lock().unwrap() = r;
                    }
                    std::thread::sleep(Duration::from_secs(2));
                }
            })
            .expect("spawn role-poll");
    }

    fn save_choice(&self, role: Role) {
        let text = format!(
            "# nyxhop start screen: the last choice\nboard {}\nmode {}\nset_role {}\n",
            self.board.trim(),
            role.board_role(),
            u8::from(self.set_role)
        );
        if let Err(e) = std::fs::write(&self.cfg_path, text) {
            log(&format!("nyxhop.cfg not saved: {e}"));
        }
    }

    /// The operator chose: put the board into the role (unless told not to), wait for it
    /// to come back, then hand over to the app. All of it off the UI thread.
    fn start(&self, role: Role) {
        if self.busy.swap(true, Ordering::Relaxed) {
            return;
        }
        self.save_choice(role);
        let board = self.board.trim().to_string();
        let set_role = self.set_role;
        let prep = self.prep.clone();
        let busy = self.busy.clone();
        let set = |prep: &Arc<Mutex<Prep>>, s: String| {
            log(&s);
            prep.lock().unwrap().status = s;
        };
        std::thread::Builder::new()
            .name("prepare".into())
            .spawn(move || {
                let addr = console_addr(&board);
                let want = role.board_role();
                if set_role {
                    match board_role_of(&board) {
                        Some(r) if r == want => set(&prep, format!("board {board} is already the {want} end")),
                        Some(r) => {
                            set(&prep, format!("board {board}: {r} end -> {want} end, restarting..."));
                            match console(&addr, &format!("role {want}"), Duration::from_secs(4)) {
                                Ok(lines) if lines.iter().any(|l| l.starts_with("err")) => {
                                    set(&prep, format!("board refused: {}", lines.join(" ")));
                                    prep.lock().unwrap().failed = true;
                                    busy.store(false, Ordering::Relaxed);
                                    return;
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    set(&prep, format!("board {board}: {e}"));
                                    prep.lock().unwrap().failed = true;
                                    busy.store(false, Ordering::Relaxed);
                                    return;
                                }
                            }
                            let t0 = Instant::now();
                            let mut back = false;
                            while t0.elapsed() < Duration::from_secs(90) {
                                std::thread::sleep(Duration::from_secs(2));
                                let secs = t0.elapsed().as_secs();
                                if board_role_of(&board).as_deref() == Some(want) {
                                    // the boot script applies the cfg for a few seconds after
                                    // the daemon answers; give it that
                                    std::thread::sleep(Duration::from_secs(4));
                                    back = true;
                                    break;
                                }
                                set(&prep, format!("board {board} restarting as the {want} end... {secs} s"));
                            }
                            if !back {
                                set(&prep, format!("board {board} did not come back as the {want} end"));
                                prep.lock().unwrap().failed = true;
                                busy.store(false, Ordering::Relaxed);
                                return;
                            }
                            set(&prep, format!("board {board} is the {want} end"));
                        }
                        None => set(&prep, format!("board {board} does not answer; starting anyway")),
                    }
                }
                prep.lock().unwrap().done = Some(role);
            })
            .expect("spawn prepare");
    }

    /// The preparation finished: build the chosen app in this window.
    fn take_over(&mut self, role: Role) {
        let channel = format!("{}:{}", self.board.trim(), role.port());
        log(&format!("{}: channel {channel}", role.label()));
        self.mode = match role {
            Role::Ground => {
                let shared = nyx_rx::setup(&Opts::from_list(["--channel", channel.as_str()]));
                Mode::Rx(nyx_rx::RxApp::new(shared.clone()), shared)
            }
            Role::Aircraft => {
                let shared = nyx_tx::setup(&Opts::from_list(["--channel", channel.as_str()]));
                Mode::Tx(nyx_tx::TxApp::new(shared.clone()), shared)
            }
        };
    }

    fn choose_ui(&mut self, ui: &mut egui::Ui) {
        let (status, done, failed) = {
            let p = self.prep.lock().unwrap();
            (p.status.clone(), p.done, p.failed)
        };
        if let Some(role) = done {
            self.prep.lock().unwrap().done = None;
            self.take_over(role);
            return;
        }
        let busy = self.busy.load(Ordering::Relaxed);
        ui.add_space(24.0);
        ui.vertical_centered(|ui| {
            ui.label(egui::RichText::new("NYXHOP").size(30.0).color(th::CYAN).strong());
            ui.label(egui::RichText::new("Which end is this computer?").size(16.0).color(th::DIM));
        });
        ui.add_space(20.0);
        let board_role = self.board_role.lock().unwrap().clone();
        th::section(ui, "Board", true, |ui| {
            th::row(ui, "Address", |ui| {
                ui.add_enabled(!busy, egui::TextEdit::singleline(&mut self.board).desired_width(200.0));
            });
            let txt = match board_role.as_deref() {
                Some("rx") => "answers: the receiving end (ground)".to_string(),
                Some("tx") => "answers: the transmitting end (aircraft)".to_string(),
                Some(o) => format!("answers: {o}"),
                None => "no answer yet".to_string(),
            };
            ui.label(egui::RichText::new(txt).small().color(th::DIM));
            ui.checkbox(&mut self.set_role, "Put the board into the matching role")
                .on_hover_text("Sends `role rx|tx`; the board restarts in about 10 s. Off: use the board as it is.");
        });
        ui.add_space(12.0);
        let big = egui::vec2(ui.available_width(), 64.0);
        for role in [Role::Ground, Role::Aircraft] {
            let sub = match role {
                Role::Ground => "this computer shows the video",
                Role::Aircraft => "this computer has the camera",
            };
            let suggested = board_role.as_deref() == Some(role.board_role());
            let text = egui::RichText::new(format!("{}\n{sub}", role.label())).size(17.0);
            let btn = egui::Button::new(if suggested { text.color(th::GOOD) } else { text }).min_size(big);
            if ui.add_enabled(!busy, btn).clicked() {
                self.start(role);
            }
            ui.add_space(8.0);
        }
        if !status.is_empty() {
            ui.add_space(8.0);
            let col = if failed { th::BAD } else { th::DIM };
            ui.label(egui::RichText::new(status).color(col));
        }
        if busy {
            ui.spinner();
            ui.ctx().request_repaint_after(Duration::from_millis(250));
        } else {
            ui.ctx().request_repaint_after(Duration::from_secs(1));
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, root: &mut egui::Ui, frame: &mut eframe::Frame) {
        match &mut self.mode {
            Mode::Rx(app, _) => app.ui(root, frame),
            Mode::Tx(app, _) => app.ui(root, frame),
            Mode::Choose => self.choose_ui(root),
        }
    }
}

fn main() -> eframe::Result {
    logging::init("nyxhop");
    let opts = Opts::from_env();
    // the choice file sits next to the exe, or in the cwd when the exe dir is not writable
    let cfg_path = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(|d| d.join("nyxhop.cfg")))
        .unwrap_or_else(|| "nyxhop.cfg".into());
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([560.0, 900.0]).with_title("NyxHop"),
        ..Default::default()
    };
    let result = eframe::run_native(
        "NyxHop",
        options,
        Box::new(move |cc| {
            nyx_common::ui::touch_style(&cc.egui_ctx);
            Ok(Box::new(App::new(cfg_path, &opts)))
        }),
    );
    result
}

impl Drop for App {
    fn drop(&mut self) {
        match &self.mode {
            Mode::Rx(_, shared) => nyx_rx::shutdown(shared),
            Mode::Tx(_, shared) => nyx_tx::shutdown(shared),
            Mode::Choose => {}
        }
    }
}
