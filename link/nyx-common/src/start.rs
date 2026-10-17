//! The start screen shared by the combined apps (`nyxhop` on the PC, the Android app):
//! which end is this device, which board it talks to, put the board into the matching
//! role (console `role tx|rx`, the daemon restarts on its own) and wait for it to come
//! back. The host then builds the ground or the aircraft screen; while it hosts one, the
//! board's role keeps being polled so the host can follow when the board is switched to
//! the other end (the Board role switch in a drawer, the other computer, a console).

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::i18n::t;
use crate::logging::log;
use crate::theme as th;
pub use crate::ui::Role;

impl Role {
    /// The word the board's console uses for this end.
    pub fn board_role(self) -> &'static str {
        match self {
            Role::Ground => "rx",
            Role::Aircraft => "tx",
        }
    }
    /// The daemon port the app of this end talks to.
    pub fn port(self) -> u16 {
        match self {
            Role::Ground => nyx_proto::DEFAULT_RX_PORT,
            Role::Aircraft => nyx_proto::DEFAULT_TX_PORT,
        }
    }
    /// The end's name in English (the hosts' log lines use it).
    pub fn label(self) -> &'static str {
        match self {
            Role::Ground => "Ground station",
            Role::Aircraft => "Aircraft",
        }
    }
    /// The end's name on screen, in the language in use.
    pub fn name(self) -> &'static str {
        match self {
            Role::Ground => t("Ground station", "Trạm mặt đất"),
            Role::Aircraft => t("Aircraft", "Máy bay"),
        }
    }
    /// `rx`/`ground` or `tx`/`aircraft` (a config file, a command line).
    pub fn from_word(s: &str) -> Option<Role> {
        match s.trim().to_ascii_lowercase().as_str() {
            "rx" | "ground" | "b" => Some(Role::Ground),
            "tx" | "aircraft" | "a" => Some(Role::Aircraft),
            _ => None,
        }
    }
    pub fn other(self) -> Role {
        match self {
            Role::Ground => Role::Aircraft,
            Role::Aircraft => Role::Ground,
        }
    }
}

/// One console exchange with a board: the reply lines up to the closing ok/err line.
pub fn console(addr: &str, cmd: &str, timeout: Duration) -> Result<Vec<String>, String> {
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

/// The console address of a board given as `ip` (or `ip:port` of any of its ports).
pub fn console_addr(board: &str) -> String {
    let host = board.trim().split(':').next().unwrap_or("").trim();
    format!("{host}:7202")
}

/// Ask the board which end it is: "tx", "rx" or nothing (no answer, or an old daemon).
pub fn board_role_of(board: &str) -> Option<String> {
    console(&console_addr(board), "role", Duration::from_secs(2))
        .ok()?
        .iter()
        .find_map(|l| l.strip_prefix("role=").map(str::to_string))
}

/// The board's word for an end (`rx`, `tx`) in the Vietnamese status lines of the start screen.
fn end_vi(word: &str) -> &str {
    match word {
        "rx" => "mặt đất",
        "tx" => "máy bay",
        o => o,
    }
}

/// `s` with its first letter in upper case (a Vietnamese sentence that starts with the device).
fn cap(s: &str) -> String {
    let mut c = s.chars();
    c.next().map(|f| f.to_uppercase().chain(c).collect()).unwrap_or_default()
}

/// What the preparation thread reports back while the board is being switched.
#[derive(Default)]
struct Prep {
    status: String,
    done: Option<Role>,
    failed: bool,
}

/// What the start screen tells its host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The operator pressed a button: a good moment to remember the choice.
    Chosen(Role),
    /// The board is in the role (or was left alone): build that end's screen now.
    Ready(Role),
}

pub struct Chooser {
    /// The board's address as typed on the start screen (`ip`, a port is ignored).
    pub board: String,
    /// Send `role rx|tx` before starting; off = use the board as it is.
    pub set_role: bool,
    /// "this computer", "this phone": the subtitle of the start screen.
    device: &'static str,
    /// the board's own answer to `role`, refreshed every 2 s while not busy
    board_role: Arc<Mutex<Option<String>>>,
    /// the poll thread follows the address box through this
    poll_board: Arc<Mutex<String>>,
    prep: Arc<Mutex<Prep>>,
    busy: Arc<AtomicBool>,
    /// the end whose screen the host shows, once it hosts one
    hosted: Option<Role>,
}

impl Chooser {
    pub fn new(board: String, set_role: bool, device: &'static str) -> Self {
        let c = Chooser {
            poll_board: Arc::new(Mutex::new(board.clone())),
            board,
            set_role,
            device,
            board_role: Arc::new(Mutex::new(None)),
            prep: Arc::new(Mutex::new(Prep::default())),
            busy: Arc::new(AtomicBool::new(false)),
            hosted: None,
        };
        c.spawn_role_poll();
        c
    }

    /// The board's latest answer to `role`.
    pub fn board_role(&self) -> Option<String> {
        self.board_role.lock().unwrap().clone()
    }

    pub fn busy(&self) -> bool {
        self.busy.load(Ordering::Relaxed)
    }

    pub fn hosted(&self) -> Option<Role> {
        self.hosted
    }

    pub fn status(&self) -> String {
        self.prep.lock().unwrap().status.clone()
    }

    /// `device` in the language on screen. The hosts pass "this computer" / "this phone", or
    /// its Vietnamese as it was when the app started; either way it follows a later switch.
    fn device_word(&self) -> &'static str {
        const PAIRS: [(&str, &str); 3] =
            [("this computer", "máy tính này"), ("this phone", "điện thoại này"), ("this device", "thiết bị này")];
        PAIRS
            .iter()
            .find(|(en, vi)| self.device == *en || self.device == *vi)
            .map(|&(en, vi)| t(en, vi))
            .unwrap_or(self.device)
    }

    /// Keep asking the board which end it is (2 s), except while a switch is in progress.
    fn spawn_role_poll(&self) {
        let slot = self.board_role.clone();
        let busy = self.busy.clone();
        let poll_board = self.poll_board.clone();
        std::thread::Builder::new()
            .name("role-poll".into())
            .spawn(move || {
                let mut last = String::new();
                loop {
                    if !busy.load(Ordering::Relaxed) {
                        let board = poll_board.lock().unwrap().clone();
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

    /// The operator chose: put the board into the role (unless told not to), wait for it
    /// to come back, then report Ready. All of it off the UI thread.
    pub fn start(&self, role: Role) {
        if self.busy.swap(true, Ordering::Relaxed) {
            return;
        }
        let board = self.board.trim().to_string();
        *self.poll_board.lock().unwrap() = board.clone();
        let set_role = self.set_role;
        let prep = self.prep.clone();
        let busy = self.busy.clone();
        *prep.lock().unwrap() = Prep::default();
        // the log keeps the English line; the screen shows the line in the language in use
        let set = |prep: &Arc<Mutex<Prep>>, s: String, vi: String| {
            log(&s);
            prep.lock().unwrap().status = if crate::i18n::vi() { vi } else { s };
        };
        std::thread::Builder::new()
            .name("prepare".into())
            .spawn(move || {
                let addr = console_addr(&board);
                let want = role.board_role();
                let want_vi = end_vi(want);
                if set_role {
                    match board_role_of(&board) {
                        Some(r) if r == want => set(
                            &prep,
                            format!("board {board} is already the {want} end"),
                            format!("mạch {board} đã ở vai trò {want_vi}"),
                        ),
                        Some(r) => {
                            set(
                                &prep,
                                format!("board {board}: {r} end -> {want} end, restarting..."),
                                format!("mạch {board}: {} → {want_vi}, đang khởi động lại...", end_vi(&r)),
                            );
                            match console(&addr, &format!("role {want}"), Duration::from_secs(4)) {
                                Ok(lines) if lines.iter().any(|l| l.starts_with("err")) => {
                                    set(
                                        &prep,
                                        format!("board refused: {}", lines.join(" ")),
                                        format!("mạch từ chối: {}", lines.join(" ")),
                                    );
                                    prep.lock().unwrap().failed = true;
                                    busy.store(false, Ordering::Relaxed);
                                    return;
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    set(&prep, format!("board {board}: {e}"), format!("mạch {board}: {e}"));
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
                                set(
                                    &prep,
                                    format!("board {board} restarting as the {want} end... {secs} s"),
                                    format!("mạch {board} đang khởi động lại với vai trò {want_vi}... {secs} s"),
                                );
                            }
                            if !back {
                                set(
                                    &prep,
                                    format!("board {board} did not come back as the {want} end"),
                                    format!("mạch {board} không lên lại với vai trò {want_vi}"),
                                );
                                prep.lock().unwrap().failed = true;
                                busy.store(false, Ordering::Relaxed);
                                return;
                            }
                            set(
                                &prep,
                                format!("board {board} is the {want} end"),
                                format!("mạch {board} đã ở vai trò {want_vi}"),
                            );
                        }
                        None => set(
                            &prep,
                            format!("board {board} does not answer; starting anyway"),
                            format!("mạch {board} không trả lời; vẫn bắt đầu"),
                        ),
                    }
                }
                prep.lock().unwrap().done = Some(role);
            })
            .expect("spawn prepare");
    }

    /// The host now shows this end's screen. The role poll resumes (the drawer has a Board
    /// role switch, and `follow` watches for the board coming back as the other end). The
    /// slot still names the end the board was before the switch: overwrite it first,
    /// otherwise the stale value would make the host follow at once.
    pub fn take_over(&mut self, role: Role) {
        self.hosted = Some(role);
        *self.board_role.lock().unwrap() = Some(role.board_role().to_string());
        self.busy.store(false, Ordering::Relaxed);
    }

    /// Back to the start screen.
    pub fn leave(&mut self) {
        self.hosted = None;
        *self.prep.lock().unwrap() = Prep::default();
    }

    /// While hosting: the board answers as the OTHER end -> that end (the host should switch).
    pub fn follow(&self) -> Option<Role> {
        let mine = self.hosted?;
        let other = mine.other();
        (self.board_role().as_deref() == Some(other.board_role())).then_some(other)
    }

    /// The start screen. `Event::Chosen` when a button was pressed, `Event::Ready` once the
    /// board is in the role: the host builds the screen and calls `take_over`.
    pub fn ui(&mut self, ui: &mut egui::Ui) -> Option<Event> {
        let (status, done, failed) = {
            let p = self.prep.lock().unwrap();
            (p.status.clone(), p.done, p.failed)
        };
        if let Some(role) = done {
            self.prep.lock().unwrap().done = None;
            return Some(Event::Ready(role));
        }
        let busy = self.busy.load(Ordering::Relaxed);
        let mut event = None;
        // a phone held sideways is short: less air, the two buttons side by side, and the
        // whole thing scrolls
        let short = ui.available_height() < 700.0;
        let wide = ui.available_width() >= 640.0;
        let device = self.device_word();
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add_space(if short { 6.0 } else { 24.0 });
            ui.vertical_centered(|ui| {
                ui.label(egui::RichText::new("NYXHOP").size(if short { 24.0 } else { 30.0 }).color(th::CYAN).strong());
                let ask = if crate::i18n::vi() { format!("{} là đầu nào?", cap(device)) } else { format!("Which end is {device}?") };
                ui.label(egui::RichText::new(ask).size(16.0).color(th::DIM));
            });
            ui.add_space(if short { 4.0 } else { 10.0 });
            let board_role = self.board_role();
            th::section(ui, t("Board", "Mạch"), true, |ui| {
                th::row(ui, t("Address", "Địa chỉ"), |ui| {
                    let r = ui.add_enabled(!busy, egui::TextEdit::singleline(&mut self.board).desired_width(200.0));
                    if r.changed() {
                        *self.poll_board.lock().unwrap() = self.board.trim().to_string();
                    }
                });
                let txt = match board_role.as_deref() {
                    Some("rx") => t("answers: the receiving end (ground)", "trả lời: đầu thu (mặt đất)").to_string(),
                    Some("tx") => t("answers: the transmitting end (aircraft)", "trả lời: đầu phát (máy bay)").to_string(),
                    Some(o) => crate::tf!("answers: {o}", "trả lời: {o}"),
                    None => t("no answer yet", "chưa trả lời").to_string(),
                };
                ui.label(egui::RichText::new(txt).small().color(th::DIM));
                ui.checkbox(&mut self.set_role, t("Put the board into the matching role", "Chuyển mạch sang vai trò tương ứng"))
                    .on_hover_text(t(
                        "Sends `role rx|tx`; the board restarts in about 10 s. Off: use the board as it is.",
                        "Gửi `role rx|tx`; mạch khởi động lại trong khoảng 10 s. Tắt: dùng mạch như hiện tại.",
                    ));
            });
            ui.add_space(12.0);
            let mut chosen = None;
            ui.columns(if wide { 2 } else { 1 }, |cols| {
                for (i, role) in [Role::Ground, Role::Aircraft].into_iter().enumerate() {
                    let ui = &mut cols[if wide { i } else { 0 }];
                    let sub = match role {
                        Role::Ground => crate::tf!("{device} shows the video", "{device} hiện hình"),
                        Role::Aircraft => crate::tf!("{device} has the camera", "{device} có camera"),
                    };
                    let suggested = board_role.as_deref() == Some(role.board_role());
                    let text = egui::RichText::new(format!("{}
{sub}", role.name())).size(17.0);
                    let big = egui::vec2(ui.available_width(), 72.0);
                    let btn = egui::Button::new(if suggested { text.color(th::GOOD) } else { text }).min_size(big);
                    if ui.add_enabled(!busy, btn).clicked() {
                        chosen = Some(role);
                    }
                    ui.add_space(8.0);
                }
            });
            if let Some(role) = chosen {
                event = Some(Event::Chosen(role));
                self.start(role);
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
        });
        event
    }
}
