//! nyxhop: one app for either end of the link.
//!
//! The start screen (`nyx_common::start`, shared with the Android app) asks which end this
//! computer is and which board it talks to. The board is put into the matching role
//! (console `role tx|rx`, the daemon restarts on its own), then the ground app (nyx-rx) or
//! the aircraft app (nyx-tx) runs inside this window. Both remain available as their own
//! binaries; this is the same code hosted.
//!
//!   nyxhop                       the start screen (remembers the last choice in nyxhop.cfg)
//!   nyxhop --mode rx|tx --board 192.168.0.12 [--no-role]   straight in

use std::sync::Arc;

use nyx_common::logging::{self, log};
use nyx_common::start::{Chooser, Event, Role};
use nyx_common::Opts;

enum Mode {
    Choose,
    Rx(nyx_rx::RxApp, Arc<nyx_rx::Shared>),
    Tx(nyx_tx::TxApp, Arc<nyx_tx::Shared>),
}

struct App {
    mode: Mode,
    chooser: Chooser,
    cfg_path: std::path::PathBuf,
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
        let app = App { mode: Mode::Choose, chooser: Chooser::new(board, set_role, "this computer"), cfg_path };
        if let Some(role) = Role::from_word(&opts.arg("--mode", "")) {
            app.chooser.start(role);
        }
        app
    }

    fn save_choice(&self, role: Role) {
        let text = format!(
            "# nyxhop start screen: the last choice\nboard {}\nmode {}\nset_role {}\n",
            self.chooser.board.trim(),
            role.board_role(),
            u8::from(self.chooser.set_role)
        );
        if let Err(e) = std::fs::write(&self.cfg_path, text) {
            log(&format!("nyxhop.cfg not saved: {e}"));
        }
    }

    /// The preparation finished: build the chosen app in this window.
    fn take_over(&mut self, role: Role) {
        let channel = format!("{}:{}", self.chooser.board.trim(), role.port());
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
        self.chooser.take_over(role);
    }
}

impl eframe::App for App {
    fn ui(&mut self, root: &mut egui::Ui, frame: &mut eframe::Frame) {
        // The board changed ends (the switch in the drawer, the other computer, a
        // console): this window follows by starting over as the other end. A fresh
        // process, so every thread and port of the old screen is gone for certain.
        if let Some(other) = self.chooser.follow() {
            let board = self.chooser.board.trim().to_string();
            log(&format!("board {board} is now the {} end: starting over as {}", other.board_role(), other.label()));
            self.save_choice(other);
            if let Ok(exe) = std::env::current_exe() {
                let r = std::process::Command::new(exe).args(["--mode", other.board_role(), "--board", &board]).spawn();
                if let Err(e) = r {
                    log(&format!("could not start over: {e}"));
                }
            }
            self.chooser.leave();
            root.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
            return;
        }
        match &mut self.mode {
            Mode::Rx(app, _) => app.ui(root, frame),
            Mode::Tx(app, _) => app.ui(root, frame),
            Mode::Choose => match self.chooser.ui(root) {
                Some(Event::Chosen(role)) => self.save_choice(role),
                Some(Event::Ready(role)) => self.take_over(role),
                None => {}
            },
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
