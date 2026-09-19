//! nyx-ipcam-ctl: the PC window for a board that sends an IP camera itself.
//!
//! Talks to two consoles on the board: the camera app (nyx-ipcam, :7201) for the camera URL
//! and what goes on air, and the radio daemon (:7202) for the link, role, radio and licence
//! sections every NyxHop app shows. Nothing video passes through this PC; the picture is at
//! the ground station.
//!
//!   nyx-ipcam-ctl [--board 192.168.0.10]

use std::sync::{Arc, Mutex};

use nyx_common::boardctl::{BoardCtl, BoardState};
use nyx_common::theme as th;
use nyx_common::ui as nu;

fn main() -> eframe::Result {
    nyx_common::logging::init("ipcam-ctl");
    let opts = nyx_common::Opts::from_env();
    let board = opts.arg("--board", "192.168.0.10");
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([560.0, 900.0])
            .with_title("NyxHop · camera on the board"),
        ..Default::default()
    };
    eframe::run_native(
        "NyxHop camera on the board",
        options,
        Box::new(move |cc| {
            nu::touch_style(&cc.egui_ctx);
            Ok(Box::new(Ctl::new(board)))
        }),
    )
}

struct Ctl {
    host: Arc<Mutex<String>>,
    host_buf: String,
    /// the camera app on the board (:7201)
    app: BoardCtl,
    /// the radio daemon (:7202)
    radio: BoardCtl,
    url_buf: String,
    url_seeded: bool,
    note: String,
    form: nu::RadioForm,
    lic_buf: String,
    lic_status: String,
    ceiling_buf: String,
    onvif_buf: String,
    ceiling_seeded: bool,
}

impl Ctl {
    fn new(board: String) -> Self {
        let host = Arc::new(Mutex::new(board.clone()));
        let h1 = host.clone();
        let app = BoardCtl::spawn(Arc::new(move || format!("{}:7201", h1.lock().unwrap())), &["get", "stats"]);
        let h2 = host.clone();
        let radio = BoardCtl::spawn(
            Arc::new(move || format!("{}:7202", h2.lock().unwrap())),
            &["get", "stats", "hop status", "license", "role"],
        );
        Ctl {
            host,
            host_buf: board,
            app,
            radio,
            url_buf: String::new(),
            url_seeded: false,
            note: String::new(),
            form: nu::RadioForm::default(),
            lic_buf: String::new(),
            lic_status: String::new(),
            ceiling_buf: String::new(),
            onvif_buf: String::new(),
            ceiling_seeded: false,
        }
    }

    fn camera_section(&mut self, ui: &mut egui::Ui, a: &BoardState) {
        let g = |k: &str| a.kv.get(k).cloned().unwrap_or_default();
        let num = |k: &str| a.kv.get(k).and_then(|v| v.parse::<f32>().ok()).unwrap_or(0.0);
        th::section(ui, "Camera", true, |ui| {
            if a.connected && !self.url_seeded {
                self.url_buf = g("rtsp");
                self.url_seeded = true;
            }
            // what the board app is doing: it follows the board's role by itself
            let mode = g("mode").replace("%3D", "=");
            if !mode.is_empty() {
                let col = if mode.starts_with("sending") {
                    th::GOOD
                } else if mode.starts_with("waiting") {
                    th::WARN
                } else {
                    th::DIM
                };
                th::pill(ui, &mode, col);
            }
            th::row(ui, "Board sends", |ui| {
                let cur = if g("source") == "Rtsp" { "cam" } else { "off" };
                if let Some(v) = nu::segmented(ui, &[("The camera", "cam"), ("Nothing (PC app sends)", "off")], cur) {
                    self.app.send(if v == "cam" { "set source rtsp" } else { "set source pattern" });
                    self.app.send("save");
                    self.note = if v == "cam" {
                        "the board sends the camera whenever it is the aircraft end (saved)".into()
                    } else {
                        "the board leaves its transmit port to a PC app (saved)".into()
                    };
                }
            });
            th::row(ui, "Camera URL", |ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.url_buf)
                        .hint_text("rtsp://user:pass@192.168.0.64:554/stream1")
                        .desired_width(f32::INFINITY),
                );
            });
            ui.horizontal(|ui| {
                if nu::action(ui, "Apply") {
                    let url = self.url_buf.trim().to_string();
                    if url.contains(char::is_whitespace) {
                        self.note = "the URL cannot contain spaces".into();
                    } else {
                        self.app.send("set source rtsp");
                        self.app.send("set pass 1");
                        self.app.send(format!("set rtsp {url}"));
                        self.note = "sent to the board; the camera reconnects in a few seconds".into();
                    }
                }
                if nu::action(ui, "Save on board") {
                    self.app.send("save");
                    self.note = "saved: the board starts with this camera after a power cycle".into();
                }
                if g("persist") == "failed" {
                    self.note = "saved in RAM, but the copy to the SD card failed".into();
                }
                let paused = g("pause") == "true";
                if nu::action(ui, if paused { "Resume" } else { "Pause" }) {
                    self.app.send(format!("set pause {}", if paused { 0 } else { 1 }));
                }
            });
            if !self.note.is_empty() {
                ui.label(egui::RichText::new(&self.note).small().color(th::DIM));
            }
            let status = g("cam_status").replace("%3D", "=");
            let col = if status.starts_with("streaming") {
                th::GOOD
            } else if status.starts_with("error") {
                th::BAD
            } else {
                th::WARN
            };
            ui.label(egui::RichText::new(if status.is_empty() { "—".to_string() } else { status }).monospace().small().color(col));
            let fps = num("cam_fps");
            let gop = num("cam_gop");
            ui.horizontal_wrapped(|ui| {
                th::stat(ui, "picture", &g("cam_res"), "", th::TXT);
                ui.add_space(14.0);
                th::stat(ui, "frame rate", &format!("{fps:.0}"), "fps", th::TXT);
                ui.add_space(14.0);
                th::stat(ui, "bitrate", &format!("{:.0}", num("cam_kbps")), "kbps", th::TXT);
                ui.add_space(14.0);
                let gop_s = if fps > 0.0 && gop > 0.0 { gop / fps } else { 0.0 };
                th::stat(
                    ui,
                    "key frame every",
                    &if gop_s > 0.0 { format!("{gop_s:.1}") } else { "—".into() },
                    "s",
                    if gop_s > 2.0 { th::WARN } else { th::TXT },
                );
            });
            let gop_s = if fps > 0.0 { gop / fps } else { 0.0 };
            if gop_s > 2.0 {
                ui.label(
                    egui::RichText::new(format!(
                        "After a lost frame the ground waits for the camera's next key frame, up to {gop_s:.1} s. \
                         Set the camera's I-frame interval to about 1 s."
                    ))
                    .small()
                    .color(th::WARN),
                );
            }
        });
    }

    /// ONVIF: the camera's bitrate following the link.
    fn onvif_section(&mut self, ui: &mut egui::Ui, a: &BoardState) {
        let g = |k: &str| a.kv.get(k).cloned().unwrap_or_default();
        th::section(ui, "Camera bitrate (ONVIF)", true, |ui| {
            let adapt = g("camadapt") == "1";
            th::row(ui, "Follow the link", |ui| {
                if let Some(v) = nu::segmented(ui, &[("On", "1"), ("Off", "0")], if adapt { "1" } else { "0" }) {
                    self.app.send(format!("set camadapt {v}"));
                    self.app.send("save");
                }
            });
            let status = g("onvif_status").replace("%3D", "=");
            let col = if status.starts_with("error") {
                th::BAD
            } else if status.starts_with("profile") {
                th::GOOD
            } else {
                th::DIM
            };
            ui.label(egui::RichText::new(if status.is_empty() { "—".to_string() } else { status }).monospace().small().color(col));
            ui.horizontal_wrapped(|ui| {
                th::stat(ui, "camera set to", &g("cam_set_kbps"), "kbps", th::TXT);
                ui.add_space(14.0);
                th::stat(ui, "link wants", &g("cam_want_kbps"), "kbps", th::TXT);
                ui.add_space(14.0);
                th::stat(ui, "ceiling", &g("cam_ceiling_kbps"), "kbps", th::TXT);
            });
            if !self.ceiling_seeded && a.connected {
                self.ceiling_buf = g("cammax");
                self.onvif_buf = g("onvif");
                self.ceiling_seeded = true;
            }
            th::row(ui, "Ceiling", |ui| {
                ui.add(egui::TextEdit::singleline(&mut self.ceiling_buf).desired_width(90.0).hint_text("kbps"));
                if nu::action(ui, "Set") {
                    match self.ceiling_buf.trim().parse::<u32>() {
                        Ok(v) => {
                            self.app.send(format!("set cammax {v}"));
                            self.app.send("save");
                        }
                        Err(_) => self.note = "the ceiling is a number of kbit/s".into(),
                    }
                }
            });
            th::row(ui, "ONVIF address", |ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.onvif_buf)
                        .hint_text("auto (port 80 of the camera)")
                        .desired_width(260.0),
                );
                if nu::action(ui, "Set") {
                    let v = self.onvif_buf.trim();
                    self.app.send(format!("set onvif {}", if v.is_empty() { "auto" } else { v }));
                    self.app.send("save");
                }
            });
            ui.label(
                egui::RichText::new(
                    "The board sets the camera's bitrate limit to what the air carries now, with room for key frames. \
                     The ceiling is the most it asks for; the camera's user and password are the ones in the camera URL.",
                )
                .small()
                .color(th::DIM),
            );
        });
    }

    fn air_section(&mut self, ui: &mut egui::Ui, a: &BoardState) {
        let g = |k: &str| a.kv.get(k).cloned().unwrap_or_default();
        let num = |k: &str| a.kv.get(k).and_then(|v| v.parse::<f32>().ok()).unwrap_or(0.0);
        th::section(ui, "On air", true, |ui| {
            let radio_ok = g("connected") == "true";
            ui.horizontal(|ui| {
                if radio_ok {
                    th::pill(ui, "sending through the radio", th::GOOD);
                } else {
                    th::pill(ui, "not connected to the radio daemon", th::BAD);
                }
                if g("pass") == "1" {
                    th::pill(ui, "camera H.264 as it is", th::CYAN);
                }
            });
            let mcs = g("active_mcs").chars().filter(char::is_ascii_digit).collect::<String>();
            let snr = num("fb_snr");
            let cam = num("pass_kbps");
            let cap = num("link_cap_kbps");
            ui.horizontal_wrapped(|ui| {
                th::stat(ui, "rate", &if mcs.is_empty() { "—".into() } else { format!("MCS{mcs}") }, "", th::TXT);
                ui.add_space(14.0);
                th::stat(ui, "snr at ground", &format!("{snr:.1}"), "dB", th::grade(snr, 15.0, 8.0));
                ui.add_space(14.0);
                th::stat(ui, "sent", &format!("{:.0}", num("pass_fps")), "fps", th::TXT);
            });
            // what the camera makes against what this rate carries
            let ratio = if cap > 0.0 { cam / cap } else { 0.0 };
            let col = if ratio >= 1.0 {
                th::BAD
            } else if ratio >= 0.75 {
                th::WARN
            } else {
                th::GOOD
            };
            th::row(ui, "Camera / air", |ui| {
                ui.add(egui::ProgressBar::new(ratio.min(1.0)).fill(col).desired_width(200.0));
                ui.label(egui::RichText::new(format!("{cam:.0} of {cap:.0} kbps")).monospace());
            });
            if ratio >= 1.0 {
                ui.label(
                    egui::RichText::new(
                        "The camera sends more than the air carries at this rate: frames will be dropped. \
                         Lower the camera's bitrate.",
                    )
                    .small()
                    .color(th::BAD),
                );
            }
            egui::Grid::new("air").num_columns(2).striped(true).spacing([16.0, 4.0]).show(ui, |ui| {
                let rows = [
                    ("Frames dropped, too late", g("pass_drop_late")),
                    ("Frames dropped, waiting for a key frame", g("pass_drop_nokey")),
                    ("Key frames the ground asked for", g("pass_idr_req")),
                    ("Retransmitted blocks", g("retx")),
                    ("Blocks dropped in the queue", g("cq_drop")),
                ];
                for (k, v) in rows {
                    ui.label(egui::RichText::new(k).color(th::DIM));
                    ui.label(egui::RichText::new(if v.is_empty() { "—".into() } else { v }).monospace());
                    ui.end_row();
                }
            });
        });
    }
}

impl eframe::App for Ctl {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = root.ctx().clone();
        let a = self.app.snapshot();
        let r = self.radio.snapshot();
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(th::BG).inner_margin(egui::Margin::same(10)))
            .show(root, |ui| {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("CAMERA ON THE BOARD").monospace().size(18.0).strong().color(th::CYAN));
                });
                ui.horizontal_wrapped(|ui| {
                    let resp = ui.add(egui::TextEdit::singleline(&mut self.host_buf).desired_width(150.0).hint_text("board IP"));
                    let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if nu::action(ui, "Connect") || enter {
                        *self.host.lock().unwrap() = self.host_buf.trim().to_string();
                        self.url_seeded = false;
                        self.ceiling_seeded = false;
                    }
                    th::pill(ui, if a.connected { "camera app online" } else { "camera app offline" }, if a.connected { th::GOOD } else { th::BAD });
                    th::pill(ui, if r.connected { "radio online" } else { "radio offline" }, if r.connected { th::GOOD } else { th::BAD });
                });
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.camera_section(ui, &a);
                    self.onvif_section(ui, &a);
                    self.air_section(ui, &a);
                    nu::link_section(ui, &r, nu::Role::Aircraft, &self.radio);
                    nu::role_section(ui, &r, &self.radio);
                    nu::radio_section(ui, &r, nu::Role::Aircraft, &mut self.form, &self.radio, &mut |_| {});
                    nu::licence_section(ui, &r, None, &mut self.lic_buf, &mut self.lic_status, &self.radio);
                });
            });
        ctx.request_repaint_after(std::time::Duration::from_millis(250));
    }
}
