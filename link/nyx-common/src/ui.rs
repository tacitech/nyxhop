//! v40.34: one touch-first user interface for every NyxHop app.
//!
//! The screen is the video (HUD plates in the corners, status pills at the top,
//! a gear in the top-right). The gear opens a DRAWER: on a wide screen it is a
//! right-hand column next to the video, on a narrow one it covers the screen.
//! Every control is finger-sized; nothing plots, nothing scrolls sideways.
//! nyx-rx (ground station), the phone app and nyx-tx (aircraft setup) all use
//! this module, so they look and behave the same on a PC, a tablet or an
//! embedded Linux touch screen.

use crate::boardctl::{BoardCtl, BoardState};
use crate::theme as th;
use egui::{Color32, Stroke};

/// Which end of the link this app sits on: it decides the labels and which
/// link / channel buttons make sense.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Role {
    /// Video receiver (ground station / RC): owns the channel schedule, links.
    Ground,
    /// Video transmitter (aircraft): accepts a link, sends video.
    Aircraft,
}

/// Bigger fonts and hit areas on top of the dark theme (touch screens).
pub fn touch_style(ctx: &egui::Context) {
    th::apply(ctx);
    let mut st = (*ctx.global_style()).clone();
    use egui::{FontFamily, FontId, TextStyle};
    st.text_styles.insert(TextStyle::Heading, FontId::new(22.0, FontFamily::Proportional));
    st.text_styles.insert(TextStyle::Body, FontId::new(16.0, FontFamily::Proportional));
    st.text_styles.insert(TextStyle::Button, FontId::new(16.0, FontFamily::Proportional));
    st.text_styles.insert(TextStyle::Monospace, FontId::new(15.0, FontFamily::Monospace));
    st.text_styles.insert(TextStyle::Small, FontId::new(12.5, FontFamily::Proportional));
    st.spacing.item_spacing = egui::vec2(10.0, 10.0);
    st.spacing.button_padding = egui::vec2(16.0, 9.0);
    st.spacing.interact_size = egui::vec2(48.0, 36.0);
    st.spacing.slider_width = 200.0;
    st.spacing.icon_width = 22.0;
    st.spacing.icon_width_inner = 14.0;
    ctx.set_global_style(st);
}

// ---------------------------------------------------------------- HUD

/// What the HUD shows about the link (the apps fill it from their own metrics).
#[derive(Clone, Default)]
pub struct HudData {
    pub connected: bool,
    pub live: bool,
    pub snr_db: f32,
    pub fps: f32,
    pub kbps: f32,
    pub mcs: String,
    /// top-right pills: (text, colour)
    pub pills: Vec<(String, Color32)>,
    /// centre banner (event), if any
    pub banner: Option<(String, Color32)>,
    pub title: String,
    pub empty_text: String,
    /// Draw the two corner readouts. Off leaves the video and the pills alone.
    pub plates: bool,
}

fn plate(p: &egui::Painter, at: egui::Pos2, al: egui::Align2, lines: &[(String, f32, Color32)]) {
    let pad = egui::vec2(12.0, 8.0);
    let mut w: f32 = 0.0;
    let mut h = 0.0;
    let mut gs = Vec::new();
    for (s, size, c) in lines {
        let g = p.layout_no_wrap(s.clone(), egui::FontId::monospace(*size), *c);
        w = w.max(g.size().x);
        h += g.size().y + 2.0;
        gs.push(g);
    }
    let r = al.anchor_size(at, egui::vec2(w, h) + pad * 2.0);
    p.rect_filled(r, 9.0, Color32::from_black_alpha(125));
    let mut y = r.min.y + pad.y;
    for g in gs {
        let gh = g.size().y;
        p.galley(egui::pos2(r.min.x + pad.x, y), g, Color32::WHITE);
        y += gh + 2.0;
    }
}

/// Full-area video with HUD plates; returns true when the gear was tapped.
pub fn hud(ui: &mut egui::Ui, tex: Option<&egui::TextureHandle>, d: &HudData) -> bool {
    let rect = ui.max_rect();
    let p = ui.painter().clone();
    p.rect_filled(rect, 0.0, Color32::BLACK);
    if let Some(t) = tex {
        let sz = t.size_vec2();
        let s = (rect.width() / sz.x).min(rect.height() / sz.y);
        let vr = egui::Rect::from_center_size(rect.center(), sz * s);
        p.image(
            t.id(),
            vr,
            egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
            Color32::WHITE,
        );
    } else {
        p.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            &d.empty_text,
            egui::FontId::monospace(16.0),
            th::DIM,
        );
    }
    let pad = 16.0;
    let (live_txt, live_col) = if d.live {
        ("● LIVE", th::GOOD)
    } else if d.connected {
        ("○ NO VIDEO", th::WARN)
    } else {
        ("○ NO LINK", th::BAD)
    };
    plate(
        &p,
        rect.left_top() + egui::vec2(pad, pad),
        egui::Align2::LEFT_TOP,
        &[(d.title.clone(), 17.0, th::CYAN), (live_txt.into(), 13.0, live_col)],
    );
    // pills across the top, right of the title, left of the gear
    let mut x = rect.right() - pad - 52.0;
    for (text, col) in d.pills.iter().rev() {
        let g = p.layout_no_wrap(text.clone(), egui::FontId::monospace(12.5), *col);
        let w = g.size().x + 18.0;
        let r = egui::Rect::from_min_size(egui::pos2(x - w, rect.top() + pad + 4.0), egui::vec2(w, 26.0));
        p.rect_filled(r, 13.0, Color32::from_black_alpha(140));
        p.rect_stroke(r, 13.0, Stroke::new(1.0, col.gamma_multiply(0.8)), egui::StrokeKind::Inside);
        p.galley(egui::pos2(r.min.x + 9.0, r.min.y + 5.0), g, Color32::WHITE);
        x -= w + 8.0;
    }
    if d.plates {
    plate(
        &p,
        rect.left_bottom() + egui::vec2(pad, -pad),
        egui::Align2::LEFT_BOTTOM,
        &[
            ("SIGNAL".into(), 11.0, th::DIM),
            (format!("{:.1} dB", d.snr_db), 24.0, th::grade(d.snr_db, 20.0, 12.0)),
            (format!("MCS {}", d.mcs), 12.0, th::TXT),
        ],
    );
    plate(
        &p,
        rect.right_bottom() + egui::vec2(-pad, -pad),
        egui::Align2::RIGHT_BOTTOM,
        &[
            ("VIDEO".into(), 11.0, th::DIM),
            (format!("{:.0} FPS", d.fps), 24.0, if d.live { th::GOOD } else { th::BAD }),
            (format!("{:.0} kbps", d.kbps), 12.0, th::TXT),
        ],
    );
    }
    if let Some((txt, col)) = &d.banner {
        plate(
            &p,
            egui::pos2(rect.center().x, rect.top() + pad + 56.0),
            egui::Align2::CENTER_TOP,
            &[(txt.clone(), 15.0, *col)],
        );
    }
    // gear, top-right
    let gear = egui::Rect::from_min_size(egui::pos2(rect.right() - pad - 44.0, rect.top() + pad), egui::vec2(44.0, 44.0));
    let resp = ui.interact(gear, ui.id().with("gear"), egui::Sense::click());
    p.rect_filled(gear, 22.0, Color32::from_black_alpha(if resp.hovered() { 180 } else { 120 }));
    p.text(gear.center(), egui::Align2::CENTER_CENTER, "\u{2699}", egui::FontId::proportional(24.0), th::TXT);
    resp.clicked()
}

// ---------------------------------------------------------------- drawer

/// Video + drawer layout. Wide (>= 900 px): the drawer is a right column and the
/// video keeps the rest. Narrow: the drawer covers the screen, with a Close bar.
pub fn screen(
    root: &mut egui::Ui,
    open: &mut bool,
    plates: &mut bool,
    drawer_title: &str,
    drawer: impl FnOnce(&mut egui::Ui),
    main: impl FnOnce(&mut egui::Ui) -> bool,
) {
    let wide = root.max_rect().width() >= 900.0;
    if *open && !wide {
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(th::BG).inner_margin(egui::Margin::same(10)))
            .show(root, |ui| {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(drawer_title).monospace().size(18.0).strong().color(th::CYAN));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.add(egui::Button::new("Close").min_size(egui::vec2(110.0, 38.0))).clicked() {
                            *open = false;
                        }
                        ui.checkbox(plates, "Readouts")
                            .on_hover_text("The SNR and frame-rate plates over the video");
                    });
                });
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| drawer(ui));
            });
        return;
    }
    if *open {
        egui::Panel::right("nyx_drawer")
            .resizable(false)
            // default_size, not exact_size: with an exact size egui clamps the panel back
            // over content that did not fit, and the central panel then paints black over
            // the first pixels of every label. Letting it grow keeps the drawer readable.
            .default_size((root.max_rect().width() * 0.36).clamp(420.0, 520.0))
            .frame(egui::Frame::new().fill(th::BG).inner_margin(egui::Margin::same(10)))
            .show(root, |ui| {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(drawer_title).monospace().size(18.0).strong().color(th::CYAN));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.add(egui::Button::new("Close").min_size(egui::vec2(90.0, 38.0))).clicked() {
                            *open = false;
                        }
                        ui.checkbox(plates, "Readouts")
                            .on_hover_text("The SNR and frame-rate plates over the video");
                    });
                });
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| drawer(ui));
            });
    }
    let gear = egui::CentralPanel::default()
        .frame(egui::Frame::NONE.fill(Color32::BLACK))
        .show(root, |ui| main(ui))
        .inner;
    if gear {
        *open = !*open;
    }
}

// ---------------------------------------------------------------- widgets

/// A row of big toggle buttons; returns the value tapped.
pub fn segmented(ui: &mut egui::Ui, options: &[(&str, &str)], current: &str) -> Option<String> {
    let mut hit = None;
    ui.horizontal_wrapped(|ui| {
        for (label, value) in options {
            let on = *value == current;
            let b = egui::Button::new(egui::RichText::new(*label).strong())
                .min_size(egui::vec2(64.0, 38.0))
                .fill(if on { th::CYAN.gamma_multiply(0.35) } else { th::PANEL2 })
                .stroke(Stroke::new(1.0, if on { th::CYAN } else { th::LINE }));
            if ui.add(b).clicked() {
                hit = Some((*value).to_string());
            }
        }
    });
    hit
}

/// A big action button.
pub fn action(ui: &mut egui::Ui, label: &str) -> bool {
    ui.add(egui::Button::new(label).min_size(egui::vec2(120.0, 38.0))).clicked()
}

fn num(s: &str) -> Option<f32> {
    s.split_whitespace().next().and_then(|x| x.parse::<f32>().ok())
}

/// Editable copy of the board's tuning (seeded once from what it reports).
#[derive(Clone, Default)]
pub struct RadioForm {
    pub video_lo: String,
    pub video_gain: f32,
    pub ctl_lo: String,
    pub ctl_gain: f32,
    pub rmin: String,
    pub seeded: bool,
}

impl RadioForm {
    pub fn seed(&mut self, st: &BoardState, role: Role) {
        if self.seeded {
            return;
        }
        let (vf, vg, cf, cg) = match role {
            Role::Ground => ("rxfreq", "rxgain", "txfreq", "txgain"),
            Role::Aircraft => ("txfreq", "txgain", "rxfreq", "rxgain"),
        };
        if let Some(v) = st.kv.get(vf).and_then(|v| v.parse::<f64>().ok()) {
            self.video_lo = format!("{}", v / 1e6);
            self.seeded = true;
        }
        if let Some(g) = st.kv.get(vg).and_then(|s| num(s)) {
            self.video_gain = g;
        }
        if let Some(v) = st.kv.get(cf).and_then(|v| v.parse::<f64>().ok()) {
            self.ctl_lo = format!("{}", v / 1e6);
        }
        if let Some(g) = st.kv.get(cg).and_then(|s| num(s)) {
            self.ctl_gain = g;
        }
        if let Some(r) = st.kv.get("rmin") {
            self.rmin = r.clone();
        }
    }
}

/// Link: pair state + the buttons this end has (Ground: Link / Unlink;
/// Aircraft: Accept link / Unlink).
pub fn link_section(ui: &mut egui::Ui, st: &BoardState, role: Role, board: &BoardCtl) {
    th::section(ui, "Link", true, |ui| {
        let ls = st.kv.get("hop_link").cloned().unwrap_or_else(|| "?".into());
        let (txt, col) = if ls.starts_with("bound") {
            (format!("Linked  ·  pair {}", ls.trim_start_matches("bound:")), th::GOOD)
        } else if ls.starts_with("binding") {
            (format!("Linking…  {}", ls.trim_start_matches("binding:")), th::WARN)
        } else if ls.starts_with("accepting") {
            (format!("Accepting…  {}", ls.trim_start_matches("accepting:")), th::WARN)
        } else {
            ("Not linked".to_string(), th::BAD)
        };
        ui.horizontal(|ui| {
            th::pill(ui, &txt, col);
            if st.connected {
                th::pill(ui, "radio online", th::GOOD);
            } else {
                th::pill(ui, "radio offline", th::BAD);
            }
        });
        ui.horizontal(|ui| match role {
            Role::Ground => {
                if ui
                    .add(egui::Button::new("Link aircraft").min_size(egui::vec2(150.0, 40.0)))
                    .on_hover_text("New pair key, sent on the link channel for 20 s. The aircraft takes it when unlinked, or while Accept link runs on it.")
                    .clicked()
                {
                    board.send("link bind 20".to_string());
                }
                if action(ui, "Unlink") {
                    board.send("link unbind".to_string());
                }
            }
            Role::Aircraft => {
                if ui
                    .add(egui::Button::new("Accept link (60 s)").min_size(egui::vec2(170.0, 40.0)))
                    .on_hover_text("Take the next pair key a ground station sends.")
                    .clicked()
                {
                    board.send("link accept 60".to_string());
                }
                if action(ui, "Unlink") {
                    board.send("link unbind".to_string());
                }
            }
        });
    });
}

/// Channel policy (ground end only): Off = one channel you pick, Auto = the receiver picks.
/// The tables (video channels, control pool) are the operator's: any MHz between 70 and 6000,
/// edited here; the aircraft receives them over the air.
pub fn channel_section(ui: &mut egui::Ui, st: &BoardState, f: &mut ChannelForm, board: &BoardCtl) {
    th::section(ui, "Channel", true, |ui| {
        let mode = st.kv.get("hop_mode").cloned().unwrap_or_default();
        let auto = matches!(
            mode.as_str(),
            "auto" | "time" | "auto24" | "auto58" | "autofull" | "band24" | "band58" | "full"
        );
        let cur = if auto { "auto" } else { "manual" };
        let hz = st.kv.get("hop_cur_hz").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
        if let Some(v) = segmented(ui, &[("Off", "manual"), ("Auto", "auto")], cur) {
            if v == "auto" {
                board.send("hop auto rx 1000".to_string());
            } else {
                board.send("hop manual rx".to_string());
            }
        }
        ui.add_space(4.0);
        let table: Vec<String> = st
            .kv
            .get("hop_table")
            .map(|t| t.split(',').filter(|s| !s.is_empty()).map(|s| s.trim().to_string()).collect())
            .unwrap_or_default();
        if auto {
            let hold = st.kv.get("hop_hold_hz").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
            let active = st.kv.get("hop_active").cloned().unwrap_or_default();
            let txt = if hold > 0 {
                format!("holding {} MHz", hold / 1_000_000)
            } else if hz > 0 {
                format!("scanning {active} channels   ·   now {} MHz", hz / 1_000_000)
            } else {
                "scanning".to_string()
            };
            ui.label(egui::RichText::new(txt).monospace().small().color(th::DIM));
        } else if !table.is_empty() {
            // Off = a fixed channel of the table - pick it here; the aircraft follows
            let cur_mhz = (hz / 1_000_000).to_string();
            let opts: Vec<(&str, &str)> = table.iter().map(|m| (m.as_str(), m.as_str())).collect();
            if let Some(m) = segmented(ui, &opts, &cur_mhz) {
                board.send(format!("hop manual {m}"));
            }
            if hz > 0 {
                ui.label(egui::RichText::new(format!("now {} MHz", hz / 1_000_000)).monospace().small().color(th::DIM));
            }
        }
        // ---- the tables (v40.37) ----
        f.seed(st);
        ui.add_space(6.0);
        let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
        // Right to left: the button takes its own width and the box fills the rest, so the
        // row cannot come out wider than the drawer (which egui would answer by clipping
        // the labels of every row - see `screen`).
        th::row(ui, "Video MHz", |ui| {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let go = ui.button("Apply").clicked();
                let r = ui.add(egui::TextEdit::singleline(&mut f.video)
                    .desired_width(ui.available_width())
                    .hint_text("5735,5755,5775 …"));
                if (r.lost_focus() && enter) || go {
                    board.send(format!("hop chans {}", f.video.trim()));
                }
            });
        });
        th::row(ui, "Control MHz", |ui| {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let go = ui.button("Apply").clicked();
                let r = ui.add(egui::TextEdit::singleline(&mut f.ctl)
                    .desired_width(ui.available_width())
                    .hint_text("2412,2432 …"));
                if (r.lost_focus() && enter) || go {
                    board.send(format!("hop ctlchans {}", f.ctl.trim()));
                }
            });
        });
        if let Some(p) = st.kv.get("hop_ctl_pending").filter(|p| p.as_str() != "-") {
            ui.label(egui::RichText::new(format!("control pool switching: {p}")).small().color(th::WARN));
        }
        ui.label(
            egui::RichText::new(
                "Off = one channel you pick, the aircraft follows · Auto = the receiver holds the best channel of the table and re-scans by hopping when it degrades or the link drops; the aircraft hops over the whole table on the shared clock while it cannot hear the receiver. Tables: any MHz from 70 to 6000, video first then control (the pool keeps 20 MHz clear of every video channel); the aircraft gets them over the air.",
            )
            .small()
            .color(th::DIM),
        );
    });
}

/// v40.37: the channel-table editor's text, seeded from the radio and kept while typing.
#[derive(Default)]
pub struct ChannelForm {
    pub video: String,
    pub ctl: String,
    seen_video: String,
    seen_ctl: String,
}

impl ChannelForm {
    fn seed(&mut self, st: &BoardState) {
        let v = st.kv.get("hop_table").cloned().unwrap_or_default();
        if v != self.seen_video {
            self.seen_video = v.clone();
            self.video = v;
        }
        let c = st.kv.get("hop_ctl_table").cloned().unwrap_or_default();
        if c != self.seen_ctl {
            self.seen_ctl = c.clone();
            self.ctl = c;
        }
    }
}

/// Radio: video LO/gain, control LO/gain, antenna ports, sample rate, AGC, trigger.
pub fn radio_section(ui: &mut egui::Ui, st: &BoardState, role: Role, f: &mut RadioForm, board: &BoardCtl, on_rate: &mut dyn FnMut(u64)) {
    th::section(ui, "Radio", false, |ui| {
        f.seed(st, role);
        let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
        let (video_lbl, ctl_lbl, vcmd, vgcmd, ccmd, cgcmd, vrange, crange) = match role {
            Role::Ground => ("Video RX", "Control TX", "rxfreq", "rxgain", "txfreq", "txgain", 0.0..=73.0, -89.0..=0.0),
            Role::Aircraft => ("Video TX", "Control RX", "txfreq", "txgain", "rxfreq", "rxgain", -89.0..=0.0, 0.0..=73.0),
        };
        let g = |k: &str| st.kv.get(k).cloned().unwrap_or_else(|| "—".into());
        ui.label(egui::RichText::new(format!("rms {}   peak {}   rssi {}", g("rms"), g("peak"), g("rssi"))).monospace().small().color(th::DIM));
        if !st.verdict.is_empty() {
            let c = if st.verdict.contains("PREAMBLE") { th::GOOD } else { th::WARN };
            ui.label(egui::RichText::new(&st.verdict).small().color(c));
        }
        th::row(ui, &format!("{video_lbl} MHz"), |ui| {
            let r = ui.add(egui::TextEdit::singleline(&mut f.video_lo).desired_width(90.0));
            if (r.lost_focus() && enter) || ui.button("Set").clicked() {
                if let Ok(m) = f.video_lo.trim().parse::<f64>() {
                    board.send(format!("{vcmd} {}", (m * 1e6).round() as u64));
                }
            }
        });
        th::row(ui, "gain dB", |ui| {
            let r = ui.add(egui::Slider::new(&mut f.video_gain, vrange));
            if r.drag_stopped() {
                board.send(format!("{vgcmd} {}", f.video_gain.round() as i32));
            }
            if role == Role::Ground {
                let mut agc = st.kv.get("softagc").map(|v| v == "1").unwrap_or(false);
                if ui.checkbox(&mut agc, "AGC").on_hover_text("Daemon auto-gain loop (overrides the slider)").changed() {
                    board.send(format!("softagc {}", u8::from(agc)));
                }
            }
        });
        th::row(ui, &format!("{ctl_lbl} MHz"), |ui| {
            let r = ui.add(egui::TextEdit::singleline(&mut f.ctl_lo).desired_width(90.0));
            if (r.lost_focus() && enter) || ui.button("Set").clicked() {
                if let Ok(m) = f.ctl_lo.trim().parse::<f64>() {
                    board.send(format!("{ccmd} {}", (m * 1e6).round() as u64));
                }
            }
        });
        th::row(ui, "gain dB", |ui| {
            let r = ui.add(egui::Slider::new(&mut f.ctl_gain, crange));
            if r.drag_stopped() {
                board.send(format!("{cgcmd} {}", f.ctl_gain.round() as i32));
            }
            if role == Role::Aircraft {
                let mut agc = st.kv.get("softagc").map(|v| v == "1").unwrap_or(false);
                if ui.checkbox(&mut agc, "AGC").changed() {
                    board.send(format!("softagc {}", u8::from(agc)));
                }
            }
        });
        // Antenna ports: only a board that reports them (2R2T bitstream).
        if st.kv.contains_key("txsel") || st.kv.contains_key("rxsel") {
            let txs = st.kv.get("txsel").cloned().unwrap_or_default();
            let rxs = st.kv.get("rxsel").cloned().unwrap_or_default();
            let ant = |v: &str| match v { "1" => "SMA", "2" => "U.FL", "both" => "both", _ => "?" };
            th::row(ui, "TX port", |ui| {
                if let Some(v) = segmented(ui, &[("SMA", "1"), ("U.FL", "2"), ("both", "both"), ("auto", "auto")], &txs) {
                    board.send(format!("txsel {v}"));
                }
                if txs == "auto" {
                    ui.weak(format!("({})", ant(&st.kv.get("txant").cloned().unwrap_or_default())));
                }
            });
            th::row(ui, "RX port", |ui| {
                if let Some(v) = segmented(ui, &[("SMA", "1"), ("U.FL", "2"), ("auto", "auto")], &rxs) {
                    board.send(format!("rxsel {v}"));
                }
                if rxs == "auto" {
                    ui.weak(format!("({})", ant(&st.kv.get("rxant").cloned().unwrap_or_default())));
                }
            });
        }
        th::row(ui, "Sample rate", |ui| {
            let fs_now = st.kv.get("fs").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
            let cur = fs_now.to_string();
            if let Some(v) = segmented(ui, &[("15.36M", "15360000"), ("30.72M", "30720000")], &cur) {
                let hz: u64 = v.parse().unwrap_or(15_360_000);
                let bw = if hz == 30_720_000 { 24_000_000u64 } else { 14_000_000 };
                board.send(format!("fs {hz}"));
                board.send(format!("bw {bw}"));
                on_rate(hz);
            }
            let mut fir = st.kv.get("fir").map(|v| v == "1").unwrap_or(false);
            if ui.checkbox(&mut fir, "FIR").on_hover_text("AD9361 programmable FIR (30.72M needs fir20MHz.ftr loaded first).").changed() {
                board.send(format!("firen {}", u8::from(fir)));
            }
        });
        th::row(ui, "AGC mode", |ui| {
            let cur = st.kv.get("agc").cloned().unwrap_or_default();
            let cur = ["manual", "slow", "fast"].iter().find(|m| cur.starts_with(*m)).copied().unwrap_or("manual");
            if let Some(v) = segmented(ui, &[("manual", "manual"), ("slow", "slow"), ("fast", "fast")], cur) {
                board.send(format!("agc {v}"));
            }
        });
        th::row(ui, "Trigger rmin", |ui| {
            let r = ui.add(egui::TextEdit::singleline(&mut f.rmin).desired_width(70.0));
            if (r.lost_focus() && enter) || ui.button("Set").clicked() {
                if !f.rmin.trim().is_empty() {
                    board.send(format!("trig rmin {}", f.rmin.trim()));
                }
            }
            ui.label(egui::RichText::new(format!("caps {}  suppr {}", g("captures"), g("suppressed"))).monospace().small().color(th::DIM));
        });
        ui.label(
            egui::RichText::new(match role {
                Role::Ground => "Both ends must use the same sample rate. RX = video down, TX = control up (FDD).",
                Role::Aircraft => "Both ends must use the same sample rate. TX = video down, RX = control up (FDD).",
            })
            .small()
            .color(th::DIM),
        );
    });
}

/// Licence: this board (and the aircraft when its lines arrive in-band).
pub fn licence_section(ui: &mut egui::Ui, st: &BoardState, far: Option<&str>, buf: &mut String, status: &mut String, board: &BoardCtl) {
    th::section(ui, "Licence", true, |ui| {
        crate::licpanel::lic_rows(ui, &st.kv, far, buf, status, &mut |cmd| board.send(cmd));
    });
}

/// Connection to the board: address + Connect + transport. Returns true when
/// Connect was pressed (the app then applies `addr`).
pub fn connection_section(ui: &mut egui::Ui, addr: &mut String, hint: &str, connected: bool, udp: Option<&mut bool>) -> bool {
    let mut connect = false;
    th::section(ui, "Connection", false, |ui| {
        th::row(ui, "Board", |ui| {
            let r = ui.add(egui::TextEdit::singleline(addr).desired_width(170.0).hint_text(hint));
            let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if action(ui, "Connect") || enter {
                connect = true;
            }
        });
        ui.horizontal(|ui| {
            if connected {
                th::pill(ui, "connected", th::GOOD);
            } else {
                th::pill(ui, "connecting…", th::WARN);
            }
        });
        if let Some(udp) = udp {
            ui.checkbox(udp, "UDP transport")
                .on_hover_text("Connectionless, survives a daemon restart. Off = TCP with backpressure.");
        }
    });
    connect
}

/// Two-way text over the air. Returns the line to send, if any.
pub fn messages_section(ui: &mut egui::Ui, msgs: &[String], buf: &mut String) -> Option<String> {
    let mut out = None;
    th::section(ui, "Messages", false, |ui| {
        egui::ScrollArea::vertical().id_salt("nyx_msgs").max_height(120.0).stick_to_bottom(true).show(ui, |ui| {
            for m in msgs {
                ui.label(m);
            }
        });
        ui.horizontal(|ui| {
            let r = ui.add(egui::TextEdit::singleline(buf).desired_width(ui.available_width() - 100.0).hint_text("type a message…"));
            let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if (action(ui, "Send") || enter) && !buf.trim().is_empty() {
                out = Some(buf.trim().to_string());
                buf.clear();
            }
        });
    });
    out
}

/// Colour + text of the licence pill from the board's lic_* lines.
pub fn licence_pill(st: &BoardState) -> (String, Color32) {
    let s = st.kv.get("lic_state").cloned().unwrap_or_else(|| "?".into());
    match s.as_str() {
        "licensed" => ("licensed".into(), th::GOOD),
        "trial" => {
            let used: i64 = st.kv.get("lic_min").and_then(|v| v.parse().ok()).unwrap_or(0);
            let total: i64 = st.kv.get("lic_trial").and_then(|v| v.parse().ok()).unwrap_or(0);
            let left = (total - used).max(0);
            let txt = if left >= 120 {
                format!("trial · {} h left", left / 60)
            } else {
                format!("trial · {left} min left")
            };
            // quiet while there is plenty, loud only near the end
            let col = if left <= 30 {
                th::BAD
            } else if left <= 120 {
                th::WARN
            } else {
                th::TXT
            };
            (txt, col)
        }
        "locked" => ("LOCKED · no licence".into(), th::BAD),
        "nogate" => ("no licence gate".into(), th::DIM),
        _ => ("licence ?".into(), th::DIM),
    }
}

/// Colour + text of the link pill.
pub fn link_pill(st: &BoardState) -> (String, Color32) {
    let ls = st.kv.get("hop_link").cloned().unwrap_or_default();
    if ls.starts_with("bound") {
        ("linked".into(), th::GOOD)
    } else if ls.starts_with("binding") || ls.starts_with("accepting") {
        ("linking…".into(), th::WARN)
    } else if !st.connected {
        ("radio offline".into(), th::BAD)
    } else {
        ("not linked".into(), th::BAD)
    }
}

/// Text of the channel-mode pill (ground end).
pub fn mode_pill(st: &BoardState) -> (String, Color32) {
    let m = st.kv.get("hop_mode").cloned().unwrap_or_default();
    let hz = st.kv.get("hop_cur_hz").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
    let hold = st.kv.get("hop_hold_hz").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
    let auto = matches!(
        m.as_str(),
        "auto" | "time" | "auto24" | "auto58" | "autofull" | "band24" | "band58" | "full"
    );
    let txt = if auto {
        if hold > 0 {
            format!("Auto · {} MHz", hold / 1_000_000)
        } else if hz > 0 {
            format!("Auto · scan {} MHz", hz / 1_000_000)
        } else {
            "Auto".to_string()
        }
    } else if hz > 0 {
        format!("Fixed · {} MHz", hz / 1_000_000)
    } else {
        "Fixed".to_string()
    };
    (txt, th::CYAN)
}
