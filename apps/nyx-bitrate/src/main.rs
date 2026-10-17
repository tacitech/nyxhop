//! NyxHop bitrate meter: how many Mbit/s the radio link carries. The aircraft board sends test
//! packets at the MCS, size and channel width chosen here; the ground board counts what arrives;
//! the rate, the loss and the SNR are drawn as they come. A sweep measures every MCS at every
//! width. Without the test packets it watches the video already on the link.

#![windows_subsystem = "windows"]

mod chart;
mod link;

use egui::{Color32, RichText};
use link::{Job, Link, DIVS, MCS_NAME, PHY_MBPS, width_mhz};
use nyx_common::i18n::t;
use nyx_common::theme as th;
use nyx_common::tf;
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[cfg(windows)]
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Test,
    Video,
    Sweep,
}

#[derive(Clone, PartialEq)]
struct Cfg {
    tx: String,
    rx: String,
    mode: Mode,
    mcs: usize,
    bytes: usize,
    div: u64,
    secs: f64,
    divs: [bool; 4],
    /// the channel to measure on, MHz (0: the one the link is on)
    chan: f64,
}

impl Default for Cfg {
    fn default() -> Cfg {
        Cfg {
            tx: "192.168.0.10".into(),
            rx: "192.168.0.11".into(),
            mode: Mode::Test,
            mcs: 7,
            bytes: 2016,
            div: 1,
            secs: 3.0,
            divs: [true; 4],
            chan: 0.0,
        }
    }
}

impl Cfg {
    fn text(&self) -> String {
        format!(
            "tx {}\nrx {}\nmode {}\nmcs {}\nbytes {}\ndiv {}\nsecs {}\ndivs {}\nchan {}\n",
            self.tx,
            self.rx,
            match self.mode {
                Mode::Test => "test",
                Mode::Video => "video",
                Mode::Sweep => "sweep",
            },
            self.mcs,
            self.bytes,
            self.div,
            self.secs,
            self.divs.iter().map(|b| if *b { '1' } else { '0' }).collect::<String>(),
            self.chan
        )
    }

    fn parse(t: &str) -> Cfg {
        let mut c = Cfg::default();
        for l in t.lines() {
            let Some((k, v)) = l.split_once(' ') else { continue };
            let v = v.trim();
            match k {
                "tx" => c.tx = v.into(),
                "rx" => c.rx = v.into(),
                "mode" => c.mode = match v { "video" => Mode::Video, "sweep" => Mode::Sweep, _ => Mode::Test },
                "mcs" => c.mcs = v.parse().unwrap_or(7).min(7),
                "bytes" => c.bytes = v.parse().unwrap_or(2016).clamp(64, 4095),
                "div" => c.div = v.parse().ok().filter(|d| DIVS.contains(d)).unwrap_or(1),
                "secs" => c.secs = v.parse().unwrap_or(3.0f64).clamp(1.0, 20.0),
                "divs" if v.len() == 4 => c.divs = std::array::from_fn(|i| v.as_bytes()[i] == b'1'),
                "chan" => c.chan = v.parse().unwrap_or(0.0f64).max(0.0),
                _ => {}
            }
        }
        c
    }
}

struct App {
    c: Cfg,
    /// `--start`: the measurement chosen starts at once
    autostart: bool,
    link: Link,
    path: PathBuf,
    saved: String,
    saved_at: Instant,
    csv_note: String,
}

fn dir() -> PathBuf {
    std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into())
}

impl App {
    fn new() -> App {
        let path = dir().join("nyx-bitrate.cfg");
        let c = std::fs::read_to_string(&path).map(|t| Cfg::parse(&t)).unwrap_or_default();
        let link = Link::new(c.tx.clone(), c.rx.clone());
        let saved = c.text();
        let autostart = std::env::args().any(|a| a == "--start");
        App { c, autostart, link, path, saved, saved_at: Instant::now(), csv_note: String::new() }
    }

    fn top(&mut self, ui: &mut egui::Ui) {
        let s = self.link.sh.lock().unwrap();
        // the language keeps its room at the right end: the pills wrap before it, never under it
        ui.horizontal(|ui| {
            let w = (ui.available_width() - 210.0).max(200.0);
            ui.allocate_ui_with_layout(egui::vec2(w, 24.0), egui::Layout::left_to_right(egui::Align::Center).with_main_wrap(true), |ui| {
            ui.label(RichText::new("NyxHop").strong().size(18.0).color(th::CYAN));
            ui.label(RichText::new(t("link bitrate", "đo tốc độ link")).size(16.0).color(th::TXT));
            ui.add_space(12.0);
            th::pill(
                ui,
                &if s.tx_ok { tf!("transmitter {} MHz", "máy phát {} MHz", width_mhz(s.tx_div)) } else { t("transmitter: no answer", "máy phát: không trả lời").into() },
                if s.tx_ok { th::GOOD } else { th::BAD },
            );
            th::pill(
                ui,
                &if s.rx_ok { tf!("receiver {} MHz", "máy thu {} MHz", width_mhz(s.width_div)) } else { t("receiver: no answer", "máy thu: không trả lời").into() },
                if s.rx_ok { th::GOOD } else { th::BAD },
            );
            if s.rx_ok && s.chan_mhz > 0.0 {
                th::pill(ui, &tf!("channel {:.1} MHz", "kênh {:.1} MHz", s.chan_mhz), th::CYAN);
            }
            if s.rx_ok && s.auto_bw {
                th::pill(ui, t("width auto", "băng thông tự động"), th::DIM);
            }
            if let Some(_) = s.running {
                th::pill(ui, &format!("● {}", s.phase), th::WARN);
            }
            if !s.msg.is_empty() {
                th::pill(ui, &s.msg, th::BAD);
            }
            });
        });
    }

    fn side(&mut self, ui: &mut egui::Ui) {
        let running = self.link.running();
        th::section(ui, t("Boards", "Mạch"), true, |ui| {
            let mut changed = false;
            th::row(ui, t("Aircraft (sends)", "Máy phát (bay)"), |ui| {
                changed |= ui.add_enabled(!running, egui::TextEdit::singleline(&mut self.c.tx).desired_width(140.0)).changed();
            });
            th::row(ui, t("Ground (counts)", "Máy thu (mặt đất)"), |ui| {
                changed |= ui.add_enabled(!running, egui::TextEdit::singleline(&mut self.c.rx).desired_width(140.0)).changed();
            });
            if changed {
                let mut s = self.link.sh.lock().unwrap();
                s.tx_host = self.c.tx.trim().to_string();
                s.rx_host = self.c.rx.trim().to_string();
                s.samples.clear();
            }
        });
        th::section(ui, t("Measure", "Đo"), true, |ui| {
            let cur = match self.c.mode {
                Mode::Test => "test",
                Mode::Video => "video",
                Mode::Sweep => "sweep",
            };
            let opts = [(t("Maximum", "Tối đa"), "test"), (t("Video", "Video đang chạy"), "video"), (t("Sweep", "Quét"), "sweep")];
            if !running {
                if let Some(v) = nyx_common::ui::segmented(ui, &opts, cur) {
                    self.c.mode = match v.as_str() {
                        "video" => Mode::Video,
                        "sweep" => Mode::Sweep,
                        _ => Mode::Test,
                    };
                }
            }
            ui.label(
                RichText::new(match self.c.mode {
                    Mode::Test => t(
                        "The aircraft sends test packets at the MCS chosen, as fast as the air takes them; the ground counts what arrives.",
                        "Máy phát bắn gói thử ở MCS đã chọn, nhanh hết mức sóng chở được; máy thu đếm gói nhận được.",
                    ),
                    Mode::Video => t("The video already on the link, as the ground receives it. Nothing is sent.", "Video đang chạy trên link, như máy thu nhận. Không phát gì thêm."),
                    Mode::Sweep => t("Every MCS at every width ticked, a few seconds each, into the table.", "Mọi MCS ở mọi băng thông đã chọn, mỗi ô vài giây, ra bảng."),
                })
                .size(12.0)
                .color(th::DIM),
            );
        });
        let (width_now, table) = {
            let s = self.link.sh.lock().unwrap();
            (s.width_div, s.table.clone())
        };
        if self.c.mode != Mode::Video {
            th::section(ui, t("Channel", "Kênh đo"), true, |ui| {
                ui.add_enabled_ui(!running, |ui| {
                    let labels: Vec<String> = table.iter().map(|m| format!("{m:.1}")).collect();
                    let values: Vec<String> = table.iter().map(|m| format!("{m}")).collect();
                    let mut opts: Vec<(&str, &str)> = vec![(t("Now in use", "Đang dùng"), "0")];
                    opts.extend(labels.iter().map(String::as_str).zip(values.iter().map(String::as_str)));
                    let cur = if self.c.chan > 0.0 { format!("{}", self.c.chan) } else { "0".to_string() };
                    if let Some(v) = nyx_common::ui::segmented(ui, &opts, &cur) {
                        self.c.chan = v.parse().unwrap_or(0.0);
                    }
                    ui.label(
                        RichText::new(t(
                            "A channel WiFi uses holds the high MCS back: measure the link on a clean one.",
                            "Kênh trùng WiFi làm MCS cao không qua được: đo trên kênh sạch.",
                        ))
                        .size(11.5)
                        .color(th::DIM),
                    );
                });
            });
        }
        match self.c.mode {
            Mode::Test => {
                th::section(ui, t("Test packets", "Gói thử"), true, |ui| {
                    ui.add_enabled_ui(!running, |ui| {
                        egui::Grid::new("mcs").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                            for m in 0..8 {
                                let rate = PHY_MBPS[m] / self.c.div as f64;
                                let label = format!("MCS{m}  {}\n{:.1} Mbit/s", MCS_NAME[m], rate);
                                let sel = self.c.mcs == m;
                                let b = egui::Button::new(RichText::new(label).size(12.0).color(if sel { th::TXT } else { th::DIM }))
                                    .fill(if sel { th::CYAN.gamma_multiply(0.3) } else { th::PANEL2 })
                                    .min_size(egui::vec2(140.0, 38.0));
                                if ui.add(b).clicked() {
                                    self.c.mcs = m;
                                }
                                if m % 2 == 1 {
                                    ui.end_row();
                                }
                            }
                        });
                        ui.add_space(4.0);
                        th::row(ui, t("Width MHz", "Băng thông MHz"), |ui| {
                            let opts = [("20", "1"), ("10", "2"), ("5", "4"), ("2.5", "8")];
                            if let Some(v) = nyx_common::ui::segmented(ui, &opts, &self.c.div.to_string()) {
                                self.c.div = v.parse().unwrap_or(1);
                            }
                        });
                        self.packet_row(ui);
                    });
                });
            }
            Mode::Sweep => {
                th::section(ui, t("Sweep", "Quét"), true, |ui| {
                    ui.add_enabled_ui(!running, |ui| {
                        th::row(ui, t("Widths", "Băng thông"), |ui| {
                            for (i, d) in DIVS.iter().enumerate() {
                                ui.checkbox(&mut self.c.divs[i], format!("{} MHz", width_mhz(*d)));
                            }
                        });
                        self.packet_row(ui);
                        th::row(ui, t("Each cell", "Mỗi ô"), |ui| {
                            ui.add(egui::DragValue::new(&mut self.c.secs).range(1.0..=20.0).speed(0.1).suffix(" s"));
                        });
                        let n = self.c.divs.iter().filter(|b| **b).count() * 8;
                        ui.label(RichText::new(tf!("{} cells, about {:.0} s", "{} ô, khoảng {:.0} giây", n, n as f64 * (self.c.secs + 1.2) + 8.0)).size(12.0).color(th::DIM));
                    });
                });
            }
            Mode::Video => {}
        }
        if self.c.mode != Mode::Video {
            ui.add_space(6.0);
            let (label, col) = if running { (t("■  Stop", "■  Dừng"), th::BAD) } else { (t("▶  Start", "▶  Bắt đầu"), th::GOOD) };
            let b = egui::Button::new(RichText::new(label).strong().size(16.0).color(Color32::WHITE))
                .fill(col.gamma_multiply(0.75))
                .min_size(egui::vec2(ui.available_width(), 40.0));
            if ui.add(b).clicked() {
                if running {
                    self.link.stop();
                } else {
                    self.start();
                }
            }
            ui.label(
                RichText::new(t(
                    "While measuring, the ground holds its channel and width (Auto would read the test packets as a lost link) and puts them back after. Stop the video first: it shares the air with the test.",
                    "Trong lúc đo, máy thu giữ cố định kênh và băng thông (chế độ tự động sẽ hiểu gói thử là mất link), đo xong trả lại như cũ. Nên tắt video trước: video dùng chung sóng với gói thử.",
                ))
                .size(11.5)
                .color(th::DIM),
            );
        }
        // the SNR the MCS chosen needs (2016 B packets on the bench, 14/10: at 13.5 dB MCS4 went
        // through, MCS5 hardly; at 34 dB all of them)
        const SNR_NEED: [f32; 8] = [4.0, 6.0, 7.0, 10.0, 13.0, 16.0, 19.0, 21.0];
        if self.c.mode == Mode::Test && running {
            let last = self.link.sh.lock().unwrap().samples.back().copied();
            if let Some(smp) = last {
                if let (Some(snr), Some(loss)) = (smp.snr, smp.air_loss) {
                    let need = SNR_NEED[self.c.mcs];
                    if loss > 0.5 && snr < need {
                        let fit = (0..8).rev().find(|m| SNR_NEED[*m] <= snr).unwrap_or(0);
                        ui.label(
                            RichText::new(tf!(
                                "SNR {:.0} dB: MCS{} needs about {:.0} dB. Pick MCS{} or lower, or a cleaner channel.",
                                "SNR {:.0} dB: MCS{} cần khoảng {:.0} dB. Chọn MCS{} trở xuống, hoặc kênh sạch hơn.",
                                snr, self.c.mcs, need, fit
                            ))
                            .size(12.5)
                            .color(th::WARN),
                        );
                    }
                }
            }
        }
        if width_now != self.c.div && self.c.mode == Mode::Test && !running {
            ui.label(RichText::new(tf!("the link is on {} MHz now; Start moves it to {} MHz", "link đang ở {} MHz; Bắt đầu sẽ chuyển sang {} MHz", width_mhz(width_now), width_mhz(self.c.div))).size(11.5).color(th::WARN));
        }
        ui.add_space(8.0);
    }

    fn start(&self) {
        match self.c.mode {
            Mode::Video => {}
            Mode::Sweep => self.link.start(Job::Sweep { divs: self.c.divs, bytes: self.c.bytes, secs: self.c.secs, chan: self.c.chan }),
            Mode::Test => self.link.start(Job::Test { mcs: self.c.mcs, bytes: self.c.bytes, div: self.c.div, chan: self.c.chan }),
        }
    }

    fn packet_row(&mut self, ui: &mut egui::Ui) {
        th::row(ui, t("Packet", "Độ dài gói"), |ui| {
            ui.add(egui::DragValue::new(&mut self.c.bytes).range(64..=4095).speed(8.0).suffix(" B"));
            for b in [512usize, 2016, 4095] {
                if ui.small_button(format!("{b}")).clicked() {
                    self.c.bytes = b;
                }
            }
        });
    }

    fn central(&mut self, ui: &mut egui::Ui) {
        let now = self.link.now();
        let (samples, cells, phase_cell) = {
            let s = self.link.sh.lock().unwrap();
            let cell = s.running.and_then(|_| {
                // the cell being measured, from the phase ("5 MHz · MCS3 ...")
                let p = &s.phase;
                let d = DIVS.iter().copied().find(|d| p.starts_with(&format!("{} MHz", width_mhz(*d))))?;
                let m = p.split("MCS").nth(1)?.chars().next()?.to_digit(10)? as usize;
                Some((d, m))
            });
            (s.samples.iter().copied().collect::<Vec<_>>(), s.cells.clone(), cell)
        };
        let video = self.c.mode == Mode::Video;
        // the last second, averaged
        let last: Vec<&link::Sample> = samples.iter().rev().take(4).collect();
        let avg = |f: &dyn Fn(&link::Sample) -> f64| if last.is_empty() { 0.0 } else { last.iter().map(|s| f(s)).sum::<f64>() / last.len() as f64 };
        let rx = avg(&|s| if video { s.video } else { s.rx });
        let tx = avg(&|s| s.tx);
        let dec = avg(&|s| s.phy);
        let air = last.first().and_then(|s| s.air_loss);
        let board = last.first().and_then(|s| s.board_loss);
        let loss = last.first().and_then(|s| if video { s.vloss } else { s.loss });
        let snr = last.first().and_then(|s| s.snr);
        let peak = samples.iter().filter(|s| s.t >= now - 60.0).map(|s| if video { s.video } else { s.rx }).fold(0.0, f64::max);
        let n_cards = if video { 4.0 } else { 7.0 };
        let card_w = ((ui.available_width() - 10.0 * n_cards) / n_cards - 22.0).max(90.0);
        ui.horizontal_wrapped(|ui| {
            let card = |ui: &mut egui::Ui, title: &str, v: String, unit: &str, col: Color32| {
                th::card().show(ui, |ui| {
                    ui.set_width(card_w);
                    th::stat(ui, title, &v, unit, col);
                });
            };
            card(ui, if video { t("Video received", "Video nhận") } else { t("Received by the software", "Phần mềm nhận") }, format!("{rx:.2}"), "Mbit/s", th::CYAN);
            if video {
                card(ui, t("Loss", "Mất gói"), loss.map_or("-".into(), |l| format!("{:.1}", l * 100.0)), "%", loss.map_or(th::DIM, |l| th::grade(l as f32 * 100.0, 1.0, 10.0)));
            } else {
                card(ui, t("Decoded off the air", "Giải được trên sóng"), format!("{dec:.2}"), "Mbit/s", th::GOOD);
                card(ui, t("Sent", "Phát"), format!("{tx:.2}"), "Mbit/s", th::TXT);
                card(ui, t("Lost on the air", "Mất trên sóng"), air.map_or("-".into(), |l| format!("{:.1}", l * 100.0)), "%", air.map_or(th::DIM, |l| th::grade(l as f32 * 100.0, 1.0, 10.0)));
                card(ui, t("Lost in the receiver board", "Mất trong mạch thu"), board.map_or("-".into(), |l| format!("{:.1}", l * 100.0)), "%", board.map_or(th::DIM, |l| th::grade(l as f32 * 100.0, 1.0, 10.0)));
            }
            card(ui, "SNR", snr.map_or("-".into(), |v| format!("{v:.1}")), "dB", snr.map_or(th::DIM, |v| th::grade(v, 20.0, 10.0)));
            if video {
                card(ui, t("Peak, 1 min", "Cao nhất 1 phút"), format!("{peak:.2}"), "Mbit/s", th::GOOD);
            }
        });
        ui.add_space(6.0);
        let avail = ui.available_rect_before_wrap();
        let sweep = self.c.mode == Mode::Sweep;
        let rate_h = if sweep { avail.height() * 0.30 } else { avail.height() * 0.62 };
        let q_h = if sweep { avail.height() * 0.14 } else { avail.height() * 0.30 };
        let r1 = egui::Rect::from_min_size(avail.min, egui::vec2(avail.width(), rate_h));
        let r2 = egui::Rect::from_min_size(avail.min + egui::vec2(0.0, rate_h + 8.0), egui::vec2(avail.width(), q_h));
        chart::rate(ui, r1, &samples, now, video);
        chart::quality(ui, r2, &samples, now);
        ui.allocate_rect(egui::Rect::from_min_max(avail.min, r2.max), egui::Sense::hover());
        if sweep {
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                th::card_title(ui, t("Sweep: Mbit/s received by the software (right: decoded off the air | lost on the air)", "Quét: Mbit/s phần mềm nhận (bên phải: giải được trên sóng | mất trên sóng)"));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if !cells.is_empty() && ui.button(t("Save CSV", "Lưu CSV")).clicked() {
                        self.csv_note = save_csv(&cells);
                    }
                    if !self.csv_note.is_empty() {
                        ui.label(RichText::new(&self.csv_note).size(11.5).color(th::DIM));
                    }
                });
            });
            egui::ScrollArea::vertical().show(ui, |ui| chart::sweep(ui, &cells, self.c.divs, phase_cell));
        }
    }
}

fn save_csv(cells: &[link::Cell]) -> String {
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let p = dir().join(format!("nyx-bitrate-{stamp}.csv"));
    let mut s = String::from("width_mhz,mcs,modulation,phy_rate_mbps,decoded_mbps,received_mbps,lost_on_air_pct,lost_total_pct,snr_db\n");
    for c in cells {
        s += &format!(
            "{},{},{},{:.2},{:.3},{:.3},{:.2},{:.2},{}\n",
            width_mhz(c.div),
            c.mcs,
            MCS_NAME[c.mcs],
            PHY_MBPS[c.mcs] / c.div as f64,
            c.phy_mbps,
            c.rx_mbps,
            c.air_loss * 100.0,
            c.loss * 100.0,
            c.snr.map_or(String::new(), |v| format!("{v:.1}"))
        );
    }
    match std::fs::write(&p, s) {
        Ok(()) => p.display().to_string(),
        Err(e) => e.to_string(),
    }
}

impl eframe::App for App {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if self.autostart && self.link.sh.lock().unwrap().rx_ok {
            self.autostart = false;
            self.start();
        }
        let bar = egui::Frame::new().fill(th::PANEL).inner_margin(egui::Margin::symmetric(14, 9));
        egui::Panel::top("top").frame(bar).show(root, |ui| self.top(ui));
        let side = egui::Frame::new().fill(th::BG).inner_margin(egui::Margin::same(10));
        egui::Panel::left("side").resizable(true).default_size(340.0).frame(side).show(root, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| self.side(ui));
        });
        let page = egui::Frame::new().fill(th::BG).inner_margin(egui::Margin::same(12));
        egui::CentralPanel::default().frame(page).show(root, |ui| self.central(ui));
        if self.saved_at.elapsed() >= Duration::from_secs(2) {
            self.saved_at = Instant::now();
            let t = self.c.text();
            if t != self.saved {
                let _ = std::fs::write(&self.path, &t);
                self.saved = t;
            }
        }
        root.ctx().request_repaint_after(Duration::from_millis(100));
    }
}

// (not App::on_exit: its signature takes the GL context when another crate of the workspace
// turns eframe's `glow` on, and a workspace build then fails here)
impl Drop for App {
    fn drop(&mut self) {
        // a run left going puts the link back on its way out
        if self.link.running() {
            self.link.stop();
            let t0 = Instant::now();
            while self.link.running() && t0.elapsed() < Duration::from_secs(8) {
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

fn main() -> eframe::Result {
    nyx_common::i18n::init(&dir());
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1360.0, 860.0]).with_title("NyxHop · Bitrate"),
        ..Default::default()
    };
    eframe::run_native(
        "NyxHop Bitrate",
        options,
        Box::new(|cc| {
            th::apply(&cc.egui_ctx);
            nyx_common::i18n::fonts(&cc.egui_ctx);
            Ok(Box::new(App::new()))
        }),
    )
}
