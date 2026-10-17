//! The plots: the received rate over the last minute (an area, the sent rate as a line over it),
//! the SNR and the loss under it; the sweep as a grid of bars.

use crate::link::{Cell, Sample, DIVS, MCS_NAME, PHY_MBPS, width_mhz};
use egui::{Align2, FontId, Pos2, Rect, Shape, Stroke, Vec2};
use nyx_common::i18n::t;
use nyx_common::theme as th;

const SPAN_S: f64 = 60.0;
/// the loss inside the receiving board (between its receiver and its processor)
const BOARD: egui::Color32 = egui::Color32::from_rgb(190, 130, 255);

/// A round top for an axis (1, 2, 5 x 10^n).
fn nice(v: f64) -> f64 {
    let v = v.max(1e-3);
    let e = 10f64.powf(v.log10().floor());
    for m in [1.0, 2.0, 2.5, 5.0, 10.0] {
        if m * e >= v {
            return m * e;
        }
    }
    10.0 * e
}

fn frame(ui: &egui::Ui, rect: Rect) {
    let p = ui.painter();
    p.rect_filled(rect, 9.0, th::PANEL2);
    p.rect_stroke(rect, 9.0, Stroke::new(1.0, th::LINE), egui::StrokeKind::Inside);
}

/// The rate plot: `video` instead of the test packets' rate when watching the video.
pub fn rate(ui: &mut egui::Ui, rect: Rect, s: &[Sample], now: f64, video: bool) {
    frame(ui, rect);
    let p = ui.painter_at(rect);
    let plot = Rect::from_min_max(rect.min + Vec2::new(52.0, 14.0), rect.max - Vec2::new(14.0, 26.0));
    let val = |x: &Sample| if video { x.video } else { x.rx };
    let vis: Vec<&Sample> = s.iter().filter(|x| x.t >= now - SPAN_S).collect();
    let top = nice(vis.iter().map(|x| val(x).max(if video { 0.0 } else { x.tx.max(x.phy) })).fold(1.0, f64::max) * 1.1);
    let small = FontId::monospace(11.0);
    for i in 0..=4 {
        let y = plot.bottom() - plot.height() * i as f32 / 4.0;
        p.line_segment([Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)], Stroke::new(1.0, th::LINE));
        let v = top * i as f64 / 4.0;
        let lab = if top >= 10.0 { format!("{v:.0}") } else { format!("{v:.1}") };
        p.text(Pos2::new(plot.left() - 8.0, y), Align2::RIGHT_CENTER, lab, small.clone(), th::DIM);
    }
    for k in 0..=6 {
        let x = plot.left() + plot.width() * k as f32 / 6.0;
        p.text(Pos2::new(x, plot.bottom() + 6.0), Align2::CENTER_TOP, format!("-{}s", 60 - k * 10), small.clone(), th::DIM);
    }
    p.text(Pos2::new(plot.left() + 6.0, plot.top() + 2.0), Align2::LEFT_TOP, "Mbit/s", small.clone(), th::DIM);
    let fx = |t: f64| plot.left() + ((t - (now - SPAN_S)) / SPAN_S) as f32 * plot.width();
    let fy = |v: f64| plot.bottom() - (v / top) as f32 * plot.height();
    if vis.len() >= 2 {
        // the received rate as an area
        let mut poly: Vec<Pos2> = vis.iter().map(|x| Pos2::new(fx(x.t), fy(val(x)))).collect();
        let line = poly.clone();
        poly.push(Pos2::new(fx(vis.last().unwrap().t), plot.bottom()));
        poly.push(Pos2::new(fx(vis[0].t), plot.bottom()));
        // (as a mesh of quads: egui's convex polygon would fill a jagged line wrong)
        let mut mesh = egui::epaint::Mesh::default();
        let fill = th::CYAN.gamma_multiply(0.22);
        for w in line.windows(2) {
            let i = mesh.vertices.len() as u32;
            mesh.colored_vertex(w[0], fill);
            mesh.colored_vertex(w[1], fill);
            mesh.colored_vertex(Pos2::new(w[1].x, plot.bottom()), fill);
            mesh.colored_vertex(Pos2::new(w[0].x, plot.bottom()), fill);
            mesh.add_triangle(i, i + 1, i + 2);
            mesh.add_triangle(i, i + 2, i + 3);
        }
        p.add(Shape::mesh(mesh));
        p.add(Shape::line(line, Stroke::new(2.0, th::CYAN)));
        if !video {
            let dec: Vec<Pos2> = vis.iter().map(|x| Pos2::new(fx(x.t), fy(x.phy))).collect();
            p.add(Shape::line(dec, Stroke::new(1.4, th::GOOD)));
            let sent: Vec<Pos2> = vis.iter().map(|x| Pos2::new(fx(x.t), fy(x.tx))).collect();
            p.add(Shape::dashed_line(&sent, Stroke::new(1.2, th::DIM), 6.0, 4.0));
        }
    }
    // the legend
    let lx = plot.right() - 8.0;
    p.text(Pos2::new(lx, plot.top() + 2.0), Align2::RIGHT_TOP, if video { t("video received", "video nhận") } else { t("received by the software", "phần mềm nhận") }, small.clone(), th::CYAN);
    if !video {
        p.text(Pos2::new(lx, plot.top() + 16.0), Align2::RIGHT_TOP, t("decoded off the air", "giải được trên sóng"), small.clone(), th::GOOD);
        p.text(Pos2::new(lx, plot.top() + 30.0), Align2::RIGHT_TOP, t("- - sent", "- - phát"), small, th::DIM);
    }
}

/// SNR (left scale, 0..40 dB) and the loss (right scale, 0..100 %).
pub fn quality(ui: &mut egui::Ui, rect: Rect, s: &[Sample], now: f64) {
    frame(ui, rect);
    let p = ui.painter_at(rect);
    let plot = Rect::from_min_max(rect.min + Vec2::new(52.0, 12.0), rect.max - Vec2::new(44.0, 12.0));
    let small = FontId::monospace(11.0);
    for i in 0..=2 {
        let y = plot.bottom() - plot.height() * i as f32 / 2.0;
        p.line_segment([Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)], Stroke::new(1.0, th::LINE));
        p.text(Pos2::new(plot.left() - 8.0, y), Align2::RIGHT_CENTER, format!("{}", 20 * i), small.clone(), th::WARN);
        p.text(Pos2::new(plot.right() + 8.0, y), Align2::LEFT_CENTER, format!("{}%", 50 * i), small.clone(), th::BAD);
    }
    p.text(Pos2::new(plot.left() + 6.0, plot.top()), Align2::LEFT_TOP, "SNR dB", small.clone(), th::WARN);
    p.text(Pos2::new(plot.right() - 6.0, plot.top()), Align2::RIGHT_TOP, t("lost on the air", "mất trên sóng"), small.clone(), th::BAD);
    p.text(Pos2::new(plot.right() - 6.0, plot.top() + 14.0), Align2::RIGHT_TOP, t("lost in the receiving board", "mất trong mạch thu"), small, BOARD);
    let fx = |t: f64| plot.left() + ((t - (now - SPAN_S)) / SPAN_S) as f32 * plot.width();
    let snr: Vec<Pos2> = s
        .iter()
        .filter(|x| x.t >= now - SPAN_S)
        .filter_map(|x| x.snr.map(|v| Pos2::new(fx(x.t), plot.bottom() - (v.clamp(0.0, 40.0) / 40.0) * plot.height())))
        .collect();
    if snr.len() >= 2 {
        p.add(Shape::line(snr, Stroke::new(1.6, th::WARN)));
    }
    let loss: Vec<Pos2> = s
        .iter()
        .filter(|x| x.t >= now - SPAN_S)
        .filter_map(|x| x.air_loss.or(x.vloss).map(|v| Pos2::new(fx(x.t), plot.bottom() - v as f32 * plot.height())))
        .collect();
    if loss.len() >= 2 {
        p.add(Shape::line(loss, Stroke::new(1.6, th::BAD)));
    }
    let board: Vec<Pos2> = s
        .iter()
        .filter(|x| x.t >= now - SPAN_S)
        .filter_map(|x| x.board_loss.map(|v| Pos2::new(fx(x.t), plot.bottom() - v as f32 * plot.height())))
        .collect();
    if board.len() >= 2 {
        p.add(Shape::line(board, Stroke::new(1.4, BOARD)));
    }
}

/// The sweep: MCS down, widths across, each cell its rate as a bar (coloured by its loss) and the
/// number; the best cell ringed.
pub fn sweep(ui: &mut egui::Ui, cells: &[Cell], divs: [bool; 4], running: Option<(u64, usize)>) {
    let cols: Vec<u64> = DIVS.iter().enumerate().filter(|(i, _)| divs[*i]).map(|(_, d)| *d).collect();
    if cols.is_empty() {
        return;
    }
    let max = cells.iter().map(|c| c.rx_mbps).fold(0.1, f64::max);
    let best = cells.iter().max_by(|a, b| a.rx_mbps.total_cmp(&b.rx_mbps)).map(|c| (c.div, c.mcs));
    let w = ui.available_width();
    let head_w = 150.0;
    let col_w = ((w - head_w) / cols.len() as f32).max(80.0);
    let row_h = 30.0;
    let (rect, _) = ui.allocate_exact_size(Vec2::new(w, row_h * 9.0 + 8.0), egui::Sense::hover());
    let p = ui.painter_at(rect);
    let small = FontId::monospace(11.5);
    for (j, d) in cols.iter().enumerate() {
        let x = rect.left() + head_w + col_w * j as f32;
        p.text(Pos2::new(x + col_w / 2.0, rect.top() + row_h / 2.0), Align2::CENTER_CENTER, format!("{} MHz", width_mhz(*d)), FontId::proportional(13.5), th::TXT);
    }
    for mcs in 0..8 {
        let y = rect.top() + row_h * (mcs + 1) as f32;
        p.text(Pos2::new(rect.left() + 4.0, y + row_h / 2.0), Align2::LEFT_CENTER, format!("MCS{mcs} {}", MCS_NAME[mcs]), small.clone(), th::DIM);
        for (j, d) in cols.iter().enumerate() {
            let x = rect.left() + head_w + col_w * j as f32;
            let cell = Rect::from_min_size(Pos2::new(x + 4.0, y + 3.0), Vec2::new(col_w - 8.0, row_h - 6.0));
            p.rect_filled(cell, 5.0, th::PANEL2);
            if running == Some((*d, mcs)) {
                p.rect_stroke(cell, 5.0, Stroke::new(1.5, th::CYAN), egui::StrokeKind::Inside);
            }
            let Some(c) = cells.iter().find(|c| c.div == *d && c.mcs == mcs) else { continue };
            let col = if c.loss < 0.01 { th::GOOD } else if c.loss < 0.10 { th::WARN } else { th::BAD };
            let bar = Rect::from_min_size(cell.min, Vec2::new(cell.width() * (c.rx_mbps / max) as f32, cell.height()));
            p.rect_filled(bar, 5.0, col.gamma_multiply(0.35));
            let phy = PHY_MBPS[mcs] / *d as f64;
            p.text(
                Pos2::new(cell.left() + 6.0, cell.center().y),
                Align2::LEFT_CENTER,
                format!("{:.2}", c.rx_mbps),
                FontId::monospace(12.5),
                th::TXT,
            );
            let _ = phy;
            p.text(
                Pos2::new(cell.right() - 6.0, cell.center().y),
                Align2::RIGHT_CENTER,
                format!("{:.1} | {:.0}%", c.phy_mbps, c.air_loss * 100.0),
                FontId::monospace(10.5),
                if c.air_loss < 0.01 { th::DIM } else if c.air_loss < 0.10 { th::WARN } else { th::BAD },
            );
            if best == Some((c.div, c.mcs)) {
                p.rect_stroke(cell, 5.0, Stroke::new(2.0, th::GOOD), egui::StrokeKind::Inside);
            }
        }
    }
}
