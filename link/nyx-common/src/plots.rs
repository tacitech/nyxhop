//! Small egui painter widgets shared by the RX/channel GUIs.

use egui::{Color32, Pos2, Sense, Stroke, Vec2};

pub fn constellation_plot(ui: &mut egui::Ui, points: &[(f32, f32)]) {
    let side = ui.available_width().min(230.0);
    let (resp, painter) = ui.allocate_painter(Vec2::splat(side), Sense::hover());
    let rect = resp.rect;
    painter.rect_filled(rect, 4.0, Color32::from_gray(12));
    let c = rect.center();
    painter.line_segment(
        [Pos2::new(rect.left(), c.y), Pos2::new(rect.right(), c.y)],
        Stroke::new(1.0, Color32::from_gray(50)),
    );
    painter.line_segment(
        [Pos2::new(c.x, rect.top()), Pos2::new(c.x, rect.bottom())],
        Stroke::new(1.0, Color32::from_gray(50)),
    );
    let range = 1.6f32;
    let scale = side / (2.0 * range);
    for &(re, im) in points {
        if re.abs() > range || im.abs() > range {
            continue;
        }
        let p = Pos2::new(c.x + re * scale, c.y - im * scale);
        painter.circle_filled(p, 1.2, Color32::from_rgb(120, 220, 255));
    }
}

pub fn line_plot(ui: &mut egui::Ui, data: &[f32], h: f32, color: Color32, normalize: bool) {
    let w = ui.available_width().min(520.0);
    let (resp, painter) = ui.allocate_painter(Vec2::new(w, h), Sense::hover());
    let rect = resp.rect;
    painter.rect_filled(rect, 4.0, Color32::from_gray(12));
    if data.len() < 2 {
        return;
    }
    let max = if normalize {
        data.iter().cloned().fold(0.0f32, f32::max).max(1e-6)
    } else {
        1.0
    };
    let pts: Vec<Pos2> = data
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            Pos2::new(
                rect.left() + i as f32 / (data.len() - 1) as f32 * rect.width(),
                rect.bottom() - (v / max).clamp(0.0, 1.0) * (rect.height() - 6.0) - 3.0,
            )
        })
        .collect();
    for pair in pts.windows(2) {
        painter.line_segment([pair[0], pair[1]], Stroke::new(1.3, color));
    }
}

/// Sparkline scaled to [min, max].
pub fn sparkline(ui: &mut egui::Ui, data: &[f32], min: f32, max: f32, color: Color32) {
    let scaled: Vec<f32> = data.iter().map(|&v| (v - min) / (max - min)).collect();
    line_plot(ui, &scaled, 60.0, color, false);
}
