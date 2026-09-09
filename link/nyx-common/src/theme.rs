//! Shared theme for the nyx-tx / nyx-rx GUIs: dark "ground station" (colours in step with the web
//! dashboard). Call `apply(ctx)` once when the App is created; wrap widget groups in `card()` +
//! `card_title()`; big figures use `big_value()`.
use egui::{Color32, CornerRadius, Margin, Stroke};

pub const BG: Color32 = Color32::from_rgb(10, 14, 20);
pub const PANEL: Color32 = Color32::from_rgb(16, 23, 34);
pub const PANEL2: Color32 = Color32::from_rgb(13, 19, 29);
pub const LINE: Color32 = Color32::from_rgb(28, 38, 52);
pub const TXT: Color32 = Color32::from_rgb(219, 228, 240);
pub const DIM: Color32 = Color32::from_rgb(109, 125, 149);
pub const GOOD: Color32 = Color32::from_rgb(53, 217, 154);
pub const WARN: Color32 = Color32::from_rgb(255, 180, 84);
pub const BAD: Color32 = Color32::from_rgb(255, 93, 93);
pub const CYAN: Color32 = Color32::from_rgb(74, 168, 255);

/// Colour by threshold: good/bad like a status lamp.
pub fn grade(v: f32, good_from: f32, warn_from: f32) -> Color32 {
    // good_from > warn_from: high is good (SNR). Otherwise low is good (BLER).
    if good_from >= warn_from {
        if v >= good_from {
            GOOD
        } else if v >= warn_from {
            WARN
        } else {
            BAD
        }
    } else if v <= good_from {
        GOOD
    } else if v <= warn_from {
        WARN
    } else {
        BAD
    }
}

pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    let mut st = (*ctx.global_style()).clone();
    let v = &mut st.visuals;
    *v = egui::Visuals::dark();
    v.panel_fill = BG;
    v.window_fill = PANEL;
    v.extreme_bg_color = PANEL2;
    v.faint_bg_color = PANEL;
    v.window_stroke = Stroke::new(1.0, LINE);
    v.override_text_color = Some(TXT);
    v.selection.bg_fill = CYAN.gamma_multiply(0.35);
    let r = CornerRadius::same(7);
    v.widgets.noninteractive.corner_radius = r;
    v.widgets.inactive.corner_radius = r;
    v.widgets.hovered.corner_radius = r;
    v.widgets.active.corner_radius = r;
    v.widgets.open.corner_radius = r;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, LINE);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TXT);
    v.widgets.inactive.bg_fill = PANEL2;
    v.widgets.inactive.weak_bg_fill = PANEL2;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, LINE);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, TXT);
    v.widgets.hovered.bg_fill = Color32::from_rgb(21, 31, 46);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(21, 31, 46);
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, CYAN);
    v.widgets.active.bg_fill = CYAN.gamma_multiply(0.25);
    v.widgets.active.weak_bg_fill = CYAN.gamma_multiply(0.25);
    st.spacing.item_spacing = egui::vec2(8.0, 7.0);
    st.spacing.button_padding = egui::vec2(12.0, 5.0);
    use egui::{FontFamily, FontId, TextStyle};
    st.text_styles
        .insert(TextStyle::Heading, FontId::new(19.0, FontFamily::Proportional));
    st.text_styles
        .insert(TextStyle::Body, FontId::new(14.0, FontFamily::Proportional));
    st.text_styles
        .insert(TextStyle::Button, FontId::new(14.0, FontFamily::Proportional));
    st.text_styles.insert(
        TextStyle::Monospace,
        FontId::new(13.5, FontFamily::Monospace),
    );
    ctx.set_global_style(st);
}

/// Card: a rounded group frame on the panel background.
pub fn card() -> egui::Frame {
    egui::Frame::new()
        .fill(PANEL)
        .stroke(Stroke::new(1.0, LINE))
        .corner_radius(CornerRadius::same(9))
        .inner_margin(Margin::same(10))
}

/// Group title (mono, dimmed, upper-case, ground-station style).
pub fn card_title(ui: &mut egui::Ui, t: &str) {
    ui.label(
        egui::RichText::new(t.to_uppercase())
            .monospace()
            .size(10.5)
            .color(DIM),
    );
}

/// Big figure + small unit.
pub fn big_value(ui: &mut egui::Ui, v: &str, unit: &str, color: Color32) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(v)
                .monospace()
                .size(23.0)
                .strong()
                .color(color),
        );
        ui.label(egui::RichText::new(unit).monospace().size(11.5).color(DIM));
    });
}

/// A label-value pair on one line (value in mono).
pub fn kv(ui: &mut egui::Ui, k: &str, v: &str) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(k).size(12.5).color(DIM));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(egui::RichText::new(v).monospace().size(12.5).color(TXT));
        });
    });
}

// ---- v40.34 layout helpers shared by the redesigned apps ----------------------

/// Status pill: a small rounded tag with a tinted background (top bar).
pub fn pill(ui: &mut egui::Ui, text: &str, color: Color32) {
    egui::Frame::new()
        .fill(color.gamma_multiply(0.16))
        .stroke(Stroke::new(1.0, color.gamma_multiply(0.7)))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::symmetric(9, 3))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(text).monospace().size(12.0).color(color));
        });
}

/// One collapsible section of a side panel: card + uppercase title.
pub fn section(ui: &mut egui::Ui, title: &str, open: bool, add: impl FnOnce(&mut egui::Ui)) {
    card().show(ui, |ui| {
        ui.set_width(ui.available_width());
        egui::CollapsingHeader::new(
            egui::RichText::new(title.to_uppercase()).monospace().size(11.0).color(DIM),
        )
        .id_salt(title)
        .default_open(open)
        .show_unindented(ui, |ui| {
            ui.add_space(4.0);
            add(ui);
        });
    });
    ui.add_space(4.0);
}

/// Compact stat block: small title, big number, unit.
pub fn stat(ui: &mut egui::Ui, title: &str, v: &str, unit: &str, color: Color32) {
    ui.vertical(|ui| {
        card_title(ui, title);
        big_value(ui, v, unit, color);
    });
}

/// Form row: a fixed-width dim label, then whatever widgets the row needs.
pub fn row(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui)) {
    // Wrapped, not plain horizontal: a row wider than the drawer used to push the panel out,
    // and egui answers that by clamping the panel back over the content, which ate the first
    // pixels of every label on the drawer's left edge. Wrapping keeps every row inside.
    ui.horizontal_wrapped(|ui| {
        ui.allocate_ui_with_layout(
            egui::vec2(98.0, 20.0),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                ui.set_min_width(98.0);
                ui.label(egui::RichText::new(label).size(12.5).color(DIM));
            },
        );
        add(ui);
    });
}

/// Fit a texture into the available rect keeping its aspect ratio, on black.
pub fn video_frame(ui: &mut egui::Ui, tex: Option<&egui::TextureHandle>, empty_text: &str) {
    let rect = ui.available_rect_before_wrap();
    let p = ui.painter();
    p.rect_filled(rect, 6.0, Color32::BLACK);
    match tex {
        Some(t) => {
            let sz = t.size_vec2();
            let s = (rect.width() / sz.x).min(rect.height() / sz.y);
            let vr = egui::Rect::from_center_size(rect.center(), sz * s);
            p.image(
                t.id(),
                vr,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                Color32::WHITE,
            );
        }
        None => {
            p.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                empty_text,
                egui::FontId::monospace(15.0),
                DIM,
            );
        }
    }
    ui.allocate_rect(rect, egui::Sense::hover());
}
