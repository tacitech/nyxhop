//! v40.33 licence rows shared by the apps (nyx-tx, nyx-rx, the phone app).
//!
//! The board reports `lic_*` lines on its console (`license`, and inside `hop
//! status`); the transmit end's state also arrives in-band (SourceType::Info) so
//! the app on the receive end can see the aircraft's DNA and push a licence to
//! it over the control channel (`license far`). Laid out for a narrow, touch
//! screen: nothing wider than the panel, buttons finger-sized.

use egui::Color32;
use std::collections::HashMap;

pub fn state_color(state: &str) -> Color32 {
    match state {
        "licensed" => Color32::LIGHT_GREEN,
        "trial" => Color32::YELLOW,
        "nogate" => Color32::GRAY,
        _ => Color32::LIGHT_RED,
    }
}

/// `key=value` lines -> map (the in-band Info frame and the console use the same form).
pub fn parse_kv(text: &str) -> HashMap<String, String> {
    text.lines()
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| (k.trim().to_string(), v.trim().to_string()))
        .collect()
}

fn state_block(ui: &mut egui::Ui, label: &str, kv: &HashMap<String, String>) {
    let g = |k: &str| kv.get(k).cloned().unwrap_or_else(|| "?".into());
    let ls = g("lic_state");
    let usage = match ls.as_str() {
        "licensed" => format!("{} min used", g("lic_min")),
        "trial" => format!("trial {} of {} min used", g("lic_min"), g("lic_trial")),
        "locked" => "trial over - video off".to_string(),
        "nogate" => "no licence gate in this bitstream".to_string(),
        _ => String::new(),
    };
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new(label).strong());
        ui.colored_label(state_color(&ls), ls.clone());
        if !usage.is_empty() {
            ui.label(egui::RichText::new(usage).small().color(Color32::GRAY));
        }
    });
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(format!("DNA {}", g("lic_dna"))).monospace());
        if ui
            .add(egui::Button::new("Copy").min_size(egui::vec2(70.0, 32.0)))
            .on_hover_text("The board's device DNA. Send it with a licence request; the licence fits this board only.")
            .clicked()
        {
            ui.ctx().copy_text(g("lic_dna"));
        }
    });
    ui.label(
        egui::RichText::new("The DNA is the chip's factory serial number, not personal data; a licence works offline and only on this chip.")
            .small()
            .color(crate::theme::DIM),
    );
    if let Some(e) = kv.get("lic_err") {
        if e != "-" && !e.is_empty() {
            ui.colored_label(Color32::LIGHT_RED, e);
        }
    }
}

/// This board, the far board (if `far` has its Info lines), and the paste box.
/// `send` takes a console command for THIS board.
pub fn lic_rows(
    ui: &mut egui::Ui,
    kv: &HashMap<String, String>,
    far: Option<&str>,
    buf: &mut String,
    status: &mut String,
    send: &mut dyn FnMut(String),
) {
    state_block(ui, "This board", kv);
    let far_kv = far.map(parse_kv).filter(|m| m.contains_key("lic_dna"));
    if let Some(fk) = &far_kv {
        ui.add_space(4.0);
        state_block(ui, "Aircraft", fk);
    }
    ui.add_space(4.0);
    ui.add(
        egui::TextEdit::multiline(buf)
            .hint_text("paste a licence file here (nyxhop-license 1 ...)")
            .desired_rows(2)
            .desired_width(ui.available_width()),
    );
    let parsed = nyx_proto::lic::License::parse(buf);
    ui.horizontal_wrapped(|ui| {
        if ui
            .add(egui::Button::new("Apply here").min_size(egui::vec2(120.0, 38.0)))
            .on_hover_text("Load the pasted licence on this board (its gate checks it against its DNA).")
            .clicked()
        {
            match &parsed {
                Ok(l) => {
                    send(format!("license put {}", l.to_hex()));
                    *status = format!("sent {}", l.summary());
                    buf.clear();
                }
                Err(e) => *status = format!("not a licence: {e}"),
            }
        }
        if far_kv.is_some()
            && ui
                .add(egui::Button::new("Send to aircraft").min_size(egui::vec2(150.0, 38.0)))
                .on_hover_text("Push the pasted licence to the linked aircraft over the control channel (20 s).")
                .clicked()
        {
            match &parsed {
                Ok(l) => {
                    send(format!("license far {}", l.to_hex()));
                    *status = format!("pushing {} to the aircraft", l.summary());
                    buf.clear();
                }
                Err(e) => *status = format!("not a licence: {e}"),
            }
        }
    });
    if !status.is_empty() {
        ui.label(egui::RichText::new(status.as_str()).small().color(Color32::GRAY));
    }
}
