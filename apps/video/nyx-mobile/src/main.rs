//! Run nyx-mobile on the PC (the SAME code as the Android build) to check the logic before
//! packaging the APK, without plugging in a phone to find out.
//!   cargo run --release -p nyx-mobile --bin nyx-mobile-desktop -- 192.168.0.11:7011
fn main() -> eframe::Result {
    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(nyx_mobile::board_addr);
    let sh = nyx_mobile::Shared::new(addr);
    nyx_mobile::spawn_net(sh.clone());
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([420.0, 780.0]) // phone-shaped
            .with_title("NyxHop mobile (desktop test)"),
        ..Default::default()
    };
    eframe::run_native(
        "NyxHop mobile",
        opts,
        Box::new(move |cc| {
            nyx_common::theme::apply(&cc.egui_ctx);
            Ok(Box::new(nyx_mobile::App::new(sh)))
        }),
    )
}
