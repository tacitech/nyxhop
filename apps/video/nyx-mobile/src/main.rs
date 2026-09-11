//! Run nyx-mobile on the PC (the SAME code as the Android build) to check the logic before
//! packaging the APK, without plugging in a phone to find out. The phone camera is Android
//! only; on the PC the aircraft screen has the test pattern and an IP camera.
//!   cargo run --release -p nyx-mobile --bin nyx-mobile-desktop -- 192.168.0.12
fn main() -> eframe::Result {
    nyx_common::logging::init("mobile");
    let board = std::env::args().nth(1);
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
            nyx_common::ui::touch_style(&cc.egui_ctx);
            Ok(Box::new(nyx_mobile::Host::new(board)))
        }),
    )
}
