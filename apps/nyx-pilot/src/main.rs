//! nyxhop-pilot: the pilot app on a PC (the app itself is the library, nyx_pilot - the phone's
//! APK hosts the same). Here: the window, opened again on a new GPU device when the old one is
//! lost.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use egui::Rect;
use nyx_common::Opts;
use nyx_common::logging::{self, log};
use nyx_pilot::Pilot;

#[cfg(windows)]
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

// ---------------------------------------------------------------- the window and its GPU
//
// 12/11 the pilot app died (10:18, the aircraft on the bench, a cargo build busy) with the wgpu
// panic "Texture with 'egui_texid_Managed(5)' label is invalid" in create_view - a LOST GPU
// DEVICE, as nyxhop's ground screen on 29/10: after the loss every new texture is an invalid one
// and the first use panics. As nyxhop: the device-lost callback or a panic from the GPU layer
// closes the window and a new one with a new device opens on the same app; the link, the MAVLink
// and the board threads never stop; any other panic stays fatal.

std::thread_local! {
    /// The file of this thread's last panic, to tell a GPU-layer panic from the rest.
    static PANIC_FILE: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}

fn note_panic_file() {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if let Some(l) = info.location() {
            let _ = PANIC_FILE.try_with(|f| *f.borrow_mut() = l.file().to_string());
        }
        prev(info);
    }));
}

/// Where the window was, to open the next one in its place.
#[derive(Clone, Copy, Default)]
struct WinPlace {
    inner: Option<Rect>,
    outer: Option<Rect>,
    maximized: bool,
    fullscreen: bool,
}

impl WinPlace {
    fn viewport(&self) -> egui::ViewportBuilder {
        let mut b = egui::ViewportBuilder::default().with_inner_size([1280.0, 800.0]).with_title("NyxHop Pilot");
        if let Some(r) = self.inner {
            b = b.with_inner_size(r.size());
        }
        if let Some(r) = self.outer {
            b = b.with_position(r.min);
        }
        if self.maximized {
            b = b.with_maximized(true);
        }
        if self.fullscreen {
            b = b.with_fullscreen(true);
        }
        b
    }
}

/// One window over the app, which outlives it.
struct Window<'a> {
    app: &'a mut Pilot,
    place: &'a mut WinPlace,
    lost: Arc<AtomicBool>,
    /// this window closed itself for a new device
    reopen: &'a mut bool,
    /// NYX_GPU_TEST_LOSE: when to destroy the device
    test_lose: Option<Instant>,
}

impl eframe::App for Window<'_> {
    fn ui(&mut self, root: &mut egui::Ui, frame: &mut eframe::Frame) {
        root.ctx().input(|i| {
            let v = i.viewport();
            // a minimized window sits at -32000 on Windows
            if v.minimized != Some(true) {
                *self.place = WinPlace {
                    inner: v.inner_rect,
                    outer: v.outer_rect,
                    maximized: v.maximized == Some(true),
                    fullscreen: v.fullscreen == Some(true),
                };
            }
        });
        if self.lost.load(Ordering::Relaxed) {
            if !*self.reopen {
                *self.reopen = true;
                root.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
            }
            return;
        }
        if self.test_lose.is_some_and(|t| Instant::now() >= t) {
            self.test_lose = None;
            if let Some(rs) = frame.wgpu_render_state() {
                log("NYX_GPU_TEST_LOSE: destroying the GPU device");
                rs.device.destroy();
                self.app.gpu_reset();
            }
        }
        eframe::App::ui(self.app, root, frame);
    }
}

fn main() -> eframe::Result {
    logging::init("pilot");
    note_panic_file();
    nyx_common::sys::keep_up();
    let opts = Opts::from_env();
    let board = opts.arg("--board", "192.168.0.11");
    let dir = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into());
    nyx_common::i18n::init(&dir);
    let mut app = Pilot::new(&board, &opts, &dir);
    let mut place = WinPlace::default();
    // NYX_GPU_TEST_LOSE=<s>: <s> seconds after each window opens, its GPU device is destroyed
    let test_lose = std::env::var("NYX_GPU_TEST_LOSE").ok().and_then(|v| v.trim().parse::<f32>().ok()).map(Duration::from_secs_f32);
    // windows in a row that did not open (or died at once) since the GPU was lost; None until the first loss
    let mut failed: Option<u32> = None;
    loop {
        let lost = Arc::new(AtomicBool::new(false));
        let mut reopen = false;
        let opened = Instant::now();
        let viewport = place.viewport();
        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let (app, place, reopen, lost) = (&mut app, &mut place, &mut reopen, lost.clone());
            eframe::run_native(
                "NyxHop Pilot",
                eframe::NativeOptions { viewport, ..Default::default() },
                Box::new(move |cc| {
                    nyx_common::ui::touch_style(&cc.egui_ctx);
                    if let Some(rs) = &cc.wgpu_render_state {
                        let i = rs.adapter.get_info();
                        log(&format!("gpu: {} ({:?}, {} {})", i.name, i.backend, i.driver, i.driver_info));
                        let lost = lost.clone();
                        rs.device.set_device_lost_callback(move |reason, msg| {
                            log(&format!("gpu: device lost ({reason:?}) {msg}"));
                            lost.store(true, Ordering::Relaxed);
                        });
                    }
                    let test_lose = test_lose.map(|d| Instant::now() + d);
                    Ok(Box::new(Window { app, place, lost, reopen, test_lose }))
                }),
            )
        }));
        let panic = match run {
            Ok(Ok(())) if !reopen => return Ok(()),
            Ok(Ok(())) => {
                log("gpu: the device was lost: a new window on a new device");
                None
            }
            Ok(Err(e)) => {
                // the first window as before; after a loss the GPU may need a moment (a driver
                // reset), so try for a minute
                let Some(n) = failed.filter(|&n| n < 30) else { return Err(e) };
                log(&format!("gpu: no window yet ({e}), trying again"));
                failed = Some(n + 1);
                std::thread::sleep(Duration::from_secs(2));
                continue;
            }
            Err(panic) => {
                let file = PANIC_FILE.with_borrow(|f| f.clone());
                if !file.contains("wgpu") {
                    std::panic::resume_unwind(panic);
                }
                log(&format!("gpu: the GPU layer panicked ({file}): a new window on a new device"));
                Some(panic)
            }
        };
        // a window that dies at once counts as one that did not open
        let n = if opened.elapsed() < Duration::from_secs(5) { failed.unwrap_or(0) + 1 } else { 0 };
        if n > 30 {
            log("gpu: no window lasted for a minute: giving up");
            match panic {
                Some(p) => std::panic::resume_unwind(p),
                None => return Ok(()),
            }
        }
        failed = Some(n);
        app.gpu_reset();
        if n > 0 {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
}

