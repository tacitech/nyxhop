//! Dead-simple logger: every line goes to the console AND to
//! `logs/<app>.log`, timestamped. The GUI status panes mirror the same
//! lines, so a session can be debugged from the log file alone.

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::sync::{Mutex, OnceLock};
use std::time::{SystemTime, UNIX_EPOCH};

struct Logger {
    app: &'static str,
    file: Option<Mutex<File>>,
}

static LOGGER: OnceLock<Logger> = OnceLock::new();
/// An extra sink for every line (the Android app hands lines to logcat: it has no stdout).
static HOOK: OnceLock<fn(&str)> = OnceLock::new();

pub fn set_hook(f: fn(&str)) {
    let _ = HOOK.set(f);
}

/// Call once at startup. Creates `logs/<app>.log` next to the CWD.
pub fn init(app: &'static str) {
    let file = std::fs::create_dir_all("logs").ok().and_then(|_| {
        OpenOptions::new()
            .create(true)
            .append(true)
            .open(format!("logs/{app}.log"))
            .ok()
    });
    let _ = LOGGER.set(Logger { app, file: file.map(Mutex::new) });
    log(&format!("=== {app} started (pid {}) ===", std::process::id()));
}

fn timestamp() -> String {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = now.as_secs();
    let ms = now.subsec_millis();
    let (h, m, s) = ((secs / 3600) % 24, (secs / 60) % 60, secs % 60);
    format!("{h:02}:{m:02}:{s:02}.{ms:03}")
}

/// Write one line to console + log file.
pub fn log(msg: &str) {
    let Some(l) = LOGGER.get() else {
        println!("{msg}");
        if let Some(h) = HOOK.get() {
            h(msg);
        }
        return;
    };
    let line = format!("[{}] [{}] {}", timestamp(), l.app, msg);
    println!("{line}");
    if let Some(h) = HOOK.get() {
        h(msg);
    }
    if let Some(f) = &l.file
        && let Ok(mut f) = f.lock()
    {
        let _ = writeln!(f, "{line}");
    }
}

/// Rate-limited status logger: emits at most one line per `period`.
/// NOTE: never subtract Durations from Instant::now() to fake "long ago" —
/// on Windows Instant counts from boot, and on a freshly rebooted machine
/// that subtraction underflows and panics the thread.
pub struct StatusLogger {
    last: Option<std::time::Instant>,
    period: std::time::Duration,
}

impl StatusLogger {
    pub fn new(period_secs: f32) -> Self {
        StatusLogger { last: None, period: std::time::Duration::from_secs_f32(period_secs) }
    }

    /// Log `line` if the period has elapsed (always logs the first call);
    /// returns true when logged.
    pub fn tick(&mut self, line: &str) -> bool {
        if self.last.is_none_or(|t| t.elapsed() >= self.period) {
            self.last = Some(std::time::Instant::now());
            log(line);
            true
        } else {
            false
        }
    }
}
