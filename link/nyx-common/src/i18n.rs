//! The interface language: English, or Vietnamese when the app's folder holds nyx-lang.txt with
//! "vi" in it (`picker` puts the choice on screen, where an app draws it). Every string on screen is written as a pair
//! where it is drawn: `t("Connect", "Kết nối")`, `tf!("{n} channels", "{n} kênh")`. Logs, console
//! commands and config keys stay English.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

static VI: AtomicBool = AtomicBool::new(false);
static STORE: Mutex<Option<PathBuf>> = Mutex::new(None);
const FILE: &str = "nyx-lang.txt";

/// Vietnamese on screen.
pub fn vi() -> bool {
    VI.load(Ordering::Relaxed)
}

/// The pair's string for the language in use.
pub fn t(en: &'static str, vi_s: &'static str) -> &'static str {
    if vi() { vi_s } else { en }
}

/// A formatted pair: `tf!("{n} channels", "{n} kênh")`, `tf!("{} dB", "{} dB", snr)`.
#[macro_export]
macro_rules! tf {
    ($en:literal, $vi:literal $(, $arg:expr)* $(,)?) => {
        if $crate::i18n::vi() { format!($vi $(, $arg)*) } else { format!($en $(, $arg)*) }
    };
}

/// The language from `dir/nyx-lang.txt` ("vi" or "en"), else English; later choices are saved
/// there.
pub fn init(dir: &Path) {
    let p = dir.join(FILE);
    let saved = std::fs::read_to_string(&p).ok().map(|s| s.trim().to_ascii_lowercase());
    let on = match saved.as_deref() {
        Some("vi") => true,
        _ => false,
    };
    VI.store(on, Ordering::Relaxed);
    *STORE.lock().unwrap() = Some(p);
}

/// Switch the language (remembered when `init` named a place).
pub fn set_vi(on: bool) {
    VI.store(on, Ordering::Relaxed);
    if let Some(p) = STORE.lock().unwrap().as_ref() {
        let _ = std::fs::write(p, if on { "vi\n" } else { "en\n" });
    }
}

/// The language picker on screen (`picker`); the public export sets it false, English only there.
pub const PICKER: bool = false;

/// English · Tiếng Việt, the choice saved beside the program. For a right-to-left layout (English
/// ends up on the left).
pub fn picker(ui: &mut egui::Ui) {
    if !PICKER {
        return;
    }
    let on = vi();
    let mut pick = on;
    ui.selectable_value(&mut pick, true, "Tiếng Việt");
    ui.selectable_value(&mut pick, false, "English");
    if pick != on {
        set_vi(pick);
    }
}

/// The device's own font in front of egui's: egui's has no Vietnamese (88 of the 128 accented
/// letters missing, they draw as boxes). Segoe UI on Windows, Roboto on Android, read from the
/// device; egui's stays behind for the symbols, and the monospace falls back to it as well.
pub fn fonts(ctx: &egui::Context) {
    let paths = [
        "C:/Windows/Fonts/segoeui.ttf",
        "/system/fonts/Roboto-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    ];
    let Some(bytes) = paths.iter().find_map(|p| std::fs::read(p).ok()) else {
        return;
    };
    let mut f = egui::FontDefinitions::default();
    f.font_data.insert("sys".into(), std::sync::Arc::new(egui::FontData::from_owned(bytes)));
    if let Some(prop) = f.families.get_mut(&egui::FontFamily::Proportional) {
        prop.insert(0, "sys".into());
    }
    if let Some(mono) = f.families.get_mut(&egui::FontFamily::Monospace) {
        mono.push("sys".into());
    }
    ctx.set_fonts(f);
}

#[cfg(test)]
mod tests {
    #[test]
    fn pairs_follow_the_language() {
        let n = 3;
        super::VI.store(true, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(super::t("Connect", "Kết nối"), "Kết nối");
        assert_eq!(tf!("{n} channels", "{n} kênh"), "3 kênh");
        assert_eq!(tf!("{} dB", "{} dB", 5), "5 dB");
        super::VI.store(false, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(tf!("{n} channels", "{n} kênh"), "3 channels");
    }
}
