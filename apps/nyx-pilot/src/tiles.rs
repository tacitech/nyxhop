//! The map's tiles: raster slippy-map tiles (zoom/x/y, 256 px) from a tile server, each kept as a
//! file once fetched, so what was seen once still shows with no internet.
//!
//! Only the tiles on the screen are asked for (no fetching ahead), one request at a time, under
//! the app's own User-Agent; a failure leaves the server alone for a while. Two threads: one reads
//! the disk, one fetches, so a slow network never holds up a tile the disk has.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant, SystemTime};

use nyx_common::logging::log;

/// OpenStreetMap's standard tiles, and the credit their licence asks for on the map
pub const OSM_URL: &str = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
pub const OSM_CREDIT: &str = "© OpenStreetMap contributors";

const USER_AGENT: &str = concat!("nyxhop-pilot/", env!("CARGO_PKG_VERSION"), " (+https://nyxhop.com)");
/// A tile on the disk this old is fetched again when it is on the screen (the old one shows
/// meanwhile, and stays if there is no internet)
const REFRESH: Duration = Duration::from_secs(14 * 86400);
/// Textures kept: past this, those not drawn for a while go (the disk has them)
const KEEP: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Key {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

impl Key {
    /// The tile `up` levels above that holds this one, and this one's corner and size in it (0..1)
    pub fn parent(self, up: u8) -> Option<(Key, [f32; 3])> {
        if up == 0 || up > self.z {
            return None;
        }
        let n = (1u32 << up) as f32;
        let m = (1u32 << up) - 1;
        Some((Key { z: self.z - up, x: self.x >> up, y: self.y >> up }, [(self.x & m) as f32 / n, (self.y & m) as f32 / n, 1.0 / n]))
    }
}

#[derive(Default)]
struct State {
    /// the tiles on the screen, nearest the middle first
    want: Vec<Key>,
    /// handed to the screen
    have: HashSet<Key>,
    /// not on the disk: to fetch
    miss: HashSet<Key>,
    /// on the disk but old: to fetch again
    old: HashSet<Key>,
    /// the server has none (or what it sent is no picture): not asked again
    gone: HashSet<Key>,
}

struct Shared {
    url: String,
    dir: PathBuf,
    ext: String,
    st: Mutex<State>,
    cv: Condvar,
}

/// The tile's address: {z} {x} {y} filled in ({s}, a mirror's letter in older addresses, too)
fn tile_url(template: &str, k: Key) -> String {
    let s = ["a", "b", "c"][((k.x + k.y) % 3) as usize];
    template.replace("{z}", &k.z.to_string()).replace("{x}", &k.x.to_string()).replace("{y}", &k.y.to_string()).replace("{s}", s)
}

/// The folder for one server's tiles: its host, and a hash of the whole address (two addresses on
/// one host are two maps)
fn source_dir(template: &str) -> String {
    let host = template.split("://").nth(1).unwrap_or(template).split('/').next().unwrap_or("");
    let host: String = host.chars().map(|c| if c.is_ascii_alphanumeric() || c == '.' || c == '-' { c } else { '_' }).collect();
    let hash = template.bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193));
    format!("{host}-{hash:08x}")
}

/// The files' extension: the address's own (png, jpg), else "tile"
fn source_ext(template: &str) -> String {
    let path = template.split(['?', '#']).next().unwrap_or("");
    let last = path.rsplit('/').next().unwrap_or("");
    match last.rsplit_once('.') {
        Some((_, e)) if (2..=4).contains(&e.len()) && e.chars().all(|c| c.is_ascii_alphanumeric()) => e.to_ascii_lowercase(),
        _ => "tile".into(),
    }
}

impl Shared {
    fn path(&self, k: Key) -> PathBuf {
        self.dir.join(k.z.to_string()).join(k.x.to_string()).join(format!("{}.{}", k.y, self.ext))
    }
}

fn decode(bytes: &[u8]) -> Option<egui::ColorImage> {
    let img = image::load_from_memory(bytes).ok()?.to_rgba8();
    let size = [img.width() as usize, img.height() as usize];
    (size[0] > 0 && size[1] > 0).then(|| egui::ColorImage::from_rgba_unmultiplied(size, img.as_raw()))
}

fn age(path: &Path) -> Duration {
    std::fs::metadata(path).and_then(|m| m.modified()).ok().and_then(|t| SystemTime::now().duration_since(t).ok()).unwrap_or_default()
}

fn disk_thread(sh: Arc<Shared>, out: Sender<(Key, egui::ColorImage)>) {
    loop {
        let k = {
            let mut st = sh.st.lock().unwrap();
            loop {
                let next = st.want.iter().find(|k| !st.have.contains(k) && !st.miss.contains(k) && !st.gone.contains(k)).copied();
                if let Some(k) = next {
                    break k;
                }
                st = sh.cv.wait(st).unwrap();
            }
        };
        let path = sh.path(k);
        let img = std::fs::read(&path).ok().and_then(|b| decode(&b));
        let old = img.is_some() && age(&path) > REFRESH;
        let mut st = sh.st.lock().unwrap();
        match img {
            Some(img) => {
                st.have.insert(k);
                if old {
                    st.old.insert(k);
                }
                if out.send((k, img)).is_err() {
                    return;
                }
            }
            None => {
                st.miss.insert(k);
            }
        }
        drop(st);
        sh.cv.notify_all();
    }
}

fn save(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }
    // (whole or not at all: a half-written tile would be read as a broken one next time)
    let part = path.with_extension("part");
    std::fs::write(&part, bytes)?;
    std::fs::rename(&part, path)
}

fn net_thread(sh: Arc<Shared>, out: Sender<(Key, egui::ColorImage)>) {
    let agent: ureq::Agent = ureq::Agent::config_builder().timeout_global(Some(Duration::from_secs(12))).user_agent(USER_AGENT).build().into();
    // no request before this: after a failure the server is left alone a while
    let mut hold = Instant::now();
    let mut up = true;
    loop {
        let k = {
            let mut st = sh.st.lock().unwrap();
            loop {
                let next = st.want.iter().find(|k| st.miss.contains(k) || st.old.contains(k)).copied();
                if let Some(k) = next.filter(|_| Instant::now() >= hold) {
                    break k;
                }
                st = sh.cv.wait_timeout(st, Duration::from_millis(500)).unwrap().0;
            }
        };
        let url = tile_url(&sh.url, k);
        let got = agent.get(&url).call().and_then(|mut r| r.body_mut().with_config().limit(8 << 20).read_to_vec());
        let img = got.as_ref().ok().and_then(|b| decode(b));
        let status = if let Err(ureq::Error::StatusCode(c)) = &got { Some(*c) } else { None };
        if let (Ok(bytes), Some(_)) = (&got, &img)
            && let Err(e) = save(&sh.path(k), bytes)
        {
            log(&format!("map: cannot keep a tile in {}: {e}", sh.dir.display()));
        }
        let mut st = sh.st.lock().unwrap();
        match (got, img) {
            (Ok(_), Some(img)) => {
                st.miss.remove(&k);
                st.old.remove(&k);
                st.have.insert(k);
                if !up {
                    log("map: the tile server answers again");
                }
                up = true;
                if out.send((k, img)).is_err() {
                    return;
                }
            }
            // the server's own answer: no such tile (or no picture). Not asked again; refused or
            // told to slow down, nothing is asked for a while
            (Ok(_), None) | (Err(ureq::Error::StatusCode(_)), _) => {
                if st.miss.remove(&k) {
                    st.gone.insert(k);
                }
                st.old.remove(&k);
                if let Some(code) = status.filter(|c| matches!(c, 403 | 429 | 500..)) {
                    log(&format!("map: the tile server answers {code}; no tiles asked for 5 minutes"));
                    hold = Instant::now() + Duration::from_secs(300);
                }
            }
            (Err(e), _) => {
                if up {
                    log(&format!("map: no tile server ({e}); the map shows what is on the disk"));
                }
                up = false;
                hold = Instant::now() + Duration::from_secs(15);
            }
        }
    }
}

pub struct Tiles {
    sh: Arc<Shared>,
    rx: Receiver<(Key, egui::ColorImage)>,
    tex: HashMap<Key, (egui::TextureHandle, u64)>,
    frame: u64,
    /// the credit drawn on the map
    pub credit: String,
    /// the server's deepest zoom
    pub max_zoom: u8,
}

impl Tiles {
    /// Tiles from `url` ({z}/{x}/{y}), kept under `cache`
    pub fn new(url: &str, cache: &Path, credit: String, max_zoom: u8) -> Self {
        let sh = Arc::new(Shared {
            url: url.to_string(),
            dir: cache.join(source_dir(url)),
            ext: source_ext(url),
            st: Mutex::new(State::default()),
            cv: Condvar::new(),
        });
        log(&format!("map: tiles from {url}, kept in {}", sh.dir.display()));
        let (tx, rx) = channel();
        let (a, b) = (sh.clone(), tx.clone());
        std::thread::Builder::new().name("tiles-disk".into()).spawn(move || disk_thread(a, b)).expect("spawn tiles-disk");
        let a = sh.clone();
        std::thread::Builder::new().name("tiles-net".into()).spawn(move || net_thread(a, tx)).expect("spawn tiles-net");
        Tiles { sh, rx, tex: HashMap::new(), frame: 0, credit, max_zoom: max_zoom.min(24) }
    }

    /// The tile's texture, if it is in memory
    pub fn get(&mut self, k: Key) -> Option<egui::TextureId> {
        let frame = self.frame;
        self.tex.get_mut(&k).map(|(t, used)| {
            *used = frame;
            t.id()
        })
    }

    /// Once a frame, after drawing: the tiles on the screen, nearest the middle first. Those not in
    /// memory are read from the disk, or fetched.
    pub fn want(&mut self, ctx: &egui::Context, want: Vec<Key>) {
        self.frame += 1;
        while let Ok((k, img)) = self.rx.try_recv() {
            let name = format!("tile {}/{}/{}", k.z, k.x, k.y);
            self.tex.insert(k, (ctx.load_texture(name, img, egui::TextureOptions::LINEAR), self.frame));
        }
        let mut dropped = Vec::new();
        if self.tex.len() > KEEP {
            let frame = self.frame;
            self.tex.retain(|k, (_, used)| {
                let keep = frame - *used < 300;
                if !keep {
                    dropped.push(*k);
                }
                keep
            });
        }
        let mut st = self.sh.st.lock().unwrap();
        for k in &dropped {
            st.have.remove(k);
        }
        if st.want != want {
            st.want = want;
            drop(st);
            self.sh.cv.notify_all();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_address_and_the_file_of_a_tile() {
        let k = Key { z: 17, x: 68647, y: 45888 };
        assert_eq!(tile_url(OSM_URL, k), "https://tile.openstreetmap.org/17/68647/45888.png");
        assert_eq!(tile_url("http://{s}.example.org/t/{z}/{y}/{x}?key=1", k), "http://b.example.org/t/17/45888/68647?key=1");
        assert_eq!(source_ext(OSM_URL), "png");
        assert_eq!(source_ext("https://example.org/sat/{z}/{y}/{x}.JPG?token=a.b"), "jpg");
        assert_eq!(source_ext("https://example.org/sat/{z}/{y}/{x}"), "tile");
        let dir = source_dir(OSM_URL);
        assert!(dir.starts_with("tile.openstreetmap.org-") && dir.len() == "tile.openstreetmap.org-".len() + 8, "{dir}");
        assert_ne!(dir, source_dir("https://tile.openstreetmap.org/hot/{z}/{x}/{y}.png"));
        assert!(source_dir("https://ex.org:8080/{z}/{x}/{y}.png").starts_with("ex.org_8080-"));
        let sh = Shared { url: OSM_URL.into(), dir: "cache".into(), ext: "png".into(), st: Mutex::default(), cv: Condvar::new() };
        assert_eq!(sh.path(k), Path::new("cache").join("17").join("68647").join("45888.png"));
    }

    #[test]
    fn a_tile_in_its_parent() {
        let k = Key { z: 17, x: 68647, y: 45888 };
        assert_eq!(k.parent(1), Some((Key { z: 16, x: 34323, y: 22944 }, [0.5, 0.0, 0.5])));
        assert_eq!(k.parent(2), Some((Key { z: 15, x: 17161, y: 11472 }, [0.75, 0.0, 0.25])));
        assert_eq!(k.parent(18), None);
        assert_eq!(Key { z: 1, x: 1, y: 1 }.parent(1), Some((Key { z: 0, x: 0, y: 0 }, [0.5, 0.5, 0.5])));
    }

    #[test]
    fn a_tile_is_kept_whole_and_read_back() {
        let dir = std::env::temp_dir().join(format!("nyx-pilot-tiles-{}", std::process::id()));
        let path = dir.join("3").join("4").join("5.png");
        let mut png = Vec::new();
        let img = image::RgbaImage::from_pixel(4, 2, image::Rgba([10, 20, 30, 255]));
        image::DynamicImage::ImageRgba8(img).write_to(&mut std::io::Cursor::new(&mut png), image::ImageFormat::Png).unwrap();
        save(&path, &png).unwrap();
        assert!(!path.with_extension("part").exists());
        let back = decode(&std::fs::read(&path).unwrap()).unwrap();
        assert_eq!(back.size, [4, 2]);
        assert_eq!(back.pixels[0], egui::Color32::from_rgb(10, 20, 30));
        assert!(decode(b"not a picture").is_none());
        assert!(age(&path) < Duration::from_secs(60));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
