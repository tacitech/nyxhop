//! The map: the aircraft from above, north up. Its arrow turns with its heading; home, the track
//! flown since it armed, how far and which way home is. Small in a corner over the video or
//! large behind the HUD; the wheel, a pinch or its buttons zoom, a drag moves the large one.
//!
//! While the aircraft tells where it is on the globe, the ground under it is a slippy map's tiles
//! (tiles.rs). When it does not (indoors, no satellite fix), a plain grid in metres, and the track
//! from its local position, from where it armed.

use std::f64::consts::PI;

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, pos2, vec2};
use nyx_common::i18n::t;

use crate::mav::Vehicle;
use crate::mav::Place;
use crate::tiles::{Key, Tiles};
use crate::{ACCENT, AMBER, WHITE};

/// The equator's length on Web Mercator's sphere, metres
const EARTH: f64 = 40_075_016.686;
const PLATE: Color32 = Color32::from_rgba_premultiplied(8, 12, 17, 200);
const DARK: Color32 = Color32::from_rgb(28, 39, 51);

/// Web Mercator: latitude and longitude in degrees to the world's square, 0..1 (x east, y south)
pub fn merc(lat: f64, lon: f64) -> (f64, f64) {
    let s = lat.to_radians().sin().clamp(-0.9999, 0.9999);
    ((lon + 180.0) / 360.0, 0.5 - ((1.0 + s) / (1.0 - s)).ln() / (4.0 * PI))
}

/// From a to b on the globe (latitude, longitude in degrees): metres, and the bearing in degrees
/// from north, clockwise
pub fn dist_bearing(a: (f64, f64), b: (f64, f64)) -> (f32, f32) {
    let (p1, p2, dl) = (a.0.to_radians(), b.0.to_radians(), (b.1 - a.1).to_radians());
    let h = ((p2 - p1) / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    let d = 2.0 * 6_371_008.8 * h.sqrt().min(1.0).asin();
    let brg = (dl.sin() * p2.cos()).atan2(p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos());
    (d as f32, brg.to_degrees().rem_euclid(360.0) as f32)
}

/// The round length (1, 2 or 5 times a power of ten) not above `m`
pub fn nice(m: f64) -> f64 {
    let p = 10f64.powf(m.log10().floor());
    let f = m / p;
    p * if f >= 5.0 { 5.0 } else if f >= 2.0 { 2.0 } else { 1.0 }
}

pub fn dist_text(m: f32) -> String {
    if m < 1000.0 {
        format!("{m:.0} m")
    } else if m < 10_000.0 {
        format!("{:.2} km", m / 1000.0)
    } else {
        format!("{:.1} km", m / 1000.0)
    }
}

/// A scale bar's length (a round one), worded
fn scale_text(m: f64) -> String {
    if m >= 1000.0 {
        format!("{:.0} km", m / 1000.0)
    } else if m >= 1.0 {
        format!("{m:.0} m")
    } else {
        format!("{:.0} cm", m * 100.0)
    }
}

/// The tiles of zoom level `z` that cover a view `w` by `h` points around `mid` (the world's
/// square) at `scale` points for the whole world: columns (they wrap around the world) and rows
fn tile_span(mid: (f64, f64), scale: f64, w: f32, h: f32, z: u8) -> (std::ops::RangeInclusive<i64>, std::ops::RangeInclusive<i64>) {
    let n = (1u64 << z) as f64;
    let (hw, hh) = (w as f64 / 2.0 / scale, h as f64 / 2.0 / scale);
    let x = ((mid.0 - hw) * n).floor() as i64..=((mid.0 + hw) * n).floor() as i64;
    let y = (((mid.1 - hh) * n).floor() as i64).max(0)..=(((mid.1 + hh) * n).floor() as i64).min(n as i64 - 1);
    (x, y)
}

/// Words on the map, on a dark plate so that they read over any ground; the plate
pub fn tag(p: &egui::Painter, at: Pos2, anchor: Align2, text: &str, size: f32, color: Color32) -> Rect {
    let g = p.layout_no_wrap(text.to_string(), FontId::proportional(size), color);
    let r = anchor.anchor_size(at, g.size() + vec2(10.0, 4.0));
    p.rect_filled(r, 4.0, PLATE);
    p.galley(r.min + vec2(5.0, 2.0), g, color);
    r
}

fn button(ui: &egui::Ui, r: Rect, id: &str, label: &str, size: f32) -> bool {
    let resp = ui.interact(r, ui.id().with(id), Sense::click());
    let fill = if resp.is_pointer_button_down_on() {
        ACCENT
    } else if resp.hovered() {
        Color32::from_rgb(44, 60, 78)
    } else {
        DARK
    };
    ui.painter().rect_filled(r, 6.0, fill);
    ui.painter().rect_stroke(r, 6.0, Stroke::new(1.0, Color32::from_white_alpha(40)), egui::StrokeKind::Inside);
    ui.painter().text(r.center(), Align2::CENTER_CENTER, label, FontId::proportional(size), WHITE);
    resp.clicked()
}

/// The aircraft: a dart pointing its heading (0 = north = up); `faint` where it was last known
fn dart(p: &egui::Painter, c: Pos2, yaw: f32, size: f32, faint: bool) {
    let f = vec2(yaw.sin(), -yaw.cos());
    let s = vec2(-f.y, f.x);
    // (from the notch at its back: the fill fans out from the first point)
    let shape = |k: f32| vec![c - f * (0.3 * k), c - f * (0.8 * k) - s * (0.72 * k), c + f * k, c - f * (0.8 * k) + s * (0.72 * k)];
    p.add(Shape::convex_polygon(shape(size + 3.0), Color32::from_black_alpha(110), Stroke::NONE));
    let (fill, edge) = if faint { (ACCENT.gamma_multiply(0.3), Color32::from_gray(170)) } else { (ACCENT, WHITE) };
    p.add(Shape::convex_polygon(shape(size), fill, Stroke::new(1.5, edge)));
}

/// Where the map's words go (the small map's own corners, the large one's clear of the HUD) and
/// its zoom buttons (their top right corner; they stack downwards)
#[derive(Clone, Copy)]
pub struct Corners {
    pub tl: Pos2,
    pub bl: Pos2,
    pub br: Pos2,
    pub zoom: Pos2,
}

pub struct MapView {
    pub tiles: Tiles,
    /// The zoom, as a slippy map counts it (a level up is twice as close; fractions in between):
    /// the map's, and the grid's (what the same number shows at the equator)
    zoom: f32,
    grid_zoom: f32,
    /// The large map dragged away from the aircraft: its middle, in the world drawn (the map's
    /// square, or the local frame's east and south in metres). None follows the aircraft.
    centre: Option<(f64, f64)>,
    centre_on_globe: bool,
}

impl MapView {
    pub fn new(tiles: Tiles) -> Self {
        MapView { tiles, zoom: 16.0, grid_zoom: 21.0, centre: None, centre_on_globe: false }
    }

    /// Zoom by levels (+ closer): the map while the aircraft is on the globe, else the grid
    pub fn zoom_by(&mut self, on_globe: bool, d: f32) {
        if on_globe {
            self.zoom = (self.zoom + d).clamp(2.0, self.tiles.max_zoom as f32 + 3.0);
        } else {
            self.grid_zoom = (self.grid_zoom + d).clamp(13.0, 25.0);
        }
    }

    /// The map in `r`. True when the small one is tapped (it and the video change places).
    pub fn show(&mut self, ui: &mut egui::Ui, r: Rect, at: Corners, v: &Vehicle, large: bool) -> bool {
        let place = v.place();
        let (globe, from) = (place.map(|p| p.0), place.map(|p| p.1));
        let on_globe = globe.is_some();
        if !large || self.centre_on_globe != on_globe {
            self.centre = None;
        }
        let resp = ui.interact(r, ui.id().with(("map", large)), if large { Sense::click_and_drag() } else { Sense::click() });
        if resp.hovered() || resp.dragged() {
            let (wheel, pinch) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            self.zoom_by(on_globe, wheel / 100.0 + pinch.log2());
        }

        // home on the globe; with no fix, where it armed, carried from the last place as it is
        let last = from == Some(Place::Last);
        let home_ll = v.home_global().or_else(|| v.takeoff.filter(|_| last).and_then(|(n, e)| v.carried(n, e)));
        // the world drawn, x east and y south: the map's square, or the local frame in metres
        let (craft, home, zoom, unit) = match globe {
            Some((lat, lon)) => (merc(lat, lon), home_ll.map(|(la, lo)| merc(la, lo)), self.zoom, 1.0),
            None => {
                let (n, e) = v.home_local();
                ((v.y as f64, -v.x as f64), Some((e as f64, -n as f64)), self.grid_zoom, EARTH)
            }
        };
        // points on the screen for one of the world's units
        let scale = 256.0 * 2f64.powf(zoom as f64) / unit;
        if large && resp.dragged() {
            let (c, d) = (self.centre.unwrap_or(craft), resp.drag_delta());
            self.centre = Some((c.0 - d.x as f64 / scale, c.1 - d.y as f64 / scale));
            self.centre_on_globe = on_globe;
        }
        let mid = self.centre.unwrap_or(craft);
        let to = |w: (f64, f64)| r.center() + vec2(((w.0 - mid.0) * scale) as f32, ((w.1 - mid.1) * scale) as f32);
        let p = ui.painter().with_clip_rect(r);

        // how many metres a point on the screen is; the scale bar, a round length (the grid's
        // squares are as long)
        let m_per_pt = globe.map_or(1.0, |(lat, _)| EARTH * lat.to_radians().cos()) / scale;
        let bar_m = nice((if large { 110.0 } else { 56.0 }) * m_per_pt);

        // the ground: tiles, or the grid
        match globe {
            Some(_) => {
                p.rect_filled(r, 0.0, Color32::from_rgb(40, 46, 54));
                self.ground(ui.ctx(), &p, r, mid, scale, large);
            }
            None => {
                self.tiles.want(ui.ctx(), Vec::new());
                p.rect_filled(r, 0.0, Color32::from_rgb(13, 19, 27));
                let step = bar_m;
                let line = Stroke::new(1.0, Color32::from_white_alpha(22));
                let (hw, hh) = (r.width() as f64 / 2.0 / scale, r.height() as f64 / 2.0 / scale);
                let mut x = ((mid.0 - hw) / step).floor() * step;
                while x <= mid.0 + hw {
                    let sx = to((x, mid.1)).x;
                    p.line_segment([pos2(sx, r.top()), pos2(sx, r.bottom())], line);
                    x += step;
                }
                let mut y = ((mid.1 - hh) / step).floor() * step;
                while y <= mid.1 + hh {
                    let sy = to((mid.0, y)).y;
                    p.line_segment([pos2(r.left(), sy), pos2(r.right(), sy)], line);
                    y += step;
                }
            }
        }

        // the track, up to the aircraft (points closer than a pixel or two to the last are skipped)
        let cp = to(craft);
        let mut pts: Vec<Pos2> = Vec::new();
        let mut add = |w: (f64, f64)| {
            let q = to(w);
            if pts.last().is_none_or(|l| (*l - q).length_sq() > 2.5) {
                pts.push(q);
            }
        };
        if last {
            v.track.iter().filter_map(|&(n, e)| v.carried(n, e)).for_each(|(lat, lon)| add(merc(lat, lon)));
        } else if on_globe {
            v.gtrack.iter().for_each(|&(lat, lon)| add(merc(lat as f64 * 1e-7, lon as f64 * 1e-7)));
        } else {
            v.track.iter().for_each(|&(n, e)| add((e as f64, -n as f64)));
        }
        if !pts.is_empty() {
            pts.push(cp);
            p.add(Shape::line(pts.clone(), Stroke::new(5.0, Color32::from_black_alpha(90))));
            p.add(Shape::line(pts, Stroke::new(2.5, ACCENT)));
        }

        // home: the line to it, how far and which way
        let home_pt = home.map(to);
        let mut home_text = None;
        if let Some(hp) = home_pt {
            // (dashed while it is a line on the screen: zoomed in on an aircraft far out, the
            // dashes of a line a million points long would be drawn one by one)
            let line = Stroke::new(1.5, AMBER.gamma_multiply(0.8));
            if (hp - cp).length() < 6000.0 {
                p.extend(Shape::dashed_line(&[cp, hp], line, 7.0, 5.0));
            } else {
                p.line_segment([cp, hp], line);
            }
            let (dist, brg) = match globe {
                Some(ll) => home_ll.map_or((0.0, 0.0), |hll| dist_bearing(ll, hll)),
                None => {
                    let (n, e) = v.home_local();
                    ((n - v.x).hypot(e - v.y), (e - v.y).atan2(n - v.x).to_degrees().rem_euclid(360.0))
                }
            };
            home_text = Some(format!("{} {} · {brg:03.0}°", t("Home", "Nhà"), dist_text(dist)));
        }

        // the words at the top: north and the satellites, home
        let fs = if large { 13.0 } else { 11.0 };
        let sats = v.gps_fix().filter(|g| g.1 != 255).map(|g| format!("{} {}", g.1, t("sat", "vệ tinh")));
        let (sat, sat_col) = match (from, v.gps_fix()) {
            (None, _) => (format!("   {}", t("no GPS", "không có GPS")), AMBER),
            (Some(Place::Last), _) => (format!("   {}", t("no GPS · last place", "không có GPS · vị trí cũ")), AMBER),
            // (the receiver's place alone: the autopilot does not hold position by it yet)
            (Some(Place::Receiver), _) => (format!("   {} · {}", sats.unwrap_or_default(), t("GPS only", "chỉ GPS")), AMBER),
            (Some(Place::Autopilot), Some((fix, _))) if sats.is_some() => (format!("   {}", sats.unwrap()), if fix >= 3 { WHITE } else { AMBER }),
            _ => (String::new(), WHITE),
        };
        let line = tag(&p, at.tl, Align2::LEFT_TOP, &format!("N ↑{sat}"), fs, sat_col);
        if let Some(text) = home_text {
            tag(&p, line.left_bottom() + vec2(0.0, 4.0), Align2::LEFT_TOP, &text, fs, WHITE);
        }

        // home's mark and the aircraft, over those words. Off the map, home's mark stays at the
        // edge, the way it lies (above the words along the bottom)
        if let Some(hp) = home_pt {
            let edge = if large { 22.0 } else { 13.0 };
            let inner = Rect::from_min_max(r.min + vec2(edge, edge), r.max - vec2(edge, edge + 26.0));
            let (c, d) = (r.center(), hp - r.center());
            let kx = if d.x > 0.0 { (inner.right() - c.x) / d.x } else { (c.x - inner.left()) / -d.x.min(-1e-3) };
            let ky = if d.y > 0.0 { (inner.bottom() - c.y) / d.y } else { (c.y - inner.top()) / -d.y.min(-1e-3) };
            let k = kx.min(ky).min(1.0);
            let (at_edge, rad) = (c + d * k, if k < 1.0 { 7.5 } else { 9.5 });
            p.circle_filled(at_edge, rad + 1.5, Color32::from_black_alpha(150));
            p.circle_filled(at_edge, rad, AMBER);
            p.text(at_edge, Align2::CENTER_CENTER, "H", FontId::proportional(rad * 1.35), Color32::from_rgb(20, 22, 26));
        }
        dart(&p, cp, v.yaw, if large { 17.0 } else { 12.0 }, from == Some(Place::Last));

        // along the bottom: the scale, the tiles' credit
        let bar = (bar_m / m_per_pt) as f32;
        let (a, b) = (at.bl + vec2(1.0, -4.0), at.bl + vec2(1.0 + bar, -4.0));
        for (w, col) in [(5.0, Color32::from_black_alpha(170)), (2.0, WHITE)] {
            let e = (w - 2.0) / 2.0;
            p.line_segment([a - vec2(e, 0.0), b + vec2(e, 0.0)], Stroke::new(w, col));
            p.line_segment([a + vec2(0.0, e), a - vec2(0.0, 5.0 + e)], Stroke::new(w, col));
            p.line_segment([b + vec2(0.0, e), b - vec2(0.0, 5.0 + e)], Stroke::new(w, col));
        }
        tag(&p, at.bl + vec2(0.0, -13.0), Align2::LEFT_BOTTOM, &scale_text(bar_m), fs - 1.0, WHITE);
        if on_globe && !self.tiles.credit.is_empty() {
            tag(&p, at.br, Align2::RIGHT_BOTTOM, &self.tiles.credit, if large { 11.0 } else { 9.5 }, Color32::from_gray(225));
        }
        if !large {
            ui.painter().rect_stroke(r, 0.0, Stroke::new(1.0, Color32::from_white_alpha(90)), egui::StrokeKind::Outside);
        }

        // closer, further; back to the aircraft once dragged away (beside the map, not on it)
        let slot = |i: usize, w: f32| Rect::from_min_size(pos2(at.zoom.x - w, at.zoom.y + i as f32 * 47.0), vec2(w, 42.0));
        if button(ui, slot(0, 46.0), "map+", "+", 22.0) {
            self.zoom_by(on_globe, 1.0);
        }
        if button(ui, slot(1, 46.0), "map-", "−", 22.0) {
            self.zoom_by(on_globe, -1.0);
        }
        if self.centre.is_some() && button(ui, slot(2, 124.0), "map-follow", t("Follow", "Theo máy bay"), 14.0) {
            self.centre = None;
        }
        resp.clicked()
    }

    /// The tiles under the view, and the list of them for the loader. A tile not in memory yet
    /// shows as the part of a shallower one that covers it, if one is.
    fn ground(&mut self, ctx: &egui::Context, p: &egui::Painter, r: Rect, mid: (f64, f64), scale: f64, large: bool) {
        let z = ((scale / 256.0).log2().round() as i32).clamp(0, self.tiles.max_zoom as i32) as u8;
        let n = (1u64 << z) as f64;
        let (cx, cy) = (r.center().x as f64, r.center().y as f64);
        let sx = |i: i64| (cx + (i as f64 / n - mid.0) * scale) as f32;
        let sy = |j: i64| (cy + (j as f64 / n - mid.1) * scale) as f32;
        // (a little darker under the HUD's white words)
        let tint = if large { Color32::from_gray(205) } else { Color32::from_gray(235) };
        let whole = Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0));
        let (xs, ys) = tile_span(mid, scale, r.width(), r.height(), z);
        let mut want = Vec::new();
        for j in ys {
            for i in xs.clone() {
                let k = Key { z, x: i.rem_euclid(n as i64) as u32, y: j as u32 };
                let rect = Rect::from_min_max(pos2(sx(i), sy(j)), pos2(sx(i + 1), sy(j + 1)));
                let far = (i as f64 + 0.5 - mid.0 * n).powi(2) + (j as f64 + 0.5 - mid.1 * n).powi(2);
                want.push((far, k));
                if let Some(id) = self.tiles.get(k) {
                    p.image(id, rect, whole, tint);
                    continue;
                }
                for up in 1..=6 {
                    let Some((pk, [u, v, s])) = k.parent(up) else { break };
                    if let Some(id) = self.tiles.get(pk) {
                        p.image(id, rect, Rect::from_min_size(pos2(u, v), vec2(s, s)), tint);
                        break;
                    }
                }
            }
        }
        want.sort_by(|a, b| a.0.total_cmp(&b.0));
        self.tiles.want(ctx, want.into_iter().map(|w| w.1).collect());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // (the numbers: the slippy map formulas and the haversine, worked in Python)
    #[test]
    fn the_world_square_and_its_tiles() {
        let (x, y) = merc(47.397742, 8.545594);
        assert!((x * 131072.0 - 68647.3558).abs() < 1e-3 && (y * 131072.0 - 45888.3266).abs() < 1e-3, "{x} {y}");
        let (x0, y0) = merc(0.0, 0.0);
        assert!((x0 - 0.5).abs() < 1e-12 && (y0 - 0.5).abs() < 1e-12);
        // a view 230 by 170 points at zoom 17: the tile it is in and the ones it reaches into
        let (xs, ys) = tile_span((x, y), 256.0 * 131072.0, 230.0, 170.0, 17);
        assert_eq!((xs, ys), (68646..=68647, 45887..=45888));
        // the same view from level 16's tiles (each twice as large on the screen)
        let (xs, ys) = tile_span((x, y), 256.0 * 131072.0, 230.0, 170.0, 16);
        assert_eq!((xs, ys), (34323..=34323, 22943..=22944));
        // the rows stop at the world's edge, the columns go round it
        let (xs, ys) = tile_span((0.0, 0.0), 256.0 * 4.0, 600.0, 600.0, 2);
        assert_eq!((xs, ys), (-2..=1, 0..=1));
    }

    #[test]
    fn how_far_and_which_way() {
        let (d, b) = dist_bearing((47.397742, 8.545594), (47.398742, 8.546594));
        assert!((d - 134.27).abs() < 0.05 && (b - 34.09).abs() < 0.05, "{d} {b}");
        let (d, b) = dist_bearing((47.397742, 8.545594), (47.397742, 8.535594));
        assert!((d - 752.69).abs() < 0.05 && (b - 270.0).abs() < 0.05, "{d} {b}");
        let (d, b) = dist_bearing((-35.363261, 149.165230), (-35.373261, 149.165230));
        assert!((d - 1111.95).abs() < 0.05 && (b - 180.0).abs() < 0.05, "{d} {b}");
        assert_eq!(dist_bearing((10.0, 20.0), (10.0, 20.0)).0, 0.0);
    }

    #[test]
    fn round_lengths() {
        for (m, want) in [(0.37, 0.2), (1.0, 1.0), (4.99, 2.0), (5.0, 5.0), (73.0, 50.0), (1999.0, 1000.0), (2400.0, 2000.0)] {
            assert!((nice(m) - want).abs() < 1e-9, "{m}: {}", nice(m));
        }
        assert_eq!((scale_text(0.2), scale_text(50.0), scale_text(2000.0)), ("20 cm".into(), "50 m".into(), "2 km".into()));
        assert_eq!((dist_text(12.4), dist_text(1234.0), dist_text(12345.0)), ("12 m".into(), "1.23 km".into(), "12.3 km".into()));
    }
}
