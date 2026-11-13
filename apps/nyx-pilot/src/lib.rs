//! nyxhop-pilot: the ground station a pilot flies with. The ground end of the link as in nyxhop
//! (the video, nyx-rx underneath), the aircraft's flight controller (PX4) over the same link:
//! its attitude, height, speed and heading drawn over the video, two sticks on the screen (mouse
//! or touch), a gamepad or an RC transmitter plugged in as a joystick, the keyboard; arm, take off,
//! land, return, hold, and the flight mode.
//!
//!   nyxhop-pilot [--board 192.168.0.11]   the ground board (its link's receive end)
//!   ... --demo                            fly a short demo by the sticks on its own (take off,
//!                                         forward, turn, sideways, climb, back, land). No
//!                                         aircraft answers: a made-up one flies a loop, so the
//!                                         map and the HUD show with no hardware
//!   ... --demo-at <lat,lon>               where the made-up one flies; --no-gps: it has no
//!                                         satellite fix, as indoors
//!   ... --tiles <address>                 the map's tiles: a slippy map's address with {z} {x}
//!                                         {y} (OpenStreetMap's standard tiles by default)
//!   ... --tiles-credit <text>             the credit that server asks for on the map
//!   ... --tiles-max-zoom <n>              its deepest zoom level (19)
//!   ... --map-cache <folder>              where the tiles seen are kept (map-cache beside the
//!                                         program): what was seen once shows with no internet
//!   ... --settings [--tab <page>]         open with Settings showing (safety, control, sensors, video,
//!                                         link or about)
//!
//! Keys: arrows roll and pitch, W/S throttle, A/D yaw; T take off, L land, R return, H hold,
//! 1/2/3 Position / Altitude / Stabilized; M the map large and the video small, or back; + and -
//! zoom the map; Esc closes Settings. None of them while a text box in Settings has the keys.
//!
//! Settings (the button by the mode's panel, as DJI's): the flight's safety - what the aircraft
//! does when the signal is lost (return home, land, hover), its return height (the flight
//! controller's parameters, over MAVLink), the take-off height - then the link, the same sections
//! as nyxhop's ground drawer on the ground board's console: the link's state, Auto or fixed
//! channels, the licence, the radio (frequencies, gains, AGC, power, bandwidth).
//!
//! The map (map.rs) is small under the link's panel, or large with the video in its place: a tap
//! on the small one, or M. It draws tiles while the aircraft tells where it is on the globe
//! (GLOBAL_POSITION_INT), a grid in metres and its local track when it does not.
//!
//! MAVLink goes through nyx-rx's own UDP pipe on private ports (it listens on 127.0.0.1:14565 for
//! what goes up, hands what comes down to 127.0.0.1:14568), so nyxhop can stay installed beside.
//! The sticks go up 50 times a second as MANUAL_CONTROL: PX4 takes them with COM_RC_IN_MODE 1.
//! ArduPilot ignores them in GUIDED (its take-off's mode): LOITER once the take-off is done or a
//! stick moves.

mod map;
mod mav;
mod rc;
mod tiles;

use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, Vec2, pos2, vec2};
use mav::{Autopilot, Encoder, FlightMode, Msg, Place, Vehicle};
use nyx_common::Opts;
use nyx_common::i18n::t;
use nyx_common::logging::log;
use nyx_common::theme as th;
use nyx_common::ui as nu;


const UP: &str = "127.0.0.1:14565";
const DOWN: &str = "127.0.0.1:14568";

/// Stick positions: roll, pitch (forward +), yaw (right +) in -1..1; throttle -1..1 (0 = hold)
#[derive(Clone, Copy, Default)]
struct Sticks {
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32,
}

enum Cmd {
    Arm(bool),
    Takeoff(f32),
    Land,
    Return,
    Mode(FlightMode),
    /// the emergency stop: disarmed, forced (in the air too) - the motors stop at once
    Kill,
    /// a flight controller's parameter to this number (PARAM_SET), and the ones to read back
    SetParam(&'static str, f64),
    ReadParams(&'static [&'static str]),
    /// a sensor calibration on the flight controller (None: the one running cancelled)
    Calibrate(Option<CalKind>),
}

/// The calibrations Settings > Safety > Sensors runs, as DJI's IMU and compass calibration
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CalKind {
    Gyro,
    Accel,
    Level,
    Mag,
}

impl CalKind {
    /// PREFLIGHT_CALIBRATION's seven params for it
    fn params(self) -> [f32; 7] {
        match self {
            CalKind::Gyro => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            CalKind::Mag => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            CalKind::Accel => [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            CalKind::Level => [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
        }
    }

    fn name(self) -> &'static str {
        match self {
            CalKind::Gyro => t("gyro", "gyro"),
            CalKind::Accel => t("accelerometer", "gia tốc kế"),
            CalKind::Level => t("level", "cân mặt phẳng"),
            CalKind::Mag => t("compass", "la bàn"),
        }
    }
}

/// A side of the vehicle as PX4's calibration names it, as the pilot puts it down
fn side_name(side: &str) -> &'static str {
    match side {
        "down" => t("level, as it flies", "đặt bằng, như lúc bay"),
        "up" => t("upside down", "lật ngửa"),
        "left" => t("on its left side", "nghiêng, cạnh trái chạm đất"),
        "right" => t("on its right side", "nghiêng, cạnh phải chạm đất"),
        "front" => t("nose down", "chúc mũi xuống đất"),
        "back" => t("tail down (nose up)", "dựng đuôi xuống đất (mũi chĩa lên)"),
        _ => "?",
    }
}

/// How the board sits on the frame (SENS_BOARD_ROT): turned about the vertical, or upside down
const BOARD_ROT: [(&str, &str, f64); 5] = [("0°", "0°", 0.0), ("90°", "90°", 2.0), ("180°", "180°", 4.0), ("270°", "270°", 6.0), ("Upside down", "Lật ngược", 8.0)];

/// What the aircraft does when the signal is lost, as the pilot picks it (Settings, as DJI's
/// "signal lost: return / land / hover"). PX4: NAV_RCL_ACT (the sticks lost) and NAV_DLL_ACT (the
/// ground station lost), both the same; ArduPilot: FS_GCS_ENABLE (its sticks come from the ground
/// station). Return without a satellite fix: the autopilot lands where it is.
#[derive(Clone, Copy, PartialEq, Eq)]
enum LostAction {
    Return,
    Land,
    Hover,
    /// PX4 NAV_RCL_ACT 6: shown only when the aircraft is set so (a bench without propellers)
    Disarm,
}

impl LostAction {
    const ALL: [LostAction; 4] = [LostAction::Return, LostAction::Land, LostAction::Hover, LostAction::Disarm];

    fn key(self) -> &'static str {
        match self {
            LostAction::Return => "return",
            LostAction::Land => "land",
            LostAction::Hover => "hover",
            LostAction::Disarm => "disarm",
        }
    }

    /// PX4 1 Hold, 2 Return, 3 Land, 6 Disarm
    fn px4(self) -> f64 {
        match self {
            LostAction::Return => 2.0,
            LostAction::Land => 3.0,
            LostAction::Hover => 1.0,
            LostAction::Disarm => 6.0,
        }
    }

    /// ArduPilot Copter FS_GCS_ENABLE: 1 RTL, 5 Land, 7 Brake or Land (no disarm there)
    fn ardu(self) -> Option<f64> {
        match self {
            LostAction::Return => Some(1.0),
            LostAction::Land => Some(5.0),
            LostAction::Hover => Some(7.0),
            LostAction::Disarm => None,
        }
    }

    fn of(ap: Autopilot, n: f64) -> Option<LostAction> {
        Self::ALL.into_iter().find(|a| match ap {
            Autopilot::Px4 => a.px4() == n,
            Autopilot::ArduPilot => a.ardu() == Some(n),
        })
    }
}

/// The flight controller's parameters the Settings show
const PX4_SAFETY: &[&str] = &[
    "NAV_RCL_ACT",
    "NAV_DLL_ACT",
    "RTL_RETURN_ALT",
    "COM_LOW_BAT_ACT",
    "GF_MAX_VER_DIST",
    "GF_MAX_HOR_DIST",
    "MPC_THR_MAX",
    "NYX_GRD_TILT",
    "NYX_GRD_YAWR",
    "NYX_GRD_HGT",
    "SENS_BOARD_ROT",
];
/// (ArduCopter 4.7 has RTL_ALT_M in metres; before, RTL_ALT in centimetres: whichever answers)
const ARDU_SAFETY: &[&str] = &["FS_GCS_ENABLE", "RTL_ALT_M", "RTL_ALT", "BATT_FS_LOW_ACT"];

/// What it does on a low battery: PX4 COM_LOW_BAT_ACT 0 warning, 3 return (land at the emergency
/// level), 2 land; ArduPilot BATT_FS_LOW_ACT 0 none, 2 RTL, 1 land
const LOW_BAT: [(&str, &str, f64, f64); 3] =
    [("Warn only", "Chỉ cảnh báo", 0.0, 0.0), ("Return home", "Quay về", 3.0, 2.0), ("Land", "Hạ cánh", 2.0, 1.0)];

/// DJI's C / N / S: how fast the sticks fly it, in its position mode (PX4 Position, ArduPilot
/// LOITER). The same mode, its limits set on the flight controller: horizontal speed and
/// acceleration, climb and descent, yaw rate, tilt.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Speed {
    Cine,
    Normal,
    Sport,
}

impl Speed {
    const ALL: [Speed; 3] = [Speed::Cine, Speed::Normal, Speed::Sport];

    fn letter(self) -> &'static str {
        match self {
            Speed::Cine => "C",
            Speed::Normal => "N",
            Speed::Sport => "S",
        }
    }

    fn name(self) -> &'static str {
        match self {
            Speed::Cine => "Cine",
            Speed::Normal => "Normal",
            Speed::Sport => "Sport",
        }
    }

    fn key(self) -> &'static str {
        match self {
            Speed::Cine => "cine",
            Speed::Normal => "normal",
            Speed::Sport => "sport",
        }
    }

    fn of_key(k: &str) -> Option<Speed> {
        Self::ALL.into_iter().find(|s| s.key() == k)
    }

    /// horizontal m/s, acceleration m/s², climb m/s, descent m/s, yaw °/s, tilt °
    fn limits(self) -> [f64; 6] {
        match self {
            Speed::Cine => [3.0, 2.0, 1.5, 1.0, 45.0, 20.0],
            Speed::Normal => [8.0, 4.0, 3.0, 1.5, 90.0, 30.0],
            Speed::Sport => [12.0, 7.0, 5.0, 2.5, 150.0, 40.0],
        }
    }

    /// The flight controller's parameters for them, as each names them (ArduCopter 4.7: metres)
    fn params(self, ap: Autopilot) -> [(&'static str, f64); 6] {
        let [v, a, up, dn, yaw, tilt] = self.limits();
        match ap {
            Autopilot::Px4 => [
                ("MPC_VEL_MANUAL", v),
                ("MPC_ACC_HOR_MAX", a),
                ("MPC_Z_VEL_MAX_UP", up),
                ("MPC_Z_VEL_MAX_DN", dn),
                ("MPC_MAN_Y_MAX", yaw),
                ("MPC_MAN_TILT_MAX", tilt),
            ],
            Autopilot::ArduPilot => [
                ("LOIT_SPEED_MS", v),
                ("LOIT_ACC_MAX_M", a),
                ("PILOT_SPD_UP", up),
                ("PILOT_SPD_DN", dn),
                ("PILOT_Y_RATE", yaw),
                ("LOIT_ANG_MAX", tilt),
            ],
        }
    }

    /// The one the flight controller flies with now, by its horizontal speed (what it told)
    fn told(v: &Vehicle) -> Option<Speed> {
        let id = Self::Normal.params(v.autopilot)[0].0;
        let n = v.param(id)?;
        Self::ALL.into_iter().find(|s| (s.limits()[0] - n).abs() < 0.05)
    }
}

/// The link's own distance reading (two-way ranging on the boards), an extra: on only when the
/// pilot set it so, and then only while there are no satellites - 12/11, user. On once none for
/// RANGE_ON_AFTER, off once they held again for RANGE_OFF_AFTER (a fix that comes and goes does not
/// switch it back and forth). `gps`: a position from them now; `since`: how long that has held;
/// `cur`: on now.
fn range_wanted(enabled: bool, gps: bool, cur: bool, since: Duration) -> bool {
    if !enabled {
        return false;
    }
    if cur { !(gps && since >= RANGE_OFF_AFTER) } else { !gps && since >= RANGE_ON_AFTER }
}
const RANGE_ON_AFTER: Duration = Duration::from_secs(3);
const RANGE_OFF_AFTER: Duration = Duration::from_secs(20);

/// Take off, land, return home, the motors: held this long (a touch alone does nothing)
const HOLD_S: f32 = 1.0;

/// A held button (`holds`: the ones down, since when): its progress 0..1 at `now`, and true once
/// when it has been held `HOLD_S` - then nothing until it is let go and held again
fn held(holds: &mut std::collections::HashMap<&'static str, Instant>, id: &'static str, down: bool, now: Instant) -> (f32, bool) {
    if !down {
        holds.remove(id);
        return (0.0, false);
    }
    let t0 = *holds.entry(id).or_insert(now);
    // (fired: "since" an hour on, so it reads as done until let go)
    let Some(d) = now.checked_duration_since(t0) else { return (1.0, false) };
    let k = d.as_secs_f32() / HOLD_S;
    if k >= 1.0 {
        holds.insert(id, now + Duration::from_secs(3600));
        return (1.0, true);
    }
    (k, false)
}

/// The Settings' pages, as DJI's
#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Safety,
    Control,
    Sensors,
    Video,
    Link,
    About,
}

/// The sticks' layout, as DJI's modes: which stick moves what. 2: left yaw and throttle, right
/// roll and pitch; 1: left yaw and pitch, right roll and throttle; 3: left roll and pitch, right
/// yaw and throttle.
fn sticks_of(mode: u8, left: Vec2, right: Vec2) -> Sticks {
    match mode {
        1 => Sticks { roll: right.x, pitch: left.y, yaw: left.x, throttle: right.y },
        3 => Sticks { roll: left.x, pitch: left.y, yaw: right.x, throttle: right.y },
        _ => Sticks { roll: right.x, pitch: right.y, yaw: left.x, throttle: left.y },
    }
}

/// ... and back: where each stick's knob is for these stick values
fn knobs_of(mode: u8, s: Sticks) -> (Vec2, Vec2) {
    match mode {
        1 => (vec2(s.yaw, s.pitch), vec2(s.roll, s.throttle)),
        3 => (vec2(s.roll, s.pitch), vec2(s.yaw, s.throttle)),
        _ => (vec2(s.yaw, s.throttle), vec2(s.roll, s.pitch)),
    }
}

/// The names under the sticks
fn stick_names(mode: u8) -> (&'static str, &'static str) {
    match mode {
        1 => (t("yaw · forward", "xoay · tới lui"), t("roll · throttle", "nghiêng · ga")),
        3 => (t("roll · forward", "nghiêng · tới lui"), t("yaw · throttle", "xoay · ga")),
        _ => (t("yaw · throttle", "xoay · ga"), t("roll · pitch", "nghiêng · tới lui")),
    }
}

/// "3D fix · 12 satellites" (255: the receiver does not tell)
fn tf_sats(fix: &str, sats: u8) -> String {
    if sats == 255 {
        fix.to_string()
    } else {
        format!("{fix} · {sats} {}", t("satellites", "vệ tinh"))
    }
}

/// Softer around the middle (DJI's EXP): the stick's share `expo` goes as its cube
fn expo(x: f32, e: f32) -> f32 {
    (1.0 - e) * x + e * x * x * x
}

/// The app's own settings, beside the program (nyxhop-pilot.cfg, key=value lines)
#[derive(Clone, Copy, PartialEq, Debug)]
struct AppCfg {
    takeoff_h: f32,
    stick_mode: u8,
    expo: f32,
    speed: Speed,
    /// the link's own distance reading, while there are no satellites (off: never)
    range_nogps: bool,
    /// the emergency stop on the screen (and Space)
    estop: bool,
    /// the tether test (Settings > Safety): on, and its limits - tilt deg, yaw rate deg/s, the
    /// most throttle %, height m (0: none) - and what the flight controller had before it
    tether: bool,
    tether_tilt: f32,
    tether_yawr: f32,
    tether_thr: f32,
    tether_hgt: f32,
    tether_prev: Option<[f64; 3]>,
    /// an RC transmitter's channel order (Settings > Control)
    rc_order: rc::Order,
    /// its throttle stick springs back to the middle (as DJI's; a GX15 or TX16S gimbal set so):
    /// read as a gamepad's then - the middle holds, in Stabilized too - not as the throttle itself
    rc_center: bool,
}

/// The flight controller's parameters the tether test sets (PX4: the guard is nyx_dshot's): the
/// guard's limits, the most throttle, and the motors off when the sticks or the link are lost
fn tether_params(c: &AppCfg) -> Vec<(&'static str, f64)> {
    vec![
        ("NYX_GRD_TILT", f64::from(c.tether_tilt)),
        ("NYX_GRD_YAWR", f64::from(c.tether_yawr)),
        ("NYX_GRD_HGT", f64::from(c.tether_hgt)),
        ("NYX_GRD_T", 0.3),
        ("MPC_THR_MAX", f64::from(c.tether_thr) / 100.0),
        ("NAV_RCL_ACT", 6.0),
        ("NAV_DLL_ACT", 6.0),
    ]
}

/// ... and back as they were (the guard off; the throttle and the signal-lost actions as the
/// flight controller had them when the test came on)
fn tether_off_params(prev: [f64; 3]) -> Vec<(&'static str, f64)> {
    vec![
        ("NYX_GRD_TILT", 0.0),
        ("NYX_GRD_YAWR", 0.0),
        ("NYX_GRD_HGT", 0.0),
        ("MPC_THR_MAX", prev[0]),
        ("NAV_RCL_ACT", prev[1]),
        ("NAV_DLL_ACT", prev[2]),
    ]
}

/// What the tether test changes besides the guard: read before it comes on, to put back
const TETHER_KEEP: [&str; 3] = ["MPC_THR_MAX", "NAV_RCL_ACT", "NAV_DLL_ACT"];

/// Take off to this height and hover (DJI's 1.2 m; a little more for a 7-inch quad's own wash)
const TAKEOFF_H: f32 = 1.5;
const TAKEOFF_RANGE: std::ops::RangeInclusive<f32> = 1.0..=30.0;

impl Default for AppCfg {
    fn default() -> Self {
        AppCfg {
            takeoff_h: TAKEOFF_H,
            stick_mode: 2,
            expo: 0.0,
            speed: Speed::Normal,
            range_nogps: false,
            estop: true,
            tether: false,
            tether_tilt: 45.0,
            tether_yawr: 300.0,
            tether_thr: 60.0,
            tether_hgt: 0.0,
            tether_prev: None,
            rc_order: rc::Order::Aetr,
            rc_center: false,
        }
    }
}

impl AppCfg {
    fn load(path: &Path) -> AppCfg {
        let mut c = AppCfg::default();
        let text = std::fs::read_to_string(path).unwrap_or_default();
        // (a file from before the take-off became DJI's short hop has no `speed`: its 10 m was
        // the old default, not a choice)
        let old = !text.contains("speed=");
        for l in text.lines() {
            let Some((k, v)) = l.split_once('=') else { continue };
            let v = v.trim();
            match k.trim() {
                "takeoff_h" if !old => c.takeoff_h = v.parse::<f32>().ok().filter(|h| TAKEOFF_RANGE.contains(h)).unwrap_or(c.takeoff_h),
                "stick_mode" => c.stick_mode = v.parse::<u8>().ok().filter(|m| (1..=3).contains(m)).unwrap_or(c.stick_mode),
                "expo" => c.expo = v.parse::<f32>().ok().filter(|e| (0.0..=0.8).contains(e)).unwrap_or(c.expo),
                "speed" => c.speed = Speed::of_key(v).unwrap_or(c.speed),
                "range_nogps" => c.range_nogps = v == "1",
                "estop" => c.estop = v != "0",
                "tether" => c.tether = v == "1",
                "tether_tilt" => c.tether_tilt = v.parse::<f32>().ok().filter(|x| (10.0..=90.0).contains(x)).unwrap_or(c.tether_tilt),
                "tether_yawr" => c.tether_yawr = v.parse::<f32>().ok().filter(|x| (30.0..=1000.0).contains(x)).unwrap_or(c.tether_yawr),
                "tether_thr" => c.tether_thr = v.parse::<f32>().ok().filter(|x| (20.0..=100.0).contains(x)).unwrap_or(c.tether_thr),
                "tether_hgt" => c.tether_hgt = v.parse::<f32>().ok().filter(|x| (0.0..=20.0).contains(x)).unwrap_or(c.tether_hgt),
                "tether_prev" => {
                    let n: Vec<f64> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                    c.tether_prev = (n.len() == 3).then(|| [n[0], n[1], n[2]]);
                }
                "rc_order" => c.rc_order = rc::Order::from_key(v).unwrap_or(c.rc_order),
                "rc_center" => c.rc_center = v == "1",
                _ => {}
            }
        }
        c
    }

    fn save(&self, path: &Path) {
        let _ = std::fs::write(
            path,
            format!(
                "takeoff_h={:.1}\nstick_mode={}\nexpo={:.2}\nspeed={}\nrange_nogps={}\nestop={}\ntether={}\ntether_tilt={:.0}\ntether_yawr={:.0}\ntether_thr={:.0}\ntether_hgt={:.1}\nrc_order={}\nrc_center={}\n{}",
                self.takeoff_h,
                self.stick_mode,
                self.expo,
                self.speed.key(),
                u8::from(self.range_nogps),
                u8::from(self.estop),
                u8::from(self.tether),
                self.tether_tilt,
                self.tether_yawr,
                self.tether_thr,
                self.tether_hgt,
                self.rc_order.key(),
                u8::from(self.rc_center),
                self.tether_prev.map_or(String::new(), |p| format!("tether_prev={},{},{}\n", p[0], p[1], p[2]))
            ),
        );
    }
}

/// A gamepad's sticks: this much either side of the middle reads as the middle
const PAD_DZ: f32 = 0.08;

/// The throttle stick moves the held throttle in a manual-thrust mode by this much a second at
/// full deflection (the W key, at half, takes it from 0 to 100 % in 2.5 s)
const THR_RATE: f32 = 0.8;

struct Link {
    veh: Mutex<Vehicle>,
    /// an aircraft's messages came down the link
    heard: AtomicBool,
    sticks: Mutex<Sticks>,
    /// the held throttle 0..1 in a manual-thrust mode (Stabilized), None in the others
    thr: Mutex<Option<f32>>,
    /// the sticks are an RC transmitter's (rc.rs): its throttle stays where it is put, so in a
    /// manual-thrust mode it is the throttle itself (bottom 0, top 100 %), not a rate that moves
    /// the held one; and it alone may arm in one only at the bottom
    rc_thr: AtomicBool,
    /// the flight mode picked by the app as DJI picks P or ATTI (Vehicle::best_mode); off once the
    /// pilot picks one in Advanced modes (or keys 4, 5), on again with its Auto
    auto_mode: AtomicBool,
    /// --demo: the sticks it holds (over every other input) and what it is doing
    demo: Mutex<Option<Sticks>>,
    demo_step: Mutex<String>,
    /// the app is gone: the MAVLink thread ends (and gives its port back)
    stop: AtomicBool,
}

/// Where --demo's made-up aircraft flies unless told (--demo-at): PX4's own simulators' home
const DEMO_AT: (f64, f64) = (47.397742, 8.545594);

/// --demo with no aircraft: one made up, `t` seconds into its flight, as a PX4's messages would
/// tell it. Two seconds on the ground, armed, up to 30 m, then round an oval 140 m by 90 m that
/// passes over where it took off, once in 45 s. `at`: where on the globe; None, it has no
/// satellite fix (indoors) and tells its local position alone.
fn sim_flight(v: &mut Vehicle, t: f32, at: Option<(f64, f64)>) {
    let armed = t > 2.0;
    let (climb, cruise) = ((t - 2.5) / 5.0, t - 7.5);
    let h = climb.clamp(0.0, 1.0) * 30.0;
    let w = std::f32::consts::TAU / 45.0;
    let th = cruise.max(0.0) * w;
    let (n, e) = (70.0 * (1.0 - th.cos()), 45.0 * th.sin());
    let (vn, ve) = if cruise > 0.0 { (70.0 * w * th.sin(), 45.0 * w * th.cos()) } else { (0.0, 0.0) };
    let (roll, pitch) = if cruise > 0.0 { (-0.16, -0.1) } else { (0.0, 0.0) };
    let mut tell = |m: Msg| v.update(1, 1, &m);
    tell(Msg::Heartbeat { custom_mode: 3 << 16, autopilot: 12, base_mode: if armed { 0x81 } else { 1 } });
    tell(Msg::SysStatus { load: 250, voltage_mv: (16_400.0 - t * 2.0).max(14_000.0) as u16, remaining: (95.0 - t / 12.0).max(20.0) as i8, health: 0x1000_0000 });
    tell(Msg::ExtSys { landed: if t < 2.5 { 1 } else { 2 } });
    // (its speed limits as PX4 tells them: Normal's)
    for (id, n) in Speed::Normal.params(Autopilot::Px4) {
        tell(Msg::ParamValue { id: id.into(), value: n as f32, ptype: mav::PARAM_REAL32 });
    }
    tell(Msg::Attitude { roll, pitch, yaw: (45.0 * th.cos()).atan2(70.0 * th.sin()), yawspeed: if cruise > 0.0 { w } else { 0.0 } });
    tell(Msg::LocalPos { x: n, y: e, z: -h, vx: vn, vy: ve, vz: if (0.0..1.0).contains(&climb) { -6.0 } else { 0.0 } });
    if let Some((lat0, lon0)) = at {
        let lat = lat0 + n as f64 / 111_320.0;
        let lon = lon0 + e as f64 / (111_320.0 * lat0.to_radians().cos());
        let int = |deg: f64| (deg * 1e7).round() as i32;
        tell(Msg::GpsRaw { fix: 3, sats: 14, lat: int(lat), lon: int(lon) });
        tell(Msg::GlobalPos { lat: int(lat), lon: int(lon), alt_mm: ((488.0 + h) * 1000.0) as i32, rel_mm: (h * 1000.0) as i32 });
        if armed {
            tell(Msg::Home { lat: int(lat0), lon: int(lon0), alt_mm: 488_000 });
        }
    }
}

/// --demo: a short flight by the sticks (MANUAL_CONTROL), as a pilot would fly it
fn demo(link: Arc<Link>, cmds: Sender<Cmd>, at: Option<(f64, f64)>) {
    let wait = |what: &str, secs: f32, ok: &dyn Fn(&Vehicle) -> bool| {
        let t0 = Instant::now();
        while t0.elapsed().as_secs_f32() < secs {
            if ok(&link.veh.lock().unwrap()) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        log(&format!("demo: no {what} after {secs} s"));
        false
    };
    let step = |label: &str| {
        log(&format!("demo: {label}"));
        *link.demo_step.lock().unwrap() = label.to_string();
    };
    step("waiting for the aircraft");
    // none answers: a made-up one flies meanwhile (the map and the HUD, with no hardware)
    let t0 = Instant::now();
    let mut sim: Option<Instant> = None;
    while !link.heard.load(Ordering::Relaxed) {
        if sim.is_none() && t0.elapsed() >= Duration::from_secs(4) {
            step("no aircraft: a simulated flight");
            sim = Some(Instant::now());
        }
        if let Some(since) = sim {
            sim_flight(&mut link.veh.lock().unwrap(), since.elapsed().as_secs_f32(), at);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    if sim.is_some() {
        *link.veh.lock().unwrap() = Vehicle::default();
        step("waiting for the aircraft");
    }
    if !wait("telemetry", 120.0, &|v| v.link()) {
        return;
    }
    std::thread::sleep(Duration::from_secs(3));
    step("take off to 10 m");
    let _ = cmds.send(Cmd::Takeoff(10.0));
    if !wait("height", 40.0, &|v| -v.z > 9.3) {
        return;
    }
    std::thread::sleep(Duration::from_secs(2));
    step("position mode: the sticks fly it");
    let _ = cmds.send(Cmd::Mode(FlightMode::Position));
    std::thread::sleep(Duration::from_millis(1500));
    let s = |roll: f32, pitch: f32, yaw: f32, throttle: f32| Sticks { roll, pitch, yaw, throttle };
    for (label, st, secs) in [
        ("stick forward", s(0.0, 0.7, 0.0, 0.0), 4.0),
        ("sticks centred: it holds", s(0.0, 0.0, 0.0, 0.0), 2.5),
        ("yaw right", s(0.0, 0.0, 0.6, 0.0), 3.0),
        ("sticks centred", s(0.0, 0.0, 0.0, 0.0), 1.5),
        ("forward and right", s(0.6, 0.6, 0.0, 0.0), 4.0),
        ("sticks centred", s(0.0, 0.0, 0.0, 0.0), 2.0),
        ("climb", s(0.0, 0.0, 0.0, 0.6), 3.0),
        ("sticks centred", s(0.0, 0.0, 0.0, 0.0), 2.0),
        ("stick back", s(0.0, -0.7, 0.0, 0.0), 5.0),
        ("sticks centred", s(0.0, 0.0, 0.0, 0.0), 2.0),
    ] {
        step(label);
        *link.demo.lock().unwrap() = Some(st);
        std::thread::sleep(Duration::from_secs_f32(secs));
    }
    *link.demo.lock().unwrap() = None;
    step("land");
    let _ = cmds.send(Cmd::Land);
    wait("landing", 60.0, &|v| -v.z < 0.3 && v.vz.abs() < 0.2);
    std::thread::sleep(Duration::from_secs(2));
    step("");
    log("demo: done");
}

/// MAVLink with the aircraft: what comes down updates the vehicle; up go the sticks (50 Hz), the
/// ground station's heartbeat and a TIMESYNC (1 Hz), the stream rates (every 5 s) and the commands.
fn link_thread(link: Arc<Link>, cmds: Receiver<Cmd>, up: String, down: String) {
    let sock = match UdpSocket::bind(&down) {
        Ok(s) => s,
        Err(e) => {
            log(&format!("pilot: cannot listen on {down}: {e}"));
            return;
        }
    };
    let _ = sock.set_read_timeout(Some(Duration::from_millis(5)));
    let up: SocketAddr = up.parse().unwrap();
    let mut enc = Encoder::default();
    let send = |b: Vec<u8>| {
        let _ = sock.send_to(&b, up);
    };
    let t0 = Instant::now();
    let mut buf = [0u8; 2048];
    let (mut t_mc, mut t_hb, mut t_rates) = (t0, t0, t0 - Duration::from_secs(10));
    let mut pending_ts: Vec<(i64, Instant)> = Vec::new();
    let mut takeoff_at: Option<(Instant, f32)> = None;
    // our take-off, under way: sent at, to this many metres above where it stood (its local
    // height then): the hover it ends in hands over to the sticks (PX4 Hold, ArduPilot GUIDED)
    let mut handover: Option<(Instant, f32, f32)> = None;
    let mut t_loiter = t0;
    // parameters to set whose type PX4 has not told yet: asked for first, set once it tells
    // (PX4 takes PARAM_SET's four bytes as its own type: an integer's bits as a float are junk)
    let mut pending_sets: Vec<(&'static str, f64, Instant)> = Vec::new();
    let (mut n_msgs, mut t_rate) = (0u32, t0);
    // Stabilized: the throttle is the thrust, not "hold the height", so a stick that springs back
    // to the middle would leave half thrust on (and PX4 refuses to arm: "high throttle"). There
    // the sticks move a throttle that stays where it is left: 0 while disarmed, the vehicle's own
    // throttle when the mode comes on armed (from Position or Altitude in flight).
    let mut thr: Option<f32> = None;
    // the auto mode's last change, and an arm sent once the mode it needed has come on
    let mut t_auto = t0;
    let mut arm_at: Option<Instant> = None;
    loop {
        if link.stop.load(Ordering::Relaxed) {
            log("pilot: MAVLink thread done");
            return;
        }
        if let Ok(n) = sock.recv(&mut buf) {
            let msgs = mav::parse(&buf[..n]);
            if !msgs.is_empty() {
                link.heard.store(true, Ordering::Relaxed);
            }
            let mut v = link.veh.lock().unwrap();
            for (sys, comp, m) in msgs {
                n_msgs += 1;
                if let Msg::Timesync { tc1, ts1 } = m {
                    if tc1 != 0 {
                        if let Some(i) = pending_ts.iter().position(|p| p.0 == ts1) {
                            v.rtt_ms = Some(pending_ts[i].1.elapsed().as_secs_f32() * 1000.0);
                            pending_ts.remove(i);
                        }
                    }
                    continue;
                }
                if let Msg::Text { ref text, .. } = m {
                    log(&format!("autopilot: {text}"));
                }
                v.update(sys, comp, &m);
            }
        }
        let now = Instant::now();
        let (sys, comp) = {
            let v = link.veh.lock().unwrap();
            (if v.sys == 0 { 1 } else { v.sys }, if v.comp == 0 { 1 } else { v.comp })
        };
        if now - t_mc >= Duration::from_millis(20) {
            let dt = (now - t_mc).as_secs_f32().min(0.1);
            t_mc = now;
            let s = *link.sticks.lock().unwrap();
            let k = |v: f32| (v.clamp(-1.0, 1.0) * 1000.0) as i16;
            let mut z = (500.0 + 500.0 * s.throttle.clamp(-1.0, 1.0)) as i16;
            let (ardu, armed, manual, vfr) = {
                let v = link.veh.lock().unwrap();
                (v.autopilot == Autopilot::ArduPilot, v.armed, v.manual_thrust(), v.throttle_pct)
            };
            let rc_thr = link.rc_thr.load(Ordering::Relaxed);
            if manual {
                let level = match thr {
                    _ if !armed => 0.0,
                    _ if rc_thr => (s.throttle.clamp(-1.0, 1.0) + 1.0) / 2.0,
                    Some(l) => l + s.throttle.clamp(-1.0, 1.0) * THR_RATE * dt,
                    None => vfr as f32 / 100.0,
                }
                .clamp(0.0, 1.0);
                thr = Some(level);
                z = (level * 1000.0) as i16;
            } else {
                thr = None;
                // ArduPilot arms only with the throttle down: down while disarmed and left alone
                // (armed, the middle holds the height, as on PX4)
                if ardu && !armed && s.throttle.abs() < 0.05 {
                    z = 0;
                }
            }
            *link.thr.lock().unwrap() = thr;
            send(enc.manual_control(sys, k(s.pitch), k(s.roll), z, k(s.yaw)));
            // the sticks always fly it, as DJI's: from the take-off's hover and from a pause (PX4
            // Takeoff and Hold, ArduPilot GUIDED and BRAKE - GUIDED ignores the sticks) to the
            // position mode as soon as a stick moves, or once our take-off is done. Return and
            // Land go on (a failsafe's or the pilot's own: the Hold button stops them).
            let (mode, sub, up) = {
                let v = link.veh.lock().unwrap();
                (v.main_mode, v.sub_mode, -v.z)
            };
            let paused = if ardu { matches!(mode, 4 | 17) } else { mode == 4 && matches!(sub, 2 | 3) };
            if !armed {
                handover = None;
            } else if paused && now - t_loiter >= Duration::from_secs(1) {
                // (an RC transmitter's throttle rests anywhere: its sticks that spring back alone)
                let thr_moved = if rc_thr { 0.0 } else { s.throttle };
                let moved = [s.roll, s.pitch, s.yaw, thr_moved].iter().any(|v| v.abs() > 0.15);
                let done = handover.is_some_and(|(at, h, z0)| {
                    up - z0 >= h - 0.3 || at.elapsed() >= Duration::from_secs(20) || (!ardu && sub == 3 && at.elapsed() >= Duration::from_secs(3))
                });
                if moved || done {
                    t_loiter = now;
                    handover = None;
                    let ap = if ardu { Autopilot::ArduPilot } else { Autopilot::Px4 };
                    let (m, s2) = FlightMode::Position.params(ap);
                    send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, m, s2, 0.0, 0.0, 0.0, 0.0]));
                    log(if moved { "pilot: a stick moved in a hover - position mode" } else { "pilot: take-off done - position mode" });
                }
            }
        }
        if now - t_hb >= Duration::from_secs(1) {
            t_hb = now;
            send(enc.heartbeat());
            let ts = t0.elapsed().as_nanos() as i64 + 1;
            pending_ts.push((ts, now));
            pending_ts.retain(|p| p.1.elapsed() < Duration::from_secs(5));
            send(enc.timesync(ts));
            let mut v = link.veh.lock().unwrap();
            v.rate = n_msgs as f32 / t_rate.elapsed().as_secs_f32();
            (n_msgs, t_rate) = (0, now);
        }
        if now - t_rates >= Duration::from_secs(5) {
            t_rates = now;
            // ATTITUDE 30 Hz, LOCAL_POSITION_NED 20, VFR_HUD 10, EXTENDED_SYS_STATE 2, HOME 1; for
            // the map GLOBAL_POSITION_INT 10, GPS_RAW_INT 1; for the auto mode ESTIMATOR_STATUS 2;
            // for Settings > Sensors DISTANCE_SENSOR and OPTICAL_FLOW_RAD 5
            for (id, hz) in [(30.0, 30.0), (32.0, 20.0), (74.0, 10.0), (245.0, 2.0), (242.0, 1.0), (33.0, 10.0), (24.0, 1.0), (230.0, 2.0), (132.0, 5.0), (106.0, 5.0)] {
                send(enc.command_long(sys, comp, mav::MAV_CMD_SET_MESSAGE_INTERVAL, [id, 1e6 / hz, 0.0, 0.0, 0.0, 0.0, 0.0]));
            }
        }
        let ap = link.veh.lock().unwrap().autopilot;
        // the mode as DJI picks P or ATTI, on the ground: the best its estimator allows - PX4
        // comes up in Position, which indoors with nothing to hold a position by refuses to arm
        let auto = link.auto_mode.load(Ordering::Relaxed);
        let (armed, cur, best) = {
            let v = link.veh.lock().unwrap();
            (v.armed, v.flight_mode(), v.best_mode())
        };
        let flyable = |m: Option<FlightMode>| matches!(m, Some(FlightMode::Position | FlightMode::Altitude | FlightMode::Stabilized));
        let want = if auto && flyable(cur) && flyable(best) && cur != best { best } else { None };
        if let Some(b) = want {
            if !armed && now - t_auto >= Duration::from_secs(2) {
                t_auto = now;
                let (main, sub) = b.params(ap);
                send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, main, sub, 0.0, 0.0, 0.0, 0.0]));
                log(&format!("pilot: auto mode {} -> {b:?} (what its estimator holds)", cur.map_or("-".to_string(), |c| format!("{c:?}"))));
            }
        }
        if arm_at.is_some_and(|t| now >= t) {
            arm_at = None;
            send(enc.command_long(sys, comp, mav::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
        }
        while let Ok(c) = cmds.try_recv() {
            let arm = |enc: &mut Encoder, on: bool| {
                enc.command_long(sys, comp, mav::MAV_CMD_COMPONENT_ARM_DISARM, [if on { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            };
            match c {
                // the motors on in a mode it could not fly (Position indoors): the auto mode first,
                // the arm once it is on
                Cmd::Arm(true) if want.is_some() => {
                    let b = want.unwrap();
                    let (main, sub) = b.params(ap);
                    send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, main, sub, 0.0, 0.0, 0.0, 0.0]));
                    t_auto = now;
                    arm_at = Some(now + Duration::from_millis(500));
                }
                Cmd::Arm(on) => {
                    if !on {
                        arm_at = None;
                    }
                    send(arm(&mut enc, on));
                }
                Cmd::Kill => {
                    // (21196: forced, PX4 and ArduPilot alike; three times, a lost one does not count)
                    for _ in 0..3 {
                        send(enc.command_long(sys, comp, mav::MAV_CMD_COMPONENT_ARM_DISARM, [0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
                    }
                    log("pilot: EMERGENCY STOP - the motors off (forced disarm)");
                }
                Cmd::Takeoff(h) => {
                    // armed first, the takeoff a moment later; ArduPilot takes off from GUIDED only
                    let delay = if ap == Autopilot::ArduPilot {
                        let (m, _) = FlightMode::Guided.params(ap);
                        send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, m, 0.0, 0.0, 0.0, 0.0, 0.0]));
                        std::thread::sleep(Duration::from_millis(300));
                        1000
                    } else {
                        500
                    };
                    if !link.veh.lock().unwrap().armed {
                        send(arm(&mut enc, true));
                    }
                    takeoff_at = Some((now + Duration::from_millis(delay), h));
                }
                Cmd::Land => send(enc.command_long(sys, comp, mav::MAV_CMD_NAV_LAND, [0.0, 0.0, 0.0, f32::NAN, f32::NAN, f32::NAN, 0.0])),
                Cmd::Return => send(enc.command_long(sys, comp, mav::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7])),
                Cmd::Mode(m) => {
                    if matches!(m, FlightMode::Position | FlightMode::Altitude | FlightMode::Stabilized) {
                        link.auto_mode.store(false, Ordering::Relaxed);
                    }
                    let (main, sub) = m.params(ap);
                    send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, main, sub, 0.0, 0.0, 0.0, 0.0]));
                }
                Cmd::SetParam(id, n) => {
                    // its own type, as it told it; PX4 not told yet: asked first (ArduPilot goes by
                    // its own table whatever is sent)
                    let told = link.veh.lock().unwrap().params.get(id).map(|p| p.1);
                    match (told, ap) {
                        (Some(ptype), _) => {
                            send(enc.param_set(sys, comp, id, mav::param_wire(ap, ptype, n), ptype));
                            log(&format!("pilot: {id} = {n} asked of the flight controller"));
                        }
                        (None, Autopilot::ArduPilot) => {
                            send(enc.param_set(sys, comp, id, mav::param_wire(ap, mav::PARAM_REAL32, n), mav::PARAM_REAL32));
                            log(&format!("pilot: {id} = {n} asked of the flight controller"));
                        }
                        (None, Autopilot::Px4) => {
                            send(enc.param_request_read(sys, comp, id));
                            pending_sets.retain(|p| p.0 != id);
                            pending_sets.push((id, n, now));
                        }
                    }
                }
                Cmd::ReadParams(ids) => {
                    for id in ids {
                        send(enc.param_request_read(sys, comp, id));
                    }
                }
                Cmd::Calibrate(kind) => {
                    // (all zeros: the one running cancelled)
                    let p = kind.map_or([0.0; 7], |k| k.params());
                    send(enc.command_long(sys, comp, mav::MAV_CMD_PREFLIGHT_CALIBRATION, p));
                    log(&format!("pilot: calibration {}", kind.map_or("cancelled".to_string(), |k| format!("{k:?} asked"))));
                }
            }
        }
        // the parameters waiting for PX4 to tell their type: set as it does, given up after 5 s
        if !pending_sets.is_empty() {
            let v = link.veh.lock().unwrap();
            pending_sets.retain(|&(id, n, at)| match v.params.get(id).map(|p| p.1) {
                Some(ptype) => {
                    send(enc.param_set(sys, comp, id, mav::param_wire(ap, ptype, n), ptype));
                    log(&format!("pilot: {id} = {n} asked of the flight controller"));
                    false
                }
                None if at.elapsed() >= Duration::from_secs(5) => {
                    log(&format!("pilot: {id} - the flight controller did not tell it, not set"));
                    false
                }
                None => true,
            });
        }
        if let Some((at, h)) = takeoff_at.filter(|(at, _)| now >= *at) {
            let _ = at;
            takeoff_at = None;
            let v = link.veh.lock().unwrap().clone();
            // (the height: PX4 above sea level, ArduPilot above home)
            let alt = if ap == Autopilot::ArduPilot { h } else { v.home_amsl.unwrap_or(v.amsl - v.alt) + h };
            send(enc.command_long(sys, comp, mav::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, f32::NAN, f32::NAN, f32::NAN, alt]));
            handover = Some((now, h, -v.z));
        }
    }
}

pub struct Pilot {
    rx: Arc<nyx_rx::Shared>,
    link: Arc<Link>,
    cmds: Sender<Cmd>,
    tex: Option<egui::TextureHandle>,
    tex_ver: u64,
    gilrs: Option<gilrs::Gilrs>,
    pad_name: Option<String>,
    /// an RC transmitter plugged in (rc.rs), when it was last looked for, and what it read last
    /// (roll, pitch, throttle, yaw: Settings > Control shows them)
    rc: Option<rc::Rc>,
    rc_seek: Instant,
    rc_seen: Option<Sticks>,
    /// the motors asked on with its throttle up (refused): when, for the status line
    rc_note: Option<Instant>,
    /// a gamepad's sticks smoothed, and when last read
    pad_s: Sticks,
    pad_t: Instant,
    /// DJI's stick commands: both sticks in the bottom corners (since when; done, not again until
    /// let go), and the throttle held down on the ground (since when; it came up once since the
    /// motors started)
    csc_since: Option<Instant>,
    csc_done: bool,
    /// the calibrations still to run after the one going (the IMU's: gyro, accel, level), the
    /// last "[cal]" line acted on, the one asked and when (not started yet: its first line not
    /// come), and how the last run ended
    cal_queue: Vec<CalKind>,
    cal_seq: u32,
    cal_asked: Option<(CalKind, Instant)>,
    cal_note: Option<(bool, String)>,
    thr_low_since: Option<Instant>,
    thr_up_since_arm: bool,
    /// the on-screen sticks: left (yaw, throttle), right (roll, pitch)
    left: Vec2,
    right: Vec2,
    /// the buttons being held (take off, land, return, the motors): since when
    holds: std::collections::HashMap<&'static str, Instant>,
    /// --demo's banner on the HUD (off: --demo-quiet)
    demo_tag: bool,
    /// the tether test asked for before the flight controller told its settings: when
    tether_note: Option<Instant>,
    /// inside the phone's app (--embedded), and its start screen asked for (About)
    embedded: bool,
    leave: bool,
    /// the touches holding the on-screen sticks: left, right (each finger the stick it went down on)
    fingers: [Option<u64>; 2],
    takeoff_h: f32,
    /// the speed mode's limits put on the flight controller: last tried, how many times (it stops
    /// trying a few times over until the link or the mode changes)
    speed_sync_at: Instant,
    speed_tries: u32,
    /// the link's ranging asked of the ground board (range_wanted): on now, asked last at; the
    /// satellites' position there or not, since when
    range_on: bool,
    range_asked_at: Instant,
    gps_seen: (bool, Instant),
    /// the knobs as drawn: the sticks that went up last frame
    shown: (Vec2, Vec2),
    map: map::MapView,
    /// the map large and the video small (M, or a tap on the small one)
    map_large: bool,
    /// where the aircraft was last seen on the globe, kept in a file for the next run's map
    place_file: PathBuf,
    place_kept: Option<(f64, f64)>,
    /// Settings (the button by the mode's panel): the ground board's console (the link's
    /// sections, as nyxhop's drawer has them) and the flight controller's safety parameters
    settings_open: bool,
    /// the ground board (its address, as given)
    board_addr: String,
    board: nyx_common::boardctl::BoardCtl,
    form: nu::RadioForm,
    chan_form: nu::ChannelForm,
    lic_buf: String,
    lic_status: String,
    /// a parameter's number being edited (by name), sent when the edit ends
    edits: std::collections::HashMap<&'static str, f32>,
    params_asked: Instant,
    tab: Tab,
    /// the app's own settings and where they are kept (beside the program)
    cfg: AppCfg,
    cfg_file: PathBuf,
    /// the aircraft's settings sent up (`!set <session>:<n> <key> <value>`): this run's session
    /// and the last number; what was asked shows until the aircraft tells it runs it (or 8 s)
    air_session: u32,
    air_n: u32,
    air_asked: std::collections::HashMap<&'static str, (String, Instant)>,
}

impl Pilot {
    /// A setting of the aircraft's, up the link (its app applies the ones it allows and tells back
    /// what it runs in its Info frames)
    fn air_set(&mut self, key: &'static str, val: String) {
        self.air_n += 1;
        nyx_rx::send_text(&self.rx, &format!("!set {}:{} {key} {val}", self.air_session, self.air_n));
        log(&format!("pilot: aircraft {key} = {val} asked"));
        self.air_asked.insert(key, (val, Instant::now()));
    }

    /// The aircraft's value of `key` as shown: what was just asked of it, while it has not told
    /// it yet, else what it told (`air_<key>` of its Info frame)
    fn air_value(&self, air: &std::collections::HashMap<String, String>, key: &str) -> Option<String> {
        let told = air.get(&format!("air_{key}")).cloned();
        match self.air_asked.get(key) {
            Some((v, at)) if at.elapsed() < Duration::from_secs(8) && told.as_deref() != Some(v.as_str()) => Some(v.clone()),
            _ => told,
        }
    }

    /// The aircraft's number `key` as a field, sent when the edit ends
    fn air_field(&mut self, ui: &mut egui::Ui, air: &std::collections::HashMap<String, String>, label: &str, key: &'static str, lo: f32, hi: f32, suffix: &str, zero: Option<&str>) {
        let shown = self.air_value(air, key).and_then(|v| v.parse::<f32>().ok());
        th::row(ui, label, |ui| {
            let Some(shown) = shown else {
                ui.label(egui::RichText::new("—").color(th::DIM));
                return;
            };
            let edit_key = key;
            let mut x = self.edits.get(edit_key).copied().unwrap_or(shown);
            let r = ui.add(egui::DragValue::new(&mut x).speed(((hi - lo) / 300.0).max(0.2)).range(lo..=hi).clamp_existing_to_range(false).suffix(suffix));
            if r.changed() && (r.dragged() || r.has_focus()) {
                self.edits.insert(edit_key, x);
            }
            if r.drag_stopped() || r.lost_focus() {
                if let Some(x) = self.edits.remove(edit_key) {
                    self.air_set(key, format!("{}", x.clamp(lo, hi).round() as i64));
                }
            }
            if let Some(z) = zero.filter(|_| x <= 0.0) {
                ui.label(egui::RichText::new(z).color(th::DIM));
            }
        });
    }

    /// `dir`: the program's folder (its settings, the map's tiles)
    pub fn new(board: &str, opts: &Opts, dir: &Path) -> Self {
        let url = opts.arg("--tiles", tiles::OSM_URL);
        // (the credit is the server's to ask for: OpenStreetMap's goes with its own tiles only)
        let credit = opts.arg("--tiles-credit", if url == tiles::OSM_URL { tiles::OSM_CREDIT } else { "" });
        let max_zoom = opts.arg("--tiles-max-zoom", "19").parse().unwrap_or(19);
        let cache = PathBuf::from(opts.arg("--map-cache", &dir.join("map-cache").to_string_lossy()));
        let map = map::MapView::new(tiles::Tiles::new(&url, &cache, credit, max_zoom));
        // where the aircraft was last seen (an earlier run): the map opens there until it has a fix
        let place_file = cache.join("last-place.txt");
        let place_kept = std::fs::read_to_string(&place_file).ok().and_then(|s| {
            let mut it = s.trim().split(',').map(|x| x.trim().parse::<f64>().ok());
            match (it.next().flatten(), it.next().flatten()) {
                (Some(lat), Some(lon)) if lat.abs() <= 85.0 && lon.abs() <= 180.0 => Some((lat, lon)),
                _ => None,
            }
        });
        let mut veh = Vehicle::default();
        veh.last_ll = place_kept.map(|(lat, lon)| ((lat * 1e7).round() as i32, (lon * 1e7).round() as i32));
        let demo_on = opts.flag("--demo");
        let demo_at = if opts.flag("--no-gps") {
            None
        } else {
            let s = opts.arg("--demo-at", "");
            let mut it = s.split(',').map(|x| x.trim().parse::<f64>().ok());
            match (it.next().flatten(), it.next().flatten()) {
                (Some(lat), Some(lon)) if lat.abs() <= 85.0 && lon.abs() <= 180.0 => Some((lat, lon)),
                _ => Some(DEMO_AT),
            }
        };
        let addr = format!("{}:{}", board, nyx_common::start::Role::Ground.port());
        log(&format!("pilot: ground board {addr}"));
        // --embedded (inside the phone's app): no console or web ports, and MAVLink's two ports
        // free ones of this run's (the app may be opened again in the same process, the old
        // threads' sockets still there); on a PC the fixed pair beside nyxhop's
        let embedded = opts.flag("--embedded");
        let free = || UdpSocket::bind("127.0.0.1:0").and_then(|s| s.local_addr()).map_or_else(|_| "0".to_string(), |a| a.to_string());
        let (up, down) = if embedded { (free(), free()) } else { (UP.to_string(), DOWN.to_string()) };
        let mut rx_opts = vec!["--channel".to_string(), addr, "--tlm-in".into(), up.clone(), "--tlm-out".into(), down.clone()];
        if embedded {
            rx_opts.extend(["--ctl", "0", "--web", "0"].map(String::from));
        }
        let rx = nyx_rx::setup(&Opts::from_list(rx_opts));
        let link = Arc::new(Link {
            veh: Mutex::new(veh),
            heard: AtomicBool::new(false),
            sticks: Mutex::new(Sticks::default()),
            thr: Mutex::new(None),
            rc_thr: AtomicBool::new(false),
            auto_mode: AtomicBool::new(true),
            demo: Mutex::new(None),
            demo_step: Mutex::new(String::new()),
            stop: AtomicBool::new(false),
        });
        let (cmds, cmd_rx) = channel();
        let l = link.clone();
        std::thread::Builder::new().name("mavlink".into()).spawn(move || link_thread(l, cmd_rx, up, down)).expect("spawn mavlink");
        if demo_on {
            let (l, c) = (link.clone(), cmds.clone());
            std::thread::Builder::new().name("demo".into()).spawn(move || demo(l, c, demo_at)).expect("spawn demo");
        }
        let gilrs = gilrs::Gilrs::new().map_err(|e| log(&format!("pilot: no gamepad support: {e}"))).ok();
        let console = format!("{board}:7202");
        let board_ctl = nyx_common::boardctl::BoardCtl::spawn(
            Arc::new(move || console.clone()),
            &["get", "capstat", "rssi", "trig", "softagc", "hop status", "license", "role", "range", "chbw"],
        );
        let cfg_file = dir.join("nyxhop-pilot.cfg");
        let cfg = AppCfg::load(&cfg_file);
        Pilot {
            rx,
            link,
            cmds,
            tex: None,
            tex_ver: u64::MAX,
            gilrs,
            pad_name: None,
            rc: None,
            rc_seek: Instant::now() - Duration::from_secs(10),
            rc_seen: None,
            rc_note: None,
            pad_s: Sticks::default(),
            pad_t: Instant::now(),
            csc_since: None,
            csc_done: false,
            cal_queue: Vec::new(),
            cal_seq: 0,
            cal_asked: None,
            cal_note: None,
            thr_low_since: None,
            thr_up_since_arm: false,
            left: Vec2::ZERO,
            right: Vec2::ZERO,
            holds: std::collections::HashMap::new(),
            demo_tag: !opts.flag("--demo-quiet"),
            tether_note: None,
            embedded: opts.flag("--embedded"),
            leave: false,
            fingers: [None, None],
            takeoff_h: cfg.takeoff_h,
            speed_sync_at: Instant::now() - Duration::from_secs(60),
            speed_tries: 0,
            range_on: false,
            range_asked_at: Instant::now(),
            gps_seen: (false, Instant::now()),
            shown: (Vec2::ZERO, Vec2::ZERO),
            map,
            map_large: false,
            place_file,
            place_kept,
            settings_open: opts.flag("--settings"),
            board_addr: board.to_string(),
            board: board_ctl,
            form: nu::RadioForm::default(),
            chan_form: nu::ChannelForm::default(),
            lic_buf: String::new(),
            lic_status: String::new(),
            edits: std::collections::HashMap::new(),
            params_asked: Instant::now() - Duration::from_secs(60),
            tab: match opts.arg("--tab", "").as_str() {
                "control" => Tab::Control,
                "sensors" => Tab::Sensors,
                "video" => Tab::Video,
                "link" => Tab::Link,
                "about" => Tab::About,
                _ => Tab::Safety,
            },
            cfg,
            cfg_file,
            // (a new run is a new session: the aircraft takes its numbers afresh)
            air_session: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(1, |d| (d.as_millis() % 1_000_000_000) as u32),
            air_n: 0,
            air_asked: std::collections::HashMap::new(),
        }
    }

    /// Everything behind Settings, in DJI's pages: the flight's safety (the flight controller's
    /// parameters, over MAVLink), the sticks, the aircraft's video, the link (the ground board's
    /// own sections, the ones nyxhop's drawer has) and what is what
    fn settings(&mut self, ui: &mut egui::Ui, v: &Vehicle) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(t("Settings", "Cài đặt")).monospace().size(18.0).strong().color(th::CYAN));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.add(egui::Button::new(t("Close", "Đóng")).min_size(vec2(90.0, 38.0))).clicked() {
                    self.settings_open = false;
                }
            });
        });
        let pages = [
            (Tab::Safety, t("Safety", "An toàn"), "safety"),
            (Tab::Control, t("Control", "Điều khiển"), "control"),
            (Tab::Sensors, t("Sensors", "Cảm biến"), "sensors"),
            (Tab::Video, t("Video", "Video"), "video"),
            (Tab::Link, t("Link", "Truyền dẫn"), "link"),
            (Tab::About, t("About", "Thông tin"), "about"),
        ];
        let opts: Vec<(&str, &str)> = pages.iter().map(|p| (p.1, p.2)).collect();
        let cur = pages.iter().find(|p| p.0 == self.tab).map_or("safety", |p| p.2);
        if let Some(hit) = nu::segmented(ui, &opts, cur) {
            if let Some(p) = pages.iter().find(|p| p.2 == hit) {
                self.tab = p.0;
            }
        }
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| match self.tab {
            Tab::Safety => self.safety_page(ui, v),
            Tab::Control => self.control_page(ui, v),
            Tab::Sensors => self.sensors_section(ui, v),
            Tab::Video => self.video_page(ui),
            Tab::Link => {
                let st = self.board.snapshot();
                nu::link_section(ui, &st, nu::Role::Ground, &self.board);
                self.aircraft_radio_section(ui);
                nu::channel_section(ui, &st, &mut self.chan_form, &self.board);
                nu::radio_section(ui, &st, nu::Role::Ground, &mut self.form, &self.board, &mut |hz| nyx_rx::set_samp_rate_hz(hz));
                let far = self.rx.far_licence();
                nu::licence_section(ui, &st, (!far.is_empty()).then_some(far.as_str()), &mut self.lic_buf, &mut self.lic_status, &self.board);
            }
            Tab::About => self.about_page(ui, v),
        });
    }

    /// A flight controller's number as a field, in metres: its value as it told it, editable once it
    /// has, sent when the edit ends (the answer comes back as its PARAM_VALUE). `k`: the parameter's
    /// units per metre shown (100: centimetres); `off`: 0 is allowed and means no limit.
    fn param_field(&mut self, ui: &mut egui::Ui, v: &Vehicle, label: &str, id: &'static str, k: f64, max: f32, off: bool) {
        let told = v.param(id).map(|n| (n / k) as f32);
        th::row(ui, label, |ui| {
            // (nothing to edit until it told its own: a made-up number in the box was one touch
            // from being sent - 12/11, 0 clamped to 5 m showed where PX4 had 30)
            let Some(told) = told else {
                ui.label(egui::RichText::new("—").color(th::DIM));
                return;
            };
            let mut x = self.edits.get(id).copied().unwrap_or(told);
            let lo = if off { 0.0 } else { 5.0 };
            let r = ui.add_enabled(
                v.link(),
                egui::DragValue::new(&mut x).speed((max / 240.0).max(0.2)).range(lo..=max).clamp_existing_to_range(false).suffix(" m"),
            );
            // (only the pilot's own drag or typing is an edit)
            if r.changed() && (r.dragged() || r.has_focus()) {
                self.edits.insert(id, x);
            }
            if r.drag_stopped() || r.lost_focus() {
                if let Some(x) = self.edits.remove(id) {
                    let _ = self.cmds.send(Cmd::SetParam(id, f64::from(x.clamp(lo, max).round()) * k));
                }
            }
            if off && x <= 0.0 {
                ui.label(egui::RichText::new(t("no limit", "không giới hạn")).color(th::DIM));
            }
        });
    }

    fn safety_page(&mut self, ui: &mut egui::Ui, v: &Vehicle) {
        let ap = v.autopilot;
        let heard = v.link();
        if !heard {
            ui.label(
                egui::RichText::new(t(
                    "The flight controller is not heard: its settings show once it is.",
                    "Chưa nghe thấy bộ điều khiển bay: các cài đặt của nó hiện khi nghe được.",
                ))
                .small()
                .color(th::WARN),
            );
        }
        th::section(ui, t("Signal lost", "Mất tín hiệu"), true, |ui| {
            let cur = match ap {
                Autopilot::Px4 => v.param("NAV_RCL_ACT"),
                Autopilot::ArduPilot => v.param("FS_GCS_ENABLE"),
            }
            .and_then(|n| LostAction::of(ap, n));
            let mut opts = vec![
                (t("Return home", "Quay về"), LostAction::Return.key()),
                (t("Land", "Hạ cánh"), LostAction::Land.key()),
                (t("Hover", "Đứng yên"), LostAction::Hover.key()),
            ];
            if cur == Some(LostAction::Disarm) {
                opts.push((t("Stop the motors", "Tắt motor"), LostAction::Disarm.key()));
            }
            if let Some(hit) = nu::segmented(ui, &opts, cur.map_or("", |a| a.key())) {
                if let Some(a) = LostAction::ALL.into_iter().find(|a| a.key() == hit).filter(|_| heard) {
                    match ap {
                        Autopilot::Px4 => {
                            let _ = self.cmds.send(Cmd::SetParam("NAV_RCL_ACT", a.px4()));
                            let _ = self.cmds.send(Cmd::SetParam("NAV_DLL_ACT", a.px4()));
                        }
                        Autopilot::ArduPilot => {
                            if let Some(n) = a.ardu() {
                                let _ = self.cmds.send(Cmd::SetParam("FS_GCS_ENABLE", n));
                            }
                        }
                    }
                }
            }
            let note = match cur {
                Some(LostAction::Disarm) => t(
                    "The motors stop when the signal is lost: for tests on the bench without propellers only. Pick Return or Land before flying.",
                    "Mất tín hiệu là tắt motor: chỉ dùng khi thử trên bàn, không lắp cánh. Chọn Quay về hoặc Hạ cánh trước khi bay.",
                ),
                Some(LostAction::Return) => t(
                    "It climbs to the return height, flies back to where it took off and lands. With no satellite fix (indoors) it lands where it is.",
                    "Máy bay lên độ cao quay về, bay về chỗ cất cánh rồi hạ. Không bắt được vệ tinh (trong nhà) thì hạ cánh tại chỗ.",
                ),
                Some(LostAction::Hover) => t("It stops and waits where it is until the signal comes back.", "Máy bay dừng lại chờ tại chỗ cho tới khi có tín hiệu lại."),
                Some(LostAction::Land) => t("It lands where it is.", "Máy bay hạ cánh tại chỗ."),
                None => "",
            };
            if !note.is_empty() {
                ui.label(egui::RichText::new(note).small().color(if cur == Some(LostAction::Disarm) { th::WARN } else { th::DIM }));
            }
        });
        th::section(ui, t("Return home", "Quay về"), true, |ui| {
            let (id, k) = match ap {
                Autopilot::Px4 => ("RTL_RETURN_ALT", 1.0),
                Autopilot::ArduPilot if v.param("RTL_ALT_M").is_none() && v.param("RTL_ALT").is_some() => ("RTL_ALT", 100.0),
                Autopilot::ArduPilot => ("RTL_ALT_M", 1.0),
            };
            self.param_field(ui, v, t("Return height", "Độ cao quay về"), id, k, 120.0, false);
            ui.label(
                egui::RichText::new(t(
                    "It climbs to this height before flying back: above the highest obstacle on the way.",
                    "Máy bay lên tới độ cao này rồi mới bay về: cao hơn vật cản cao nhất trên đường.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Battery", "Pin"), true, |ui| {
            let id = if ap == Autopilot::ArduPilot { "BATT_FS_LOW_ACT" } else { "COM_LOW_BAT_ACT" };
            let cur = v.param(id).and_then(|n| LOW_BAT.iter().position(|b| if ap == Autopilot::ArduPilot { b.3 == n } else { b.2 == n }));
            th::row(ui, t("Low battery", "Pin yếu"), |ui| {
                let opts: Vec<(&str, &str)> = LOW_BAT.iter().map(|b| (t(b.0, b.1), b.0)).collect();
                if let Some(hit) = nu::segmented(ui, &opts, cur.map_or("", |i| LOW_BAT[i].0)) {
                    if let Some(b) = LOW_BAT.iter().find(|b| b.0 == hit).filter(|_| heard && cur.is_some()) {
                        let _ = self.cmds.send(Cmd::SetParam(id, if ap == Autopilot::ArduPilot { b.3 } else { b.2 }));
                    }
                }
            });
            let batt = if v.batt_v > 0.0 { format!("{:.1} V  ·  {}%", v.batt_v, v.batt_pct.max(0)) } else { "—".into() };
            th::kv(ui, t("Now", "Hiện tại"), &batt);
        });
        if ap == Autopilot::Px4 {
            th::section(ui, t("Flight limits", "Giới hạn bay"), true, |ui| {
                self.param_field(ui, v, t("Max height", "Độ cao tối đa"), "GF_MAX_VER_DIST", 1.0, 500.0, true);
                self.param_field(ui, v, t("Max distance", "Khoảng cách tối đa"), "GF_MAX_HOR_DIST", 1.0, 5000.0, true);
                ui.label(
                    egui::RichText::new(t(
                        "Counted from home. At the limit the aircraft stops and holds there; 0 = no limit.",
                        "Tính từ điểm cất cánh. Tới giới hạn máy bay dừng lại giữ chỗ; 0 = không giới hạn.",
                    ))
                    .small()
                    .color(th::DIM),
                );
            });
        }
        th::section(ui, t("Take-off", "Cất cánh"), true, |ui| {
            th::row(ui, t("Take-off height", "Độ cao cất cánh"), |ui| {
                let r = ui.add(egui::DragValue::new(&mut self.takeoff_h).speed(0.1).range(TAKEOFF_RANGE).fixed_decimals(1).suffix(" m"));
                if r.drag_stopped() || r.lost_focus() {
                    self.cfg.takeoff_h = self.takeoff_h;
                    self.cfg.save(&self.cfg_file);
                }
            });
            ui.label(
                egui::RichText::new(t(
                    "Take off (held a second) starts the motors, climbs to this height and hovers; then the sticks fly it.",
                    "Cất cánh (giữ một giây) tự bật motor, lên tới độ cao này rồi đứng yên; sau đó bay bằng cần.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Distance without GPS", "Khoảng cách khi mất GPS"), true, |ui| {
            th::row(ui, t("Measure over the link", "Đo qua sóng"), |ui| {
                let opts = [(t("On", "Bật"), "1"), (t("Off", "Tắt"), "0")];
                if let Some(hit) = nu::segmented(ui, &opts, if self.cfg.range_nogps { "1" } else { "0" }) {
                    self.cfg.range_nogps = hit == "1";
                    self.cfg.save(&self.cfg_file);
                }
            });
            let now = if !self.cfg.range_nogps {
                t("off", "tắt").to_string()
            } else if !self.range_on {
                t("waiting: the satellites give the distance", "đang chờ: có GPS nên dùng GPS").to_string()
            } else {
                match nu::range_of(&self.board.snapshot()) {
                    Some((m, s)) => format!("{m:.0} m  ±{s:.0}"),
                    None => t("measuring…", "đang đo…").to_string(),
                }
            };
            th::kv(ui, t("Now", "Hiện tại"), &now);
            ui.label(
                egui::RichText::new(t(
                    "An extra: with no satellite fix (or lost), the link measures how far the aircraft is and D shows it as ≈. Off by default.",
                    "Tính năng phụ: không có hoặc mất GPS thì link tự đo máy bay cách bao xa, D hiện dấu ≈. Mặc định tắt.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Emergency stop", "Dừng khẩn cấp"), true, |ui| {
            th::row(ui, t("On the screen", "Nút trên màn hình"), |ui| {
                let opts = [(t("Show", "Hiện"), "1"), (t("Hide", "Ẩn"), "0")];
                if let Some(hit) = nu::segmented(ui, &opts, if self.cfg.estop { "1" } else { "0" }) {
                    self.cfg.estop = hit == "1";
                    self.cfg.save(&self.cfg_file);
                }
            });
            ui.label(
                egui::RichText::new(t(
                    "One touch (or Space) stops the motors at once, in the air too: the aircraft falls. For an aircraft out of control, on a tether above all. Hidden, Space does nothing.",
                    "Chạm một lần (hoặc phím Space) là motor tắt ngay, kể cả đang bay: máy bay rơi. Dùng khi máy bay mất kiểm soát, nhất là lúc thử có dây. Ẩn nút thì phím Space cũng không có tác dụng.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Tether test", "Thử có dây"), true, |ui| {
            if ap != Autopilot::Px4 {
                ui.label(egui::RichText::new(t("With PX4 on the NyxHop board only.", "Chỉ có với PX4 trên mạch NyxHop.")).small().color(th::DIM));
                return;
            }
            let mut want = None;
            th::row(ui, t("Tether test", "Thử có dây"), |ui| {
                let opts = [(t("On", "Bật"), "1"), (t("Off", "Tắt"), "0")];
                if let Some(hit) = nu::segmented(ui, &opts, if self.cfg.tether { "1" } else { "0" }) {
                    want = Some(hit == "1");
                }
            });
            if let Some(on) = want.filter(|_| heard) {
                if !self.set_tether(on, v) {
                    self.tether_note = Some(Instant::now());
                }
            }
            if self.tether_note.is_some_and(|t| t.elapsed() < Duration::from_secs(5)) {
                ui.label(egui::RichText::new(t("Reading the flight controller's settings first: try again in a moment.", "Đang đọc cài đặt của bộ điều khiển bay: thử lại sau giây lát.")).small().color(th::WARN));
            }
            let mut changed = false;
            let mut num = |ui: &mut egui::Ui, label: &str, x: &mut f32, lo: f32, hi: f32, suffix: &str, step: f64| {
                th::row(ui, label, |ui| {
                    let r = ui.add(egui::DragValue::new(x).speed(step).range(lo..=hi).suffix(suffix));
                    changed |= r.drag_stopped() || r.lost_focus();
                });
            };
            num(ui, t("Tilt over", "Nghiêng quá"), &mut self.cfg.tether_tilt, 10.0, 90.0, " °", 1.0);
            num(ui, t("Yaw faster than", "Xoay nhanh quá"), &mut self.cfg.tether_yawr, 30.0, 1000.0, " °/s", 5.0);
            num(ui, t("Height over", "Cao quá"), &mut self.cfg.tether_hgt, 0.0, 20.0, " m", 0.1);
            num(ui, t("Most throttle", "Ga tối đa"), &mut self.cfg.tether_thr, 20.0, 100.0, " %", 1.0);
            if changed {
                self.cfg.save(&self.cfg_file);
                self.speed_tries = 0;
                self.speed_sync_at = Instant::now() - Duration::from_secs(60);
            }
            let guard_known = v.param("NYX_GRD_TILT").is_some();
            let state = if !heard {
                t("not heard", "chưa nghe thấy").to_string()
            } else if !guard_known {
                t("the flight controller has no guard yet (it needs the NyxHop PX4 update)", "bộ điều khiển bay chưa có bộ ngắt (cần bản PX4 NyxHop mới)").to_string()
            } else if !self.cfg.tether {
                t("off", "tắt").to_string()
            } else if tether_params(&self.cfg).iter().all(|(id, n)| v.param(id).is_some_and(|x| (x - n).abs() < 0.01)) {
                t("on: the aircraft guards these", "đang bật: máy bay canh các ngưỡng này").to_string()
            } else {
                t("being set…", "đang cài…").to_string()
            };
            th::kv(ui, t("Aircraft", "Máy bay"), &state);
            ui.label(
                egui::RichText::new(t(
                    "On: the aircraft stops its motors by itself (and disarms) when it tilts or yaws past these for 0.3 s, or goes higher (a downward distance sensor needed; 0 = no height limit). The throttle stays under its most, and a lost signal stops the motors. Off: all as it was.",
                    "Bật: máy bay tự tắt motor (và disarm) khi nghiêng hoặc xoay quá ngưỡng 0,3 giây, hoặc lên cao quá (cần cảm biến khoảng cách hướng xuống; 0 = không giới hạn độ cao). Ga không vượt mức tối đa, mất tín hiệu là tắt motor. Tắt: mọi thứ trở lại như cũ.",
                ))
                .small()
                .color(th::DIM),
            );
        });
    }

    fn control_page(&mut self, ui: &mut egui::Ui, v: &Vehicle) {
        th::section(ui, t("Flight mode", "Chế độ bay"), true, |ui| {
            let opts: Vec<(&str, &str)> = Speed::ALL.iter().map(|s| (s.name(), s.key())).collect();
            if let Some(hit) = nu::segmented(ui, &opts, self.cfg.speed.key()) {
                if let Some(s) = Speed::of_key(&hit) {
                    self.pick_speed(s, v);
                }
            }
            let [vh, acc, up, dn, yaw, tilt] = self.cfg.speed.limits();
            th::kv(ui, t("Speed", "Tốc độ"), &format!("{vh:.0} m/s  ·  {} {acc:.0} m/s²", t("accel.", "tăng tốc")));
            th::kv(ui, t("Climb / descend", "Lên / xuống"), &format!("{up:.1} / {dn:.1} m/s"));
            th::kv(ui, t("Turn · tilt", "Xoay · nghiêng"), &format!("{yaw:.0} °/s  ·  {tilt:.0} °"));
            let told = Speed::told(v);
            th::kv(
                ui,
                t("Aircraft", "Máy bay"),
                match told {
                    _ if !v.link() => t("not heard", "chưa nghe thấy"),
                    Some(s) if s == self.cfg.speed => t("flies with it", "đang bay theo chế độ này"),
                    _ => t("being set…", "đang cài…"),
                },
            );
            ui.label(
                egui::RichText::new(t(
                    "All three hold position on satellites (ATTI shows when it has none and holds the height alone). Cine: slow and smooth, for filming; Normal; Sport: fast. Keys 1, 2, 3.",
                    "Cả ba đều giữ vị trí theo vệ tinh (mất vệ tinh thì hiện ATTI, chỉ giữ độ cao). Cine: chậm, mượt để quay phim; Normal: thường; Sport: nhanh. Phím 1, 2, 3.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Advanced modes", "Chế độ nâng cao"), false, |ui| {
            let cur = v.flight_mode();
            let auto = self.link.auto_mode.load(Ordering::Relaxed);
            ui.horizontal_wrapped(|ui| {
                // DJI's way: the app picks what the aircraft can fly now (satellites or the
                // optical flow: Position; a height alone: Altitude; nothing: Stabilized)
                if ui.add(egui::Button::new(t("Auto (as DJI)", "Tự chọn (như DJI)")).selected(auto).min_size(vec2(130.0, 36.0))).clicked() {
                    self.link.auto_mode.store(true, Ordering::Relaxed);
                }
                for (m, label) in [
                    (FlightMode::Position, t("Position", "Giữ vị trí")),
                    (FlightMode::Altitude, t("Altitude (ATTI)", "Giữ độ cao (ATTI)")),
                    (FlightMode::Stabilized, t("Stabilized", "Cân bằng (tay ga)")),
                ] {
                    let on = !auto && cur == Some(m);
                    if ui.add_enabled(v.link(), egui::Button::new(label).selected(on).min_size(vec2(130.0, 36.0))).clicked() {
                        let _ = self.cmds.send(Cmd::Mode(m));
                    }
                }
            });
            ui.label(
                egui::RichText::new(t(
                    "Altitude: holds the height, drifts with the wind. Stabilized: the throttle is yours (it stays where the stick leaves it). For pilots who know them.",
                    "Giữ độ cao: chỉ giữ độ cao, trôi theo gió. Cân bằng: tự cầm ga (ga đứng ở chỗ cần để lại). Dành cho người đã quen.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Stick mode", "Kiểu cần"), true, |ui| {
            let opts = [("Mode 1", "1"), ("Mode 2", "2"), ("Mode 3", "3")];
            let cur = self.cfg.stick_mode.to_string();
            if let Some(hit) = nu::segmented(ui, &opts, &cur) {
                if let Ok(m) = hit.parse::<u8>() {
                    self.cfg.stick_mode = m;
                    self.cfg.save(&self.cfg_file);
                }
            }
            let (l, r) = stick_names(self.cfg.stick_mode);
            th::kv(ui, t("Left stick", "Cần trái"), l);
            th::kv(ui, t("Right stick", "Cần phải"), r);
            ui.label(
                egui::RichText::new(t(
                    "The on-screen sticks and a gamepad follow it; the keyboard stays: arrows roll and pitch, W/S throttle, A/D yaw.",
                    "Cần trên màn hình và tay cầm đổi theo; bàn phím giữ nguyên: mũi tên nghiêng và tới lui, W/S ga, A/D xoay.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Stick feel", "Độ nhạy cần"), true, |ui| {
            th::row(ui, t("Softer middle", "Êm ở giữa"), |ui| {
                let mut pct = self.cfg.expo * 100.0;
                let r = ui.add(egui::Slider::new(&mut pct, 0.0..=60.0).suffix(" %").fixed_decimals(0));
                if r.changed() {
                    self.cfg.expo = pct / 100.0;
                }
                if r.drag_stopped() || r.lost_focus() {
                    self.cfg.save(&self.cfg_file);
                }
            });
            ui.label(
                egui::RichText::new(t(
                    "Higher: small stick moves turn and tilt it less, for fine control; full stick is still full.",
                    "Cao hơn: đẩy cần ít thì máy bay nghiêng, xoay ít hơn, dễ chỉnh nhẹ; đẩy hết cần vẫn là hết cỡ.",
                ))
                .small()
                .color(th::DIM),
            );
        });
        th::section(ui, t("Controller", "Tay cầm"), true, |ui| {
            if let Some(s) = self.rc_seen {
                // an RC transmitter: its sticks as they come, its channel order
                th::kv(ui, t("Connected", "Đã nối"), self.pad_name.as_deref().unwrap_or("RC"));
                th::row(ui, t("Channel order", "Thứ tự kênh"), |ui| {
                    let opts: Vec<(&str, &str)> = rc::Order::ALL.iter().map(|o| (o.key(), o.key())).collect();
                    if let Some(hit) = nu::segmented(ui, &opts, self.cfg.rc_order.key()) {
                        if let Some(o) = rc::Order::from_key(&hit) {
                            self.cfg.rc_order = o;
                            self.cfg.save(&self.cfg_file);
                        }
                    }
                });
                th::row(ui, t("Throttle stick springs back", "Cần ga tự về giữa"), |ui| {
                    let opts = [(t("No", "Không"), "0"), (t("Yes", "Có"), "1")];
                    if let Some(hit) = nu::segmented(ui, &opts, if self.cfg.rc_center { "1" } else { "0" }) {
                        self.cfg.rc_center = hit == "1";
                        self.cfg.save(&self.cfg_file);
                    }
                });
                let bar = |ui: &mut egui::Ui, name: &str, x: f32, from_bottom: bool| {
                    th::row(ui, name, |ui| {
                        let (r, _) = ui.allocate_exact_size(vec2(180.0, 14.0), egui::Sense::hover());
                        let p = ui.painter();
                        p.rect_filled(r, 3.0, Color32::from_rgb(40, 52, 66));
                        let x = x.clamp(-1.0, 1.0);
                        let fill = if from_bottom {
                            Rect::from_min_max(r.min, pos2(r.left() + r.width() * (x + 1.0) / 2.0, r.bottom()))
                        } else {
                            let c = r.center().x;
                            let e = c + r.width() / 2.0 * x;
                            Rect::from_min_max(pos2(c.min(e), r.top()), pos2(c.max(e), r.bottom()))
                        };
                        p.rect_filled(fill, 3.0, ACCENT);
                        ui.label(if from_bottom { format!("{:.0} %", (x + 1.0) * 50.0) } else { format!("{:+.0} %", x * 100.0) });
                    });
                };
                bar(ui, t("Throttle", "Ga"), s.throttle, true);
                bar(ui, t("Yaw (right +)", "Xoay (phải +)"), s.yaw, false);
                bar(ui, t("Roll (right +)", "Nghiêng (phải +)"), s.roll, false);
                bar(ui, t("Pitch (forward +)", "Tới lui (tới +)"), s.pitch, false);
                ui.label(
                    egui::RichText::new(t(
                        "Each stick should move its own bar, the right way. A wrong one: the other channel order, or reverse that channel on the transmitter. In the height-holding modes the middle holds the height. In Stabilized: a throttle stick that stays put is the throttle itself (the motors start only with it at the bottom); one that springs back moves the throttle up and down and the middle keeps it, as DJI's.",
                        "Mỗi cần phải chạy đúng thanh của nó và đúng chiều. Sai thì chọn thứ tự kênh kia, hoặc đảo chiều kênh đó trên tay. Ở các chế độ giữ độ cao, ga ở giữa là giữ độ cao. Ở chế độ Cân bằng: cần ga đứng yên thì cần ở đâu là ga bấy nhiêu (chỉ bật được motor khi ga ở đáy); cần ga tự về giữa thì đẩy lên là tăng ga, kéo xuống là giảm, thả về giữa là giữ nguyên ga, như DJI.",
                    ))
                    .small()
                    .color(th::DIM),
                );
                return;
            }
            match &self.pad_name {
                Some(n) => th::kv(ui, t("Connected", "Đã nối"), n),
                None => {
                    ui.label(
                        egui::RichText::new(t(
                            "No gamepad: plug in a gamepad, or an RC transmitter as a USB joystick, and it flies by it.",
                            "Chưa có tay cầm: cắm tay cầm chơi game, hoặc tay điều khiển RC ở chế độ joystick USB, là bay bằng nó.",
                        ))
                        .small()
                        .color(th::DIM),
                    );
                }
            }
        });
        th::section(ui, t("Keys", "Phím tắt"), false, |ui| {
            for (k, en, vi) in [
                ("CSC", "both sticks to the bottom inner (or outer) corners: the motors on; on the ground again for 2 s, or the throttle held down 1 s after landing: off",
                    "gạt hai cần xuống góc dưới phía trong (hoặc phía ngoài): bật motor; dưới đất làm lại giữ 2 giây, hoặc hạ cánh xong giữ cần ga xuống 1 giây: tắt"),
                ("T", "take off (held a second)", "cất cánh (giữ một giây)"),
                ("L", "land (held a second)", "hạ cánh (giữ một giây)"),
                ("R", "return home (held a second)", "về nhà (giữ một giây)"),
                ("H", "stop and hover", "dừng, đứng yên"),
                ("Space", "emergency stop (when shown)", "dừng khẩn cấp (khi nút đang hiện)"),
                ("1 2 3", "Cine, Normal, Sport", "Cine, Normal, Sport"),
                ("4 5", "Altitude (ATTI), Stabilized", "giữ độ cao (ATTI), cân bằng"),
                ("M", "map large or small", "phóng to hoặc thu nhỏ bản đồ"),
                ("+ −", "zoom the map", "zoom bản đồ"),
                ("Esc", "close Settings", "đóng Cài đặt"),
            ] {
                th::kv(ui, k, t(en, vi));
            }
        });
    }

    fn video_page(&mut self, ui: &mut egui::Ui) {
        th::section(ui, t("Picture here", "Hình nhận được"), true, |ui| {
            let lv = self.rx.link_view();
            let res = self.tex.as_ref().map_or("—".to_string(), |t| format!("{} × {}", t.size()[0], t.size()[1]));
            th::kv(ui, t("Resolution", "Độ phân giải"), &res);
            th::kv(ui, t("Frames a second", "Khung hình/giây"), &format!("{:.0}", lv.fps));
            th::kv(ui, t("Bit rate", "Tốc độ bit"), &format!("{:.0} kbit/s", lv.kbps));
        });
        let air = nyx_common::licpanel::parse_kv(&self.rx.far_licence());
        th::section(ui, t("Aircraft camera", "Camera máy bay"), true, |ui| {
            if !air.contains_key("air_source") {
                ui.label(
                    egui::RichText::new(t(
                        "The aircraft has not told its video settings yet: it does every few seconds once the link carries its video (and its camera program is up to date).",
                        "Máy bay chưa báo cài đặt video: nó báo vài giây một lần khi link đã có hình (và chương trình camera trên máy bay là bản mới).",
                    ))
                    .small()
                    .color(th::WARN),
                );
                return;
            }
            // the source: the ones the aircraft can run (`air_sources`; not told: the camera alone).
            // 12/11: the board's camera program offered "Test picture" went into standby, off the
            // link, and nothing sent from here could reach it to put the camera back
            let src = self.air_value(&air, "source").unwrap_or_default();
            let name = |k: &str| match k {
                "rtsp" => t("Camera", "Camera"),
                "pattern" => t("Test picture", "Hình mẫu"),
                "webcam" => t("Webcam", "Webcam"),
                _ => "—",
            };
            let can: Vec<String> = air.get("air_sources").map_or(vec!["rtsp".into()], |s| s.split(',').map(str::to_string).collect());
            th::row(ui, t("Source", "Nguồn hình"), |ui| {
                if can.len() <= 1 {
                    ui.label(name(&src));
                    return;
                }
                let opts: Vec<(&str, &str)> = can.iter().map(|k| (name(k), k.as_str())).collect();
                if let Some(v) = nu::segmented(ui, &opts, &src) {
                    if v != src {
                        self.air_set("source", v);
                    }
                }
            });
            let paused = self.air_value(&air, "pause").as_deref() == Some("1");
            th::row(ui, t("Video", "Hình"), |ui| {
                let opts = [(t("On", "Bật"), "0"), (t("Off", "Tắt"), "1")];
                if let Some(v) = nu::segmented(ui, &opts, if paused { "1" } else { "0" }) {
                    if (v == "1") != paused {
                        self.air_set("pause", v);
                    }
                }
            });
            // a camera passed through as it sends (an IP camera): its own size and rate, and the
            // bitrate it is taken to; else the aircraft encodes the picture itself
            let pass = self.air_value(&air, "pass").as_deref() == Some("1") && src == "rtsp";
            if pass {
                let cam = |k: &str| air.get(k).cloned().unwrap_or_else(|| "—".into());
                th::kv(ui, t("Camera picture", "Hình của camera"), &format!("{}  ·  {} fps  ·  {} kbit/s", cam("air_cam_res").replace('x', " × "), cam("air_cam_fps"), cam("air_cam_kbps")));
                self.air_field(ui, &air, t("Max bit rate", "Bitrate tối đa"), "cammax", 0.0, 20000.0, " kbit/s", Some(t("the camera's own", "theo camera")));
                let adapt = self.air_value(&air, "camadapt").as_deref() == Some("1");
                th::row(ui, t("Follow the link", "Tự theo link"), |ui| {
                    let opts = [(t("On", "Bật"), "1"), (t("Off", "Tắt"), "0")];
                    if let Some(v) = nu::segmented(ui, &opts, if adapt { "1" } else { "0" }) {
                        if (v == "1") != adapt {
                            self.air_set("camadapt", v);
                        }
                    }
                });
                ui.label(
                    egui::RichText::new(t(
                        "Follow the link: the camera's bitrate goes down when the link carries less, up again when it carries more. Its size and frame rate are the camera's own.",
                        "Tự theo link: bitrate của camera tự giảm khi link chở được ít, tăng lại khi chở được nhiều. Độ phân giải và khung hình do camera đặt.",
                    ))
                    .small()
                    .color(th::DIM),
                );
            } else {
                let res = self.air_value(&air, "res").unwrap_or_default();
                th::row(ui, t("Resolution", "Độ phân giải"), |ui| {
                    let opts = [("640×360", "640x360"), ("960×540", "960x540"), ("1280×720", "1280x720"), ("1920×1080", "1920x1080")];
                    if let Some(v) = nu::segmented(ui, &opts, &res) {
                        if v != res {
                            self.air_set("res", v);
                        }
                    }
                });
                let fps = self.air_value(&air, "fps").unwrap_or_default();
                th::row(ui, t("Frame rate", "Khung hình"), |ui| {
                    let opts = [("15", "15"), ("20", "20"), ("25", "25"), ("30", "30")];
                    if let Some(v) = nu::segmented(ui, &opts, &fps) {
                        if v != fps {
                            self.air_set("fps", v);
                        }
                    }
                });
                self.air_field(ui, &air, t("Max bit rate", "Bitrate tối đa"), "maxkbps", 0.0, 20000.0, " kbit/s", Some(t("as the link allows", "theo link")));
                let auto = self.air_value(&air, "autores").as_deref() == Some("1");
                th::row(ui, t("Smaller when weak", "Tự hạ khi yếu"), |ui| {
                    let opts = [(t("On", "Bật"), "1"), (t("Off", "Tắt"), "0")];
                    if let Some(v) = nu::segmented(ui, &opts, if auto { "1" } else { "0" }) {
                        if (v == "1") != auto {
                            self.air_set("autores", v);
                        }
                    }
                });
            }
        });
    }

    /// The aircraft's own radio (its transmit power), as it told it
    fn aircraft_radio_section(&mut self, ui: &mut egui::Ui) {
        let air = nyx_common::licpanel::parse_kv(&self.rx.far_licence());
        th::section(ui, t("Aircraft radio", "Radio máy bay"), true, |ui| {
            if !air.contains_key("air_tpc_max_db") {
                ui.label(
                    egui::RichText::new(t(
                        "The aircraft has not told its transmit power yet (its camera program tells it, once up to date).",
                        "Máy bay chưa báo công suất phát (chương trình camera trên máy bay báo khi là bản mới).",
                    ))
                    .small()
                    .color(th::WARN),
                );
                return;
            }
            self.air_field(ui, &air, t("Max transmit power", "Công suất phát tối đa"), "txpower", -40.0, 0.0, " dB", None);
            let auto = self.air_value(&air, "tpc").as_deref() == Some("1");
            th::row(ui, t("Lower when near", "Tự giảm khi gần"), |ui| {
                let opts = [(t("On", "Bật"), "1"), (t("Off", "Tắt"), "0")];
                if let Some(v) = nu::segmented(ui, &opts, if auto { "1" } else { "0" }) {
                    if (v == "1") != auto {
                        self.air_set("tpc", v);
                    }
                }
            });
            th::kv(ui, t("Now", "Hiện tại"), &air.get("air_tpc_cur_db").map_or("—".into(), |v| format!("{v} dB")));
            ui.label(
                egui::RichText::new(t(
                    "0 dB is the most it has. Lower when near: it turns its power down while the ground hears it well, and back up when it does not.",
                    "0 dB là mức cao nhất. Tự giảm khi gần: máy bay hạ công suất khi mặt đất nghe rõ, tăng lại khi nghe kém.",
                ))
                .small()
                .color(th::DIM),
            );
        });
    }

    fn about_page(&mut self, ui: &mut egui::Ui, v: &Vehicle) {
        th::section(ui, t("Flight controller", "Bộ điều khiển bay"), true, |ui| {
            if v.link() {
                th::kv(ui, t("Type", "Loại"), if v.autopilot == Autopilot::ArduPilot { "ArduPilot" } else { "PX4" });
                th::kv(ui, t("Mode", "Chế độ"), v.mode_name());
                th::kv(ui, t("Armed", "Đã arm"), if v.armed { t("yes", "có") } else { t("no", "chưa") });
                th::kv(ui, t("Processor load", "Tải bộ xử lý"), &format!("{:.0} %", v.load));
                let gps = match v.gps_fix() {
                    Some((f, s)) if f >= 3 => tf_sats(t("3D fix", "fix 3D"), s),
                    Some((f, s)) if f == 2 => tf_sats(t("2D fix", "fix 2D"), s),
                    Some((_, s)) => tf_sats(t("no fix", "chưa fix"), s),
                    None => t("no receiver heard", "không nghe thấy máy thu").to_string(),
                };
                th::kv(ui, "GPS", &gps);
            } else {
                ui.label(egui::RichText::new(t("not heard", "chưa nghe thấy")).color(th::WARN));
            }
        });
        th::section(ui, t("This app", "Ứng dụng"), true, |ui| {
            th::kv(ui, t("Version", "Phiên bản"), env!("CARGO_PKG_VERSION"));
            th::kv(ui, t("Ground station", "Trạm mặt đất"), &self.board_addr);
            if self.embedded && ui.add(egui::Button::new(t("Start screen", "Màn hình bắt đầu")).min_size(vec2(180.0, 40.0))).clicked() {
                self.leave = true;
            }
        });
        th::section(ui, t("Technical details", "Chi tiết kỹ thuật"), false, |ui| {
            let lv = self.rx.link_view();
            th::kv(ui, "SNR", &format!("{:.1} dB", lv.snr_db));
            th::kv(ui, "MCS", &lv.mcs);
            th::kv(ui, t("Blocks lost", "Khối mất"), &format!("{:.1} %", lv.bler * 100.0));
            th::kv(ui, t("Round trip", "Khứ hồi"), &v.rtt_ms.map_or("—".into(), |r| format!("{r:.0} ms")));
            th::kv(ui, t("Flight controller messages", "Tin từ bộ điều khiển bay"), &format!("{:.0} /s", v.rate));
        });
    }
    /// A big button of the left column, as DJI's take-off and return: held `HOLD_S` to act (its
    /// fill grows meanwhile), by the pointer or by its key; `hold` false: a touch acts (the stop).
    /// True when it acts.
    #[allow(clippy::too_many_arguments)]
    fn big_button(&mut self, ui: &egui::Ui, r: Rect, id: &'static str, label: &str, fill: Color32, hold: bool, key_down: bool, key_pressed: bool) -> bool {
        let resp = ui.interact(r, ui.id().with(id), egui::Sense::click_and_drag()).on_hover_cursor(egui::CursorIcon::PointingHand);
        let p = ui.painter();
        let down = resp.is_pointer_button_down_on() || key_down;
        let lit = if resp.hovered() || down { fill.gamma_multiply(1.25) } else { fill };
        p.rect_filled(r, 10.0, lit);
        p.rect_stroke(r, 10.0, Stroke::new(1.0, Color32::from_white_alpha(60)), egui::StrokeKind::Inside);
        let fired = if hold {
            let (k, fired) = held(&mut self.holds, id, down, Instant::now());
            if k > 0.0 {
                let bar = Rect::from_min_size(r.min, vec2(r.width() * k, r.height()));
                p.rect_filled(bar, 10.0, Color32::from_rgba_unmultiplied(255, 200, 60, 90));
            }
            fired
        } else {
            resp.clicked() || key_pressed
        };
        p.text(r.center() - vec2(0.0, if hold { 6.0 } else { 0.0 }), Align2::CENTER_CENTER, label, FontId::proportional(16.0), WHITE);
        if hold {
            p.text(r.center() + vec2(0.0, 13.0), Align2::CENTER_CENTER, t("hold", "giữ"), FontId::proportional(10.5), Color32::from_white_alpha(170));
        }
        fired
    }

    /// The tether test on (its limits on the flight controller, kept there while it is on - see
    /// sync_speed - and what they replace remembered) or off (all put back). On needs what the
    /// flight controller has now, to put it back after: false while it has not told it.
    fn set_tether(&mut self, on: bool, v: &Vehicle) -> bool {
        if on == self.cfg.tether {
            return true;
        }
        if on {
            let prev: Vec<f64> = TETHER_KEEP.iter().filter_map(|id| v.param(id)).collect();
            if prev.len() != TETHER_KEEP.len() {
                let _ = self.cmds.send(Cmd::ReadParams(&TETHER_KEEP));
                return false;
            }
            self.cfg.tether_prev = Some([prev[0], prev[1], prev[2]]);
            self.cfg.tether = true;
            log(&format!("pilot: tether test on (was MPC_THR_MAX {} NAV_RCL_ACT {} NAV_DLL_ACT {})", prev[0], prev[1], prev[2]));
        } else {
            if let Some(prev) = self.cfg.tether_prev {
                for (id, n) in tether_off_params(prev) {
                    let _ = self.cmds.send(Cmd::SetParam(id, n));
                }
            }
            self.cfg.tether = false;
            log("pilot: tether test off - the flight controller's settings put back");
        }
        self.cfg.save(&self.cfg_file);
        self.speed_tries = 0;
        self.speed_sync_at = Instant::now() - Duration::from_secs(60);
        true
    }

    /// The speed mode picked (C / N / S): its limits onto the flight controller; armed in a mode
    /// that is not its position mode (Altitude, Stabilized), back into it
    fn pick_speed(&mut self, s: Speed, v: &Vehicle) {
        if self.cfg.speed != s {
            self.cfg.speed = s;
            self.cfg.save(&self.cfg_file);
        }
        self.speed_tries = 0;
        self.speed_sync_at = Instant::now() - Duration::from_secs(60);
        self.sync_speed(v);
        if v.armed && matches!(v.flight_mode(), Some(FlightMode::Altitude | FlightMode::Stabilized)) {
            let _ = self.cmds.send(Cmd::Mode(FlightMode::Position));
        }
        log(&format!("pilot: speed mode {}", s.name()));
    }

    /// Keeps the flight controller on the speed mode's limits: what it told differs (or was never
    /// told), they are set again - every 3 s, five times at most until the link or the mode changes
    fn sync_speed(&mut self, v: &Vehicle) {
        if !v.link() {
            self.speed_tries = 0;
            return;
        }
        let mut want: Vec<(&'static str, f64)> = self.cfg.speed.params(v.autopilot).to_vec();
        if self.cfg.tether && v.autopilot == Autopilot::Px4 {
            want.extend(tether_params(&self.cfg));
        }
        let off: Vec<(&'static str, f64)> = want.into_iter().filter(|(id, n)| v.param(id).is_none_or(|x| (x - n).abs() > 0.01)).collect();
        if off.is_empty() || self.speed_tries >= 5 || self.speed_sync_at.elapsed() < Duration::from_secs(3) {
            return;
        }
        self.speed_sync_at = Instant::now();
        self.speed_tries += 1;
        for (id, n) in off {
            let _ = self.cmds.send(Cmd::SetParam(id, n));
        }
    }

    /// The gamepad (or RC transmitter) in mode 2: left stick yaw and throttle, right stick roll
    /// and pitch
    fn gamepad(&mut self) -> Option<Sticks> {
        // an RC transmitter first (read as it is held, rc.rs); looked for every 2 s while none
        if self.rc.is_none() && self.rc_seek.elapsed() >= Duration::from_secs(2) {
            self.rc_seek = Instant::now();
            self.rc = rc::Rc::find();
            if let Some(r) = &self.rc {
                log(&format!("pilot: RC transmitter '{}' - channels {}", r.name, self.cfg.rc_order.key()));
            }
        }
        if let Some(r) = &self.rc {
            match r.channels() {
                Some(ch) => {
                    let (roll, pitch, throttle, yaw) = self.cfg.rc_order.sticks(&ch);
                    let s = Sticks { roll, pitch, yaw, throttle };
                    self.pad_name = Some(format!("{} (RC)", if r.name.is_empty() { "RC" } else { &r.name }));
                    self.rc_seen = Some(s);
                    self.link.rc_thr.store(!self.cfg.rc_center, Ordering::Relaxed);
                    if let Some(g) = self.gilrs.as_mut() {
                        while g.next_event().is_some() {}
                    }
                    // (the throttle as it stands; the springing sticks a hair's dead band)
                    let dz = |v: f32| if v.abs() < 0.03 { 0.0 } else { v };
                    return Some(Sticks { roll: dz(roll), pitch: dz(pitch), yaw: dz(yaw), throttle });
                }
                None => {
                    log("pilot: the RC transmitter is gone");
                    self.rc = None;
                    self.rc_seen = None;
                    self.pad_name = None;
                }
            }
        }
        self.link.rc_thr.store(false, Ordering::Relaxed);
        let g = self.gilrs.as_mut()?;
        while g.next_event().is_some() {}
        let (_, pad) = g.gamepads().find(|(_, p)| p.is_connected())?;
        self.pad_name = Some(pad.name().to_string());
        use gilrs::Axis;
        // a gamepad's short sticks shake a little under the thumb: a dead band (the rest scaled
        // back to full) and a 60 ms smoothing, so what goes up - the throttle a rate in
        // Stabilized - does not tremble with them
        let dz = |v: f32| {
            let a = v.abs();
            if a < PAD_DZ { 0.0 } else { v.signum() * ((a - PAD_DZ) / (1.0 - PAD_DZ)).min(1.0) }
        };
        let raw = sticks_of(
            self.cfg.stick_mode,
            vec2(dz(pad.value(Axis::LeftStickX)), dz(pad.value(Axis::LeftStickY))),
            vec2(dz(pad.value(Axis::RightStickX)), dz(pad.value(Axis::RightStickY))),
        );
        let now = Instant::now();
        let dt = (now - self.pad_t).as_secs_f32().min(0.1);
        self.pad_t = now;
        let k = 1.0 - (-dt / 0.06).exp();
        let f = |s: &mut f32, r: f32| {
            *s += (r - *s) * k;
            // (let go: back to exactly the middle, nothing left over to creep the throttle)
            if r == 0.0 && s.abs() < 0.02 {
                *s = 0.0;
            }
        };
        let s = &mut self.pad_s;
        f(&mut s.roll, raw.roll);
        f(&mut s.pitch, raw.pitch);
        f(&mut s.yaw, raw.yaw);
        f(&mut s.throttle, raw.throttle);
        Some(*s)
    }
}

/// DJI's top line: how it is and what it is doing, in a word or two, and its colour. `video`: the
/// link carries the picture. Not ready: the flight controller's own last warning says why.
fn flight_status(v: &Vehicle, video: bool) -> (String, Color32) {
    if !v.link() {
        return if video {
            (t("Video only: the flight controller is not heard", "Chỉ có hình: chưa nghe thấy bộ điều khiển bay").into(), RED)
        } else {
            (t("Disconnected", "Mất kết nối").into(), RED)
        };
    }
    let gps = v.global().is_some();
    let tag = if gps { "GPS" } else { t("no GPS", "không GPS") };
    let col = if gps { GREEN } else { AMBER };
    let ardu = v.autopilot == Autopilot::ArduPilot;
    if v.armed {
        let doing = match (ardu, v.main_mode, v.sub_mode) {
            (false, 4, 2) => Some(t("Taking off", "Đang cất cánh")),
            (false, 4, 5) | (true, 6 | 21, _) => Some(t("Returning home", "Đang quay về")),
            (false, 4, 6) | (true, 9, _) => Some(t("Landing", "Đang hạ cánh")),
            (false, 4, 3) | (true, 17, _) => Some(t("Hovering", "Đang dừng")),
            _ => None,
        };
        if let Some(d) = doing {
            return (format!("{d} ({tag})"), col);
        }
        if v.in_air() {
            return (format!("{} ({tag})", t("In flight", "Đang bay")), col);
        }
        return (t("Motors running", "Motor đang quay").into(), AMBER);
    }
    match v.prearm {
        Some(true) => (format!("{} ({tag})", t("Ready to fly", "Sẵn sàng bay")), col),
        Some(false) => {
            // (its words: the last warning it gave this minute)
            let why = v.texts.iter().rev().find(|(at, sev, _)| *sev <= 4 && at.elapsed() < Duration::from_secs(60)).map(|x| x.2.clone());
            let why = why.unwrap_or_else(|| t("checks not passed", "chưa qua kiểm tra trước bay").into());
            (format!("{}: {why}", t("Not ready", "Chưa sẵn sàng")), RED)
        }
        None => (t("Checking…", "Đang kiểm tra…").into(), MUTED),
    }
}

/// "12:34" (minutes and seconds; an hour and more as "1:02:03")
fn clock(d: Duration) -> String {
    let s = d.as_secs();
    if s >= 3600 {
        format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{:02}:{:02}", s / 60, s % 60)
    }
}

/// The signal's bars, as DJI's: four, lit by the video's SNR (none and red: no picture)
fn signal_bars(p: &egui::Painter, at: Pos2, live: bool, snr: f32) {
    let lit = if !live { 0 } else { [5.0, 10.0, 15.0, 20.0].iter().filter(|&&s| snr >= s).count().max(1) };
    let col = match lit {
        0 => RED,
        1 => AMBER,
        _ => WHITE,
    };
    for i in 0..4 {
        let h = 5.0 + 3.5 * i as f32;
        let r = Rect::from_min_max(pos2(at.x + i as f32 * 6.0, at.y - h), pos2(at.x + i as f32 * 6.0 + 4.0, at.y));
        p.rect_filled(r, 1.0, if i < lit { col } else { Color32::from_white_alpha(60) });
    }
}

fn shadow_text(p: &egui::Painter, at: Pos2, anchor: Align2, text: &str, font: FontId, color: Color32) {
    p.text(at + vec2(1.0, 1.0), anchor, text, font.clone(), Color32::from_black_alpha(200));
    p.text(at, anchor, text, font, color);
}

/// Words straight over the large view: a shadow does over the video, over the map (any colour
/// under them) they stand on a plate
fn over(p: &egui::Painter, at: Pos2, anchor: Align2, text: &str, size: f32, color: Color32, on_map: bool) {
    if on_map {
        map::tag(p, at, anchor, text, size, color);
    } else {
        shadow_text(p, at, anchor, text, FontId::proportional(size), color);
    }
}

fn panel(p: &egui::Painter, r: Rect) {
    p.rect_filled(r, 8.0, Color32::from_rgba_unmultiplied(10, 15, 22, 185));
}

const WHITE: Color32 = Color32::from_rgb(235, 240, 245);
const MUTED: Color32 = Color32::from_rgb(150, 162, 176);
const GREEN: Color32 = Color32::from_rgb(63, 185, 80);
const AMBER: Color32 = Color32::from_rgb(230, 170, 40);
const RED: Color32 = Color32::from_rgb(248, 81, 73);
const ACCENT: Color32 = Color32::from_rgb(74, 163, 255);

/// The artificial horizon: the horizon and a pitch ladder rotated by the roll and moved by the
/// pitch, the aircraft's mark fixed in the middle
fn horizon(p: &egui::Painter, c: Pos2, r: f32, roll: f32, pitch: f32) {
    let ppd = r / 25.0;
    let d = vec2(roll.cos(), -roll.sin());
    let n = vec2(roll.sin(), roll.cos());
    let clip = p.with_clip_rect(Rect::from_center_size(c, Vec2::splat(2.2 * r)));
    let line = |deg: f32, half: f32, w: f32| {
        let m = c + n * ((pitch.to_degrees() - deg) * ppd);
        let s = Stroke::new(w, Color32::from_white_alpha(220));
        clip.line_segment([m - d * half + vec2(1.0, 1.0), m + d * half + vec2(1.0, 1.0)], Stroke::new(w, Color32::from_black_alpha(160)));
        clip.line_segment([m - d * half, m + d * half], s);
    };
    line(0.0, r * 1.6, 2.0);
    for deg in [-20.0, -10.0, 10.0, 20.0] {
        line(deg, r * 0.35, 1.5);
        let m = c + n * ((pitch.to_degrees() - deg) * ppd) + d * (r * 0.35 + 14.0);
        clip.text(m, Align2::CENTER_CENTER, format!("{}", deg as i32), FontId::proportional(12.0), WHITE);
    }
    // the aircraft
    let y = Stroke::new(3.0, AMBER);
    p.line_segment([c + vec2(-r * 0.45, 0.0), c + vec2(-r * 0.15, 0.0)], y);
    p.line_segment([c + vec2(-r * 0.15, 0.0), c + vec2(0.0, r * 0.08)], y);
    p.line_segment([c + vec2(0.0, r * 0.08), c + vec2(r * 0.15, 0.0)], y);
    p.line_segment([c + vec2(r * 0.15, 0.0), c + vec2(r * 0.45, 0.0)], y);
    p.circle_filled(c, 3.0, AMBER);
}

/// The heading tape along the top: +-45 degrees, a tick every 5, a label every 30
fn heading_tape(p: &egui::Painter, top: Pos2, w: f32, yaw: f32, on_map: bool) {
    let hdg = (yaw.to_degrees() + 360.0) % 360.0;
    let r = Rect::from_center_size(top + vec2(0.0, 18.0), vec2(w, 36.0));
    panel(p, r);
    let ppd = w / 90.0;
    let clip = p.with_clip_rect(r);
    let start = (hdg - 45.0).floor() as i32;
    for d in start..=start + 90 {
        if d % 5 != 0 {
            continue;
        }
        let x = r.center().x + (d as f32 - hdg) * ppd;
        let dd = d.rem_euclid(360);
        let tall = dd % 30 == 0;
        clip.line_segment([pos2(x, r.bottom()), pos2(x, r.bottom() - if tall { 10.0 } else { 5.0 })], Stroke::new(1.0, MUTED));
        if tall {
            let label = match dd {
                0 => "N".to_string(),
                90 => "E".to_string(),
                180 => "S".to_string(),
                270 => "W".to_string(),
                _ => format!("{dd}"),
            };
            clip.text(pos2(x, r.top() + 11.0), Align2::CENTER_CENTER, label, FontId::proportional(13.0), WHITE);
        }
    }
    over(p, pos2(r.center().x, r.bottom() + 12.0), Align2::CENTER_CENTER, &format!("{hdg:03.0}°"), 15.0, AMBER, on_map);
}

/// A stick: a ring, its knob where the pointer drags it (back to the middle on release); the
/// knob's offset in -1..1 (x right, y up)
/// An on-screen stick. `touched`: a finger holds it (the touches, above): `pos` is theirs, the
/// pointer (which a touch screen makes of its first finger) left out.
fn stick(ui: &mut egui::Ui, centre: Pos2, radius: f32, pos: &mut Vec2, shown: Vec2, id: &str, touched: bool) {
    let rect = Rect::from_center_size(centre, Vec2::splat(radius * 2.4));
    let resp = ui.interact(rect, ui.id().with(id), egui::Sense::drag());
    if touched {
    } else if let Some(pt) = resp.interact_pointer_pos().filter(|_| resp.dragged()) {
        let v = (pt - centre) / radius;
        let v = if v.length() > 1.0 { v.normalized() } else { v };
        *pos = vec2(v.x, -v.y);
    } else {
        *pos = Vec2::ZERO;
    }
    let dragged = touched || resp.dragged();
    let p = ui.painter();
    p.circle_filled(centre, radius, Color32::from_rgba_unmultiplied(10, 15, 22, 140));
    p.circle_stroke(centre, radius, Stroke::new(2.0, Color32::from_white_alpha(70)));
    p.line_segment([centre - vec2(radius, 0.0), centre + vec2(radius, 0.0)], Stroke::new(1.0, Color32::from_white_alpha(35)));
    p.line_segment([centre - vec2(0.0, radius), centre + vec2(0.0, radius)], Stroke::new(1.0, Color32::from_white_alpha(35)));
    // (not dragged: where the sticks that go up are, the gamepad's or the demo's)
    let k = if dragged { *pos } else { shown };
    let knob = centre + vec2(k.x, -k.y) * radius;
    p.circle_filled(knob, radius * 0.28, if dragged { ACCENT } else { Color32::from_white_alpha(160) });
}

/// The video in `within`, its aspect kept
fn video(p: &egui::Painter, within: Rect, tex: &egui::TextureHandle) {
    let [w, h] = tex.size();
    let s = (within.width() / w as f32).min(within.height() / h as f32);
    let r = Rect::from_center_size(within.center(), vec2(w as f32 * s, h as f32 * s));
    p.image(tex.id(), r, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)), Color32::WHITE);
}

impl eframe::App for Pilot {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = root.ctx().clone();
        ctx.request_repaint_after(Duration::from_millis(16));
        // a screen too low for the layout (a phone on its side: ~400 points): drawn smaller until
        // it is 560 points high; a large one as the system has it
        if let Some(native) = ctx.native_pixels_per_point() {
            let px_high = ctx.content_rect().height() * ctx.pixels_per_point();
            let want = (px_high / 560.0).clamp(0.5, native);
            if (ctx.pixels_per_point() - want).abs() > 0.02 {
                ctx.set_pixels_per_point(want);
            }
        }
        if let Some((ver, f)) = self.rx.frame_since(self.tex_ver) {
            self.tex_ver = ver;
            if f.width > 0 && f.height > 0 && f.rgb.len() == f.width * f.height * 3 {
                let img = nyx_common::to_color_image(&f);
                match &mut self.tex {
                    Some(t) if t.size() == [f.width, f.height] => t.set(img, egui::TextureOptions::LINEAR),
                    _ => self.tex = Some(ctx.load_texture("video", img, egui::TextureOptions::LINEAR)),
                }
            }
        }
        let lv = self.rx.link_view();
        let v = self.link.veh.lock().unwrap().clone();
        // where it is seen on the globe, kept for the next run's map (written as it moves on)
        if let Some((ll, from)) = v.place() {
            if from != Place::Last && self.place_kept.is_none_or(|k| map::dist_bearing(k, ll).0 > 25.0) {
                self.place_kept = Some(ll);
                if let Some(dir) = self.place_file.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                let _ = std::fs::write(&self.place_file, format!("{:.7},{:.7}\n", ll.0, ll.1));
            }
        }
        // the map and the video change places: a tap on the small one, or M
        let mut swap = false;
        // a text box in Settings has the keys: none of them flies the aircraft then (T takes off,
        // L lands, the arrows and W/A/S/D are the sticks)
        let typing = ctx.egui_wants_keyboard_input();

        // Settings: a column on the right (wide), or the whole window (narrow, as nyxhop's drawer)
        let mut toggle_settings = false;
        let mut covered = false;
        if self.settings_open {
            // the flight controller's answers kept fresh while they are on show
            if v.link() && self.params_asked.elapsed() >= Duration::from_secs(3) {
                self.params_asked = Instant::now();
                let ids = if v.autopilot == Autopilot::ArduPilot { ARDU_SAFETY } else { PX4_SAFETY };
                let _ = self.cmds.send(Cmd::ReadParams(ids));
            }
            let frame = egui::Frame::new().fill(th::BG).inner_margin(egui::Margin::same(10));
            if root.max_rect().width() >= 900.0 {
                egui::Panel::right("pilot_settings")
                    .resizable(false)
                    .default_size((root.max_rect().width() * 0.36).clamp(420.0, 520.0))
                    .frame(frame)
                    .show(root, |ui| self.settings(ui, &v));
            } else {
                egui::CentralPanel::default().frame(frame).show(root, |ui| self.settings(ui, &v));
                covered = true;
                // (the on-screen sticks are under it: let go)
                self.left = Vec2::ZERO;
                self.right = Vec2::ZERO;
            }
        }

        if !covered {
        egui::CentralPanel::default().frame(egui::Frame::NONE.fill(Color32::BLACK)).show(root, |ui| {
            let full = ui.max_rect();
            let p = ui.painter().clone();
            // DJI's layout: a bar along the top (how it is, C / N / S; the satellites, the signal,
            // the battery, the flight's time, Settings), the take-off / return column on the left,
            // the link and the small view on the right, D / H / HS / VS between the sticks
            let bar = Rect::from_min_size(full.left_top(), vec2(full.width(), 44.0));
            let tr = Rect::from_min_size(pos2(full.right() - 244.0, bar.bottom() + 10.0), vec2(230.0, 52.0));
            let small = Rect::from_min_size(tr.left_bottom() + vec2(0.0, 10.0), vec2(230.0, if self.map_large { 129.0 } else { 172.0 }));
            // (the map's zoom buttons: under the small view)
            let zoom = small.right_bottom() + vec2(0.0, 10.0);
            let (col_x, bw, bh) = (full.left() + 14.0, 118.0, 56.0);
            // the large view: the video, or the map (first, so that all else is over it)
            if self.map_large {
                let at = map::Corners {
                    tl: pos2(col_x + bw + 14.0, bar.bottom() + 10.0),
                    bl: full.left_bottom() + vec2(14.0, -4.0),
                    br: full.right_bottom() + vec2(-6.0, -3.0),
                    zoom,
                };
                self.map.show(ui, full, at, &v, true);
            } else if let Some(t) = &self.tex {
                video(&p, full, t);
            } else {
                p.text(full.center() + vec2(0.0, 80.0), Align2::CENTER_CENTER, t("waiting for video…", "đang chờ hình…"), FontId::proportional(18.0), MUTED);
            }

            // the attitude (over the video: the horizon belongs to what the camera sees), the heading
            let c = full.center() - vec2(0.0, full.height() * 0.05);
            let r = full.height().min(full.width()) * 0.16;
            if !self.map_large {
                horizon(&p, c, r, v.roll, v.pitch);
            }
            heading_tape(&p, pos2(full.center().x, bar.bottom() + 8.0), (full.width() * 0.32).max(280.0), v.yaw, self.map_large);

            // the top bar, right to left: Settings, the flight's time, the battery, the signal, the
            // satellites
            p.rect_filled(bar, 0.0, Color32::from_rgba_unmultiplied(10, 14, 20, 210));
            let mid = bar.center().y;
            let sb = Rect::from_min_max(pos2(bar.right() - 114.0, bar.top() + 5.0), pos2(bar.right() - 10.0, bar.bottom() - 5.0));
            if map::button(ui, sb, "settings", t("Settings", "Cài đặt"), 15.0) {
                toggle_settings = true;
            }
            if self.settings_open {
                p.rect_stroke(sb, 6.0, Stroke::new(2.0, ACCENT), egui::StrokeKind::Outside);
            }
            let mut x = sb.left() - 16.0;
            let item = |x: &mut f32, text: &str, col: Color32| {
                let g = p.layout_no_wrap(text.to_string(), FontId::proportional(15.0), col);
                let size = g.size();
                p.galley(pos2(*x - size.x, mid - size.y / 2.0), g, col);
                *x -= size.x + 18.0;
            };
            item(&mut x, &v.flight_time().map_or_else(|| "00:00".to_string(), clock), if v.armed { WHITE } else { MUTED });
            let (batt, bcol) = if v.link() && v.batt_v > 0.0 {
                let pct = v.batt_pct;
                let col = if (0..=15).contains(&pct) {
                    RED
                } else if (0..=30).contains(&pct) {
                    AMBER
                } else {
                    WHITE
                };
                (if pct >= 0 { format!("{pct}%  {:.1} V", v.batt_v) } else { format!("{:.1} V", v.batt_v) }, col)
            } else {
                ("—".to_string(), MUTED)
            };
            item(&mut x, &batt, bcol);
            signal_bars(&p, pos2(x - 22.0, mid + 8.0), lv.live, lv.snr_db);
            x -= 40.0;
            let (gps_txt, gps_col) = match v.gps_fix() {
                Some((f, s)) if f >= 3 => (if s == 255 { "GPS".to_string() } else { format!("GPS {s}") }, if s != 255 && s < 8 { AMBER } else { WHITE }),
                Some((_, s)) => (if s == 255 { "GPS".to_string() } else { format!("GPS {s}") }, AMBER),
                None => ("GPS —".to_string(), MUTED),
            };
            item(&mut x, &gps_txt, gps_col);
            let right_edge = x;

            // ... left to right: how it is (green ready / flying with satellites, amber without,
            // red not ready and why), C / N / S, the flight controller's own mode
            let (st, st_col) = if self.rc_note.is_some_and(|t| t.elapsed() < Duration::from_secs(4)) {
                let s = if v.manual_thrust() && self.cfg.rc_center {
                    t("Throttle stick to the middle, then the motors on", "Để cần ga ở giữa rồi mới bật motor")
                } else if v.manual_thrust() {
                    t("Throttle to the bottom, then the motors on", "Kéo ga xuống đáy rồi mới bật motor")
                } else {
                    t("Throttle to the middle or lower, then the motors on", "Để ga ở giữa hoặc thấp hơn rồi mới bật motor")
                };
                (s.to_string(), AMBER)
            } else {
                flight_status(&v, lv.live)
            };
            let room = (right_edge - bar.left() - 10.0 - 3.0 * 40.0 - 30.0).max(140.0);
            let max_chars = ((room - 24.0) / 7.6).max(10.0) as usize;
            let st = if st.chars().count() > max_chars { st.chars().take(max_chars - 1).collect::<String>().trim_end().to_string() + "…" } else { st };
            let ink = Color32::from_rgb(12, 16, 22);
            let g = p.layout_no_wrap(st, FontId::proportional(15.0), ink);
            let pill = Rect::from_min_size(pos2(bar.left() + 10.0, mid - 15.0), vec2(g.size().x + 24.0, 30.0));
            p.rect_filled(pill, 6.0, st_col);
            p.galley(pill.center() - g.size() / 2.0, g, ink);
            let mut lx = pill.right() + 12.0;
            let told = Speed::told(&v);
            let mut picked = None;
            for s in Speed::ALL {
                let r = Rect::from_min_size(pos2(lx, mid - 15.0), vec2(36.0, 30.0));
                let on = self.cfg.speed == s;
                let [vh, ..] = s.limits();
                let resp = ui.interact(r, ui.id().with(("speed", s.key())), egui::Sense::click()).on_hover_text(format!("{} · {vh:.0} m/s", s.name()));
                let fill = if on { ACCENT } else { Color32::from_rgba_unmultiplied(40, 52, 66, 230) };
                p.rect_filled(r, 6.0, if resp.hovered() && !on { fill.gamma_multiply(1.5) } else { fill });
                // (picked here, the aircraft not flying with it yet: an amber edge)
                if on && v.link() && told != Some(s) {
                    p.rect_stroke(r, 6.0, Stroke::new(2.0, AMBER), egui::StrokeKind::Inside);
                }
                p.text(r.center(), Align2::CENTER_CENTER, s.letter(), FontId::proportional(17.0), WHITE);
                if resp.clicked() {
                    picked = Some(s);
                }
                lx = r.right() + 4.0;
            }
            if let Some(s) = picked {
                self.pick_speed(s, &v);
            }
            // the tether test on: how far it is from its limits now
            if self.cfg.tether && v.link() {
                let tilt = (v.roll.cos() * v.pitch.cos()).clamp(-1.0, 1.0).acos().to_degrees();
                let yaw = v.yaw_rate.abs().to_degrees();
                let txt = format!("{} {tilt:.0}°/{:.0} · {yaw:.0}/{:.0}°/s", t("TETHER", "CÓ DÂY"), self.cfg.tether_tilt, self.cfg.tether_yawr);
                let g = p.layout_no_wrap(txt, FontId::proportional(13.0), AMBER);
                let w = g.size().x + 14.0;
                if lx + w + 10.0 < right_edge {
                    let r = Rect::from_min_size(pos2(lx + 8.0, mid - 12.0), vec2(w, 24.0));
                    p.rect_stroke(r, 5.0, Stroke::new(1.5, AMBER), egui::StrokeKind::Inside);
                    p.galley(r.center() - g.size() / 2.0, g, AMBER);
                    lx = r.right() + 2.0;
                }
            }
            if v.link() && lx + 130.0 < right_edge {
                // ATTI (DJI's word): holding the height alone, no position (no satellites, or lost)
                let fm = v.flight_mode();
                let (txt, col) = match fm {
                    Some(FlightMode::Altitude) => ("ATTI".to_string(), AMBER),
                    Some(FlightMode::Stabilized) => (v.mode_name().to_string(), AMBER),
                    _ => (v.mode_name().to_string(), MUTED),
                };
                p.text(pos2(lx + 10.0, mid), Align2::LEFT_CENTER, txt, FontId::proportional(13.0), col);
            }

            // right: the link, the small view under it
            panel(&p, tr);
            let (lc, lt) = match (lv.live, v.link()) {
                (true, true) => (GREEN, t("link · video and control", "link · hình và điều khiển")),
                (_, true) => (AMBER, t("control only", "chỉ có điều khiển")),
                (true, false) => (AMBER, t("video only", "chỉ có hình")),
                _ => (RED, t("no link", "mất link")),
            };
            shadow_text(&p, tr.left_top() + vec2(12.0, 8.0), Align2::LEFT_TOP, lt, FontId::proportional(14.0), lc);
            let rtt = v.rtt_ms.map_or("—".into(), |r| format!("{r:.0} ms"));
            shadow_text(&p, tr.left_top() + vec2(12.0, 30.0), Align2::LEFT_TOP, &format!("SNR {:.0} dB · MCS {} · {:.0} fps · {rtt}", lv.snr_db, lv.mcs, lv.fps), FontId::proportional(12.0), MUTED);
            // under it the small view: the map, or the video once they changed places
            if self.map_large {
                p.rect_filled(small, 0.0, Color32::BLACK);
                match &self.tex {
                    Some(t) => video(&p, small, t),
                    None => {
                        p.text(small.center(), Align2::CENTER_CENTER, t("waiting for video…", "đang chờ hình…"), FontId::proportional(13.0), MUTED);
                    }
                }
                p.rect_stroke(small, 0.0, Stroke::new(1.0, Color32::from_white_alpha(90)), egui::StrokeKind::Outside);
                swap |= ui.interact(small, ui.id().with("small video"), egui::Sense::click()).clicked();
            } else {
                let within = small.shrink(5.0);
                let at = map::Corners { tl: within.left_top(), bl: within.left_bottom(), br: within.right_bottom(), zoom };
                swap |= self.map.show(ui, small, at, &v, false);
            }

            // the flight controller's last words, a few seconds each
            if let Some((at, sev, txt)) = v.texts.back().filter(|x| x.0.elapsed() < Duration::from_secs(6)) {
                let _ = at;
                let col = if *sev <= 3 { RED } else if *sev <= 4 { AMBER } else { WHITE };
                over(&p, pos2(full.center().x, bar.bottom() + 80.0), Align2::CENTER_CENTER, txt, 15.0, col, self.map_large);
            }
            // a low battery in the air: land (DJI's red line)
            if v.in_air() && v.batt_v > 0.0 && (0..=15).contains(&v.batt_pct) {
                over(&p, pos2(full.center().x, bar.bottom() + 106.0), Align2::CENTER_CENTER, t("Low battery: land now", "Pin yếu: hạ cánh ngay"), 17.0, RED, self.map_large);
            }

            // --demo: what it is doing (--demo-quiet: not said, a still for the docs)
            let step = self.link.demo_step.lock().unwrap().clone();
            if !step.is_empty() && self.demo_tag {
                let r = Rect::from_center_size(pos2(full.center().x, bar.bottom() + 134.0), vec2(360.0, 30.0));
                panel(&p, r);
                shadow_text(&p, r.center(), Align2::CENTER_CENTER, &format!("DEMO · {step}"), FontId::proportional(14.0), ACCENT);
            }

            // the left column, as DJI's: take off (land once up) and return home held for a
            // second, stop at a touch; the motors by hand (take off starts them itself)
            let key_down = |k: egui::Key| !typing && ctx.input(|i| i.key_down(k));
            let at = |i: usize, gap: f32| Rect::from_min_size(pos2(col_x, bar.bottom() + 16.0 + i as f32 * (bh + 10.0) + gap), vec2(bw, bh));
            let blue = Color32::from_rgb(36, 104, 196);
            let dark = Color32::from_rgba_unmultiplied(28, 39, 51, 235);
            if v.in_air() {
                if self.big_button(ui, at(0, 0.0), "land", t("Land", "Hạ cánh"), dark, true, key_down(egui::Key::L), false) {
                    let _ = self.cmds.send(Cmd::Land);
                }
            } else if self.big_button(ui, at(0, 0.0), "takeoff", t("Take off", "Cất cánh"), blue, true, key_down(egui::Key::T), false) {
                let _ = self.cmds.send(Cmd::Takeoff(self.takeoff_h));
            }
            if self.big_button(ui, at(1, 0.0), "return", t("Return home", "Về nhà"), dark, true, key_down(egui::Key::R), false) {
                let _ = self.cmds.send(Cmd::Return);
            }
            let stop_key = !typing && ctx.input(|i| i.key_pressed(egui::Key::H));
            if self.big_button(ui, at(2, 0.0), "stop", t("Hover", "Đứng yên"), dark, false, false, stop_key) {
                let _ = self.cmds.send(Cmd::Mode(FlightMode::Hold));
            }
            let (ml, mf) = if v.armed { (t("Motors off", "Tắt motor"), Color32::from_rgb(140, 40, 36)) } else { (t("Motors on", "Bật motor"), dark) };
            if self.big_button(ui, at(3, 14.0), "motors", ml, mf, true, false, false) {
                // an RC transmitter's throttle up: not armed (it would leap) - at the bottom in a
                // manual-thrust mode (the middle, its stick springing back: the held throttle
                // starts at 0 and the stick moves it), at most the middle (hold the height) in the
                // others
                let center = self.cfg.rc_center;
                let high = self.rc_seen.is_some_and(|s| match (v.manual_thrust(), center) {
                    (true, false) => s.throttle > -0.9,
                    (true, true) => s.throttle.abs() > 0.1,
                    (false, _) => s.throttle > 0.1,
                });
                if !v.armed && high {
                    self.rc_note = Some(Instant::now());
                } else {
                    let _ = self.cmds.send(Cmd::Arm(!v.armed));
                }
            }
            // the emergency stop (Settings > Safety shows or hides it): one touch, or Space
            if self.cfg.estop {
                let kill_key = !typing && ctx.input(|i| i.key_pressed(egui::Key::Space));
                let r = Rect::from_min_size(at(4, 30.0).min, vec2(bw, bh + 14.0));
                if self.big_button(ui, r, "estop", t("EMERGENCY\nSTOP", "DỪNG\nKHẨN CẤP"), Color32::from_rgb(205, 28, 28), false, false, kill_key) {
                    let _ = self.cmds.send(Cmd::Kill);
                }
            }

            // the sticks (what each moves: the stick mode, Settings > Control); smaller on a
            // narrow view (Settings open beside it) so the numbers fit between them
            let sr = (full.height() * 0.12).min(full.width() * 0.075).clamp(48.0, 110.0);
            let ly = full.bottom() - sr - 40.0;
            let (lc, rc) = (pos2(full.left() + sr + 40.0, ly), pos2(full.right() - sr - 40.0, ly));
            let (ln, rn) = stick_names(self.cfg.stick_mode);
            let name = |at: Pos2, text: &str| over(&p, at + vec2(0.0, sr + 14.0), Align2::CENTER_CENTER, text, 12.0, MUTED, self.map_large);
            // touches (a phone, a tablet): each finger holds the stick it went down on, two at once
            // (the mouse's single pointer cannot); a stick held so is left alone by stick() below
            let touches: Vec<(u64, egui::TouchPhase, Pos2)> = ctx.input(|i| {
                i.events
                    .iter()
                    .filter_map(|e| match e {
                        egui::Event::Touch { id, phase, pos, .. } => Some((id.0, *phase, *pos)),
                        _ => None,
                    })
                    .collect()
            });
            for (id, phase, pos) in touches {
                let centres = [lc, rc];
                if phase == egui::TouchPhase::Start {
                    if let Some(side) = (0..2).find(|&k| self.fingers[k].is_none() && (pos - centres[k]).length() <= sr * 1.3) {
                        self.fingers[side] = Some(id);
                    }
                }
                for side in 0..2 {
                    if self.fingers[side] != Some(id) {
                        continue;
                    }
                    let knob = if matches!(phase, egui::TouchPhase::End | egui::TouchPhase::Cancel) {
                        self.fingers[side] = None;
                        Vec2::ZERO
                    } else {
                        let v = (pos - centres[side]) / sr;
                        let v = if v.length() > 1.0 { v.normalized() } else { v };
                        vec2(v.x, -v.y)
                    };
                    if side == 0 { self.left = knob } else { self.right = knob }
                }
            }
            stick(ui, lc, sr, &mut self.left, self.shown.0, "L", self.fingers[0].is_some());
            name(lc, ln);
            // Stabilized: the throttle held (it stays where the stick left it), over its stick
            if let Some(l) = *self.link.thr.lock().unwrap() {
                let at = if self.cfg.stick_mode == 2 { lc } else { rc } - vec2(0.0, sr + 18.0);
                over(&p, at, Align2::CENTER_CENTER, &format!("{} {:.0} %", t("throttle", "ga"), l * 100.0), 16.0, if l > 0.0 { AMBER } else { WHITE }, self.map_large);
            }
            stick(ui, rc, sr, &mut self.right, self.shown.1, "R", self.fingers[1].is_some());
            name(rc, rn);

            // D / H / HS / VS, as DJI's: from home, height, speed over the ground, climb
            let room = full.width() - 2.0 * (2.0 * sr + 50.0);
            let sw = room.clamp(300.0, 560.0);
            let strip = Rect::from_center_size(pos2(full.center().x, full.bottom() - 54.0), vec2(sw, 50.0));
            panel(&p, strip);
            let on = v.link();
            let num = |s: String| if on { s } else { "—".to_string() };
            // (no "-0.0" for a hair below nought)
            let z = |x: f32| if x.abs() < 0.05 { 0.0 } else { x };
            let height = if v.global().is_some() { v.alt } else { -v.z };
            let far = |d: f32| if d >= 1000.0 { format!("{:.2} km", d / 1000.0) } else { format!("{d:.0} m") };
            // D: on the satellites; with none, the link's own reading when it is on ("≈")
            let d = match (v.home_dist().filter(|_| on), self.range_on.then(|| nu::range_of(&self.board.snapshot())).flatten()) {
                (Some(d), _) => far(d),
                (None, Some((m, _))) => format!("≈{}", far(m)),
                _ => "—".to_string(),
            };
            let cells = [
                ("D", d),
                ("H", num(format!("{:.1} m", z(height)))),
                ("HS", num(format!("{:.1} m/s", z(v.vx.hypot(v.vy))))),
                ("VS", num(format!("{:.1} m/s", z(-v.vz)))),
            ];
            let cw = sw / 4.0;
            for (i, (k, val)) in cells.iter().enumerate() {
                let cx = strip.left() + cw * (i as f32 + 0.5);
                shadow_text(&p, pos2(cx, strip.top() + 12.0), Align2::CENTER_CENTER, k, FontId::proportional(11.0), MUTED);
                shadow_text(&p, pos2(cx, strip.top() + 32.0), Align2::CENTER_CENTER, val, FontId::proportional(19.0), WHITE);
            }
            let src = match &self.pad_name {
                Some(n) => format!("{}: {n}", t("sticks", "cần")),
                None => t("sticks: on screen · keyboard", "cần: trên màn hình · bàn phím").into(),
            };
            over(&p, pos2(full.center().x, full.bottom() - 12.0), Align2::CENTER_CENTER, &src, 11.0, MUTED, self.map_large);
        });
        }
        if toggle_settings {
            self.settings_open = !self.settings_open;
        }
        if self.settings_open && !typing && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.settings_open = false;
        }

        // the keys (not while a text box has them): T, L, R held as their buttons (in the HUD), H
        // there too; 1 2 3 the speed modes, 4 and 5 the advanced ones; the map
        if !typing {
            let (speed, adv, closer, m) = ctx.input(|i| {
                let speed = [egui::Key::Num1, egui::Key::Num2, egui::Key::Num3].iter().position(|&k| i.key_pressed(k));
                let adv = if i.key_pressed(egui::Key::Num4) {
                    Some(FlightMode::Altitude)
                } else if i.key_pressed(egui::Key::Num5) {
                    Some(FlightMode::Stabilized)
                } else {
                    None
                };
                let closer = (i.key_pressed(egui::Key::Plus) || i.key_pressed(egui::Key::Equals)) as i32 - i.key_pressed(egui::Key::Minus) as i32;
                (speed, adv, closer, i.key_pressed(egui::Key::M))
            });
            if let Some(s) = speed {
                self.pick_speed(Speed::ALL[s], &v);
            }
            if let Some(f) = adv {
                let _ = self.cmds.send(Cmd::Mode(f));
            }
            swap |= m;
            if closer != 0 {
                self.map.zoom_by(v.place().is_some(), closer as f32);
            }
        }
        // the speed mode's limits kept on the flight controller
        self.sync_speed(&v);
        // the link's own distance reading: on the ground board (the aircraft follows it) while the
        // pilot wants it and there are no satellites; asked again now and then while on (a board
        // that restarts comes up with it off), off only when this app turned it on
        let gps = v.global().is_some();
        if gps != self.gps_seen.0 {
            self.gps_seen = (gps, Instant::now());
        }
        let want = range_wanted(self.cfg.range_nogps, gps, self.range_on, self.gps_seen.1.elapsed());
        if want != self.range_on || (want && self.range_asked_at.elapsed() >= Duration::from_secs(10)) {
            if want != self.range_on {
                log(&format!("pilot: the link's distance reading {}", if want { "on (no satellites)" } else { "off" }));
            }
            self.board.send(if want { "range on" } else { "range off" });
            self.range_on = want;
            self.range_asked_at = Instant::now();
        }
        if swap {
            self.map_large = !self.map_large;
        }

        // the sticks going up: the demo's, else the gamepad, the screen and the keyboard
        let kb = if typing {
            Sticks::default()
        } else {
            ctx.input(|i| {
                let k = |a: egui::Key, b: egui::Key| (i.key_down(a) as i32 - i.key_down(b) as i32) as f32;
                Sticks {
                    roll: 0.6 * k(egui::Key::ArrowRight, egui::Key::ArrowLeft),
                    pitch: 0.6 * k(egui::Key::ArrowUp, egui::Key::ArrowDown),
                    yaw: 0.6 * k(egui::Key::D, egui::Key::A),
                    throttle: 0.5 * k(egui::Key::W, egui::Key::S),
                }
            })
        };
        let mode = self.cfg.stick_mode;
        let screen = sticks_of(mode, self.left, self.right);
        let pad = self.gamepad();
        let pick = |a: f32, b: f32, c: f32| {
            let s = a + b;
            if c.abs() > s.abs() { c } else { s.clamp(-1.0, 1.0) }
        };
        let pad = pad.unwrap_or_default();
        // an RC transmitter's throttle is where it stands: the screen's and the keys' do not add
        let rc_thr = self.rc_seen.is_some() && !self.cfg.rc_center;
        let demo = *self.link.demo.lock().unwrap();
        // the stick feel (Settings > Control) on roll, pitch and yaw; the throttle as it is
        let e = self.cfg.expo;
        let sticks = demo.unwrap_or(Sticks {
            roll: expo(pick(screen.roll, kb.roll, pad.roll), e),
            pitch: expo(pick(screen.pitch, kb.pitch, pad.pitch), e),
            yaw: expo(pick(screen.yaw, kb.yaw, pad.yaw), e),
            throttle: if rc_thr { pad.throttle } else { pick(screen.throttle, kb.throttle, pad.throttle) },
        });
        *self.link.sticks.lock().unwrap() = sticks;
        self.stick_commands(&v, sticks);
        self.calibration_steps(&v);
        // the on-screen sticks show what goes up (the demo's, the gamepad's) when not dragged
        if self.left == Vec2::ZERO && self.right == Vec2::ZERO {
            self.shown = knobs_of(mode, sticks);
        } else {
            self.shown = (self.left, self.right);
        }
    }
}

impl Pilot {
    /// A calibration asked: sent, and the next of its steps once the flight controller says this
    /// one is done (the IMU: gyro, then the accelerometer's six sides, then level)
    fn calibrate(&mut self, steps: &[CalKind]) {
        let Some((&first, rest)) = steps.split_first() else { return };
        self.cal_queue = rest.to_vec();
        self.cal_note = None;
        self.cal_asked = Some((first, Instant::now()));
        let _ = self.cmds.send(Cmd::Calibrate(Some(first)));
    }

    fn calibration_steps(&mut self, v: &Vehicle) {
        if v.cal.seq != self.cal_seq {
            self.cal_seq = v.cal.seq;
            if v.cal.what.is_some() {
                self.cal_asked = None;
            } else if let Some(r) = &v.cal.result {
                match r {
                    Ok(_) if !self.cal_queue.is_empty() => {
                        let next = self.cal_queue.remove(0);
                        self.cal_asked = Some((next, Instant::now()));
                        let _ = self.cmds.send(Cmd::Calibrate(Some(next)));
                    }
                    Ok(what) => {
                        self.cal_asked = None;
                        self.cal_note = Some((true, format!("{} {what}", t("Done:", "Xong:"))));
                    }
                    Err(why) => {
                        self.cal_queue.clear();
                        self.cal_asked = None;
                        self.cal_note = Some((false, format!("{} {why}", t("Not done:", "Không xong:"))));
                    }
                }
            }
        }
        // asked, and nothing came back in 5 s: it was not taken (armed, busy, not PX4)
        if self.cal_asked.is_some_and(|(_, at)| at.elapsed() > Duration::from_secs(5)) {
            self.cal_asked = None;
            self.cal_queue.clear();
            self.cal_note = Some((false, t("The flight controller did not start it (motors on, or another calibration running?)", "Bộ điều khiển bay không bắt đầu (motor đang bật, hoặc đang hiệu chuẩn khác?)").to_string()));
        }
    }

    /// Settings > Safety > Sensors: the board's orientation on the frame, and the calibrations
    fn sensors_section(&mut self, ui: &mut egui::Ui, v: &Vehicle) {
        th::section(ui, t("Board orientation and calibration", "Hướng mạch và hiệu chuẩn"), true, |ui| {
            // how it sits: tilt it by hand and see that the numbers go the right way
            th::kv(
                ui,
                t("Now", "Lúc này"),
                &format!(
                    "{} {:+.0}° · {} {:+.0}° · {} {:.0}°",
                    t("roll", "nghiêng"),
                    v.roll.to_degrees(),
                    t("pitch", "ngóc"),
                    v.pitch.to_degrees(),
                    t("heading", "hướng"),
                    (v.yaw.to_degrees() + 360.0) % 360.0
                ),
            );
            // the range finder and the optical flow as they read now (the HUD's H is the flight
            // controller's estimate, smoothed: a hand passing under it does not move it)
            let fresh = |at: Instant| at.elapsed().as_secs_f32() < 2.0;
            th::kv(
                ui,
                t("Range down", "Đo cao (hướng xuống)"),
                &v.range.filter(|r| fresh(r.0)).map_or("—".to_string(), |r| format!("{:.2} m", r.1)),
            );
            th::kv(
                ui,
                t("Optical flow", "Luồng quang"),
                &v.flow.filter(|f| fresh(f.0)).map_or("—".to_string(), |f| format!("{} {}/255 · x {:+.2} · y {:+.2} rad/s", t("quality", "chất lượng"), f.1, f.2, f.3)),
            );
            ui.label(
                egui::RichText::new(t(
                    "Lift the nose: pitch goes up (+). Lower the right side: roll goes up (+). Turn the nose right: the heading goes up. The wrong way: the board sits turned on the frame - pick how.",
                    "Nhấc mũi lên: ngóc tăng (+). Hạ cạnh phải xuống: nghiêng tăng (+). Quay mũi sang phải: hướng tăng. Sai chiều là mạch gắn quay trên khung - chọn cho đúng.",
                ))
                .small()
                .color(th::DIM),
            );
            th::row(ui, t("Board turned", "Mạch gắn quay"), |ui| {
                let cur = v.param("SENS_BOARD_ROT");
                let opts: Vec<(&str, String)> = BOARD_ROT.iter().map(|(en, vi, n)| (t(en, vi), format!("{n}"))).collect();
                let opts_ref: Vec<(&str, &str)> = opts.iter().map(|(a, b)| (*a, b.as_str())).collect();
                let cur_s = cur.map_or(String::new(), |n| format!("{n}"));
                if let Some(hit) = nu::segmented(ui, &opts_ref, &cur_s) {
                    if let Ok(n) = hit.parse::<f64>() {
                        if v.link() && !v.armed {
                            let _ = self.cmds.send(Cmd::SetParam("SENS_BOARD_ROT", n));
                        }
                    }
                }
            });
            ui.separator();
            let running = v.cal.what.clone();
            let busy = running.is_some() || self.cal_asked.is_some();
            let can = v.link() && !v.armed && !busy && v.autopilot == Autopilot::Px4;
            ui.horizontal_wrapped(|ui| {
                let b = |ui: &mut egui::Ui, label: &str| ui.add_enabled(can, egui::Button::new(label).min_size(vec2(150.0, 40.0))).clicked();
                if b(ui, t("Calibrate IMU", "Hiệu chuẩn IMU")) {
                    self.calibrate(&[CalKind::Gyro, CalKind::Accel, CalKind::Level]);
                }
                if b(ui, t("Calibrate compass", "Hiệu chuẩn la bàn")) {
                    self.calibrate(&[CalKind::Mag]);
                }
                if b(ui, t("Level only", "Chỉ cân mặt phẳng")) {
                    self.calibrate(&[CalKind::Level]);
                }
                if busy && ui.add(egui::Button::new(t("Cancel", "Huỷ")).min_size(vec2(100.0, 40.0))).clicked() {
                    self.cal_queue.clear();
                    self.cal_asked = None;
                    let _ = self.cmds.send(Cmd::Calibrate(None));
                }
            });
            if let Some((k, _)) = self.cal_asked {
                th::kv(ui, t("Starting", "Đang bắt đầu"), k.name());
            }
            if let Some(what) = &running {
                let kind = match what.as_str() {
                    "gyro" => Some(CalKind::Gyro),
                    "accel" => Some(CalKind::Accel),
                    "level" => Some(CalKind::Level),
                    "mag" => Some(CalKind::Mag),
                    _ => None,
                };
                th::kv(ui, t("Calibrating", "Đang hiệu chuẩn"), kind.map_or(what.as_str(), |k| k.name()));
                ui.add(egui::ProgressBar::new(f32::from(v.cal.progress) / 100.0).show_percentage());
                let what_to_do = match kind {
                    Some(CalKind::Gyro) => t("Keep it still on a flat surface.", "Để máy bay đứng yên trên mặt phẳng.").to_string(),
                    Some(CalKind::Level) => t("Set it level, as it flies, and keep it still.", "Đặt máy bay bằng phẳng như lúc bay, để yên.").to_string(),
                    Some(CalKind::Accel) => match (&v.cal.current, v.cal.pending.first()) {
                        (Some(side), _) => format!("{} {} - {}", t("Measuring:", "Đang đo:"), side_name(side), t("keep it still", "giữ yên")),
                        (None, Some(next)) => format!("{} {} {}", t("Put it", "Đặt máy bay"), side_name(next), t("and keep it still", "rồi giữ yên")),
                        _ => t("Keep it still.", "Giữ yên.").to_string(),
                    },
                    Some(CalKind::Mag) => match (&v.cal.current, v.cal.pending.first()) {
                        (Some(side), _) => format!("{} {} - {}", t("This side:", "Mặt này:"), side_name(side), t("turn it slowly round and round about the vertical", "xoay chậm từng vòng quanh trục thẳng đứng")),
                        (None, Some(next)) => format!("{} {} {}", t("Hold it", "Cầm máy bay"), side_name(next), t("and turn it round about the vertical", "rồi xoay vòng quanh trục thẳng đứng")),
                        _ => t("Turn it slowly round every way.", "Xoay chậm đủ các hướng.").to_string(),
                    },
                    None => v.cal.line.clone(),
                };
                ui.label(egui::RichText::new(what_to_do).strong().color(th::WARN));
                if matches!(kind, Some(CalKind::Accel | CalKind::Mag)) {
                    let all = ["down", "up", "left", "right", "front", "back"];
                    for side in all {
                        let state = if v.cal.done_sides.iter().any(|d| d == side) {
                            t("done", "xong")
                        } else if v.cal.current.as_deref() == Some(side) {
                            t("measuring", "đang đo")
                        } else if v.cal.pending.iter().any(|p| p == side) {
                            t("to do", "chưa")
                        } else {
                            "—"
                        };
                        th::kv(ui, side_name(side), state);
                    }
                }
                ui.label(egui::RichText::new(&v.cal.line).small().color(th::DIM));
            }
            if let Some((ok, note)) = &self.cal_note {
                ui.label(egui::RichText::new(note).color(if *ok { th::GOOD } else { th::WARN }));
            }
            ui.label(
                egui::RichText::new(t(
                    "With the motors off. IMU: gyro, then the six sides one by one (the app says which), then level - after the board is fixed on the frame. Compass: away from metal and wires, turn it round on each side it asks for.",
                    "Khi motor tắt. IMU: gyro, rồi lần lượt sáu mặt (app báo mặt nào), rồi cân mặt phẳng - làm sau khi mạch đã gắn chắc lên khung. La bàn: tránh xa kim loại và dây điện, xoay vòng ở từng mặt app yêu cầu.",
                ))
                .small()
                .color(th::DIM),
            );
        });
    }

    /// DJI's stick commands (its CSC), from the sticks going up - a gamepad's, a transmitter's,
    /// the screen's two at once: both sticks pulled to the bottom inner corners (or the outer)
    /// starts the motors (0.4 s; let go once they spin); on the ground the same held 2 s, or the
    /// throttle held down 1 s after landing, stops them. Never in the air (the emergency stop
    /// button is for that). A transmitter's throttle that stays put rests at the bottom: the
    /// throttle-down stop is not for it.
    fn stick_commands(&mut self, v: &Vehicle, s: Sticks) {
        if !v.link() || self.link.demo.lock().unwrap().is_some() {
            (self.csc_since, self.thr_low_since) = (None, None);
            return;
        }
        let now = Instant::now();
        let (l, r) = knobs_of(self.cfg.stick_mode, s);
        let corner = |k: Vec2| k.y <= -0.6 && k.x.abs() >= 0.6;
        let csc = corner(l) && corner(r) && (l.x > 0.0) != (r.x > 0.0);
        if !csc {
            (self.csc_since, self.csc_done) = (None, false);
        } else if !self.csc_done {
            let since = *self.csc_since.get_or_insert(now);
            let hold = if v.armed { 2.0 } else { 0.4 };
            if since.elapsed().as_secs_f32() >= hold {
                self.csc_done = true;
                if !v.armed {
                    log("pilot: stick command - the motors on");
                    let _ = self.cmds.send(Cmd::Arm(true));
                } else if !v.in_air() {
                    log("pilot: stick command - the motors off");
                    let _ = self.cmds.send(Cmd::Arm(false));
                }
            }
        }
        let rc_abs = self.link.rc_thr.load(Ordering::Relaxed);
        if !v.armed || rc_abs {
            (self.thr_low_since, self.thr_up_since_arm) = (None, false);
            return;
        }
        if s.throttle > -0.5 {
            self.thr_up_since_arm = true;
        }
        if self.thr_up_since_arm && s.throttle <= -0.9 && !csc && !v.in_air() {
            if self.thr_low_since.get_or_insert(now).elapsed().as_secs_f32() >= 1.0 {
                self.thr_low_since = None;
                self.thr_up_since_arm = false;
                log("pilot: throttle held down on the ground - the motors off");
                let _ = self.cmds.send(Cmd::Arm(false));
            }
        } else {
            self.thr_low_since = None;
        }
    }

    /// The start screen asked for (embedded, Settings > About): the host closes this app
    pub fn leave_asked(&self) -> bool {
        self.leave
    }
}

impl Drop for Pilot {
    fn drop(&mut self) {
        self.link.stop.store(true, Ordering::Relaxed);
        nyx_rx::shutdown(&self.rx);
    }
}

impl Pilot {
    /// A new GPU device: the textures of the old one (the video's, the map's tiles) are gone
    pub fn gpu_reset(&mut self) {
        self.tex = None;
        self.tex_ver = u64::MAX;
        self.map.tiles.gpu_reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --demo's made-up aircraft, as the map reads it: armed where it stands, its track on the
    // globe, home where it took off, never further than its oval is long
    #[test]
    fn the_made_up_flight_goes_round_and_comes_home() {
        let mut v = Vehicle::default();
        let (mut far, mut near) = (0.0f32, f32::MAX);
        for i in 0..(53 * 20) {
            let t = i as f32 / 20.0;
            sim_flight(&mut v, t, Some(DEMO_AT));
            let (here, home) = (v.global().unwrap(), v.home_global());
            if t > 3.0 {
                let d = map::dist_bearing(here, home.unwrap()).0;
                far = far.max(d);
                if t > 40.0 {
                    near = near.min(d);
                }
            }
        }
        assert!(v.armed && v.link() && v.gps_fix() == Some((3, 14)));
        let (lat, lon) = v.home_global().unwrap();
        assert!((lat - DEMO_AT.0).abs() < 1e-6 && (lon - DEMO_AT.1).abs() < 1e-6);
        assert!((far - 140.0).abs() < 2.0, "{far}");
        assert!(near < 3.0, "{near}");
        assert!(v.gtrack.len() > 200 && (-v.z - 30.0).abs() < 0.01);

        // with no fix: nothing on the globe, the same flight in the local frame from where it armed
        let mut v = Vehicle::default();
        for i in 0..(30 * 20) {
            sim_flight(&mut v, i as f32 / 20.0, None);
        }
        assert!(v.place().is_none() && v.home_global().is_none() && v.gps_fix().is_none());
        assert!(v.track.len() > 200 && v.home_local() == (0.0, 0.0) && v.x > 100.0);
    }

    // DJI's top line, and its numbers along the bottom: ready, flying, not ready and why
    #[test]
    fn the_status_line() {
        let mut v = Vehicle::default();
        assert_eq!(flight_status(&v, false).1, RED);
        sim_flight(&mut v, 1.0, Some(DEMO_AT));
        let (s, c) = flight_status(&v, true);
        assert!(s.contains("Ready") && s.contains("GPS") && c == GREEN, "{s}");
        assert!(!v.in_air() && v.flight_time().is_none());
        for i in 0..(20 * 20) {
            sim_flight(&mut v, 2.0 + i as f32 / 20.0, Some(DEMO_AT));
        }
        let (s, c) = flight_status(&v, true);
        assert!(s.starts_with("In flight") && c == GREEN, "{s}");
        assert!(v.in_air() && v.flight_time().is_some());
        let d = v.home_dist().unwrap();
        assert!(d > 10.0 && d < 141.0, "{d}");
        // not ready, in the flight controller's own words
        let mut v = Vehicle::default();
        sim_flight(&mut v, 1.0, None);
        v.update(1, 1, &Msg::SysStatus { load: 0, voltage_mv: 16_000, remaining: 90, health: 0 });
        v.update(1, 1, &Msg::Text { severity: 4, text: "Preflight Fail: Accel Sensor 0 missing".into() });
        let (s, c) = flight_status(&v, true);
        assert!(s.contains("Accel Sensor 0 missing") && c == RED, "{s}");
        // (no satellites: no D, its local frame drifts)
        assert!(v.home_dist().is_none());
        assert_eq!(clock(Duration::from_secs(75)), "01:15");
        assert_eq!(clock(Duration::from_secs(3723)), "1:02:03");
    }

    // the link's distance reading: never unless the pilot set it; then on after 3 s with no
    // satellites, off after 20 s of them back (a fix that blinks does not toggle it)
    #[test]
    fn ranging_only_without_satellites() {
        let s = Duration::from_secs;
        assert!(!range_wanted(false, false, false, s(600)));
        assert!(!range_wanted(false, false, true, s(600)));
        assert!(!range_wanted(true, true, false, s(600)));
        assert!(!range_wanted(true, false, false, s(1)));
        assert!(range_wanted(true, false, false, s(3)));
        assert!(range_wanted(true, false, true, s(0)));
        assert!(range_wanted(true, true, true, s(5)));
        assert!(!range_wanted(true, true, true, s(20)));
    }

    // the tether test: the guard's limits, the throttle capped, the motors off when the signal is
    // lost; off puts back what the flight controller had
    #[test]
    fn the_tether_test() {
        let get = |l: &[(&str, f64)], id: &str| l.iter().find(|x| x.0 == id).map(|x| x.1);
        let on = tether_params(&AppCfg::default());
        assert_eq!(get(&on, "NYX_GRD_TILT"), Some(45.0));
        assert_eq!(get(&on, "NYX_GRD_YAWR"), Some(300.0));
        assert_eq!(get(&on, "NYX_GRD_HGT"), Some(0.0));
        assert_eq!(get(&on, "MPC_THR_MAX"), Some(0.6));
        assert_eq!((get(&on, "NAV_RCL_ACT"), get(&on, "NAV_DLL_ACT")), (Some(6.0), Some(6.0)));
        let off = tether_off_params([1.0, 2.0, 0.0]);
        assert_eq!((get(&off, "NYX_GRD_TILT"), get(&off, "NYX_GRD_YAWR")), (Some(0.0), Some(0.0)));
        assert_eq!(get(&off, "MPC_THR_MAX"), Some(1.0));
        assert_eq!((get(&off, "NAV_RCL_ACT"), get(&off, "NAV_DLL_ACT")), (Some(2.0), Some(0.0)));
        // what it keeps to put back is what it changes besides the guard
        for id in TETHER_KEEP {
            assert!(get(&on, id).is_some() && get(&off, id).is_some(), "{id}");
        }
    }

    // take off, land, return: a touch does nothing, a second's hold does it once, let go and hold
    // again for another
    #[test]
    fn held_a_second() {
        let mut h = std::collections::HashMap::new();
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        assert_eq!(held(&mut h, "takeoff", true, at(0)), (0.0, false));
        assert_eq!(held(&mut h, "takeoff", false, at(300)), (0.0, false));
        assert!(held(&mut h, "takeoff", true, at(400)) == (0.0, false));
        let (k, fired) = held(&mut h, "takeoff", true, at(900));
        assert!((k - 0.5).abs() < 1e-3 && !fired);
        assert_eq!(held(&mut h, "takeoff", true, at(1400)), (1.0, true));
        for ms in [1500, 3000, 9000] {
            assert_eq!(held(&mut h, "takeoff", true, at(ms)), (1.0, false), "{ms}");
        }
        // (another button meanwhile: its own)
        assert_eq!(held(&mut h, "return", true, at(9000)).1, false);
        held(&mut h, "takeoff", false, at(9100));
        held(&mut h, "takeoff", true, at(9200));
        assert_eq!(held(&mut h, "takeoff", true, at(10_200)), (1.0, true));
    }

    // DJI's stick modes: each stick moves what the mode says, and the knobs show it back
    #[test]
    fn the_stick_modes() {
        let (l, r) = (vec2(0.1, 0.2), vec2(0.3, 0.4));
        let s2 = sticks_of(2, l, r);
        assert_eq!((s2.yaw, s2.throttle, s2.roll, s2.pitch), (0.1, 0.2, 0.3, 0.4));
        let s1 = sticks_of(1, l, r);
        assert_eq!((s1.yaw, s1.pitch, s1.roll, s1.throttle), (0.1, 0.2, 0.3, 0.4));
        let s3 = sticks_of(3, l, r);
        assert_eq!((s3.roll, s3.pitch, s3.yaw, s3.throttle), (0.1, 0.2, 0.3, 0.4));
        for m in 1..=3 {
            assert_eq!(knobs_of(m, sticks_of(m, l, r)), (l, r), "mode {m}");
        }
        // the feel: the middle softer, the ends where they were
        assert_eq!(expo(1.0, 0.6), 1.0);
        assert_eq!(expo(-1.0, 0.6), -1.0);
        assert!(expo(0.3, 0.6) < 0.3 && expo(0.3, 0.0) == 0.3);
    }

    #[test]
    fn the_settings_file() {
        let dir = std::env::temp_dir().join(format!("nyxhop-pilot-test-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let f = dir.join("nyxhop-pilot.cfg");
        let c = AppCfg { takeoff_h: 12.5, stick_mode: 1, expo: 0.35, speed: Speed::Cine, range_nogps: true, estop: false, tether: true, tether_tilt: 40.0, tether_yawr: 250.0, tether_thr: 55.0, tether_hgt: 1.5, tether_prev: Some([1.0, 2.0, 0.0]), rc_order: rc::Order::Taer, rc_center: true };
        c.save(&f);
        assert_eq!(AppCfg::load(&f), c);
        // what is not there or out of range: the default
        std::fs::write(&f, "stick_mode=7\nexpo=3\ntakeoff_h=abc\nspeed=warp\n").unwrap();
        assert_eq!(AppCfg::load(&f), AppCfg::default());
        assert_eq!(AppCfg::load(&dir.join("none.cfg")), AppCfg::default());
        // a file from before the speed modes: its take-off height was the old default (10 m), not
        // a choice - DJI's short hop instead; the rest kept
        std::fs::write(&f, "takeoff_h=10.0\nstick_mode=1\nexpo=0.20\n").unwrap();
        let old = AppCfg::load(&f);
        assert_eq!((old.takeoff_h, old.stick_mode, old.speed), (TAKEOFF_H, 1, Speed::Normal));
        assert!((old.expo - 0.2).abs() < 1e-6);
        // the speed modes: each autopilot's names, inside PX4's own limits
        for s in Speed::ALL {
            let [vh, acc, up, dn, yaw, tilt] = s.limits();
            assert!((3.0..=20.0).contains(&vh) && (2.0..=15.0).contains(&acc), "{s:?}");
            assert!((0.5..=8.0).contains(&up) && (0.5..=4.0).contains(&dn), "{s:?}");
            assert!((0.0..=400.0).contains(&yaw) && (1.0..=45.0).contains(&tilt), "{s:?}");
            assert_eq!(s.params(Autopilot::Px4)[0], ("MPC_VEL_MANUAL", vh));
            assert_eq!(s.params(Autopilot::ArduPilot)[0], ("LOIT_SPEED_MS", vh));
            assert_eq!(Speed::of_key(s.key()), Some(s));
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}
