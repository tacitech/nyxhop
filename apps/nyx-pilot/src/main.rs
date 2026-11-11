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
//!
//! Keys: arrows roll and pitch, W/S throttle, A/D yaw; T take off, L land, R return, H hold,
//! 1/2/3 Position / Altitude / Stabilized; M the map large and the video small, or back; + and -
//! zoom the map.
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
use nyx_common::logging::{self, log};

#[cfg(windows)]
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

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
}

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
    /// --demo: the sticks it holds (over every other input) and what it is doing
    demo: Mutex<Option<Sticks>>,
    demo_step: Mutex<String>,
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
    tell(Msg::SysStatus { load: 250, voltage_mv: (16_400.0 - t * 2.0).max(14_000.0) as u16, remaining: (95.0 - t / 12.0).max(20.0) as i8 });
    tell(Msg::Attitude { roll, pitch, yaw: (45.0 * th.cos()).atan2(70.0 * th.sin()) });
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
fn link_thread(link: Arc<Link>, cmds: Receiver<Cmd>) {
    let sock = match UdpSocket::bind(DOWN) {
        Ok(s) => s,
        Err(e) => {
            log(&format!("pilot: cannot listen on {DOWN}: {e}"));
            return;
        }
    };
    let _ = sock.set_read_timeout(Some(Duration::from_millis(5)));
    let up: SocketAddr = UP.parse().unwrap();
    let mut enc = Encoder::default();
    let send = |b: Vec<u8>| {
        let _ = sock.send_to(&b, up);
    };
    let t0 = Instant::now();
    let mut buf = [0u8; 2048];
    let (mut t_mc, mut t_hb, mut t_rates) = (t0, t0, t0 - Duration::from_secs(10));
    let mut pending_ts: Vec<(i64, Instant)> = Vec::new();
    let mut takeoff_at: Option<(Instant, f32)> = None;
    // ArduPilot's take-off (GUIDED, which ignores the sticks): sent at, to the height
    let mut ap_takeoff: Option<(Instant, f32)> = None;
    let mut t_loiter = t0;
    let (mut n_msgs, mut t_rate) = (0u32, t0);
    // Stabilized: the throttle is the thrust, not "hold the height", so a stick that springs back
    // to the middle would leave half thrust on (and PX4 refuses to arm: "high throttle"). There
    // the sticks move a throttle that stays where it is left: 0 while disarmed, the vehicle's own
    // throttle when the mode comes on armed (from Position or Altitude in flight).
    let mut thr: Option<f32> = None;
    loop {
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
            if manual {
                let level = match thr {
                    _ if !armed => 0.0,
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
            // ArduPilot in GUIDED (our take-off) does not fly by the sticks: LOITER once the take-off
            // is done, or as soon as a stick moves - the sticks always fly it, as on PX4
            let (mode, up) = {
                let v = link.veh.lock().unwrap();
                (v.main_mode, -v.z)
            };
            if ardu && armed && mode == 4 && now - t_loiter >= Duration::from_secs(1) {
                let moved = [s.roll, s.pitch, s.yaw, s.throttle].iter().any(|v| v.abs() > 0.15);
                let done = ap_takeoff.is_some_and(|(at, h)| up >= h - 0.5 || at.elapsed() >= Duration::from_secs(20));
                if moved || done {
                    t_loiter = now;
                    ap_takeoff = None;
                    let (m, _) = FlightMode::Position.params(Autopilot::ArduPilot);
                    send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, m, 0.0, 0.0, 0.0, 0.0, 0.0]));
                    log(if moved { "ArduPilot: a stick in GUIDED - LOITER" } else { "ArduPilot: take-off done - LOITER" });
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
            // the map GLOBAL_POSITION_INT 10, GPS_RAW_INT 1
            for (id, hz) in [(30.0, 30.0), (32.0, 20.0), (74.0, 10.0), (245.0, 2.0), (242.0, 1.0), (33.0, 10.0), (24.0, 1.0)] {
                send(enc.command_long(sys, comp, mav::MAV_CMD_SET_MESSAGE_INTERVAL, [id, 1e6 / hz, 0.0, 0.0, 0.0, 0.0, 0.0]));
            }
        }
        let ap = link.veh.lock().unwrap().autopilot;
        while let Ok(c) = cmds.try_recv() {
            let arm = |enc: &mut Encoder, on: bool| {
                enc.command_long(sys, comp, mav::MAV_CMD_COMPONENT_ARM_DISARM, [if on { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            };
            match c {
                Cmd::Arm(on) => send(arm(&mut enc, on)),
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
                    let (main, sub) = m.params(ap);
                    send(enc.command_long(sys, comp, mav::MAV_CMD_DO_SET_MODE, [1.0, main, sub, 0.0, 0.0, 0.0, 0.0]));
                }
            }
        }
        if let Some((at, h)) = takeoff_at.filter(|(at, _)| now >= *at) {
            let _ = at;
            takeoff_at = None;
            let v = link.veh.lock().unwrap().clone();
            // (the height: PX4 above sea level, ArduPilot above home)
            let alt = if ap == Autopilot::ArduPilot { h } else { v.home_amsl.unwrap_or(v.amsl - v.alt) + h };
            send(enc.command_long(sys, comp, mav::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, f32::NAN, f32::NAN, f32::NAN, alt]));
            if ap == Autopilot::ArduPilot {
                ap_takeoff = Some((now, h));
            }
        }
    }
}

struct Pilot {
    rx: Arc<nyx_rx::Shared>,
    link: Arc<Link>,
    cmds: Sender<Cmd>,
    tex: Option<egui::TextureHandle>,
    tex_ver: u64,
    gilrs: Option<gilrs::Gilrs>,
    pad_name: Option<String>,
    /// the on-screen sticks: left (yaw, throttle), right (roll, pitch)
    left: Vec2,
    right: Vec2,
    arm_hold: Option<Instant>,
    takeoff_h: f32,
    /// the knobs as drawn: the sticks that went up last frame
    shown: (Vec2, Vec2),
    map: map::MapView,
    /// the map large and the video small (M, or a tap on the small one)
    map_large: bool,
    /// where the aircraft was last seen on the globe, kept in a file for the next run's map
    place_file: PathBuf,
    place_kept: Option<(f64, f64)>,
}

impl Pilot {
    /// `dir`: the program's folder (its settings, the map's tiles)
    fn new(board: &str, opts: &Opts, dir: &Path) -> Self {
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
        let rx = nyx_rx::setup(&Opts::from_list(vec![
            "--channel".into(),
            addr,
            "--tlm-in".into(),
            UP.into(),
            "--tlm-out".into(),
            DOWN.into(),
        ]));
        let link = Arc::new(Link {
            veh: Mutex::new(veh),
            heard: AtomicBool::new(false),
            sticks: Mutex::new(Sticks::default()),
            thr: Mutex::new(None),
            demo: Mutex::new(None),
            demo_step: Mutex::new(String::new()),
        });
        let (cmds, cmd_rx) = channel();
        let l = link.clone();
        std::thread::Builder::new().name("mavlink".into()).spawn(move || link_thread(l, cmd_rx)).expect("spawn mavlink");
        if demo_on {
            let (l, c) = (link.clone(), cmds.clone());
            std::thread::Builder::new().name("demo".into()).spawn(move || demo(l, c, demo_at)).expect("spawn demo");
        }
        let gilrs = gilrs::Gilrs::new().map_err(|e| log(&format!("pilot: no gamepad support: {e}"))).ok();
        Pilot {
            rx,
            link,
            cmds,
            tex: None,
            tex_ver: u64::MAX,
            gilrs,
            pad_name: None,
            left: Vec2::ZERO,
            right: Vec2::ZERO,
            arm_hold: None,
            takeoff_h: 10.0,
            shown: (Vec2::ZERO, Vec2::ZERO),
            map,
            map_large: false,
            place_file,
            place_kept,
        }
    }

    /// The gamepad (or RC transmitter) in mode 2: left stick yaw and throttle, right stick roll
    /// and pitch
    fn gamepad(&mut self) -> Option<Sticks> {
        let g = self.gilrs.as_mut()?;
        while g.next_event().is_some() {}
        let (_, pad) = g.gamepads().find(|(_, p)| p.is_connected())?;
        self.pad_name = Some(pad.name().to_string());
        use gilrs::Axis;
        let dz = |v: f32| if v.abs() < 0.06 { 0.0 } else { v };
        Some(Sticks {
            yaw: dz(pad.value(Axis::LeftStickX)),
            throttle: dz(pad.value(Axis::LeftStickY)),
            roll: dz(pad.value(Axis::RightStickX)),
            pitch: dz(pad.value(Axis::RightStickY)),
        })
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
fn stick(ui: &mut egui::Ui, centre: Pos2, radius: f32, pos: &mut Vec2, shown: Vec2, id: &str) {
    let rect = Rect::from_center_size(centre, Vec2::splat(radius * 2.4));
    let resp = ui.interact(rect, ui.id().with(id), egui::Sense::drag());
    if let Some(pt) = resp.interact_pointer_pos().filter(|_| resp.dragged()) {
        let v = (pt - centre) / radius;
        let v = if v.length() > 1.0 { v.normalized() } else { v };
        *pos = vec2(v.x, -v.y);
    } else {
        *pos = Vec2::ZERO;
    }
    let p = ui.painter();
    p.circle_filled(centre, radius, Color32::from_rgba_unmultiplied(10, 15, 22, 140));
    p.circle_stroke(centre, radius, Stroke::new(2.0, Color32::from_white_alpha(70)));
    p.line_segment([centre - vec2(radius, 0.0), centre + vec2(radius, 0.0)], Stroke::new(1.0, Color32::from_white_alpha(35)));
    p.line_segment([centre - vec2(0.0, radius), centre + vec2(0.0, radius)], Stroke::new(1.0, Color32::from_white_alpha(35)));
    // (not dragged: where the sticks that go up are, the gamepad's or the demo's)
    let k = if resp.dragged() { *pos } else { shown };
    let knob = centre + vec2(k.x, -k.y) * radius;
    p.circle_filled(knob, radius * 0.28, if resp.dragged() { ACCENT } else { Color32::from_white_alpha(160) });
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

        egui::CentralPanel::default().frame(egui::Frame::NONE.fill(Color32::BLACK)).show(root, |ui| {
            let full = ui.max_rect();
            let p = ui.painter().clone();
            // the corners' panels: the mode, the link; the small view under the link's
            let tl = Rect::from_min_size(full.left_top() + vec2(14.0, 14.0), vec2(230.0, 92.0));
            let tr = Rect::from_min_size(full.right_top() + vec2(-244.0, 14.0), vec2(230.0, 92.0));
            let small = Rect::from_min_size(tr.left_bottom() + vec2(0.0, 10.0), vec2(230.0, if self.map_large { 129.0 } else { 172.0 }));
            // (the map's zoom buttons: under the small view)
            let zoom = small.right_bottom() + vec2(0.0, 10.0);
            // the large view: the video, or the map (first, so that all else is over it)
            if self.map_large {
                let at = map::Corners {
                    tl: tl.left_bottom() + vec2(0.0, 10.0),
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

            // the attitude (over the video: the horizon belongs to what the camera sees), the
            // heading, speed and height
            let c = full.center() - vec2(0.0, full.height() * 0.05);
            let r = full.height().min(full.width()) * 0.16;
            if !self.map_large {
                horizon(&p, c, r, v.roll, v.pitch);
            }
            heading_tape(&p, pos2(full.center().x, full.top() + 12.0), (full.width() * 0.32).max(280.0), v.yaw, self.map_large);
            let gs = v.vx.hypot(v.vy);
            let box_l = Rect::from_center_size(c - vec2(r * 2.1, 0.0), vec2(96.0, 54.0));
            let box_r = Rect::from_center_size(c + vec2(r * 2.1, 0.0), vec2(110.0, 54.0));
            panel(&p, box_l);
            panel(&p, box_r);
            shadow_text(&p, box_l.center() - vec2(0.0, 8.0), Align2::CENTER_CENTER, &format!("{gs:.1}"), FontId::proportional(24.0), WHITE);
            shadow_text(&p, box_l.center() + vec2(0.0, 16.0), Align2::CENTER_CENTER, "m/s", FontId::proportional(11.0), MUTED);
            shadow_text(&p, box_r.center() - vec2(0.0, 8.0), Align2::CENTER_CENTER, &format!("{:.1}", -v.z), FontId::proportional(24.0), WHITE);
            shadow_text(&p, box_r.center() + vec2(0.0, 16.0), Align2::CENTER_CENTER, &format!("m   {:+.1} m/s", -v.vz), FontId::proportional(11.0), MUTED);

            // top left: the mode, armed, battery
            panel(&p, tl);
            let (mode_col, armed_txt) = if v.armed { (AMBER, t("ARMED", "ĐÃ ARM")) } else { (MUTED, t("disarmed", "chưa arm")) };
            shadow_text(&p, tl.left_top() + vec2(12.0, 10.0), Align2::LEFT_TOP, if v.link() { v.mode_name() } else { "—" }, FontId::proportional(22.0), WHITE);
            shadow_text(&p, tl.left_top() + vec2(12.0, 40.0), Align2::LEFT_TOP, armed_txt, FontId::proportional(14.0), mode_col);
            if v.link() {
                let ap = if v.autopilot == Autopilot::ArduPilot { "ArduPilot" } else { "PX4" };
                shadow_text(&p, tl.right_top() + vec2(-12.0, 44.0), Align2::RIGHT_TOP, ap, FontId::proportional(12.0), MUTED);
            }
            let batt = if v.batt_v > 0.0 { format!("{:.1} V  {}%", v.batt_v, v.batt_pct.max(0)) } else { "—".into() };
            shadow_text(&p, tl.left_top() + vec2(12.0, 64.0), Align2::LEFT_TOP, &format!("{}  {batt}", t("Battery", "Pin")), FontId::proportional(13.0), MUTED);

            // top right: the link
            panel(&p, tr);
            let (lc, lt) = match (lv.live, v.link()) {
                (true, true) => (GREEN, t("link · video and control", "link · hình và điều khiển")),
                (_, true) => (AMBER, t("control only", "chỉ có điều khiển")),
                (true, false) => (AMBER, t("video only", "chỉ có hình")),
                _ => (RED, t("no link", "mất link")),
            };
            shadow_text(&p, tr.left_top() + vec2(12.0, 10.0), Align2::LEFT_TOP, lt, FontId::proportional(14.0), lc);
            let rtt = v.rtt_ms.map_or("—".into(), |r| format!("{r:.0} ms"));
            shadow_text(&p, tr.left_top() + vec2(12.0, 36.0), Align2::LEFT_TOP, &format!("SNR {:.0} dB   MCS {}   {:.0} fps", lv.snr_db, lv.mcs, lv.fps), FontId::proportional(13.0), WHITE);
            shadow_text(&p, tr.left_top() + vec2(12.0, 60.0), Align2::LEFT_TOP, &format!("{} {rtt}   {:.0} msg/s", t("round trip", "khứ hồi"), v.rate), FontId::proportional(13.0), MUTED);
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

            // PX4's last words, a few seconds each
            if let Some((at, sev, txt)) = v.texts.back().filter(|x| x.0.elapsed() < Duration::from_secs(6)) {
                let _ = at;
                let col = if *sev <= 3 { RED } else if *sev <= 4 { AMBER } else { WHITE };
                over(&p, pos2(full.center().x, full.top() + 84.0), Align2::CENTER_CENTER, txt, 15.0, col, self.map_large);
            }

            // --demo: what it is doing
            let step = self.link.demo_step.lock().unwrap().clone();
            if !step.is_empty() {
                let r = Rect::from_center_size(pos2(full.center().x, full.top() + 116.0), vec2(360.0, 30.0));
                panel(&p, r);
                shadow_text(&p, r.center(), Align2::CENTER_CENTER, &format!("DEMO · {step}"), FontId::proportional(14.0), ACCENT);
            }

            // the sticks: left yaw/throttle, right roll/pitch (mode 2)
            let sr = (full.height() * 0.12).clamp(60.0, 110.0);
            let ly = full.bottom() - sr - 40.0;
            let name = |at: Pos2, text: &str| over(&p, at + vec2(0.0, sr + 14.0), Align2::CENTER_CENTER, text, 12.0, MUTED, self.map_large);
            stick(ui, pos2(full.left() + sr + 40.0, ly), sr, &mut self.left, self.shown.0, "L");
            name(pos2(full.left() + sr + 40.0, ly), t("yaw · throttle", "xoay · ga"));
            // Stabilized: the throttle held (it stays where the stick left it)
            if let Some(l) = *self.link.thr.lock().unwrap() {
                let at = pos2(full.left() + sr + 40.0, ly - sr - 18.0);
                over(&p, at, Align2::CENTER_CENTER, &format!("{} {:.0} %", t("throttle", "ga"), l * 100.0), 16.0, if l > 0.0 { AMBER } else { WHITE }, self.map_large);
            }
            stick(ui, pos2(full.right() - sr - 40.0, ly), sr, &mut self.right, self.shown.1, "R");
            name(pos2(full.right() - sr - 40.0, ly), t("roll · pitch", "nghiêng · tới lui"));

            // the buttons and the mode
            let bw = vec2(118.0, 40.0);
            let row = full.bottom() - 64.0;
            let x0 = full.center().x - (bw.x * 4.0 + 30.0) / 2.0;
            let mut b = |i: usize, label: &str, fill: Color32| -> bool {
                let r = Rect::from_min_size(pos2(x0 + i as f32 * (bw.x + 10.0), row), bw);
                ui.put(r, egui::Button::new(egui::RichText::new(label).strong().size(15.0)).fill(fill)).clicked()
            };
            let dark = Color32::from_rgba_unmultiplied(28, 39, 51, 230);
            if b(0, t("Take off", "Cất cánh"), Color32::from_rgb(40, 110, 200)) {
                let _ = self.cmds.send(Cmd::Takeoff(self.takeoff_h));
            }
            if b(1, t("Land", "Hạ cánh"), dark) {
                let _ = self.cmds.send(Cmd::Land);
            }
            if b(2, t("Return", "Về nhà"), dark) {
                let _ = self.cmds.send(Cmd::Return);
            }
            if b(3, t("Hold", "Dừng"), dark) {
                let _ = self.cmds.send(Cmd::Mode(FlightMode::Hold));
            }
            // arm / disarm: held for a second (a press alone does nothing)
            let ar = Rect::from_min_size(pos2(x0, row - 52.0), vec2(bw.x * 1.5, 40.0));
            let arm_label = if v.armed { t("Hold to disarm", "Giữ để tắt") } else { t("Hold to arm", "Giữ để arm") };
            let resp = ui.put(ar, egui::Button::new(egui::RichText::new(arm_label).strong().size(14.0)).fill(if v.armed { Color32::from_rgb(120, 40, 36) } else { dark }).sense(egui::Sense::click_and_drag()));
            if resp.is_pointer_button_down_on() {
                let t0 = *self.arm_hold.get_or_insert_with(Instant::now);
                let k = (t0.elapsed().as_secs_f32() / 1.0).min(1.0);
                ui.painter().rect_filled(Rect::from_min_size(ar.left_bottom() - vec2(0.0, 4.0), vec2(ar.width() * k, 4.0)), 0.0, AMBER);
                if k >= 1.0 {
                    let _ = self.cmds.send(Cmd::Arm(!v.armed));
                    self.arm_hold = Some(Instant::now() + Duration::from_secs(3600));
                }
            } else {
                self.arm_hold = None;
            }
            let modes = [(FlightMode::Position, t("Position", "Giữ vị trí")), (FlightMode::Altitude, t("Altitude", "Giữ độ cao")), (FlightMode::Stabilized, t("Stabilized", "Cân bằng"))];
            for (i, (m, label)) in modes.iter().enumerate() {
                let on = v.flight_mode() == Some(*m);
                let r = Rect::from_min_size(pos2(ar.right() + 10.0 + i as f32 * 104.0, row - 52.0), vec2(96.0, 40.0));
                if ui.put(r, egui::Button::new(egui::RichText::new(*label).size(13.0)).fill(if on { ACCENT } else { dark })).clicked() {
                    let _ = self.cmds.send(Cmd::Mode(*m));
                }
            }
            let src = match &self.pad_name {
                Some(n) => format!("{}: {n}", t("sticks", "cần")),
                None => t("sticks: on screen · keyboard", "cần: trên màn hình · bàn phím").into(),
            };
            over(&p, pos2(full.center().x, full.bottom() - 12.0), Align2::CENTER_CENTER, &src, 11.0, MUTED, self.map_large);
        });

        // the keys for the buttons
        ctx.input(|i| {
            for (k, c) in [
                (egui::Key::T, Cmd::Takeoff(self.takeoff_h)),
                (egui::Key::L, Cmd::Land),
                (egui::Key::R, Cmd::Return),
                (egui::Key::H, Cmd::Mode(FlightMode::Hold)),
                (egui::Key::Num1, Cmd::Mode(FlightMode::Position)),
                (egui::Key::Num2, Cmd::Mode(FlightMode::Altitude)),
                (egui::Key::Num3, Cmd::Mode(FlightMode::Stabilized)),
            ] {
                if i.key_pressed(k) {
                    let _ = self.cmds.send(c);
                }
            }
            // the map: M large or small, + and - closer and further
            swap |= i.key_pressed(egui::Key::M);
            let closer = (i.key_pressed(egui::Key::Plus) || i.key_pressed(egui::Key::Equals)) as i32 - i.key_pressed(egui::Key::Minus) as i32;
            if closer != 0 {
                self.map.zoom_by(v.place().is_some(), closer as f32);
            }
        });
        if swap {
            self.map_large = !self.map_large;
        }

        // the sticks going up: the demo's, else the gamepad, the screen and the keyboard
        let kb = ctx.input(|i| {
            let k = |a: egui::Key, b: egui::Key| (i.key_down(a) as i32 - i.key_down(b) as i32) as f32;
            Sticks {
                roll: 0.6 * k(egui::Key::ArrowRight, egui::Key::ArrowLeft),
                pitch: 0.6 * k(egui::Key::ArrowUp, egui::Key::ArrowDown),
                yaw: 0.6 * k(egui::Key::D, egui::Key::A),
                throttle: 0.5 * k(egui::Key::W, egui::Key::S),
            }
        });
        let screen = Sticks { roll: self.right.x, pitch: self.right.y, yaw: self.left.x, throttle: self.left.y };
        let pad = self.gamepad();
        let pick = |a: f32, b: f32, c: f32| {
            let s = a + b;
            if c.abs() > s.abs() { c } else { s.clamp(-1.0, 1.0) }
        };
        let pad = pad.unwrap_or_default();
        let demo = *self.link.demo.lock().unwrap();
        let sticks = demo.unwrap_or(Sticks {
            roll: pick(screen.roll, kb.roll, pad.roll),
            pitch: pick(screen.pitch, kb.pitch, pad.pitch),
            yaw: pick(screen.yaw, kb.yaw, pad.yaw),
            throttle: pick(screen.throttle, kb.throttle, pad.throttle),
        });
        *self.link.sticks.lock().unwrap() = sticks;
        // the on-screen sticks show what goes up (the demo's, the gamepad's) when not dragged
        if self.left == Vec2::ZERO && self.right == Vec2::ZERO {
            self.shown = (vec2(sticks.yaw, sticks.throttle), vec2(sticks.roll, sticks.pitch));
        } else {
            self.shown = (self.left, self.right);
        }
    }
}

impl Drop for Pilot {
    fn drop(&mut self) {
        nyx_rx::shutdown(&self.rx);
    }
}

fn main() -> eframe::Result {
    logging::init("pilot");
    nyx_common::sys::keep_up();
    let opts = Opts::from_env();
    let board = opts.arg("--board", "192.168.0.11");
    let dir = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into());
    nyx_common::i18n::init(&dir);
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 800.0]).with_title("NyxHop Pilot"),
        ..Default::default()
    };
    eframe::run_native(
        "NyxHop Pilot",
        options,
        Box::new(move |cc| {
            nyx_common::ui::touch_style(&cc.egui_ctx);
            Ok(Box::new(Pilot::new(&board, &opts, &dir)))
        }),
    )
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
}
